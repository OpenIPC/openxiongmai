// SPDX-License-Identifier: GPL-2.0
/*
 * The XM530V200's VPSS (/dev/vpss): a drop-in for the vendor vpss.ko.
 *
 * On the XM530V200 VPSS is a memory-to-memory engine. The ISP stores each
 * group's (sensor's) picture into a vb.ko buffer on list 1, bound to VPSS
 * (module 8, the group as dev); this driver's thread takes it from there,
 * programs the engine (0x34000000) to scale it into the group's three
 * channels, starts it (+0x10020) and waits for its interrupt (IRQ 72,
 * +0x10044), then hands the channels' buffers to what each is bound to
 * (VENC: list 2; VPSS: list 1; VO through vo.ko) or to its queue for user
 * space. isp.ko takes part through vpss_isp_set_val/_post_deal.
 *
 * With the ISP's BPU online (HDR) no picture comes to list 1: the BPU's
 * output goes straight to the scalers, and the store channels write it,
 * driven by their frame start and end interrupts (IRQ 68), as the vendor's
 * vi.ko does; see online().
 *
 * The engine scales in two stages: the input (cropped by the group) to a
 * "middle" picture as wide and high as the largest channel or the input,
 * then each channel from that. A channel the size of an uncropped input
 * takes the input buffer itself. Splicing puts two groups' channels into
 * one picture (modes 1/2 top/bottom, 3/4 left/right).
 *
 * ioctls (a 128-byte record: group, channel, then the call's words 4..):
 *   0x5000 init, 0x5010 every group destroyed
 *   0x40805001 create a group (attributes: max w, h, ...; word 10 = 1 also
 *   passes them to isp.ko), 0x40805002/0xc0805003 its attributes
 *   0x40045008 start {grp}, 0x40045009 stop
 *   0x40805004/0xc0805005 a channel's format (2 = SP420; channels 0 and 1
 *   only that), w, h, source and output frame rates, mirror, flip
 *   0x40805006 enable a channel, 0x40805007 disable
 *   0x4080500a bind: module, dev, chn (channel 100: the group's input,
 *   passed on after use)
 *   0x4080500b the group's crop, 0x4080500c a channel's crop
 *   0x4080500d the depth of a channel's queue
 *   0x4080500e a picture from that queue, 0x4080500f that picture back
 *   0x40805012 splice: mode, id; 0x420c5011 debug register writes
 * spec=N, vpss.ko's special memory modes (none of the vendor's loaders
 * sets one): 1 takes a picture only with another queued behind it (a
 * frame of latency); 4, a channel the size of the uncropped input taking
 * the input buffer, is what this driver always does. 2 (a store-status
 * check at a group's start), 3 (channel buffers three pictures deep, with
 * the picture placed after two, and the queued picture's addresses to
 * match) and 5 (splice buffers three pictures (mode 1) or one and a half
 * (mode 2) deep) are not supported and run as 0. rst=1 resets the engine
 * before every picture, as vpss.ko.
 */

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/kthread.h>
#include <linux/list.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/proc_fs.h>
#include <linux/sched.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <asm/outercache.h>

#define VPSS_BASE 0x34000000
#define VPSS_SIZE 0x100000
#define VPSS_IRQ 72
#define DDR_BASE 0x80000000u
#define N_GRP 64
#define N_CHN 3
#define N_WIN 16
#define N_SPLICE 9
#define MID_MAX 0xa50 /* the middle picture's widest */
#define FWD_CHN 100   /* VPSS_BIND's channel for the group's input */

#define VPSS_INIT 0x5000
#define VPSS_DESTROY_ALL 0x5010
#define VPSS_CREATE_GRP 0x40805001
#define VPSS_SET_GRP_ATTR 0x40805002
#define VPSS_SET_GRP_ATTR2 0xc0805003
#define VPSS_SET_CHN_ATTR 0x40805004
#define VPSS_SET_CHN_ATTR2 0xc0805005
#define VPSS_ENABLE_CHN 0x40805006
#define VPSS_DISABLE_CHN 0x40805007
#define VPSS_START_GRP 0x40045008
#define VPSS_STOP_GRP 0x40045009
#define VPSS_BIND 0x4080500a
#define VPSS_SET_GRP_CROP 0x4080500b
#define VPSS_SET_CHN_CROP 0x4080500c
#define VPSS_SET_DEPTH 0x4080500d
#define VPSS_GET_FRAME 0x4080500e
#define VPSS_RELEASE_FRAME 0x4080500f
#define VPSS_DEBUG 0x420c5011
#define VPSS_SPLICE 0x40805012

#define MOD_VENC 7
#define MOD_VPSS 8
#define MOD_VO 0xf
#define MOD_VAR 0x11
#define FMT_SP420 2

/* vb.ko's buffer descriptors (64 bytes, the XM530V200 layout) */
#define WORD(b, n) (((u32 *)(b))[n])
#define W_PHYS 4
#define W_SIZE 5
#define W_STRIDE 6
#define W_WIDTH 7
#define W_HEIGHT 8
#define W_PTS 10
#define W_BIND 12 /* module, dev, chn */

#define VB_FREE 0
#define VB_VPSS 1
int vb_get_buf(u32 list, u32 size, void **buf, int ms);
int vb_get_length(u32 list, int *n);
int vb_put_buf(u32 list, void *buf);
int vb_add_buf_reference(void *buf);
void clk_get_pts(u64 *pts);
/* vo.ko */
int vo_get_buf(u32 dev, u32 chn, void **buf);
int vo_put_buf(u32 dev, u32 chn, void *buf, int show);
/* isp.ko */
int vpss_isp_init(void __iomem *base);
int vpss_isp_set_attr(u32 grp, u32 w, u32 h, u32 *attr);
int vpss_isp_set_val(u32 grp, u32 w, u32 h, u32 first);
int vpss_isp_post_deal(u32 grp, u32 w, u32 h);
/* open_isp only, taken with symbol_get */
u32 vpss_isp_online(u32 grp);

static int rst = 1;
module_param(rst, int, 0444);
MODULE_PARM_DESC(rst, "reset the engine: 0 never, 1 at init and each picture, 2 at init");
static int spec;
module_param(spec, int, 0444);
MODULE_PARM_DESC(spec, "the vendor's special memory modes: 0, 1 (a frame of latency), 4 (always)");

struct chn {
	u32 enabled;
	u32 fmt, w, h, src_rate, dst_rate, mirror, flip;
	u32 crop[5];	   /* enable, x, y, w, h */
	u32 win[N_WIN][8]; /* rgn: enable, x, y, w, h, colour, fill, thick */
	u32 osd[8][9];	   /* vpss_set_chn_osd's, kept */
	u32 bind[3];
	u32 depth, count;
	struct list_head q;
	wait_queue_head_t wq;
	u32 splice, splice_id;
	u32 acc;
	/* /proc */
	u64 rate_pts;
	u32 rate_cnt, rate, sent, vb_ok, vb_fail;
};

struct grp {
	u32 created, started, first;
	u32 attr[28]; /* max w, h, ... */
	u32 crop[5];  /* enable, x, y, w, h */
	u32 fwd[3];   /* where the input goes on */
	u32 recv, timeouts, src_w, src_h, src_s;
	u64 time;
	struct chn chn[N_CHN];
	u32 dbg_on, dbg_n, dbg_addr[64], dbg_val[64];
};

/* the engine's registers for one picture */
struct job {
	u32 in[0x27];		 /* input stage: shadow words 0..0x26 */
	u32 scale[N_CHN];	 /* scaler enable */
	u32 v[N_CHN][7], h[N_CHN][7]; /* num den int frac recip flag predown */
	u32 crop[N_CHN][4];
	u32 stride[N_CHN][6], mirror[N_CHN], store[N_CHN], fmt[N_CHN];
	u32 addr[N_CHN][3], last[N_CHN][3], size[N_CHN][3];
	u32 win_en[N_CHN], win_fill[N_CHN], win[N_CHN][N_WIN][4];
};

struct splice {
	void *node;
	u32 state;
	u64 pts, interval;
	struct chn *owner;
};

static struct grp grps[N_GRP];
static struct splice splices[N_SPLICE];
static struct job job;
static DEFINE_MUTEX(vpss_mutex);
static DECLARE_WAIT_QUEUE_HEAD(engine_wq);
static atomic_t opens = ATOMIC_INIT(0);
static struct task_struct *thread;
static void __iomem *regs;

#define N_OUT 32
static void *out[N_OUT];

static inline void reg_w(u32 off, u32 v)
{
	writel(v, regs + off);
}

static u32 list_of(void *b)
{
	switch (WORD(b, W_BIND)) {
	case MOD_VENC:
		return 2;
	case MOD_VPSS:
		return VB_VPSS;
	case MOD_VAR:
		return 3;
	}
	return VB_FREE;
}

static void put_on(void *b)
{
	vb_put_buf(list_of(b), b);
}

static void queue_flush(struct chn *c)
{
	while (c->count) {
		struct list_head *n = c->q.next;

		list_del(n);
		c->count--;
		put_on(n);
	}
}

static bool takes_frame(struct chn *c)
{
	u32 s = c->src_rate, d = c->dst_rate;

	if (!s || !d || d >= s)
		return true;
	c->acc += d;
	if (c->acc < s)
		return false;
	c->acc -= s;
	return true;
}

/* the channel scaler's ratio, as the XM535's: in > out only */
static int ratio(u32 in, u32 out, u32 r[7])
{
	u32 num, den, a, b, t, k;

	memset(r, 0, 7 * sizeof(u32));
	if (out >= in) {
		r[0] = r[1] = 0x400;
		r[2] = 1;
		r[4] = 0x100;
		r[5] = 1;
		return 0;
	}
	if (out < 2 || in >= out * 16)
		return -1;
	if (in >= out * 8) {
		r[6] = 3;
		out *= 4;
	} else if (in >= out * 4) {
		r[6] = 1;
		out *= 2;
	}
	a = in - 1;
	b = out - 1;
	while (b) {
		t = a % b;
		a = b;
		b = t;
	}
	num = (out - 1) / a;
	den = (in - 1) / a;
	if (den <= 0x400) {
		k = 0x400 / den;
		den *= k;
		num *= k;
		if (den < 0x400) {
			den *= 2;
			num *= 2;
		}
	}
	r[0] = num;
	r[1] = den;
	r[2] = den / num;
	r[3] = den - num * r[2];
	r[4] = (num / 2 + 0x40000) / num;
	r[5] = num * r[4] <= 0x40000;
	return 0;
}

static u32 gcd(u32 a, u32 b)
{
	while (b) {
		u32 t = a % b;

		a = b;
		b = t;
	}
	return a;
}

/* the input stage's ratio from in to mid pixels: num, den, half, recip */
static void mid_ratio(u32 in, u32 mid, u32 *r)
{
	u32 g, q, p, k;

	if (in == mid) {
		r[0] = r[1] = r[2] = 0x400;
		r[3] = 0x100;
		return;
	}
	g = gcd(in, mid);
	q = mid / g;
	p = in / g;
	if (q > 0x400) {
		/* too fine: the nearest multiples of 4 */
		u32 m4 = (mid + 3) & ~3u, i4 = in & ~3u;

		g = gcd(i4, m4);
		q = m4 / g;
		p = i4 / g;
	}
	k = 0x400 / q;
	r[0] = q * k;
	r[1] = p * k;
	r[2] = (r[0] + 1) >> 1;
	r[3] = (r[2] + 0x40000) / r[0];
}

/* this picture's buffer for channel c: its own or its splice's */
static void *take_buffer(struct chn *c, u32 size, u32 w, u32 h)
{
	u32 m = c->splice, stride = ALIGN(w, 16);
	struct splice *s;
	void *b;

	if (!m) {
		if (vb_get_buf(VB_FREE, size, &b, 0))
			return NULL;
		WORD(b, W_STRIDE) = stride;
		WORD(b, W_WIDTH) = w;
		WORD(b, W_HEIGHT) = h;
		return b;
	}
	s = &splices[c->splice_id];
	if (s->node && (s->state & 1u << m)) {
		vb_put_buf(VB_FREE, s->node);
		s->node = NULL;
		s->state = 0;
	}
	if (!s->node) {
		if (vb_get_buf(VB_FREE, size * 2, &b, 0))
			return NULL;
		s->node = b;
		s->state = 0;
	}
	b = s->node;
	WORD(b, W_STRIDE) = m >= 3 ? stride * 2 : stride;
	WORD(b, W_WIDTH) = m >= 3 ? w * 2 : w;
	WORD(b, W_HEIGHT) = m <= 2 ? h * 2 : h;
	return b;
}

static void store_addr(int n, void *b, u32 m, u32 w, u32 stride, u32 y,
		       u32 uv, u32 third)
{
	u32 o = WORD(b, W_PHYS) - DDR_BASE, *a = job.addr[n], *l = job.last[n];

	switch (m) {
	case 0:
		a[0] = o >> 4;
		a[1] = (o + y) >> 4;
		a[2] = (o + y + uv) >> 4;
		l[0] = (o - w + y) >> 4;
		l[1] = (o - w + y + uv) >> 4;
		l[2] = (o - w + y + uv + third) >> 4;
		job.size[n][2] = third;
		break;
	case 1: /* top */
		a[0] = o >> 4;
		a[1] = (o + 2 * y) >> 4;
		l[0] = (o - w + y) >> 4;
		l[1] = (o - w + uv + 2 * y) >> 4;
		break;
	case 2: /* bottom */
		a[0] = (o + y) >> 4;
		a[1] = (o + uv + 2 * y) >> 4;
		l[0] = (o - w + 2 * y) >> 4;
		l[1] = (o - w + 2 * y + 2 * uv) >> 4;
		break;
	case 3: /* left */
		job.stride[n][0] = job.stride[n][2] = stride * 2 >> 4;
		a[0] = o >> 4;
		a[1] = (o + 2 * y) >> 4;
		l[0] = (o - 2 * stride + 2 * y) >> 4;
		l[1] = (o - 2 * stride + 2 * y + 2 * uv) >> 4;
		break;
	case 4: /* right */
		job.stride[n][0] = job.stride[n][2] = stride * 2 >> 4;
		a[0] = (o + stride) >> 4;
		a[1] = (o + stride + 2 * y) >> 4;
		l[0] = (o - stride + 2 * y) >> 4;
		l[1] = (o - stride + 2 * y + 2 * uv) >> 4;
		break;
	}
	job.size[n][0] = y;
	job.size[n][1] = uv;
}

/*
 * Channel n of g for this picture, from the middle picture mid_w x mid_h;
 * its buffer in *bufp. Returns whether the engine stores it.
 */
static bool chn_job(struct grp *g, int n, void *in, u32 mid_w, u32 mid_h,
		    void **bufp, bool *passed)
{
	struct chn *c = &g->chn[n];
	u32 w = c->crop[0] ? c->crop[3] : c->w;
	u32 h = c->crop[0] ? c->crop[4] : c->h;
	u32 stride = ALIGN(w, 16), y = h * stride, uv = y, third = y, i;
	bool secondary = c->splice == 2 || c->splice == 4;
	void *b = NULL;

	*bufp = NULL;
	if (c->fmt == FMT_SP420) {
		third = 0;
		uv = y / 2;
	}
	if (!c->enabled || !(secondary || takes_frame(c)))
		return false;
	if (c->bind[0] == MOD_VO) {
		if (vo_get_buf(c->bind[1], c->bind[2], &b)) {
			c->vb_fail++;
			return false;
		}
		if (WORD(b, W_SIZE) < y + uv + third) {
			vo_put_buf(c->bind[1], c->bind[2], b, 0);
			return false;
		}
	} else if (n == 0 && !c->splice && !*passed && !g->crop[0] &&
		   !c->crop[0] && !g->attr[6] && c->w == WORD(in, W_WIDTH) &&
		   c->h == WORD(in, W_HEIGHT)) {
		/* the input as it is */
		*passed = true;
		*bufp = in;
		c->vb_ok++;
		return false;
	} else {
		b = take_buffer(c, y + uv + third, w, h);
		if (!b) {
			c->vb_fail++;
			return false;
		}
	}
	if (ratio(mid_w, c->w, job.h[n]) || ratio(mid_h, c->h, job.v[n])) {
		pr_err("vpss too small %ux%u -> %ux%u\n", mid_w, mid_h, c->w, c->h);
		if (c->bind[0] == MOD_VO)
			vo_put_buf(c->bind[1], c->bind[2], b, 0);
		else if (!c->splice)
			vb_put_buf(VB_FREE, b);
		return false;
	}
	c->vb_ok++;
	*bufp = b;
	job.scale[n] = c->w < mid_w || c->h < mid_h;
	if (c->crop[0]) {
		job.crop[n][0] = c->crop[1];
		job.crop[n][1] = c->crop[1] + w;
		job.crop[n][2] = c->crop[2];
		job.crop[n][3] = c->crop[2] + h;
	} else {
		job.crop[n][1] = w;
		job.crop[n][3] = h;
	}
	for (i = 0; i < N_WIN; i++) {
		const u32 *a = c->win[i];

		if (!a[0])
			continue;
		job.win_en[n] |= 1u << i;
		job.win_fill[n] |= a[6] << i;
		job.win[n][i][0] = a[1] | a[2] << 16;
		job.win[n][i][1] = (a[1] + a[3]) | (a[2] + a[4]) << 16;
		job.win[n][i][2] = a[5];
		job.win[n][i][3] = a[7];
	}
	for (i = 0; i < 6; i++)
		job.stride[n][i] = stride >> 4;
	job.fmt[n] = c->fmt;
	job.mirror[n] = c->mirror << 2 | c->flip << 1 | c->mirror;
	store_addr(n, b, c->splice, w, stride, y, uv, third);
	job.store[n] = 1;
	return true;
}

/* the input stage: in (cropped by the group) to mid_w x mid_h */
static void in_job(struct grp *g, void *in, u32 mid_w, u32 mid_h, u32 in_w,
		   u32 in_h)
{
	u32 *r = job.in, s = WORD(in, W_STRIDE), o = WORD(in, W_PHYS) - DDR_BASE;

	r[0] = 1;
	r[3] = mid_w + 0x40;
	r[4] = mid_h + 0x10;
	r[5] = 0;
	r[6] = mid_w;
	r[7] = 0x20;
	r[8] = mid_h + 8;
	r[9] = 1;
	r[10] = 1;
	r[11] = 1;
	if (g->crop[0]) {
		r[12] = (s * g->crop[2] + g->crop[1] + o) >> 4;
		r[15] = (s * (WORD(in, W_HEIGHT) + (g->crop[2] >> 1)) + g->crop[1] + o) >> 4;
	} else {
		r[12] = o >> 4;
		r[15] = (s * WORD(in, W_HEIGHT) + o) >> 4;
	}
	r[13] = r[14] = r[16] = r[17] = s >> 4;
	r[18] = WORD(in, W_HEIGHT);
	r[19] = 1;
	mid_ratio(in_w, mid_w, r + 20);
	mid_ratio(in_h, mid_h, r + 24);
	r[28] = 0;
	r[29] = 0;
	r[30] = WORD(in, W_WIDTH);
	r[31] = 1;
	r[32] = 0;
	r[33] = mid_w;
	r[34] = 0;
	r[35] = mid_h;
	if (mid_w < g->chn[0].w) {
		/* channel 0 wider than the engine: its own first stage */
		r[3] = g->chn[0].w + 0x40;
		r[36] = 0;
		r[37] = (mid_w << 10) / g->chn[0].w;
		r[38] = g->chn[0].w;
	} else {
		r[36] = 1;
	}
}

static void update_regs(void)
{
	/* the input stage's shadow words 0..31 (0: not written) */
	static const u32 in_regs[] = {
		0x12004, 0, 0, 0x10000, 0x10004, 0x10008, 0x1000c, 0x10010,
		0x10014, 0, 0x10024, 0x10040, 0x11000, 0x11008, 0x1100c, 0x11010,
		0x11018, 0x1101c, 0x11020, 0x11040, 0x13000, 0x13004, 0x13008,
		0x1300c, 0x13010, 0x13014, 0x13018, 0x1301c, 0x13020, 0x13024,
		0x13028, 0x1302c,
	};
	u32 i, k;

	for (i = 0; i < ARRAY_SIZE(in_regs); i++)
		if (in_regs[i])
			reg_w(in_regs[i], job.in[i]);
	reg_w(0xb020, 1);
	for (i = 0; i < 4; i++)
		reg_w(0xb000 + 4 * i, job.in[32 + i]);
	reg_w(0x8e000, job.in[36]);
	reg_w(0x8e008, job.in[37]);
	reg_w(0x8e00c, job.in[38]);
	for (i = 0; i < N_CHN; i++) {
		u32 c = i * 0x1000, s = 0x80000 + i * 0x40;

		reg_w(c + 0x90034, job.store[i]);
		reg_w(c + 0x81020, 1);
		if (!job.store[i]) {
			reg_w(c + 0x81000, 0);
			reg_w(c + 0x81004, ~0u);
			reg_w(c + 0x81008, 0);
			reg_w(c + 0x8100c, ~0u);
			continue;
		}
		for (k = 0; k < 6; k++) {
			reg_w(s + 0x04 + 4 * k, job.v[i][k]);
			reg_w(s + 0x1c + 4 * k, job.h[i][k]);
		}
		reg_w(s + 0x00, job.scale[i]);
		reg_w(s + 0x34, job.h[i][6]);
		reg_w(s + 0x3c, job.v[i][6]);
		for (k = 0; k < 4; k++)
			reg_w(c + 0x81000 + 4 * k, job.crop[i][k]);
		for (k = 0; k < 4; k++)
			reg_w(c + 0x90020 + 4 * k, job.stride[i][k]);
		reg_w(c + 0x90030, job.mirror[i]);
		reg_w(c + 0x90038, 0);
		reg_w(c + 0x90054, 1);
		reg_w(c + 0x90000, job.addr[i][0]);
		reg_w(c + 0x90010, job.addr[i][1]);
		reg_w(c + 0x90080, job.last[i][0]);
		reg_w(c + 0x90090, job.last[i][1]);
		reg_w(c + 0x90058, job.size[i][0]);
		reg_w(c + 0x9005c, job.size[i][1]);
		if (i == 2) {
			reg_w(c + 0x900e0, job.stride[i][4]);
			reg_w(c + 0x900e4, job.stride[i][5]);
			reg_w(c + 0x900c0, job.addr[i][2]);
			reg_w(c + 0x900d0, job.last[i][2]);
			reg_w(c + 0x900e8, job.size[i][2]);
			reg_w(c + 0x90078, job.fmt[i]);
		}
		/* the engine's own OSD: off (rgn.ko draws on the encoder's side) */
		for (k = 0; k < 8; k++) {
			reg_w(c + 0x84004, 0x100);
			reg_w(c + 0x84004, 0x101);
			reg_w(c + 0x84000, 0);
		}
		for (k = 0; k < N_WIN; k++)
			if (job.win_en[i] & 1u << k) {
				u32 o = c + 0x84140 + k * 0x10;

				reg_w(o, job.win[i][k][0]);
				reg_w(o + 4, job.win[i][k][1]);
				reg_w(o + 8, job.win[i][k][2]);
				reg_w(o + 12, job.win[i][k][3]);
			}
		reg_w(c + 0x84100, job.win_en[i]);
		reg_w(c + 0x84104, job.win_fill[i]);
	}
	reg_w(0x10018, 1);
}

static irqreturn_t vpss_int(int irq, void *dev)
{
	irqreturn_t ret = IRQ_NONE;

	if (readl(regs + 0x10044)) {
		writel(0, regs + 0x10044);
		ret = IRQ_HANDLED;
	}
	wake_up(&engine_wq);
	return ret;
}

static void deliver(struct chn *c, void *b, u64 now)
{
	if (c->bind[0] == MOD_VO) {
		vo_put_buf(c->bind[1], c->bind[2], b, 1);
	} else if (!c->depth) {
		put_on(b);
	} else {
		list_add_tail((struct list_head *)b, &c->q);
		if (++c->count > c->depth) {
			struct list_head *n = c->q.next;

			list_del(n);
			c->count--;
			put_on(n);
		}
		wake_up(&c->wq);
	}
	c->sent++;
	if (now - c->rate_pts >= 4000000) {
		c->rate = (c->sent - c->rate_cnt + 2) / 4;
		c->rate_cnt = c->sent;
		c->rate_pts = now;
	}
}

static void deliver_all(struct grp *g, void *in, void **bufs, u64 now)
{
	int i;

	for (i = 0; i < N_CHN; i++) {
		struct chn *c = &g->chn[i], *to = c;
		struct splice *s;
		void *b = bufs[i];
		u32 m = c->splice;

		if (!b)
			continue;
		WORD(b, W_PTS) = WORD(in, W_PTS);
		WORD(b, W_PTS + 1) = WORD(in, W_PTS + 1);
		memcpy(&WORD(b, W_BIND), c->bind, sizeof(c->bind));
		if (m && c->bind[0] != MOD_VO) {
			s = &splices[c->splice_id];
			if (m == 1 || m == 3)
				s->owner = c;
			s->state |= 1u << m;
			if (s->state != 0x6 && s->state != 0x18) {
				s->pts = now;
				continue;
			}
			s->interval = now - s->pts;
			s->node = NULL;
			s->state = 0;
			if (s->owner) {
				to = s->owner;
				memcpy(&WORD(b, W_BIND), to->bind, sizeof(to->bind));
			}
		}
		deliver(to, b, now);
	}
}

static void drop_all(struct grp *g, void **bufs, void *in)
{
	int i;

	for (i = 0; i < N_CHN; i++) {
		struct chn *c = &g->chn[i];

		if (!bufs[i] || bufs[i] == in || c->splice)
			continue;
		if (c->bind[0] == MOD_VO)
			vo_put_buf(c->bind[1], c->bind[2], bufs[i], 0);
		else
			vb_put_buf(VB_FREE, bufs[i]);
	}
}

/*
 * Online (HDR): the BPU's output goes straight to the scalers, and each
 * channel's store writes it at its frame end, as the vendor's vi.ko drives
 * it (vi_yuv_store_int; vpss.ko has no such path). At a channel's frame
 * start (+0x8705C) the next buffer is programmed, at its end (+0x8704C) the
 * one filled is done; the thread hands it on. Group 0 alone (one sensor),
 * its channels' sizes and rates; no splices or VO.
 */
#define ON_IRQ 68
#define ON_DONE 16

static u32 (*isp_online)(u32 grp);
static bool isp_asked;
static DEFINE_SPINLOCK(on_lock);
static u32 on_wh, on_gen, on_set;
static void *on_now[N_CHN], *on_next[N_CHN];
static bool on_valid[N_CHN];
static struct {
	void *b;
	u32 chn;
	u64 pts;
} on_done[ON_DONE];
static u32 on_head, on_tail;

static void on_drop(void *b)
{
	memset(&WORD(b, W_BIND), 0, 12);
	vb_put_buf(VB_FREE, b);
}

static void on_program(int i, struct chn *c, void *b)
{
	u32 stride = ALIGN(c->w, 16), y = stride * c->h, uv = y / 2;
	u32 o = WORD(b, W_PHYS) - DDR_BASE, r = i * 0x1000 + 0x90000;

	WORD(b, W_STRIDE) = stride;
	WORD(b, W_WIDTH) = c->w;
	WORD(b, W_HEIGHT) = c->h;
	reg_w(r + 0x00, o >> 4);
	reg_w(r + 0x10, (o + y) >> 4);
	/* the last lines, for mirror and flip */
	reg_w(r + 0x80, (o - stride + y) >> 4);
	reg_w(r + 0x90, (o - stride + y + uv) >> 4);
	reg_w(r + 0x58, y >> 4);
	reg_w(r + 0x5c, uv >> 4);
	reg_w(r + 0x34, 1);
}

static void on_frame_start(int i)
{
	struct chn *c = &grps[0].chn[i];
	u32 k;

	if (c->enabled && takes_frame(c)) {
		if (on_next[i])
			on_drop(on_next[i]);
		on_next[i] = NULL;
		if (vb_get_buf(VB_FREE, ALIGN(c->w, 16) * c->h * 3 / 2,
			       &on_next[i], 0) == 0)
			c->vb_ok++;
		else
			c->vb_fail++;
	}
	if (on_next[i])
		on_program(i, c, on_next[i]);
	else
		reg_w(i * 0x1000 + 0x90034, 0);
	/* rgn.ko's covers */
	for (k = 0; k < N_WIN; k++)
		if (c->win[k][0]) {
			const u32 *a = c->win[k];
			u32 o = i * 0x1000 + 0x84140 + k * 0x10;

			reg_w(o, a[1] | a[2] << 16);
			reg_w(o + 4, (a[1] + a[3]) | (a[2] + a[4]) << 16);
			reg_w(o + 8, a[5]);
			reg_w(o + 12, a[7]);
		}
}

static void on_frame_end(int i, u64 pts)
{
	void *last = on_now[i];

	on_now[i] = on_next[i];
	on_next[i] = NULL;
	if (last && on_valid[i] && (on_head + 1) % ON_DONE != on_tail) {
		on_done[on_head].b = last;
		on_done[on_head].chn = i;
		on_done[on_head].pts = pts;
		on_head = (on_head + 1) % ON_DONE;
	} else if (last) {
		on_drop(last);
	}
	reg_w(i * 0x1000 + 0x90034, 0);
	/* the hardware took the buffer at the frame start */
	on_valid[i] = on_now[i] && readl(regs + i * 0x1000 + 0x90100) !=
				   readl(regs + i * 0x1000 + 0x90108);
}

static irqreturn_t on_int(int irq, void *dev)
{
	irqreturn_t ret = IRQ_NONE;
	unsigned long flags;
	u64 pts;
	int i;

	clk_get_pts(&pts);
	spin_lock_irqsave(&on_lock, flags);
	for (i = 0; i < N_CHN; i++) {
		u32 c = i * 0x1000;

		if (readl(regs + c + 0x8704c)) {
			writel(0, regs + c + 0x8704c);
			if (on_wh)
				on_frame_end(i, pts);
			ret = IRQ_HANDLED;
		}
		if (readl(regs + c + 0x8705c)) {
			writel(0, regs + c + 0x8705c);
			if (on_wh)
				on_frame_start(i);
			ret = IRQ_HANDLED;
		}
	}
	spin_unlock_irqrestore(&on_lock, flags);
	return ret;
}

/* the store and scalers for in_w x in_h; with on_lock held */
static void on_setup(u32 in_w, u32 in_h)
{
	u32 i, k, h[7], v[7];

	reg_w(0x87044, 0x1e);
	for (i = 0; i < N_CHN; i++) {
		struct chn *c = &grps[0].chn[i];
		u32 n = i * 0x1000, s = n + 0x90000, sc = 0x80000 + i * 0x40;
		bool on = c->enabled && c->w && c->h &&
			  !ratio(in_w, c->w, h) && !ratio(in_h, c->h, v);

		reg_w(s + 0x34, 0);
		reg_w(n + 0x87040, 0);
		reg_w(n + 0x87050, 0);
		if (!on)
			continue;
		reg_w(s + 0x54, 3);
		reg_w(s + 0x38, 0);
		for (k = 0; k < 4; k++)
			reg_w(s + 0x20 + 4 * k, ALIGN(c->w, 16) >> 4);
		reg_w(s + 0x30, c->mirror << 2 | c->flip << 1 | c->mirror);
		reg_w(s + 0x78, c->fmt);
		reg_w(s + 0x40, 0xff);
		reg_w(s + 0x44, 0xffffffff);
		reg_w(sc + 0x00, in_w > c->w || in_h > c->h);
		for (k = 0; k < 6; k++) {
			reg_w(sc + 0x04 + 4 * k, v[k]);
			reg_w(sc + 0x1c + 4 * k, h[k]);
		}
		reg_w(sc + 0x34, h[6]);
		reg_w(sc + 0x3c, v[6]);
		reg_w(n + 0x81000, 0);
		reg_w(n + 0x81004, c->w);
		reg_w(n + 0x81008, 0);
		reg_w(n + 0x8100c, c->h);
		reg_w(n + 0x81020, 1);
		reg_w(n + 0x84100, 0);
		for (k = 0; k < N_WIN; k++)
			if (c->win[k][0])
				reg_w(n + 0x84100, readl(regs + n + 0x84100) | 1u << k);
		reg_w(n + 0x87044, 3);
		reg_w(n + 0x87040, 1);
		reg_w(n + 0x87054, c->h * 15 / 16);
		reg_w(n + 0x87050, 1);
	}
}

/* the thread, with vpss_mutex held: follows the ISP's mode, hands on the
 * pictures stored */
static void online(void)
{
	struct grp *g = &grps[0];
	unsigned long flags;
	u32 wh = 0, i;

	if (!isp_online && !isp_asked && g->started) {
		isp_online = symbol_get(vpss_isp_online);
		isp_asked = true;
	}
	if (isp_online && g->created && g->started && atomic_read(&opens))
		wh = isp_online(0);
	if (!(wh & 0xffff) || !(wh >> 16))
		wh = 0;
	spin_lock_irqsave(&on_lock, flags);
	if (wh != on_wh || (wh && on_set != on_gen)) {
		on_wh = wh;
		on_set = on_gen;
		if (wh) {
			on_setup(wh & 0xffff, wh >> 16);
		} else {
			for (i = 0; i < N_CHN; i++) {
				reg_w(i * 0x1000 + 0x90034, 0);
				reg_w(i * 0x1000 + 0x87040, 0);
				reg_w(i * 0x1000 + 0x87050, 0);
			}
		}
		for (i = 0; i < N_CHN; i++) {
			if (on_now[i])
				on_drop(on_now[i]);
			if (on_next[i])
				on_drop(on_next[i]);
			on_now[i] = on_next[i] = NULL;
			on_valid[i] = false;
		}
	}
	while (on_tail != on_head) {
		void *b = on_done[on_tail].b;
		struct chn *c = &g->chn[on_done[on_tail].chn];
		u64 pts = on_done[on_tail].pts;

		on_tail = (on_tail + 1) % ON_DONE;
		spin_unlock_irqrestore(&on_lock, flags);
		if (c == &g->chn[0]) {
			g->recv++;
			g->src_w = wh & 0xffff;
			g->src_h = wh >> 16;
			g->src_s = ALIGN(g->src_w, 16);
		}
		WORD(b, W_PTS) = (u32)pts;
		WORD(b, W_PTS + 1) = pts >> 32;
		memcpy(&WORD(b, W_BIND), c->bind, sizeof(c->bind));
		if (c->enabled && c->bind[0] != MOD_VO)
			deliver(c, b, pts);
		else
			on_drop(b);
		spin_lock_irqsave(&on_lock, flags);
	}
	spin_unlock_irqrestore(&on_lock, flags);
}

/* one input picture of group g; with vpss_mutex held */
static void process(struct grp *g, u32 grp, void *in)
{
	u32 in_w = g->crop[0] ? g->crop[3] : WORD(in, W_WIDTH);
	u32 in_h = g->crop[0] ? g->crop[4] : WORD(in, W_HEIGHT);
	u32 mid_w = in_w, mid_h = in_h, isp_w, mask = 0;
	void *bufs[N_CHN] = {NULL};
	bool passed = false;
	u64 t0, now;
	long t;
	int i;

	for (i = 0; i < N_CHN; i++) {
		mid_w = max(mid_w, g->chn[i].w);
		mid_h = max(mid_h, g->chn[i].h);
	}
	mid_w = min(mid_w, (u32)MID_MAX);
	memset(&job, 0, sizeof(job));
	in_job(g, in, mid_w, mid_h, in_w, in_h);
	for (i = 0; i < N_CHN; i++)
		if (chn_job(g, i, in, mid_w, mid_h, &bufs[i], &passed))
			mask |= 1u << i;
	clk_get_pts(&t0);
	if (!mask) {
		deliver_all(g, in, bufs, t0);
		goto input;
	}
	if (rst == 1) {
		reg_w(0x12000, 0);
		udelay(10);
		reg_w(0x12000, 3);
		udelay(10);
	}
	isp_w = max(mid_w, WORD(in, W_STRIDE));
	vpss_isp_set_val(grp, isp_w, mid_h, g->first);
	update_regs();
	if (g->dbg_on)
		for (i = 0; i < g->dbg_n && i < 64; i++)
			writel(g->dbg_val[i], regs + (g->dbg_addr[i] & (VPSS_SIZE - 4)));
	reg_w(0x10020, 1);
	g->first = 0;
	t = wait_event_interruptible_timeout(engine_wq, !readl(regs + 0x10020),
					     2 * HZ);
	clk_get_pts(&now);
	if (readl(regs + 0x10020)) {
		pr_err("vpss: grp %u timeout (%ld)\n", grp, t);
		g->timeouts++;
		drop_all(g, bufs, in);
		goto input;
	}
	g->time = now - t0;
	vpss_isp_post_deal(grp, isp_w, mid_h);
	deliver_all(g, in, bufs, now);
input:
	if (passed)
		return; /* the input went on as channel 0's */
	if (g->fwd[0] == MOD_VPSS || g->fwd[0] == MOD_VENC) {
		memcpy(&WORD(in, W_BIND), g->fwd, sizeof(g->fwd));
		put_on(in);
	} else {
		vb_put_buf(VB_FREE, in);
	}
}

static int vpss_thread(void *arg)
{
	void *in;

	while (!kthread_should_stop()) {
		struct grp *g;
		u32 grp;

		/* spec=1: a picture is taken only with another queued behind
		 * it (vpss.ko's one frame of latency) */
		int queued = 2;

		if (spec == 1)
			vb_get_length(VB_VPSS, &queued);
		if (!atomic_read(&opens) || queued < 2 ||
		    vb_get_buf(VB_VPSS, 0, &in, 0)) {
			mutex_lock(&vpss_mutex);
			online();
			mutex_unlock(&vpss_mutex);
			schedule_timeout_interruptible(1);
			continue;
		}
		grp = WORD(in, W_BIND + 1);
		if (WORD(in, W_BIND) != MOD_VPSS || grp >= N_GRP) {
			vb_put_buf(VB_FREE, in);
			continue;
		}
		g = &grps[grp];
		mutex_lock(&vpss_mutex);
		online();
		g->recv++;
		g->src_s = WORD(in, W_STRIDE);
		g->src_w = WORD(in, W_WIDTH);
		g->src_h = WORD(in, W_HEIGHT);
		if (g->created && g->started &&
		    WORD(in, W_STRIDE) * WORD(in, W_HEIGHT) <= g->attr[0] * g->attr[1])
			process(g, grp, in);
		else
			vb_put_buf(VB_FREE, in);
		mutex_unlock(&vpss_mutex);
	}
	return 0;
}

/* rgn.ko: cover idx of a channel */
int vpss_set_chn_win(u32 grp, u32 n, u32 idx, u32 *w)
{
	if (grp >= N_GRP || !grps[grp].created || n >= N_CHN || idx >= N_WIN) {
		pr_err("vpss: grp %u not created\n", grp);
		return -1;
	}
	mutex_lock(&vpss_mutex);
	memcpy(grps[grp].chn[n].win[idx], w, 8 * sizeof(u32));
	mutex_unlock(&vpss_mutex);
	return 0;
}
EXPORT_SYMBOL(vpss_set_chn_win);

/* the engine's own OSD regions: kept, not drawn */
int vpss_set_chn_osd(u32 grp, u32 n, u32 idx, u32 *o)
{
	if (grp >= N_GRP || !grps[grp].created || n >= N_CHN || idx >= 8)
		return -1;
	mutex_lock(&vpss_mutex);
	memcpy(grps[grp].chn[n].osd[idx], o, 9 * sizeof(u32));
	mutex_unlock(&vpss_mutex);
	return 0;
}
EXPORT_SYMBOL(vpss_set_chn_osd);

/* as vpss.ko: VI has no covers here */
int vi_set_chn_win(u32 dev, u32 n, u32 idx, u32 *w)
{
	return 0;
}
EXPORT_SYMBOL(vi_set_chn_win);

static void vpss_init(void)
{
	int g, c;

	if (rst) {
		reg_w(0x12000, 0);
		udelay(100);
		reg_w(0x12000, 3);
		udelay(100);
	}
	mutex_lock(&vpss_mutex);
	memset(grps, 0, sizeof(grps));
	memset(splices, 0, sizeof(splices));
	memset(out, 0, sizeof(out));
	for (g = 0; g < N_GRP; g++)
		for (c = 0; c < N_CHN; c++) {
			INIT_LIST_HEAD(&grps[g].chn[c].q);
			init_waitqueue_head(&grps[g].chn[c].wq);
		}
	isp_asked = false;
	mutex_unlock(&vpss_mutex);
	vpss_isp_init(regs);
}

static int out_take(void *b)
{
	int i;

	for (i = 0; i < N_OUT; i++)
		if (!out[i]) {
			out[i] = b;
			return 0;
		}
	return -1;
}

static bool out_give(void *b)
{
	int i;

	for (i = 0; i < N_OUT; i++)
		if (b && out[i] == b) {
			out[i] = NULL;
			return true;
		}
	return false;
}

static long get_frame(struct chn *c, u32 *w)
{
	struct list_head *n;
	long t;
	void *b;

	t = wait_event_interruptible_timeout(c->wq, c->count,
					     msecs_to_jiffies(w[2]));
	mutex_lock(&vpss_mutex);
	if (!c->count) {
		mutex_unlock(&vpss_mutex);
		return t < 0 ? t : t ? -1 : -ERESTARTSYS;
	}
	n = c->q.next;
	list_del(n);
	c->count--;
	b = n;
	if (out_take(b)) {
		put_on(b);
		mutex_unlock(&vpss_mutex);
		return -EBUSY;
	}
	if (c->bind[0]) {
		vb_add_buf_reference(b);
		put_on(b);
	}
	w[4] = WORD(b, W_PHYS);
	w[5] = w[4] + WORD(b, W_STRIDE) * WORD(b, W_HEIGHT);
	w[6] = w[4] + WORD(b, W_STRIDE) * WORD(b, W_HEIGHT) * 2;
	w[7] = c->fmt;
	w[8] = WORD(b, W_STRIDE);
	w[9] = WORD(b, W_WIDTH);
	w[10] = WORD(b, W_HEIGHT);
	w[12] = WORD(b, W_PTS);
	w[13] = WORD(b, W_PTS + 1);
	w[14] = (u32)b;
	mutex_unlock(&vpss_mutex);
	return 0;
}

static long vpss_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	void __user *uarg = (void __user *)arg;
	struct grp *g;
	struct chn *c;
	u32 w[32], dbg[131], n;
	long ret = 0;
	bool back = false;

	switch (cmd) {
	case VPSS_INIT:
		vpss_init();
		return 0;
	case VPSS_DESTROY_ALL:
		mutex_lock(&vpss_mutex);
		for (n = 0; n < N_GRP; n++)
			grps[n].created = grps[n].started = 0;
		mutex_unlock(&vpss_mutex);
		return 0;
	case VPSS_START_GRP:
	case VPSS_STOP_GRP:
		if (get_user(n, (u32 __user *)uarg))
			return -EFAULT;
		if (n >= N_GRP || !grps[n].created)
			return -EINVAL;
		mutex_lock(&vpss_mutex);
		grps[n].started = cmd == VPSS_START_GRP;
		if (grps[n].started)
			grps[n].first = 1;
		mutex_unlock(&vpss_mutex);
		return 0;
	case VPSS_DEBUG:
		if (copy_from_user(dbg, uarg, sizeof(dbg)))
			return -EFAULT;
		if (dbg[0] >= N_GRP)
			return -EINVAL;
		mutex_lock(&vpss_mutex);
		g = &grps[dbg[0]];
		g->dbg_on = dbg[1];
		g->dbg_n = min(dbg[2], 64u);
		memcpy(g->dbg_addr, dbg + 3, sizeof(g->dbg_addr));
		memcpy(g->dbg_val, dbg + 67, sizeof(g->dbg_val));
		mutex_unlock(&vpss_mutex);
		return 0;
	}
	if (_IOC_TYPE(cmd) != 'P' || _IOC_SIZE(cmd) != sizeof(w))
		return -ENOTTY;
	if (copy_from_user(w, uarg, sizeof(w)))
		return -EFAULT;
	if (cmd == VPSS_RELEASE_FRAME) {
		void *b = (void *)w[14];

		mutex_lock(&vpss_mutex);
		if (out_give(b))
			vb_put_buf(VB_FREE, b);
		else
			ret = -EINVAL;
		mutex_unlock(&vpss_mutex);
		return ret;
	}
	if (w[0] >= N_GRP)
		return -EINVAL;
	g = &grps[w[0]];
	if (cmd == VPSS_CREATE_GRP) {
		if (g->created) {
			pr_err("vpss: grp %u exists\n", w[0]);
			return -EEXIST;
		}
		memcpy(g->attr, w + 4, sizeof(g->attr));
		if (g->attr[6] == 1)
			vpss_isp_set_attr(w[0], g->attr[0], g->attr[1], g->attr + 2);
		g->created = 1;
		return 0;
	}
	if (!g->created) {
		pr_err("vpss: grp %u not created\n", w[0]);
		return -EINVAL;
	}
	if (cmd == VPSS_BIND && w[1] == FWD_CHN) {
		mutex_lock(&vpss_mutex);
		memcpy(g->fwd, w + 4, sizeof(g->fwd));
		mutex_unlock(&vpss_mutex);
		return 0;
	}
	if (cmd == VPSS_SET_GRP_ATTR || cmd == VPSS_SET_GRP_ATTR2) {
		mutex_lock(&vpss_mutex);
		memcpy(g->attr, w + 4, sizeof(g->attr));
		vpss_isp_set_attr(w[0], g->attr[0], g->attr[1], g->attr + 2);
		mutex_unlock(&vpss_mutex);
		back = cmd == VPSS_SET_GRP_ATTR2;
		goto done;
	}
	if (w[1] >= N_CHN)
		return -EINVAL;
	c = &g->chn[w[1]];
	if (cmd == VPSS_GET_FRAME) {
		ret = get_frame(c, w);
		back = true;
		goto done;
	}
	mutex_lock(&vpss_mutex);
	switch (cmd) {
	case VPSS_SET_CHN_ATTR:
	case VPSS_SET_CHN_ATTR2:
		if ((w[1] < 2 && w[4] != FMT_SP420) || !w[5] || !w[6]) {
			pr_err("vpss: grp %u chn %u attributes\n", w[0], w[1]);
			ret = -EINVAL;
			break;
		}
		c->fmt = w[4];
		c->w = w[5];
		c->h = w[6];
		c->src_rate = w[7];
		c->dst_rate = w[8];
		c->mirror = w[9];
		c->flip = w[10];
		back = cmd == VPSS_SET_CHN_ATTR2;
		break;
	case VPSS_ENABLE_CHN:
	case VPSS_DISABLE_CHN:
		c->enabled = cmd == VPSS_ENABLE_CHN;
		break;
	case VPSS_BIND:
	case VPSS_SET_DEPTH:
		queue_flush(c);
		if (cmd == VPSS_BIND) {
			c->depth = 0;
			memcpy(c->bind, w + 4, sizeof(c->bind));
		} else {
			c->depth = w[4];
		}
		break;
	case VPSS_SET_GRP_CROP:
		g->crop[0] = w[4];
		g->crop[1] = w[5] & ~15u;
		g->crop[2] = w[6] & ~1u;
		g->crop[3] = w[7];
		g->crop[4] = w[8];
		break;
	case VPSS_SET_CHN_CROP:
		/* without the sums wrapping */
		if (w[5] > c->w || w[7] > c->w - w[5] || w[6] > c->h ||
		    w[8] > c->h - w[6]) {
			pr_err("vpss: crop out of range\n");
			ret = -EINVAL;
			break;
		}
		memcpy(c->crop, w + 4, sizeof(c->crop));
		break;
	case VPSS_SPLICE:
		if (w[5] >= N_SPLICE || w[4] > 4) {
			ret = -EINVAL;
			break;
		}
		c->splice = w[4];
		c->splice_id = w[5];
		if (!w[4] && splices[w[5]].node) {
			vb_put_buf(VB_FREE, splices[w[5]].node);
			splices[w[5]].node = NULL;
			splices[w[5]].state = 0;
		}
		break;
	default:
		ret = -ENOTTY;
	}
	if (w[0] == 0)
		on_gen++;
	mutex_unlock(&vpss_mutex);
done:
	if (back && copy_to_user(uarg, w, sizeof(w)))
		return -EFAULT;
	return ret;
}

static int vpss_open(struct inode *inode, struct file *file)
{
	atomic_inc(&opens);
	return 0;
}

static int vpss_close(struct inode *inode, struct file *file)
{
	atomic_dec(&opens);
	return 0;
}

static int vpss_show(struct seq_file *s, void *v)
{
	int i, j;

	seq_puts(s, "[VPSS] openxiongmai xm530v200_vpss\n\n");
	seq_puts(s, "---------------------------------VPSS CHN ATTR--------------------------------------\n");
	seq_puts(s, "GrpID  ChnID  Enable  Width  Height  PixFmt  Mirror  Flip  Depth  SrcRat  DstRat  Splice\n");
	for (i = 0; i < N_GRP; i++)
		for (j = 0; grps[i].created && j < N_CHN; j++) {
			const struct chn *c = &grps[i].chn[j];

			seq_printf(s, "%-5d  %-5d  %-6s  %-5u  %-6u  %-6u  %-6s  %-4s  %-5u  %-6u  %-6u  %u/%u\n",
				   i, j, c->enabled ? "Y" : "N", c->w, c->h, c->fmt,
				   c->mirror ? "Y" : "N", c->flip ? "Y" : "N", c->depth,
				   c->src_rate, c->dst_rate, c->splice, c->splice_id);
		}
	seq_puts(s, "---------------------------------VPSS GRP INFO--------------------------------------\n");
	seq_puts(s, "GrpID  Start  RecvPic  SrcS  SrcW  SrcH  Timeout  Time\n");
	for (i = 0; i < N_GRP; i++) {
		const struct grp *g = &grps[i];

		if (g->created)
			seq_printf(s, "%-5d  %-5s  %-7u  %-4u  %-4u  %-4u  %-7u  %-8llu\n",
				   i, g->started ? "Y" : "N", g->recv, g->src_s, g->src_w,
				   g->src_h, g->timeouts, g->time);
	}
	seq_puts(s, "---------------------------------VPSS CHN INFO--------------------------------------\n");
	seq_puts(s, "GrpID  ChnID  FrmRate  VBSucc  VBFail  SendCnt\n");
	for (i = 0; i < N_GRP; i++)
		for (j = 0; grps[i].created && j < N_CHN; j++) {
			const struct chn *c = &grps[i].chn[j];

			if (c->vb_ok || c->vb_fail)
				seq_printf(s, "%-5d  %-5d  %-7u  %-6u  %-6u  %-6u\n", i,
					   j, c->rate, c->vb_ok, c->vb_fail, c->sent);
		}
	return 0;
}

static int vpss_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, vpss_show, NULL);
}

static const struct file_operations vpss_proc_fops = {
	.owner = THIS_MODULE,
	.open = vpss_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static const struct file_operations vpss_fops = {
	.owner = THIS_MODULE,
	.open = vpss_open,
	.release = vpss_close,
	.unlocked_ioctl = vpss_ioctl,
};

static struct miscdevice vpss_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "vpss",
	.fops = &vpss_fops,
};

static int __init xm530v200_vpss_init(void)
{
	int ret;

	if (spec > 1 && spec != 4)
		pr_warn("vpss: spec=%d not supported, as spec=0\n", spec);
	regs = ioremap(VPSS_BASE, VPSS_SIZE);
	if (!regs) {
		pr_err("vpss: ioremap failed\n");
		return -ENOMEM;
	}
	vpss_init();
	if (!proc_create("umap/vpss", 0444, NULL, &vpss_proc_fops))
		pr_err("vpss: create vpss proc fail!\n");
	ret = misc_register(&vpss_misc);
	if (ret)
		goto proc;
	ret = request_irq(VPSS_IRQ, vpss_int, 0, "vpss", NULL);
	if (ret) {
		pr_err("vpss: request_irq failed\n");
		goto misc;
	}
	ret = request_irq(ON_IRQ, on_int, 0, "vpss_store", NULL);
	if (ret) {
		pr_err("vpss: request_irq failed\n");
		free_irq(VPSS_IRQ, NULL);
		goto misc;
	}
	thread = kthread_run(vpss_thread, NULL, "vpss");
	if (IS_ERR(thread)) {
		ret = PTR_ERR(thread);
		thread = NULL;
		goto irq;
	}
	pr_info("vpss: openxiongmai xm530v200_vpss init ok\n");
	return 0;
irq:
	free_irq(ON_IRQ, NULL);
	free_irq(VPSS_IRQ, NULL);
misc:
	misc_deregister(&vpss_misc);
proc:
	remove_proc_entry("umap/vpss", NULL);
	iounmap(regs);
	return ret;
}

static void __exit xm530v200_vpss_exit(void)
{
	if (thread)
		kthread_stop(thread);
	if (isp_online)
		symbol_put(vpss_isp_online);
	misc_deregister(&vpss_misc);
	free_irq(ON_IRQ, NULL);
	free_irq(VPSS_IRQ, NULL);
	remove_proc_entry("umap/vpss", NULL);
	iounmap(regs);
}

module_init(xm530v200_vpss_init);
module_exit(xm530v200_vpss_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("XM530V200 VPSS, compatible with vpss.ko");
