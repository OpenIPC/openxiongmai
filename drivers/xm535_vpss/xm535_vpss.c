// SPDX-License-Identifier: GPL-2.0
/*
 * The XM535D1/D2's VPSS (/dev/vpss): a drop-in for the vendor vpss.ko.
 *
 * VPSS scales a group's picture (one per sensor, up to 4) into three
 * channels and stores them into vb.ko's buffers. It has no interrupt of
 * its own: isp.ko drives it, a group a frame:
 *   vpss_thread_set_val     the frame starts: each channel that takes it
 *                           gets a buffer, and the store, scaler, crop,
 *                           cover and line registers are written
 *   vpss_thread_post_deal   the frame is done: the buffers go to what each
 *                           channel is bound to (VENC: vb.ko's list 2) or
 *                           to the channel's queue for user space
 *   vpss_thread_middle_deal for the vendor's line mode only (line=0 here)
 *
 * Splicing puts up to three groups' channels into one picture: a channel
 * in splice mode 1/2 is the top/bottom half, 3/4 the left/right half, 5/6/7
 * a third from the top; the channels of one splice share a buffer, which
 * goes on when every part is stored, as the first part's channel's.
 *
 * ioctls (a 128-byte record: group, channel, then the call's words 4..):
 *   0x5000      init             0x5010      every group destroyed
 *   0x40805001  create a group   0x40805002  its attributes (kept)
 *   0x40045008  start {grp}      0x40045009  stop
 *   0x40805004  a channel's attributes: pixel format, w, h, source and
 *               output frame rates, mirror, flip
 *   0x40805006  enable a channel / 0x40805007 disable
 *   0x4080500a  bind: module (7 VENC), dev, chn
 *   0x4080500b  the group's crop (kept)    0x4080500c  a channel's crop
 *   0x4080500d  the depth of the channel's queue
 *   0x4080500e  a picture from that queue, waiting word 2 ms: words 4..14
 *               luma, chroma, 3rd plane, format, stride, w, h, -, pts,
 *               buffer
 *   0x4080500f  that picture back
 *   0x40805012  splice: mode, id (0..8)
 *   0x420c5011  debug: register writes after each frame's
 * vpss_set_chn_win and vpss_set_grp_line take rgn.ko's covers and lines;
 * vi_set_chn_win and vi_set_chn_line are empty, as in vpss.ko.
 *
 * Registers (0x34000000): +0xB000 the input window, +0x80000 + 0x40n the
 * scalers (as the XM533's VI), +0x8D000/+0x8E000 the lines (15 + 15), and
 * a 0x1000 block per channel: +0x81000 crop, +0x84100 covers, +0x90000 the
 * store (addresses in 16-byte units from DDR, strides, sizes, enable at
 * +0x34).
 */

#include <linux/fs.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/list.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/sched.h>
#include <linux/seq_file.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>
#include <linux/wait.h>

#define VPSS_BASE 0x34000000
#define VPSS_SIZE 0x100000
#define DDR_BASE 0x80000000u
#define N_GRP 4
#define N_CHN 3
#define N_WIN 16
#define N_LINE 30
#define N_SPLICE 9

#define VPSS_INIT 0x5000
#define VPSS_DESTROY_ALL 0x5010
#define VPSS_CREATE_GRP 0x40805001
#define VPSS_SET_GRP_ATTR 0x40805002
#define VPSS_SET_CHN_ATTR 0x40805004
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
#define MOD_VO 0x11
#define FMT_SP420 2

/* vb.ko's buffer descriptors (72 bytes, the XM535 layout) */
#define WORD(b, n) (((u32 *)(b))[n])
#define W_PHYS 4
#define W_STRIDE 6
#define W_WIDTH 7
#define W_HEIGHT 8
#define W_OWN 9
#define W_PTS 12
#define W_BIND 14 /* module, dev, chn */

#define VB_FREE 0
int vb_get_buf(u32 list, u32 size, void **buf, int ms);
int vb_put_buf(u32 list, void *buf);
int vb_add_buf_reference(void *buf);
void clk_get_pts(u64 *pts);

static int line;
module_param(line, int, 0444);
MODULE_PARM_DESC(line, "the vendor's line mode: 0 only");

/* the registers a channel's store takes for a frame */
struct chn_regs {
	u32 store, scale;
	u32 v[7], h[7]; /* num, den, int, frac, recip, flag, predown */
	u32 crop[4];    /* x0, x1, y0, y1 */
	u32 stride[6], mirror, fmt;
	u32 addr[3], last[3], size[3];
	u32 win_en, win_fill, win[N_WIN][4]; /* x0 y0, x1 y1, colour, thick */
};

struct chn {
	u32 enabled;
	u32 fmt, w, h, src_rate, dst_rate, mirror, flip;
	u32 crop[5]; /* enable, x, y, w, h */
	u32 win[N_WIN][8]; /* rgn: enable, x, y, w, h, colour, fill, thick */
	u32 bind[3];
	u32 depth, count;
	struct list_head q;
	wait_queue_head_t wq;
	u32 splice, splice_id;
	u32 acc;
	void *cur; /* this frame's buffer */
	/* /proc */
	u64 rate_pts;
	u32 rate_cnt, rate, sent, vb_ok, vb_fail;
};

struct grp {
	u32 created, started, busy;
	u32 attr[28], crop[5];
	u32 recv, timeouts, src, src_w, src_h;
	u64 pts, time;
	u32 line[N_LINE][8]; /* rgn: enable, x0, y0, mode, len, step, thick,
				colour */
	struct chn chn[N_CHN];
	struct chn_regs regs[N_CHN];
	u32 in_w, in_h;
	u32 dbg_on, dbg_n, dbg_addr[64], dbg_val[64];
};

/* a spliced picture's buffer and the parts stored in it */
struct splice {
	void *node;
	u32 state;
	u64 pts, interval;
	struct chn *owner;
};

static struct grp grps[N_GRP];
static struct splice splices[N_SPLICE];
static DEFINE_SPINLOCK(vpss_lock);
static void __iomem *regs;

#define N_OUT 32
static void *out[N_OUT];

static inline void reg_w(u32 off, u32 v)
{
	writel(v, regs + off);
}

static u32 list_of(void *b)
{
	u32 m = WORD(b, W_BIND);

	return m == MOD_VENC ? 2 : m == MOD_VO ? 3 : VB_FREE;
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

/*
 * The scaler's ratio from in to out pixels: a predown stage (out x1, 2 or
 * 4) and the fraction (out' - 1) / (in - 1), reduced and scaled up to at
 * least 1024 in the denominator.
 */
static int ratio(u32 in, u32 out, u32 r[7])
{
	u32 num, den, a, b, t, k;

	memset(r, 0, 7 * sizeof(u32));
	if (out == in) {
		r[0] = r[1] = 0x400;
		r[2] = 1;
		r[4] = 0x100;
		r[5] = 1;
		return 0;
	}
	if (out < 2 || in < 2 || in >= out * 16)
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

/* this frame's buffer for channel c: its own or its splice's */
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
		/* this part again: the last picture was never completed */
		vb_put_buf(VB_FREE, s->node);
		s->node = NULL;
		s->state = 0;
	}
	if (!s->node) {
		if (vb_get_buf(VB_FREE, size * (m >= 5 ? 3 : 2), &b, 0))
			return NULL;
		s->node = b;
		s->state = 0;
	}
	b = s->node;
	WORD(b, W_STRIDE) = m == 3 || m == 4 ? stride * 2 : stride;
	WORD(b, W_WIDTH) = m == 3 || m == 4 ? w * 2 : w;
	WORD(b, W_HEIGHT) = m == 1 || m == 2 ? h * 2 : m >= 5 ? h * 3 : h;
	return b;
}

/*
 * The store's addresses for the part of b splice mode m takes: luma,
 * chroma and their last lines, from a plane of y bytes (chroma uv).
 */
static void store_addr(struct chn_regs *r, void *b, u32 m, u32 w, u32 stride,
		       u32 y, u32 uv, u32 third)
{
	u32 o = WORD(b, W_PHYS) - DDR_BASE;

	switch (m) {
	case 0:
		r->addr[0] = o >> 4;
		r->addr[1] = (o + y) >> 4;
		r->addr[2] = (o + y + uv) >> 4;
		r->last[0] = (o - w + y) >> 4;
		r->last[1] = (o - w + y + uv) >> 4;
		r->last[2] = (o - w + y + uv + third) >> 4;
		if (r->fmt == FMT_SP420) {
			r->size[0] = y >> 4;
			r->size[1] = uv >> 4;
		} else {
			r->size[0] = y;
			r->size[1] = uv;
			r->size[2] = third;
		}
		return;
	case 1: /* top of two */
	case 5: /* top of three */
		r->addr[0] = o >> 4;
		r->addr[1] = (o + (m == 1 ? 2 : 3) * y) >> 4;
		r->last[0] = (o - w + y) >> 4;
		r->last[1] = (o - w + uv + (m == 1 ? 2 : 3) * y) >> 4;
		break;
	case 2: /* bottom of two */
		r->addr[0] = (o + y) >> 4;
		r->addr[1] = (o + uv + 2 * y) >> 4;
		r->last[0] = (o - w + 2 * y) >> 4;
		r->last[1] = (o - w + 2 * y + 2 * uv) >> 4;
		break;
	case 3: /* left of two */
	case 4: /* right of two */
		r->stride[0] = r->stride[2] = stride * 2 >> 4;
		if (m == 3) {
			r->addr[0] = o >> 4;
			r->addr[1] = (o + 2 * y) >> 4;
			r->last[0] = (o - 2 * stride + 2 * y) >> 4;
			r->last[1] = (o - 2 * stride + 2 * y + 2 * uv) >> 4;
		} else {
			r->addr[0] = (o + stride) >> 4;
			r->addr[1] = (o + stride + 2 * y) >> 4;
			r->last[0] = (o - stride + 2 * y) >> 4;
			r->last[1] = (o - stride + 2 * y + 2 * uv) >> 4;
		}
		break;
	case 6: /* middle of three */
		r->addr[0] = (o + y) >> 4;
		r->addr[1] = (o + uv + 3 * y) >> 4;
		r->last[0] = (o - w + 2 * y) >> 4;
		r->last[1] = (o - w + 3 * y + 2 * uv) >> 4;
		break;
	case 7: /* bottom of three */
		r->addr[0] = (o + 2 * y) >> 4;
		r->addr[1] = (o + 2 * uv + 3 * y) >> 4;
		r->last[0] = (o - w + 3 * y) >> 4;
		r->last[1] = (o - w + 3 * y + 3 * uv) >> 4;
		break;
	}
	r->size[0] = y;
	r->size[1] = uv;
}

/* channel n of g for this frame; with vpss_lock held */
static void chn_frame(struct grp *g, int n, u32 pts_lo, u32 pts_hi)
{
	struct chn *c = &g->chn[n];
	struct chn_regs *r = &g->regs[n];
	u32 w = c->crop[0] ? c->crop[3] : c->w;
	u32 h = c->crop[0] ? c->crop[4] : c->h;
	u32 stride = ALIGN(w, 16), y = stride * h, uv = y, third = y;
	bool secondary = c->splice == 2 || c->splice == 4 || c->splice >= 6;
	int i;
	void *b;

	c->cur = NULL;
	if (c->fmt == FMT_SP420) {
		third = 0;
		uv = y / 2;
	}
	if (!c->enabled || !(secondary || takes_frame(c)))
		return;
	b = take_buffer(c, y + uv + third, w, h);
	if (!b) {
		c->vb_fail++;
		return;
	}
	if (ratio(g->in_w, c->w, r->h) || ratio(g->in_h, c->h, r->v)) {
		pr_err("vpss too small %x -> %x\n", g->in_w, c->w);
		if (!c->splice)
			vb_put_buf(VB_FREE, b);
		return;
	}
	c->cur = b;
	c->vb_ok++;
	WORD(b, W_PTS) = pts_lo;
	WORD(b, W_PTS + 1) = pts_hi;
	WORD(b, W_OWN) = 0;
	r->fmt = c->fmt;
	r->scale = c->w < g->in_w || c->h < g->in_h;
	if (c->crop[0]) {
		r->crop[0] = c->crop[1];
		r->crop[1] = c->crop[1] + w;
		r->crop[2] = c->crop[2];
		r->crop[3] = c->crop[2] + h;
	} else {
		r->crop[1] = w;
		r->crop[3] = h;
	}
	for (i = 0; i < N_WIN; i++) {
		const u32 *a = c->win[i];

		if (!a[0])
			continue;
		r->win_en |= 1u << i;
		r->win_fill |= a[6] << i;
		r->win[i][0] = a[1] | a[2] << 16;
		r->win[i][1] = (a[1] + a[3]) | (a[2] + a[4]) << 16;
		r->win[i][2] = a[5];
		r->win[i][3] = a[7];
	}
	for (i = 0; i < 6; i++)
		r->stride[i] = (w + 15) >> 4;
	r->mirror = c->mirror << 2 | c->flip << 1 | c->mirror;
	store_addr(r, b, c->splice, w, stride, y, uv, third);
	r->store = 1;
}

static void update_regs(struct grp *g)
{
	u32 n = 0, i, k;

	reg_w(0x12004, 0);
	reg_w(0xb020, 1);
	reg_w(0xb000, 0);
	reg_w(0xb004, g->in_w);
	reg_w(0xb008, 0);
	reg_w(0xb00c, g->in_h);
	/* the lines: 15 in the first bank, the rest in the second */
	for (i = 0; i < N_LINE; i++)
		n += g->line[i][0] != 0;
	reg_w(0x8d008, 0);
	reg_w(0x8e008, 0);
	reg_w(0x8d004, min(n, 15u));
	reg_w(0x8d000, n != 0);
	reg_w(0x8e004, n > 15 ? n - 15 : 0);
	reg_w(0x8e000, n > 15);
	for (i = 0, k = 0; i < N_LINE; i++) {
		const u32 *l = g->line[i];
		u32 o = 0x8d010 + k * 0x30 + (k > 14 ? 0xd30 : 0);

		if (!l[0])
			continue;
		reg_w(o + 0x00, l[3]);
		reg_w(o + 0x04, l[4]);
		reg_w(o + 0x08, l[5]);
		reg_w(o + 0x0c, l[6]);
		reg_w(o + 0x10, (l[7] & 0xffffff) >> 16);
		reg_w(o + 0x14, (l[7] & 0xffff) >> 8);
		reg_w(o + 0x18, l[7] & 0xff);
		reg_w(o + 0x1c, l[1]);
		reg_w(o + 0x20, l[2]);
		k++;
	}
	reg_w(0x8d008, 1);
	reg_w(0x8e008, 1);
	for (i = 0; i < N_CHN; i++) {
		const struct chn_regs *r = &g->regs[i];
		u32 c = i * 0x1000, s = 0x80000 + i * 0x40;

		reg_w(c + 0x90034, r->store);
		reg_w(c + 0x81020, 1);
		if (!r->store) {
			reg_w(c + 0x81000, 0);
			reg_w(c + 0x81004, ~0u);
			reg_w(c + 0x81008, 0);
			reg_w(c + 0x8100c, ~0u);
			continue;
		}
		for (k = 0; k < 6; k++) {
			reg_w(s + 0x04 + 4 * k, r->v[k]);
			reg_w(s + 0x1c + 4 * k, r->h[k]);
		}
		reg_w(s + 0x00, r->scale);
		reg_w(s + 0x34, r->h[6]);
		reg_w(s + 0x3c, r->v[6]);
		for (k = 0; k < 4; k++)
			reg_w(c + 0x81000 + 4 * k, r->crop[k]);
		for (k = 0; k < 4; k++)
			reg_w(c + 0x90020 + 4 * k, r->stride[k]);
		reg_w(c + 0x90030, r->mirror);
		reg_w(c + 0x90038, 0);
		reg_w(c + 0x90054, 3);
		reg_w(c + 0x90000, r->addr[0]);
		reg_w(c + 0x90010, r->addr[1]);
		reg_w(c + 0x90080, r->last[0]);
		reg_w(c + 0x90090, r->last[1]);
		reg_w(c + 0x90058, r->size[0]);
		reg_w(c + 0x9005c, r->size[1]);
		if (i == 2) {
			/* channel 2's other formats: the third plane */
			reg_w(c + 0x900e0, r->stride[4]);
			reg_w(c + 0x900e4, r->stride[5]);
			reg_w(c + 0x900c0, r->addr[2]);
			reg_w(c + 0x900d0, r->last[2]);
			reg_w(c + 0x900e8, r->size[2]);
			reg_w(c + 0x90078, r->fmt);
		}
		for (k = 0; k < N_WIN; k++)
			if (r->win_en & 1u << k) {
				u32 o = c + 0x84140 + k * 0x10;

				reg_w(o, r->win[k][0]);
				reg_w(o + 4, r->win[k][1]);
				reg_w(o + 8, r->win[k][2]);
				reg_w(o + 12, r->win[k][3]);
			}
		reg_w(c + 0x84100, r->win_en);
		reg_w(c + 0x84104, r->win_fill);
	}
}

/* isp.ko: group grp's frame starts, w x h from source src */
int vpss_thread_set_val(u32 grp, u32 src, u32 w, u32 h, u32 pts_lo,
			u32 pts_hi)
{
	unsigned long flags;
	struct grp *g;
	int i;

	if (grp >= N_GRP)
		return -1;
	g = &grps[grp];
	spin_lock_irqsave(&vpss_lock, flags);
	g->src = src;
	g->src_w = w;
	g->src_h = h;
	g->recv++;
	if (!g->created || !g->started) {
		spin_unlock_irqrestore(&vpss_lock, flags);
		return -1;
	}
	if (g->busy) {
		spin_unlock_irqrestore(&vpss_lock, flags);
		pr_err("vpss: grp %u busy\n", grp);
		return -1;
	}
	g->busy = 1;
	/* the input is at least as large as every channel */
	for (i = 0; i < N_CHN; i++) {
		w = max(w, g->chn[i].w);
		h = max(h, g->chn[i].h);
	}
	g->in_w = w;
	g->in_h = h;
	memset(g->regs, 0, sizeof(g->regs));
	for (i = 0; i < N_CHN; i++)
		chn_frame(g, i, pts_lo, pts_hi);
	update_regs(g);
	if (g->dbg_on)
		for (i = 0; i < g->dbg_n && i < 64; i++)
			writel(g->dbg_val[i], regs + (g->dbg_addr[i] & (VPSS_SIZE - 4)));
	clk_get_pts(&g->pts);
	spin_unlock_irqrestore(&vpss_lock, flags);
	return 0;
}
EXPORT_SYMBOL(vpss_thread_set_val);

static void deliver(struct chn *c, void *b, u64 now)
{
	if (!c->depth) {
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

/* isp.ko: group grp's frame is stored (ok) or was not */
int vpss_thread_post_deal(u32 grp, int ok)
{
	unsigned long flags;
	struct grp *g;
	u64 now;
	int i;

	if (grp >= N_GRP)
		return -1;
	g = &grps[grp];
	spin_lock_irqsave(&vpss_lock, flags);
	if (g->busy != 1) {
		spin_unlock_irqrestore(&vpss_lock, flags);
		return -1;
	}
	if (!ok) {
		g->timeouts++;
		for (i = 0; i < N_CHN; i++)
			if (g->chn[i].cur && !g->chn[i].splice)
				vb_put_buf(VB_FREE, g->chn[i].cur);
		goto done;
	}
	clk_get_pts(&now);
	g->time = now - g->pts;
	for (i = 0; i < N_CHN; i++) {
		struct chn *c = &g->chn[i], *to = c;
		void *b = c->cur;
		struct splice *s;
		u32 m = c->splice;

		if (!b)
			continue;
		memcpy(&WORD(b, W_BIND), c->bind, sizeof(c->bind));
		if (m) {
			s = &splices[c->splice_id];
			s->state |= 1u << m;
			if (m == 1 || m == 3 || m == 5)
				s->owner = c;
			if (s->state != 0x6 && s->state != 0x18 && s->state != 0xe0) {
				s->pts = g->pts;
				continue;
			}
			/* every part stored: on as the first part's channel's */
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
done:
	for (i = 0; i < N_CHN; i++)
		g->chn[i].cur = NULL;
	g->busy = 0;
	spin_unlock_irqrestore(&vpss_lock, flags);
	return 0;
}
EXPORT_SYMBOL(vpss_thread_post_deal);

/* isp.ko: the vendor's line mode (line=N); nothing with line=0 */
int vpss_thread_middle_deal(u32 grp, int ok)
{
	return 0;
}
EXPORT_SYMBOL(vpss_thread_middle_deal);

/* rgn.ko: cover idx of a channel */
int vpss_set_chn_win(u32 grp, u32 n, u32 idx, u32 *w)
{
	unsigned long flags;

	if (grp >= N_GRP || !grps[grp].created || n >= N_CHN || idx >= N_WIN) {
		pr_err("vpss: grp %u not created\n", grp);
		return -1;
	}
	spin_lock_irqsave(&vpss_lock, flags);
	memcpy(grps[grp].chn[n].win[idx], w, 8 * sizeof(u32));
	spin_unlock_irqrestore(&vpss_lock, flags);
	return 0;
}
EXPORT_SYMBOL(vpss_set_chn_win);

/* rgn.ko: line idx of a group */
int vpss_set_grp_line(u32 grp, u32 idx, u32 *l)
{
	unsigned long flags;

	if (grp >= N_GRP || !grps[grp].created || idx >= N_LINE) {
		pr_err("vpss: grp %u not created\n", grp);
		return -1;
	}
	spin_lock_irqsave(&vpss_lock, flags);
	memcpy(grps[grp].line[idx], l, 8 * sizeof(u32));
	spin_unlock_irqrestore(&vpss_lock, flags);
	return 0;
}
EXPORT_SYMBOL(vpss_set_grp_line);

/* as vpss.ko: VI has no covers or lines on the XM535 */
int vi_set_chn_win(u32 dev, u32 n, u32 idx, u32 *w)
{
	return 0;
}
EXPORT_SYMBOL(vi_set_chn_win);

int vi_set_chn_line(u32 dev, u32 n, u32 idx, u32 *l)
{
	return 0;
}
EXPORT_SYMBOL(vi_set_chn_line);

static void vpss_init(void)
{
	unsigned long flags;
	int g, c;

	spin_lock_irqsave(&vpss_lock, flags);
	memset(grps, 0, sizeof(grps));
	memset(splices, 0, sizeof(splices));
	memset(out, 0, sizeof(out));
	for (g = 0; g < N_GRP; g++)
		for (c = 0; c < N_CHN; c++) {
			INIT_LIST_HEAD(&grps[g].chn[c].q);
			init_waitqueue_head(&grps[g].chn[c].wq);
		}
	spin_unlock_irqrestore(&vpss_lock, flags);
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
	unsigned long flags;
	struct list_head *n;
	long t;
	void *b;

	t = wait_event_interruptible_timeout(c->wq, c->count,
					     msecs_to_jiffies(w[2]));
	spin_lock_irqsave(&vpss_lock, flags);
	if (!c->count) {
		spin_unlock_irqrestore(&vpss_lock, flags);
		/* as vpss.ko: -ERESTARTSYS when the wait ran out */
		return t < 0 ? t : t ? -1 : -ERESTARTSYS;
	}
	n = c->q.next;
	list_del(n);
	c->count--;
	b = n;
	if (out_take(b)) {
		put_on(b);
		spin_unlock_irqrestore(&vpss_lock, flags);
		return -EBUSY;
	}
	if (c->bind[0]) {
		/* also to the bound module; user space holds a reference */
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
	spin_unlock_irqrestore(&vpss_lock, flags);
	return 0;
}

static long vpss_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	void __user *uarg = (void __user *)arg;
	unsigned long flags;
	struct grp *g;
	struct chn *c;
	u32 w[32], dbg[131];
	long ret = 0;

	switch (cmd) {
	case VPSS_INIT:
		vpss_init();
		return 0;
	case VPSS_DESTROY_ALL:
		spin_lock_irqsave(&vpss_lock, flags);
		for (ret = 0; ret < N_GRP; ret++)
			grps[ret].created = grps[ret].started = 0;
		spin_unlock_irqrestore(&vpss_lock, flags);
		return 0;
	case VPSS_START_GRP:
	case VPSS_STOP_GRP:
		if (get_user(w[0], (u32 __user *)uarg))
			return -EFAULT;
		if (w[0] >= N_GRP || !grps[w[0]].created)
			return -EINVAL;
		grps[w[0]].started = cmd == VPSS_START_GRP;
		return 0;
	case VPSS_DEBUG:
		if (copy_from_user(dbg, uarg, sizeof(dbg)))
			return -EFAULT;
		if (dbg[0] >= N_GRP)
			return -EINVAL;
		spin_lock_irqsave(&vpss_lock, flags);
		g = &grps[dbg[0]];
		g->dbg_on = dbg[1];
		g->dbg_n = min(dbg[2], 64u);
		memcpy(g->dbg_addr, dbg + 3, sizeof(g->dbg_addr));
		memcpy(g->dbg_val, dbg + 67, sizeof(g->dbg_val));
		spin_unlock_irqrestore(&vpss_lock, flags);
		return 0;
	}
	if (_IOC_TYPE(cmd) != 'P' || _IOC_SIZE(cmd) != sizeof(w))
		return -ENOTTY;
	if (copy_from_user(w, uarg, sizeof(w)))
		return -EFAULT;
	if (cmd == VPSS_RELEASE_FRAME) {
		void *b = (void *)w[14];

		spin_lock_irqsave(&vpss_lock, flags);
		if (out_give(b))
			vb_put_buf(VB_FREE, b);
		else
			ret = -EINVAL;
		spin_unlock_irqrestore(&vpss_lock, flags);
		return ret;
	}
	if (w[0] >= N_GRP || w[1] >= N_CHN)
		return -EINVAL;
	g = &grps[w[0]];
	c = &g->chn[w[1]];
	if (cmd == VPSS_CREATE_GRP) {
		if (g->created) {
			pr_err("vpss: grp %u exists\n", w[0]);
			return -EEXIST;
		}
		memcpy(g->attr, w + 4, sizeof(g->attr));
		g->created = 1;
		return 0;
	}
	if (!g->created) {
		pr_err("vpss: grp %u not created\n", w[0]);
		return -EINVAL;
	}
	spin_lock_irqsave(&vpss_lock, flags);
	switch (cmd) {
	case VPSS_SET_GRP_ATTR:
		memcpy(g->attr, w + 4, sizeof(g->attr));
		break;
	case VPSS_SET_CHN_ATTR:
		/* channels 0 and 1 store SP420 only */
		if ((w[1] < 2 && w[4] != FMT_SP420) || !w[5] || !w[6]) {
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
		if (w[7] + w[5] > c->w || w[8] + w[6] > c->h) {
			pr_err("vpss: crop out of range\n");
			ret = -EINVAL;
			break;
		}
		memcpy(c->crop, w + 4, sizeof(c->crop));
		break;
	case VPSS_SPLICE:
		if (w[5] >= N_SPLICE || w[4] > 7) {
			ret = -EINVAL;
			break;
		}
		c->splice = w[4];
		c->splice_id = w[5];
		if (!w[4]) {
			splices[w[5]].node = NULL;
			splices[w[5]].state = 0;
		}
		break;
	case VPSS_GET_FRAME:
		spin_unlock_irqrestore(&vpss_lock, flags);
		ret = get_frame(c, w);
		if (copy_to_user(uarg, w, sizeof(w)))
			return -EFAULT;
		return ret;
	default:
		ret = -ENOTTY;
	}
	spin_unlock_irqrestore(&vpss_lock, flags);
	return ret;
}

static int vpss_show(struct seq_file *s, void *v)
{
	int i, j;

	seq_puts(s, "[VPSS] openxiongmai xm535_vpss\n\n");
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
				   i, g->started ? "Y" : "N", g->recv, g->src, g->src_w,
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
	.unlocked_ioctl = vpss_ioctl,
};

static struct miscdevice vpss_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "vpss",
	.fops = &vpss_fops,
};

static int __init xm535_vpss_init(void)
{
	int ret;

	if (line)
		pr_warn("vpss: line=%d not supported, as line=0\n", line);
	regs = ioremap(VPSS_BASE, VPSS_SIZE);
	if (!regs) {
		pr_err("vpss: ioremap failed\n");
		return -ENOMEM;
	}
	vpss_init();
	if (!proc_create("umap/vpss", 0444, NULL, &vpss_proc_fops))
		pr_err("vpss: create vpss proc fail!\n");
	ret = misc_register(&vpss_misc);
	if (ret) {
		pr_err("vpss: register device failed!\n");
		remove_proc_entry("umap/vpss", NULL);
		iounmap(regs);
		return ret;
	}
	pr_info("vpss: openxiongmai xm535_vpss init ok\n");
	return 0;
}

static void __exit xm535_vpss_exit(void)
{
	misc_deregister(&vpss_misc);
	remove_proc_entry("umap/vpss", NULL);
	iounmap(regs);
}

module_init(xm535_vpss_init);
module_exit(xm535_vpss_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("XM535D1/D2 VPSS, compatible with vpss.ko");
