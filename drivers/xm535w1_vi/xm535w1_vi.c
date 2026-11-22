// SPDX-License-Identifier: GPL-2.0
/*
 * The XM535W1's video input (/dev/vi): a drop-in for the vendor vi.ko.
 *
 * The W1 has no VPSS module: its VI drives the XM535's VPSS block itself
 * (0x34000000), as the XM533's VI does its own, from each channel's frame
 * start and end interrupts (IRQ 57, 58, 67, 68; status +0x8705C, +0x8704C
 * in the channel's 0x1000 block). At the start a channel that takes the
 * frame gets a free vb.ko buffer programmed as its next one; at the end
 * the buffer filled goes on to what the channel is bound to (VENC: vb.ko's
 * list 2) or to the channel's queue for user space.
 *
 *   0x5600      init: the VI reset, every channel cleared
 *   0x40505601  a device's attributes (kept)
 *   0x40505602  a channel's attributes: words 4..14 x, y, w, h of the
 *               capture, pixel format (2 SP420), output w, h, mirror,
 *               flip, source and output frame rates
 *   0x40045603  enable {chn} / 0x40045604 disable
 *   0x40505605  bind: words 4..6 module (7 VENC), dev, chn
 *   0x40505606  the channel's queue depth for user space (word 4)
 *   0x40505607  a picture from the queue, waiting word 2 ms: words 4..14
 *               luma, chroma, 3rd plane, format, stride, w, h, -, pts,
 *               buffer
 *   0x40505608  that picture back
 *   0x4a18560e  OSD background windows for a channel: the luminance of
 *               up to 256 rectangles, two measured a frame
 *   0x4a18560f  them back, with their luminance
 *
 * rgn.ko draws covers (vi_set_chn_win) and channel 0's lines
 * (vi_set_chn_line) through it; the vpss_* hooks isp.ko and rgn.ko link
 * against are empty, as in vi.ko.
 *
 * Registers (a block of 0x1000 per channel): +0xB000 channel 0's capture
 * window, +0x80000 + 0x40n the scaler, +0x81000 crop, +0x84100 covers,
 * +0x84240 the background measurement, +0x87040 the frame start/end lines
 * and status, +0x8D000/+0x8E000 the lines (15 + 15), +0x90000 the store.
 */

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
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

#define VI_BASE 0x34000000
#define VI_SIZE 0x100000
#define VI_RESET 0x2000011c /* the system controller's VI reset */
#define N_CHN 3
#define N_LINE 30
#define N_BG 256
#define DDR_BASE 0x80000000u

static const int vi_irqs[] = {57, 58, 67, 68};

#define VI_INIT 0x5600
#define VI_SET_DEV_ATTR 0x40505601
#define VI_SET_CHN_ATTR 0x40505602
#define VI_ENABLE_CHN 0x40045603
#define VI_DISABLE_CHN 0x40045604
#define VI_BIND 0x40505605
#define VI_SET_DEPTH 0x40505606
#define VI_GET_FRAME 0x40505607
#define VI_RELEASE_FRAME 0x40505608
#define VI_SET_BG 0x4a18560e
#define VI_GET_BG 0x4a18560f

#define MOD_VENC 7
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
#define W_VALID 17

#define VB_FREE 0
#define VB_VENC 2
int vb_get_buf(u32 list, u32 size, void **buf, int ms);
int vb_put_buf(u32 list, void *buf);
void clk_get_pts(u64 *pts);

static int rst = 1;
module_param(rst, int, 0444);
MODULE_PARM_DESC(rst, "reset VI on init (1)");
static int line;
module_param(line, int, 0444);
MODULE_PARM_DESC(line, "the vendor's line mode: 0 only");

/* words 4..14 of VI_SET_CHN_ATTR */
struct chn_attr {
	u32 x, y, w, h; /* the capture */
	u32 fmt;	/* 2 SP420; others three planes */
	u32 out_w, out_h;
	u32 mirror, flip;
	u32 src_rate, dst_rate;
};

/*
 * VI_SET_BG's 0xa18 bytes: the channel, enable, the number of windows,
 * the window measured now and the two being measured, then x, y, w, h
 * and the luminance measured, 256 16-bit words each.
 */
struct bg {
	u32 chn, en, n, cur, a, b;
	u16 x[N_BG], y[N_BG], w[N_BG], h[N_BG], lum[N_BG];
};

struct chn {
	struct chn_attr a;
	u32 bind[3];
	u32 size;
	u32 enabled;
	u32 acc;
	void *now, *next;
	struct list_head q;
	u32 count, depth;
	wait_queue_head_t wq;
	/* /proc */
	u64 last_pts;
	u32 frame_time, vb_ok, vb_fail, vb_dup, ints, sent;
	u32 last_addr, now_addr, next_addr;
};

static struct chn chns[N_CHN];
static struct bg bgs[N_CHN];
static u32 bg_chn; /* VI_GET_BG's: the last set */
static u32 lines[N_LINE][8]; /* channel 0's: enable, x0, y0, mode, len,
				step, thick, colour */
static u32 dev_attr[N_CHN][16];
static DEFINE_SPINLOCK(vi_lock);
static atomic_t opens = ATOMIC_INIT(0);
static void __iomem *regs;

#define N_OUT 32
static void *out[N_OUT];

static inline void reg_w(u32 off, u32 v)
{
	writel(v, regs + off);
}

static inline u32 reg_r(u32 off)
{
	return readl(regs + off);
}

#define CH(n) ((n) * 0x1000)

static void put_on(void *b)
{
	vb_put_buf(WORD(b, W_BIND) == MOD_VENC ? VB_VENC : VB_FREE, b);
}

static void drop(void *b)
{
	memset(&WORD(b, W_BIND), 0, 12);
	vb_put_buf(VB_FREE, b);
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

static void deliver(struct chn *c, void *b, u64 pts)
{
	WORD(b, W_PTS) = (u32)pts;
	WORD(b, W_PTS + 1) = pts >> 32;
	if (!c->depth) {
		put_on(b);
	} else {
		list_add_tail((struct list_head *)b, &c->q);
		wake_up(&c->wq);
		if (++c->count > c->depth) {
			struct list_head *n = c->q.next;

			list_del(n);
			c->count--;
			put_on(n);
		}
	}
	c->sent++;
}

static void frame_end(int i, u64 pts)
{
	struct chn *c = &chns[i];
	void *last = c->now;

	c->now = c->next;
	c->next = NULL;
	if (last) {
		if (WORD(last, W_VALID) == 1)
			deliver(c, last, pts);
		else
			drop(last);
	}
	reg_w(CH(i) + 0x90034, 0);
	if (c->now)
		WORD(c->now, W_VALID) =
			reg_r(CH(i) + 0x90100) != reg_r(CH(i) + 0x90108);
	c->last_addr = last ? WORD(last, W_PHYS) : 0;
	c->now_addr = c->now ? WORD(c->now, W_PHYS) : 0;
	c->frame_time = pts - c->last_pts;
	c->last_pts = pts;
}

static bool takes_frame(struct chn *c)
{
	u32 s = c->a.src_rate, d = c->a.dst_rate;

	if (!s || !c->enabled || !atomic_read(&opens))
		return false;
	if (!d || d >= s)
		return true;
	c->acc += d;
	if (c->acc < s)
		return false;
	c->acc -= s;
	return true;
}

static void program(int i, struct chn *c, void *b)
{
	u32 w = c->a.out_w, h = c->a.out_h, stride = ALIGN(w, 32);
	u32 y = stride * h, uv = y, third = y, k;
	u32 o = WORD(b, W_PHYS) - DDR_BASE, r = CH(i) + 0x90000;

	memcpy(&WORD(b, W_BIND), c->bind, sizeof(c->bind));
	WORD(b, W_WIDTH) = w;
	WORD(b, W_HEIGHT) = h;
	WORD(b, W_STRIDE) = stride;
	WORD(b, W_OWN) = 0;
	if (c->a.fmt == FMT_SP420) {
		uv = y / 2;
		third = 0;
	}
	reg_w(r + 0x20, stride >> 4);
	reg_w(r + 0x24, stride >> 4);
	reg_w(r + 0x28, stride >> 4);
	reg_w(r + 0x2c, stride >> 4);
	reg_w(r + 0xe0, stride >> 4);
	reg_w(r + 0xe4, stride >> 4);
	reg_w(r + 0x00, o >> 4);
	reg_w(r + 0x10, (o + y) >> 4);
	reg_w(r + 0xc0, (o + y + uv) >> 4);
	/* the last lines, for mirror and flip */
	reg_w(r + 0x80, (o - stride + y) >> 4);
	reg_w(r + 0x90, (o - stride + y + uv) >> 4);
	reg_w(r + 0xd0, (o - stride + third + y + uv) >> 4);
	if (!rst && i != 2 && c->sent < 32)
		/* without the reset, every address slot of the store */
		for (k = 0; k < 32; k++) {
			reg_w(r + (k < 4 ? 0 : 0x100) + 4 * k, o >> 4);
			reg_w(r + (k < 4 ? 0x10 : 0x170) + 4 * k, (o + y) >> 4);
		}
	reg_w(r + 0x58, y >> 4);
	reg_w(r + 0x5c, uv >> 4);
	reg_w(r + 0x34, 1);
}

/* the background luminance: two windows measured a frame */
static void bg_measure(int i)
{
	struct bg *g = &bgs[i];
	u32 r = CH(i) + 0x84240, k;
	u32 *slot[2] = {&g->a, &g->b};

	if (!g->en || !g->n || g->n > N_BG) {
		reg_w(r, 0);
		return;
	}
	reg_w(r, 1);
	for (k = 0; k < 2; k++) {
		u32 busy = r + 0x2c + 4 * k, x;

		if (reg_r(busy))
			continue;
		x = *slot[k];
		if (x < N_BG && g->w[x] && g->h[x]) {
			g->lum[x] = reg_r(r + 0x24 + 4 * k) / (g->w[x] * g->h[x]);
			g->cur = (g->cur + 1) % g->n;
			*slot[k] = g->cur;
		}
		x = g->cur;
		reg_w(r + 0x04 + 0x10 * k, g->x[x]);
		reg_w(r + 0x08 + 0x10 * k, g->y[x]);
		reg_w(r + 0x0c + 0x10 * k, g->x[x] + g->w[x]);
		reg_w(r + 0x10 + 0x10 * k, g->y[x] + g->h[x]);
		reg_w(busy, 1);
	}
}

static void frame_start(int i)
{
	struct chn *c = &chns[i];

	c->ints++;
	if (takes_frame(c)) {
		if (c->next) {
			drop(c->next);
			c->next = NULL;
			c->vb_dup++;
		}
		if (vb_get_buf(VB_FREE, c->size, &c->next, 0) == 0)
			c->vb_ok++;
		else
			c->vb_fail++;
	}
	if (c->next)
		program(i, c, c->next);
	else
		reg_w(CH(i) + 0x90034, 0);
	c->next_addr = c->next ? WORD(c->next, W_PHYS) : 0;
	bg_measure(i);
}

static irqreturn_t vi_int(int irq, void *dev)
{
	irqreturn_t ret = IRQ_NONE;
	unsigned long flags;
	u64 pts;
	int i;

	clk_get_pts(&pts);
	spin_lock_irqsave(&vi_lock, flags);
	for (i = 0; i < N_CHN; i++) {
		if (reg_r(CH(i) + 0x8704c)) {
			reg_w(CH(i) + 0x8704c, 0);
			frame_end(i, pts);
			ret = IRQ_HANDLED;
		}
		if (reg_r(CH(i) + 0x8705c)) {
			reg_w(CH(i) + 0x8705c, 0);
			frame_start(i);
			ret = IRQ_HANDLED;
		}
	}
	spin_unlock_irqrestore(&vi_lock, flags);
	return ret;
}

/* the scaler's ratio for in to out pixels (out < in), as the XM533's VI */
static int ratio(u32 in, u32 out, u32 r[7])
{
	u32 num, den, a, b, t, k, pre;

	if (out >= in) {
		u32 one[7] = {0x400, 0x400, 1, 0, 0x100, 1, 0};

		memcpy(r, one, sizeof(one));
		return 0;
	}
	if (out < 2 || in >= out * 16)
		return -1;
	if (in >= out * 8)
		pre = 3, out *= 4;
	else if (in >= out * 4)
		pre = 1, out *= 2;
	else
		pre = 0;
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
	r[6] = pre;
	return 0;
}

static int set_chn_attr(u32 n, const struct chn_attr *a)
{
	struct chn *c = &chns[n];
	u32 size = ALIGN(a->out_w, 32) * a->out_h * 3, h[7], v[7], s, k;
	unsigned long flags;

	if (a->fmt == FMT_SP420)
		size /= 2;
	if (ratio(a->w, a->out_w, h) || ratio(a->h, a->out_h, v)) {
		pr_err("vi: chn %u: %ux%u to %ux%u is more than 16x down\n", n,
		       a->w, a->h, a->out_w, a->out_h);
		return -EINVAL;
	}
	spin_lock_irqsave(&vi_lock, flags);
	if (c->size && c->size < size) {
		spin_unlock_irqrestore(&vi_lock, flags);
		pr_err("vi: chn %u: buffers of %#x, %#x wanted\n", n, c->size, size);
		return -EINVAL;
	}
	if (!c->size)
		c->size = size;
	c->a = *a;
	if (n == 0) {
		reg_w(0xb000, a->x);
		reg_w(0xb008, a->y);
		reg_w(0xb004, a->x + a->w);
		reg_w(0xb00c, a->y + a->h);
	}
	s = CH(n) + 0x90000;
	reg_w(s + 0x34, 0);
	reg_w(s + 0x54, 3);
	reg_w(s + 0x38, 0);
	for (k = 0; k < 4; k++)
		reg_w(s + 0x20 + 4 * k, a->out_w >> 4);
	reg_w(s + 0xe0, a->out_w >> 4);
	reg_w(s + 0xe4, a->out_w >> 4);
	reg_w(s + 0x30, a->mirror << 2 | a->flip << 1 | a->mirror);
	reg_w(s + 0x78, a->fmt);
	reg_w(s + 0x40, 0xff);
	reg_w(s + 0x44, 0xffffffff);
	/* the frame start and end lines */
	reg_w(CH(n) + 0x87044, 3);
	reg_w(CH(n) + 0x87040, 1);
	reg_w(CH(n) + 0x87054, a->out_h * 15 / 16);
	reg_w(CH(n) + 0x87050, 1);
	s = 0x80000 + n * 0x40;
	reg_w(s + 0x00, a->w > a->out_w || a->h > a->out_h);
	for (k = 0; k < 6; k++) {
		reg_w(s + 0x04 + 4 * k, v[k]);
		reg_w(s + 0x1c + 4 * k, h[k]);
	}
	reg_w(s + 0x34, h[6]);
	reg_w(s + 0x3c, v[6]);
	reg_w(CH(n) + 0x81000, 0);
	reg_w(CH(n) + 0x81004, a->out_w);
	reg_w(CH(n) + 0x81008, 0);
	reg_w(CH(n) + 0x8100c, a->out_h);
	spin_unlock_irqrestore(&vi_lock, flags);
	return 0;
}

/* rgn.ko: cover idx of channel n, as on the XM533 */
int vi_set_chn_win(u32 dev, u32 n, u32 idx, u32 *w)
{
	unsigned long flags;
	u32 bit, v, x, y, r;

	if (n >= N_CHN || idx >= 16)
		return -1;
	bit = 1u << idx;
	spin_lock_irqsave(&vi_lock, flags);
	if (w[0] == 1) {
		v = reg_r(CH(n) + 0x84104);
		reg_w(CH(n) + 0x84104, w[6] == 1 ? v | bit : v & ~bit);
		x = w[1] ? w[1] : 1;
		y = w[2] ? w[2] : 1;
		if (w[1] + w[3] >= chns[n].a.out_w)
			w[3] = chns[n].a.out_w - 1 - w[1];
		if (w[2] + w[4] >= chns[n].a.out_h)
			w[4] = chns[n].a.out_h - 1 - w[2];
		r = CH(n) + 0x84140 + idx * 0x10;
		reg_w(r, x | y << 16);
		reg_w(r + 4, (w[1] + w[3]) | (w[2] + w[4]) << 16);
		reg_w(r + 8, w[5]);
		reg_w(r + 0xc, w[7]);
	}
	v = reg_r(CH(n) + 0x84100);
	reg_w(CH(n) + 0x84100, w[0] == 1 ? v | bit : v & ~bit);
	spin_unlock_irqrestore(&vi_lock, flags);
	return 0;
}
EXPORT_SYMBOL(vi_set_chn_win);

/* rgn.ko: line idx of channel 0, all of them written again */
int vi_set_chn_line(u32 dev, u32 n, u32 idx, u32 *l)
{
	unsigned long flags;
	u32 i, k = 0;

	if (dev || n || idx >= N_LINE)
		return -1;
	spin_lock_irqsave(&vi_lock, flags);
	memcpy(lines[idx], l, sizeof(lines[idx]));
	reg_w(0x8d008, 0);
	reg_w(0x8e008, 0);
	for (i = 0; i < N_LINE; i++) {
		const u32 *a = lines[i];
		u32 o = 0x8d010 + k * 0x30 + (k > 14 ? 0xd30 : 0);

		if (!a[0])
			continue;
		reg_w(o + 0x00, a[3]);
		reg_w(o + 0x1c, a[1]);
		reg_w(o + 0x20, a[2]);
		reg_w(o + 0x04, a[4]);
		reg_w(o + 0x08, a[5]);
		reg_w(o + 0x0c, a[6]);
		reg_w(o + 0x10, (a[7] >> 16) & 0xff);
		reg_w(o + 0x14, (a[7] >> 8) & 0xff);
		reg_w(o + 0x18, a[7] & 0xff);
		k++;
	}
	reg_w(0x8d004, min(k, 15u));
	reg_w(0x8d000, k != 0);
	reg_w(0x8e004, k > 15 ? k - 15 : 0);
	reg_w(0x8e000, k > 15);
	reg_w(0x8d008, 1);
	reg_w(0x8e008, 1);
	spin_unlock_irqrestore(&vi_lock, flags);
	return 0;
}
EXPORT_SYMBOL(vi_set_chn_line);

/* as vi.ko: isp.ko's and rgn.ko's VPSS hooks, with no VPSS; with
 * open_vpss beside it (WITH_VPSS=1, the XM535D1/D2's online BPU) those are
 * its */
#ifndef WITH_VPSS
int vpss_thread_set_val(u32 grp, u32 src, u32 w, u32 h, u32 lo, u32 hi)
{
	return 0;
}
EXPORT_SYMBOL(vpss_thread_set_val);

int vpss_thread_post_deal(u32 grp, int ok)
{
	return 0;
}
EXPORT_SYMBOL(vpss_thread_post_deal);

int vpss_thread_middle_deal(u32 grp, int ok)
{
	return 0;
}
EXPORT_SYMBOL(vpss_thread_middle_deal);

int vpss_set_chn_win(u32 grp, u32 n, u32 idx, u32 *w)
{
	return 0;
}
EXPORT_SYMBOL(vpss_set_chn_win);

int vpss_set_grp_line(u32 grp, u32 idx, u32 *l)
{
	return 0;
}
EXPORT_SYMBOL(vpss_set_grp_line);
#endif

static void vi_init(void)
{
	unsigned long flags;
	void __iomem *r;
	int i;

	if (rst == 1) {
		r = ioremap(VI_RESET, 4);
		if (r) {
			writel(0x1590000, r);
			udelay(100);
			writel(1, r);
			udelay(100);
			iounmap(r);
		}
		reg_w(0x12000, 0);
		udelay(100);
		reg_w(0x12000, 3);
		udelay(100);
	}
	spin_lock_irqsave(&vi_lock, flags);
	for (i = 0; i < N_CHN; i++) {
		struct chn *c = &chns[i];

		queue_flush(c);
		if (c->now)
			drop(c->now);
		if (c->next)
			drop(c->next);
		memset(c, 0, offsetof(struct chn, q));
		INIT_LIST_HEAD(&c->q);
		c->count = c->depth = 0;
		c->last_pts = 0;
		c->frame_time = c->vb_ok = c->vb_fail = c->vb_dup = 0;
		c->ints = c->sent = 0;
		c->last_addr = c->now_addr = c->next_addr = 0;
	}
	memset(bgs, 0, sizeof(bgs));
	memset(lines, 0, sizeof(lines));
	memset(dev_attr, 0, sizeof(dev_attr));
	spin_unlock_irqrestore(&vi_lock, flags);
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

static long get_frame(u32 *w)
{
	struct chn *c = &chns[w[1]];
	unsigned long flags;
	struct list_head *n;
	long t;
	void *b;

	t = wait_event_interruptible_timeout(c->wq, c->count,
					     msecs_to_jiffies(w[2]));
	spin_lock_irqsave(&vi_lock, flags);
	if (!c->count) {
		spin_unlock_irqrestore(&vi_lock, flags);
		return t < 0 ? t : t ? -1 : -ERESTARTSYS;
	}
	n = c->q.next;
	list_del(n);
	c->count--;
	b = n;
	if (out_take(b)) {
		put_on(b);
		spin_unlock_irqrestore(&vi_lock, flags);
		return -EBUSY;
	}
	w[4] = WORD(b, W_PHYS);
	w[5] = w[4] + WORD(b, W_STRIDE) * WORD(b, W_HEIGHT);
	w[6] = w[4] + WORD(b, W_STRIDE) * WORD(b, W_HEIGHT) * 2;
	w[7] = c->a.fmt;
	w[8] = WORD(b, W_STRIDE);
	w[9] = WORD(b, W_WIDTH);
	w[10] = WORD(b, W_HEIGHT);
	w[12] = WORD(b, W_PTS);
	w[13] = WORD(b, W_PTS + 1);
	w[14] = (u32)b;
	spin_unlock_irqrestore(&vi_lock, flags);
	return 0;
}

static long bg_ioctl(unsigned int cmd, void __user *uarg)
{
	static struct bg tmp;
	unsigned long flags;

	if (cmd == VI_SET_BG) {
		if (copy_from_user(&tmp, uarg, sizeof(tmp)))
			return -EFAULT;
		if (tmp.chn >= N_CHN || tmp.n > N_BG)
			return -EINVAL;
		spin_lock_irqsave(&vi_lock, flags);
		bg_chn = tmp.chn;
		bgs[tmp.chn] = tmp;
		spin_unlock_irqrestore(&vi_lock, flags);
		return 0;
	}
	spin_lock_irqsave(&vi_lock, flags);
	tmp = bgs[bg_chn];
	spin_unlock_irqrestore(&vi_lock, flags);
	return copy_to_user(uarg, &tmp, sizeof(tmp)) ? -EFAULT : 0;
}

static DEFINE_MUTEX(bg_mutex);

static long vi_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	void __user *uarg = (void __user *)arg;
	unsigned long flags;
	u32 w[20], n;
	long ret = 0;

	if (cmd == VI_INIT) {
		vi_init();
		return 0;
	}
	if (cmd == VI_SET_BG || cmd == VI_GET_BG) {
		mutex_lock(&bg_mutex);
		ret = bg_ioctl(cmd, uarg);
		mutex_unlock(&bg_mutex);
		return ret;
	}
	if (cmd == VI_ENABLE_CHN || cmd == VI_DISABLE_CHN) {
		if (get_user(n, (u32 __user *)uarg))
			return -EFAULT;
		if (n >= N_CHN)
			return -EINVAL;
		spin_lock_irqsave(&vi_lock, flags);
		chns[n].enabled = cmd == VI_ENABLE_CHN;
		spin_unlock_irqrestore(&vi_lock, flags);
		return 0;
	}
	if (_IOC_TYPE(cmd) != 'V' || _IOC_SIZE(cmd) != sizeof(w))
		return -ENOTTY;
	if (copy_from_user(w, uarg, sizeof(w)))
		return -EFAULT;
	n = w[1];
	if (cmd == VI_SET_DEV_ATTR) {
		if (w[0] >= N_CHN)
			return -EINVAL;
		memcpy(dev_attr[w[0]], w + 4, sizeof(dev_attr[0]));
		return 0;
	}
	if (cmd == VI_RELEASE_FRAME) {
		void *b = (void *)w[14];

		spin_lock_irqsave(&vi_lock, flags);
		if (out_give(b))
			vb_put_buf(VB_FREE, b);
		else
			ret = -EINVAL;
		spin_unlock_irqrestore(&vi_lock, flags);
		return ret;
	}
	if (n >= N_CHN)
		return -EINVAL;
	switch (cmd) {
	case VI_SET_CHN_ATTR:
		return set_chn_attr(n, (struct chn_attr *)(w + 4));
	case VI_BIND:
	case VI_SET_DEPTH:
		spin_lock_irqsave(&vi_lock, flags);
		queue_flush(&chns[n]);
		if (cmd == VI_BIND) {
			chns[n].depth = 0;
			memcpy(chns[n].bind, w + 4, sizeof(chns[n].bind));
		} else {
			chns[n].depth = w[4];
		}
		spin_unlock_irqrestore(&vi_lock, flags);
		return 0;
	case VI_GET_FRAME:
		ret = get_frame(w);
		break;
	default:
		return -ENOTTY;
	}
	if (copy_to_user(uarg, w, sizeof(w)))
		return -EFAULT;
	return ret;
}

static int vi_open(struct inode *inode, struct file *file)
{
	atomic_inc(&opens);
	return 0;
}

static int vi_close(struct inode *inode, struct file *file)
{
	atomic_dec(&opens);
	return 0;
}

static int vi_show(struct seq_file *s, void *v)
{
	int i, k;

	seq_puts(s, "[VI] openxiongmai xm535w1_vi\n\n");
	seq_puts(s, "--------------------------VI PHYCHN ATTR------------------------------------------------------\n");
	seq_puts(s, "PhyChn  CapX  CapY  CapW  CapH  DstW  DstH  Mirror  Flip  IntEn  StEn  Depth  PixFom  SrcRat  DstRat\n");
	for (i = 0; i < N_CHN; i++) {
		const struct chn *c = &chns[i];

		seq_printf(s, "%-6d  %-4u  %-4u  %-4u  %-4u  %-4u  %-4u  %-6s  %-4s  %-5s  %-4s  %-5u  %-6u  %-6u  %-6u\n",
			   i, c->a.x, c->a.y, c->a.w, c->a.h, c->a.out_w, c->a.out_h,
			   c->a.mirror ? "Y" : "N", c->a.flip ? "Y" : "N", "Y",
			   c->enabled ? "Y" : "N", c->depth, c->a.fmt, c->a.src_rate,
			   c->a.dst_rate);
	}
	seq_puts(s, "--------------------------VI CHN STATUS-------------------------------------------------------\n");
	seq_puts(s, "ViChn  FrmTime  FrmRate  VBSucc    VBFail    VBDup     IntCnt    SendCnt   LastAddr  NowAddr   NextAddr\n");
	for (i = 0; i < N_CHN; i++) {
		const struct chn *c = &chns[i];
		u32 t = c->frame_time;

		seq_printf(s, "%-5d  %-7u  %-7u  %-8u  %-8u  %-8u  %-8u  %-8u  %-8x  %-8x  %-8x\n",
			   i, t, t ? (t / 2 + 1000000) / t : 0, c->vb_ok, c->vb_fail,
			   c->vb_dup, c->ints, c->sent, c->last_addr, c->now_addr,
			   c->next_addr);
	}
	seq_puts(s, "--------------------------VI OSD BACKGROUND----------------------------------------------------\n");
	seq_puts(s, "ViChn  En  Id    X      Y      W      H      Lum\n");
	for (i = 0; i < N_CHN; i++)
		for (k = 0; k < bgs[i].n && k < N_BG; k++)
			seq_printf(s, "%-5d  %-2s  %-3d  %-4u  %-4u  %-4u  %-4u  %-3x\n", i,
				   bgs[i].en ? "Y" : "N", k, bgs[i].x[k], bgs[i].y[k],
				   bgs[i].w[k], bgs[i].h[k], bgs[i].lum[k]);
	return 0;
}

static int vi_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, vi_show, NULL);
}

static const struct file_operations vi_proc_fops = {
	.owner = THIS_MODULE,
	.open = vi_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static const struct file_operations vi_fops = {
	.owner = THIS_MODULE,
	.open = vi_open,
	.release = vi_close,
	.unlocked_ioctl = vi_ioctl,
};

static struct miscdevice vi_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "vi",
	.fops = &vi_fops,
};

static void free_irqs(int n)
{
	while (n--)
		free_irq(vi_irqs[n], NULL);
}

static int __init xm535w1_vi_init(void)
{
	int i, ret;

	BUILD_BUG_ON(sizeof(struct bg) != 0xa18);
	if (line)
		pr_warn("vi: line=%d not supported, as line=0\n", line);
	regs = ioremap(VI_BASE, VI_SIZE);
	if (!regs) {
		pr_err("vi: ioremap failed\n");
		return -ENOMEM;
	}
	for (i = 0; i < N_CHN; i++) {
		INIT_LIST_HEAD(&chns[i].q);
		init_waitqueue_head(&chns[i].wq);
	}
	vi_init();
	for (i = 0; i < ARRAY_SIZE(vi_irqs); i++) {
		ret = request_irq(vi_irqs[i], vi_int, 0, "vi_yuv_store", NULL);
		if (ret) {
			pr_err("vi: request_irq failed\n");
			free_irqs(i);
			iounmap(regs);
			return ret;
		}
	}
	ret = misc_register(&vi_misc);
	if (ret) {
		pr_err("vi: register device failed!\n");
		free_irqs(ARRAY_SIZE(vi_irqs));
		iounmap(regs);
		return ret;
	}
	if (!proc_create("umap/vi", 0444, NULL, &vi_proc_fops))
		pr_err("vi: create vi proc fail!\n");
	pr_info("vi: openxiongmai xm535w1_vi init ok\n");
	return 0;
}

static void __exit xm535w1_vi_exit(void)
{
	unsigned long flags;
	int i;

	remove_proc_entry("umap/vi", NULL);
	misc_deregister(&vi_misc);
	free_irqs(ARRAY_SIZE(vi_irqs));
	spin_lock_irqsave(&vi_lock, flags);
	for (i = 0; i < N_CHN; i++) {
		reg_w(CH(i) + 0x90034, 0);
		queue_flush(&chns[i]);
		if (chns[i].now)
			drop(chns[i].now);
		if (chns[i].next)
			drop(chns[i].next);
		chns[i].now = chns[i].next = NULL;
	}
	spin_unlock_irqrestore(&vi_lock, flags);
	iounmap(regs);
}

module_init(xm535w1_vi_init);
module_exit(xm535w1_vi_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("XM535W1 video input, compatible with vi.ko");
