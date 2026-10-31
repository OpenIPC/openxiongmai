// SPDX-License-Identifier: GPL-2.0
/*
 * The XM533's video input (/dev/vi): a drop-in for the vendor vi.ko.
 *
 * The XM533 has no VPSS. Its VI stores each of three channels to memory
 * itself, scaled down from the ISP's picture, into vb.ko's buffers: at the
 * start of a frame (IRQ 59/60, status +0x6705C) a channel that takes this
 * frame gets a free buffer programmed as its next one; at the end of the
 * frame (+0x6704C) the buffer filled goes on to what the channel is bound
 * to (VENC: vb.ko's list 2) or to the channel's own queue for user space.
 *
 *   0x5600      init: the VI reset, every channel cleared
 *   0x40505601  a device's attributes (kept)
 *   0x40505602  a channel's attributes: words 4..14 x, y, w, h of the
 *               capture, pixel format, output w, h, mirror, flip, source
 *               and output frame rates
 *   0x40045603  enable {chn} / 0x40045604 disable
 *   0x40505605  bind: words 4..6 module (7 VENC), dev, chn
 *   0x40505606  the channel's queue depth for user space (word 4)
 *   0x40505607  a picture from the queue, waiting word 2 ms: words 4..14
 *               luma, chroma, 0, format, stride, w, h, -, pts, buffer
 *   0x40505608  that picture back
 *   0x40505609  crop: words 4..8 enable, x, y, w, h
 *   0x4050560e  the last picture sent: its address, size and format
 *
 * Channel 0's crop and scaling-down flag are the ISP's (isp.ko's
 * vpss_isp_set_attr); channels 1 and 2 have their own. vi_set_chn_win
 * draws rgn.ko's covers.
 *
 * Registers (0x30000000, a block of 0x1000 per channel):
 *   +0x60000 + 0x40n  the scaler: enable, vertical num, den, int, frac,
 *                     recip, flag, horizontal ditto, predown h, -, v
 *   +0x61000          channel 1, 2: crop x0, x1, y0, y1
 *   +0x64100          covers: enable bits, fill bits, then 0x10 a cover:
 *                     x0 | y0 << 16, x1 | y1 << 16, colour, thickness
 *   +0x67000          interrupt enables; channel 0 also frame start and
 *                     end lines and the status words
 *   +0x70000          the store: luma, chroma (16-byte units from DDR),
 *                     strides, enable (+0x34), mirror/flip, format, ...
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

#define VI_BASE 0x30000000
#define VI_SIZE 0x100000
#define VI_RESET 0x2000011c /* the system controller's VI reset */
#define VI_IRQ0 59
#define VI_IRQ1 60
#define N_CHN 3
#define DDR_BASE 0x80000000u

#define VI_INIT 0x5600
#define VI_SET_DEV_ATTR 0x40505601
#define VI_SET_CHN_ATTR 0x40505602
#define VI_ENABLE_CHN 0x40045603
#define VI_DISABLE_CHN 0x40045604
#define VI_BIND 0x40505605
#define VI_SET_DEPTH 0x40505606
#define VI_GET_FRAME 0x40505607
#define VI_RELEASE_FRAME 0x40505608
#define VI_SET_CROP 0x40505609
#define VI_QUERY 0x4050560e

#define MOD_VENC 7

/* vb.ko's buffer descriptors (64 bytes, the XM530V200/XM533 layout) */
#define WORD(b, n) (((u32 *)(b))[n])
#define W_PHYS 4
#define W_STRIDE 6
#define W_WIDTH 7
#define W_HEIGHT 8
#define W_PTS 10
#define W_BIND 12 /* module, dev, chn */
#define W_VALID 15

#define VB_FREE 0
#define VB_VENC 2
int vb_get_buf(u32 list, u32 size, void **buf, int ms);
int vb_put_buf(u32 list, void *buf);
int vb_add_buf_reference(void *buf);
/* isp.ko: channel 0's crop and size */
int vpss_isp_set_attr(u32 grp, u32 w, u32 h, u32 *attr);
void clk_get_pts(u64 *pts);

static int rst = 1;
module_param(rst, int, 0444);
MODULE_PARM_DESC(rst, "reset VI on init (1)");

/* words 4..14 of VI_SET_CHN_ATTR */
struct chn_attr {
	u32 x, y, w, h; /* the capture */
	u32 fmt;	/* 0 YUV420SP; 3 2 bytes a pixel; others 4 */
	u32 out_w, out_h;
	u32 mirror, flip;
	u32 src_rate, dst_rate;
};

struct chn {
	struct chn_attr a;
	u32 crop[5]; /* enable, x, y, w, h */
	u32 bind[3];
	u32 size;    /* the buffers, from the first attributes */
	u32 enabled;
	u32 acc;     /* frame rate: dst_rate of every src_rate frames */
	void *now, *next;
	/* the queue for user space */
	struct list_head q;
	u32 count, depth;
	wait_queue_head_t wq;
	/* /proc */
	u64 last_pts;
	u32 frame_time, vb_ok, vb_fail, vb_dup, ints, sent;
	u32 last_addr, now_addr, next_addr, sent_addr;
};

static struct chn chns[N_CHN];
static u32 dev_attr[N_CHN][16];
static DEFINE_SPINLOCK(vi_lock);
static atomic_t opens = ATOMIC_INIT(0);
static void __iomem *regs;

/* the pictures user space holds */
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

/* with vi_lock held */
static void queue_flush(struct chn *c)
{
	while (c->count) {
		struct list_head *n = c->q.next;

		list_del(n);
		c->count--;
		put_on(n);
	}
}

/* a picture filled: on to the bound module, or to the queue */
static void deliver(struct chn *c, void *b, u64 pts)
{
	c->sent_addr = WORD(b, W_PHYS);
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

/* a frame ended: what was stored is done, the next buffer stores now */
static void frame_end(u64 pts)
{
	int i;

	for (i = 0; i < N_CHN; i++) {
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
		reg_w(CH(i) + 0x70034, 0);
		if (c->now)
			WORD(c->now, W_VALID) =
				reg_r(CH(i) + 0x70100) != reg_r(CH(i) + 0x70108);
		c->last_addr = last ? WORD(last, W_PHYS) : 0;
		c->now_addr = c->now ? WORD(c->now, W_PHYS) : 0;
		c->frame_time = pts - c->last_pts;
		c->last_pts = pts;
	}
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

/* the buffer for the frame after this one */
static void program(int i, struct chn *c, void *b)
{
	u32 w = c->crop[0] ? c->crop[3] : c->a.out_w;
	u32 h = c->crop[0] ? c->crop[4] : c->a.out_h;
	u32 stride = ALIGN(w, 32), size = stride * h, uv = 0, uv32 = 0;
	u32 off = WORD(b, W_PHYS) - DDR_BASE;

	memcpy(&WORD(b, W_BIND), c->bind, sizeof(c->bind));
	WORD(b, W_WIDTH) = w;
	WORD(b, W_HEIGHT) = h;
	WORD(b, W_STRIDE) = stride;
	if (c->a.fmt == 0) {
		uv = size / 2;
		uv32 = size / 32;
	} else if (c->a.fmt == 3) {
		size *= 2;
		stride *= 2;
	} else {
		size *= 4;
		stride *= 4;
	}
	reg_w(CH(i) + 0x70020, stride >> 4);
	reg_w(CH(i) + 0x70024, stride >> 4);
	reg_w(CH(i) + 0x70028, stride >> 4);
	reg_w(CH(i) + 0x7002c, stride >> 4);
	reg_w(CH(i) + 0x70000, off >> 4);
	reg_w(CH(i) + 0x70010, (off + size) >> 4);
	/* the last lines, for mirror and flip */
	reg_w(CH(i) + 0x70080, (off + size - stride) >> 4);
	reg_w(CH(i) + 0x70090, (off + uv + size - stride) >> 4);
	reg_w(CH(i) + 0x70058, size >> 4);
	reg_w(CH(i) + 0x7005c, uv32);
	reg_w(CH(i) + 0x70034, 1);
}

static void frame_start(void)
{
	int i;

	for (i = 0; i < N_CHN; i++) {
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
			reg_w(CH(i) + 0x70034, 0);
		c->next_addr = c->next ? WORD(c->next, W_PHYS) : 0;
	}
}

static irqreturn_t vi_int(int irq, void *dev)
{
	irqreturn_t ret = IRQ_NONE;
	unsigned long flags;
	u64 pts;

	clk_get_pts(&pts);
	spin_lock_irqsave(&vi_lock, flags);
	if (reg_r(0x6704c)) {
		reg_w(0x6704c, 0);
		frame_end(pts);
		ret = IRQ_HANDLED;
	}
	if (reg_r(0x6705c)) {
		reg_w(0x6705c, 0);
		frame_start();
		ret = IRQ_HANDLED;
	}
	spin_unlock_irqrestore(&vi_lock, flags);
	return ret;
}

/*
 * The scaler's ratio for in to out pixels (out < in): the predown stage
 * (out x1, 2 or 4) and the fraction (out' - 1) / (in - 1), reduced and
 * scaled up to at least 1024 in the denominator.
 */
static int ratio(u32 in, u32 out, u32 r[7])
{
	u32 num, den, a, b, t, k, pre;

	if (out >= in) {
		/* num den int frac recip flag predown */
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
	u32 size = ALIGN(a->out_w, 32) * a->out_h * 3, h[7], v[7], s;
	unsigned long flags;

	if (!a->fmt)
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
	memset(c->crop, 0, sizeof(c->crop));
	c->a = *a;
	reg_w(CH(n) + 0x70034, 0);
	reg_w(CH(n) + 0x70054, 3);
	reg_w(CH(n) + 0x70038, 0);
	reg_w(CH(n) + 0x70020, a->out_w >> 4);
	reg_w(CH(n) + 0x70024, a->out_w >> 4);
	reg_w(CH(n) + 0x70028, a->out_w >> 4);
	reg_w(CH(n) + 0x7002c, a->out_w >> 4);
	reg_w(CH(n) + 0x70030, a->mirror << 2 | a->flip << 1 | a->mirror);
	reg_w(CH(n) + 0x70074, a->fmt);
	reg_w(CH(n) + 0x67000, 0xff);
	reg_w(CH(n) + 0x67004, 0xffffffff);
	if (n == 0) {
		/* the frame start and end lines */
		reg_w(0x67044, 0x10);
		reg_w(0x67054, a->out_h * 15 / 16);
		reg_w(0x67040, 1);
		reg_w(0x67050, 1);
	}
	s = 0x60000 + n * 0x40;
	reg_w(s + 0x00, a->w > a->out_w || a->h > a->out_h);
	reg_w(s + 0x04, v[0]);
	reg_w(s + 0x08, v[1]);
	reg_w(s + 0x0c, v[2]);
	reg_w(s + 0x10, v[3]);
	reg_w(s + 0x14, v[4]);
	reg_w(s + 0x18, v[5]);
	reg_w(s + 0x1c, h[0]);
	reg_w(s + 0x20, h[1]);
	reg_w(s + 0x24, h[2]);
	reg_w(s + 0x28, h[3]);
	reg_w(s + 0x2c, h[4]);
	reg_w(s + 0x30, h[5]);
	reg_w(s + 0x34, h[6]);
	reg_w(s + 0x3c, v[6]);
	if (n) {
		reg_w(CH(n) + 0x61000, 0);
		reg_w(CH(n) + 0x61004, a->out_w);
		reg_w(CH(n) + 0x61008, 0);
		reg_w(CH(n) + 0x6100c, a->out_h);
	}
	spin_unlock_irqrestore(&vi_lock, flags);
	if (n == 0) {
		u32 isp[19] = {0};
		u32 down = a->w > a->out_w || a->h > a->out_h;

		isp[3] = down;
		isp[5] = down;
		isp[6] = a->out_w;
		isp[7] = a->out_h;
		vpss_isp_set_attr(0, a->out_w, a->out_h, isp);
	}
	return 0;
}

static int set_crop(u32 n, const u32 *crop)
{
	struct chn *c = &chns[n];
	u32 x = 0, y = 0, w = c->a.out_w, h = c->a.out_h;
	unsigned long flags;

	/* without the sums wrapping */
	if (crop[1] > c->a.out_w || crop[3] > c->a.out_w - crop[1] ||
	    crop[2] > c->a.out_h || crop[4] > c->a.out_h - crop[2]) {
		pr_err("vi: crop out of range\n");
		return -EINVAL;
	}
	spin_lock_irqsave(&vi_lock, flags);
	memcpy(c->crop, crop, sizeof(c->crop));
	reg_w(CH(n) + 0x70034, 0);
	if (crop[0]) {
		x = crop[1];
		y = crop[2];
		w = crop[3];
		h = crop[4];
	}
	if (n) {
		reg_w(CH(n) + 0x61000, x);
		reg_w(CH(n) + 0x61004, x + w);
		reg_w(CH(n) + 0x61008, y);
		reg_w(CH(n) + 0x6100c, y + h);
	}
	reg_w(CH(n) + 0x67054, h * 15 / 16);
	spin_unlock_irqrestore(&vi_lock, flags);
	if (n == 0) {
		u32 isp[19] = {0};

		isp[4] = x;
		isp[5] = y;
		isp[6] = w;
		isp[7] = h;
		vpss_isp_set_attr(0, w, h, isp);
	}
	return 0;
}

/*
 * rgn.ko: cover idx of channel n. w {enable, x, y, w, h, colour, fill,
 * thickness}; the rectangle is cut to the channel's picture.
 */
int vi_set_chn_win(u32 dev, u32 n, u32 idx, u32 *w)
{
	unsigned long flags;
	u32 bit, v, x, y, cw, ch, r;

	if (n >= N_CHN || idx >= 16)
		return -1;
	bit = 1u << idx;
	spin_lock_irqsave(&vi_lock, flags);
	if (w[0] == 1) {
		cw = chns[n].a.out_w;
		ch = chns[n].a.out_h;
		v = reg_r(CH(n) + 0x64104);
		reg_w(CH(n) + 0x64104, w[6] == 1 ? v | bit : v & ~bit);
		x = w[1] ? w[1] : 1;
		y = w[2] ? w[2] : 1;
		if (w[1] + w[3] >= cw)
			w[3] = cw - 1 - w[1];
		if (w[2] + w[4] >= ch)
			w[4] = ch - 1 - w[2];
		r = CH(n) + 0x64140 + idx * 0x10;
		reg_w(r, x | y << 16);
		reg_w(r + 4, (w[1] + w[3]) | (w[2] + w[4]) << 16);
		reg_w(r + 8, w[5]);
		reg_w(r + 0xc, w[7]);
	}
	v = reg_r(CH(n) + 0x64100);
	reg_w(CH(n) + 0x64100, w[0] == 1 ? v | bit : v & ~bit);
	spin_unlock_irqrestore(&vi_lock, flags);
	return 0;
}
EXPORT_SYMBOL(vi_set_chn_win);

/* a picture of a record whose word 10 is its buffer, back to the pool */
int vi_release_frame(u32 n, u32 *rec)
{
	return vb_put_buf(VB_FREE, (void *)rec[10]);
}
EXPORT_SYMBOL(vi_release_frame);

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
		c->last_addr = c->now_addr = c->next_addr = c->sent_addr = 0;
	}
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
		/* as vi.ko: -ERESTARTSYS when the wait ran out */
		return t < 0 ? t : t ? -1 : -ERESTARTSYS;
	}
	n = c->q.next;
	list_del(n);
	c->count--;
	b = n;
	if (out_take(b)) {
		/* user space holds too many: this one goes on */
		put_on(b);
		spin_unlock_irqrestore(&vi_lock, flags);
		return -EBUSY;
	}
	if (c->bind[0]) {
		/* also to the bound module; user space holds a reference */
		vb_add_buf_reference(b);
		put_on(b);
	}
	w[4] = WORD(b, W_PHYS);
	w[5] = WORD(b, W_PHYS) + WORD(b, W_STRIDE) * WORD(b, W_HEIGHT);
	w[6] = WORD(b, W_PHYS) + WORD(b, W_STRIDE) * WORD(b, W_HEIGHT) * 2;
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
	case VI_SET_CROP:
		return set_crop(n, w + 4);
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
	case VI_QUERY: {
		struct chn *c = &chns[n];
		u32 m = c->bind[0];

		spin_lock_irqsave(&vi_lock, flags);
		w[7] = c->a.fmt;
		w[9] = c->a.out_w;
		w[10] = c->a.out_h;
		w[8] = m == 0xf || m == 0x11 ? ALIGN(w[9], 16) : ALIGN(w[9], 32);
		w[4] = c->sent_addr;
		w[5] = w[4] + w[8] * w[10];
		spin_unlock_irqrestore(&vi_lock, flags);
		break;
	}
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
	int i;

	seq_puts(s, "[VI] openxiongmai xm533_vi\n\n");
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

static int __init xm533_vi_init(void)
{
	int i, ret;

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
	ret = request_irq(VI_IRQ0, vi_int, 0, "vi_yuv_store", NULL);
	if (!ret) {
		ret = request_irq(VI_IRQ1, vi_int, 0, "vi_yuv_store", NULL);
		if (ret)
			free_irq(VI_IRQ0, NULL);
	}
	if (ret) {
		pr_err("vi: request_irq failed\n");
		iounmap(regs);
		return ret;
	}
	ret = misc_register(&vi_misc);
	if (ret) {
		pr_err("vi: register device failed!\n");
		free_irq(VI_IRQ0, NULL);
		free_irq(VI_IRQ1, NULL);
		iounmap(regs);
		return ret;
	}
	if (!proc_create("umap/vi", 0444, NULL, &vi_proc_fops))
		pr_err("vi: create vi proc fail!\n");
	pr_info("vi: openxiongmai xm533_vi init ok\n");
	return 0;
}

static void __exit xm533_vi_exit(void)
{
	unsigned long flags;
	int i;

	remove_proc_entry("umap/vi", NULL);
	misc_deregister(&vi_misc);
	free_irq(VI_IRQ0, NULL);
	free_irq(VI_IRQ1, NULL);
	spin_lock_irqsave(&vi_lock, flags);
	for (i = 0; i < N_CHN; i++) {
		reg_w(CH(i) + 0x70034, 0);
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

module_init(xm533_vi_init);
module_exit(xm533_vi_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("XM533 video input, compatible with vi.ko");
