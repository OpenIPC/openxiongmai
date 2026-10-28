// SPDX-License-Identifier: GPL-2.0
/*
 * xm510_vi: video input (capture into memory) for the XM510, a drop-in
 * replacement for the vendor's xm510_vi.ko with the same /dev/vi ioctl ABI.
 *
 * VI takes the picture inside a capture window and writes it into memory on
 * three channels: channel 0 cropped to its size, channel 1 scaled from
 * channel 0, channel 2 at its own size. Each channel has a store engine
 * (registers at 0x40081000 + chn * 0x1000) that fills a ring of 2-4 buffers
 * in MMZ. One interrupt marks the start of a frame (IRQ 16), another its end
 * (IRQ 18), for all channels at once.
 *
 * Software keeps a state per buffer: free, ready (filled, not yet taken) or
 * held (taken, not yet released). At the end of a frame the buffer the store
 * engine finished becomes ready, and if the next one in the ring is not free
 * the engine is held (+0x4c), so it never writes over a frame somebody has
 * not read; it is let go at a later frame end once that buffer is free. This
 * is the vendor driver's protocol, kept as it is. What differs:
 *  - frames a descriptor still holds are released when it is closed;
 *  - poll(), and GET_FRAME on an O_NONBLOCK descriptor returns -EAGAIN;
 *  - VI init stops the store engines and lets their DMA drain before the
 *    reset, as the XM530 driver must;
 *  - a channel's buffers are laid out again whenever its size changes;
 *  - channel numbers are checked.
 */

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/poll.h>
#include <linux/proc_fs.h>
#include <linux/sched.h>
#include <linux/seq_file.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/time.h>
#include <linux/uaccess.h>
#include <linux/wait.h>

/* mmz.ko: mmz_alloc in every version; mmz_get_addr in open_mmz only, so it
 * is looked up when needed rather than linked against */
extern int mmz_alloc(const char *name, u32 size, u32 *phys);
extern int mmz_get_addr(const char *name, u32 *size, u32 *phys);

#define VI_REG_BASE 0x40000000
#define VI_REG_SIZE 0x100000
/* system controller: VI reset */
#define VI_RESET_REG 0x2000007c
#define VI_RESET_VAL 0x01590000

#define VI_CHNS 3
#define VI_MAX_BUFS 4
#define VI_IRQ_FRONT 16 /* frame start, shared */
#define VI_IRQ_BACK 18 /* frame end */

/* register offsets from VI_REG_BASE */
#define REG_WIN_X0 0x98
#define REG_WIN_X1 0x9c
#define REG_WIN_Y0 0xa0
#define REG_WIN_Y1 0xa4
#define REG_CAP_W 0x50008
#define REG_CAP_H 0x5000c
#define REG_SCALE1 0x78000 /* channel 1: h, v ((src << 10) / dst), 0 */
#define REG_CROP0 0x7b000 /* channel 0: 0, w, 0, h */
#define REG_STORE(chn) (0x81000 + ((chn) << 12))
#define ST_Y(k) (0x00 + 4 * (k))
#define ST_C(k) (0x10 + 4 * (k))
#define ST_Y_LINE8 0x20
#define ST_Y_LINE 0x24
#define ST_C_LINE8 0x28
#define ST_C_LINE 0x2c
#define ST_ENABLE 0x34
#define ST_NBUF 0x38 /* buffers - 1 */
#define ST_CUR 0x3c /* the buffer being written */
#define ST_RATE 0x40 /* input frames - 1 */
#define ST_MASK 0x44 /* which of them to keep */
#define ST_DONE 0x48
#define ST_HOLD 0x4c /* 1: do not move on to the next buffer */
#define REG_LINE 0xf0008 /* the line being captured */
#define REG_EN_A 0xf0010
#define REG_EN_B 0xf0014
#define REG_FRONT_STATUS 0xf001c
#define REG_EN_C 0xf0030
#define REG_HEIGHT 0xf0034
#define REG_BACK_STATUS 0xf003c

/* store engine addresses: DDR offset in 8-byte units */
#define DDR_BASE 0x80000000u
#define DDR_ADDR(phys) (((phys) - DDR_BASE) >> 3)

#define VI_IOC_SET_DEV _IOW('V', 0, struct vi_dev_attr)
#define VI_IOC_SET_CHN _IOW('V', 1, struct vi_chn_attr)
#define VI_IOC_ENABLE _IOW('V', 2, u32)
#define VI_IOC_DISABLE _IOW('V', 3, u32)
#define VI_IOC_GET_FRAME _IOWR('V', 4, struct vi_frame)
#define VI_IOC_RELEASE_FRAME _IOW('V', 5, struct vi_frame)
#define VI_IOC_INIT _IO('V', 6)
#define VI_IOC_SET_RATE _IOW('V', 7, struct vi_rate)

#define VI_MAX_DIM 4096
/* GET_FRAME waits this long for a frame */
#define VI_WAIT_JIFFIES 200
/* a store engine's DMA drain before its layout changes: two frames at 25 fps */
#define VI_DRAIN_MS 80
/* DMA drain before the reset: three frames at 15 fps */
#define VI_QUIESCE_MS 200

enum { BUF_UNSET = -1, BUF_FREE, BUF_READY, BUF_HELD };

struct vi_dev_attr {
	u32 x0, y0, x1, y1;
};

struct vi_chn_attr {
	u32 chn, w, h;
};

struct vi_rate {
	u32 chn, rate, mask;
};

/* the vendor's 24-byte record */
struct vi_frame {
	u32 chn;
	u32 idx;
	u32 phys_y, phys_c;
	u32 stride, h;
};

struct vi_chn {
	u32 phys_y[VI_MAX_BUFS], phys_c[VI_MAX_BUFS];
	int state[VI_MAX_BUFS];
	struct file *owner[VI_MAX_BUFS]; /* while held through /dev/vi */
	u32 last; /* the newest ready buffer */
	u32 taken, held_cnt, short_cnt;
	u32 w, h;
	u32 phys, mmz_size; /* the MMZ block */
	u32 rate, mask;
	wait_queue_head_t wq;
};

static unsigned int buffer = 4;
module_param(buffer, uint, 0444);
MODULE_PARM_DESC(buffer, "buffer use num");

static void __iomem *regs;
static void __iomem *reset_reg;
static DEFINE_SPINLOCK(vi_lock);
/* serialises setup and VI init, which sleep */
static DEFINE_MUTEX(vi_cfg_lock);
static struct vi_dev_attr win;
static bool win_set;
static struct vi_chn chns[VI_CHNS];
static u32 nbuf;
static struct timeval last_front;
static u32 frame_us;

static inline void vi_write(u32 off, u32 val)
{
	writel(val, regs + off);
}

static inline u32 vi_read(u32 off)
{
	return readl(regs + off);
}

static u32 vi_stride(u32 w)
{
	return ALIGN(w, 128);
}

/* Forget the channels; every buffer unset. vi_lock held */
static void vi_forget(void)
{
	int i, k;

	win_set = false;
	memset(&win, 0, sizeof(win));
	for (i = 0; i < VI_CHNS; i++) {
		memset(&chns[i], 0, offsetof(struct vi_chn, wq));
		for (k = 0; k < VI_MAX_BUFS; k++)
			chns[i].state[k] = BUF_UNSET;
	}
}

/*
 * VI init: stop the store engines, let their DMA drain, reset VI and forget
 * every channel. The vendor driver resets with the engines still writing.
 * Frames held across it can no longer be released. Sleeps.
 */
static void vi_reset(void)
{
	unsigned long flags;
	int i;

	for (i = 0; i < VI_CHNS; i++)
		vi_write(REG_STORE(i) + ST_ENABLE, 0);
	msleep(VI_QUIESCE_MS);
	writel(VI_RESET_VAL, reset_reg);
	udelay(1250);

	spin_lock_irqsave(&vi_lock, flags);
	vi_forget();
	spin_unlock_irqrestore(&vi_lock, flags);
	for (i = 0; i < VI_CHNS; i++)
		wake_up_interruptible(&chns[i].wq);
}

static void vi_set_dev(const struct vi_dev_attr *a)
{
	unsigned long flags;

	spin_lock_irqsave(&vi_lock, flags);
	win = *a;
	win_set = true;
	vi_write(REG_WIN_X0, a->x0);
	vi_write(REG_WIN_X1, a->x1);
	vi_write(REG_WIN_Y0, a->y0);
	vi_write(REG_WIN_Y1, a->y1);
	vi_write(REG_CAP_W, a->x1 - a->x0);
	vi_write(REG_CAP_H, a->y1 - a->y0);
	vi_write(REG_HEIGHT, a->y1 - a->y0);
	vi_write(REG_EN_B, 1);
	vi_write(REG_EN_A, 1);
	vi_write(REG_EN_C, 1);
	spin_unlock_irqrestore(&vi_lock, flags);
}

static void vi_block_name(char *name, size_t len, u32 chn)
{
	snprintf(name, len, "vi_way[%u]", chn);
}

/*
 * Whether the block a channel has is still its block: an MMZ reset forgets
 * every block, VI init or not, and the zone is then handed out again. Only
 * an MMZ module that looks names up (open_mmz) can tell; with one that
 * cannot, the block is taken to be there, as the vendor driver does.
 */
static bool vi_block_valid(u32 chn)
{
	struct vi_chn *c = &chns[chn];
	int (*get_addr)(const char *name, u32 *size, u32 *phys);
	char name[16];
	u32 phys = 0, have = 0;
	bool ok = true;

	get_addr = symbol_get(mmz_get_addr);
	if (!get_addr)
		return true;
	vi_block_name(name, sizeof(name), chn);
	if (get_addr(name, &have, &phys) || phys != c->phys ||
	    have < c->mmz_size)
		ok = false;
	symbol_put(mmz_get_addr);
	return ok;
}

/* The channel's MMZ block, "vi_way[chn]": made on the first setup since VI
 * init; one made before it (by a previous load) is found by name where the
 * MMZ module can look names up */
static int vi_get_block(u32 chn, u32 size)
{
	struct vi_chn *c = &chns[chn];
	int (*get_addr)(const char *name, u32 *size, u32 *phys);
	char name[16];
	u32 phys, have;
	int ret = -1;

	vi_block_name(name, sizeof(name), chn);
	if (!mmz_alloc(name, size, &phys)) {
		c->phys = phys;
		c->mmz_size = size;
		return 0;
	}
	get_addr = symbol_get(mmz_get_addr);
	if (get_addr) {
		if (!get_addr(name, &have, &phys) && phys && have >= size) {
			c->phys = phys;
			c->mmz_size = have;
			ret = 0;
		}
		symbol_put(mmz_get_addr);
	}
	if (ret)
		pr_err("VI: can't malloc memory\n");
	return ret;
}

static int vi_set_chn(u32 chn, u32 w, u32 h)
{
	struct vi_chn *c;
	struct vi_chn *c0 = &chns[0];
	unsigned long flags;
	u32 stride, frame, k, on;

	if (chn >= VI_CHNS || !w || w > VI_MAX_DIM || h < 2 ||
	    h > VI_MAX_DIM) {
		pr_err("VI: invalid size %ux%u\n", w, h);
		return -1;
	}
	if (chn == 1 && (!c0->w || !c0->h)) {
		pr_err("VI: channel 1 scales channel 0, which has no size\n");
		return -1;
	}
	c = &chns[chn];
	for (k = 0; k < nbuf; k++)
		if (c->state[k] == BUF_HELD) {
			pr_err("VI: chn[%u] has frames held\n", chn);
			return -1;
		}
	stride = vi_stride(w);
	frame = stride * h + stride * h / 2;
	if (c->mmz_size && !vi_block_valid(chn)) {
		pr_warn("VI: chn[%u] block is gone from MMZ\n", chn);
		c->mmz_size = 0;
	}
	if (!c->mmz_size) {
		if (vi_get_block(chn, nbuf * frame))
			return -1;
	} else if (c->mmz_size < nbuf * frame) {
		pr_err("VI: too large size\n");
		return -1;
	}

	/* not while the engine writes through the old layout */
	on = vi_read(REG_STORE(chn) + ST_ENABLE);
	if (on) {
		vi_write(REG_STORE(chn) + ST_ENABLE, 0);
		msleep(VI_DRAIN_MS);
	}

	spin_lock_irqsave(&vi_lock, flags);
	c->w = w;
	c->h = h;
	vi_write(REG_STORE(chn) + ST_NBUF, nbuf - 1);
	for (k = 0; k < nbuf; k++) {
		c->phys_y[k] = c->phys + k * frame;
		c->phys_c[k] = c->phys_y[k] + stride * h;
		c->state[k] = BUF_FREE;
		c->owner[k] = NULL;
		vi_write(REG_STORE(chn) + ST_Y(k), DDR_ADDR(c->phys_y[k]));
		vi_write(REG_STORE(chn) + ST_C(k), DDR_ADDR(c->phys_c[k]));
	}
	vi_write(REG_STORE(chn) + ST_Y_LINE8, stride >> 3);
	vi_write(REG_STORE(chn) + ST_Y_LINE, w);
	vi_write(REG_STORE(chn) + ST_C_LINE8, stride >> 3);
	vi_write(REG_STORE(chn) + ST_C_LINE, w);
	if (chn == 0) {
		vi_write(REG_CROP0 + 0, 0);
		vi_write(REG_CROP0 + 4, w);
		vi_write(REG_CROP0 + 8, 0);
		vi_write(REG_CROP0 + 12, h);
	}
	/* channel 1 scales from channel 0: again when either changes */
	if (chns[1].w && chns[1].h && c0->w && c0->h) {
		vi_write(REG_SCALE1 + 0, (c0->w << 10) / chns[1].w);
		vi_write(REG_SCALE1 + 4, (c0->h << 10) / chns[1].h);
		vi_write(REG_SCALE1 + 8, 0);
	}
	spin_unlock_irqrestore(&vi_lock, flags);
	if (on)
		vi_write(REG_STORE(chn) + ST_ENABLE, on);
	return 0;
}

/* Only a channel with buffers: the engine would write wherever its address
 * registers point, DDR offset 0 after a reset */
static int vi_store_enable(u32 chn, u32 on)
{
	if (chn >= VI_CHNS || (on && chns[chn].state[0] == BUF_UNSET))
		return -1;
	vi_write(REG_STORE(chn) + ST_ENABLE, on);
	return 0;
}

static int vi_set_rate(const struct vi_rate *r)
{
	unsigned long flags;

	if (r->chn >= VI_CHNS)
		return -1;
	spin_lock_irqsave(&vi_lock, flags);
	chns[r->chn].rate = r->rate;
	chns[r->chn].mask = r->mask;
	vi_write(REG_STORE(r->chn) + ST_RATE, r->rate);
	vi_write(REG_STORE(r->chn) + ST_MASK, r->mask);
	spin_unlock_irqrestore(&vi_lock, flags);
	return 0;
}

static bool vi_has_ready(const struct vi_chn *c)
{
	u32 k;

	for (k = 0; k < nbuf; k++)
		if (c->state[k] == BUF_READY)
			return true;
	return false;
}

/* Hand out the oldest ready buffer of f->chn: the first after the newest one
 * around the ring. owner is the open file that gets it, NULL for kernel
 * callers */
static int vi_take_frame(struct vi_frame *f, struct file *owner, bool nonblock)
{
	struct vi_chn *c;
	unsigned long flags;
	long left;
	u32 i, k;

	if (f->chn >= VI_CHNS)
		return -1;
	c = &chns[f->chn];
	if (nonblock) {
		if (!vi_has_ready(c))
			return -EAGAIN;
	} else {
		left = wait_event_interruptible_timeout(c->wq, vi_has_ready(c),
							VI_WAIT_JIFFIES);
		if (left < 0)
			return left;
	}

	spin_lock_irqsave(&vi_lock, flags);
	for (i = 1; i <= nbuf; i++) {
		k = (c->last + i) % nbuf;
		if (c->state[k] == BUF_READY)
			break;
	}
	if (i > nbuf) {
		spin_unlock_irqrestore(&vi_lock, flags);
		return -1;
	}
	c->state[k] = BUF_HELD;
	c->owner[k] = owner;
	c->taken++;
	f->idx = k;
	f->phys_y = c->phys_y[k];
	f->phys_c = c->phys_c[k];
	f->stride = vi_stride(c->w);
	f->h = c->h;
	spin_unlock_irqrestore(&vi_lock, flags);
	return 0;
}

int vi_get_frame(struct vi_frame *f)
{
	return vi_take_frame(f, NULL, false);
}
EXPORT_SYMBOL(vi_get_frame);

/* Free a held buffer for whoever took it: the open file, NULL for kernel
 * callers */
static int vi_put_frame(const struct vi_frame *f, struct file *owner)
{
	unsigned long flags;

	spin_lock_irqsave(&vi_lock, flags);
	if (f->chn >= VI_CHNS || f->idx >= VI_MAX_BUFS ||
	    chns[f->chn].state[f->idx] != BUF_HELD ||
	    chns[f->chn].owner[f->idx] != owner) {
		spin_unlock_irqrestore(&vi_lock, flags);
		pr_warn("vi: chn[%u] frame = %u is not using\n", f->chn,
			f->idx);
		return -1;
	}
	chns[f->chn].state[f->idx] = BUF_FREE;
	chns[f->chn].owner[f->idx] = NULL;
	spin_unlock_irqrestore(&vi_lock, flags);
	return 0;
}

int vi_release_frame(struct vi_frame *f)
{
	return vi_put_frame(f, NULL);
}
EXPORT_SYMBOL(vi_release_frame);

/* Give back what an open file still holds, so a consumer that dies or
 * forgets does not keep the store engine held until the next VI init */
static void vi_release_owned(struct file *owner)
{
	unsigned long flags;
	u32 chn, k;

	spin_lock_irqsave(&vi_lock, flags);
	for (chn = 0; chn < VI_CHNS; chn++)
		for (k = 0; k < VI_MAX_BUFS; k++)
			if (chns[chn].state[k] == BUF_HELD &&
			    chns[chn].owner[k] == owner) {
				chns[chn].state[k] = BUF_FREE;
				chns[chn].owner[k] = NULL;
			}
	spin_unlock_irqrestore(&vi_lock, flags);
}

static irqreturn_t vi_front_irq(int irq, void *dev_id)
{
	struct timeval now;

	if (!vi_read(REG_FRONT_STATUS))
		return IRQ_NONE;
	vi_write(REG_FRONT_STATUS, 0);
	do_gettimeofday(&now);
	spin_lock(&vi_lock);
	if (last_front.tv_sec)
		frame_us = (now.tv_sec - last_front.tv_sec) * 1000000 +
			   now.tv_usec - last_front.tv_usec;
	last_front = now;
	spin_unlock(&vi_lock);
	return IRQ_HANDLED;
}

/* The end of a frame, for every channel: as the vendor driver does it */
static void vi_back_chn(u32 chn)
{
	struct vi_chn *c = &chns[chn];
	u32 base = REG_STORE(chn), cur;

	if (!nbuf || c->state[0] == BUF_UNSET)
		return;
	if (vi_read(base + ST_HOLD)) {
		/* held: go on once the next buffer is free */
		c->held_cnt++;
		if (c->state[(vi_read(base + ST_CUR) + 1) % nbuf] == BUF_FREE)
			vi_write(base + ST_HOLD, 0);
		return;
	}
	if (vi_read(base + ST_ENABLE) != 1 || vi_read(base + ST_DONE) != 1)
		return;
	cur = vi_read(base + ST_CUR) % nbuf;
	/* the engine moved on before the last line: the frame is the
	 * previous buffer */
	if (vi_read(REG_LINE) < vi_read(REG_HEIGHT)) {
		cur = (cur + nbuf - 1) % nbuf;
		c->short_cnt++;
	}
	/* not a buffer a reader has: the vendor driver marks it ready, and
	 * its release then fails */
	if (c->state[cur] != BUF_HELD) {
		c->state[cur] = BUF_READY;
		c->last = cur;
	}
	if (c->state[(cur + 1) % nbuf] != BUF_FREE)
		vi_write(base + ST_HOLD, 1);
}

static irqreturn_t vi_back_irq(int irq, void *dev_id)
{
	u32 chn;

	if (!vi_read(REG_BACK_STATUS))
		return IRQ_NONE;
	vi_write(REG_BACK_STATUS, 0);
	spin_lock(&vi_lock);
	for (chn = 0; chn < VI_CHNS; chn++)
		vi_back_chn(chn);
	spin_unlock(&vi_lock);
	for (chn = 0; chn < VI_CHNS; chn++)
		wake_up_interruptible(&chns[chn].wq);
	return IRQ_HANDLED;
}

static long vi_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	void __user *uarg = (void __user *)arg;
	union {
		struct vi_dev_attr dev;
		struct vi_chn_attr chn;
		struct vi_frame frame;
		struct vi_rate rate;
		u32 chn_nr;
	} a;
	int ret;

	if (_IOC_DIR(cmd) & _IOC_WRITE &&
	    (_IOC_SIZE(cmd) > sizeof(a) ||
	     copy_from_user(&a, uarg, _IOC_SIZE(cmd)))) {
		pr_err("vi_ioctl: copy_from_user error.\n");
		return -1;
	}
	switch (cmd) {
	case VI_IOC_SET_DEV:
		mutex_lock(&vi_cfg_lock);
		vi_set_dev(&a.dev);
		mutex_unlock(&vi_cfg_lock);
		return 0;
	case VI_IOC_SET_CHN:
		mutex_lock(&vi_cfg_lock);
		ret = vi_set_chn(a.chn.chn, a.chn.w, a.chn.h);
		mutex_unlock(&vi_cfg_lock);
		return ret;
	case VI_IOC_ENABLE:
	case VI_IOC_DISABLE:
		/* not while VI init drains the DMA */
		mutex_lock(&vi_cfg_lock);
		ret = vi_store_enable(a.chn_nr, cmd == VI_IOC_ENABLE);
		mutex_unlock(&vi_cfg_lock);
		return ret;
	case VI_IOC_GET_FRAME:
		ret = vi_take_frame(&a.frame, file, file->f_flags & O_NONBLOCK);
		if (ret)
			return ret;
		if (copy_to_user(uarg, &a.frame, sizeof(a.frame))) {
			pr_err("vi_ioctl: copy_to_user error.\n");
			vi_put_frame(&a.frame, file);
			return -1;
		}
		/* poll() watches the channels this descriptor takes from */
		file->private_data = (void *)((unsigned long)file->private_data |
					      1ul << a.frame.chn);
		return 0;
	case VI_IOC_RELEASE_FRAME:
		/* only by the descriptor that took it */
		return vi_put_frame(&a.frame, file);
	case VI_IOC_INIT:
		mutex_lock(&vi_cfg_lock);
		vi_reset();
		mutex_unlock(&vi_cfg_lock);
		return 0;
	case VI_IOC_SET_RATE:
		mutex_lock(&vi_cfg_lock);
		ret = vi_set_rate(&a.rate);
		mutex_unlock(&vi_cfg_lock);
		return ret;
	default:
		pr_err("vi_ioctl: cmd error\n");
		return 0;
	}
}

/* /proc/umap/vi in the vendor's layout */
static int vi_seq_show(struct seq_file *s, void *unused)
{
	struct vi_chn copy[VI_CHNS];
	struct vi_dev_attr w;
	unsigned long flags;
	u32 us, fps, ready, k;
	bool ws;
	int i;

	spin_lock_irqsave(&vi_lock, flags);
	memcpy(copy, chns, sizeof(copy));
	w = win;
	ws = win_set;
	us = frame_us;
	spin_unlock_irqrestore(&vi_lock, flags);

	seq_puts(s, "[VI] openxiongmai xm510_vi\n\n");
	seq_puts(s, "-----VI PHYCHN ATTR----------------------------------"
		    "--------------------------\n");
	seq_puts(s, "PhyChn  CapX  CapY  CapW  CapH  DstW  DstH  Mirror  "
		    "Flip  IntEn  PixFom  SrcRat  DstRat\n");
	seq_printf(s, "%-6d  %-4d  %-4d  %-4d  %-4d  %-4d  %-4d  ", 0, w.x0,
		   w.y0, w.x1 - w.x0, w.y1 - w.y0, copy[0].w, copy[0].h);
	seq_printf(s, "%-6s  %-4s  %-5s  %-6s  %-6d  %-6d\n", "N", "N",
		   ws ? "Y" : "N", "sp420", copy[0].rate + 1,
		   hweight32(copy[0].mask));
	seq_puts(s, "-----VI EXTCHN ATTR----------------------------------"
		    "--------------------------\n");
	seq_puts(s, "ExtChn  BindChn  DstW  DstH  PixFom SrcRat DstRat\n");
	for (i = 1; i < VI_CHNS; i++)
		seq_printf(s, "%-6d  %-7d  %-4d  %-4d  %-6s  %-6d  %-6d\n", i,
			   0, copy[i].w, copy[i].h, "sp420", copy[i].rate + 1,
			   hweight32(copy[i].mask));
	seq_puts(s, "-----VI CHN STATUS-----------------------------------"
		    "--------------------------\n");
	seq_puts(s, "ViChn  FrmTime  FrmRate  SendCnt  HoldCnt  OverCnt  "
		    "Buf0  Buf1  Buf2  Buf3  UsableCnt\n");
	fps = us ? (1000000 + us / 2) / us : 0;
	for (i = 0; i < VI_CHNS; i++) {
		for (ready = 0, k = 0; k < VI_MAX_BUFS; k++)
			ready += copy[i].state[k] == BUF_READY;
		seq_printf(s,
			   "%-5d  %-7d  %-7d  %-7d  %-7d  %-7d  %-4d  %-4d  "
			   "%-4d  %-4d  %-9d\n",
			   i, us, fps, copy[i].taken, copy[i].held_cnt,
			   copy[i].short_cnt, copy[i].state[0],
			   copy[i].state[1], copy[i].state[2],
			   copy[i].state[3], ready);
	}
	return 0;
}

static int vi_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, vi_seq_show, NULL);
}

static const struct file_operations vi_proc_fops = {
	.owner = THIS_MODULE,
	.open = vi_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

/* Readable while a channel has a ready buffer: any of the channels this
 * descriptor has taken frames from, or any before its first frame */
static unsigned int vi_poll(struct file *file, poll_table *wait)
{
	unsigned long chns_of = (unsigned long)file->private_data;
	unsigned long flags;
	unsigned int mask = 0;
	int i;

	for (i = 0; i < VI_CHNS; i++)
		poll_wait(file, &chns[i].wq, wait);
	spin_lock_irqsave(&vi_lock, flags);
	for (i = 0; i < VI_CHNS; i++)
		if ((!chns_of || chns_of & 1ul << i) && vi_has_ready(&chns[i]))
			mask = POLLIN | POLLRDNORM;
	spin_unlock_irqrestore(&vi_lock, flags);
	return mask;
}

static int vi_file_open(struct inode *inode, struct file *file)
{
	file->private_data = NULL; /* channels taken from, for poll() */
	return 0;
}

static int vi_file_release(struct inode *inode, struct file *file)
{
	vi_release_owned(file);
	return 0;
}

static const struct file_operations vi_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = vi_ioctl,
	.open = vi_file_open,
	.poll = vi_poll,
	.release = vi_file_release,
};

static struct miscdevice vi_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "vi",
	.fops = &vi_fops,
};

/* the shared front interrupt needs a dev_id of its own */
static int vi_front_dev;

static void vi_stop(void)
{
	int i;

	vi_write(REG_EN_A, 0);
	vi_write(REG_EN_C, 0);
	for (i = 0; i < VI_CHNS; i++)
		vi_write(REG_STORE(i) + ST_ENABLE, 0);
}

static int __init xm510_vi_init(void)
{
	int i, ret;

	nbuf = buffer >= 2 && buffer <= VI_MAX_BUFS ? buffer : VI_MAX_BUFS;
	regs = ioremap(VI_REG_BASE, VI_REG_SIZE);
	reset_reg = ioremap(VI_RESET_REG, 4);
	if (!regs || !reset_reg) {
		pr_err("VI: ioremap video in base failed!\n");
		ret = -ENOMEM;
		goto unmap;
	}
	for (i = 0; i < VI_CHNS; i++)
		init_waitqueue_head(&chns[i].wq);
	vi_reset();
	ret = request_irq(VI_IRQ_FRONT, vi_front_irq, IRQF_SHARED,
			  "vi_front_v", &vi_front_dev);
	if (ret) {
		pr_err("VI: VI_FRONT_V_IRQ request_irq failed!\n");
		goto unmap;
	}
	ret = request_irq(VI_IRQ_BACK, vi_back_irq, 0, "vi_back_v", NULL);
	if (ret) {
		pr_err("VI: VI_BACK_V_IRQ request_irq failed!\n");
		goto free_front;
	}
	ret = misc_register(&vi_misc);
	if (ret) {
		pr_err("VI: register vi device failed!\n");
		goto free_back;
	}
	proc_create("umap/vi", 0444, NULL, &vi_proc_fops);
	pr_info("VI: openxiongmai xm510_vi init ok, %u buffers\n", nbuf);
	return 0;

free_back:
	free_irq(VI_IRQ_BACK, NULL);
free_front:
	free_irq(VI_IRQ_FRONT, &vi_front_dev);
unmap:
	if (reset_reg)
		iounmap(reset_reg);
	if (regs)
		iounmap(regs);
	return ret;
}

static void __exit xm510_vi_exit(void)
{
	remove_proc_entry("umap/vi", NULL);
	misc_deregister(&vi_misc);
	/* the vendor driver's enables off, then the store engines' DMA */
	vi_stop();
	msleep(VI_QUIESCE_MS);
	free_irq(VI_IRQ_FRONT, &vi_front_dev);
	free_irq(VI_IRQ_BACK, NULL);
	iounmap(reset_reg);
	iounmap(regs);
}

module_init(xm510_vi_init);
module_exit(xm510_vi_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("XM510 video input, compatible with xm510_vi.ko");
