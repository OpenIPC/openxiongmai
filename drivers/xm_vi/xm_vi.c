// SPDX-License-Identifier: GPL-2.0
/*
 * xm_vi: video input (capture into memory) for the XM530/XM550, a drop-in
 * replacement for the vendor's xm530_vi.ko with the same /dev/vi ioctl ABI
 * (OpenIPC/openxiongmai#34).
 *
 * The ISP hands VI a picture; VI writes it into memory on up to six "ways":
 * way 0 at the capture size, ways 1-5 scaled from it. Ways 0-3 have a store
 * engine of their own (registers at 0x30084000 + way * 0x1000) that fills a
 * ring of buffers in MMZ and raises an interrupt per frame; ways 4 and 5 were
 * produced in software by the vendor driver for motion detection and are not
 * supported here (their setup succeeds, they never deliver a frame).
 *
 * A buffer belongs to the hardware until its bit in the store engine's
 * buffer-full register (+0x08) is set, to software until it is released by
 * writing the complement of that bit back.
 */

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/poll.h>
#include <linux/proc_fs.h>
#include <linux/sched.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/time.h>
#include <linux/uaccess.h>
#include <linux/wait.h>

/* mmz.ko */
extern int mmz_alloc(const char *name, u32 size, u32 *phys);
extern int mmz_get_addr(const char *name, u32 *size, u32 *phys);

#define VI_REG_BASE 0x30000000
#define VI_REG_SIZE 0x100000
/* system controller: VI and ISP reset */
#define VI_RESET_REG 0x2000011c
#define VI_RESET_ASSERT 0x01590000
#define VI_RESET_RELEASE 0x00000001

#define VI_WAYS 6
#define VI_HW_WAYS 4 /* with a store engine and an IRQ */
#define VI_MAX_BUFS 4
#define VI_IRQ_FIRST 65
#define VI_LUMA_WAY 3 /* luma only */

/* register offsets from VI_REG_BASE */
#define REG_CAP_WIN 0x7c000 /* x, x + w, y, y + h */
#define REG_SCALE(way) (0x78000 + (way) * 12) /* h, v, 0 */
#define REG_UPSCALE 0x7d000 /* enable, factor, width */
#define REG_OUT_WIN(way) (0x81000 + ((way) << 12)) /* 0, w, 0, h */
#define REG_STORE(way) (0x84000 + ((way) << 12))
#define ST_RATE 0x00
#define ST_MASK 0x04
#define ST_FULL 0x08 /* buffer k filled: bit k; write ~bit to free */
#define ST_STAT 0x0c
#define ST_Y_LINE8 0x20
#define ST_Y_LINE 0x24
#define ST_C_LINE8 0x28
#define ST_C_LINE 0x2c
#define ST_ENABLE 0x34
#define ST_NBUF 0x38 /* buffers - 1 */
#define ST_Y(k) (0x40 + 4 * (k))
#define ST_C(k) (0xc0 + 4 * (k))
#define ST_Y_LAST(k) (0x140 + 4 * (k))
#define ST_C_LAST(k) (0x1c0 + 4 * (k))
#define REG_IRQ(way) (0xf0080 + ((way) << 5))
#define IRQ_ENABLE 0x00
#define IRQ_LINES 0x04 /* height + 1 */
#define IRQ_STATUS 0x0c
#define REG_PRESHIFT(way) ((way) == 1 ? 0xf0020 : 0xf0024)
#define REG_BIND 0xd2000 /* mode 2: buffers handed to another engine */
#define REG_BIND_Y(val, k) (0xd2044 + (val) * 32 + 4 * (k))
#define REG_BIND_C(val, k) (0xd2054 + (val) * 32 + 4 * (k))
#define REG_BIND_IDX0 0xd2098
#define REG_BIND_IDX1 0xd209c
/* 3D noise reduction control (ISP): on, it reads and writes reference
 * frames in DDR, and a reset in the middle of that DMA freezes the bus */
#define REG_NR3D 0x40000

/* store engine addresses: DDR offset in 8-byte units */
#define DDR_BASE 0x80000000u
#define DDR_ADDR(phys) (((phys) - DDR_BASE) >> 3)

/* ioctls, as the vendor SDK issues them */
#define VI_IOC_SET_DEV _IOW('V', 0, struct vi_dev_attr)
#define VI_IOC_SET_CHN _IOW('V', 1, struct vi_chn_attr)
#define VI_IOC_ENABLE _IOW('V', 2, u32)
#define VI_IOC_DISABLE _IOW('V', 3, u32)
#define VI_IOC_GET_FRAME _IOWR('V', 4, struct vi_frame)
#define VI_IOC_RELEASE_FRAME _IOW('V', 5, struct vi_frame)
#define VI_IOC_INIT _IO('V', 6)
#define VI_IOC_SET_RATE _IOW('V', 7, struct vi_mode)
#define VI_IOC_SET_MODE _IOW('V', 8, struct vi_mode)

#define VI_MODE_BIND 2
#define VI_BIND_DESTS 2 /* bind val: REG_BIND_IDX0 or REG_BIND_IDX1 */
/* largest frame SET_CHN takes, beyond any XM530/XM550 sensor */
#define VI_MAX_DIM 8192
/* GET_FRAME waits this long for a frame */
#define VI_WAIT_JIFFIES 200
/* DMA drain before the reset: three frames at 15 fps */
#define VI_QUIESCE_MS 200

struct vi_dev_attr {
	u32 x, y, w, h;
};

struct vi_chn_attr {
	u32 way, w, h;
};

struct vi_mode {
	u32 way, mode, val;
};

struct vi_frame {
	u32 way;
	u32 seq; /* the way's frame number, for the release */
	u32 phys_y, phys_c;
	u32 w, h;
	u64 pts; /* microseconds, do_gettimeofday */
};

struct vi_buf {
	u32 phys_y, phys_c;
	u64 pts;
	u32 seq; /* while held: the frame number it was handed out as */
	struct file *owner; /* while held through /dev/vi: by which open file */
};

struct vi_way {
	struct vi_buf buf[VI_MAX_BUFS];
	u32 full_seen; /* ST_FULL at the last interrupt */
	u32 held; /* given to user space, not released */
	/* filled buffers not yet handed out, oldest first */
	u8 ready[VI_MAX_BUFS];
	u32 nready;
	u32 recv, sent, dropped;
	u32 stat; /* ST_STAT at the last interrupt */
	u32 w, h;
	u32 mmz_size;
	u32 rate, mask;
	u32 mode, mode_val;
	bool irq_en, store_en;
	struct timeval last_irq;
	u32 frame_us;
	wait_queue_head_t wq;
};

static unsigned int buffer[VI_WAYS] = { 2, 2, 2, 2, 2, 2 };
static int n_buffer = VI_WAYS;
module_param_array(buffer, uint, &n_buffer, 0444);
MODULE_PARM_DESC(buffer, "buffers per way, 1-4");

static void __iomem *regs;
static void __iomem *reset_reg;
static DEFINE_SPINLOCK(vi_lock);
/* serialises setup and VI init, which sleep */
static DEFINE_MUTEX(vi_cfg_lock);
static struct vi_dev_attr cap;
static struct vi_way ways[VI_WAYS];
static u32 nbuf[VI_WAYS];

static inline void vi_write(u32 off, u32 val)
{
	writel(val, regs + off);
}

static inline u32 vi_read(u32 off)
{
	return readl(regs + off);
}

/*
 * Stop the DMA the reset below would cut off: the store engines and 3D noise
 * reduction, which the vendor driver leaves running (the XM550 freeze,
 * tools/xm-vi-hang -N). Whoever sets the ISP up again turns 3D-NR back on.
 */
static void vi_quiesce(void)
{
	int i;

	for (i = 0; i < VI_HW_WAYS; i++)
		vi_write(REG_STORE(i) + ST_ENABLE, 0);
	vi_write(REG_NR3D, 0);
	/* always: the enables may have been cleared just before, with the DMA
	 * still running */
	msleep(VI_QUIESCE_MS);
}

/*
 * Reset VI (and with it the ISP blocks on the same reset line) and forget
 * every way. Frames held across it can no longer be released. Sleeps.
 */
static void vi_reset(void)
{
	unsigned long flags;
	int i;

	vi_quiesce();
	writel(VI_RESET_ASSERT, reset_reg);
	udelay(1250);
	writel(VI_RESET_RELEASE, reset_reg);
	udelay(1250);

	spin_lock_irqsave(&vi_lock, flags);
	memset(&cap, 0, sizeof(cap));
	for (i = 0; i < VI_WAYS; i++)
		memset(&ways[i], 0, offsetof(struct vi_way, wq));
	spin_unlock_irqrestore(&vi_lock, flags);
	for (i = 0; i < VI_WAYS; i++)
		wake_up_interruptible(&ways[i].wq);
}

static int vi_store_enable(u32 way, u32 on)
{
	unsigned long flags;

	if (way >= VI_WAYS || !ways[way].mmz_size)
		return -1;
	spin_lock_irqsave(&vi_lock, flags);
	ways[way].store_en = on;
	if (way < VI_HW_WAYS)
		vi_write(REG_STORE(way) + ST_ENABLE, on);
	spin_unlock_irqrestore(&vi_lock, flags);
	return 0;
}

/* Fill the way's buffers with grey, as the vendor driver does. A block
 * open_mmz took from CMA is RAM, which ARM does not ioremap; the kernel's own
 * mapping of it is uncached already. */
static int vi_clear_buffers(u32 phys, u32 size)
{
	void __iomem *p;

	if (pfn_valid(__phys_to_pfn(phys))) {
		if (PageHighMem(pfn_to_page(__phys_to_pfn(phys + size - 1)))) {
			pr_err("VI: buffer in high memory\n");
			return -ENOMEM;
		}
		memset(phys_to_virt(phys), 0x80, size);
		wmb();
		return 0;
	}
	p = ioremap(phys, size);
	if (!p) {
		pr_err("VI: ioremap video in base failed!\n");
		return -ENOMEM;
	}
	memset_io(p, 0x80, size);
	iounmap(p);
	return 0;
}

/* Pre-shift and scale factors of an ext way: -1 if VI cannot shrink that
 * far */
static int vi_scale(u32 way, u32 w, u32 h, u32 *shift)
{
	if (way == 0 && (cap.w > w * 16 || cap.h > h * 4))
		return -1;
	if (cap.w > w * 64 || cap.h > h * 16)
		return -1;
	if (way == 0)
		*shift = 0;
	else if (cap.w > w * 32)
		*shift = 2;
	else if (cap.h > h * 8 || cap.w > w * 16 || cap.h > h * 4)
		*shift = 1;
	else
		*shift = 0;
	return 0;
}

static int vi_set_chn(u32 way, u32 w, u32 h)
{
	struct vi_way *v;
	char name[16];
	u32 frame, size, phys, shift = 0, k;
	unsigned long flags;
	int ret;

	if (way >= VI_WAYS)
		return -1;
	if (!w || w > VI_MAX_DIM || h < 2 || h > VI_MAX_DIM) {
		pr_err("VI: invalid size %ux%u\n", w, h);
		return -1;
	}
	v = &ways[way];
	frame = w * h + w * h / 2;
	snprintf(name, sizeof(name), "vi_way[%u]", way);

	spin_lock_irqsave(&vi_lock, flags);
	v->w = w;
	v->h = h;
	if (way < VI_HW_WAYS)
		vi_write(REG_STORE(way) + ST_NBUF, nbuf[way] - 1);
	spin_unlock_irqrestore(&vi_lock, flags);

	/* the way's buffers, in MMZ by name; the block outlives VI init and
	 * module reloads, so reuse one already there */
	if (!v->mmz_size) {
		size = nbuf[way] * frame;
		if (mmz_alloc(name, size, &phys)) {
			u32 have;

			if (mmz_get_addr(name, &have, &phys) || !phys ||
			    have < size) {
				pr_err("VI: can't malloc memory\n");
				return -1;
			}
			size = have;
		}
		v->mmz_size = size;
	} else if (v->mmz_size < nbuf[way] * frame) {
		pr_err("VI: too large size\n");
		return -1;
	} else if (mmz_get_addr(name, &size, &phys)) {
		pr_err("VI: mmz_get_addr %s error\n", name);
		return -1;
	}
	for (k = 0; k < nbuf[way]; k++) {
		v->buf[k].phys_y = phys + k * frame;
		v->buf[k].phys_c = v->buf[k].phys_y + w * h;
	}
	ret = vi_clear_buffers(phys, v->mmz_size);
	if (ret)
		return ret;
	if (way >= VI_HW_WAYS)
		return 0;

	spin_lock_irqsave(&vi_lock, flags);
	for (k = 0; k < nbuf[way]; k++) {
		u32 y = v->buf[k].phys_y, c = v->buf[k].phys_c;

		vi_write(REG_STORE(way) + ST_Y(k), DDR_ADDR(y));
		vi_write(REG_STORE(way) + ST_C(k), DDR_ADDR(c));
		vi_write(REG_STORE(way) + ST_Y_LAST(k),
			 DDR_ADDR(y + w * (h - 1)));
		vi_write(REG_STORE(way) + ST_C_LAST(k),
			 DDR_ADDR(c + w * (h / 2 - 1)));
	}
	vi_write(REG_STORE(way) + ST_Y_LINE8, w >> 3);
	vi_write(REG_STORE(way) + ST_Y_LINE, w);
	vi_write(REG_STORE(way) + ST_C_LINE8, way == VI_LUMA_WAY ? 0 : w >> 3);
	vi_write(REG_STORE(way) + ST_C_LINE, way == VI_LUMA_WAY ? 0 : w);
	vi_write(REG_IRQ(way) + IRQ_LINES, h + 1);
	vi_write(REG_IRQ(way) + IRQ_ENABLE, 1);
	v->irq_en = true;
	if (way == VI_LUMA_WAY)
		goto out;

	if (vi_scale(way, w, h, &shift)) {
		spin_unlock_irqrestore(&vi_lock, flags);
		pr_err("VI: too small size\n");
		return -1;
	}
	if (way)
		vi_write(REG_PRESHIFT(way), shift);
	vi_write(REG_SCALE(way) + 0, ((cap.w << 10) >> shift) / w);
	vi_write(REG_SCALE(way) + 4, ((cap.h << 10) >> shift) / h);
	vi_write(REG_SCALE(way) + 8, 0);
	if (way == 0) {
		/* wider than the capture: the upscaler */
		if (w > cap.w) {
			vi_write(REG_SCALE(0) + 0, 0x400);
			vi_write(REG_UPSCALE + 4, (cap.w << 10) / w);
			vi_write(REG_UPSCALE + 8, w);
			vi_write(REG_UPSCALE + 0, 1);
		} else {
			vi_write(REG_UPSCALE + 0, 0);
		}
	}
	vi_write(REG_OUT_WIN(way) + 0, 0);
	vi_write(REG_OUT_WIN(way) + 4, w);
	vi_write(REG_OUT_WIN(way) + 8, 0);
	vi_write(REG_OUT_WIN(way) + 12, h);
out:
	spin_unlock_irqrestore(&vi_lock, flags);
	return 0;
}

static void vi_set_dev(const struct vi_dev_attr *a)
{
	unsigned long flags;

	spin_lock_irqsave(&vi_lock, flags);
	cap = *a;
	vi_write(REG_CAP_WIN + 0, a->x);
	vi_write(REG_CAP_WIN + 4, a->x + a->w);
	vi_write(REG_CAP_WIN + 8, a->y);
	vi_write(REG_CAP_WIN + 12, a->y + a->h);
	spin_unlock_irqrestore(&vi_lock, flags);
}

/* RATE (rate, keep mask) and MODE share their argument; RATE also does what
 * MODE does with it, as in the vendor driver */
static int vi_set_mode(const struct vi_mode *m, bool rate)
{
	struct vi_way *v;
	unsigned long flags;
	u32 k;

	if (m->way >= VI_WAYS)
		return -1;
	v = &ways[m->way];
	if (rate && m->way < VI_HW_WAYS) {
		spin_lock_irqsave(&vi_lock, flags);
		v->rate = m->mode;
		v->mask = m->val;
		spin_unlock_irqrestore(&vi_lock, flags);
		vi_write(REG_STORE(m->way) + ST_RATE, m->mode);
		vi_write(REG_STORE(m->way) + ST_MASK, m->val);
	}
	if (!v->mmz_size)
		return -1;
	if (m->mode == VI_MODE_BIND && m->val >= VI_BIND_DESTS)
		return -1;
	spin_lock_irqsave(&vi_lock, flags);
	v->mode_val = m->val;
	v->mode = m->mode;
	if (m->mode == VI_MODE_BIND) {
		for (k = 0; k < nbuf[m->way]; k++) {
			vi_write(REG_BIND_Y(m->val, k), v->buf[k].phys_y);
			vi_write(REG_BIND_C(m->val, k), v->buf[k].phys_c);
		}
		vi_write(REG_BIND, 1);
	}
	spin_unlock_irqrestore(&vi_lock, flags);
	return 0;
}

/* Hand out the oldest filled buffer of f->way; owner is the open file that
 * gets it, NULL for kernel callers */
static int vi_take_frame(struct vi_frame *f, struct file *owner, bool nonblock)
{
	struct vi_way *v;
	unsigned long flags;
	long left;
	u32 k;

	if (f->way >= VI_WAYS)
		return -1;
	v = &ways[f->way];
	if (nonblock) {
		if (!v->nready)
			return -EAGAIN;
	} else {
		left = wait_event_interruptible_timeout(v->wq, v->nready,
							VI_WAIT_JIFFIES);
		if (left < 0)
			return left;
	}

	spin_lock_irqsave(&vi_lock, flags);
	if (!v->nready) {
		spin_unlock_irqrestore(&vi_lock, flags);
		return -1;
	}
	k = v->ready[0];
	v->nready--;
	memmove(v->ready, v->ready + 1, v->nready);
	f->phys_y = v->buf[k].phys_y;
	f->phys_c = v->buf[k].phys_c;
	f->w = v->w;
	f->h = v->h;
	f->pts = v->buf[k].pts;
	f->seq = v->buf[k].seq = v->sent++;
	v->held |= 1u << k;
	v->buf[k].owner = owner;
	spin_unlock_irqrestore(&vi_lock, flags);
	return 0;
}

int vi_get_frame(struct vi_frame *f)
{
	return vi_take_frame(f, NULL, false);
}
EXPORT_SYMBOL(vi_get_frame);

/* Give buffer k of the way back to the hardware; vi_lock held */
static void vi_put_buf(u32 way, u32 k)
{
	struct vi_way *v = &ways[way];

	v->held &= ~(1u << k);
	v->buf[k].owner = NULL;
	if (way < VI_HW_WAYS)
		vi_write(REG_STORE(way) + ST_FULL, ~(1u << k));
	v->full_seen &= ~(1u << k);
}

int vi_release_frame(struct vi_frame *f)
{
	struct vi_way *v;
	unsigned long flags;
	u32 k;

	if (f->way >= VI_WAYS)
		return -1;
	v = &ways[f->way];
	spin_lock_irqsave(&vi_lock, flags);
	for (k = 0; k < VI_MAX_BUFS; k++)
		if (v->held & 1u << k && v->buf[k].seq == f->seq)
			break;
	if (k == VI_MAX_BUFS) {
		spin_unlock_irqrestore(&vi_lock, flags);
		pr_warn("vi: chn[%u] frame = %u is not using\n", f->way,
			f->seq);
		return -1;
	}
	vi_put_buf(f->way, k);
	spin_unlock_irqrestore(&vi_lock, flags);
	return 0;
}
EXPORT_SYMBOL(vi_release_frame);

/* Give back what an open file still holds, so a consumer that dies or
 * forgets does not keep buffers from the hardware until the next VI init */
static void vi_release_owned(struct file *owner)
{
	unsigned long flags;
	u32 way, k;

	spin_lock_irqsave(&vi_lock, flags);
	for (way = 0; way < VI_WAYS; way++)
		for (k = 0; k < VI_MAX_BUFS; k++)
			if (ways[way].held & 1u << k &&
			    ways[way].buf[k].owner == owner)
				vi_put_buf(way, k);
	spin_unlock_irqrestore(&vi_lock, flags);
}

static irqreturn_t vi_irq(int irq, void *dev_id)
{
	u32 way = (u32)(uintptr_t)dev_id, full, fresh;
	struct vi_way *v = &ways[way];
	struct timeval now;
	int k = -1;

	do_gettimeofday(&now);
	spin_lock(&vi_lock);
	if (!vi_read(REG_IRQ(way) + IRQ_STATUS)) {
		spin_unlock(&vi_lock);
		wake_up_interruptible(&v->wq);
		return IRQ_NONE;
	}
	vi_write(REG_IRQ(way) + IRQ_STATUS, 0);

	full = vi_read(REG_STORE(way) + ST_FULL);
	fresh = full & ~v->full_seen;
	switch (fresh) {
	case 0:
		break;
	case 1:
		k = 0;
		break;
	case 2:
		k = 1;
		break;
	case 4:
		k = 2;
		break;
	case 8:
		k = 3;
		break;
	default:
		/* more than one at once, order unknown: give them back */
		vi_write(REG_STORE(way) + ST_FULL, ~fresh);
		pr_warn("vi: chn[%u] buffers 0x%x at once\n", way, fresh);
		full &= ~fresh;
		v->dropped++;
		break;
	}
	if (k >= 0) {
		v->buf[k].pts = (u64)now.tv_sec * 1000000 + now.tv_usec;
		v->recv++;
		if (v->mode == VI_MODE_BIND) {
			vi_write(v->mode_val ? REG_BIND_IDX1 : REG_BIND_IDX0,
				 k);
			vi_write(REG_STORE(way) + ST_FULL, ~fresh);
			full &= ~fresh;
			v->sent++;
		} else if (v->nready < VI_MAX_BUFS) {
			v->ready[v->nready++] = k;
		}
	}
	v->full_seen = full;
	v->stat = vi_read(REG_STORE(way) + ST_STAT);
	if (v->last_irq.tv_sec)
		v->frame_us = (now.tv_sec - v->last_irq.tv_sec) * 1000000 +
			      now.tv_usec - v->last_irq.tv_usec;
	v->last_irq = now;
	spin_unlock(&vi_lock);
	wake_up_interruptible(&v->wq);
	return IRQ_HANDLED;
}

static long vi_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	void __user *uarg = (void __user *)arg;
	union {
		struct vi_dev_attr dev;
		struct vi_chn_attr chn;
		struct vi_frame frame;
		struct vi_mode mode;
		u32 way;
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
		ret = vi_set_chn(a.chn.way, a.chn.w, a.chn.h);
		mutex_unlock(&vi_cfg_lock);
		return ret;
	case VI_IOC_ENABLE:
	case VI_IOC_DISABLE:
		/* not while VI init drains the DMA */
		mutex_lock(&vi_cfg_lock);
		ret = vi_store_enable(a.way, cmd == VI_IOC_ENABLE);
		mutex_unlock(&vi_cfg_lock);
		return ret;
	case VI_IOC_GET_FRAME:
		/* poll() watches the ways this descriptor asks for */
		if (a.frame.way < VI_WAYS)
			file->private_data =
				(void *)((unsigned long)file->private_data |
					 1ul << a.frame.way);
		ret = vi_take_frame(&a.frame, file, file->f_flags & O_NONBLOCK);
		if (ret)
			return ret;
		if (copy_to_user(uarg, &a.frame, sizeof(a.frame))) {
			pr_err("vi_ioctl: copy_to_user error.\n");
			vi_release_frame(&a.frame);
			return -1;
		}
		return 0;
	case VI_IOC_RELEASE_FRAME:
		return vi_release_frame(&a.frame);
	case VI_IOC_INIT:
		mutex_lock(&vi_cfg_lock);
		vi_reset();
		mutex_unlock(&vi_cfg_lock);
		return 0;
	case VI_IOC_SET_RATE:
		mutex_lock(&vi_cfg_lock);
		ret = vi_set_mode(&a.mode, true);
		mutex_unlock(&vi_cfg_lock);
		return ret;
	case VI_IOC_SET_MODE:
		mutex_lock(&vi_cfg_lock);
		ret = vi_set_mode(&a.mode, false);
		mutex_unlock(&vi_cfg_lock);
		return ret;
	default:
		pr_err("vi_ioctl: cmd error\n");
		return 0;
	}
}

static int vi_seq_show(struct seq_file *s, void *unused)
{
	int i;

	seq_puts(s, "[VI] openxiongmai xm_vi\n\n");
	seq_printf(s, "CapX  CapY  CapW  CapH\n%-4u  %-4u  %-4u  %-4u\n\n",
		   cap.x, cap.y, cap.w, cap.h);
	seq_puts(s, "Way  DstW  DstH  Bufs  IntEn  StEn  Rate  Mask      "
		    "FrmTime  RecvCnt  SendCnt  Dropped  Held\n");
	for (i = 0; i < VI_WAYS; i++) {
		struct vi_way *v = &ways[i];

		seq_printf(s,
			   "%-3d  %-4u  %-4u  %-4u  %-5s  %-4s  %-4u  "
			   "%08x  %-7u  %-7u  %-7u  %-7u  %x\n",
			   i, v->w, v->h, nbuf[i], v->irq_en ? "Y" : "N",
			   v->store_en ? "Y" : "N", v->rate, v->mask,
			   v->frame_us, v->recv, v->sent, v->dropped, v->held);
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

/* Readable while a way has a filled buffer waiting: any of the ways this
 * descriptor has taken frames from, or any way before its first GET_FRAME.
 * GET_FRAME on a descriptor opened O_NONBLOCK returns -EAGAIN for a way that
 * has none. */
static unsigned int vi_poll(struct file *file, poll_table *wait)
{
	unsigned long flags;
	unsigned long ways_of = (unsigned long)file->private_data;
	unsigned int mask = 0;
	int i;

	for (i = 0; i < VI_WAYS; i++)
		poll_wait(file, &ways[i].wq, wait);
	spin_lock_irqsave(&vi_lock, flags);
	for (i = 0; i < VI_WAYS; i++)
		if ((!ways_of || ways_of & 1ul << i) && ways[i].nready)
			mask = POLLIN | POLLRDNORM;
	spin_unlock_irqrestore(&vi_lock, flags);
	return mask;
}

static int vi_file_open(struct inode *inode, struct file *file)
{
	file->private_data = NULL; /* ways taken from, for poll() */
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

static int __init xm_vi_init(void)
{
	int i, ret;

	for (i = 0; i < VI_WAYS; i++)
		nbuf[i] = buffer[i] >= 1 && buffer[i] <= VI_MAX_BUFS ?
				  buffer[i] :
				  VI_MAX_BUFS;
	regs = ioremap(VI_REG_BASE, VI_REG_SIZE);
	reset_reg = ioremap(VI_RESET_REG, 4);
	if (!regs || !reset_reg) {
		pr_err("VI: ioremap failed\n");
		ret = -ENOMEM;
		goto unmap;
	}
	for (i = 0; i < VI_WAYS; i++)
		init_waitqueue_head(&ways[i].wq);
	vi_reset();
	for (i = 0; i < VI_HW_WAYS; i++) {
		ret = request_irq(VI_IRQ_FIRST + i, vi_irq, 0, "vi_back_v",
				  (void *)(uintptr_t)i);
		if (ret) {
			pr_err("VI: VI_BACK_V_IRQ request_irq %d failed!\n",
			       VI_IRQ_FIRST + i);
			goto free_irqs;
		}
	}
	ret = misc_register(&vi_misc);
	if (ret) {
		pr_err("VI: register vi device failed!\n");
		goto free_irqs;
	}
	proc_create("umap/vi", 0444, NULL, &vi_proc_fops);
	pr_info("VI: openxiongmai xm_vi init ok\n");
	return 0;

free_irqs:
	while (--i >= 0)
		free_irq(VI_IRQ_FIRST + i, (void *)(uintptr_t)i);
unmap:
	if (reset_reg)
		iounmap(reset_reg);
	if (regs)
		iounmap(regs);
	return ret;
}

static void __exit xm_vi_exit(void)
{
	int i;

	remove_proc_entry("umap/vi", NULL);
	misc_deregister(&vi_misc);
	for (i = 0; i < VI_HW_WAYS; i++)
		free_irq(VI_IRQ_FIRST + i, (void *)(uintptr_t)i);
	iounmap(reset_reg);
	iounmap(regs);
}

module_init(xm_vi_init);
module_exit(xm_vi_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("XM530/XM550 video input, compatible with xm530_vi.ko");
