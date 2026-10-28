// SPDX-License-Identifier: GPL-2.0
/*
 * xm510_rgn: the XM510 region (OSD) device, a drop-in replacement for the
 * vendor's xm510_rgn.ko with the same /dev/rgn ABI.
 *
 * VI draws two kinds of region over the picture of its two channels:
 *  - overlays: up to 16 bitmaps a channel, 1 bit a pixel, MSB first, rows
 *    not padded, a foreground and a background colour each from the
 *    channel's 16-entry YUV palette. The channel's bitmaps are packed one
 *    after another in its OSD SRAM (12 KiB for channel 0, 8 KiB for channel
 *    1), which the driver loads from a shadow copy;
 *  - covers: up to 8 rectangles, over channel 0, all of one YUV colour.
 *
 * The ioctls ('R', records copied in only):
 *  - 1: reset: every region is removed;
 *  - 2, {chn, handle, w, h}: create an overlay;
 *  - 3, {chn, handle, -, -}: destroy it;
 *  - 4, {chn, handle, x, y, fg, bg, effect}: place it; fg and bg are
 *    palette entries, the index in bits 27-24 and the colour 0xYYUUVV below;
 *  - 5, {chn, handle, data, size}: load its bitmap from user space;
 *  - 6, {chn, handle, on}: show (1) or hide it;
 *  - 7, {handle, w, h, x, y, colour}: create a cover;
 *  - 8, {handle, ...}: destroy it;
 *  - 9, the same as 7: place it, and set the colour of all covers;
 *  - 10, {-, handle, on}: show (1) or hide it.
 * /proc/umap/rgn lists the regions.
 *
 * What differs from the vendor module:
 *  - channel, handle and size are checked (the vendor module indexes its
 *    lists and registers with them unchecked), and errors are errnos;
 *  - destroying an overlay waits for the SRAM handshake as loading a bitmap
 *    does, with a timeout (the vendor module spins for it, forever if VI is
 *    stopped, holding the lock), moves the bitmaps after it with memmove,
 *    and leaves the channel's enable as it was;
 *  - the shadow is the module's own memory, not an MMZ block, so the module
 *    needs no MMZ module; it is not reallocated by each reset;
 *  - a reset and unloading hide every region;
 *  - showing a region needs the region;
 *  - a channel 1 position that scales below 0 is 0 (the vendor module
 *    writes -1, the far right edge, when VI channel 1 is not set up);
 *  - /proc/umap/rgn is a seq_file that releases what it allocates.
 */

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/proc_fs.h>
#include <linux/ratelimit.h>
#include <linux/sched.h>
#include <linux/seq_file.h>
#include <linux/string.h>
#include <linux/swab.h>
#include <linux/uaccess.h>

#define OSD_REG_BASE 0x40079000
#define OSD_REG_SIZE 0x2000
#define OSD_CHN_REGS 0x1000
#define COVER_REG_BASE 0x40077000 /* and, at 0x1000, the VI channel 1 scaler */
#define COVER_REG_SIZE 0x2000

#define RGN_CHNS 2
#define RGN_OVERLAYS 16
#define RGN_COVERS 8
#define RGN_X_MAX 0x7ff /* overlay x and w are 11-bit fields */
#define RGN_Y_MAX 0x3ff /* and y and h 10-bit */

/* per channel, at OSD_REG_BASE + chn * OSD_CHN_REGS */
#define REG_OSD_CTRL 0x000 /* bit h shown, bit 31 the channel's OSD */
#define REG_OSD_SYNC 0x004 /* SRAM load: bit 0 ours, bit 1 the hardware's */
#define REG_PALETTE(i) (0x010 + 4 * (i)) /* 0x00YYUUVV */
#define REG_OSD(h) (0x080 + 16 * (h)) /* 4 words an overlay */
#define OSD_CTRL_ON BIT(31)

/* at COVER_REG_BASE */
#define REG_COVER_EN 0x004
#define REG_COVER_COLOR 0x010 /* one for all covers */
#define REG_COVER(h) (0x040 + 16 * (h)) /* x0, x1, y0, y1 */
#define REG_VI_SCALE1 0x1000 /* VI channel 1 horizontal: (src << 10) / dst */

#define SYNC_TRIES 514 /* of 10 ms, as the vendor module waits */

struct rgn_osd_create {
	u32 chn, handle, w, h;
};

struct rgn_osd_attr {
	u32 chn, handle, x, y, fg, bg, effect;
};

struct rgn_bitmap {
	u32 chn, handle;
	u32 data; /* a user pointer */
	u32 size;
};

struct rgn_show {
	u32 chn, handle, on;
};

struct rgn_cover {
	u32 handle, w, h, x, y, color;
};

#define RGN_IOC_RESET _IO('R', 1)
#define RGN_IOC_OSD_CREATE _IOW('R', 2, struct rgn_osd_create)
#define RGN_IOC_OSD_DESTROY _IOW('R', 3, struct rgn_osd_create)
#define RGN_IOC_OSD_ATTR _IOW('R', 4, struct rgn_osd_attr)
#define RGN_IOC_OSD_BITMAP _IOW('R', 5, struct rgn_bitmap)
#define RGN_IOC_OSD_SHOW _IOW('R', 6, struct rgn_show)
#define RGN_IOC_COVER_CREATE _IOW('R', 7, struct rgn_cover)
#define RGN_IOC_COVER_DESTROY _IOW('R', 8, struct rgn_cover)
#define RGN_IOC_COVER_ATTR _IOW('R', 9, struct rgn_cover)
#define RGN_IOC_COVER_SHOW _IOW('R', 10, struct rgn_show)

struct rgn_osd {
	bool used;
	u32 w, h;
	u32 x, y, fg, bg, effect;
	u32 off; /* in the channel's SRAM */
};

struct rgn_cover_state {
	bool used;
	u32 w, h, x, y, color;
};

static const struct {
	phys_addr_t base;
	u32 size;
} sram_map[RGN_CHNS] = {
	{ 0x40090000, 0x3000 },
	{ 0x40094000, 0x2000 },
};

static DEFINE_MUTEX(rgn_lock);
static void __iomem *osd_regs;
static void __iomem *cover_regs;
static void __iomem *sram[RGN_CHNS];
static u32 shadow[RGN_CHNS][0x3000 / 4];
static struct rgn_osd osds[RGN_CHNS][RGN_OVERLAYS];
static struct rgn_cover_state covers[RGN_COVERS];
static struct proc_dir_entry *rgn_proc;

static inline void osd_wr(u32 chn, u32 off, u32 val)
{
	writel(val, osd_regs + chn * OSD_CHN_REGS + off);
}

static inline u32 osd_rd(u32 chn, u32 off)
{
	return readl(osd_regs + chn * OSD_CHN_REGS + off);
}

static void osd_update(u32 chn, u32 clear, u32 set)
{
	osd_wr(chn, REG_OSD_CTRL, (osd_rd(chn, REG_OSD_CTRL) & ~clear) | set);
}

static void cover_update(u32 clear, u32 set)
{
	writel((readl(cover_regs + REG_COVER_EN) & ~clear) | set,
	       cover_regs + REG_COVER_EN);
}

/* The SRAM an overlay takes: whole words, as the vendor module counts */
static u32 osd_slot(u32 w, u32 h)
{
	return 4 * DIV_ROUND_UP(w * h, 32);
}

/* Where the next overlay goes: bitmaps are kept packed from 0 */
static u32 osd_end(u32 chn)
{
	u32 h, end = 0;

	for (h = 0; h < RGN_OVERLAYS; h++)
		if (osds[chn][h].used)
			end = max(end, osds[chn][h].off +
					       osd_slot(osds[chn][h].w,
							osds[chn][h].h));
	return end;
}

/* Channel 1 is scaled from channel 0's picture horizontally: its x goes to
 * the hardware in channel 0 pixels, as the vendor module converts it */
static u32 osd_x(u32 chn, u32 x)
{
	u32 r, p;

	if (chn != 1)
		return x;
	r = readl(cover_regs + REG_VI_SCALE1);
	p = r * x;
	if (!(p & 0x3ff))
		return p >> 10 ? (p >> 10) - 1 : 0;
	return p >> 10;
}

static void osd_write_offset(u32 chn, u32 h)
{
	const struct rgn_osd *o = &osds[chn][h];

	osd_wr(chn, REG_OSD(h) + 8,
	       (o->off & 0x3fff) | ((o->bg >> 24) & 0xf) << 16 |
		       ((o->fg >> 24) & 0xf) << 20 |
		       ((o->effect >> 12) & 0xff) << 24);
}

/* An overlay's colours and registers, from its state */
static void osd_write(u32 chn, u32 h)
{
	const struct rgn_osd *o = &osds[chn][h];

	osd_wr(chn, REG_PALETTE((o->fg >> 24) & 0xf), o->fg);
	osd_wr(chn, REG_PALETTE((o->bg >> 24) & 0xf), o->bg);
	osd_wr(chn, REG_OSD(h), (osd_x(chn, o->x) & RGN_X_MAX) |
					(o->y & RGN_Y_MAX) << 16);
	osd_wr(chn, REG_OSD(h) + 4, o->w | o->h << 16);
	osd_write_offset(chn, h);
	osd_wr(chn, REG_OSD(h) + 12, o->effect & 0xfff);
}

/*
 * Loads the channel's shadow into its SRAM, each word byte-swapped, once the
 * hardware has taken the previous load (bit 1 equal to bit 0), then asks it
 * to take this one.
 */
static int osd_push(u32 chn)
{
	u32 i, n, sync;
	int tries;

	for (tries = 0;; tries++) {
		sync = osd_rd(chn, REG_OSD_SYNC);
		if ((sync & 1) == ((sync >> 1) & 1))
			break;
		if (tries == SYNC_TRIES) {
			pr_err("RGN: setbitmap timeout\n");
			return -ETIMEDOUT;
		}
		if (schedule_timeout_interruptible(msecs_to_jiffies(10)) ||
		    signal_pending(current))
			return -EINTR;
	}
	n = DIV_ROUND_UP(osd_end(chn), 4);
	for (i = 0; i < n; i++)
		writel(swab32(shadow[chn][i]), sram[chn] + 4 * i);
	osd_wr(chn, REG_OSD_SYNC, (sync & 1) ^ 1);
	return 0;
}

/* After a destroy: the bitmaps moved down to close the gap, in order */
static void osd_compact(u32 chn)
{
	u8 *base = (u8 *)shadow[chn];
	struct rgn_osd *o, *next;
	u32 h, at = 0;

	for (;;) {
		next = NULL;
		for (h = 0; h < RGN_OVERLAYS; h++) {
			o = &osds[chn][h];
			if (o->used && o->off >= at && (!next || o->off < next->off))
				next = o;
		}
		if (!next)
			break;
		if (next->off != at) {
			memmove(base + at, base + next->off,
				osd_slot(next->w, next->h));
			next->off = at;
			osd_write_offset(chn, next - osds[chn]);
		}
		at += osd_slot(next->w, next->h);
	}
}

static void rgn_hide_all(void)
{
	u32 chn;

	for (chn = 0; chn < RGN_CHNS; chn++)
		osd_wr(chn, REG_OSD_CTRL, 0);
	writel(0, cover_regs + REG_COVER_EN);
	memset(osds, 0, sizeof(osds));
	memset(covers, 0, sizeof(covers));
	memset(shadow, 0, sizeof(shadow));
}

static int osd_create(const struct rgn_osd_create *r)
{
	struct rgn_osd *o;
	u32 slot, end;

	if (r->chn >= RGN_CHNS || r->handle >= RGN_OVERLAYS || !r->w ||
	    !r->h || r->w > RGN_X_MAX || r->h > RGN_Y_MAX)
		return -EINVAL;
	o = &osds[r->chn][r->handle];
	if (o->used) {
		pr_err("RGN: block already exists\n");
		return -EEXIST;
	}
	slot = osd_slot(r->w, r->h);
	end = osd_end(r->chn);
	if (end + slot > sram_map[r->chn].size) {
		pr_err("RGN: size too large\n");
		return -ENOSPC;
	}
	memset(o, 0, sizeof(*o));
	o->used = true;
	o->w = r->w;
	o->h = r->h;
	o->off = end;
	memset((u8 *)shadow[r->chn] + o->off, 0, slot);
	return 0;
}

static int osd_destroy(u32 chn, u32 h)
{
	u32 on;
	int ret;

	if (chn >= RGN_CHNS || h >= RGN_OVERLAYS)
		return -EINVAL;
	if (!osds[chn][h].used) {
		pr_warn("RGN: block is not create\n");
		return -ENOENT;
	}
	on = osd_rd(chn, REG_OSD_CTRL) & OSD_CTRL_ON;
	osd_update(chn, OSD_CTRL_ON | BIT(h), 0);
	osds[chn][h].used = false;
	osd_compact(chn);
	ret = osd_push(chn);
	osd_update(chn, 0, on);
	return ret;
}

static int osd_attr(const struct rgn_osd_attr *r)
{
	struct rgn_osd *o;

	if (r->chn >= RGN_CHNS || r->handle >= RGN_OVERLAYS ||
	    r->x > RGN_X_MAX || r->y > RGN_Y_MAX)
		return -EINVAL;
	o = &osds[r->chn][r->handle];
	if (!o->used) {
		pr_err("RGN: block not create\n");
		return -ENOENT;
	}
	o->x = r->x ? r->x : 1; /* the vendor module never writes x 0 */
	o->y = r->y;
	o->fg = r->fg;
	o->bg = r->bg;
	o->effect = r->effect;
	osd_write(r->chn, r->handle);
	return 0;
}

static int osd_bitmap(const struct rgn_bitmap *r)
{
	struct rgn_osd *o;
	u32 n;

	if (r->chn >= RGN_CHNS || r->handle >= RGN_OVERLAYS)
		return -EINVAL;
	o = &osds[r->chn][r->handle];
	if (!o->used) {
		pr_err("RGN: block not create\n");
		return -ENOENT;
	}
	n = min(r->size, DIV_ROUND_UP(o->w * o->h, 8));
	if (copy_from_user((u8 *)shadow[r->chn] + o->off,
			   (const void __user *)(unsigned long)r->data, n)) {
		pr_err("RGN: copy_from_user error.\n");
		return -EFAULT;
	}
	return osd_push(r->chn);
}

static int osd_show(const struct rgn_show *r)
{
	if (r->chn >= RGN_CHNS || r->handle >= RGN_OVERLAYS)
		return -EINVAL;
	if (r->on != 1) {
		osd_update(r->chn, BIT(r->handle), 0);
		return 0;
	}
	if (!osds[r->chn][r->handle].used)
		return -ENOENT;
	osd_update(r->chn, 0, BIT(r->handle) | OSD_CTRL_ON);
	return 0;
}

static bool cover_valid(const struct rgn_cover *r)
{
	return r->handle < RGN_COVERS && r->w && r->h && r->w <= RGN_X_MAX &&
	       r->h <= RGN_X_MAX && r->x <= RGN_X_MAX && r->y <= RGN_X_MAX;
}

static int cover_create(const struct rgn_cover *r)
{
	struct rgn_cover_state *c;

	if (!cover_valid(r))
		return -EINVAL;
	c = &covers[r->handle];
	if (c->used) {
		pr_err("RGN: block already exists\n");
		return -EEXIST;
	}
	c->used = true;
	c->w = r->w;
	c->h = r->h;
	c->x = r->x;
	c->y = r->y;
	c->color = r->color;
	return 0;
}

static int cover_destroy(u32 h)
{
	if (h >= RGN_COVERS)
		return -EINVAL;
	covers[h].used = false;
	cover_update(BIT(h), 0);
	return 0;
}

static int cover_attr(const struct rgn_cover *r)
{
	struct rgn_cover_state *c;
	void __iomem *v;

	if (!cover_valid(r))
		return -EINVAL;
	c = &covers[r->handle];
	if (!c->used) {
		pr_err("RGN: block not create\n");
		return -ENOENT;
	}
	c->w = r->w;
	c->h = r->h;
	c->x = r->x ? r->x : 1; /* as the vendor module */
	c->y = r->y;
	c->color = r->color;
	v = cover_regs + REG_COVER(r->handle);
	writel(c->color, cover_regs + REG_COVER_COLOR);
	writel(c->x, v);
	writel(c->x + c->w, v + 4);
	writel(c->y, v + 8);
	writel(c->y + c->h, v + 12);
	return 0;
}

static int cover_show(const struct rgn_show *r)
{
	if (r->handle >= RGN_COVERS)
		return -EINVAL;
	if (r->on != 1) {
		cover_update(BIT(r->handle), 0);
		return 0;
	}
	if (!covers[r->handle].used)
		return -ENOENT;
	cover_update(0, BIT(r->handle));
	return 0;
}

static long rgn_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	void __user *uarg = (void __user *)arg;
	union {
		struct rgn_osd_create oc;
		struct rgn_osd_attr oa;
		struct rgn_bitmap bm;
		struct rgn_show sh;
		struct rgn_cover cv;
	} r;
	long ret;

	if (_IOC_TYPE(cmd) != 'R' || _IOC_SIZE(cmd) > sizeof(r) ||
	    (_IOC_DIR(cmd) != _IOC_NONE && _IOC_DIR(cmd) != _IOC_WRITE)) {
		pr_err_ratelimited("rgn_ioctl: cmd error\n");
		return 0;
	}
	if (_IOC_SIZE(cmd) && copy_from_user(&r, uarg, _IOC_SIZE(cmd))) {
		pr_err("rgn_ioctl: copy_from_user error.\n");
		return -EFAULT;
	}

	mutex_lock(&rgn_lock);
	switch (cmd) {
	case RGN_IOC_RESET:
		rgn_hide_all();
		ret = 0;
		break;
	case RGN_IOC_OSD_CREATE:
		ret = osd_create(&r.oc);
		break;
	case RGN_IOC_OSD_DESTROY:
		ret = osd_destroy(r.oc.chn, r.oc.handle);
		break;
	case RGN_IOC_OSD_ATTR:
		ret = osd_attr(&r.oa);
		break;
	case RGN_IOC_OSD_BITMAP:
		ret = osd_bitmap(&r.bm);
		break;
	case RGN_IOC_OSD_SHOW:
		ret = osd_show(&r.sh);
		break;
	case RGN_IOC_COVER_CREATE:
		ret = cover_create(&r.cv);
		break;
	case RGN_IOC_COVER_DESTROY:
		ret = cover_destroy(r.cv.handle);
		break;
	case RGN_IOC_COVER_ATTR:
		ret = cover_attr(&r.cv);
		break;
	case RGN_IOC_COVER_SHOW:
		ret = cover_show(&r.sh);
		break;
	default:
		pr_err_ratelimited("rgn_ioctl: cmd error\n");
		ret = 0;
		break;
	}
	mutex_unlock(&rgn_lock);
	return ret;
}

static int rgn_seq_show(struct seq_file *s, void *unused)
{
	const struct rgn_osd *o, *next;
	const struct rgn_cover_state *c;
	u32 chn, h, at;

	seq_puts(s, "[RGN] openxiongmai xm510_rgn\n\n");
	seq_puts(s, "-----REGION CHN STATUS OF OVERLAY----------------------"
		    "-------------------------\n");
	seq_puts(s, "Chn  Block  X     Y     W     H     FgColor     BgColor"
		    "     Effect      Offect\n");
	mutex_lock(&rgn_lock);
	for (chn = 0; chn < RGN_CHNS; chn++) {
		/* in SRAM order, which is the order they were made in */
		for (at = 0;; at = next->off + 1) {
			next = NULL;
			for (h = 0; h < RGN_OVERLAYS; h++) {
				o = &osds[chn][h];
				if (o->used && o->off >= at &&
				    (!next || o->off < next->off))
					next = o;
			}
			if (!next)
				break;
			seq_printf(s,
				   "%-3d  %-5d  %-4d  %-4d  %-4d  %-4d  0x%-8x"
				   "  0x%-8x  0x%-8x  0x%-8x\n",
				   chn, (int)(next - osds[chn]), next->x,
				   next->y, next->w, next->h, next->fg,
				   next->bg, next->effect,
				   chn * 0x3000 + next->off);
		}
	}
	seq_puts(s, "-----REGION CHN STATUS OF COVER------------------------"
		    "-----------------------\n");
	seq_puts(s, "Chn  Block  X     Y     W     H     Color\n");
	for (h = 0; h < RGN_COVERS; h++) {
		c = &covers[h];
		if (c->used)
			seq_printf(s,
				   "%-3d  %-5d  %-4d  %-4d  %-4d  %-4d  0x%-8x\n",
				   0, h, c->x, c->y, c->w, c->h, c->color);
	}
	mutex_unlock(&rgn_lock);
	return 0;
}

static int rgn_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, rgn_seq_show, NULL);
}

static const struct file_operations rgn_proc_fops = {
	.owner = THIS_MODULE,
	.open = rgn_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static const struct file_operations rgn_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = rgn_ioctl,
};

static struct miscdevice rgn_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "rgn",
	.fops = &rgn_fops,
};

static void rgn_unmap(void)
{
	int i;

	for (i = 0; i < RGN_CHNS; i++)
		if (sram[i])
			iounmap(sram[i]);
	if (cover_regs)
		iounmap(cover_regs);
	iounmap(osd_regs);
}

static int __init xm510_rgn_init(void)
{
	int i, ret;

	osd_regs = ioremap(OSD_REG_BASE, OSD_REG_SIZE);
	if (!osd_regs) {
		pr_err("RGN: ioremap rgn reg base failed!\n");
		return -ENOMEM;
	}
	cover_regs = ioremap(COVER_REG_BASE, COVER_REG_SIZE);
	for (i = 0; i < RGN_CHNS; i++)
		sram[i] = ioremap(sram_map[i].base, sram_map[i].size);
	if (!cover_regs || !sram[0] || !sram[1]) {
		pr_err("RGN: ioremap rgn buf base failed!\n");
		rgn_unmap();
		return -ENOMEM;
	}
	rgn_hide_all();
	ret = misc_register(&rgn_misc);
	if (ret) {
		pr_err("rgn: register rgn device failed!\n");
		rgn_unmap();
		return ret;
	}
	/* /proc/umap is the MMZ module's */
	rgn_proc = proc_create("umap/rgn", 0444, NULL, &rgn_proc_fops);
	if (!rgn_proc)
		pr_warn("RGN: no /proc/umap/rgn, MMZ not loaded?\n");
	pr_info("RGN: openxiongmai xm510_rgn init ok\n");
	return 0;
}

static void __exit xm510_rgn_exit(void)
{
	if (rgn_proc)
		remove_proc_entry("umap/rgn", NULL);
	misc_deregister(&rgn_misc);
	rgn_hide_all();
	rgn_unmap();
}

module_init(xm510_rgn_init);
module_exit(xm510_rgn_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("XM510 region (OSD) device, compatible with xm510_rgn.ko");
