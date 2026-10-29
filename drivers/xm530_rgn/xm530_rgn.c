// SPDX-License-Identifier: GPL-2.0
/*
 * xm530_rgn: the XM530/XM550 region (OSD) device, a drop-in replacement for
 * the vendor's xm530_rgn.ko with the same /dev/rgn ABI.
 *
 * VI draws two kinds of region over the picture of each of its three
 * channels, so they appear in every stream encoded from that channel:
 *  - overlays: up to 8 bitmaps a channel, coloured through the channel's
 *    16-entry YUV palette and a colour map per overlay. A bitmap has 1 bit a
 *    pixel (format 0), 2 (format 1) or 4 (format 2), MSB first, rows not
 *    padded. The channel's bitmaps are packed one after another in its OSD
 *    SRAM (8 KiB for channels 0 and 1, 2 KiB for channel 2), which the
 *    driver loads from a shadow copy;
 *  - covers: up to 32 rectangles a channel of one YUV colour, solid or
 *    outlined.
 *
 * The ioctls ('R', records copied in only):
 *  - 1: reset: every region of every channel is removed;
 *  - 2, {chn, handle, w, h, format}: create an overlay;
 *  - 3, {chn, handle, -, -, -}: destroy it;
 *  - 4, {chn, handle, x, y, colour map, effect}: place it;
 *  - 5, {chn, handle, data, size}: load its bitmap from user space;
 *  - 6, {chn, handle, on}: show (1) or hide it;
 *  - 7, {chn, handle, w, h, x, y, colour, effect}: create a cover;
 *  - 8, {chn, handle, ...}: destroy it;
 *  - 9, the same as 7: place it (effect 1 is solid);
 *  - 10, {chn, handle, on}: show (1) or hide it;
 *  - 11, {chn, palette[16]}: load a channel's palette.
 * /proc/umap/rgn lists the regions.
 *
 * What differs from the vendor module:
 *  - channel, handle, size, position and format are checked (the vendor
 *    module indexes its lists and registers with them unchecked);
 *  - errors are errnos, not -1;
 *  - the shadow is the module's own memory, not an MMZ block, so the module
 *    needs no MMZ module; it is not reallocated by each reset (the vendor
 *    module maps a new block on every ioctl 1 and leaks the old mapping);
 *  - a reset and unloading hide every region (the vendor module leaves them
 *    on the picture, with no way left to remove them);
 *  - showing a region needs the region (the vendor module shows whatever its
 *    registers hold); placing a cover that does not exist fails (the vendor
 *    module dereferences NULL, or places another cover);
 *  - destroying an overlay moves the bitmaps after it with memmove (the
 *    vendor module uses an overlapping memcpy);
 *  - the palette ioctl takes the lock, and /proc/umap/rgn is a seq_file
 *    that releases what it allocates.
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
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/swab.h>
#include <linux/uaccess.h>

#define RGN_REG_BASE 0x30079000
#define RGN_REG_SIZE 0x3000
#define RGN_CHN_REGS 0x1000

#define RGN_CHNS 3
#define RGN_OVERLAYS 8
#define RGN_COVERS 32
#define RGN_PALETTE 16
#define RGN_FORMATS 3 /* 1, 2 and 4 bits a pixel */
#define RGN_POS_MAX 0xfff /* overlay x, y, w and h are 12-bit fields */
#define RGN_COVER_MAX 0xffff /* cover corners are 16-bit fields */

/* per channel, at RGN_REG_BASE + chn * RGN_CHN_REGS */
#define REG_OSD_CTRL 0x000 /* bit h shown, 8 + h and 16 + h format */
#define REG_OSD_SYNC 0x004 /* SRAM load: bit 0 ours, bit 1 the hardware's */
#define REG_PALETTE 0x010 /* 16 x 0x00YYUUVV */
#define REG_OSD(h) (0x080 + 16 * (h)) /* 4 words an overlay */
#define REG_COVER_EN 0x200
#define REG_COVER_SOLID 0x204
#define REG_COVER(h) (0x210 + 12 * (h)) /* x0y0, x1y1, colour */
#define OSD_CTRL_ON BIT(31)

#define SYNC_TRIES 514 /* of 10 ms, as the vendor module waits */

struct rgn_osd_create {
	u32 chn, handle, w, h, format;
};

struct rgn_osd_attr {
	u32 chn, handle, x, y, colormap, effect;
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
	u32 chn, handle, w, h, x, y, color, effect;
};

struct rgn_palette {
	u32 chn;
	u32 color[RGN_PALETTE];
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
#define RGN_IOC_PALETTE _IOW('R', 11, struct rgn_palette)

struct rgn_osd {
	bool used;
	u32 w, h, format;
	u32 x, y, colormap, effect;
	u32 off; /* in the channel's SRAM */
};

struct rgn_cover_state {
	bool used;
	u32 w, h, x, y, color, effect;
};

static const struct {
	phys_addr_t base;
	u32 size;
} sram_map[RGN_CHNS] = {
	{ 0x30090000, 0x2000 },
	{ 0x30092000, 0x2000 },
	{ 0x30094000, 0x800 },
};

static DEFINE_MUTEX(rgn_lock);
static void __iomem *rgn_regs;
static void __iomem *sram[RGN_CHNS];
static u32 shadow[RGN_CHNS][0x2000 / 4];
static struct rgn_osd osds[RGN_CHNS][RGN_OVERLAYS];
static struct rgn_cover_state covers[RGN_CHNS][RGN_COVERS];
static struct proc_dir_entry *rgn_proc;

static inline void rgn_write(u32 chn, u32 off, u32 val)
{
	writel(val, rgn_regs + chn * RGN_CHN_REGS + off);
}

static inline u32 rgn_read(u32 chn, u32 off)
{
	return readl(rgn_regs + chn * RGN_CHN_REGS + off);
}

static void rgn_update(u32 chn, u32 off, u32 clear, u32 set)
{
	rgn_write(chn, off, (rgn_read(chn, off) & ~clear) | set);
}

/* The SRAM an overlay takes: whole words, as the vendor module counts */
static u32 osd_slot(u32 w, u32 h, u32 format)
{
	return 4 * (DIV_ROUND_UP(w * h, 32) << format);
}

static u32 osd_bytes(const struct rgn_osd *o)
{
	return DIV_ROUND_UP((o->w * o->h) << o->format, 8);
}

/* Where the next overlay goes: bitmaps are kept packed from 0 */
static u32 osd_end(u32 chn)
{
	u32 h, end = 0;

	for (h = 0; h < RGN_OVERLAYS; h++)
		if (osds[chn][h].used)
			end = max(end, osds[chn][h].off +
					       osd_slot(osds[chn][h].w,
							osds[chn][h].h,
							osds[chn][h].format));
	return end;
}

/* An overlay's four registers, from its state */
static void osd_write(u32 chn, u32 h)
{
	const struct rgn_osd *o = &osds[chn][h];

	/* the vendor module never writes x 0 */
	rgn_write(chn, REG_OSD(h), (o->x ? o->x : 1) | o->y << 16);
	rgn_write(chn, REG_OSD(h) + 4, o->w | o->h << 16);
	rgn_write(chn, REG_OSD(h) + 8,
		  (o->off & 0x3fff) | (o->colormap & 0xff) << 16 |
			  ((o->effect >> 12) & 0xff) << 24);
	rgn_write(chn, REG_OSD(h) + 12,
		  (o->effect & 0xfff) | ((o->colormap >> 8) & 0xf) << 12 |
			  ((o->effect >> 20) & 0xf) << 16);
}

/*
 * Loads the channel's shadow into its SRAM, each word byte-swapped, once the
 * hardware has taken the previous load (bit 1 equal to bit 0), then asks it
 * to take this one. The overlays' registers are written after it, so their
 * offsets change only with the SRAM they point into.
 */
static int osd_push(u32 chn)
{
	u32 i, n, sync;
	int tries;

	for (tries = 0;; tries++) {
		sync = rgn_read(chn, REG_OSD_SYNC);
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
	rgn_write(chn, REG_OSD_SYNC, (sync & 1) ^ 1);
	for (i = 0; i < RGN_OVERLAYS; i++)
		if (osds[chn][i].used)
			osd_write(chn, i);
	return 0;
}

/* After a destroy: the bitmaps moved down in the shadow to close the gap, in
 * order; the next load gives the hardware their new offsets */
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
				osd_slot(next->w, next->h, next->format));
			next->off = at;
		}
		at += osd_slot(next->w, next->h, next->format);
	}
}

static void rgn_hide_all(void)
{
	u32 chn;

	for (chn = 0; chn < RGN_CHNS; chn++) {
		rgn_write(chn, REG_OSD_CTRL, 0);
		rgn_write(chn, REG_COVER_EN, 0);
		rgn_write(chn, REG_COVER_SOLID, 0);
	}
	memset(osds, 0, sizeof(osds));
	memset(covers, 0, sizeof(covers));
	memset(shadow, 0, sizeof(shadow));
}

static int osd_create(const struct rgn_osd_create *r)
{
	struct rgn_osd *o;
	u32 slot, end, h = r->handle, chn = r->chn;
	int ret;

	if (chn >= RGN_CHNS || h >= RGN_OVERLAYS || !r->w || !r->h ||
	    r->w > RGN_POS_MAX || r->h > RGN_POS_MAX || r->format >= RGN_FORMATS)
		return -EINVAL;
	o = &osds[chn][h];
	if (o->used) {
		pr_err("RGN: block already exists\n");
		return -EEXIST;
	}
	slot = osd_slot(r->w, r->h, r->format);
	end = osd_end(chn);
	if (end + slot > sram_map[chn].size) {
		pr_err("RGN: size too large\n");
		return -ENOSPC;
	}
	memset(o, 0, sizeof(*o));
	o->used = true;
	o->w = r->w;
	o->h = r->h;
	o->format = r->format;
	o->off = end;
	/* the SRAM there may hold an earlier overlay's pixels */
	memset((u8 *)shadow[chn] + o->off, 0, slot);
	ret = osd_push(chn);
	if (ret) {
		o->used = false;
		return ret;
	}
	rgn_update(chn, REG_OSD_CTRL, BIT(8 + h) | BIT(16 + h),
		   r->format ? BIT(8 * r->format + h) : 0);
	return 0;
}

static int osd_destroy(u32 chn, u32 h)
{
	int ret;

	if (chn >= RGN_CHNS || h >= RGN_OVERLAYS)
		return -EINVAL;
	if (!osds[chn][h].used) {
		pr_warn("RGN: block is not create\n");
		return -ENOENT;
	}
	rgn_update(chn, REG_OSD_CTRL,
		   OSD_CTRL_ON | BIT(h) | BIT(8 + h) | BIT(16 + h), 0);
	osds[chn][h].used = false;
	osd_compact(chn);
	ret = osd_push(chn);
	rgn_update(chn, REG_OSD_CTRL, 0, OSD_CTRL_ON);
	return ret;
}

static int osd_attr(const struct rgn_osd_attr *r)
{
	struct rgn_osd *o;

	if (r->chn >= RGN_CHNS || r->handle >= RGN_OVERLAYS ||
	    r->x > RGN_POS_MAX || r->y > RGN_POS_MAX)
		return -EINVAL;
	o = &osds[r->chn][r->handle];
	if (!o->used) {
		pr_err("RGN: block not create\n");
		return -ENOENT;
	}
	o->x = r->x;
	o->y = r->y;
	o->colormap = r->colormap;
	o->effect = r->effect;
	osd_write(r->chn, r->handle);
	return 0;
}

static int osd_bitmap(const struct rgn_bitmap *r)
{
	struct rgn_osd *o;
	void *buf;
	u32 n;

	if (r->chn >= RGN_CHNS || r->handle >= RGN_OVERLAYS)
		return -EINVAL;
	o = &osds[r->chn][r->handle];
	if (!o->used) {
		pr_err("RGN: block not create\n");
		return -ENOENT;
	}
	/* more than the overlay takes is cut, as the vendor module does */
	n = min(r->size, osd_bytes(o));
	/* the shadow changes only with the whole bitmap */
	buf = memdup_user((const void __user *)(unsigned long)r->data, n);
	if (IS_ERR(buf)) {
		pr_err("RGN: copy_from_user error.\n");
		return PTR_ERR(buf);
	}
	memcpy((u8 *)shadow[r->chn] + o->off, buf, n);
	kfree(buf);
	return osd_push(r->chn);
}

static int osd_show(const struct rgn_show *r)
{
	if (r->chn >= RGN_CHNS || r->handle >= RGN_OVERLAYS)
		return -EINVAL;
	if (r->on != 1) {
		rgn_update(r->chn, REG_OSD_CTRL, BIT(r->handle), 0);
		return 0;
	}
	if (!osds[r->chn][r->handle].used)
		return -ENOENT;
	rgn_update(r->chn, REG_OSD_CTRL, 0, BIT(r->handle) | OSD_CTRL_ON);
	return 0;
}

static bool cover_valid(const struct rgn_cover *r)
{
	return r->chn < RGN_CHNS && r->handle < RGN_COVERS && r->w && r->h &&
	       r->x <= RGN_COVER_MAX && r->w <= RGN_COVER_MAX - r->x &&
	       r->y <= RGN_COVER_MAX && r->h <= RGN_COVER_MAX - r->y;
}

static void cover_set(struct rgn_cover_state *c, const struct rgn_cover *r)
{
	c->w = r->w;
	c->h = r->h;
	c->x = r->x;
	c->y = r->y;
	c->color = r->color;
	c->effect = r->effect;
}

static int cover_create(const struct rgn_cover *r)
{
	struct rgn_cover_state *c;

	if (!cover_valid(r))
		return -EINVAL;
	c = &covers[r->chn][r->handle];
	if (c->used) {
		pr_err("RGN: block already exists\n");
		return -EEXIST;
	}
	c->used = true;
	cover_set(c, r);
	return 0;
}

static int cover_destroy(u32 chn, u32 h)
{
	bool was;

	if (chn >= RGN_CHNS || h >= RGN_COVERS)
		return -EINVAL;
	was = covers[chn][h].used;
	covers[chn][h].used = false;
	rgn_update(chn, REG_COVER_EN, BIT(h), 0);
	return was ? 0 : -ENOENT;
}

static int cover_attr(const struct rgn_cover *r)
{
	struct rgn_cover_state *c;
	u32 chn = r->chn, h = r->handle;

	if (!cover_valid(r))
		return -EINVAL;
	c = &covers[chn][h];
	if (!c->used) {
		pr_err("RGN: block not create\n");
		return -ENOENT;
	}
	cover_set(c, r);
	rgn_write(chn, REG_COVER(h) + 8, c->color);
	rgn_write(chn, REG_COVER(h), c->x | c->y << 16);
	rgn_write(chn, REG_COVER(h) + 4, (c->x + c->w) | (c->y + c->h) << 16);
	rgn_update(chn, REG_COVER_SOLID, BIT(h), c->effect == 1 ? BIT(h) : 0);
	return 0;
}

static int cover_show(const struct rgn_show *r)
{
	if (r->chn >= RGN_CHNS || r->handle >= RGN_COVERS)
		return -EINVAL;
	if (r->on != 1) {
		rgn_update(r->chn, REG_COVER_EN, BIT(r->handle), 0);
		return 0;
	}
	if (!covers[r->chn][r->handle].used)
		return -ENOENT;
	rgn_update(r->chn, REG_COVER_EN, 0, BIT(r->handle));
	return 0;
}

static int rgn_palette(const struct rgn_palette *r)
{
	int i;

	if (r->chn >= RGN_CHNS)
		return -EINVAL;
	for (i = 0; i < RGN_PALETTE; i++)
		rgn_write(r->chn, REG_PALETTE + 4 * i, r->color[i]);
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
		struct rgn_palette pal;
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
		ret = cover_destroy(r.cv.chn, r.cv.handle);
		break;
	case RGN_IOC_COVER_ATTR:
		ret = cover_attr(&r.cv);
		break;
	case RGN_IOC_COVER_SHOW:
		ret = cover_show(&r.sh);
		break;
	case RGN_IOC_PALETTE:
		ret = rgn_palette(&r.pal);
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

	seq_puts(s, "[RGN] openxiongmai xm530_rgn\n\n");
	seq_puts(s, "-----REGION CHN STATUS OF OVERLAY----------------------"
		    "-------------------------\n");
	seq_puts(s, "Chn  Block  X     Y     W     H     Format     ColorMap"
		    "    Effect      Offect\n");
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
				   "%-3d  %-5d  %-4d  %-4d  %-4d  %-4d  %-9d  "
				   "0x%-8x  0x%-8x  0x%-8x\n",
				   chn, (int)(next - osds[chn]), next->x,
				   next->y, next->w, next->h, next->format,
				   next->colormap, next->effect,
				   chn * 0x2000 + next->off);
		}
	}
	seq_puts(s, "-----REGION CHN STATUS OF COVER------------------------"
		    "-----------------------\n");
	seq_puts(s, "Chn  Block  X     Y     W     H     Color\n");
	for (chn = 0; chn < RGN_CHNS; chn++)
		for (h = 0; h < RGN_COVERS; h++) {
			c = &covers[chn][h];
			if (c->used)
				seq_printf(s,
					   "%-3d  %-5d  %-4d  %-4d  %-4d  %-4d"
					   "  0x%-8x\n",
					   chn, h, c->x, c->y, c->w, c->h,
					   c->color);
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
	iounmap(rgn_regs);
}

static int __init xm530_rgn_init(void)
{
	int i, ret;

	rgn_regs = ioremap(RGN_REG_BASE, RGN_REG_SIZE);
	if (!rgn_regs) {
		pr_err("RGN: ioremap rgn reg base failed!\n");
		return -ENOMEM;
	}
	for (i = 0; i < RGN_CHNS; i++) {
		sram[i] = ioremap(sram_map[i].base, sram_map[i].size);
		if (!sram[i]) {
			pr_err("RGN: ioremap rgn buf base failed!\n");
			rgn_unmap();
			return -ENOMEM;
		}
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
	pr_info("RGN: openxiongmai xm530_rgn init ok\n");
	return 0;
}

static void __exit xm530_rgn_exit(void)
{
	if (rgn_proc)
		remove_proc_entry("umap/rgn", NULL);
	misc_deregister(&rgn_misc);
	rgn_hide_all();
	rgn_unmap();
}

module_init(xm530_rgn_init);
module_exit(xm530_rgn_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("XM530/XM550 region (OSD) device, compatible with xm530_rgn.ko");
