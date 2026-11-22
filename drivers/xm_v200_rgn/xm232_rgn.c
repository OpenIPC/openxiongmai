// SPDX-License-Identifier: GPL-2.0
/*
 * The XM232's regions, /dev/rgn: a drop-in for its vendor rgn.ko
 * (RGN_CHIP=xm232).
 *
 * The XM232 has no OSD engine and no VI or VPSS covers: both kinds of
 * region are drawn by the H.264/H.265 encoder at 0x21000000 itself, and
 * venc.ko calls comp_osd_process() for each picture before it queues it.
 *   overlays  kind 0, handles 0..255: formats 0 (ARGB1555), 4 (1 bit) and 5
 *             (2 bits), their bitmaps kept in 16x16 tiles; up to 8 a VENC
 *             channel, the encoder's region n at +0x2A8 + 0x24 * n, the
 *             enable mask at +0x2A4
 *   covers    kind 1, handles 0..255: filled or framed rectangles; up to 16
 *             a VENC channel, the encoder's cover n at +0x1A4 + 0x10 * n,
 *             the enable mask at +0x1A0
 *
 * Every ioctl takes the same 60 bytes (struct rgn_arg): the handle, the kind
 * and 13 words, as on the other V200 chips:
 *   0x403c5201  create: an overlay's struct osd_attr from word 2, a cover's
 *               x, y, w, h, colour, type and thickness
 *   0x403c5202  destroy (detached only)
 *   0x403c5203  set the attributes (an overlay keeps format and canvas)
 *   0x403c5204  an overlay's bitmap: words 2, 3 the user address and length
 *   0x403c5207  attach: words 2..4 module (7, VENC), dev, chn
 *   0x403c5208  detach
 *   0x5200      every region forgotten
 *
 * The encoder's overlay region: +0 the first 16x16 block (column << 10 |
 * row), +4 the last, +8 the format (format 4: 0, 5: 1, 0: 3), +0xC the scale
 * (down | across << 3, a shift: 0 1x, 1 2x, 2 4x), +0x10..+0x1C the four
 * colours, +0x20 the bitmap. A cover: +0 its thickness, +4 y | x << 14,
 * +8 (y + h) | (x + w) << 14, +0xC its colour.
 */

#include <asm/cacheflush.h>
#include <asm/outercache.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>

#define COMP_BASE 0x21000000 /* the encoder */
#define N_HANDLE 256
#define VENC_CHNS 32	 /* overlay tables */
#define WIN_VENC_CHNS 20 /* cover tables */
#define OSD_PER_CHN 8
#define WIN_PER_CHN 16
#define MOD_VENC 7

#define RGN_RESET 0x5200
#define RGN_CREATE 0x403c5201
#define RGN_DESTROY 0x403c5202
#define RGN_SET_ATTR 0x403c5203
#define RGN_BITMAP 0x403c5204
#define RGN_ATTACH 0x403c5207
#define RGN_DETACH 0x403c5208

#define FMT_ARGB1555 0
#define FMT_COMP_1BIT 4
#define FMT_COMP_2BIT 5

#define OSD_MASK 0x2a4
#define OSD_REGION(n) (0x2a8 + 0x24 * (n))
#define WIN_MASK 0x1a0
#define WIN_REGION(n) (0x1a4 + 0x10 * (n))

struct rgn_arg {
	u32 w[15];
};

/* an overlay's attributes: words 2..14 */
struct osd_attr {
	u32 fmt;
	u32 x, y;
	u32 canvas_w, canvas_h; /* the bitmap, as allocated */
	u32 w, h;		/* the part shown */
	u32 color[4];
	u32 alpha;
	u32 scale; /* bits 0..3 across, 4..7 down: a shift */
};

struct osd_block {
	u32 created, attached;
	u32 phys;
	void __iomem *virt;
	u32 size;
	struct osd_attr a;
	u32 mod, dev, chn;
};

struct win_attr {
	u32 x, y, w, h, color, type, thick;
};

struct win_block {
	u32 created, attached;
	struct win_attr a;
	u32 mod, dev, chn;
};

int mmz_alloc(const char *name, u32 size, u32 *phys);
int mmz_free(u32 phys, u32 *size);

static struct osd_block osd[N_HANDLE];
static struct win_block win[N_HANDLE];
static int osd_venc[VENC_CHNS][OSD_PER_CHN];
static int win_venc[WIN_VENC_CHNS][WIN_PER_CHN];
static u32 calls;
static DEFINE_MUTEX(osd_lock);
static DEFINE_MUTEX(win_lock);
static void __iomem *comp;

static inline void comp_w(u32 off, u32 v)
{
	writel(v, comp + off);
}

static bool comp_fmt(u32 fmt)
{
	return fmt == FMT_ARGB1555 || fmt == FMT_COMP_1BIT ||
	       fmt == FMT_COMP_2BIT;
}

/* bits a pixel */
static u32 osd_bpp(u32 fmt)
{
	return fmt == FMT_COMP_1BIT ? 1 : fmt == FMT_COMP_2BIT ? 2 : 16;
}

static void tables_reset(void)
{
	memset(osd_venc, 0xff, sizeof(osd_venc));
	memset(win_venc, 0xff, sizeof(win_venc));
}

/* one of the encoder's eight overlays */
int comp_osd_process_once(struct osd_attr *a, u32 phys, int n)
{
	u32 xs = a->scale & 0xf, ys = (a->scale & 0xff) >> 4;
	u32 x = a->x & ~15u, y = a->y & ~15u;
	u32 x1 = x - 16 + (ALIGN(a->w, 16) << xs);
	u32 y1 = y - 16 + (ALIGN(a->h, 16) << ys);
	u32 r = OSD_REGION(n);

	comp_w(r, x >> 4 << 10 | y >> 4);
	comp_w(r + 4, x1 >> 4 << 10 | y1 >> 4);
	comp_w(r + 8, a->fmt == FMT_COMP_1BIT ? 0 :
		      a->fmt == FMT_COMP_2BIT ? 1 : 3);
	comp_w(r + 0xc, ys | xs << 3);
	comp_w(r + 0x10, a->color[0]);
	comp_w(r + 0x14, a->color[1]);
	comp_w(r + 0x18, a->color[2]);
	comp_w(r + 0x1c, a->color[3]);
	comp_w(r + 0x20, phys);
	return 0;
}
EXPORT_SYMBOL(comp_osd_process_once);

/* one of the encoder's sixteen covers */
int comp_win_process_once(struct win_attr *w, int n)
{
	u32 r = WIN_REGION(n);

	comp_w(r, w->thick);
	comp_w(r + 4, w->y | w->x << 14);
	comp_w(r + 8, (w->y + w->h) | (w->x + w->w) << 14);
	comp_w(r + 0xc, w->color);
	return 0;
}
EXPORT_SYMBOL(comp_win_process_once);

/*
 * venc.ko: the overlays and covers of VENC channel bind[2] into the
 * encoder, for the picture it is about to encode.
 */
int comp_osd_process(u32 *pic, u32 *bind)
{
	u32 chn = bind[2], mask = 0;
	int i, n = 0;

	if (bind[0] != MOD_VENC)
		return 0;
	calls++;
	mutex_lock(&osd_lock);
	for (i = 0; chn < VENC_CHNS && i < OSD_PER_CHN; i++) {
		int h = osd_venc[chn][i];

		if (h >= 0 && comp_fmt(osd[h].a.fmt)) {
			comp_osd_process_once(&osd[h].a, osd[h].phys, n);
			mask |= 1 << n++;
		}
	}
	comp_w(OSD_MASK, mask);
	mutex_unlock(&osd_lock);
	mutex_lock(&win_lock);
	mask = n = 0;
	for (i = 0; chn < WIN_VENC_CHNS && i < WIN_PER_CHN; i++) {
		int h = win_venc[chn][i];

		if (h >= 0) {
			comp_win_process_once(&win[h].a, n);
			mask |= 1 << n++;
		}
	}
	comp_w(WIN_MASK, mask);
	mutex_unlock(&win_lock);
	return 0;
}
EXPORT_SYMBOL(comp_osd_process);

/* the bytes of a w x h bitmap in tiles */
static u64 tiled_size(u32 fmt, u32 w, u32 h)
{
	return (u64)ALIGN((u64)w, 16) * ALIGN((u64)h, 16) * osd_bpp(fmt) / 8;
}

/* a canvas's bytes, or 0: past 8192 a side its size would wrap */
static u32 osd_size(u32 fmt, u32 w, u32 h)
{
	if (!w || !h || w > 8192 || h > 8192 || !comp_fmt(fmt))
		return 0;
	return tiled_size(fmt, w, h);
}

/* the shown part of a 1- or 2-bit overlay: whole bytes across, even down
 * (on create the vendor's checks a 1-bit one as 2 bits, so that is what is
 * asked for there) */
static bool osd_aligned(u32 fmt, u32 w, u32 h, bool create)
{
	if (fmt == FMT_ARGB1555)
		return true;
	return !((w * (fmt == FMT_COMP_1BIT && !create ? 1 : 2)) & 7) &&
	       !(h & 1);
}

static int osd_create(u32 h, const u32 *w)
{
	struct osd_block *b = &osd[h];
	struct osd_attr a;
	char name[32];
	u32 size;

	memcpy(&a, w, sizeof(a));
	if (b->created) {
		pr_err("rgn: handle %u exists\n", h);
		return -EEXIST;
	}
	size = osd_size(a.fmt, a.canvas_w, a.canvas_h);
	if (!size) {
		pr_err("rgn: handle %u: format %u or canvas %ux%u\n", h, a.fmt,
		       a.canvas_w, a.canvas_h);
		return -EINVAL;
	}
	if (!osd_aligned(a.fmt, a.w, a.h, true)) {
		pr_err("rgn: handle %u param error\n", h);
		return -EINVAL;
	}
	/* the encoder fetches the shown w x h from the block: it must fit, as
	 * osd_set checks for a change */
	if (!a.w || !a.h || tiled_size(a.fmt, a.w, a.h) > size) {
		pr_err("rgn: handle %u: %ux%u beyond its canvas\n", h, a.w, a.h);
		return -EINVAL;
	}
	snprintf(name, sizeof(name), "osd_block[%u]", h);
	mutex_lock(&osd_lock);
	if (mmz_alloc(name, size, &b->phys)) {
		mutex_unlock(&osd_lock);
		pr_err("rgn: %s alloc failed\n", name);
		return -ENOMEM;
	}
	b->virt = ioremap_cached(b->phys, size);
	if (!b->virt) {
		mmz_free(b->phys, &size);
		mutex_unlock(&osd_lock);
		return -ENOMEM;
	}
	b->created = 1;
	b->size = size;
	b->a = a;
	mutex_unlock(&osd_lock);
	return 0;
}

static int osd_set(u32 h, const u32 *w)
{
	struct osd_block *b = &osd[h];
	struct osd_attr a;

	memcpy(&a, w, sizeof(a));
	if (!b->created)
		return -ENOENT;
	if (a.fmt != b->a.fmt) {
		pr_err("rgn: handle %d type %d is err\n", h, a.fmt);
		return -EINVAL;
	}
	if (tiled_size(a.fmt, a.w, a.h) > b->size) {
		pr_err("rgn: handle %u not support modify static attr\n", h);
		return -EINVAL;
	}
	if (!osd_aligned(a.fmt, a.w, a.h, false)) {
		pr_err("rgn: handle %u param error\n", h);
		return -EINVAL;
	}
	mutex_lock(&osd_lock);
	a.canvas_w = b->a.canvas_w;
	a.canvas_h = b->a.canvas_h;
	b->a = a;
	mutex_unlock(&osd_lock);
	return 0;
}

/*
 * The bitmap, w x h of the shown part, into 16x16 tiles: left to right, top
 * to bottom, each tile 16 rows of 16 pixels; ARGB1555 pixels byte-swapped.
 */
static int osd_bitmap(u32 h, const u32 *w)
{
	struct osd_block *b = &osd[h];
	const u8 __user *src = (const u8 __user *)(uintptr_t)w[2];
	u32 len = w[3], bpp, line, ty, tx, y, i, need;
	u8 __iomem *d;
	u8 buf[32];
	int ret = 0;

	if (!b->created)
		return -ENOENT;
	mutex_lock(&osd_lock);
	bpp = osd_bpp(b->a.fmt);
	line = bpp * 16 / 8;
	need = bpp * b->a.w * b->a.h / 8;
	if (len < need || tiled_size(b->a.fmt, b->a.w, b->a.h) > b->size) {
		mutex_unlock(&osd_lock);
		pr_err("rgn: bitmap %u bytes, %u wanted\n", len, need);
		return -EINVAL;
	}
	d = b->virt;
	for (ty = 0; ty < b->a.h && !ret; ty += 16)
		for (tx = 0; tx < b->a.w && !ret; tx += 16) {
			u32 n = min(b->a.w - tx, 16u) * bpp / 8;

			for (y = ty; y < ty + 16; y++, d += line) {
				memset(buf, 0, sizeof(buf));
				if (y < b->a.h &&
				    copy_from_user(buf, src + bpp * (b->a.w * y + tx) / 8,
						   n)) {
					ret = -EFAULT;
					break;
				}
				if (b->a.fmt == FMT_ARGB1555)
					for (i = 0; i + 1 < n; i += 2)
						swap(buf[i], buf[i + 1]);
				memcpy_toio(d, buf, line);
			}
		}
	if (!ret) {
		__cpuc_flush_dcache_area((void __force *)b->virt, b->size);
		outer_flush_range(b->phys, b->phys + b->size);
	}
	mutex_unlock(&osd_lock);
	if (ret)
		pr_err("rgn: bitmap copy failed\n");
	return ret;
}

static int osd_attach(u32 h, const u32 *w, bool on)
{
	struct osd_block *b = &osd[h];
	u32 mod = w[2], dev = w[3], chn = w[4];
	int i, *t;

	if (!b->created)
		return -ENOENT;
	if (on && mod != MOD_VENC) {
		pr_err("rgn: overlays attach to VENC only\n");
		return -EINVAL;
	}
	if (chn >= VENC_CHNS)
		return -EINVAL;
	t = osd_venc[chn];
	mutex_lock(&osd_lock);
	/* attached again to the same channel: its slot */
	for (i = 0; i < OSD_PER_CHN; i++)
		if (t[i] == (int)h || (on && t[i] < 0))
			break;
	if (i == OSD_PER_CHN || (on && b->attached && t[i] != (int)h)) {
		mutex_unlock(&osd_lock);
		pr_err("rgn: handle %u, chn %u: %s\n", h, chn,
		       !on ? "not attached" : b->attached ? "attached elsewhere" : "no free slot");
		return on ? -EBUSY : -ENOENT;
	}
	t[i] = on ? (int)h : -1;
	b->attached = on;
	b->mod = on ? mod : 0;
	b->dev = on ? dev : 0;
	b->chn = on ? chn : 0;
	mutex_unlock(&osd_lock);
	return 0;
}

static int osd_ioctl(unsigned int cmd, u32 h, const u32 *w)
{
	struct osd_block *b = &osd[h];

	switch (cmd) {
	case RGN_CREATE:
		return osd_create(h, w + 2);
	case RGN_DESTROY:
		if (!b->created)
			return -ENOENT;
		if (b->attached) {
			pr_err("rgn: handle %u is attached\n", h);
			return -EBUSY;
		}
		mutex_lock(&osd_lock);
		iounmap(b->virt);
		mmz_free(b->phys, &b->size);
		memset(b, 0, sizeof(*b));
		mutex_unlock(&osd_lock);
		return 0;
	case RGN_SET_ATTR:
		return osd_set(h, w + 2);
	case RGN_BITMAP:
		return osd_bitmap(h, w);
	case RGN_ATTACH:
		return osd_attach(h, w, true);
	case RGN_DETACH:
		return osd_attach(h, w, false);
	}
	return -EINVAL;
}

static int win_ioctl(unsigned int cmd, u32 h, const u32 *w)
{
	struct win_block *b = &win[h];
	u32 mod = w[2], dev = w[3], chn = w[4];
	int i, *t, ret = 0;

	if (cmd != RGN_CREATE && !b->created) {
		pr_err("rgn: handle %u not created\n", h);
		return -ENOENT;
	}
	mutex_lock(&win_lock);
	switch (cmd) {
	case RGN_CREATE:
		if (b->created) {
			pr_err("rgn: handle %u exists\n", h);
			ret = -EEXIST;
			break;
		}
		b->created = 1;
		/* fall through */
	case RGN_SET_ATTR:
		memcpy(&b->a, w + 2, sizeof(b->a));
		break;
	case RGN_DESTROY:
		if (b->attached) {
			pr_err("rgn: handle %u is attached\n", h);
			ret = -EBUSY;
			break;
		}
		memset(b, 0, sizeof(*b));
		break;
	case RGN_ATTACH:
	case RGN_DETACH:
		if (mod != MOD_VENC || chn >= WIN_VENC_CHNS) {
			pr_err("rgn: covers attach to VENC only\n");
			ret = -EINVAL;
			break;
		}
		if (cmd == RGN_ATTACH && b->attached) {
			/* one attachment record: a second would leave the
			 * first channel's slot behind */
			pr_err("rgn: handle %u is attached\n", h);
			ret = -EBUSY;
			break;
		}
		t = win_venc[chn];
		for (i = 0; i < WIN_PER_CHN; i++)
			if (t[i] == (cmd == RGN_ATTACH ? -1 : (int)h))
				break;
		if (i == WIN_PER_CHN) {
			pr_err("rgn: handle %u, chn %u: %s\n", h, chn,
			       cmd == RGN_ATTACH ? "no free slot" : "not attached");
			ret = cmd == RGN_ATTACH ? -EBUSY : -ENOENT;
			break;
		}
		if (cmd == RGN_ATTACH) {
			t[i] = h;
			b->attached = 1;
			b->mod = mod;
			b->dev = dev;
			b->chn = chn;
		} else {
			t[i] = -1;
			b->attached = 0;
			b->mod = b->dev = b->chn = 0;
		}
		break;
	default:
		ret = -EINVAL;
	}
	mutex_unlock(&win_lock);
	return ret;
}

/* every region forgotten, as the vendor's: their MMZ blocks go with the MMZ
 * reset the pipeline starts with */
static void rgn_reset(void)
{
	int h;

	mutex_lock(&osd_lock);
	mutex_lock(&win_lock);
	for (h = 0; h < N_HANDLE; h++)
		if (osd[h].created)
			iounmap(osd[h].virt);
	memset(osd, 0, sizeof(osd));
	memset(win, 0, sizeof(win));
	tables_reset();
	mutex_unlock(&win_lock);
	mutex_unlock(&osd_lock);
}

static long rgn_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct rgn_arg a;
	u32 h;

	if (cmd == RGN_RESET) {
		rgn_reset();
		return 0;
	}
	switch (cmd) {
	case RGN_CREATE:
	case RGN_DESTROY:
	case RGN_SET_ATTR:
	case RGN_BITMAP:
	case RGN_ATTACH:
	case RGN_DETACH:
		break;
	default:
		pr_err("rgn: ioctl cmd %#x error\n", cmd);
		return 0;
	}
	if (copy_from_user(&a, (void __user *)arg, sizeof(a))) {
		pr_err("rgn: copy_from_user error\n");
		return -1;
	}
	h = a.w[0];
	if (h >= N_HANDLE) {
		pr_err("rgn: handle %u error\n", h);
		return -1;
	}
	switch (a.w[1]) {
	case 0:
		return osd_ioctl(cmd, h, a.w) ? -1 : 0;
	case 1:
		if (cmd == RGN_BITMAP)
			return -1;
		return win_ioctl(cmd, h, a.w) ? -1 : 0;
	}
	return -1;
}

static int rgn_show(struct seq_file *s, void *v)
{
	int h;

	seq_puts(s, "[RGN] openxiongmai xm232_rgn\n\n");
	seq_printf(s, "comp_osd_process calls: %u\n", calls);
	seq_puts(s, "-----REGION CHN STATUS OF OVERLAY---------------------------------------------\n");
	seq_puts(s, "Handle  En  X     Y     W     H     Type  Alpha  Up  Addr     Mod  Dev  Chn\n");
	mutex_lock(&osd_lock);
	for (h = 0; h < N_HANDLE; h++) {
		const struct osd_block *b = &osd[h];

		if (b->created)
			seq_printf(s, "%-6d  %-2s  %-4d  %-4d  %-4d  %-4d  %-4d  %-5x  %-2x  %-8X  %-3d  %-3d  %-3d\n",
				   h, b->attached ? "Y" : "N", b->a.x, b->a.y,
				   b->a.w, b->a.h, b->a.fmt, b->a.alpha,
				   b->a.scale, b->phys, b->mod, b->dev, b->chn);
	}
	mutex_unlock(&osd_lock);
	seq_puts(s, "-----REGION CHN STATUS OF COVER-----------------------------------------------\n");
	seq_puts(s, "Handle  En  X     Y     W     H     Color   Type  Thick  Mod  Dev  Chn\n");
	mutex_lock(&win_lock);
	for (h = 0; h < N_HANDLE; h++) {
		const struct win_block *b = &win[h];

		if (b->created)
			seq_printf(s, "%-6d  %-2s  %-4d  %-4d  %-4d  %-4d  %-6X  %-4d  %-5d  %-3d  %-3d  %-3d\n",
				   h, b->attached ? "Y" : "N", b->a.x, b->a.y,
				   b->a.w, b->a.h, b->a.color, b->a.type,
				   b->a.thick, b->mod, b->dev, b->chn);
	}
	mutex_unlock(&win_lock);
	return 0;
}

static int rgn_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, rgn_show, NULL);
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

static int __init xm232_rgn_init(void)
{
	int ret;

	tables_reset();
	comp = ioremap(COMP_BASE, 0x1000);
	if (!comp) {
		pr_err("rgn: ioremap of the encoder failed\n");
		return -ENOMEM;
	}
	proc_create("umap/rgn", 0644, NULL, &rgn_proc_fops);
	ret = misc_register(&rgn_misc);
	if (ret) {
		pr_err("rgn: register device failed!\n");
		remove_proc_entry("umap/rgn", NULL);
		iounmap(comp);
		return ret;
	}
	pr_info("rgn: openxiongmai xm232_rgn init ok\n");
	return 0;
}

static void __exit xm232_rgn_exit(void)
{
	misc_deregister(&rgn_misc);
	remove_proc_entry("umap/rgn", NULL);
	rgn_reset();
	iounmap(comp);
}

module_init(xm232_rgn_init);
module_exit(xm232_rgn_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("XM232 regions, compatible with its rgn.ko");
