// SPDX-License-Identifier: GPL-2.0
/*
 * The V200 series' regions (XM530V200, XM533, XM535D1/D2/W1), /dev/rgn:
 * a drop-in for the vendor rgn.ko.
 *
 * Three kinds of region, by handle (0..255 each):
 *   overlays  bitmaps the OSD engine blends into the pictures of the VENC
 *             channel they are attached to: venc.ko calls osd_process on
 *             each picture before it queues it
 *   covers    rectangles VI or VPSS fill (vi_set_chn_win, vpss_set_chn_win),
 *             on the XM535 also the encoder (comp_osd_process)
 *   lines     (XM535) lines VI or VPSS draw (vi_set_chn_line,
 *             vpss_set_grp_line)
 * On the XM535 the 1- and 2-bit overlays of formats 4 and 5 go to the
 * encoder's own OSD instead of the engine: venc.ko calls comp_osd_process
 * for those, and their bitmaps are kept in 16x16 tiles.
 *
 * Every ioctl takes the same 60 bytes, struct rgn_arg: the handle, the kind
 * (0 overlay, 1 cover, 2 line) and 13 words:
 *   0x403c5201  create: an overlay's struct osd_attr, a cover's or line's
 *               attributes from word 2
 *   0x403c5202  destroy (detached only)
 *   0x403c5203  set the attributes (an overlay keeps format and canvas)
 *   0x403c5204  an overlay's bitmap: words 2, 3 the user address and length
 *   0x403c5207  attach: words 2..4 module (7 VENC, 8 VPSS, 0x10 VI), dev, chn
 *   0x403c5208  detach
 *   0x403c5209  the engine's YUV420SP to RGB888 planar conversion: words
 *               2..5 width, height, source, destination (physical)
 *   0x403c520a  one overlay blended into a picture of user space's: words
 *               2..5 as osd_process's picture
 *   0x5200      everything destroyed, the engine reset
 *
 * The engine (0x21300000 on the XM530V200, 0x21200000 on the others, SPI
 * 62) takes up to four regions a run, region n's registers at +0x100 * n;
 * 1 to +0x60 starts it, +0xF0 says it is done, +0xFC / +0xF4 its interrupt.
 * RGN_CHIP=v200, xm533 or xm535 at build time.
 */

#include <asm/cacheflush.h>
#include <asm/outercache.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/math64.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/proc_fs.h>
#include <linux/sched.h>
#include <linux/mm.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <asm/memory.h>
#include <linux/wait.h>

#if defined(RGN_CHIP_XM535)
#define OSD_BASE 0x21200000
#define COMP_BASE 0x21000000 /* the encoder's OSD */
#define OSD_PER_CHN 4
#define HAVE_VPSS 1
#define HAVE_LINES 1
#elif defined(RGN_CHIP_XM533)
#define OSD_BASE 0x21200000
#define OSD_PER_CHN 8
#else
#define OSD_BASE 0x21300000
#define OSD_PER_CHN 8
#define HAVE_VPSS 1
#endif
#define OSD_IRQ (32 + 62)

#define N_HANDLE 256
#define VENC_CHNS 32
#define VI_CHNS 20
#define VPSS_DEVS 32 /* VPSS covers by (dev * 3 + chn) */
#define WIN_PER_CHN 16
#define LINE_PER_CHN 30
#define LINE_VI_CHNS 3

#define MOD_VENC 7
#define MOD_VPSS 8
#define MOD_VI 0x10

#define RGN_RESET 0x5200
#define RGN_CREATE 0x403c5201
#define RGN_DESTROY 0x403c5202
#define RGN_SET_ATTR 0x403c5203
#define RGN_BITMAP 0x403c5204
#define RGN_ATTACH 0x403c5207
#define RGN_DETACH 0x403c5208
#define RGN_CONVERT 0x403c5209
#define RGN_USR_PROCESS 0x403c520a

/* the engine */
#define OSD_REGION(n) (0x100 * (n))
#define OSD_START 0x60
#define OSD_COUNT 0x64 /* regions - 1 */
#define OSD_ORDER 0x68
#define OSD_DONE 0xf0
#define OSD_INT_ACK 0xf4
#define OSD_RESET_DONE 0xf8
#define OSD_INT_ST 0xfc
#define OSD_RESET 0x408

/* the overlay formats */
#define FMT_ARGB1555 0
#define FMT_ARGB8888 1
#define FMT_1BIT 2
#define FMT_2BIT 3
#define FMT_COMP_1BIT 4 /* XM535: the encoder's */
#define FMT_COMP_2BIT 5 /* on the XM535; on the others a 2-bit mode */
#define FMT_CONVERT 6
#define FMT_CONVERT2 7
#define FMT_CONVERT3 8

/* words 0, 1 the handle and kind (0 overlay, 1 cover, 2 line) */
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
	u32 scale; /* bits 0..3 across, 4..7 down: 0 1x, 1 2x (XM535: 3 4x) */
};

struct osd_block {
	u32 created, attached;
	u32 phys;
	void __iomem *virt;
	u32 size;
	struct osd_attr a;
	u32 mod, dev, chn;
};

struct win_block {
	u32 created, attached;
	u32 x, y, w, h, color, type, thick;
	u32 mod, dev, chn;
};

/* what vi_set_chn_win / vpss_set_chn_win take */
struct win_attr {
	u32 en, x, y, w, h, color, type, thick;
};

/* exported by vpss.ko (XM530V200, XM535) or vi.ko (XM533) */
int vi_set_chn_win(u32 dev, u32 chn, u32 idx, struct win_attr *w);
#ifdef HAVE_VPSS
int vpss_set_chn_win(u32 dev, u32 chn, u32 idx, struct win_attr *w);
#endif
int mmz_alloc(const char *name, u32 size, u32 *phys);
int mmz_free(u32 phys, u32 *size);

#ifdef HAVE_LINES
struct line_block {
	u32 created, attached;
	u32 x0, y0, x1, y1, color, thick;
	u32 mod, dev, chn;
};

/* what vi_set_chn_line / vpss_set_grp_line take: the line from (x0, y0) as
 * a direction, its length and a 32.32 step */
struct line_attr {
	u32 en, x0, y0, mode, len, step, thick, color;
};

int vi_set_chn_line(u32 dev, u32 chn, u32 idx, struct line_attr *l);
int vpss_set_grp_line(u32 grp, u32 idx, struct line_attr *l);

static struct line_block lines[N_HANDLE];
static int line_vi[LINE_VI_CHNS][LINE_PER_CHN];
static int line_vpss[VPSS_DEVS][LINE_PER_CHN];
static DEFINE_MUTEX(line_lock);
#endif

static struct osd_block osd[N_HANDLE];
static struct win_block win[N_HANDLE];
static int osd_venc[VENC_CHNS][OSD_PER_CHN];
static int win_vi[VI_CHNS][WIN_PER_CHN];
#ifdef HAVE_VPSS
static int win_vpss[VPSS_DEVS * 3][WIN_PER_CHN];
#endif
#ifdef RGN_CHIP_XM535
static int win_venc[VENC_CHNS][WIN_PER_CHN];
static void __iomem *comp;
#endif
static u32 calls, runs, timeouts;

/* the overlays and the engine; the covers; (lines above) */
static DEFINE_MUTEX(osd_lock);
static DEFINE_MUTEX(win_lock);

static void __iomem *regs;
static DECLARE_WAIT_QUEUE_HEAD(osd_wq);

static inline void reg_w(u32 off, u32 v)
{
	writel(v, regs + off);
}

static void fill(int *t, int n)
{
	while (n--)
		*t++ = -1;
}

static void tables_reset(void)
{
	fill(&osd_venc[0][0], sizeof(osd_venc) / sizeof(int));
	fill(&win_vi[0][0], sizeof(win_vi) / sizeof(int));
#ifdef HAVE_VPSS
	fill(&win_vpss[0][0], sizeof(win_vpss) / sizeof(int));
#endif
#ifdef RGN_CHIP_XM535
	fill(&win_venc[0][0], sizeof(win_venc) / sizeof(int));
#endif
#ifdef HAVE_LINES
	fill(&line_vi[0][0], sizeof(line_vi) / sizeof(int));
	fill(&line_vpss[0][0], sizeof(line_vpss) / sizeof(int));
#endif
}

static irqreturn_t osd_int(int irq, void *dev)
{
	irqreturn_t ret = IRQ_NONE;

	if (readl(regs + OSD_INT_ST)) {
		reg_w(OSD_INT_ACK, 0);
		ret = IRQ_HANDLED;
	}
	wake_up(&osd_wq);
	return ret;
}

static void engine_reset(void)
{
	reg_w(OSD_RESET, 0);
	udelay(50);
	reg_w(OSD_RESET, 1);
	udelay(50);
	reg_w(OSD_RESET_DONE, 1);
}

/*
 * Region n of the next run: attribute a's bitmap at phys blended into pic
 * {stride, height, source, destination}, or for the conversions pic as
 * their own {width, height, source, destination}.
 */
static void region(const u32 *pic, const struct osd_attr *a, u32 phys, int n)
{
	u32 r = OSD_REGION(n), xs = a->scale & 0xf, ys = (a->scale & 0xff) >> 4;
	u32 stride;

	switch (a->fmt) {
	case FMT_ARGB1555:
		reg_w(r + 0x00, 0);
		reg_w(r + 0x04, 1);
		reg_w(r + 0x5c, 0);
		reg_w(r + 0x20, a->alpha);
		stride = a->w * 2;
		break;
	case FMT_ARGB8888:
		reg_w(r + 0x00, 0);
		reg_w(r + 0x04, 0);
		reg_w(r + 0x5c, 0);
		stride = a->w * 4;
		break;
	case FMT_1BIT:
		reg_w(r + 0x00, 1);
		reg_w(r + 0x04, 0);
		reg_w(r + 0x08, a->color[0]);
		reg_w(r + 0x0c, a->color[1]);
		stride = a->w >> 3;
		break;
	case FMT_2BIT:
		reg_w(r + 0x00, 1);
		reg_w(r + 0x04, 4);
		reg_w(r + 0x10, a->color[0]);
#ifdef RGN_CHIP_XM535
		/* the XM535's engine has colours 1 and 2 the other way */
		reg_w(r + 0x14, a->color[2]);
		reg_w(r + 0x18, a->color[1]);
#else
		reg_w(r + 0x14, a->color[1]);
		reg_w(r + 0x18, a->color[2]);
#endif
		reg_w(r + 0x1c, a->color[3]);
		stride = a->w >> 2;
		break;
	case FMT_COMP_2BIT:
		/* not scaled down, the bitmap read from in place of the picture */
		reg_w(r + 0x00, 1);
		reg_w(r + 0x04, 0xc);
		reg_w(r + 0x10, a->color[0]);
		reg_w(r + 0x14, a->color[1]);
		reg_w(r + 0x18, a->color[2]);
		reg_w(r + 0x1c, a->color[3]);
		reg_w(r + 0x30, a->w >> 2);
		reg_w(r + 0x24, xs);
		reg_w(r + 0x28, ys);
		reg_w(r + 0x2c, pic[0]);
		reg_w(r + 0x34, a->h - 1);
		reg_w(r + 0x4c, a->w * (xs + 1));
		reg_w(r + 0x40, phys);
		reg_w(r + 0x44, phys);
		reg_w(r + 0x48, phys);
		goto out;
	case FMT_CONVERT:
		/* YUV420SP to RGB888 planar */
		reg_w(r + 0x00, 3);
		reg_w(r + 0x04, 0);
		reg_w(r + 0x2c, pic[0]);
		reg_w(r + 0x30, pic[0]);
		reg_w(r + 0x4c, pic[0]);
		reg_w(r + 0x34, pic[1] - 1);
		reg_w(r + 0x44, pic[2]);
		reg_w(r + 0x48, pic[0] * pic[1] + pic[2]);
		reg_w(r + 0x58, pic[3]);
		reg_w(r + 0x54, pic[0] * pic[1] + pic[3]);
		reg_w(r + 0x50, pic[3] + pic[1] * pic[0] * 2);
		return;
	case FMT_CONVERT2:
		reg_w(r + 0x00, 2);
		reg_w(r + 0x04, 2);
		reg_w(r + 0x5c, 0);
		reg_w(r + 0x2c, pic[0]);
		reg_w(r + 0x30, pic[0]);
		reg_w(r + 0x4c, pic[0]);
		reg_w(r + 0x34, pic[1] - 1);
		reg_w(r + 0x40, pic[2]);
		reg_w(r + 0x44, pic[0] * pic[1] + pic[2]);
		reg_w(r + 0x48, pic[2] + pic[1] * pic[0] * 2);
		reg_w(r + 0x50, pic[3]);
		reg_w(r + 0x54, pic[0] * pic[1] + pic[3]);
		return;
	case FMT_CONVERT3:
		reg_w(r + 0x00, 4);
		reg_w(r + 0x4c, pic[0]);
		reg_w(r + 0x48, pic[2]);
		reg_w(r + 0x58, pic[3]);
		return;
	default:
		return;
	}
	reg_w(r + 0x30, stride);
	reg_w(r + 0x24, xs);
	reg_w(r + 0x28, ys);
	reg_w(r + 0x2c, pic[0]);
	reg_w(r + 0x34, a->h * (ys + 1) - 1);
	reg_w(r + 0x4c, a->w * (xs + 1));
	reg_w(r + 0x40, phys);
	/* luma and chroma of the region's top left, in the source */
	reg_w(r + 0x44, a->y * pic[0] + a->x + pic[2]);
	reg_w(r + 0x48, pic[0] * (pic[1] + (a->y >> 1)) + a->x + pic[2]);
out:
	/* ...and in the destination */
	reg_w(r + 0x50, a->y * pic[0] + a->x + pic[3]);
	reg_w(r + 0x54, pic[0] * (pic[1] + (a->y >> 1)) + a->x + pic[3]);
}

/* run the engine on n regions; with osd_lock held */
static int run(int n, long timeout)
{
	long ret;

	reg_w(OSD_COUNT, n - 1);
	reg_w(OSD_ORDER, 0x3210);
	reg_w(OSD_START, 1);
	runs++;
	ret = wait_event_interruptible_timeout(osd_wq, readl(regs + OSD_DONE),
					       timeout);
	if (ret)
		return 0;
	engine_reset();
	timeouts++;
	return -ERESTARTSYS;
}

static bool fits(const struct osd_block *b, const u32 *pic)
{
	return b->a.x + b->a.w <= pic[0] && b->a.y + b->a.h <= pic[1];
}

static bool comp_fmt(u32 fmt)
{
#ifdef RGN_CHIP_XM535
	return fmt == FMT_COMP_1BIT || fmt == FMT_COMP_2BIT;
#else
	return false;
#endif
}

/*
 * venc.ko: the overlays attached to VENC channel bind[2] (bind {mod, dev,
 * chn}) blended into picture {stride, height, source, destination}.
 */
int osd_process(u32 *pic, u32 *bind)
{
	int i, n = 0, ret = 0;
	u32 chn = bind[2];

	if (bind[0] != MOD_VENC || chn >= VENC_CHNS)
		return 0;
	mutex_lock(&osd_lock);
	calls++;
	for (i = 0; i < OSD_PER_CHN; i++) {
		int h = osd_venc[chn][i];

		if (h >= 0 && !comp_fmt(osd[h].a.fmt) && fits(&osd[h], pic))
			region(pic, &osd[h].a, osd[h].phys, n++);
		/* four regions a run */
		if (n && ((i & 3) == 3 || i == OSD_PER_CHN - 1)) {
			ret = run(n, HZ / 5);
			n = 0;
			if (ret) {
				pr_err("osd: timeout\n");
				break;
			}
		}
	}
	mutex_unlock(&osd_lock);
	return ret;
}
EXPORT_SYMBOL(osd_process);

#ifdef RGN_CHIP_XM535
/* one of the encoder's four overlays: the region in 16-pixel blocks */
static void comp_osd(const struct osd_attr *a, u32 phys, int n)
{
	u32 xs = a->scale & 0xf, ys = (a->scale & 0xff) >> 4;
	u32 x = a->x & ~15, y = a->y & ~15;
	u32 x1 = (xs + 1) * ALIGN(a->w, 16) + x;
	u32 y1 = (ys + 1) * ALIGN(a->h, 16) + y;
	u32 r = (n + 9) * 0x20;

	writel(x >> 4 << 24 | y >> 4 << 16 | x1 >> 4 << 8 | y1 >> 4, comp + r);
	writel(a->fmt == FMT_COMP_2BIT, comp + r + 4);
	writel(ys | xs << 2, comp + r + 8);
	writel(a->color[0], comp + r + 0xc);
	writel(a->color[1], comp + r + 0x10);
	writel(a->color[2], comp + r + 0x14);
	writel(a->color[3], comp + r + 0x18);
	writel(phys, comp + r + 0x1c);
}

/* one of the encoder's sixteen covers, cut to the picture */
static void comp_win(const struct win_block *w, int n, const u32 *pic)
{
	u32 x1 = min(w->x + w->w, pic[0]), y1 = min(w->y + w->h, pic[1]);
	u32 r = 0x224 + n * 0x10;

	writel(w->thick, comp + r);
	writel(w->y | w->x << 12, comp + r + 4);
	writel(y1 | x1 << 12, comp + r + 8);
	writel(w->color, comp + r + 0xc);
}

/*
 * venc.ko (XM535): the encoder's overlays (formats 4, 5) and covers for
 * VENC channel bind[2], for a picture of pic[0] x pic[1].
 */
int comp_osd_process(u32 *pic, u32 *bind)
{
	u32 chn = bind[2], mask = 0;
	int i, n = 0;

	if (bind[0] != MOD_VENC || chn >= VENC_CHNS)
		return 0;
	mutex_lock(&osd_lock);
	for (i = 0; i < OSD_PER_CHN; i++) {
		int h = osd_venc[chn][i];

		if (h >= 0 && comp_fmt(osd[h].a.fmt)) {
			comp_osd(&osd[h].a, osd[h].phys, n);
			mask |= 1 << n++;
		}
	}
	writel(mask, comp + 0x11c);
	mutex_unlock(&osd_lock);
	mutex_lock(&win_lock);
	mask = n = 0;
	for (i = 0; i < WIN_PER_CHN; i++) {
		int h = win_venc[chn][i];

		if (h >= 0) {
			comp_win(&win[h], n, pic);
			mask |= 1 << n++;
		}
	}
	writel(mask, comp + 0x220);
	mutex_unlock(&win_lock);
	return 0;
}
EXPORT_SYMBOL(comp_osd_process);
#endif

/* a canvas's bytes, or 0: past 8192 a side its size would wrap */
static u32 osd_size(u32 fmt, u32 w, u32 h)
{
	if (!w || !h || w > 8192 || h > 8192)
		return 0;
	switch (fmt) {
	case FMT_1BIT:
		return w * h >> 3;
	case FMT_2BIT:
		return (w * h & 0x7fffffff) >> 2;
	case FMT_ARGB1555:
		return w * h * 2;
	case FMT_ARGB8888:
		return w * h * 4;
#ifdef RGN_CHIP_XM535
	case FMT_COMP_1BIT:
		return ALIGN(w, 16) * ALIGN(h, 16) >> 3;
	case FMT_COMP_2BIT:
		return (ALIGN(w, 16) * ALIGN(h, 16) & 0x7fffffff) >> 2;
#endif
	}
	return 0;
}

#ifdef RGN_CHIP_XM535
/* user space's 2 (2x) in either nibble the engine's 3 */
static u32 scale_hw(u32 s)
{
	u32 x = s & 0xf, y = (s & 0xff) >> 4;

	return (y == 2 ? 3 : y) << 4 | (x == 2 ? 3 : x);
}
#endif

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
#ifdef RGN_CHIP_XM535
	a.scale = scale_hw(a.scale);
#endif
	size = osd_size(a.fmt, a.canvas_w, a.canvas_h);
	if (!size)
		return -EINVAL;
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

static void osd_free(struct osd_block *b)
{
	iounmap(b->virt);
	mmz_free(b->phys, &b->size);
	memset(b, 0, sizeof(*b));
}

static int osd_set(u32 h, const u32 *w)
{
	struct osd_block *b = &osd[h];
	struct osd_attr a;

	memcpy(&a, w, sizeof(a));
	if (!b->created)
		return -ENOENT;
	if (a.fmt != b->a.fmt || a.w * a.h > b->a.canvas_w * b->a.canvas_h) {
		pr_err("rgn: handle %u: format or size\n", h);
		return -EINVAL;
	}
#ifdef RGN_CHIP_XM535
	a.scale = scale_hw(a.scale);
	if (comp_fmt(a.fmt) &&
	    ((a.w * (a.fmt == FMT_COMP_1BIT ? 1 : 2) & 7) || (a.h & 1))) {
		pr_err("rgn: handle %u: size not aligned\n", h);
		return -EINVAL;
	}
#endif
	mutex_lock(&osd_lock);
	a.canvas_w = b->a.canvas_w;
	a.canvas_h = b->a.canvas_h;
	b->a = a;
	mutex_unlock(&osd_lock);
	return 0;
}

#ifdef RGN_CHIP_XM535
/* the encoder's bitmaps: 16x16 tiles, left to right, top to bottom */
static int osd_bitmap_tiled(struct osd_block *b, const u8 __user *src,
			    u32 len)
{
	u32 bpp = b->a.fmt == FMT_COMP_1BIT ? 1 : 2, w = b->a.w, h = b->a.h;
	u32 line = bpp * 16 / 8, ty, tx, y;
	u8 __iomem *d = b->virt;
	u8 buf[4];

	if (len < bpp * w * h / 8) {
		pr_err("rgn: bitmap %u bytes, %u wanted\n", len,
		       bpp * w * h / 8);
		return -EINVAL;
	}
	if (ALIGN(w, 16) * ALIGN(h, 16) * bpp / 8 > b->size)
		return -EINVAL;
	for (ty = 0; ty < h; ty += 16)
		for (tx = 0; tx < w; tx += 16) {
			u32 n = min(w - tx, 16u) * bpp / 8;

			for (y = ty; y < ty + 16; y++, d += line) {
				memset(buf, 0, sizeof(buf));
				if (y < h &&
				    copy_from_user(buf, src + bpp * (w * y + tx) / 8, n))
					return -EFAULT;
				memcpy_toio(d, buf, line);
			}
		}
	return 0;
}
#endif

static int osd_bitmap(u32 h, const u32 *w)
{
	struct osd_block *b = &osd[h];
	const u8 __user *src = (const u8 __user *)(uintptr_t)w[2];
	u32 len = w[3];
	int ret = 0;

	if (!b->created)
		return -ENOENT;
	mutex_lock(&osd_lock);
#ifdef RGN_CHIP_XM535
	if (comp_fmt(b->a.fmt)) {
		ret = osd_bitmap_tiled(b, src, len);
		len = b->size;
	} else
#endif
	{
		len = min(len, b->size);
		if (copy_from_user((void __force *)b->virt, src, len))
			ret = -EFAULT;
	}
	if (!ret) {
		__cpuc_flush_dcache_area((void __force *)b->virt, len);
		outer_flush_range(b->phys, b->phys + len);
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

/* user space's pictures for the engine: physical memory outside the
 * kernel's RAM (its MMZ), len bytes from addr */
static bool user_phys(u32 addr, u64 len)
{
	u64 ram0 = PHYS_OFFSET, ram1 = __pa(high_memory - 1) + 1;

	return len && addr + len <= 0x100000000ull &&
	       (addr + len <= ram0 || addr >= ram1);
}

/* {w, h, src, dst}: both NV12 pictures of w x h (blend) */
static bool user_pic(const u32 *pic, u32 dst_mul2)
{
	u64 n = (u64)pic[0] * pic[1];

	return pic[0] && pic[1] && pic[0] <= 8192 && pic[1] <= 8192 &&
	       user_phys(pic[2], n * 3 / 2) && user_phys(pic[3], n * dst_mul2 / 2);
}

/* a single overlay blended into user space's picture w[2..5] */
static int osd_usr_process(u32 h, const u32 *pic)
{
	struct osd_block *b = &osd[h];
	int ret = 0;

	if (!b->created)
		return -ENOENT;
	if (!user_pic(pic, 3))
		return -EINVAL;
	mutex_lock(&osd_lock);
	if (fits(b, pic)) {
		region(pic, &b->a, b->phys, 0);
		ret = run(1, HZ / 5);
		if (ret)
			pr_err("osd: timeout\n");
	} else {
		pr_err("rgn: handle %u (%u, %u %ux%u) outside %ux%u\n", h, b->a.x,
		       b->a.y, b->a.w, b->a.h, pic[0], pic[1]);
	}
	mutex_unlock(&osd_lock);
	return ret;
}

static int osd_convert(const u32 *pic)
{
	struct osd_attr a = {.fmt = FMT_CONVERT};
	int ret;

	/* NV12 in, RGB888 planar (3 bytes a pixel) out */
	if (!user_pic(pic, 6))
		return -EINVAL;
	mutex_lock(&osd_lock);
	region(pic, &a, 0, 0);
	ret = run(1, 20 * HZ);
	mutex_unlock(&osd_lock);
	if (ret)
		pr_err("osd: convert %ux%u timeout\n", pic[0], pic[1]);
	return ret;
}

/* the slot table of a cover's module, dev, chn */
static int *win_table(u32 mod, u32 dev, u32 chn)
{
	if (mod == MOD_VI && chn < VI_CHNS)
		return win_vi[chn];
#ifdef HAVE_VPSS
	if (mod == MOD_VPSS && dev < VPSS_DEVS && chn < 3)
		return win_vpss[dev * 3 + chn];
#endif
#ifdef RGN_CHIP_XM535
	if (mod == MOD_VENC && chn < VENC_CHNS)
		return win_venc[chn];
#endif
	return NULL;
}

/* a cover to its module's slot idx (VENC's covers are comp_osd_process's) */
static void win_apply(const struct win_block *b, u32 mod, u32 dev, u32 chn,
		      int idx, bool on)
{
	struct win_attr a = {0};

	if (on)
		a = (struct win_attr){1, b->x, b->y, b->w, b->h, b->color, b->type,
				      b->thick};
	if (mod == MOD_VI)
		vi_set_chn_win(dev, chn, idx, &a);
#ifdef HAVE_VPSS
	if (mod == MOD_VPSS)
		vpss_set_chn_win(dev, chn, idx, &a);
#endif
}

static int win_ioctl(unsigned int cmd, u32 h, const u32 *w)
{
	struct win_block *b = &win[h];
	int i, *t, ret = 0;

	mutex_lock(&win_lock);
	if (cmd != RGN_CREATE && !b->created) {
		ret = -ENOENT;
		goto out;
	}
	switch (cmd) {
	case RGN_CREATE:
	case RGN_SET_ATTR:
		if (cmd == RGN_CREATE && b->created) {
			ret = -EEXIST;
			break;
		}
		b->created = 1;
		memcpy(&b->x, w + 2, 7 * sizeof(u32));
		if (cmd == RGN_SET_ATTR && b->attached) {
			t = win_table(b->mod, b->dev, b->chn);
			for (i = 0; t && i < WIN_PER_CHN; i++)
				if (t[i] == (int)h)
					win_apply(b, b->mod, b->dev, b->chn, i, true);
		}
		break;
	case RGN_DESTROY:
		if (b->attached)
			ret = -EBUSY;
		else
			memset(b, 0, sizeof(*b));
		break;
	case RGN_ATTACH:
	case RGN_DETACH:
		t = win_table(w[2], w[3], w[4]);
		if (!t || (cmd == RGN_ATTACH && b->attached)) {
			ret = -EINVAL;
			break;
		}
		for (i = 0; i < WIN_PER_CHN; i++)
			if (t[i] == (cmd == RGN_ATTACH ? -1 : (int)h))
				break;
		if (i == WIN_PER_CHN) {
			ret = cmd == RGN_ATTACH ? -EBUSY : -ENOENT;
			break;
		}
		t[i] = cmd == RGN_ATTACH ? (int)h : -1;
		b->attached = cmd == RGN_ATTACH;
		b->mod = b->attached ? w[2] : 0;
		b->dev = b->attached ? w[3] : 0;
		b->chn = b->attached ? w[4] : 0;
		win_apply(b, w[2], w[3], w[4], i, b->attached);
		break;
	default:
		ret = -ENOTTY;
	}
out:
	mutex_unlock(&win_lock);
	return ret;
}

#ifdef HAVE_LINES
/*
 * A line as the hardware draws it: from the upper point, mode 1 (shallow,
 * right), 2 (steep, right), 3 (steep, left; vertical), 4 (shallow, left),
 * its length in steps and the other axis' 32.32 increment a step.
 */
static void line_compute(s32 x0, s32 y0, s32 x1, s32 y1, u32 *mode, u32 *len,
			 u32 *step)
{
	s32 dx, dy;

	if (y1 < y0) {
		swap(x0, x1);
		swap(y0, y1);
	}
	dx = x1 - x0;
	dy = y1 - y0;
	*step = 0;
	if (!dx) {
		*mode = 3;
		*len = dy;
	} else if (!dy) {
		*mode = dx > 0 ? 1 : 4;
		*len = abs(dx);
	} else if (dx > 0) {
		if (dy < dx) {
			*mode = 1;
			*len = dx + 1;
			*step = div64_s64((s64)dy << 32, dx);
			return;
		}
		*mode = 2;
		*len = dy + 1;
		*step = div64_s64((s64)dx << 32, dy);
	} else {
		if (-dx < dy) {
			*mode = 3;
			*len = dy + 1;
			*step = div64_s64((s64)-dx << 32, dy);
			return;
		}
		*mode = 4;
		*len = 1 - dx;
		*step = div64_s64((s64)dy << 32, -dx);
	}
	/* a whole step (45 degrees) */
	if (!*step)
		*step = ~0u;
}

static int *line_table(u32 mod, u32 dev, u32 chn)
{
	if (mod == MOD_VI && chn < LINE_VI_CHNS)
		return line_vi[chn];
	if (mod == MOD_VPSS && dev < VPSS_DEVS)
		return line_vpss[dev];
	return NULL;
}

static void line_apply(const struct line_block *b, u32 mod, u32 dev, u32 chn,
		       int idx, bool on)
{
	struct line_attr a = {0};

	if (on) {
		a.en = 1;
		a.x0 = b->x0;
		a.y0 = b->y0;
		a.thick = b->thick;
		a.color = b->color;
		line_compute(b->x0, b->y0, b->x1, b->y1, &a.mode, &a.len, &a.step);
	}
	if (mod == MOD_VI)
		vi_set_chn_line(dev, chn, idx, &a);
	else
		vpss_set_grp_line(dev, idx, &a);
}

static int line_ioctl(unsigned int cmd, u32 h, const u32 *w)
{
	struct line_block *b = &lines[h];
	int i, *t, ret = 0;

	mutex_lock(&line_lock);
	if (cmd != RGN_CREATE && !b->created) {
		ret = -ENOENT;
		goto out;
	}
	switch (cmd) {
	case RGN_CREATE:
	case RGN_SET_ATTR:
		if (cmd == RGN_CREATE && b->created) {
			ret = -EEXIST;
			break;
		}
		b->created = 1;
		memcpy(&b->x0, w + 2, 6 * sizeof(u32));
		if (cmd == RGN_SET_ATTR && b->attached) {
			t = line_table(b->mod, b->dev, b->chn);
			for (i = 0; t && i < LINE_PER_CHN; i++)
				if (t[i] == (int)h)
					line_apply(b, b->mod, b->dev, b->chn, i, true);
		}
		break;
	case RGN_DESTROY:
		if (b->attached)
			ret = -EBUSY;
		else
			memset(b, 0, sizeof(*b));
		break;
	case RGN_ATTACH:
	case RGN_DETACH:
		t = line_table(w[2], w[3], w[4]);
		if (!t || (cmd == RGN_ATTACH && b->attached)) {
			ret = -EINVAL;
			break;
		}
		for (i = 0; i < LINE_PER_CHN; i++)
			if (t[i] == (cmd == RGN_ATTACH ? -1 : (int)h))
				break;
		if (i == LINE_PER_CHN) {
			ret = cmd == RGN_ATTACH ? -EBUSY : -ENOENT;
			break;
		}
		t[i] = cmd == RGN_ATTACH ? (int)h : -1;
		b->attached = cmd == RGN_ATTACH;
		b->mod = b->attached ? w[2] : 0;
		b->dev = b->attached ? w[3] : 0;
		b->chn = b->attached ? w[4] : 0;
		line_apply(b, w[2], w[3], w[4], i, b->attached);
		break;
	default:
		ret = -ENOTTY;
	}
out:
	mutex_unlock(&line_lock);
	return ret;
}
#endif

static void rgn_reset(void)
{
	int h;

	mutex_lock(&osd_lock);
	for (h = 0; h < N_HANDLE; h++)
		if (osd[h].created)
			osd_free(&osd[h]);
	mutex_lock(&win_lock);
	memset(win, 0, sizeof(win));
#ifdef HAVE_LINES
	mutex_lock(&line_lock);
	memset(lines, 0, sizeof(lines));
#endif
	tables_reset();
	calls = runs = timeouts = 0;
#ifdef HAVE_LINES
	mutex_unlock(&line_lock);
#endif
	mutex_unlock(&win_lock);
	engine_reset();
	mutex_unlock(&osd_lock);
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
		mutex_lock(&osd_lock);
		if (b->attached) {
			mutex_unlock(&osd_lock);
			pr_err("rgn: handle %u still attached\n", h);
			return -EBUSY;
		}
		osd_free(b);
		mutex_unlock(&osd_lock);
		return 0;
	case RGN_SET_ATTR:
		return osd_set(h, w + 2);
	case RGN_BITMAP:
		return osd_bitmap(h, w);
	case RGN_ATTACH:
	case RGN_DETACH:
		return osd_attach(h, w, cmd == RGN_ATTACH);
	}
	return -ENOTTY;
}

static long rgn_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct rgn_arg a;

	if (cmd == RGN_RESET) {
		rgn_reset();
		return 0;
	}
	if (_IOC_TYPE(cmd) != 'R' || _IOC_SIZE(cmd) != sizeof(a))
		return -ENOTTY;
	if (copy_from_user(&a, (void __user *)arg, sizeof(a))) {
		pr_err("rgn: copy_from_user failed\n");
		return -EFAULT;
	}
	if (cmd == RGN_CONVERT)
		return osd_convert(a.w + 2);
	if (a.w[0] >= N_HANDLE)
		return -EINVAL;
	if (cmd == RGN_USR_PROCESS)
		return osd_usr_process(a.w[0], a.w + 2);
	switch (a.w[1]) {
	case 0:
		return osd_ioctl(cmd, a.w[0], a.w);
	case 1:
		return cmd == RGN_BITMAP ? -EINVAL : win_ioctl(cmd, a.w[0], a.w);
#ifdef HAVE_LINES
	case 2:
		return cmd == RGN_BITMAP ? -EINVAL : line_ioctl(cmd, a.w[0], a.w);
#endif
	}
	return -EINVAL;
}

static int rgn_show(struct seq_file *s, void *v)
{
	int h;

	seq_puts(s, "[RGN] openxiongmai xm_v200_rgn\n\n");
	seq_puts(s, "-----REGION CHN STATUS OF OVERLAY---------------------------------------------\n");
	seq_puts(s, "Handle  En  X     Y     W     H     Type  Alpha  Up  Addr     Mod  Dev  Chn\n");
	for (h = 0; h < N_HANDLE; h++) {
		const struct osd_block *b = &osd[h];

		if (b->created)
			seq_printf(s, "%-6d  %-2s  %-4u  %-4u  %-4u  %-4u  %-4u  %-5x  %-2x  %-8X  %-3u  %-3u  %-3u\n",
				   h, b->attached ? "Y" : "N", b->a.x, b->a.y, b->a.w,
				   b->a.h, b->a.fmt, b->a.alpha, b->a.scale, b->phys,
				   b->mod, b->dev, b->chn);
	}
	seq_puts(s, "-----REGION CHN STATUS OF COVER-----------------------------------------------\n");
	seq_puts(s, "Handle  En  X     Y     W     H     Color   Type  Thick  Mod  Dev  Chn\n");
	for (h = 0; h < N_HANDLE; h++) {
		const struct win_block *b = &win[h];

		if (b->created)
			seq_printf(s, "%-6d  %-2s  %-4u  %-4u  %-4u  %-4u  %-6X  %-4u  %-5u  %-3u  %-3u  %-3u\n",
				   h, b->attached ? "Y" : "N", b->x, b->y, b->w, b->h,
				   b->color, b->type, b->thick, b->mod, b->dev, b->chn);
	}
#ifdef HAVE_LINES
	seq_puts(s, "-----REGION CHN STATUS OF LINE------------------------------------------------\n");
	seq_puts(s, "Handle  En  StartX  StartY  EndX  EndY  Color   Thick  Mod  Dev  Chn\n");
	for (h = 0; h < N_HANDLE; h++) {
		const struct line_block *b = &lines[h];

		if (b->created)
			seq_printf(s, "%-6d  %-2s  %-6u  %-6u  %-4u  %-4u  %-6X  %-5u  %-3u  %-3u  %-3u\n",
				   h, b->attached ? "Y" : "N", b->x0, b->y0, b->x1,
				   b->y1, b->color, b->thick, b->mod, b->dev, b->chn);
	}
#endif
	seq_puts(s, "-----OSD ENGINE----------------------------------------------------------------\n");
	seq_puts(s, "Process  Runs     Timeouts\n");
	seq_printf(s, "%-7u  %-7u  %-8u\n", calls, runs, timeouts);
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

static int __init xm_v200_rgn_init(void)
{
	int ret = -ENOMEM;

	tables_reset();
	regs = ioremap(OSD_BASE, 0x1000);
	if (!regs) {
		pr_err("rgn: ioremap failed\n");
		return -ENOMEM;
	}
#ifdef RGN_CHIP_XM535
	comp = ioremap(COMP_BASE, 0x1000);
	if (!comp) {
		pr_err("rgn: ioremap failed\n");
		goto unmap;
	}
#endif
	ret = request_irq(OSD_IRQ, osd_int, 0, "osd", NULL);
	if (ret) {
		pr_err("rgn: request_irq failed\n");
		goto unmap;
	}
	ret = misc_register(&rgn_misc);
	if (ret) {
		pr_err("rgn: register device failed!\n");
		free_irq(OSD_IRQ, NULL);
		goto unmap;
	}
	if (!proc_create("umap/rgn", 0444, NULL, &rgn_proc_fops))
		pr_err("rgn: create rgn proc fail!\n");
	pr_info("rgn: openxiongmai xm_v200_rgn init ok\n");
	return 0;
unmap:
#ifdef RGN_CHIP_XM535
	if (comp)
		iounmap(comp);
#endif
	iounmap(regs);
	return ret;
}

static void __exit xm_v200_rgn_exit(void)
{
	int h;

	remove_proc_entry("umap/rgn", NULL);
	misc_deregister(&rgn_misc);
	free_irq(OSD_IRQ, NULL);
	for (h = 0; h < N_HANDLE; h++)
		if (osd[h].created)
			osd_free(&osd[h]);
#ifdef RGN_CHIP_XM535
	iounmap(comp);
#endif
	iounmap(regs);
}

module_init(xm_v200_rgn_init);
module_exit(xm_v200_rgn_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("V200-series regions (OSD, covers, lines), compatible with rgn.ko");
