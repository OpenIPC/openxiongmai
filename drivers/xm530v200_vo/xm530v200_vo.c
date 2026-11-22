// SPDX-License-Identifier: GPL-2.0
/*
 * The XM530V200's video output (/dev/vo): a drop-in for the vendor vo.ko.
 * One display device (BT656, LCD or MIPI timing) with a video layer of six
 * channels, an image (OSD) layer, a cursor layer and colour controls. Each
 * channel has three NV12 buffers in the video layer's MMZ block, shared with
 * vpss.ko through vo_get_buf()/vo_put_buf() as V200 vb descriptors; the
 * display interrupt (IRQ 59, once a frame) shows the oldest filled one.
 *
 * ioctls take a 64-byte {dev, chn, ...} block (dev 0 only):
 *
 *   0x4f00      forget everything
 *   0x40404f01  device: background, interface (8 BT656, 0x40 LCD, 0x80
 *               MIPI), sync (vo_sync_name[]; 1000 user timing), the user
 *               timing
 *   0x40404f02  channel: x, y, w, h, priority (its buffers from the layer)
 *   0x40404f03  channel on,  0x40404f04 off
 *   0x40404f05  video layer: w, h, x, y, display w, h (allocates its MMZ)
 *   0x40404f06  image layer: format, address, w, h, x, y, display w, h,
 *               effect;  0x40404f07 on, 0x40404f08 off
 *   0x40404f09  cursor: format, x, y, w, h, effect;  0x40404f0a on,
 *               0x40404f0b off, 0x40404f0c its picture {user pointer, bytes}
 *   0x40404f0d  channel shown,  0x40404f0e hidden
 *   0x40404f0f  the layer's channels forgotten (all off and hidden first)
 *   0x40404f10  the channel's buffers black
 *   0x40404f11  colour: luma, saturation, contrast, hue (0..100, 50 neutral)
 *   0x40404f12  a channel buffer for user space;  0x40404f13 shown
 *
 * Registers at 0x31000000 (0x40000), word index: 0 the update strobe, 3
 * background, 4+chn the channel's shown buffer, 0x10+chn*0x20.. the
 * channels, 0x1000.. timing, 0x1011 the frame interrupt, 0x1800.. image
 * layer, 0x2400.. cursor (its picture at byte 0xa000), 0x4000.. hue,
 * 0x4400.. luma/saturation, 0x4800.. the contrast curve, 0x5000.. BT656,
 * 0x5400.. LCD, 0xc000.. clocks and reset. lcd_rgb=R,G,B (pins per colour),
 * lcd_pwm= and lcd_disp= as vo.ko's: the LCD's pins at load.
 */

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/sched.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>

#define VO_BASE 0x31000000
#define VO_SIZE 0x40000
#define VO_IRQ 59
#define MUX_BASE 0x10020000 /* a pin's function at +4 * pin */
#define N_CHN 6
#define N_BUF 3

#define INTF_BT656 0x08
#define INTF_LCD 0x40
#define INTF_MIPI 0x80
#define SYNC_USER 1000

#define BUF_FREE 0
#define BUF_VPSS 1 /* taken to fill */
#define BUF_FULL 2
#define BUF_SHOWN 3

#define VO_IOC(n) (0x40404f00 | (n))

/* A channel buffer: a V200 vb descriptor (phys 4, size 5, stride 6,
 * w 7, h 8, pts 10/11), then vo.ko's own state */
struct vo_buf {
	u32 w[16];
	u32 state;
	u32 frame; /* the frame it was last shown in */
};

struct vo_chn {
	u32 attr[7]; /* stride, x, y, w, h, -, priority */
	u32 configured, enabled, shown;
	struct vo_buf buf[N_BUF];
	u32 cur; /* the buffer shown */
	u32 pad;
};

static struct {
	spinlock_t lock;
	u32 tot_frm, dup_frm;
	u32 pad0;
	u32 recv_frm[N_CHN];
	u32 dev_attr[11]; /* bg, intf, sync, the user timing */
	u32 vl_attr[6]; /* w, h, x, y, disp w, disp h */
	u32 img_attr[9]; /* format, addr, w, h, x, y, disp w, disp h, effect */
	u32 cur_attr[6]; /* format, x, y, w, h, effect */
	u32 vl_phys;
	void __iomem *vl_virt;
	u32 vl_size, vl_used;
	u32 pad1;
	struct vo_chn chn[N_CHN];
	u32 img_en, cur_en;
	u32 frame; /* frame interrupts */
} vo;

/* mmz.ko */
int mmz_alloc(const char *name, u32 size, u32 *phys);
int mmz_get_addr(const char *name, u32 *size, u32 *phys);

static u32 vo_csc[4]; /* luma, saturation, contrast, hue */
static void __iomem *regs;
static int vo_dev_open;
static struct proc_dir_entry *proc;

static int lcd_rgb[3];
static int lcd_rgb_n;
module_param_array(lcd_rgb, int, &lcd_rgb_n, 0444);
MODULE_PARM_DESC(lcd_rgb, "Enabled RGB pin, eg: 6,6,6: R7~R2,G7~G2,B7~B2.");
static int lcd_pwm;
module_param(lcd_pwm, int, 0444);
MODULE_PARM_DESC(lcd_pwm, "LCD backlight enable pin. If set, pull up.");
static int lcd_disp;
module_param(lcd_disp, int, 0444);
MODULE_PARM_DESC(lcd_disp, "LCD display enable pin. If set, pull up.");

/* The LCD's data pins, R7..R0, G7..G0, B7..B0 */
static const int lcd_pins[3][8] = {
	{ 71, 70, 69, 68, 67, 66, 65, 64 },
	{ 79, 78, 77, 76, 75, 74, 73, 72 },
	{ 87, 86, 85, 84, 83, 82, 81, 80 },
};

/* round(256 * sin(d)), d = 0..180 degrees */
static const int sintab[181] = {
	0, 4, 9, 13, 18, 22, 27, 31, 36, 40, 44, 49, 53, 58, 62, 66, 71, 75,
	79, 83, 88, 92, 96, 100, 104, 108, 112, 116, 120, 124, 128, 132, 136,
	139, 143, 147, 150, 154, 158, 161, 165, 168, 171, 175, 178, 181, 184,
	187, 190, 193, 196, 199, 202, 204, 207, 210, 212, 215, 217, 219, 222,
	224, 226, 228, 230, 232, 234, 236, 237, 239, 241, 242, 243, 245, 246,
	247, 248, 249, 250, 251, 252, 253, 254, 254, 255, 255, 255, 256, 256,
	256, 256, 256, 256, 256, 255, 255, 255, 254, 254, 253, 252, 251, 250,
	249, 248, 247, 246, 245, 243, 242, 241, 239, 237, 236, 234, 232, 230,
	228, 226, 224, 222, 219, 217, 215, 212, 210, 207, 204, 202, 199, 196,
	193, 190, 187, 184, 181, 178, 175, 171, 168, 165, 161, 158, 154, 150,
	147, 143, 139, 136, 132, 128, 124, 120, 116, 112, 108, 104, 100, 96,
	92, 88, 83, 79, 75, 71, 66, 62, 58, 53, 49, 44, 40, 36, 31, 27, 22, 18,
	13, 9, 4, 0,
};

static const char *const vo_sync_name[] = {
	"PAL", "NTSC", "960H_PAL", "960H_NTSC", "1080P24", "1080P25",
	"1080P30", "720P50", "720P60", "720P30", "720P15", "1080P50",
	"1080P60", "576P50", "480P60", "720P25", "800x600_60", "1024x768_60",
	"1280x1024_60", "1366x768_60", "1440x900_60", "1280x800_60",
	"1680x1050_60", "1920x2160_30", "1600x1200_60", "1920x1200_60",
	"2560x1440_30", "2560x1600_60", "3840x2160_30", "480x272_60_S",
	"480x272_60", "800x480_60", "1024x600_60", "480x320_60", "480x640_60",
	"USER", "BUTT",
};

static inline void wr(u32 idx, u32 v)
{
	writel(v, regs + idx * 4);
}

static inline u32 rd(u32 idx)
{
	return readl(regs + idx * 4);
}

static void mux(int pin, u32 v)
{
	void __iomem *p = ioremap(MUX_BASE + pin * 4, 4);

	if (p) {
		writel(v, p);
		iounmap(p);
	}
}

static int sin_deg(int d)
{
	return d < 0 ? -sintab[-d] : sintab[d];
}

/* The contrast curve: 65 points, 50 a straight line */
static void vo_con(int c)
{
	u32 lut[65];
	int k;

	memset(lut, 0, sizeof(lut));
	if (c < 50) {
		u32 u = 0x20, v;

		for (k = 0; k <= 64; k++) {
			v = (50 - c) * 2 + (u >> 6);
			u += (50 - c) * -4 + 0xff;
			lut[k] = v > 0xfe ? 0xff : v;
		}
	} else {
		u32 v;

		if (c > 99)
			c = 100;
		c -= 50;
		for (k = 0; k <= 64; k++) {
			v = 0;
			if (4 * k >= 2 * c) {
				v = (u32)(k * 0x3fc - c * 0x1fe) / (0xff - c * 4);
				if (v > 0xfe)
					v = 0xff;
			}
			lut[k] = v;
		}
	}
	for (k = 0; k <= 64; k++)
		writel(lut[k], regs + 0x12000 + k * 4);
}

static void buf_black(struct vo_buf *b, u32 n_y)
{
	void __iomem *p = vo.vl_virt + (b->w[4] - vo.vl_phys);

	if (!n_y || !vo.vl_virt)
		return;
	memset_io(p, 0x10, n_y);
	if (n_y / 2)
		memset_io(p + n_y, 0x80, n_y / 2);
}

int vo_get_buf(u32 dev, u32 chn, void **buf)
{
	u64 best = ~0ull, pts;
	unsigned long flags;
	struct vo_chn *c;
	int i, pick = -1;

	if (dev || chn >= N_CHN)
		return -1;
	c = &vo.chn[chn];
	spin_lock_irqsave(&vo.lock, flags);
	if (c->enabled) {
		*buf = NULL;
		for (i = 0; i < N_BUF; i++) {
			struct vo_buf *b = &c->buf[i];

			if (b->state != BUF_FREE)
				continue;
			/* just taken off the screen while the scan is still in
			 * the frame's second half */
			if (b->frame == vo.frame && rd(0x1008) <= rd(0x100a))
				continue;
			pts = (u64)b->w[11] << 32 | b->w[10];
			if (pts < best) {
				best = pts;
				pick = i;
			}
		}
		if (pick >= 0) {
			*buf = &c->buf[pick];
			c->buf[pick].state = BUF_VPSS;
			spin_unlock_irqrestore(&vo.lock, flags);
			return 0;
		}
	}
	spin_unlock_irqrestore(&vo.lock, flags);
	return -1;
}
EXPORT_SYMBOL(vo_get_buf);

void clk_get_pts(u64 *pts);

int vo_put_buf(u32 dev, u32 chn, void *buf, int show)
{
	unsigned long flags;
	struct vo_buf *b;
	int i;
	u64 pts;

	if (dev || chn >= N_CHN)
		return -1;
	spin_lock_irqsave(&vo.lock, flags);
	for (i = 0; i < N_BUF; i++)
		if (buf == &vo.chn[chn].buf[i])
			break;
	if (i == N_BUF) {
		spin_unlock_irqrestore(&vo.lock, flags);
		return -1;
	}
	b = &vo.chn[chn].buf[i];
	if (!show) {
		b->state = BUF_FREE;
	} else {
		b->state = BUF_FULL;
		clk_get_pts(&pts);
		b->w[10] = (u32)pts;
		b->w[11] = (u32)(pts >> 32);
	}
	spin_unlock_irqrestore(&vo.lock, flags);
	return 0;
}
EXPORT_SYMBOL(vo_put_buf);

static u64 buf_pts(struct vo_buf *b)
{
	return (u64)b->w[11] << 32 | b->w[10];
}

/*
 * Once a frame: each channel moves on to the older of its next two filled
 * buffers. (vo.ko compares buffers 1 and 2 whichever is on the screen.)
 */
static irqreturn_t vo_display_int(int irq, void *dev)
{
	unsigned long flags;
	u32 shown = 0;
	int n;

	spin_lock_irqsave(&vo.lock, flags);
	if (!rd(0x1011)) {
		spin_unlock_irqrestore(&vo.lock, flags);
		return IRQ_NONE;
	}
	vo.frame++;
	wr(0x1011, 0);
	for (n = 0; n < N_CHN; n++) {
		struct vo_chn *c = &vo.chn[n];
		struct vo_buf *a, *b;
		int step = 0;

		if (!c->enabled)
			continue;
		a = &c->buf[(c->cur + 1) % N_BUF];
		b = &c->buf[(c->cur + 2) % N_BUF];
		if (a->state == BUF_FULL && buf_pts(a) != ~0ull)
			step = (b->state == BUF_FULL && buf_pts(b) < buf_pts(a)) ? 2 : 1;
		else if (b->state == BUF_FULL && buf_pts(b) != ~0ull)
			step = 2;
		if (!step)
			continue;
		c->buf[c->cur].state = BUF_FREE;
		c->buf[c->cur].frame = vo.frame;
		c->cur = (c->cur + step) % N_BUF;
		c->buf[c->cur].state = BUF_SHOWN;
		wr(4 + n, c->cur);
		wr(0, 1);
		shown |= 1 << n;
		vo.recv_frm[n]++;
	}
	vo.tot_frm++;
	if (!shown)
		vo.dup_frm++;
	spin_unlock_irqrestore(&vo.lock, flags);
	return IRQ_HANDLED;
}

/* Wait for vpss.ko to give back the buffers it holds: 0 or -1 */
static int wait_vpss_bufs(bool layer_reset)
{
	int n, i, tries = 0;

	for (n = 0; n < N_CHN; n++) {
		struct vo_chn *c = &vo.chn[n];

		if (c->enabled || c->shown) {
			if (layer_reset)
				pr_err("vo: reset video layer error dev%d chn %d enable %d show %d\n",
				       0, n, c->enabled, c->shown);
			else
				pr_err("vo: reset video layer error dev%d chn %d enable\n",
				       0, n);
			return -1;
		}
		for (i = 0; i < N_BUF; i++)
			while (c->buf[i].state == BUF_VPSS) {
				if (schedule_timeout_interruptible(msecs_to_jiffies(10)))
					return -1;
				if (tries++ > 200)
					return -1;
			}
	}
	return 0;
}

static void chn_clear(void)
{
	memset(vo.chn, 0, sizeof(vo.chn));
}

static int vo_set_video_layer_attr(u32 *a)
{
	u32 w = a[0], h = a[1], dw = a[4], dh = a[5];
	char name[32];
	unsigned long flags;
	u32 phys = 0, size = 0;
	void __iomem *virt = NULL;

	if (!dw || !dh || w > 0x1000 || (w != dw && w > 0xa80)) {
		pr_err("vo: set_video_layer_attr error [%d, %d][%d, %d, %d, %d]\n",
		       w, h, a[2], a[3], dw, dh);
		return -1;
	}
	snprintf(name, sizeof(name), "vo_v_layer[%d]", 0);
	if (!vo.vl_size) {
		u32 s = ALIGN(w, 64) * h * 3;

		size = s + s / 2;
		if (mmz_alloc(name, size, &phys)) {
			pr_err("vo: can't malloc memory\n");
			return -1;
		}
		virt = ioremap(phys, size);
	} else {
		if (vo.vl_size < (h * w * 3 >> 1) * 3) {
			pr_err("vo: too large size\n");
			return -1;
		}
		/* (vo.ko looks up an empty name here, and fails) */
		if (mmz_get_addr(name, &size, &phys)) {
			pr_err("vo: mmz_get_addr %s error\n", name);
			return -1;
		}
	}
	if (wait_vpss_bufs(false))
		return -1;
	spin_lock_irqsave(&vo.lock, flags);
	memcpy(vo.vl_attr, a, sizeof(vo.vl_attr));
	if (!vo.vl_size) {
		vo.vl_phys = phys;
		vo.vl_size = size;
		vo.vl_virt = virt;
	}
	vo.vl_used = 0;
	chn_clear();
	wr(0x1008, rd(0x1005) >> 1); /* the line the buffers change after */
	wr(0x1010, 1);
	spin_unlock_irqrestore(&vo.lock, flags);
	return 0;
}

static int vo_set_chn_attr(u32 chn, u32 *a)
{
	struct vo_chn *c = &vo.chn[chn];
	u32 stride = ALIGN(a[3], 16), h = a[4], size, at[7];
	unsigned long flags;
	int i;

	at[0] = stride;
	memcpy(&at[1], &a[1], 6 * 4);
	spin_lock_irqsave(&vo.lock, flags);
	if (!memcmp(c->attr, at, sizeof(at))) {
		spin_unlock_irqrestore(&vo.lock, flags);
		return 0;
	}
	size = stride * h * 3 >> 1;
	if (!c->configured) {
		u32 p;

		if (vo.vl_size - vo.vl_used < size * 3) {
			spin_unlock_irqrestore(&vo.lock, flags);
			pr_err("vo: chn %d out of buf\n", chn);
			return -1;
		}
		p = vo.vl_phys + vo.vl_used;
		for (i = 0; i < N_BUF; i++, p += size) {
			c->buf[i].w[4] = p;
			c->buf[i].w[5] = size;
			c->buf[i].w[6] = stride;
			c->buf[i].w[8] = h;
		}
		vo.vl_used += size * 3;
	} else if (c->buf[0].w[5] < size) {
		spin_unlock_irqrestore(&vo.lock, flags);
		pr_err("vo: chn %d need reset\n", chn);
		return -1;
	}
	for (i = 0; i < N_BUF; i++)
		buf_black(&c->buf[i], stride * h);
	c->configured = 1;
	memcpy(c->attr, at, sizeof(at));
	wr(chn * 0x20 + 0x1d, 0);
	wr(chn * 0x20 + 0x17, stride);
	wr(chn * 0x20 + 0x16, stride);
	wr(chn * 0x20 + 0x18, a[1]);
	wr(chn * 0x20 + 0x1a, a[2]);
	wr(chn * 0x20 + 0x19, a[1] + a[3] - 1);
	wr(chn * 0x20 + 0x1b, a[2] + h - 1);
	wr(chn * 0x20 + 0x1f, stride * h);
	for (i = 0; i < N_BUF; i++) {
		wr(chn * 0x20 + 0x10 + i, c->buf[i].w[4]);
		wr(chn * 0x20 + 0x13 + i, c->buf[i].w[4] + stride * h);
	}
	wr(4 + chn, 0);
	spin_unlock_irqrestore(&vo.lock, flags);
	return 0;
}

static void lcd_timing(u32 r5400, u32 r540d, u32 r540e, u32 r540f, u32 r5410)
{
	wr(0x5400, r5400);
	wr(0x540d, r540d);
	wr(0x540e, r540e);
	wr(0x540f, r540f);
	wr(0x5410, r5410);
	wr(0x5411, 0);
	wr(0x5417, 0);
	wr(0x5418, 0);
}

static void bt656_timing(u32 a, u32 b, u32 c, u32 d)
{
	wr(0x5000, a);
	wr(0x5001, b);
	wr(0x5002, c);
	wr(0x5003, d);
	wr(0x500f, 0x5f);
	wr(0x5014, 3);
}

static void frame_timing(u32 htot, u32 vtot, u32 hact, u32 vact)
{
	wr(0x1000, htot);
	wr(0x1001, vtot);
	wr(0x1002, 0);
	wr(0x1004, 0);
	wr(0x1003, hact);
	wr(0x1005, vact);
}

/* a[4..14]: bg, intf, sync, the user timing */
static int vo_set_dev_attr(u32 *a)
{
	u32 intf = a[5], sync = a[6];
	bool bt = intf == INTF_BT656;
	unsigned long flags;

	wr(0xc008, 0);
	udelay(50);
	wr(0xc008, 1);
	udelay(50);
	spin_lock_irqsave(&vo.lock, flags);
	memcpy(vo.dev_attr, &a[4], sizeof(vo.dev_attr));
	wr(0x1006, 0);
	wr(0xc000, 0);
	wr(0xc001, 0);
	switch (sync) {
	case 15: /* 720P25 */
		if (bt) {
			bt656_timing(0x27f, 0x77f, 0x1e, 1);
			wr(0x1000, 0x77f);
		} else {
			lcd_timing(0, 0x6e, 0x80, 5, 10);
			wr(0x1000, 0x7bb);
		}
		wr(0x1001, 0x2ed);
		wr(0x1002, 0);
		wr(0x1004, 0);
		wr(0x1003, 0x500);
		wr(0x1005, 0x2d0);
		break;
	case 32: /* 1024x600_60 */
		lcd_timing(0, 0xa0, 0xa0, 0x11, 0x12);
		frame_timing(0x53f, 0x28f, 0x3ff, 599);
		break;
	case SYNC_USER: {
		u32 vact = a[8] & 0xffff, vbb = a[8] >> 16;
		u32 vfb = a[9] & 0xffff, hact = a[9] >> 16;
		u32 hbb = a[10] & 0xffff, hfb = a[10] >> 16;
		u32 hpw = a[13] & 0xffff, vpw = a[13] >> 16;
		u32 idv = a[14] & 0xff, ihs = a[14] >> 8 & 0xff;
		u32 ivs = a[14] >> 16 & 0xff;

		wr(0x5400, ivs << 1 | idv << 2 | ihs);
		wr(0x540d, hfb + 1);
		wr(0x540e, hfb + hpw + 1);
		wr(0x540f, vfb);
		wr(0x5410, vfb + vpw);
		wr(0x5411, hfb + 1);
		wr(0x5417, 0);
		wr(0x5418, a[7] >> 16 & 0xff);
		frame_timing(hbb + hact + hfb - 1, vbb + vact + vfb - 1, hact,
			     vact);
		wr(0x1802, hact > 0x190 ? 0x40 : hact > 0x12c ? 0x80 : 0x100);
		break;
	}
	case 30: /* 480x272_60 */
		lcd_timing(0, 0x10, 0x32, 0x10, 0x20);
		wr(0x1802, 0x40);
		frame_timing(0x211, 0x11e, 0x1df, 0x10f);
		break;
	case 6: /* 1080P30 */
	case 5: /* 1080P25 */
		frame_timing(sync == 6 ? 0x897 : 0xa4f, 0x464, 0x780, 0x438);
		if (!bt)
			lcd_timing(0, 0x58, 0x84, 4, 9);
		else if (sync == 6)
			bt656_timing(0x117, 0x897, 0x2e, 1);
		else
			bt656_timing(0x2d0, 1, 0x30, 2);
		break;
	case 9: /* 720P30 */
		if (!bt) {
			lcd_timing(0, 0x6e, 0x80, 5, 10);
			frame_timing(0x671, 0x2ed, 0x4ff, 0x2cf);
		} else {
			bt656_timing(0x13f, 0x63f, 0x1d, 0x2ed);
			wr(0x1000, 0x63f);
			wr(0x1001, 0x2ed);
			wr(0x1002, 0);
			wr(0x1004, 0);
			wr(0x1003, 0x500);
			wr(0x1005, 0x2d0);
		}
		break;
	default:
		pr_err("VO: not support intf_sync %d\n", sync);
		break;
	}
	if (bt) {
		/* the BT656 pins */
		static const int pins[] = { 72, 73, 74, 75, 76, 77, 78, 79, 89 };
		int i;

		for (i = 0; i < ARRAY_SIZE(pins); i++)
			mux(pins[i], 2);
	}
	wr(3, a[4]);
	wr(0x1006, 1);
	wr(0, 1);
	spin_unlock_irqrestore(&vo.lock, flags);
	return 0;
}

/* Wait for reg to drop the strobe: 0, or -1 after 120 ms */
static int wait_strobe(u32 idx, u32 busy, bool eq)
{
	int n = 12;

	do {
		if (eq ? rd(idx) != busy : !rd(idx))
			return 0;
		if (schedule_timeout_interruptible(msecs_to_jiffies(10)))
			return -1;
	} while (--n);
	return -1;
}

static int vo_set_img_attr(u32 *a)
{
	u32 argb8888 = a[4];
	unsigned long flags;

	if (!a[10] || !a[11])
		return -1;
	spin_lock_irqsave(&vo.lock, flags);
	memcpy(vo.img_attr, &a[4], sizeof(vo.img_attr));
	wr(0x180e, argb8888 ? 2 : 0);
	wr(0x1805, a[6] * (argb8888 ? 4 : 2));
	wr(0x1804, a[6] / (argb8888 ? 1 : 2));
	wr(0x1803, a[7]);
	wr(0x180a, a[5]);
	wr(0x1806, a[8]);
	wr(0x1808, a[9]);
	wr(0x1807, a[8] + a[10] - 1);
	wr(0x1809, a[9] + a[11] - 1);
	wr(0x1811, 1);
	wr(0x180f, a[12] >> 8 & 0xff);
	wr(0x1810, a[12] >> 16 & 0xff);
	wr(0x1800, 1);
	spin_unlock_irqrestore(&vo.lock, flags);
	return wait_strobe(0x1800, 0, false);
}

static void vo_set_cursor_attr(u32 *a)
{
	u32 x = a[5], w = a[7], eff = a[9];
	unsigned long flags;

	spin_lock_irqsave(&vo.lock, flags);
	memcpy(vo.cur_attr, &a[4], sizeof(vo.cur_attr));
	if (rd(0x1003) - w <= x)
		x = rd(0x1003) - w;
	wr(0x240e, a[4]);
	wr(0x2404, x);
	wr(0x2406, a[6]);
	wr(0x2405, x + w - 1);
	wr(0x2407, a[6] + a[8] - 1);
	wr(0x240d, 1);
	wr(0x240b, eff >> 8 & 0xff);
	wr(0x240c, 0);
	wr(0x2409, (eff & 0xfffffff) >> 24);
	wr(0x240a, eff >> 28);
	wr(0x2400, 1);
	spin_unlock_irqrestore(&vo.lock, flags);
}

static int vo_set_cursor_pic(u32 *a)
{
	void __user *src = (void __user *)(uintptr_t)a[4];
	u32 len = a[5], i;
	u32 *p;

	if (len > 0x1000 || !src)
		return -1;
	p = kmalloc(len ? len : 1, GFP_KERNEL);
	if (!p) {
		pr_err("vo: kmalloc failed\n");
		return -1;
	}
	if (copy_from_user(p, src, len)) {
		kfree(p);
		pr_err("vo: copy_from_user error.\n");
		return -1;
	}
	wr(0x2402, 1);
	for (i = 0; i < len / 4; i++)
		writel(p[i], regs + 0xa000 + i * 4);
	if (len & 3) {
		u32 v = 0;

		memcpy(&v, (u8 *)p + (len & ~3), len & 3);
		writel(v, regs + 0xa000 + (len & ~3));
	}
	wr(0x2402, 0);
	kfree(p);
	return 0;
}

static void vo_set_csc(u32 *a)
{
	int l = a[4], s = a[5], hue = a[7], v, si, co;

	memcpy(vo_csc, &a[4], sizeof(vo_csc));
	wr(0x4400, (l << 8) / 50);
	v = (s * 0x100) / 50;
	if (hue != 50)
		v *= 2;
	if (v > 0x3fe)
		v = 0x3ff;
	wr(0x4401, v);
	wr(0x4402, v);
	vo_con(a[6]);
	v = hue * 180 / 50 - 180;
	if (v > 179)
		v = 180;
	if (v < -180)
		v = -180;
	si = sin_deg(v);
	co = v + 90 < 1 ? sin_deg(v + 90) : v + 90 < 181 ? sintab[v + 90] :
							  -sintab[270 - v];
	if (hue != 50) {
		si /= 2;
		co /= 2;
	}
	wr(0x4006, co);
	wr(0x4007, si);
	wr(0x400a, -si);
	wr(0x400b, co);
}

static long vo_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	void __user *u = (void __user *)arg;
	unsigned long flags;
	struct vo_chn *c;
	u32 a[16];
	int i, ret;

	memset(a, 0, sizeof(a));
	if (u && copy_from_user(a, u, sizeof(a))) {
		pr_err("vo_ioctl: copy_from_user error.\n");
		return -1;
	}
	if (cmd != 0x4f00 && a[0] != 0) {
		pr_err("vo: not support dev %d\n", a[0]);
		return -1;
	}
	switch (cmd) {
	case VO_IOC(0x02): case VO_IOC(0x03): case VO_IOC(0x04):
	case VO_IOC(0x0d): case VO_IOC(0x0e): case VO_IOC(0x10):
	case VO_IOC(0x12): case VO_IOC(0x13):
		if (a[1] >= N_CHN)
			return -1;
	}
	c = &vo.chn[a[1] < N_CHN ? a[1] : 0];

	switch (cmd) {
	case 0x4f00:
		spin_lock_irqsave(&vo.lock, flags);
		memset((u8 *)&vo + sizeof(vo.lock), 0, sizeof(vo) - sizeof(vo.lock));
		spin_unlock_irqrestore(&vo.lock, flags);
		return 0;
	case VO_IOC(0x01):
		return vo_set_dev_attr(a);
	case VO_IOC(0x02):
		return vo_set_chn_attr(a[1], &a[4]);
	case VO_IOC(0x03):
	case VO_IOC(0x04):
		spin_lock_irqsave(&vo.lock, flags);
		c->enabled = cmd == VO_IOC(0x03);
		spin_unlock_irqrestore(&vo.lock, flags);
		return 0;
	case VO_IOC(0x05):
		return vo_set_video_layer_attr(&a[4]);
	case VO_IOC(0x06):
		return vo_set_img_attr(a);
	case VO_IOC(0x07):
	case VO_IOC(0x08):
		spin_lock_irqsave(&vo.lock, flags);
		vo.img_en = cmd == VO_IOC(0x07);
		wr(0x1801, vo.img_en);
		wr(0x1800, 1);
		spin_unlock_irqrestore(&vo.lock, flags);
		return 0;
	case VO_IOC(0x09):
		vo_set_cursor_attr(a);
		return 0;
	case VO_IOC(0x0a):
	case VO_IOC(0x0b):
		spin_lock_irqsave(&vo.lock, flags);
		vo.cur_en = cmd == VO_IOC(0x0a);
		wr(0x2401, 1); /* as vo.ko: 1 for both */
		wr(0x1800, 1);
		spin_unlock_irqrestore(&vo.lock, flags);
		return 0;
	case VO_IOC(0x0c):
		return vo_set_cursor_pic(a);
	case VO_IOC(0x0d):
	case VO_IOC(0x0e): {
		bool show = cmd == VO_IOC(0x0d);

		if (c->shown == show)
			return 0;
		spin_lock_irqsave(&vo.lock, flags);
		if (!show || c->enabled == 1) {
			c->shown = show;
			wr(a[1] * 0x20 + 0x1c, show);
			wr(0, 1);
		}
		spin_unlock_irqrestore(&vo.lock, flags);
		ret = wait_strobe(0, 1, true);
		if (ret)
			pr_info("vo dev[%d] chn[%d] %s timeout out\n", a[0],
				a[1], show ? "show" : "hide");
		return ret;
	}
	case VO_IOC(0x0f):
		if (wait_vpss_bufs(true))
			return -1;
		spin_lock_irqsave(&vo.lock, flags);
		chn_clear();
		vo.vl_used = 0;
		spin_unlock_irqrestore(&vo.lock, flags);
		return 0;
	case VO_IOC(0x10):
		spin_lock_irqsave(&vo.lock, flags);
		for (i = 0; i < N_BUF; i++)
			if (c->buf[i].w[4] && c->buf[i].w[5])
				buf_black(&c->buf[i], c->buf[i].w[6] * c->buf[i].w[8]);
		spin_unlock_irqrestore(&vo.lock, flags);
		return 0;
	case VO_IOC(0x11):
		vo_set_csc(a);
		return 0;
	case VO_IOC(0x12): {
		struct vo_buf *b = NULL;

		ret = vo_get_buf(a[0], a[1], (void **)&b);
		if (!ret) {
			a[9] = b->w[7];
			a[4] = b->w[4];
			a[5] = a[6] = b->w[8] * b->w[7] + b->w[4];
			a[10] = b->w[8];
			a[8] = b->w[6];
			a[12] = b->w[10];
			a[13] = b->w[11];
			a[14] = (u32)(uintptr_t)b;
		} else {
			ret = -1;
		}
		if (copy_to_user(u, a, sizeof(a))) {
			pr_err("vo_ioctl: copy_to_user error.\n");
			return -1;
		}
		return ret;
	}
	case VO_IOC(0x13):
		vo_put_buf(a[0], a[1], (void *)(uintptr_t)a[14], 1);
		return 0;
	}
	pr_err("vo_ioctl: cmd error\n");
	return -1;
}

static int vo_open(struct inode *inode, struct file *file)
{
	vo_dev_open++;
	return 0;
}

static int vo_release(struct inode *inode, struct file *file)
{
	vo_dev_open--;
	return 0;
}

static const struct file_operations vo_fops = {
	.owner = THIS_MODULE,
	.open = vo_open,
	.release = vo_release,
	.unlocked_ioctl = vo_ioctl,
};

static struct miscdevice vo_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "vo",
	.fops = &vo_fops,
};

static int vo_proc_show(struct seq_file *m, void *v)
{
	u32 intf = vo.dev_attr[1], sync = vo.dev_attr[2];
	const u16 *t = (const u16 *)&vo.dev_attr[3];
	const u8 *tb = (const u8 *)&vo.dev_attr[3];
	int n;

	seq_puts(m, "[VO] BUILD TIME:[open_vo]\n\n");
	seq_puts(m, "----------------------------VO DEV ATTR------------------------------------\n");
	seq_puts(m, "DevId  DevEn  Mux1   Mux2   Mux3   InfSync       BkColor\n");
	seq_printf(m, "%-5d  %-5s  %-5s  %-5s  %-5s  %-12s  %-7x\n", 0, "Y",
		   intf & INTF_LCD ? "LCD" : " ", intf & INTF_BT656 ? "BT656" : " ",
		   intf & INTF_MIPI ? "MIPI" : " ",
		   sync == SYNC_USER ? "USER" :
		   sync < ARRAY_SIZE(vo_sync_name) ? vo_sync_name[sync] : "?",
		   vo.dev_attr[0]);
	if (sync == SYNC_USER) {
		/* dev_attr[3..]: type in byte 2, then Vact Vbb Vfb Hact Hbb
		 * Hfb, -, -, Hpw Vpw, Idv Ihs Ivs */
		seq_puts(m, "----------------------------VO USER ATTR------------------------------------\n");
		seq_puts(m, "Hpw   Hact  Hbb   Hfb   Vpw   Vact  Vbb   Vfb   Idv  Ihs  Ivs  Type\n");
		seq_printf(m, "%-4d  %-4d  %-4d  %-4d  %-4d  %-4d  %-4d  %-4d  %-3d  %-3d  %-3d  %-4d\n",
			   t[12], t[5], t[6], t[7], t[13], t[2], t[3], t[4],
			   tb[28], tb[29], tb[30], tb[2]);
	}
	seq_puts(m, "----------------------------VO CSC ATTR------------------------------------\n");
	seq_puts(m, "Type  Luma  Saturation  Contrast  Hue\n");
	seq_printf(m, "%-4s  %-4d  %-10d  %-8d  %-3d\n", "VO", vo_csc[0],
		   vo_csc[1], vo_csc[2], vo_csc[3]);
	seq_puts(m, "----------------------------VO IMAGE LAYER ATTR----------------------------\n");
	seq_puts(m, "DevId  ImgEn  ImgW  ImgH  DispX  DispY  DispW  DispH  Addr      Format    Effect\n");
	seq_printf(m, "%-5d  %-5s  %-4d  %-4d  %-5d  %-5d  %-5d  %-5d  %-8x  %-8s  %-6X\n",
		   0, vo.img_en ? "Y" : "N", vo.img_attr[2], vo.img_attr[3],
		   vo.img_attr[4], vo.img_attr[5], vo.img_attr[6],
		   vo.img_attr[7], vo.img_attr[1],
		   vo.img_attr[0] ? "ARGB8888" : "ARGB1555", vo.img_attr[8]);
	seq_puts(m, "----------------------------VO CURSOR LAYER ATTR---------------------------\n");
	seq_puts(m, "DevId  X     Y     Width  Height  Format    Effect\n");
	seq_printf(m, "%-5d  %-4d  %-4d  %-5d  %-6d  %-8s  %-6X\n", 0,
		   vo.cur_attr[1], vo.cur_attr[2], vo.cur_attr[3],
		   vo.cur_attr[4], vo.cur_attr[0] ? "ARGB8888" : "ARGB1555",
		   vo.img_attr[8]);
	seq_puts(m, "----------------------------VO VIDEO LAYER ATTR----------------------------\n");
	seq_puts(m, "DevId  ImgW  ImgH  DispX  DispY  DispW  DispH  Format\n");
	seq_printf(m, "%-5d  %-4d  %-4d  %-5d  %-5d  %-5d  %-5d  %-6s\n", 0,
		   vo.vl_attr[0], vo.vl_attr[1], vo.vl_attr[2], vo.vl_attr[3],
		   vo.vl_attr[4], vo.vl_attr[5], "SP420");
	seq_puts(m, "----------------------------VO CHN ATTR------------------------------------\n");
	seq_puts(m, "LayerId  ChnId  ChnEn  Show  X     Y     Width  Height  Format  Prior  Addr\n");
	for (n = 0; n < N_CHN; n++) {
		struct vo_chn *c = &vo.chn[n];

		if (c->configured != 1)
			continue;
		seq_printf(m, "%-7d  %-5d  %-5s  %-4s  %-4d  %-4d  %-5d  %-6d  %-6s  %-5d  %-8x\n",
			   0, n, c->enabled ? "Y" : "N", c->shown ? "Y" : "N",
			   c->attr[1], c->attr[2], c->attr[3], c->attr[4],
			   "SP420", c->attr[6], c->buf[0].w[4]);
	}
	seq_puts(m, "----------------------------VO DEV INFO------------------------------------\n");
	seq_puts(m, "Dev  TotFrm    DupFrm\n");
	seq_printf(m, "%-3d  %-8d  %-8d\n", 0, vo.tot_frm, vo.dup_frm);
	seq_puts(m, "----------------------------VO CHN INFO------------------------------------\n");
	seq_puts(m, "Dev  Chn  RevcFrm   BufState\n");
	for (n = 0; n < N_CHN; n++) {
		struct vo_chn *c = &vo.chn[n];

		if (c->configured == 1)
			seq_printf(m, "%-3d  %-3d  %-8d  %-2d  %-2d  %-2d\n", 0, n,
				   vo.recv_frm[n], c->buf[0].state,
				   c->buf[1].state, c->buf[2].state);
	}
	return 0;
}

static int vo_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, vo_proc_show, NULL);
}

static const struct file_operations vo_proc_fops = {
	.owner = THIS_MODULE,
	.open = vo_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int __init vo_init(void)
{
	int ret, i, k;

	spin_lock_init(&vo.lock);
	proc = proc_create("umap/vo", 0444, NULL, &vo_proc_fops);
	regs = ioremap(VO_BASE, VO_SIZE);
	if (!regs) {
		pr_err("VO: ioremap video in base failed!\n");
		ret = -ENOMEM;
		goto proc;
	}
	ret = misc_register(&vo_misc);
	if (ret) {
		pr_err("VO: register vo device failed!\n");
		ret = -EPERM;
		goto unmap;
	}
	ret = request_irq(VO_IRQ, vo_display_int, 0, "vo", NULL);
	if (ret) {
		pr_err("VO: VO_DEV_IRQ request_irq failed!\n");
		ret = -EIO;
		goto misc;
	}
	if (lcd_rgb[0] || lcd_rgb[1] || lcd_rgb[2]) {
		for (i = 0; i < 3; i++)
			for (k = 0; k < lcd_rgb[i] && k < 8; k++)
				if (lcd_pins[i][k] != lcd_pwm)
					mux(lcd_pins[i][k], 0x84);
		/* the clock, sync and enable lines */
		for (i = 88; i <= 91; i++)
			mux(i, 0x84);
		mux(14, 0xc00);
	}
	if (lcd_pwm)
		mux(lcd_pwm, 0xc00);
	if (lcd_disp)
		mux(lcd_disp, 0xc00);
	return 0;
misc:
	misc_deregister(&vo_misc);
unmap:
	iounmap(regs);
proc:
	if (proc)
		remove_proc_entry("umap/vo", NULL);
	return ret;
}

static void __exit vo_exit(void)
{
	misc_deregister(&vo_misc);
	free_irq(VO_IRQ, NULL);
	if (vo.vl_virt)
		iounmap(vo.vl_virt);
	iounmap(regs);
	if (proc)
		remove_proc_entry("umap/vo", NULL);
}

module_init(vo_init);
module_exit(vo_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("XM530V200 video output (open vo.ko)");
