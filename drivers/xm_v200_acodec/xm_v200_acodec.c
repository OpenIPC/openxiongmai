// SPDX-License-Identifier: GPL-2.0
/*
 * The V200 series' audio codec (/dev/acodec): a drop-in for the vendor
 * acodec.ko. The codec is at 0x10068000; the XM530V200 and XM533 have one
 * design ("inno": mute at +0xA8/+0x18, input gain at +0x8C/+0x98), the
 * XM535s another (mute at +0x14, input gain at +0xC4/+0xC8). Both take
 * the output volume at +0xAC.
 *
 *   0x40044101  output volume 0..100 (0 mutes)
 *   0x40044103  output mute {1 / 0}
 *   0x80044104  the output mute
 *   0x40044105  input gain 0..100 (10 or less mutes)
 *
 * alc=1 turns on the input's automatic level (PGA), aomute=1 starts with
 * the output muted (the inno codec). ACODEC_CHIP=v200, xm533, xm535 or
 * xm535w1 at build time.
 */

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/uaccess.h>

#define ACODEC_BASE 0x10068000
#define SET_VOLUME 0x40044101
#define SET_MUTE 0x40044103
#define GET_MUTE 0x80044104
#define SET_GAIN 0x40044105

#if defined(ACODEC_XM535) || defined(ACODEC_XM535W1)
#define CODEC_535 1
#endif

static int alc;
module_param(alc, int, 0444);
static int aomute;
module_param(aomute, int, 0444);

static void __iomem *regs;
static int vo_mute, vi_mute;
static DEFINE_MUTEX(codec_lock);

static inline void wr(u32 off, u32 v)
{
	writel(v, regs + off);
}

static inline void rmw(u32 off, u32 clr, u32 set)
{
	writel((readl(regs + off) & ~clr) | set, regs + off);
}

/* acodec.ko exports it for i2s.ko; nothing to do on these chips */
void acodec_adc_reset(void)
{
}
EXPORT_SYMBOL(acodec_adc_reset);

static void set_mute(int mute)
{
#ifdef CODEC_535
	rmw(0x14, 1, mute ? 1 : 0);
#else
	if (mute) {
		rmw(0xa8, 0x40, 0);
		wr(0x18, 0);
	} else {
		rmw(0xa8, 0, 0x40);
		wr(0x18, 0xf3);
	}
#endif
	vo_mute = mute;
}

static void set_volume(int v)
{
	u32 r;

	if (v > 100) {
		r = 0x1a;
	} else if (v <= 0) {
		set_mute(1);
		wr(0xac, 0x10);
		return;
	} else {
		r = (v / 10 + 0x10) & 0xff;
	}
	wr(0xac, r);
	if (vo_mute == 1)
		set_mute(0);
}

static void set_gain(int v)
{
#ifdef CODEC_535
	const u32 en = 0xc4, en_bit = 0x10, gain = 0xc8;
#else
	const u32 en = 0x8c, en_bit = 0x80, gain = 0x98;
#endif

	if (v <= 10) {
		rmw(en, en_bit, 0);
		vi_mute = 1;
		return;
	}
	if (v > 100) {
		if (!vi_mute) {
			wr(gain, 0x1f);
			return;
		}
		v = 100;
	}
	if (vi_mute) {
		rmw(en, 0, en_bit);
		vi_mute = 0;
	}
	wr(gain, v < 51 ? ((v >> 1) + 6) & 0xff : 0x1f);
}

static long acodec_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int v, __user *u = (int __user *)arg;

	if (cmd == GET_MUTE)
		return copy_to_user(u, &vo_mute, sizeof(vo_mute)) ? -EFAULT : 0;
	if (cmd != SET_VOLUME && cmd != SET_MUTE && cmd != SET_GAIN) {
		pr_err("acodec: unknown cmd\n");
		return -ENOTTY;
	}
	if (get_user(v, u))
		return -EFAULT;
	mutex_lock(&codec_lock);
	if (cmd == SET_VOLUME)
		set_volume(v);
	else if (cmd == SET_MUTE)
		set_mute(v);
	else
		set_gain(v);
	mutex_unlock(&codec_lock);
	return 0;
}

#ifdef CODEC_535
#ifdef ACODEC_XM535W1
#define STEP 5 /* ms */
#define CHARGE 0xf
#else
#define STEP 1
#define CHARGE 0x40
#endif

static void codec_init(void)
{
	int i;

	wr(0x000, 0xc0);
	mdelay(STEP);
	wr(0x000, 0xf1);
	mdelay(STEP);
	wr(0x004, 0x8);
	wr(0x024, 0x8);
	rmw(0x0a0, 0, 0x20);
	mdelay(STEP);
	wr(0x084, 0x1);
	mdelay(STEP);
	rmw(0x080, 0, 0x1);
	mdelay(STEP);
	/* the reference charged, then held */
	wr(0x084, 0xff);
	for (i = 0; i < 20; i++)
		mdelay(STEP);
	wr(0x084, CHARGE);
	for (i = 0; i < 20; i++)
		mdelay(STEP);
	wr(0x034, 0xc3);
	/* the ADC path, one stage at a time */
	rmw(0x080, 0, 0x4);
	rmw(0x0c0, 0, 0x4);
	rmw(0x0c4, 0, 0x2);
	rmw(0x0c4, 0, 0x1);
	rmw(0x0c0, 0, 0x2);
	rmw(0x0c0, 0, 0x1);
	rmw(0x0c0, 0, 0x10);
	rmw(0x0c4, 0, 0x4);
	rmw(0x0c4, 0, 0x8);
	rmw(0x0c4, 0, 0x10);
	wr(0x0cc, 0x3);
	wr(0x0c8, 0x1f);
	rmw(0x0c0, 0, 0x8);
	rmw(0x080, 0xffffff00, 0xe0);
	rmw(0x080, 0, 0x8);
	wr(0x018, 0xe1);
	rmw(0x080, 0, 0x2);
	/* the DAC path */
	rmw(0x0a0, 0, 0x4);
	rmw(0x0a0, 0xffffff60, 0x40);
	rmw(0x0a4, 0, 0x1);
	rmw(0x0a4, 0, 0x4);
	rmw(0x0a0, 0, 0x8);
	rmw(0x0a0, 0, 0x2);
	rmw(0x0a0, 0, 0x1);
	rmw(0x0a0, 0, 0x10);
	rmw(0x0a4, 0, 0x20);
	rmw(0x0ac, 0xfffffff7, 0x17);
}
#else
static void codec_init(void)
{
	int i;

	wr(0x000, 0x0);
	mdelay(1);
	wr(0x000, 0x1);
	mdelay(1);
	wr(0x000, 0x3);
	mdelay(1);
	rmw(0x0a4, 0, 0x10);
	/* the reference charged, then held */
	wr(0x084, 0x1);
	rmw(0x088, 0, 0x20);
	for (i = 0; i < 20; i++)
		mdelay(1);
	wr(0x084, 0xff);
	mdelay(1);
	wr(0x014, 0xe);
	wr(0x010, 0x10);
	wr(0x00c, 0xe);
	wr(0x008, 0x10);
	wr(0x020, 0xc0);
	/* the ADC path */
	rmw(0x08c, 0, 0x80);
	rmw(0x088, 0, 0x1f);
	rmw(0x08c, 0, 0x20);
	rmw(0x094, 0, 0x10);
	rmw(0x094, 0, 0x20);
	rmw(0x0a0, 0, 0x40);
	rmw(0x0a0, 0, 0x20);
	rmw(0x0a0, 0, 0x10);
	rmw(0x0a0, 0, 0x80);
	rmw(0x08c, 0, 0x40);
	rmw(0x090, 0, 0xc0);
	rmw(0x098, 0, 0x1f);
	rmw(0x08c, 0, 0x10);
	wr(0x018, 0xf3);
	/* the DAC path */
	rmw(0x0a4, 0, 0x1);
	rmw(0x0a4, 0, 0x2);
	rmw(0x0a4, 0, 0x80);
	rmw(0x0a4, 0, 0x40);
	rmw(0x0a4, 0, 0x20);
	rmw(0x0a8, 0, 0x10);
	rmw(0x0a8, 0, 0x20);
	rmw(0x0a4, 0, 0x8);
	rmw(0x0a4, 0, 0x4);
	rmw(0x0ac, 0, 0x15);
	rmw(0x0a8, 0, 0x40);
#ifdef ACODEC_XM533
	wr(0x01c, 0x14);
#else
	wr(0x01c, 0);
#endif
	wr(0x088, 0x38);
	if (alc == 1) {
		wr(0x028, 0x1c);
		wr(0x020, 0xc9);
		wr(0x104, 0xaa);
		wr(0x108, 0x41);
		wr(0x110, 0x7);
		wr(0x124, 0x78);
		wr(0x128, 0x7f);
		wr(0x12c, 0xff);
		pr_info("ALC pga enable.\n");
	}
	if (aomute) {
		wr(0x0a8, 0x38);
		wr(0x018, 0);
		pr_info("Ao mute.\n");
	}
}
#endif

static const struct file_operations acodec_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = acodec_ioctl,
};

static struct miscdevice acodec_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "acodec",
	.fops = &acodec_fops,
};

static int __init xm_v200_acodec_init(void)
{
	int ret;

	regs = ioremap(ACODEC_BASE, 0x200);
	if (!regs)
		return -ENOMEM;
	codec_init();
	ret = misc_register(&acodec_misc);
	if (ret) {
		pr_err("acodec init error!\n");
		iounmap(regs);
		return ret;
	}
	pr_info("acodec init is ok!\n");
	return 0;
}

static void __exit xm_v200_acodec_exit(void)
{
	wr(0, 0);
	misc_deregister(&acodec_misc);
	iounmap(regs);
	pr_info("remove acodec!\n");
}

module_init(xm_v200_acodec_init);
module_exit(xm_v200_acodec_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("V200-series audio codec, compatible with acodec.ko");
