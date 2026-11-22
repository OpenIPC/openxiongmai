// SPDX-License-Identifier: GPL-2.0
/*
 * The V200 series' ADC (/dev/adc): a drop-in for the vendor adc.ko.
 *
 * One ioctl, 0xc0080001 {channel, value}: the channel is converted and
 * its value read back. The converter is at 0x10020CC0: control (bit 0..1
 * on, bit 7/8 start, the channel + 1 in bits 4..6; the XM535s also bit
 * 14), result at +4. The XM535s' converter has its clock from the system
 * controller (ADC_XM535 at build time).
 *
 * The XM232's (ADC_XM232) is calibrated: at load, and on 0x40040002, it
 * converts the ten reference inputs 5..14 (control (in << 4) | 0x40db03,
 * 500 us each), and 0xc0080001 returns the reading mapped through them to
 * 0..255, piecewise linear, as its adc.ko does.
 */

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>

#define ADC_BASE 0x10020cc0
#define ADC_CTRL 0x0
#define ADC_DATA 0x4
#define ADC_READ 0xc0080001
#define ADC_CALIB 0x40040002

#if defined(ADC_XM232)
#define ADC_START 0x40db03
#elif defined(ADC_XM535)
#define ADC_START 0x4183
#else
#define ADC_START 0x183
#endif

static void __iomem *regs;

#ifdef ADC_XM232
#define REFS 10
#define REF_IN0 5

/* each reference's value, in thousandths of a step */
static const s32 calib_v[REFS] = {
	0, 28033, 56066, 84098, 112131, 140165, 168198, 196231, 224264, 255000,
};
static s32 calib_tv[REFS];	/* the reading at each reference */
static s32 calib_dv[REFS];	/* calib_v - 1000 * calib_tv */
static int ch_old = -1;
static DEFINE_MUTEX(adc_mutex);

static void adc_calib(void)
{
	int i;

	mutex_lock(&adc_mutex);
	for (i = 0; i < REFS; i++) {
		writel((REF_IN0 + i) << 4 | ADC_START, regs + ADC_CTRL);
		udelay(500);
		calib_tv[i] = readl(regs + ADC_DATA);
		calib_dv[i] = calib_v[i] - 1000 * calib_tv[i];
	}
	/* adc.ko keeps the last two references' offsets equal */
	calib_dv[REFS - 1] = calib_dv[REFS - 2];
	ch_old = -1;
	mutex_unlock(&adc_mutex);
}

/* the reading through the references, in adc.ko's integer steps */
static u32 adc_map(s32 d)
{
	s32 r;
	int k;

	if (d == calib_tv[0])
		return 0;
	for (k = 0; k < REFS - 1; k++) {
		if (d < calib_tv[k] || d > calib_tv[k + 1])
			continue;
		if (calib_tv[k + 1] == calib_tv[k])
			return 0;
		r = (calib_dv[k + 1] - calib_dv[k]) /
		    (calib_tv[k + 1] - calib_tv[k]);
		r = (d - calib_tv[k + 1]) * r + calib_dv[k + 1] + 1000 * d;
		if (r < 0)
			return 0;
		if (r > calib_v[REFS - 1])
			return 255;
		/* to the nearest step, by the hundreds digit */
		return r / 1000 + (r / 100 % 10 > 4);
	}
	return 0;
}

static long adc_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	u32 v[2], __user *u = (u32 __user *)arg;
	s32 d;

	if (cmd == ADC_CALIB) {
		adc_calib();
		return 0;
	}
	if (cmd != ADC_READ) {
		pr_err("unsupport cmd.\n");
		return -ENOTTY;
	}
	if (copy_from_user(v, u, sizeof(v)))
		return -EFAULT;
	mutex_lock(&adc_mutex);
	writel((v[0] + 1) << 4 | ADC_START, regs + ADC_CTRL);
	if (v[0] != ch_old)
		udelay(500);
	ch_old = v[0];
	d = readl(regs + ADC_DATA);
	v[1] = adc_map(d);
	mutex_unlock(&adc_mutex);
	return copy_to_user(u, v, sizeof(v)) ? -EFAULT : 0;
}
#else
static DEFINE_SPINLOCK(adc_lock);

static long adc_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	u32 v[2], __user *u = (u32 __user *)arg;
	unsigned long flags;

	if (cmd != ADC_READ) {
		pr_err("unsupport cmd.\n");
		return -ENOTTY;
	}
	if (copy_from_user(v, u, sizeof(v)))
		return -EFAULT;
	spin_lock_irqsave(&adc_lock, flags);
	writel((v[0] + 1) << 4 | ADC_START, regs + ADC_CTRL);
	udelay(10);
	v[1] = readl(regs + ADC_DATA);
	spin_unlock_irqrestore(&adc_lock, flags);
	return copy_to_user(u, v, sizeof(v)) ? -EFAULT : 0;
}
#endif

static const struct file_operations adc_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = adc_ioctl,
};

static struct miscdevice adc_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "adc",
	.fops = &adc_fops,
};

static int __init xm_v200_adc_init(void)
{
	int ret;

	regs = ioremap(ADC_BASE, 8);
	if (!regs)
		return -ENOMEM;
#ifdef ADC_XM535
	{
		void __iomem *sys = ioremap(0x20000000, 0x60);

		if (sys) {
			/* the converter's clock, then the clocks relocked */
			writel(0x80ff, sys + 0x5c);
			writel(0, sys);
			iounmap(sys);
		}
	}
#endif
	writel(0, regs + ADC_CTRL);
	udelay(100);
#ifdef ADC_XM232
	adc_calib();
#else
	writel(1, regs + ADC_CTRL);
	udelay(100);
	writel(3, regs + ADC_CTRL);
#endif
	ret = misc_register(&adc_misc);
	if (ret) {
		pr_err("adc: register failed.\n");
		iounmap(regs);
	}
	return ret;
}

static void __exit xm_v200_adc_exit(void)
{
	misc_deregister(&adc_misc);
	iounmap(regs);
}

module_init(xm_v200_adc_init);
module_exit(xm_v200_adc_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("V200-series ADC, compatible with adc.ko");
