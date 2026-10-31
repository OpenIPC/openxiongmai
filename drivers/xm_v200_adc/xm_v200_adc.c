// SPDX-License-Identifier: GPL-2.0
/*
 * The V200 series' ADC (/dev/adc): a drop-in for the vendor adc.ko.
 *
 * One ioctl, 0xc0080001 {channel, value}: the channel is converted and
 * its value read back. The converter is at 0x10020CC0: control (bit 0..1
 * on, bit 7/8 start, the channel + 1 in bits 4..6; the XM535s also bit
 * 14), result at +4. The XM535s' converter has its clock from the system
 * controller (ADC_XM535 at build time).
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

#ifdef ADC_XM535
#define ADC_START 0x4183
#else
#define ADC_START 0x183
#endif

static void __iomem *regs;
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
	writel(1, regs + ADC_CTRL);
	udelay(100);
	writel(3, regs + ADC_CTRL);
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
