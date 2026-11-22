// SPDX-License-Identifier: GPL-2.0
/*
 * The XM533's MIPI lanes as GPIOs (/dev/mipi_io): a drop-in for the
 * vendor mipi_io.ko. Pins 6..9 of the MIPI receiver (0x30000000) in
 * digital mode: +0x1D8 / +0x1E0 their input enables, +0x1E4 the output
 * levels, +0x1E8 the input levels.
 *
 *   0x40084d00  {pin, level}: the pin an output at that level
 *   0xc0084d01  {pin, level}: the pin an input, its level back
 *
 * mipi_io_set_output and mipi_io_get_input do the same for other modules.
 */

#include <linux/fs.h>
#include <linux/io.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>

#define MIPI_BASE 0x30000000
#define MIPI_MODE 0x281383e0 /* +0: the lanes in digital (GPIO) mode */
#define REG_IE0 0x1d8
#define REG_IE1 0x1e0
#define REG_OUT 0x1e4
#define REG_IN 0x1e8

#define MIPI_SET_OUTPUT 0x40084d00
#define MIPI_GET_INPUT 0xc0084d01

static void __iomem *regs;
static DEFINE_SPINLOCK(mipi_lock);

static inline void rmw(u32 off, u32 clr, u32 set)
{
	writel((readl(regs + off) & ~clr) | set, regs + off);
}

int mipi_io_set_output(u32 pin, u32 level)
{
	unsigned long flags;
	u32 bit;

	if (pin < 6 || pin > 9)
		return -1;
	bit = 1u << pin;
	spin_lock_irqsave(&mipi_lock, flags);
	writel(MIPI_MODE, regs);
	rmw(REG_IE0, bit, 0);
	rmw(REG_IE1, bit, 0);
	rmw(REG_OUT, bit, level ? bit : 0);
	spin_unlock_irqrestore(&mipi_lock, flags);
	return 0;
}
EXPORT_SYMBOL(mipi_io_set_output);

int mipi_io_get_input(u32 pin, u32 *level)
{
	unsigned long flags;
	u32 bit;

	if (pin < 6 || pin > 9)
		return -1;
	bit = 1u << pin;
	spin_lock_irqsave(&mipi_lock, flags);
	writel(MIPI_MODE, regs);
	rmw(REG_IE1, 0, bit);
	rmw(REG_IE0, 0, bit);
	*level = !!(readl(regs + REG_IN) & bit);
	spin_unlock_irqrestore(&mipi_lock, flags);
	return 0;
}
EXPORT_SYMBOL(mipi_io_get_input);

static long mipi_io_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	u32 v[2], __user *u = (u32 __user *)arg;

	if (cmd != MIPI_SET_OUTPUT && cmd != MIPI_GET_INPUT)
		return -ENOTTY;
	if (copy_from_user(v, u, sizeof(v))) {
		pr_err("mipi_ioctl: copy_from_user error.\n");
		return -EFAULT;
	}
	if (cmd == MIPI_SET_OUTPUT) {
		mipi_io_set_output(v[0], v[1]);
		return 0;
	}
	mipi_io_get_input(v[0], &v[1]);
	if (copy_to_user(u, v, sizeof(v))) {
		pr_err("mipi_ioctl: copy_to_user error.\n");
		return -EFAULT;
	}
	return 0;
}

static const struct file_operations mipi_io_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = mipi_io_ioctl,
};

static struct miscdevice mipi_io_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "mipi_io",
	.fops = &mipi_io_fops,
};

static int __init xm533_mipi_io_init(void)
{
	int ret;

	regs = ioremap(MIPI_BASE, 0x200);
	if (!regs) {
		pr_err("mipiio: ioremap mipiio base failed!\n");
		return -ENOMEM;
	}
	writel(MIPI_MODE, regs);
	ret = misc_register(&mipi_io_misc);
	if (ret)
		iounmap(regs);
	return ret;
}

static void __exit xm533_mipi_io_exit(void)
{
	misc_deregister(&mipi_io_misc);
	iounmap(regs);
}

module_init(xm533_mipi_io_init);
module_exit(xm533_mipi_io_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("XM533 MIPI lanes as GPIOs, compatible with mipi_io.ko");
