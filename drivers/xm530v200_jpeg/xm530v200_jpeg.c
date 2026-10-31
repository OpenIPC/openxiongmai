// SPDX-License-Identifier: GPL-2.0
/*
 * The XM530V200's JPEG encoder (/dev/jpeg): a drop-in for the vendor
 * jpeg.ko. User space owns the encoder between 0x4a04 and 0x4a05, hands
 * it a table of 136 register writes and waits for the picture:
 *
 *   0x4a04       take the encoder (waits for it; switch_en=1 also moves
 *                its clock to the faster PLL)
 *   0x4a05       give it back
 *   0x4a06       reset it (system controller 0x20000120)
 *   0x400c4a03   the table: 136 entries {register offset, value, changed};
 *                the changed ones are written, entry 0 (start) last
 *   0x80044a02   wait for it (up to 1 s): its status back, bit 0 done,
 *                1 an error (2..6 the error bits), 7 timed out
 *
 * The encoder is at 0x21100000, IRQ 78: +0x3c 3 when a picture is done,
 * +0x40 the error bits.
 */

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/semaphore.h>
#include <linux/uaccess.h>
#include <linux/wait.h>

#define JPEG_BASE 0x21100000
#define JPEG_SIZE 0x220
#define JPEG_IRQ 78
#define N_REG 136

#define JPEG_WAIT 0x80044a02
#define JPEG_SET_REGS 0x400c4a03
#define JPEG_TAKE 0x4a04
#define JPEG_GIVE 0x4a05
#define JPEG_RESET 0x4a06

static unsigned int switch_en;
module_param(switch_en, uint, 0444);

struct reg {
	u32 off, val, changed;
};

static struct reg hw[N_REG], cfg[N_REG];
static void __iomem *regs, *sys;
static DEFINE_SEMAPHORE(hw_sem);
static DEFINE_MUTEX(reg_lock);
static DECLARE_WAIT_QUEUE_HEAD(jpeg_wq);
static unsigned long status;
static struct file *owner;

static void jpeg_reset(void)
{
	writel(6, sys + 0x120);
	udelay(100);
	writel(7, sys + 0x120);
	memset(hw, 0, sizeof(hw));
	memset(cfg, 0, sizeof(cfg));
	status = 0;
}

/* the encoder's clock from the other PLL (bit 8 of 0x20000074), or back */
static void switch_clk(int fast)
{
	u32 v;

	writel(1, sys);
	v = readl(sys + 0x74);
	v = fast ? v | 0x100 : v & ~0x100u;
	for (;;) {
		writel(v, sys + 0x74);
		if (readl(sys + 0x74) == v)
			break;
		pr_info("jpeg: clock switch again\n");
		udelay(1000);
	}
}

static irqreturn_t jpeg_interrupt(int irq, void *dev)
{
	u32 err;
	int i;

	if (readl(regs + 0x3c) == 3) {
		writel(0, regs);
		writel(0, regs + 0x24);
		set_bit(0, &status);
		wake_up_interruptible(&jpeg_wq);
	}
	err = readl(regs + 0x40);
	if (err) {
		pr_err("JPEG_MIS_ERR: %x\n", err);
		writel(0, regs);
		writel(0, regs + 0x30);
		writel(0, regs + 0x34);
		writel(0, regs + 0x24);
		writel(0, regs + 0x28);
		for (i = 0; i < 5; i++)
			if (err & 1u << i)
				set_bit(2 + i, &status);
		set_bit(1, &status);
		wake_up_interruptible(&jpeg_wq);
	}
	return IRQ_HANDLED;
}

static long jpeg_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	long t;
	u32 v;
	int i;

	switch (cmd) {
	case JPEG_RESET:
		jpeg_reset();
		return 0;
	case JPEG_TAKE:
		if (down_interruptible(&hw_sem))
			return -ERESTARTSYS;
		owner = file;
		if (switch_en)
			switch_clk(1);
		return 0;
	case JPEG_GIVE:
		/* only the owner gives the encoder back (and gates its clock) */
		if (owner != file)
			return -EPERM;
		if (switch_en)
			switch_clk(0);
		owner = NULL;
		up(&hw_sem);
		return 0;
	case JPEG_SET_REGS:
		if (!arg)
			return -EINVAL;
		mutex_lock(&reg_lock);
		if (copy_from_user(cfg, (void __user *)arg, sizeof(cfg))) {
			mutex_unlock(&reg_lock);
			return -EFAULT;
		}
		for (i = 0; i < N_REG; i++)
			if (cfg[i].changed)
				hw[i] = cfg[i];
		for (i = 1; i < N_REG; i++)
			if (hw[i].changed && hw[i].off < JPEG_SIZE) {
				writel(hw[i].val, regs + (hw[i].off & ~3u));
				hw[i].changed = 0;
			}
		if (hw[0].off < JPEG_SIZE)
			writel(hw[0].val, regs + (hw[0].off & ~3u));
		hw[0].changed = 0;
		mutex_unlock(&reg_lock);
		return 0;
	case JPEG_WAIT:
		t = wait_event_interruptible_timeout(jpeg_wq, status, HZ);
		if (!t) {
			set_bit(7, &status);
			pr_err("jpeg: timeout\n");
		} else if (t < 0) {
			set_bit(8, &status);
		}
		v = status;
		status = 0;
		return copy_to_user((void __user *)arg, &v, 4) ? -EFAULT : 0;
	}
	pr_err("jpeg: unknown cmd\n");
	return -ENOTTY;
}

static int jpeg_release(struct inode *inode, struct file *file)
{
	if (owner == file) {
		owner = NULL;
		up(&hw_sem);
	}
	return 0;
}

static const struct file_operations jpeg_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = jpeg_ioctl,
	.release = jpeg_release,
};

static struct miscdevice jpeg_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "jpeg",
	.fops = &jpeg_fops,
};

static int __init xm530v200_jpeg_init(void)
{
	int ret;

	sys = ioremap(0x20000000, 0x200);
	regs = ioremap(JPEG_BASE, JPEG_SIZE);
	if (!sys || !regs) {
		ret = -ENOMEM;
		goto unmap;
	}
	jpeg_reset();
	ret = request_irq(JPEG_IRQ, jpeg_interrupt, 0, "jpeg", NULL);
	if (ret)
		goto unmap;
	ret = misc_register(&jpeg_misc);
	if (ret) {
		free_irq(JPEG_IRQ, NULL);
		goto unmap;
	}
	if (switch_en)
		pr_info("jpeg switch pll enabled\n");
	pr_info("jpeg: openxiongmai xm530v200_jpeg\n");
	return 0;
unmap:
	if (regs)
		iounmap(regs);
	if (sys)
		iounmap(sys);
	return ret;
}

static void __exit xm530v200_jpeg_exit(void)
{
	misc_deregister(&jpeg_misc);
	free_irq(JPEG_IRQ, NULL);
	iounmap(regs);
	iounmap(sys);
}

module_init(xm530v200_jpeg_init);
module_exit(xm530v200_jpeg_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("XM530V200 JPEG encoder, compatible with jpeg.ko");
