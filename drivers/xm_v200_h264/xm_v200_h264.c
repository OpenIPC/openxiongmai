// SPDX-License-Identifier: GPL-2.0
/*
 * The XM533's and XM535s' H.264 encoder (/dev/h264): a drop-in for the
 * vendor h264.ko, Hantro's hx280enc driver built for them. User space
 * drives the encoder's registers itself (through /dev/mem); the driver
 * gives their address and size, and waits for a frame:
 *
 *   0x80046b03  the registers' physical address (0x21100000)
 *   0x80046b04  their size (0x1c4)
 *   0x6b0c      wait for the encoder's interrupt (a frame done)
 *
 * The encoder's clock is turned on at 0x20000120. base_port= and irq=
 * as h264.ko's.
 */

#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/uaccess.h>
#include <linux/wait.h>

#define ENC_SIZE 0x1c4
#define REG_ID 0x00
#define REG_IRQ 0x04  /* bit 0 the interrupt, 0x1fe its causes */
#define REG_CTRL 0x38 /* bit 0 the encoder on */

#define IOC_MAGIC 'k'
#define IOC_MAXNR 12
#define IOCGHWOFFSET 0x80046b03
#define IOCGHWIOSIZE 0x80046b04
#define IOCWAITDONE 0x6b0c

static unsigned long base_port = 0x21100000;
module_param(base_port, ulong, 0444);
static int irq = 78;
module_param(irq, int, 0444);

static void __iomem *regs;
static DECLARE_WAIT_QUEUE_HEAD(enc_wq);
static int enc_done;

static irqreturn_t hx280enc_isr(int i, void *dev)
{
	u32 st = readl(regs + REG_IRQ);

	if (!(st & 1))
		return IRQ_NONE;
	writel(st & ~0x101u, regs + REG_IRQ);
	if ((st & 0x1fe) == 0x100)
		return IRQ_HANDLED; /* buffer full only: user space handles it */
	if (st & 0x10) {
		/* a bus error: the encoder stopped */
		writel(0, regs + REG_IRQ);
		return IRQ_HANDLED;
	}
	enc_done = 1;
	wake_up_interruptible(&enc_wq);
	return IRQ_HANDLED;
}

static long hx280enc_ioctl(struct file *file, unsigned int cmd,
			   unsigned long arg)
{
	u32 v;

	if (_IOC_TYPE(cmd) != IOC_MAGIC || _IOC_NR(cmd) > IOC_MAXNR)
		return -ENOTTY;
	switch (cmd) {
	case IOCGHWOFFSET:
		v = base_port;
		return copy_to_user((void __user *)arg, &v, 4) ? -EFAULT : 0;
	case IOCGHWIOSIZE:
		v = ENC_SIZE;
		return copy_to_user((void __user *)arg, &v, 4) ? -EFAULT : 0;
	case IOCWAITDONE:
		if (wait_event_interruptible(enc_wq, enc_done))
			return -ERESTARTSYS;
		enc_done = 0;
		return 0;
	}
	return 0;
}

static int hx280enc_mmap(struct file *file, struct vm_area_struct *vma)
{
	return -EINVAL; /* as h264.ko: the registers through /dev/mem */
}

static const struct file_operations hx280enc_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = hx280enc_ioctl,
	.mmap = hx280enc_mmap,
};

static struct miscdevice hx280enc_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "h264",
	.fops = &hx280enc_fops,
};

static int __init hx280enc_init(void)
{
	void __iomem *crg;
	u32 id, i;
	int ret;

	pr_info("h264e: module init - base_port %#lx irq %d\n", base_port, irq);
	crg = ioremap(0x20000120, 4);
	if (crg) {
		writel(readl(crg) | 3, crg); /* the encoder's clock */
		iounmap(crg);
	}
	if (!request_mem_region(base_port, ENC_SIZE, "hx280enc")) {
		pr_err("h264e: failed to reserve HW regs\n");
		return -EBUSY;
	}
	regs = ioremap(base_port, ENC_SIZE);
	if (!regs) {
		ret = -EBUSY;
		goto release;
	}
	id = readl(regs + REG_ID) >> 16;
	if ((id & ~0x1000u) != 0x6280 && ((id - 0x8270) & ~0x20u)) {
		pr_err("h264e: HW not found at %#lx\n", base_port);
		ret = -EBUSY;
		goto unmap;
	}
	pr_info("h264e: HW at base <%#lx> with ID <0x%04x>\n", base_port, id);
	writel(0, regs + REG_CTRL);
	for (i = 4; i < ENC_SIZE; i += 4)
		writel(0, regs + i);
	if (irq != -1) {
		ret = request_irq(irq, hx280enc_isr, 0, "hx280enc", NULL);
		if (ret) {
			pr_err("h264e: can't get IRQ %d\n", irq);
			goto unmap;
		}
	} else {
		pr_info("h264e: IRQ not in use!\n");
	}
	ret = misc_register(&hx280enc_misc);
	if (ret)
		goto irq;
	return 0;
irq:
	if (irq != -1)
		free_irq(irq, NULL);
unmap:
	iounmap(regs);
release:
	release_mem_region(base_port, ENC_SIZE);
	pr_info("h264e: module not inserted\n");
	return ret;
}

static void __exit hx280enc_cleanup(void)
{
	writel(0, regs + REG_CTRL);
	writel(0, regs + REG_IRQ);
	misc_deregister(&hx280enc_misc);
	if (irq != -1)
		free_irq(irq, NULL);
	iounmap(regs);
	release_mem_region(base_port, ENC_SIZE);
	pr_info("h264e: module removed\n");
}

module_init(hx280enc_init);
module_exit(hx280enc_cleanup);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("XM533/XM535 H.264 encoder (hx280enc), compatible with h264.ko");
