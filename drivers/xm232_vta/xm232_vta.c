// SPDX-License-Identifier: GPL-2.0
/*
 * XM232's VTA neural network accelerator (0x22000000): /dev/vta, a drop-in
 * for the vendor XmVtaMte.ko.
 *
 * The vendor's runtime drives the accelerator itself through /dev/mem: it
 * loads the network's instructions, sets the instruction count (+0x018)
 * and address (+0x020), starts the engine (+0x000 = 1), then polls /dev/vta
 * for its interrupt, giving up after 5 s (it then halts the engine through
 * +0x010, waits for +0x218 to read idle and resets it through the system
 * controller). The driver only acknowledges the interrupt and reports it:
 *
 *   interrupt   +0x008 written 0; after the first open, the next poll()
 *               returns POLLIN | POLLRDNORM, once
 *   open        forgets an interrupt not yet reported
 *
 * Parameters as the vendor's: vta_ip_base (0x22000000), vta_irq_num (the
 * interrupt; Xiongmai's loader passes 59, SPI 27) and mte_irq_num
 * (accepted; the vendor module uses no MTE interrupt either).
 */

#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/wait.h>

#define VTA_IRQ_ACK 0x008

static int vta_ip_base = 0x22000000;
module_param(vta_ip_base, int, 0400);
static int vta_irq_num = 54;
module_param(vta_irq_num, int, 0400);
static int mte_irq_num = 55;
module_param(mte_irq_num, int, 0400);

static void __iomem *vta;
static DECLARE_WAIT_QUEUE_HEAD(vta_wq);
static bool vta_opened;		/* interrupts are reported once opened */
static atomic_t vta_event = ATOMIC_INIT(0);
static int vta_irq;

static irqreturn_t vta_interrupt(int irq, void *dev)
{
	writel(0, vta + VTA_IRQ_ACK);
	if (vta_opened) {
		atomic_set(&vta_event, 1);
		wake_up_interruptible(&vta_wq);
	}
	return IRQ_HANDLED;
}

static int vta_mte_open(struct inode *inode, struct file *file)
{
	atomic_set(&vta_event, 0);
	vta_opened = true;
	return 0;
}

static unsigned int vta_mte_poll(struct file *file, poll_table *wait)
{
	poll_wait(file, &vta_wq, wait);
	return atomic_xchg(&vta_event, 0) ? POLLIN | POLLRDNORM : 0;
}

static const struct file_operations vta_fops = {
	.owner = THIS_MODULE,
	.open = vta_mte_open,
	.poll = vta_mte_poll,
};

static struct miscdevice vta_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "vta",
	.fops = &vta_fops,
};

static int __init vta_init(void)
{
	int ret;

	vta = ioremap(vta_ip_base, 0x1000);
	if (!vta)
		return -ENOMEM;
	ret = misc_register(&vta_misc);
	if (ret < 0) {
		pr_err("register miscdevice error code:%d\n", ret);
		goto unmap;
	}
	vta_irq = vta_irq_num;
	ret = request_irq(vta_irq, vta_interrupt, IRQF_TRIGGER_HIGH, "VTA",
			  &vta_irq);
	if (ret) {
		pr_err("register irq error code:%d\n", ret);
		misc_deregister(&vta_misc);
		ret = -EBUSY;
		goto unmap;
	}
	return 0;
unmap:
	iounmap(vta);
	return ret;
}

static void __exit vta_exit(void)
{
	free_irq(vta_irq, &vta_irq);
	misc_deregister(&vta_misc);
	iounmap(vta);
}

module_init(vta_init);
module_exit(vta_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("XM232 VTA accelerator interrupt, compatible with XmVtaMte.ko");
