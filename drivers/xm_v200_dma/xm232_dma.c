// SPDX-License-Identifier: GPL-2.0
/*
 * The XM232's memory-to-memory DMA (/dev/dma): a drop-in for its vendor
 * dma.ko. The XM232 has no PL330: the block at 0x20020000 is a one-channel
 * engine laid out like a DesignWare DMAC, its block-done interrupt IRQ 47.
 * Sofia copies VI frames for its person detector through it.
 *
 *   +0x010  controller: 0 off, 3 on
 *   +0x018  channel enable: bit 0, bit 8 its write enable
 *   +0x100  source, +0x108 destination (physical)
 *   +0x110  the block's 8-byte transfers - 1
 *   +0x118  control (0x3fc45b00), +0x11c (0x24), +0x124 (0x20000)
 *   +0x180  2, +0x190 the done mask (2)
 *   +0x188  done status; written back to +0x198 to clear it
 *
 * A copy goes as blocks of at most 16 KiB, each started from the previous
 * one's interrupt, as dma.ko's does.
 *
 * ioctl, on physical addresses:
 *   0x400c4401  {src, dst, len}: len bytes, a multiple of 8
 *
 * Unlike dma.ko it refuses copies touching the kernel's own RAM, returns
 * -EINVAL for a bad length (dma.ko: -1, "size error") and -ETIMEDOUT when
 * the 200 ms are up (dma.ko: "dma timeout", then 0). dma_mem_copy() is
 * exported as dma.ko's is.
 */

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/sched.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <asm/memory.h>

#define DMAC_BASE	0x20020000
#define XM_DMA_IRQ	47
#define CFG		0x010
#define CHEN		0x018
#define SAR		0x100
#define DAR		0x108
#define BLOCK_TS	0x110
#define CTL_LO		0x118
#define CTL_HI		0x11c
#define CFG_HI		0x124
#define DONE_EN		0x180
#define STATUS		0x188
#define MASK		0x190
#define CLEAR		0x198

#define CHEN_CH0	0x101	/* channel 0 on, with its write enable */
#define DONE		2
#define CFG_ON		3

#define DMA_COPY	0x400c4401
#define BLOCK		16384

static void __iomem *regs;
static DEFINE_MUTEX(dma_lock);
static DECLARE_WAIT_QUEUE_HEAD(dma_wq);
static DEFINE_SPINLOCK(dma_slock);
static u32 copy_src, copy_dst, copy_len, copy_sent;
static int dma_done;

static void block_start(void)
{
	u32 n = min(copy_len - copy_sent, (u32)BLOCK);

	writel(copy_src + copy_sent, regs + SAR);
	writel(copy_dst + copy_sent, regs + DAR);
	writel(n / 8 - 1, regs + BLOCK_TS);
	writel(DONE, regs + DONE_EN);
	writel(DONE, regs + MASK);
	writel(CFG_ON, regs + CFG);
	writel(readl(regs + CHEN) | CHEN_CH0, regs + CHEN);
	copy_sent += n;
}

static irqreturn_t dma_irq(int irq, void *dev)
{
	u32 st = readl(regs + STATUS);

	if (!st)
		return IRQ_NONE;
	writel(st, regs + CLEAR);
	spin_lock(&dma_slock);
	if (copy_sent < copy_len) {
		block_start();
	} else {
		dma_done = 1;
		wake_up(&dma_wq);
	}
	spin_unlock(&dma_slock);
	return IRQ_HANDLED;
}

/* not the kernel's RAM */
static bool allowed(u32 addr, u64 len)
{
	u64 ram0 = PHYS_OFFSET, ram1 = __pa(high_memory - 1) + 1;

	return len && addr + len <= 0x100000000ull &&
	       (addr + len <= ram0 || addr >= ram1);
}

int dma_mem_copy(u32 src, u32 dst, u32 len)
{
	unsigned long flags;
	int ret = 0;

	if (len & 7 || !allowed(src, len) || !allowed(dst, len))
		return -EINVAL;
	mutex_lock(&dma_lock);
	writel(0, regs + CFG);
	udelay(1);
	writel(0x3fc45b00, regs + CTL_LO);
	writel(0x24, regs + CTL_HI);
	writel(0x20000, regs + CFG_HI);
	spin_lock_irqsave(&dma_slock, flags);
	copy_src = src;
	copy_dst = dst;
	copy_len = len;
	copy_sent = 0;
	dma_done = 0;
	block_start();
	spin_unlock_irqrestore(&dma_slock, flags);
	if (!wait_event_timeout(dma_wq, dma_done, msecs_to_jiffies(200))) {
		pr_err("xm_dma: timeout after %u of %u bytes\n", copy_sent,
		       len);
		spin_lock_irqsave(&dma_slock, flags);
		copy_len = copy_sent;	/* the ISR starts nothing more */
		spin_unlock_irqrestore(&dma_slock, flags);
		writel(CHEN_CH0 & ~1, regs + CHEN);
		writel(0, regs + CFG);
		ret = -ETIMEDOUT;
	}
	mutex_unlock(&dma_lock);
	return ret;
}
EXPORT_SYMBOL(dma_mem_copy);

static long dma_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	u32 r[3];

	if (cmd != DMA_COPY)
		return -ENOTTY;
	if (copy_from_user(r, (void __user *)arg, sizeof(r)))
		return -EFAULT;
	return dma_mem_copy(r[0], r[1], r[2]);
}

static const struct file_operations dma_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = dma_ioctl,
};

static struct miscdevice dma_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "dma",
	.fops = &dma_fops,
};

static int __init xm232_dma_init(void)
{
	int ret;

	regs = ioremap(DMAC_BASE, 0x1000);
	if (!regs)
		return -ENOMEM;
	ret = request_irq(XM_DMA_IRQ, dma_irq, 0, "dma-irq", NULL);
	if (ret)
		goto unmap;
	ret = misc_register(&dma_misc);
	if (ret)
		goto irq;
	return 0;
irq:
	free_irq(XM_DMA_IRQ, NULL);
unmap:
	iounmap(regs);
	return ret;
}

static void __exit xm232_dma_exit(void)
{
	misc_deregister(&dma_misc);
	free_irq(XM_DMA_IRQ, NULL);
	writel(0, regs + CFG);
	iounmap(regs);
}

module_init(xm232_dma_init);
module_exit(xm232_dma_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("XM232 memory-to-memory DMA, compatible with its dma.ko");
