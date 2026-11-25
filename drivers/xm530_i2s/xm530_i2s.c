// SPDX-License-Identifier: GPL-2.0
/*
 * The XM530/XM550's audio input and output (/dev/ai, /dev/ao): a drop-in for
 * the vendor xm_i2s.ko.
 *
 * The codec (acodec.ko) talks I2S to one block at 0x10067000 (IRQ 49), both
 * ways. Its interrupt moves 160 16-bit samples a time between the block's
 * FIFO (+0x38) and a FIFO in memory, read from /dev/ai and written to
 * /dev/ao. Reads and writes wait unless O_NONBLOCK. There are no ioctls.
 * /proc/umap/audio counts the interrupts, and the bytes that did not fit
 * (BufFull) or were not there (BufEmpty).
 *
 * What differs from the vendor module:
 *  - /proc/umap/audio is released on close: the vendor module opens it with
 *    seq_open() and leaks 4 KiB each time (OpenIPC/firmware#2956);
 *  - a read or write larger than the FIFO is cut to it; there it waits for
 *    ever;
 *  - a failed load gives back what it took;
 *  - the control register and the FIFO resets are under the lock;
 *  - errors are errnos, not -1.
 */

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kfifo.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/sched.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>
#include <linux/wait.h>

#define I2S_BASE 0x10067000
#define PAD_BASE 0x10020000 /* pads 0 to 4 are the I2S pins */
#define PADS 5
#define I2S_IRQ 49
#define BLOCK 160 /* samples an interrupt */
#define FIFO_BYTES 3200 /* as the vendor asks; kfifo makes it 2048 */

#define I2S_CTRL 0x00 /* bit 0 on, 1 output, 2 input */
#define I2S_TX 0x10 /* bit 0 underrun, 1 empty, 2 almost empty */
#define I2S_RX 0x20 /* bit 0 overrun, 3 full, 4 almost full */
#define I2S_FIFO 0x38

static void __iomem *i2s;
static DEFINE_SPINLOCK(audio_lock); /* the FIFOs, I2S_CTRL, the opens */
static struct kfifo ai_fifo, ao_fifo;
static DECLARE_WAIT_QUEUE_HEAD(ai_wq);
static DECLARE_WAIT_QUEUE_HEAD(ao_wq);
static int ai_opens, ao_opens;
static struct proc_dir_entry *audio_proc;
static u16 block[BLOCK]; /* the interrupt's */

static struct {
	u32 ai_overrun, ai_full, ai_afull, ai_buffull;
	u32 ao_underrun, ao_empty, ao_aempty, ao_bufempty;
} stats;

static irqreturn_t i2s_interrupt(int irq, void *dev)
{
	u32 tx = readl(i2s + I2S_TX), rx;
	unsigned int n;
	int i;

	if (tx & 1) {
		stats.ao_underrun++;
		writel(0, i2s + I2S_TX);
	}
	if (tx & 4) {
		stats.ao_aempty++;
		memset(block, 0, sizeof(block));
		spin_lock(&audio_lock);
		n = kfifo_out(&ao_fifo, (u8 *)block, sizeof(block));
		spin_unlock(&audio_lock);
		stats.ao_bufempty += sizeof(block) - n;
		wake_up_interruptible(&ao_wq);
		for (i = 0; i < BLOCK; i++)
			writew(block[i], i2s + I2S_FIFO);
	}
	if (tx & 2) {
		stats.ao_empty++;
		for (i = 0; i < 2 * BLOCK; i++)
			writew(0, i2s + I2S_FIFO);
	}
	rx = readl(i2s + I2S_RX);
	if (rx & 0x10) {
		stats.ai_afull++;
		for (i = 0; i < BLOCK; i++)
			block[i] = readw(i2s + I2S_FIFO);
		spin_lock(&audio_lock);
		n = kfifo_in(&ai_fifo, (u8 *)block, sizeof(block));
		spin_unlock(&audio_lock);
		stats.ai_buffull += sizeof(block) - n;
		wake_up_interruptible(&ai_wq);
	}
	if (rx & 8)
		stats.ai_full++;
	if (rx & 1) {
		stats.ai_overrun++;
		writel(0, i2s + I2S_RX);
	}
	writel(1, i2s + I2S_TX);
	writel(1, i2s + I2S_RX);
	return IRQ_HANDLED;
}

static unsigned int fifo_len(struct kfifo *f)
{
	unsigned long flags;
	unsigned int n;

	spin_lock_irqsave(&audio_lock, flags);
	n = kfifo_len(f);
	spin_unlock_irqrestore(&audio_lock, flags);
	return n;
}

static ssize_t ao_write(struct file *file, const char __user *buf, size_t len,
			loff_t *ppos)
{
	unsigned int size = kfifo_size(&ao_fifo), n;
	unsigned long flags;
	u8 *k;

	if (len > size)
		len = size;
	if (file->f_flags & O_NONBLOCK) {
		if (fifo_len(&ao_fifo) >= size)
			return -EAGAIN;
	} else if (wait_event_interruptible(ao_wq, size - fifo_len(&ao_fifo) >=
							   len)) {
		return -ERESTARTSYS;
	}
	k = memdup_user(buf, len);
	if (IS_ERR(k))
		return PTR_ERR(k);
	spin_lock_irqsave(&audio_lock, flags);
	n = kfifo_in(&ao_fifo, k, len);
	spin_unlock_irqrestore(&audio_lock, flags);
	kfree(k);
	return n;
}

static ssize_t ai_read(struct file *file, char __user *buf, size_t len,
		       loff_t *ppos)
{
	unsigned int size = kfifo_size(&ai_fifo), n;
	unsigned long flags;
	ssize_t ret;
	u8 *k;

	if (len > size)
		len = size;
	if (file->f_flags & O_NONBLOCK) {
		if (!fifo_len(&ai_fifo))
			return -EAGAIN;
	} else if (wait_event_interruptible(ai_wq, fifo_len(&ai_fifo) >= len)) {
		return -ERESTARTSYS;
	}
	k = kmalloc(len, GFP_KERNEL);
	if (!k)
		return -ENOMEM;
	spin_lock_irqsave(&audio_lock, flags);
	n = kfifo_out(&ai_fifo, k, len);
	spin_unlock_irqrestore(&audio_lock, flags);
	ret = copy_to_user(buf, k, n) ? -EFAULT : n;
	kfree(k);
	return ret;
}

/* The first open empties the FIFO and starts that way, the last close stops
 * it; input and output share the control register */
static void dir_open(int *opens, struct kfifo *f, u32 on)
{
	unsigned long flags;

	spin_lock_irqsave(&audio_lock, flags);
	if (++*opens == 1) {
		kfifo_reset(f);
		writel(readl(i2s + I2S_CTRL) | on, i2s + I2S_CTRL);
	}
	spin_unlock_irqrestore(&audio_lock, flags);
}

static void dir_release(int *opens, u32 off)
{
	unsigned long flags;

	spin_lock_irqsave(&audio_lock, flags);
	if (--*opens == 0)
		writel(readl(i2s + I2S_CTRL) & ~off, i2s + I2S_CTRL);
	spin_unlock_irqrestore(&audio_lock, flags);
}

static int ai_open(struct inode *inode, struct file *file)
{
	dir_open(&ai_opens, &ai_fifo, 5);
	return 0;
}

static int ai_release(struct inode *inode, struct file *file)
{
	dir_release(&ai_opens, 4);
	return 0;
}

static int ao_open(struct inode *inode, struct file *file)
{
	dir_open(&ao_opens, &ao_fifo, 3);
	return 0;
}

static int ao_release(struct inode *inode, struct file *file)
{
	dir_release(&ao_opens, 2);
	return 0;
}

static int audio_show(struct seq_file *s, void *v)
{
	seq_puts(s, "[AUDIO] openxiongmai xm530_i2s\n\n");
	seq_puts(s, "-----------AI CHN STATUS-----------------\n");
	seq_puts(s, "Chn  Overrun  Full  Afull  BufFull\n");
	seq_printf(s, "%-3d  %-7d  %-4d  %-5d  %-7d\n", 0, stats.ai_overrun,
		   stats.ai_full, stats.ai_afull, stats.ai_buffull);
	seq_puts(s, "-----------AO CHN STATUS-----------------\n");
	seq_puts(s, "Chn  Underrun  Empty  AEmpty  BufEmpty\n");
	seq_printf(s, "%-3d  %-8d  %-5d  %-6d  %-8d\n", 0, stats.ao_underrun,
		   stats.ao_empty, stats.ao_aempty, stats.ao_bufempty);
	return 0;
}

static int audio_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, audio_show, NULL);
}

static const struct file_operations audio_proc_fops = {
	.owner = THIS_MODULE,
	.open = audio_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static const struct file_operations ai_fops = {
	.owner = THIS_MODULE,
	.open = ai_open,
	.release = ai_release,
	.read = ai_read,
};

static const struct file_operations ao_fops = {
	.owner = THIS_MODULE,
	.open = ao_open,
	.release = ao_release,
	.write = ao_write,
};

static struct miscdevice ai_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "ai",
	.fops = &ai_fops,
};

static struct miscdevice ao_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "ao",
	.fops = &ao_fops,
};

static void i2s_setup(void __iomem *pads)
{
	int i;

	for (i = 0; i < PADS; i++)
		writel(0, pads + 4 * i);
	writel(0x30, i2s + I2S_CTRL);
	mdelay(10);
	writel(0, i2s + I2S_TX);
	writel(0, i2s + I2S_RX);
	writel(0x61f, i2s + 0x04);
	writel(0xf, i2s + 0x08);
	writel(8, i2s + 0x0c);
	writel(160, i2s + 0x18);
	writel(160, i2s + 0x2c);
	writel(0x2c2b8, i2s + I2S_CTRL);
}

static int __init xm530_i2s_init(void)
{
	void __iomem *pads;
	int ret = -ENOMEM;

	i2s = ioremap(I2S_BASE, 0x1000);
	pads = ioremap(PAD_BASE, 4 * PADS);
	if (!i2s || !pads)
		goto unmap;
	ret = kfifo_alloc(&ai_fifo, FIFO_BYTES, GFP_KERNEL);
	if (ret)
		goto unmap;
	ret = kfifo_alloc(&ao_fifo, FIFO_BYTES, GFP_KERNEL);
	if (ret)
		goto ai;
	ret = request_irq(I2S_IRQ, i2s_interrupt, 0, "audio", NULL);
	if (ret) {
		pr_err("audio: request irq failed\n");
		goto ao;
	}
	i2s_setup(pads);
	ret = misc_register(&ai_misc);
	if (ret)
		goto irq;
	ret = misc_register(&ao_misc);
	if (ret) {
		misc_deregister(&ai_misc);
		goto irq;
	}
	/* /proc/umap is the MMZ module's */
	audio_proc = proc_create("umap/audio", 0444, NULL, &audio_proc_fops);
	if (!audio_proc)
		pr_warn("audio: no /proc/umap/audio, MMZ not loaded?\n");
	iounmap(pads);
	return 0;
irq:
	pr_err("audio: register device failed\n");
	free_irq(I2S_IRQ, NULL);
ao:
	kfifo_free(&ao_fifo);
ai:
	kfifo_free(&ai_fifo);
unmap:
	if (pads)
		iounmap(pads);
	if (i2s)
		iounmap(i2s);
	return ret;
}

static void __exit xm530_i2s_exit(void)
{
	if (audio_proc)
		remove_proc_entry("umap/audio", NULL);
	misc_deregister(&ai_misc);
	misc_deregister(&ao_misc);
	free_irq(I2S_IRQ, NULL);
	kfifo_free(&ai_fifo);
	kfifo_free(&ao_fifo);
	iounmap(i2s);
}

module_init(xm530_i2s_init);
module_exit(xm530_i2s_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("XM530/XM550 audio input and output, compatible with xm_i2s.ko");
