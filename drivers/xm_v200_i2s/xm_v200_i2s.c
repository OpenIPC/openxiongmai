// SPDX-License-Identifier: GPL-2.0
/*
 * The V200 series' audio input and output (/dev/ai, /dev/ao): a drop-in
 * for the vendor i2s.ko.
 *
 * The audio codec (acodec.ko) talks I2S to three blocks: 0x10065000 the
 * common setup, 0x10066000 input (IRQ 48), 0x10067000 output (IRQ 49).
 * Their interrupts move 160 16-bit samples a time between a block's FIFO
 * (+0x38) and a FIFO in memory: 5 KB of input read from /dev/ai, 2.5 KB of
 * output (5 KB at 16 kHz) written to /dev/ao. Reads and writes wait unless
 * O_NONBLOCK.
 *
 *   ao 0x80044105  the bytes in the output FIFO
 *   ai 0x40044107  the input's 3A state (AEC, ANS, AGC bits), shown in /proc
 *   ai 0x80844109  the capture settings /proc/umap/audio takes
 *                  ("save_chose=N", "save_path=...")
 *   ai 0x40044110  (XM535D1/D2) the sample rate, the clock set for it
 *
 * rate= the sample rate (8000, 16000, 32000): the I2S clock is set for it
 * (256 fs). gpio=N drives that GPIO high while /dev/ao is open (a speaker
 * amplifier). I2S_CHIP=v200 (XM530V200, XM533), xm535 or xm535w1.
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

#define I2S_BASE 0x10065000 /* common, input +0x1000, output +0x2000 */
#define SYS_BASE 0x20000000
#define GPIO_PAD 0x10020000
#define AI_IRQ 48
#define AO_IRQ 49
#define BLOCK 160 /* samples an interrupt */

#define AO_GET_USED 0x80044105
#define AI_SET_3A 0x40044107
#define AI_GET_SAVE 0x80844109
#define AI_SET_RATE 0x40044110

static int rate = 8000;
module_param(rate, int, 0444);
MODULE_PARM_DESC(rate, "Audio record sample rate: 8000, 16000, 32000.");
static int gpio;
module_param(gpio, int, 0444);

static void __iomem *i2s, *sys;
static DEFINE_SPINLOCK(audio_lock);
static struct kfifo ai_fifo, ao_fifo;
static DECLARE_WAIT_QUEUE_HEAD(ai_wq);
static DECLARE_WAIT_QUEUE_HEAD(ao_wq);
static atomic_t ai_opens = ATOMIC_INIT(0), ao_opens = ATOMIC_INIT(0);
static u16 block[BLOCK];

/* /proc */
static struct {
	u32 ai_overrun, ai_full, ai_afull, ai_buffull;
	u32 ao_underrun, ao_empty, ao_aempty, ao_bufempty;
} stats;
static u32 aec, ans, agc;
static struct {
	u32 chose;
	char path[128];
} save;

#define AI(off) (i2s + 0x1000 + (off))
#define AO(off) (i2s + 0x2000 + (off))

static void gpio_set(u32 v)
{
	void __iomem *pad;

	if (gpio <= 0)
		return;
	pad = ioremap(GPIO_PAD + 4 * gpio, 4);
	if (pad) {
		writel(v, pad);
		iounmap(pad);
	}
}

#ifndef I2S_XM535W1
/* the I2S clock, from its PLL and dividers */
static u32 i2sclk(void)
{
	u32 sel = (readl(sys + 0x30) >> 17) & 3, d = readl(sys + 0x64);
	u64 f;
#ifdef I2S_XM535
	u32 r = readl(sys + 0x20 + 8 * sel);

	f = (u64)(r & 0xfff) * 24000000;
	do_div(f, ((r >> 24) & 7) + 1);
	do_div(f, ((r >> 28) & 7) + 1);
	if ((r >> 16) & 0x1f)
		do_div(f, (r >> 16) & 0x1f);
#else
	u32 r20 = readl(sys + 0x20 + 8 * sel), r24 = readl(sys + 0x24 + 8 * sel);

	f = (u64)(((r24 & 0x3ffffff) >> 16) + 16) * 24000000 +
	    (((u64)(r24 & 0xffff) * 24000000) >> 16);
	f >>= (r20 & 0xf) ? 2 : 1;
	do_div(f, ((r20 & 0x7fff) >> 9) + 1);
#endif
	do_div(f, (d >> 24) + 1);
	do_div(f, ((d >> 16) & 0xff) + 1);
	return f;
}
#endif

/* the divider halved (faster) or doubled for the rate, as i2s.ko does */
static void clock_for_rate(void)
{
	u32 d = readl(sys + 0x64), n;
	bool faster;
#ifdef I2S_XM535W1
	if (rate != 16000)
		return;
	faster = true;
#else
	u32 clk = i2sclk();

	if (clk == 2048000 && rate == 16000)
		faster = true;
	else if (clk == 4096000 && rate == 8000)
		faster = false;
	else
		goto out;
#endif
	if (d >> 24 & 1) {
		n = d >> 24;
		n = faster ? ((n + 1) >> 1) - 1 : n * 2 + 1;
		d = (d & 0xffffff) | n << 24;
	} else {
		n = (d & 0xffffff) >> 16;
		n = faster ? ((n + 1) >> 1) - 1 : n * 2 + 1;
		d = (d & 0xff000000) | n << 16 | n << 8 | n;
	}
#ifndef I2S_XM535W1
	writel(1, sys);
#endif
	writel(d, sys + 0x64);
	writel(0, sys);
#ifndef I2S_XM535W1
out:
	pr_info("i2s clk:%dHz\n", i2sclk());
#endif
}

static irqreturn_t i2s1_interrupt(int irq, void *dev)
{
	void __iomem *b = irq == AI_IRQ ? AI(0) : AO(0);
	bool is_ao = irq == AO_IRQ, is_ai = irq == AI_IRQ;
	u32 tx = readl(b + 0x10), rx;
	int i, n;

	if (tx & 1) {
		if (is_ao)
			stats.ao_underrun++;
		writel(0, b + 0x10);
	}
	if (tx & 4) {
		memset(block, 0, sizeof(block));
		if (is_ao) {
			stats.ao_aempty++;
			spin_lock(&audio_lock);
			n = kfifo_out(&ao_fifo, (u8 *)block, sizeof(block));
			spin_unlock(&audio_lock);
			if (n < sizeof(block))
				stats.ao_bufempty++;
			wake_up_interruptible(&ao_wq);
		}
		for (i = 0; i < BLOCK; i++)
			writew(block[i], b + 0x38);
	}
	if (tx & 2) {
		if (is_ao)
			stats.ao_empty++;
		for (i = 0; i < 2 * BLOCK; i++)
			writel(0, b + 0x38);
	}
	rx = readl(b + 0x20);
	if (rx & 0x10) {
		for (i = 0; i < BLOCK; i++)
			block[i] = readw(b + 0x38);
		if (is_ai) {
			stats.ai_afull++;
			spin_lock(&audio_lock);
			n = kfifo_in(&ai_fifo, (u8 *)block, sizeof(block));
			spin_unlock(&audio_lock);
			if (n < sizeof(block))
				stats.ai_buffull++;
			wake_up_interruptible(&ai_wq);
		}
	}
	if ((rx & 8) && is_ai)
		stats.ai_full++;
	if (rx & 1) {
		if (is_ai)
			stats.ai_overrun++;
		writel(0, b + 0x20);
	}
	writel(1, b + 0x10);
	writel(1, b + 0x20);
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
	unsigned long flags;
	unsigned int n;
	u8 *k;

	if (len > kfifo_size(&ao_fifo))
		len = kfifo_size(&ao_fifo);
	if (file->f_flags & O_NONBLOCK) {
		if (fifo_len(&ao_fifo) >= kfifo_size(&ao_fifo))
			return -EAGAIN;
	} else if (wait_event_interruptible(ao_wq, kfifo_size(&ao_fifo) -
						      fifo_len(&ao_fifo) >= len)) {
		return -ERESTARTSYS;
	}
	k = kmalloc(len, GFP_KERNEL);
	if (!k)
		return -ENOMEM;
	if (copy_from_user(k, buf, len)) {
		kfree(k);
		return -EFAULT;
	}
	spin_lock_irqsave(&audio_lock, flags);
	n = kfifo_in(&ao_fifo, k, len);
	spin_unlock_irqrestore(&audio_lock, flags);
	kfree(k);
	return n;
}

static ssize_t ai_read(struct file *file, char __user *buf, size_t len,
		       loff_t *ppos)
{
	unsigned long flags;
	unsigned int n;
	u8 *k;

	if (len > kfifo_size(&ai_fifo))
		len = kfifo_size(&ai_fifo);
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
	if (copy_to_user(buf, k, n))
		n = -EFAULT;
	kfree(k);
	return n;
}

static long ao_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	u32 n;

	if (cmd != AO_GET_USED)
		return -ENOTTY;
	n = fifo_len(&ao_fifo);
	return copy_to_user((void __user *)arg, &n, sizeof(n)) ? -EFAULT : 0;
}

static long ai_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	u32 v;

	switch (cmd) {
	case AI_SET_3A:
		if (get_user(v, (u32 __user *)arg))
			return -EFAULT;
		aec = v & 1;
		ans = (v >> 1) & 1;
		agc = (v >> 2) & 1;
		return 0;
#ifndef I2S_XM535W1
	case AI_GET_SAVE:
		return copy_to_user((void __user *)arg, &save, sizeof(save)) ?
			       -EFAULT : 0;
#endif
#if defined(I2S_XM535) && !defined(I2S_XM535W1)
	case AI_SET_RATE:
		if (get_user(v, (u32 __user *)arg))
			return -EFAULT;
		rate = v;
		pr_info("rate:%d;", rate);
		clock_for_rate();
		return 0;
#endif
	}
	return -ENOTTY;
}

static int ai_open(struct inode *inode, struct file *file)
{
	unsigned long flags;
	u32 v;

	if (atomic_inc_return(&ai_opens) == 1) {
		spin_lock_irqsave(&audio_lock, flags);
		kfifo_reset(&ai_fifo);
		spin_unlock_irqrestore(&audio_lock, flags);
		v = readl(AI(0)) & ~0x20u;
		writel(v, AI(0));
		mdelay(1);
		writel(v | 0x25, AI(0));
	}
	return 0;
}

static int ai_release(struct inode *inode, struct file *file)
{
	if (atomic_dec_return(&ai_opens) == 0)
		writel(readl(AI(0)) & ~5u, AI(0));
	return 0;
}

static int ao_open(struct inode *inode, struct file *file)
{
	unsigned long flags;

	if (atomic_inc_return(&ao_opens) == 1) {
		gpio_set(0xc00);
		spin_lock_irqsave(&audio_lock, flags);
		kfifo_reset(&ao_fifo);
		spin_unlock_irqrestore(&audio_lock, flags);
		writel(readl(AO(0)) | 3, AO(0));
	}
	return 0;
}

static int ao_release(struct inode *inode, struct file *file)
{
	if (atomic_dec_return(&ao_opens) == 0) {
		gpio_set(0x400);
		writel(readl(AO(0)) & ~2u, AO(0));
	}
	return 0;
}

static int audio_show(struct seq_file *s, void *v)
{
	seq_puts(s, "[AUDIO] openxiongmai xm_v200_i2s\n\n");
	seq_puts(s, "-----------AI 3A STATUS-----------------\n");
	seq_printf(s, "AEC: %s       ANS: %s        AGC: %s    \n\n", aec ? "on" : "off",
		   ans ? "on" : "off", agc ? "on" : "off");
	seq_puts(s, "-----------AI CHN STATUS-----------------\n");
	seq_puts(s, "Chn  Overrun  Full  Afull  BufFull\n");
	seq_printf(s, "%-3d  %-7u  %-4u  %-5u  %-7u\n", 0, stats.ai_overrun,
		   stats.ai_full, stats.ai_afull, stats.ai_buffull);
	seq_puts(s, "-----------AO CHN STATUS-----------------\n");
	seq_puts(s, "Chn  Underrun  Empty  AEmpty  BufEmpty\n");
	seq_printf(s, "%-3d  %-8u  %-5u  %-6u  %-8u\n", 1, stats.ao_underrun,
		   stats.ao_empty, stats.ao_aempty, stats.ao_bufempty);
	seq_puts(s, "-----------SAVE AUDIO INFO---------------\n");
	seq_printf(s, "save_chose: %#x       save_path:%s\n", save.chose, save.path);
	return 0;
}

static int audio_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, audio_show, NULL);
}

static ssize_t audio_proc_write(struct file *file, const char __user *buf,
				size_t len, loff_t *ppos)
{
	char k[132], *p;
	int v;

	if (len >= sizeof(k))
		return -EINVAL;
	if (copy_from_user(k, buf, len))
		return -EFAULT;
	k[len] = 0;
	if (strstr(k, "save_chose") && sscanf(k, "save_chose=%d", &v) == 1)
		save.chose = v;
	p = strstr(k, "save_path=");
	if (p) {
		strlcpy(save.path, p + 10, sizeof(save.path));
		strim(save.path);
	}
	return len;
}

static const struct file_operations audio_proc_fops = {
	.owner = THIS_MODULE,
	.open = audio_proc_open,
	.read = seq_read,
	.write = audio_proc_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static const struct file_operations ai_fops = {
	.owner = THIS_MODULE,
	.open = ai_open,
	.release = ai_release,
	.read = ai_read,
	.unlocked_ioctl = ai_ioctl,
};

static const struct file_operations ao_fops = {
	.owner = THIS_MODULE,
	.open = ao_open,
	.release = ao_release,
	.write = ao_write,
	.unlocked_ioctl = ao_ioctl,
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

static void block_init(void __iomem *b, u32 r4, u32 r8, u32 rc, u32 r18,
		       u32 r2c, u32 r30, u32 r0)
{
	writel(0x30, b);
	mdelay(1);
	writel(0, b + 0x10);
	writel(0, b + 0x20);
	writel(r4, b + 0x04);
	writel(r8, b + 0x08);
	if (rc)
		writel(rc, b + 0x0c);
	writel(r18, b + 0x18);
	writel(r2c, b + 0x2c);
	if (r30)
		writel(r30, b + 0x30);
	writel(r0, b);
}

static int __init xm_v200_i2s_init(void)
{
	u32 div = rate == 32000 ? 4 : rate == 8000 || rate == 16000 ? 8 : 0;
	int ret;

	i2s = ioremap(I2S_BASE, 0x3000);
	sys = ioremap(SYS_BASE, 0x100);
	if (!i2s || !sys) {
		ret = -ENOMEM;
		goto unmap;
	}
	gpio_set(0x400);
	block_init(i2s, 0x61f, 0xf, 0x10, 0x3c0, 0x3c0, 0xff001f, 0x200b0);
	block_init(AI(0), 0x605, 0xf, div, 0xa0, 0xa0, 0, 0x2c2b8);
	block_init(AO(0), 0x1b, 0xf, div, 0xa0, 0xa0, 0, 0x2c2b8);
	clock_for_rate();
	if (!proc_create("umap/audio", 0644, NULL, &audio_proc_fops))
		pr_err("audio proc crate failed!\n");
	ret = kfifo_alloc(&ai_fifo, 0x1400, GFP_KERNEL);
	if (!ret)
		ret = kfifo_alloc(&ao_fifo, rate == 16000 ? 0x1400 : 0xa00, GFP_KERNEL);
	if (ret) {
		pr_err("audio: fifo alloc failed\n");
		goto proc;
	}
	ret = request_irq(AI_IRQ, i2s1_interrupt, 0, "audio", NULL);
	if (!ret) {
		ret = request_irq(AO_IRQ, i2s1_interrupt, 0, "audio", NULL);
		if (ret)
			free_irq(AI_IRQ, NULL);
	}
	if (ret) {
		pr_err("audio: request irq failed\n");
		goto fifo;
	}
	ret = misc_register(&ai_misc);
	if (!ret) {
		ret = misc_register(&ao_misc);
		if (ret)
			misc_deregister(&ai_misc);
	}
	if (!ret)
		return 0;
	pr_err("audio: ai register device failed!\n");
	free_irq(AI_IRQ, NULL);
	free_irq(AO_IRQ, NULL);
fifo:
	kfifo_free(&ai_fifo);
	kfifo_free(&ao_fifo);
proc:
	remove_proc_entry("umap/audio", NULL);
unmap:
	if (sys)
		iounmap(sys);
	if (i2s)
		iounmap(i2s);
	return ret;
}

static void __exit xm_v200_i2s_exit(void)
{
	misc_deregister(&ai_misc);
	misc_deregister(&ao_misc);
	free_irq(AI_IRQ, NULL);
	free_irq(AO_IRQ, NULL);
	kfifo_free(&ai_fifo);
	kfifo_free(&ao_fifo);
	remove_proc_entry("umap/audio", NULL);
	iounmap(sys);
	iounmap(i2s);
}

module_init(xm_v200_i2s_init);
module_exit(xm_v200_i2s_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("V200-series audio input and output, compatible with i2s.ko");
