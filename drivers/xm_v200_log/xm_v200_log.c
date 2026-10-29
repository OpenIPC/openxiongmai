// SPDX-License-Identifier: GPL-2.0
/*
 * The V200 series' media log (/dev/log, /proc/umap/logmpp): a drop-in for
 * the vendor log.ko.
 *
 * /dev/log is a 32 KB FIFO of text: writes append (the oldest dropped when
 * it is full), reads wait for some. Each media module has a log level
 * (1..7, 3 to start): /proc/umap/logmpp shows them and takes
 * "<module> <level>", and ioctl 0x800c4c00 returns the table, 13 entries
 * of {level, name[8]}.
 */

#include <linux/fs.h>
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

#define LOG_FIFO (32 * 1024)
#define LOG_GET_LEVELS 0x800c4c00
#define N_MOD 13

struct level {
	s32 level;
	char name[8];
};

static struct level levels[N_MOD] = {
	{3, "sys"}, {3, "vb"},	{3, "ai"},   {3, "ao"},	 {3, "aenc"},
	{3, "adec"}, {3, "rgn"}, {3, "vi"},   {3, "vo"},	 {3, "venc"},
	{3, "vdec"}, {3, "vpss"}, {3, "var"},
};

static DEFINE_KFIFO(fifo, char, LOG_FIFO);
static DEFINE_SPINLOCK(fifo_lock);
static DECLARE_WAIT_QUEUE_HEAD(log_wq);

static ssize_t log_write(struct file *file, const char __user *buf,
			 size_t len, loff_t *ppos)
{
	unsigned long flags;
	char *k;
	size_t n;

	if (len > LOG_FIFO)
		len = LOG_FIFO;
	k = kmalloc(len, GFP_KERNEL);
	if (!k)
		return -ENOMEM;
	if (copy_from_user(k, buf, len)) {
		kfree(k);
		return -EFAULT;
	}
	spin_lock_irqsave(&fifo_lock, flags);
	/* room for it: the oldest out */
	if (kfifo_avail(&fifo) < len) {
		size_t drop = len - kfifo_avail(&fifo);

		while (drop--) {
			char c;

			if (!kfifo_get(&fifo, &c))
				break;
		}
	}
	n = kfifo_in(&fifo, k, len);
	spin_unlock_irqrestore(&fifo_lock, flags);
	wake_up_interruptible(&log_wq);
	kfree(k);
	return n;
}

static ssize_t log_read(struct file *file, char __user *buf, size_t len,
			loff_t *ppos)
{
	unsigned long flags;
	char *k;
	size_t n;

	if (wait_event_interruptible(log_wq, !kfifo_is_empty(&fifo)))
		return -ERESTARTSYS;
	if (len > LOG_FIFO)
		len = LOG_FIFO;
	k = kmalloc(len, GFP_KERNEL);
	if (!k)
		return -ENOMEM;
	spin_lock_irqsave(&fifo_lock, flags);
	n = kfifo_out(&fifo, k, len);
	spin_unlock_irqrestore(&fifo_lock, flags);
	if (copy_to_user(buf, k, n))
		n = -EFAULT;
	kfree(k);
	return n;
}

static long log_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	if (cmd == 0x5401) /* TCGETS: not a terminal, as log.ko answers */
		return 0;
	if (cmd != LOG_GET_LEVELS)
		return 0;
	return copy_to_user((void __user *)arg, levels, sizeof(levels)) ?
		       -EFAULT : 0;
}

static int levels_show(struct seq_file *s, void *v)
{
	int i;

	seq_puts(s, "[LOG] openxiongmai xm_v200_log\n\n");
	seq_puts(s, "-------------------------------------------------------------\n");
	for (i = 0; i < N_MOD; i++)
		seq_printf(s, "%-8s = %d\n", levels[i].name, levels[i].level);
	return 0;
}

static int levels_open(struct inode *inode, struct file *file)
{
	return single_open(file, levels_show, NULL);
}

/* "<module> <level>" */
static ssize_t levels_write(struct file *file, const char __user *buf,
			    size_t len, loff_t *ppos)
{
	char k[32];
	int i;

	if (len >= sizeof(k))
		return -EINVAL;
	if (copy_from_user(k, buf, len))
		return -EFAULT;
	k[len] = 0;
	for (i = 0; i < N_MOD; i++) {
		size_t n = strlen(levels[i].name);
		long v;

		if (strncmp(k, levels[i].name, n) || k[n] != ' ')
			continue;
		if (!kstrtol(strim(k + n + 1), 10, &v) && v >= 1 && v <= 7)
			levels[i].level = v;
	}
	return len;
}

static const struct file_operations levels_fops = {
	.owner = THIS_MODULE,
	.open = levels_open,
	.read = seq_read,
	.write = levels_write,
	.llseek = seq_lseek,
	.release = single_release,
};

static const struct file_operations log_fops = {
	.owner = THIS_MODULE,
	.read = log_read,
	.write = log_write,
	.unlocked_ioctl = log_ioctl,
};

static struct miscdevice log_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "log",
	.fops = &log_fops,
};

static int __init xm_v200_log_init(void)
{
	int ret;

	proc_create("umap/logmpp", 0644, NULL, &levels_fops);
	ret = misc_register(&log_misc);
	if (ret) {
		pr_err("log: register device failed!\n");
		remove_proc_entry("umap/logmpp", NULL);
	}
	return ret;
}

static void __exit xm_v200_log_exit(void)
{
	misc_deregister(&log_misc);
	remove_proc_entry("umap/logmpp", NULL);
}

module_init(xm_v200_log_init);
module_exit(xm_v200_log_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("V200-series media log, compatible with log.ko");
