// SPDX-License-Identifier: GPL-2.0
/*
 * The XM533's FFT engine (/dev/ive): a drop-in for the vendor ive.ko.
 * User space hands the engine jobs, up to ten queued; the driver starts
 * the next on the engine's interrupt and lets user space wait for one:
 *
 *   0x80346308  a job: 13 words, 1..7 the engine's registers, 8 its handle
 *   0xc00c630d  query {wait, handle, done}: done once the handle's job has
 *               finished; with wait set, wait for it (no time limit)
 *   0xc0046312  drop every queued job
 *
 * Each returns 0, as ive.ko's: a job that does not fit is dropped with
 * "limit 10 error." in the log.
 *
 * The engine is at 0x21300000, IRQ 95: +0x00..+0x18 a job (+0x14 written
 * last starts it), +0x1C's low byte 9 when it is done, 0 to +0x18 clears
 * that.
 */

#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/list.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/sched.h>
#include <linux/seq_file.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>
#include <linux/wait.h>

#define FFT_BASE 0x21300000
#define FFT_SIZE 0x34
#define FFT_IRQ 95
#define FFT_GO 0x14 /* the job's last word: starts it */
#define FFT_ACK 0x18
#define FFT_STAT 0x1c /* low byte 9: done */
#define FFT_DONE 9
#define N_JOB 10

#define IVE_START 0x80346308
#define IVE_QUERY 0xc00c630d
#define IVE_RESET 0xc0046312

#define ST_QUEUED 1
#define ST_RUNNING 2

struct job {
	u32 w[8]; /* 0 unused, 1..7 the registers' values */
	u32 handle;
	u32 state;
	u32 used;
	struct list_head node;
};

static struct job jobs[N_JOB];
static LIST_HEAD(queue);
static DEFINE_SPINLOCK(fft_lock);
static DECLARE_WAIT_QUEUE_HEAD(fft_wq);
static void __iomem *regs;
static u32 s_fft_handle = ~0u; /* the handle waited for */
static int s_fft_done; /* it has finished */
static int s_fft_busy; /* a job running */
static struct proc_dir_entry *proc;

static void fft_start(const struct job *j)
{
	writel(j->w[1], regs + 0x00);
	writel(j->w[2], regs + 0x04);
	writel(j->w[3], regs + 0x08);
	writel(j->w[4], regs + 0x0c);
	writel(j->w[5], regs + 0x10);
	writel(j->w[7], regs + FFT_ACK);
	writel(j->w[6], regs + FFT_GO);
}

static bool fft_pending(u32 handle)
{
	struct job *j;

	list_for_each_entry(j, &queue, node)
		if (j->handle == handle)
			return true;
	return false;
}

/* The running job finished: drop it, start the next */
static void fft_tasklet_fn(unsigned long d)
{
	struct job *j, *n;
	unsigned long flags;
	bool started = false;

	spin_lock_irqsave(&fft_lock, flags);
	list_for_each_entry_safe(j, n, &queue, node) {
		if (j->state == ST_RUNNING) {
			list_del(&j->node);
			memset(j, 0, sizeof(*j));
		} else if (j->state == ST_QUEUED && !started) {
			fft_start(j);
			j->state = ST_RUNNING;
			started = true;
		}
	}
	if (list_empty(&queue))
		s_fft_busy = 0;
	if (list_empty(&queue) ||
	    (s_fft_handle != ~0u && !fft_pending(s_fft_handle))) {
		s_fft_done = 1;
		if (s_fft_handle != ~0u)
			wake_up(&fft_wq);
	}
	spin_unlock_irqrestore(&fft_lock, flags);
}
static DECLARE_TASKLET(fft_tasklet, fft_tasklet_fn, 0);

static irqreturn_t fft_irq(int irq, void *dev)
{
	if ((readl(regs + FFT_STAT) & 0xff) != FFT_DONE)
		return IRQ_NONE;
	writel(0, regs + FFT_ACK);
	tasklet_schedule(&fft_tasklet);
	return IRQ_HANDLED;
}

static void fft_clear(void)
{
	INIT_LIST_HEAD(&queue);
	memset(jobs, 0, sizeof(jobs));
	s_fft_busy = 0;
	s_fft_done = 0;
	s_fft_handle = ~0u;
}

static long fft_io_start(void __user *arg)
{
	struct job *j = NULL;
	unsigned long flags;
	u32 w[13];
	int i;

	if (copy_from_user(w, arg, sizeof(w))) {
		pr_err("ive_ioctl: copy_from_user error.\n");
		return -EFAULT;
	}
	spin_lock_irqsave(&fft_lock, flags);
	for (i = 0; i < N_JOB; i++)
		if (!jobs[i].used) {
			j = &jobs[i];
			break;
		}
	if (!j) {
		spin_unlock_irqrestore(&fft_lock, flags);
		pr_err("limit %d error.\n", N_JOB);
		return -EBUSY;
	}
	memcpy(j->w, w, sizeof(j->w));
	j->handle = w[8];
	j->used = 1;
	s_fft_done = 0;
	list_add_tail(&j->node, &queue);
	if (!s_fft_busy) {
		j->state = ST_RUNNING;
		s_fft_busy = 1;
		fft_start(j);
	} else {
		j->state = ST_QUEUED;
	}
	spin_unlock_irqrestore(&fft_lock, flags);
	return 0;
}

static long fft_query(void __user *arg)
{
	struct { u32 wait, handle, done; } q;
	unsigned long flags;
	bool pending;

	if (copy_from_user(&q, arg, sizeof(q))) {
		pr_err("ive_ioctl: copy_from_user error.\n");
		return -EFAULT;
	}
	spin_lock_irqsave(&fft_lock, flags);
	pending = fft_pending(q.handle);
	if (pending)
		s_fft_handle = q.handle;
	spin_unlock_irqrestore(&fft_lock, flags);
	q.done = !pending;
	if (pending && q.wait) {
		if (wait_event_interruptible(fft_wq, s_fft_done))
			return 0; /* as ive.ko: a signal, nothing back */
		spin_lock_irqsave(&fft_lock, flags);
		s_fft_done = 0;
		s_fft_handle = ~0u;
		spin_unlock_irqrestore(&fft_lock, flags);
		q.done = 1;
	}
	if (copy_to_user(arg, &q, sizeof(q)))
		pr_err("ive_ioctl: copy_to_user error.\n");
	return 0;
}

static long ive_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	unsigned long flags;

	switch (cmd) {
	case IVE_START:
		/* as ive.ko: 0 even for a job it could not take */
		fft_io_start((void __user *)arg);
		return 0;
	case IVE_QUERY:
		fft_query((void __user *)arg);
		return 0;
	case IVE_RESET:
		spin_lock_irqsave(&fft_lock, flags);
		fft_clear();
		spin_unlock_irqrestore(&fft_lock, flags);
		return 0;
	}
	return 0;
}

static int ive_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int ive_release(struct inode *inode, struct file *file)
{
	return 0;
}

static const struct file_operations ive_fops = {
	.owner = THIS_MODULE,
	.open = ive_open,
	.release = ive_release,
	.unlocked_ioctl = ive_ioctl,
};

static struct miscdevice ive_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "ive",
	.fops = &ive_fops,
};

static int ive_proc_show(struct seq_file *m, void *v)
{
	seq_puts(m, "[IVE] BUILD TIME:[open_ive]\n");
	seq_puts(m, "-------------------------------IVE TASK INFO-------------------------\n");
	return 0;
}

static int ive_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, ive_proc_show, NULL);
}

static const struct file_operations ive_proc_fops = {
	.owner = THIS_MODULE,
	.open = ive_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int __init ive_init(void)
{
	int ret;

	if (!request_mem_region(FFT_BASE, FFT_SIZE, "xm580_fft")) {
		pr_err("ive_kernel: ive: request mem  fail\n");
		return -EBUSY;
	}
	regs = ioremap(FFT_BASE, FFT_SIZE);
	if (!regs) {
		pr_err("ive_kernel: ive: failed to ioremap HW regs\n");
		ret = -EBUSY;
		goto release;
	}
	ret = misc_register(&ive_misc);
	if (ret) {
		pr_err("ive_kernel: driver register fail\n");
		goto unmap;
	}
	ret = request_irq(FFT_IRQ, fft_irq, 0, "fft", NULL);
	if (ret) {
		pr_err("ive_kernel: request irq fail\n");
		ret = -EIO;
		goto misc;
	}
	proc = proc_create("umap/ive", 0444, NULL, &ive_proc_fops);
	return 0;
misc:
	misc_deregister(&ive_misc);
unmap:
	iounmap(regs);
release:
	release_mem_region(FFT_BASE, FFT_SIZE);
	return ret;
}

static void __exit ive_exit(void)
{
	if (proc)
		remove_proc_entry("umap/ive", NULL);
	free_irq(FFT_IRQ, NULL);
	tasklet_kill(&fft_tasklet);
	misc_deregister(&ive_misc);
	iounmap(regs);
	release_mem_region(FFT_BASE, FFT_SIZE);
}

module_init(ive_init);
module_exit(ive_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("XM533 FFT engine (open ive.ko)");
