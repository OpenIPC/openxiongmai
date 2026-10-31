// SPDX-License-Identifier: GPL-2.0
/*
 * The XM530V200-series CNN accelerator (/dev/nn): a drop-in for the vendor
 * nn.ko. User space builds the network's command lists in MMZ memory and
 * hands the accelerator one command at a time; the driver queues them,
 * starts the next when the accelerator interrupts, and lets user space wait
 * for its own:
 *
 *   0x80186300  submit a command (XM530V200: 6 words, the last its handle)
 *   0x80246300  submit a command (XM533, XM535: 9 words, the last its handle)
 *   0xc0046301  query {wait, handle, done}: done once the handle's command
 *               has finished; with wait set, wait for it
 *   0xc0046302  drop every queued command
 *
 * Built per chip (NN_CHIP=v200|xm533|xm535): the XM530V200 and the XM535
 * wait for one handle at a time, as their nn.ko; the XM533's tells the
 * waiters apart by process too, and resets the accelerator when one of
 * them times out. config_list_num= (the queue's depth), cnn_base= and
 * int_irq= as nn.ko's.
 */

#include <linux/delay.h>
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
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>
#include <linux/wait.h>

#define NN_SIZE 400
#define REG_CMD 0x00
#define REG_CTRL 0x04 /* 4: clear the interrupt; then the command's start */
#define REG_STAT 0x08 /* bit 0: done */

#if defined(NN_XM533)
#define CMD_WORDS 9
#define DEF_LIST 0x40
#define DEF_BASE 0x21500000
#define WAIT_SEC 5
#elif defined(NN_XM535)
#define CMD_WORDS 9
#define DEF_LIST 0xa
#define DEF_BASE 0x21500000
#define WAIT_SEC 100
#else
#define NN_V200
#define CMD_WORDS 6
#define DEF_LIST 0x40
#define DEF_BASE 0x21200000
#define WAIT_SEC 50
#endif

#define NN_IOC_SUBMIT _IOR('c', 0, u32[CMD_WORDS]) /* 0x80186300, 0x80246300 */
#define NN_IOC_QUERY 0xc0046301
#define NN_IOC_RESET 0xc0046302

#define ST_QUEUED 1
#define ST_RUNNING 2

static unsigned int config_list_num = DEF_LIST;
module_param(config_list_num, uint, 0444);
static unsigned int cnn_base = DEF_BASE;
module_param(cnn_base, uint, 0444);
static unsigned int int_irq = 0x4b;
module_param(int_irq, uint, 0444);

struct nn_cmd {
	u32 handle; /* the command's last word */
	u32 state;
	u32 used;
	pid_t tgid; /* the XM533's: whose command */
	u32 cmd[CMD_WORDS];
	struct list_head node;
};

static struct nn_cmd *cmds;
static LIST_HEAD(queue);
static DEFINE_SPINLOCK(nn_lock);
static DECLARE_WAIT_QUEUE_HEAD(nn_wq);
static void __iomem *regs;
static int irq_ok;
static struct proc_dir_entry *proc;
#ifndef NN_XM533
static u32 s_nn_handle = ~0u; /* the handle waited for */
static int s_nn_op; /* it has finished */
#endif

static void nn_start(const u32 *c)
{
	writel(c[0], regs + REG_CMD);
#ifdef NN_V200
	writel(c[2], regs + 0x10);
	writel(c[3], regs + 0x14);
	writel(c[4], regs + 0x18);
	writel(4, regs + REG_CTRL);
	writel(c[1], regs + REG_CTRL);
#else
	writel(4, regs + REG_CTRL);
	writel(c[3], regs + 0x0c);
	writel(c[2], regs + 0x10);
	writel(c[4], regs + 0x14);
	writel(c[5], regs + 0x18);
	writel(c[6], regs + 0x24);
	writel(c[7], regs + 0x28);
	writel(c[1] | 4, regs + REG_CTRL);
#endif
}

#ifndef NN_XM535
static void wr(u32 addr, u32 v)
{
	void __iomem *p = ioremap(addr, 4);

	if (p) {
		writel(v, p);
		iounmap(p);
	}
}
#endif

/* Reset the accelerator: at load and when a command never finished */
static void nn_reset_hw(int timeout)
{
#ifdef NN_XM535
	void __iomem *p;

	if (!timeout)
		return; /* nn.ko's nn_soft_rst() does nothing on the XM535 */
	p = ioremap(0x20000124, 4);
	if (!p)
		return;
	writel(0xd590000, p);
	mdelay(100);
	writel(1, p);
	mdelay(100);
	iounmap(p);
#else
	wr(0x20000000, 1); /* the system controller unlocked */
#ifdef NN_XM533
	wr(0x20000124, 0xd590000);
	wr(0x20000124, 3);
	if (timeout)
		wr(0x20000068, 0xf010703);
	wr(0x200506a0, 1);
#else
	wr(0x20000124, 0xd590000);
	wr(0x20000124, 1);
	wr(0x20000068, 1);
	wr(0x200505f0, 1);
#endif
	wr(0x20000000, 0);
#endif
}

static void nn_clear(void)
{
	INIT_LIST_HEAD(&queue);
	memset(cmds, 0, config_list_num * sizeof(*cmds));
#ifndef NN_XM533
	s_nn_handle = ~0u;
	s_nn_op = 0;
#endif
}

static bool nn_pending(u32 handle, pid_t tgid)
{
	struct nn_cmd *c;

	list_for_each_entry(c, &queue, node)
		if (c->handle == handle && c->tgid == tgid)
			return true;
	return false;
}

/*
 * The accelerator finished the running command: drop it and start the
 * next. (nn.ko starts every queued command here, one over the other; the
 * accelerator runs one at a time.)
 */
static void nn_done(void)
{
	struct nn_cmd *c, *n;
	unsigned long flags;
	bool started = false;

	spin_lock_irqsave(&nn_lock, flags);
	list_for_each_entry_safe(c, n, &queue, node) {
		if (c->state == ST_RUNNING) {
			list_del(&c->node);
			memset(c, 0, sizeof(*c));
		} else if (c->state == ST_QUEUED && !started) {
			nn_start(c->cmd);
			c->state = ST_RUNNING;
			started = true;
		}
	}
#ifdef NN_XM533
	wake_up(&nn_wq);
#else
	if (s_nn_handle != ~0u && !nn_pending(s_nn_handle, 0)) {
		s_nn_op = 1;
		wake_up(&nn_wq);
	} else if (list_empty(&queue)) {
		s_nn_op = 0;
	}
#endif
	spin_unlock_irqrestore(&nn_lock, flags);
}

#ifndef NN_XM533
static void nn_tasklet_fn(unsigned long d)
{
	nn_done();
}
static DECLARE_TASKLET(nn_tasklet, nn_tasklet_fn, 0);
#endif

static irqreturn_t nn_irq(int irq, void *dev)
{
	if (!(readl(regs + REG_STAT) & 1))
		return IRQ_NONE;
	writel(4, regs + REG_CTRL);
#ifdef NN_XM533
	nn_done();
#else
	tasklet_schedule(&nn_tasklet);
#endif
	return IRQ_HANDLED;
}

static long nn_submit(void __user *arg)
{
	struct nn_cmd *c = NULL;
	unsigned long flags;
	u32 cmd[CMD_WORDS];
	unsigned int i;

	if (copy_from_user(cmd, arg, sizeof(cmd))) {
		pr_err("nn_kernel: copy from user fail\n");
		return -EFAULT;
	}
	spin_lock_irqsave(&nn_lock, flags);
	for (i = 0; i < config_list_num; i++)
		if (!cmds[i].used) {
			c = &cmds[i];
			break;
		}
	if (!c) {
		spin_unlock_irqrestore(&nn_lock, flags);
		pr_err("nn_kernel: the command list is full (%u)\n",
		       config_list_num);
		return -EBUSY;
	}
	memcpy(c->cmd, cmd, sizeof(cmd));
	c->used = 1;
	c->handle = cmd[CMD_WORDS - 1];
#ifdef NN_XM533
	c->tgid = current->tgid;
#endif
	if (list_empty(&queue)) {
		list_add_tail(&c->node, &queue);
		c->state = ST_RUNNING;
		nn_start(c->cmd);
	} else {
		list_add_tail(&c->node, &queue);
		c->state = ST_QUEUED;
	}
	spin_unlock_irqrestore(&nn_lock, flags);
	return 0;
}

static long nn_query(void __user *arg)
{
	struct { u32 wait, handle, done; } q;
	unsigned long flags;
	pid_t tgid = 0;
	bool pending;
	long t;

	if (copy_from_user(&q, arg, sizeof(q)))
		return -EFAULT;
#ifdef NN_XM533
	tgid = current->tgid;
#endif
	spin_lock_irqsave(&nn_lock, flags);
	pending = nn_pending(q.handle, tgid);
#ifndef NN_XM533
	if (pending)
		s_nn_handle = q.handle;
#endif
	spin_unlock_irqrestore(&nn_lock, flags);
	q.done = !pending;
	if (q.wait && pending) {
#ifdef NN_XM533
		t = wait_event_interruptible_timeout(nn_wq,
			({ bool p; spin_lock_irqsave(&nn_lock, flags);
			   p = nn_pending(q.handle, tgid);
			   spin_unlock_irqrestore(&nn_lock, flags); !p; }),
			WAIT_SEC * HZ);
#else
		t = wait_event_interruptible_timeout(nn_wq, s_nn_op,
						     WAIT_SEC * HZ);
#endif
		spin_lock_irqsave(&nn_lock, flags);
		if (!t) {
			pr_err("cnn wait for timeout\n");
#ifdef NN_XM533
			nn_clear();
#else
			s_nn_handle = ~0u;
			s_nn_op = 0;
#endif
		} else {
#ifndef NN_XM533
			s_nn_handle = ~0u;
			s_nn_op = 0;
#endif
		}
		spin_unlock_irqrestore(&nn_lock, flags);
		q.done = t != 0;
		if (!t) {
			if (copy_to_user(arg, &q, sizeof(q)))
				pr_err("nn_kernel: copy to user fail\n");
			nn_reset_hw(1);
			return 0;
		}
	}
	if (copy_to_user(arg, &q, sizeof(q)))
		pr_err("nn_kernel: copy to user fail\n");
	return 0;
}

static long nn_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
#ifndef NN_XM533
	unsigned long flags;
#endif

	switch (cmd) {
	case NN_IOC_SUBMIT:
		return nn_submit((void __user *)arg);
	case NN_IOC_QUERY:
		return nn_query((void __user *)arg);
	case NN_IOC_RESET:
#ifndef NN_XM533 /* the XM533's nn.ko ignores it */
		spin_lock_irqsave(&nn_lock, flags);
		nn_clear();
		spin_unlock_irqrestore(&nn_lock, flags);
#endif
		return 0;
	}
	return 0;
}

static int nn_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int nn_release(struct inode *inode, struct file *file)
{
	return 0;
}

static const struct file_operations nn_fops = {
	.owner = THIS_MODULE,
	.open = nn_open,
	.release = nn_release,
	.unlocked_ioctl = nn_ioctl,
};

static struct miscdevice nn_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "nn",
	.fops = &nn_fops,
};

static int nn_proc_show(struct seq_file *m, void *v)
{
	int i;

	seq_puts(m, "[CNN] BUILD TIME:[open_nn]\n");
	seq_puts(m, "-------------------------------CNN TASK INFO-------------------------\n");
	for (i = 0; i <= 10; i++)
		seq_printf(m, "apb data %-2d---------------------------------------------------0x%08x\n",
			   i, readl(regs + i * 4));
	return 0;
}

static int nn_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, nn_proc_show, NULL);
}

static const struct file_operations nn_proc_fops = {
	.owner = THIS_MODULE,
	.open = nn_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int __init nn_init(void)
{
	int ret;

	if (!config_list_num)
		return -EINVAL;
	cmds = kcalloc(config_list_num, sizeof(*cmds), GFP_KERNEL);
	if (!cmds)
		return -ENOMEM;
	nn_reset_hw(0);
	if (!request_mem_region(cnn_base, NN_SIZE, "nn")) {
		pr_err("nn_kernel: NN: request mem  fail\n");
		ret = -EBUSY;
		goto free;
	}
	regs = ioremap(cnn_base, NN_SIZE);
	if (!regs) {
		pr_err("nn_kernel: nn: failed to ioremap HW regs\n");
		ret = -EBUSY;
		goto release;
	}
	ret = misc_register(&nn_misc);
	if (ret) {
		pr_err("nn_kernel: driver register fail\n");
		goto unmap;
	}
	ret = request_irq(int_irq, nn_irq, 0, "nn", NULL);
	if (ret) {
		pr_err("nn_kernel: request irq fail\n");
		ret = -EIO;
		goto misc;
	}
	irq_ok = 1;
	proc = proc_create("umap/cnn", 0444, NULL, &nn_proc_fops);
	return 0;
misc:
	misc_deregister(&nn_misc);
unmap:
	iounmap(regs);
release:
	release_mem_region(cnn_base, NN_SIZE);
free:
	kfree(cmds);
	return ret;
}

static void __exit nn_exit(void)
{
	if (proc)
		remove_proc_entry("umap/cnn", NULL);
	if (irq_ok)
		free_irq(int_irq, NULL);
#ifndef NN_XM533
	tasklet_kill(&nn_tasklet);
#endif
	misc_deregister(&nn_misc);
	writel(0, regs + REG_CMD);
	iounmap(regs);
	release_mem_region(cnn_base, NN_SIZE);
	kfree(cmds);
}

module_init(nn_init);
module_exit(nn_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("XM530V200-series CNN accelerator (open nn.ko)");
