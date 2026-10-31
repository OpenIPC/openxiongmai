// SPDX-License-Identifier: GPL-2.0
/*
 * xm_hevc: the V200 series' H.265 encoder engine, a drop-in replacement for
 * the vendor's hevc.ko (built as open_hevc.ko).
 *
 * The engine's registers are at 0x21000000, its interrupt is 77. User space
 * (Sofia, or libxmv200) works out every register of a picture itself; this
 * driver only runs the jobs, through /dev/h265:
 *  - 0x4803 / 0x4804 take and give back the engine (a semaphore, which a
 *    closing holder gives back too);
 *  - 0x40044806 resets it, with the engine's key (the "version");
 *  - the job ioctl carries a list of {value, write} pairs, entry n for the
 *    register at 4n. Each entry with write set is kept; the kept ones are
 *    written, and the engine started;
 *  - the wait ioctl sleeps until the interrupt says the picture is done and
 *    gives back the job's record, its word 11 the bytes the engine wrote.
 *
 * Two engines, as the vendor builds them (layout=, or HEVC_LAYOUT at build
 * time):
 *  - v3 (XM530V200, XM533): a 56-byte job (0x40384805) with the list at
 *    word 13, 189 entries; registers 2..188 but 10..12 are written, register
 *    1 starts it; status at 0x58, the length at 0x1d8; reset pulses the
 *    engine's reset in the clock block (0x20000120: 5, then 7);
 *  - v1 (XM535D1/D2/W1): a 72-byte job (0x40484805) with the list at word
 *    17, 350 entries; registers 2..349 are written, register 0 starts it;
 *    status at 0x74, the length at 0xf8. Reset halts the engine first. Its
 *    jobs can run in line mode (word 13: the line block at 0x21010000 paces
 *    the engine with the capture, mode 1 in word 14) or continue into a next
 *    output buffer when one fills (mode 4, words 15 and 16).
 */

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/ratelimit.h>
#include <linux/semaphore.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/wait.h>

#define HEVC_BASE 0x21000000
#define HEVC_LINE_BASE 0x21010000 /* v1: the line block */
#define HEVC_RESET_REG 0x20000120 /* the engine's reset, in the clock block */
#define HEVC_IRQ 77

#define HEVC_LOCK 0x4803
#define HEVC_UNLOCK 0x4804
#define HEVC_RESET 0x40044806

#ifndef HEVC_DEFAULT_LAYOUT
#define HEVC_DEFAULT_LAYOUT "v3"
#endif
static char *layout = HEVC_DEFAULT_LAYOUT;
module_param(layout, charp, 0444);
MODULE_PARM_DESC(layout, "v3: XM530V200/XM533 engine; v1: XM535's");
static int debug;
module_param(debug, int, 0644);
MODULE_PARM_DESC(debug, "log the line mode");

struct hevc_layout {
	const char *name;
	unsigned int job_cmd, wait_cmd;
	unsigned int job_words; /* the job record */
	unsigned int out_words; /* what the wait gives back */
	unsigned int list_word; /* the list's user address in the job */
	unsigned int n_regs;	/* entries in the list */
	u32 key;		/* the reset's */
	unsigned int start;	/* the register that starts a picture */
	unsigned int status, length; /* register offsets */
	u32 reset_pulse;	/* first value of the reset register */
};

static const struct hevc_layout layouts[] = {
	{"v3", 0x40384805, 0xc0344802, 14, 13, 13, 189, 0x10002, 1, 0x58,
	 0x1d8, 5},
	{"v1", 0x40484805, 0xc0444802, 18, 17, 17, 350, 0x04000000, 0, 0x74,
	 0xf8, 6},
};
static const struct hevc_layout *L;

/* the job's words this driver reads */
#define JOB_CHN 1  /* has to be 0 */
#define JOB_LINE 13 /* v1: the line mode's line count, 0 for none */
#define JOB_MODE 14 /* v1: 1 line mode, 3 a mode the done status stands for,
		     * 4 continue into the next output buffer */
#define OUT_LEN 11 /* bytes written */

#define MAX_REGS 350
struct hevc_entry {
	u32 val, write;
};

static void __iomem *regs, *line, *reset_reg;
static struct hevc_entry *list;	/* the job's list, as copied */
static u32 shadow[MAX_REGS], dirty[MAX_REGS];
static u32 now[18];			/* the running job's record */
static bool done, failed, itf;
static unsigned int irqs_unhandled;
static DECLARE_WAIT_QUEUE_HEAD(waitq);
static struct semaphore engine;
static struct file *holder;

static u32 rd(unsigned int off)
{
	return readl(regs + off);
}

static void wr(unsigned int off, u32 v)
{
	writel(v, regs + off);
}

static void hevc_reset(void)
{
	int i;

	if (L->start == 0) {
		/* v1: halt it, and wait (up to 2 ms) for it to stop */
		wr(0x14, 3);
		wr(0x24, 2);
		wr(0x38, 2);
		wr(0x530, 1);
		for (i = 0; i < 200; i++) {
			if (((rd(0x70) & 3) >> 1) || !rd(0x64))
				break;
			udelay(10);
		}
		if (i == 200)
			pr_warn("hevc: halt wait timeout\n");
		else if (i > 10)
			pr_warn("hevc: halt more than 100us\n");
	}
	writel(L->reset_pulse, reset_reg);
	udelay(100);
	writel(7, reset_reg);
	memset(shadow, 0, sizeof(shadow));
	memset(dirty, 0, sizeof(dirty));
	memset(now, 0, sizeof(now));
	done = failed = false;
}

/* the kept registers, then start */
static void hevc_begin(void)
{
	unsigned int i;

	for (i = 2; i < L->n_regs; i++) {
		if (L->start == 1 && i >= 10 && i <= 12)
			continue; /* v3: not the engine's to set */
		if (dirty[i]) {
			wr(i * 4, shadow[i]);
			dirty[i] = 0;
		}
	}
	if (L->start == 0 && now[JOB_MODE] == 1) {
		/* line mode: start once the capture is a line further */
		u32 from = readl(line + 0x40), at = from;

		for (i = 0; i < 100; i++) {
			at = readl(line + 0x40);
			if (at != from && at)
				break;
			udelay(10);
		}
		if (i == 100)
			pr_warn("hevc: linemode timeout line_now=%d\n", at);
	}
	wr(L->start * 4, 1);
	if (debug && L->start == 0)
		pr_info("hevc: l %d\n", readl(line + 0x40));
}

/* v1: the engine stopped on an error; leave it stopped */
static void hevc_stop_v1(void)
{
	wr(0x00, 0);
	wr(0x1c, 0);
	wr(0x20, 0);
	wr(0x28, 0);
	wr(0x0c, 3);
	wr(0x10, 3);
	wr(0x18, 3);
}

static void hevc_finish(u32 status, bool ok)
{
	if (ok) {
		itf = false;
		done = true;
	} else {
		failed = true;
		pr_err_ratelimited("hevc: itfFlag == 0 && status == %#x\n",
				   status);
	}
	wake_up(&waitq);
}

static irqreturn_t hevc_irq_v3(void)
{
	u32 st = rd(0x58);

	if (st == 2) {
		itf = true;
		wr(0x30, 2);
	} else if (st & 1) {
		wr(0x30, 3);
		now[OUT_LEN] = rd(L->length);
		irqs_unhandled = 0;
		wr(0x04, 0);
		hevc_finish(st, !(st == 1 && !itf));
	}
	if (rd(0x54) & 3) {
		irqs_unhandled = 0;
		wr(0x2c, 3);
	}
	if (rd(0x50) & 3) {
		wr(0x04, 0);
		wr(0x08, 0);
		wr(0x0c, 0);
		wr(0x10, 0);
		wr(0x28, 3);
		wr(0x2c, 3);
		wr(0x30, 3);
		failed = true;
		wake_up(&waitq);
	}
	return IRQ_HANDLED;
}

static irqreturn_t hevc_irq_v1(void)
{
	u32 st = rd(0x74), l;

	if (st == 2) {
		itf = true;
		wr(0x18, 2);
	} else if (st & 1) {
		u32 len = rd(L->length);

		wr(0x18, 3);
		now[OUT_LEN] = now[JOB_MODE] == 4 ? now[OUT_LEN] + len : len;
		irqs_unhandled = 0;
		wr(0x00, 0);
		hevc_finish(st, now[JOB_MODE] == 3 || st != 1 || itf);
	}
	if (rd(0x6c) & 3) {
		irqs_unhandled = 0;
		wr(0x10, 3);
	}
	if (rd(0x70) & 1) {
		/* an output buffer is full: in mode 4, on into the next */
		irqs_unhandled = 0;
		wr(0x14, 1);
		if (now[JOB_MODE] == 4) {
			now[OUT_LEN] = rd(L->length);
			wr(0xa0, now[15]);
			wr(0xa4, now[16]);
			wr(0xd0, now[6] + now[2] + now[OUT_LEN]);
			wr(0x04, 1);
		}
	}
	if (rd(0x68) & 3) {
		pr_err_ratelimited("hevc: err %x\n", rd(0x68));
		hevc_stop_v1();
		failed = true;
		wake_up(&waitq);
	}
	l = readl(line + 0x28);
	if (l) {
		writel(0, line + 0x24);
		writel(0, line + 0x20);
		if (now[JOB_MODE] == 1) {
			if (debug)
				pr_info("hevc: line %d LINE_INT_OUT=%x\n",
					readl(line + 0x40), l);
			hevc_stop_v1();
			failed = true;
			wake_up(&waitq);
		}
	}
	return IRQ_HANDLED;
}

static irqreturn_t hevc_irq(int irq, void *dev)
{
	if (++irqs_unhandled > 10)
		pr_err_ratelimited("hevc: irq error %d\n", irqs_unhandled);
	return L->start ? hevc_irq_v3() : hevc_irq_v1();
}

static long hevc_job(void __user *arg)
{
	u32 job[18];
	unsigned int i;

	if (copy_from_user(job, arg, L->job_words * 4)) {
		pr_err("hevc: copy_from_user failed\n");
		return -EFAULT;
	}
	if (job[JOB_CHN]) {
		pr_err("h265: chn %d is invalid\n", job[JOB_CHN]);
		return -1;
	}
	if (copy_from_user(list, (void __user *)(uintptr_t)job[L->list_word],
			   L->n_regs * sizeof(*list))) {
		pr_err("h265: copy_from_user failed1\n");
		return -EFAULT;
	}
	for (i = 0; i < L->n_regs; i++)
		if (list[i].write) {
			shadow[i] = list[i].val;
			dirty[i] = 1;
		}
	memcpy(now, job, L->out_words * 4);
	now[OUT_LEN] = now[OUT_LEN + 1] = 0;
	if (L->start == 0) {
		if (job[JOB_LINE]) {
			writel(job[JOB_LINE], line + 0x00);
			writel(0, line + 0x20);
			writel(0xf, line + 0x24);
			writel(0x300000, line + 0x04);
			writel(2, line + 0x08);
		} else {
			writel(0, line + 0x24);
		}
	}
	hevc_begin();
	return 0;
}

static long hevc_wait(void __user *arg)
{
	/* a failed picture fails every wait until the next reset, as the
	 * vendor's */
	if (!done) {
		if (failed)
			return -1;
		if (wait_event_interruptible(waitq, done || failed))
			return -ERESTARTSYS;
	}
	if (failed)
		return -1;
	done = false;
	if (arg && copy_to_user(arg, now, L->out_words * 4)) {
		pr_err("hevc: copy_to_user failed\n");
		return -EFAULT;
	}
	return 0;
}

static long hevc_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	void __user *uarg = (void __user *)arg;
	u32 key;

	if (cmd == HEVC_RESET) {
		if (copy_from_user(&key, uarg, 4))
			return -EFAULT;
		if (key != L->key) {
			pr_err("hevc: Version mismatch\n");
			return -1;
		}
		hevc_reset();
		return 0;
	}
	if (cmd == HEVC_LOCK) {
		if (down_interruptible(&engine))
			return -ERESTARTSYS;
		holder = file;
		return 0;
	}
	if (cmd == HEVC_UNLOCK) {
		/* only the holder gives the engine back: a stray unlock would
		 * let two encoders in */
		if (holder != file)
			return -EPERM;
		holder = NULL;
		up(&engine);
		return 0;
	}
	if (cmd == L->job_cmd)
		return hevc_job(uarg);
	if (cmd == L->wait_cmd)
		return hevc_wait(uarg);
	pr_err("hevc command error! cmd=%x\n", cmd);
	return -1;
}

static int hevc_open(struct inode *inode, struct file *file)
{
	return 0;
}

/* a holder that goes away gives the engine back */
static int hevc_release(struct inode *inode, struct file *file)
{
	if (holder && holder == file) {
		holder = NULL;
		up(&engine);
	}
	return 0;
}

static const struct file_operations hevc_fops = {
	.owner = THIS_MODULE,
	.open = hevc_open,
	.release = hevc_release,
	.unlocked_ioctl = hevc_ioctl,
};

static struct miscdevice hevc_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "h265",
	.fops = &hevc_fops,
};

static void hevc_unmap(void)
{
	if (line)
		iounmap(line);
	if (regs)
		iounmap(regs);
	if (reset_reg)
		iounmap(reset_reg);
	kfree(list);
}

static int __init xm_hevc_init(void)
{
	unsigned int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(layouts); i++)
		if (!strcmp(layout, layouts[i].name))
			L = &layouts[i];
	if (!L) {
		pr_err("hevc: layout is v3 or v1\n");
		return -EINVAL;
	}
	sema_init(&engine, 1);
	list = kmalloc(MAX_REGS * sizeof(*list), GFP_KERNEL);
	regs = ioremap(HEVC_BASE, 0x10000);
	reset_reg = ioremap(HEVC_RESET_REG, 4);
	if (L->start == 0)
		line = ioremap(HEVC_LINE_BASE, 0x100);
	if (!list || !regs || !reset_reg || (L->start == 0 && !line)) {
		hevc_unmap();
		return -ENOMEM;
	}
	hevc_reset();
	ret = request_irq(HEVC_IRQ, hevc_irq, 0, "hevc", NULL);
	if (ret) {
		pr_err("h265: request_irq failed!\n");
		hevc_unmap();
		return ret;
	}
	ret = misc_register(&hevc_misc);
	if (ret) {
		pr_err("h265 init error!\n");
		free_irq(HEVC_IRQ, NULL);
		hevc_unmap();
		return ret;
	}
	pr_info("hevc: openxiongmai xm_hevc init ok, engine %s\n", L->name);
	return 0;
}

static void __exit xm_hevc_exit(void)
{
	misc_deregister(&hevc_misc);
	free_irq(HEVC_IRQ, NULL);
	hevc_unmap();
}

module_init(xm_hevc_init);
module_exit(xm_hevc_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("V200-series H.265 engine, compatible with hevc.ko");
