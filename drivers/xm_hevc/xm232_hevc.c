// SPDX-License-Identifier: GPL-2.0
/*
 * xm232_hevc: the XM232's H.264/H.265/JPEG encoder engine, a drop-in for its
 * vendor hevc.ko (HEVC_LAYOUT=xm232; built as open_hevc.ko).
 *
 * As on the other V200 chips, user space (Sofia) works out every register
 * of a picture and this driver runs the jobs, through /dev/h265:
 *   0x4803 / 0x4804  take and give back the engine (a semaphore; a closing
 *                    holder gives it back too). Giving it back also sets the
 *                    line block's joint mode for joint_sensor_num sensors.
 *   0x40044806       reset, with the engine's key 0x5000508
 *   0x40784805       a job: 30 words, word 0 the channel (0), word 4 the
 *                    line count for line mode (0 for none), word 5 the work
 *                    type, word 29 the user address of a list of 402
 *                    {value, write} pairs, entry n for the register at 4n.
 *                    The entries with write set are kept, entries 2..401
 *                    written, and the engine started (1 to +0x000).
 *   0xc0744802       sleeps until the picture is done, and gives back the
 *                    job's first 29 words, word 18 the stream's bytes and
 *                    word 26 the JPEG snapshot's (+0x174, +0x614). After a
 *                    failed picture every wait fails until a reset.
 *
 * The engine's registers are at 0x21000000, its interrupt is SPI 24; the
 * line block at 0x21010000 paces the engine with the capture in line mode
 * and, with two sensors captured into one picture (joint_sensor_num), takes
 * the picture's lines a sensor at a time. Status: +0xA0 halted (aux), +0xA4
 * errors, +0xB0 a job begun, +0xB4 halted (an output buffer full), +0xB8
 * the end status (2 done, 1 an error), each acknowledged by writing it back
 * at +0x10, +0x14, +0x20, +0x24, +0x28. Work type 4 goes on into the next
 * output buffers when one fills (words 6, 7, 10, 15 and 22, 25), adding up
 * the lengths.
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
#define HEVC_LINE_BASE 0x21010000
#define HEVC_VRST_BASE 0x21f00000 /* the engine's own reset */
#define HEVC_CLK_RESET 0x20000120 /* the engine's reset, in the clock block */
#define HEVC_IRQ (32 + 24)

#define HEVC_LOCK 0x4803
#define HEVC_UNLOCK 0x4804
#define HEVC_RESET 0x40044806
#define HEVC_JOB 0x40784805
#define HEVC_WAIT 0xc0744802
#define HEVC_KEY 0x5000508

#define JOB_WORDS 30
#define OUT_WORDS 29
#define N_REGS 402
#define JOB_CHN 0
#define JOB_LINES 4
#define JOB_TYPE 5
#define JOB_LIST 29
#define OUT_LEN 18
#define OUT_JPEG_LEN 26
#define TYPE_CONTINUE 4

/* registers */
#define R_START 0x000
#define R_GO_ON 0x004
#define R_ACK_AHALT 0x010
#define R_ACK_ERR 0x014
#define R_ACK_BGN 0x020
#define R_ACK_HALT 0x024
#define R_ACK_END 0x028
#define R_BUSY 0x098
#define R_AHALT 0x0a0
#define R_ERR 0x0a4
#define R_BGN 0x0b0
#define R_HALT 0x0b4
#define R_END 0x0b8
#define R_LEN 0x174
#define R_HALT_REQ 0x548
#define R_JPEG_LEN 0x614
/* the line block */
#define L_LINES 0x00
#define L_ERR_EN 0xa4
#define L_ERR 0xa8
#define L_JOINT_EN 0x90
#define L_JOINT_FNUM 0x94
#define L_JOINT_VNUM 0x98

static int joint_sensor_num;
module_param(joint_sensor_num, int, 0444);
MODULE_PARM_DESC(joint_sensor_num, "sensors captured into one picture");

struct hevc_entry {
	u32 val, write;
};

static void __iomem *regs, *line, *vrst, *clk_reset;
static struct hevc_entry *list;
static u32 shadow[N_REGS], dirty[N_REGS];
static u32 now[OUT_WORDS]; /* the running job's record */
static bool done;
static u32 err; /* why the last picture failed; until a reset */
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

/* halt the engine, wait up to 2 ms for it to stop */
static void hevc_halt(void)
{
	int i;

	wr(R_START, 0);
	wr(R_ACK_HALT, 3);
	wr(0x44, 2);
	wr(0x68, 2);
	wr(R_HALT_REQ, 1);
	for (i = 0; i < 200; i++) {
		if ((rd(R_HALT) & 2) || !rd(R_BUSY))
			break;
		udelay(10);
	}
	if (i == 200)
		pr_warn("hevc: halt wait timeout\n");
	else if (i > 10)
		pr_warn("hevc: halt more than 100us real=%d us\n", i * 10);
}

static void hevc_forget(void)
{
	memset(shadow, 0, sizeof(shadow));
	memset(dirty, 0, sizeof(dirty));
	memset(now, 0, sizeof(now));
	done = false;
	err = 0;
}

static void hevc_reset(void)
{
	hevc_halt();
	writel(0, vrst + 0x00);
	writel(0, vrst + 0x20);
	udelay(100);
	writel(1, vrst + 0x00);
	writel(1, vrst + 0x20);
	hevc_forget();
}

/* the kept registers, entries 2..401, then start */
static void hevc_begin(void)
{
	unsigned int i;

	for (i = 2; i < N_REGS; i++)
		if (dirty[i]) {
			wr(i * 4, shadow[i]);
			dirty[i] = 0;
		}
	wr(R_START, 1);
}

/* after an error: the engine and the line block stopped and cleared */
static void hevc_recover(u32 flags)
{
	writel(0, line + L_ERR_EN);
	writel(0, line + 0xa0);
	wr(R_START, 0);
	wr(R_HALT_REQ, 1);
	wr(0x30, 0);
	wr(0x34, 0);
	wr(0x40, 0);
	wr(0x44, 0);
	wr(0x48, 0);
	wr(0x4c, 0);
	wr(0x5cc, 0);
	writel(0, line + L_ERR_EN);
	wr(R_ACK_AHALT, 0x3f);
	wr(R_ACK_ERR, 0xfff);
	wr(R_ACK_BGN, 3);
	wr(R_ACK_HALT, 3);
	wr(R_ACK_END, 3);
	wr(0x2c, 0xf);
	wr(0x5c4, 0xf);
	writel(0, line + 0xa0);
	err = flags;
	pr_err_ratelimited("hevc: err_flag=%x linenum %d busy %d\n", flags,
			   readl(line + 0xb0), rd(R_BUSY));
	wake_up(&waitq);
}

static irqreturn_t hevc_irq(int irq, void *dev)
{
	u32 ahalt = rd(R_AHALT), e = rd(R_ERR), bgn = rd(R_BGN);
	u32 halt = rd(R_HALT), end = rd(R_END);
	u32 lerr = readl(line + L_ERR), flags = 0;

	if (++irqs_unhandled > 10)
		pr_err_ratelimited("hevc: irq error %d\n", irqs_unhandled);
	if (end == 1) {
		wr(R_ACK_END, end);
		pr_warn_ratelimited("hevc: err: end_status %d\n", end);
	} else if (end & 2) {
		wr(R_ACK_END, end);
		if (now[JOB_TYPE] == TYPE_CONTINUE) {
			now[OUT_LEN] += rd(R_LEN);
			now[OUT_JPEG_LEN] += rd(R_JPEG_LEN);
		} else {
			now[OUT_LEN] = rd(R_LEN);
			now[OUT_JPEG_LEN] = rd(R_JPEG_LEN);
		}
		irqs_unhandled = 0;
		wr(R_START, 0);
		done = true;
		wake_up(&waitq);
	}
	if (bgn) {
		irqs_unhandled = 0;
		wr(R_ACK_BGN, bgn);
	}
	if (ahalt) {
		wr(R_ACK_AHALT, ahalt);
		pr_err_ratelimited("hevc: aux halt=%d busy=%d\n", ahalt,
				   rd(R_BUSY));
		flags |= 0x80;
	}
	if (halt) {
		wr(R_ACK_HALT, halt);
		if (now[JOB_TYPE] == TYPE_CONTINUE) {
			/* an output buffer is full: on into the next */
			now[OUT_LEN] = rd(R_LEN);
			now[OUT_JPEG_LEN] = rd(R_JPEG_LEN);
			wr(0x10c, now[6]);
			wr(0x110, now[7]);
			wr(0x13c, now[15] + now[10] + now[OUT_LEN]);
			wr(0x5fc, now[25] + now[22] + now[OUT_JPEG_LEN]);
			wr(R_GO_ON, 1);
		} else {
			pr_err_ratelimited("hevc: halt=%d busy=%d\n", halt,
					   rd(R_BUSY));
		}
	}
	if (e) {
		pr_err_ratelimited("hevc: err %x\n", e);
		flags |= e & 0xf;
		wr(R_ACK_ERR, e);
	}
	if (lerr) {
		pr_err_ratelimited("hevc: line mode err %x linenum %d\n", lerr,
				   readl(line + 0xb0));
		writel(0, line + L_ERR_EN);
		flags |= 0x40;
	}
	if (flags) {
		irqs_unhandled = 0;
		hevc_recover(flags);
	}
	return IRQ_HANDLED;
}

static long hevc_job(void __user *arg)
{
	u32 job[JOB_WORDS];
	unsigned int i;

	if (copy_from_user(job, arg, sizeof(job))) {
		pr_err("hevc: copy_from_user failed\n");
		return -EFAULT;
	}
	if (job[JOB_CHN]) {
		pr_err("h265: chn %d is invalid\n", job[JOB_CHN]);
		return -1;
	}
	if (copy_from_user(list, (void __user *)(uintptr_t)job[JOB_LIST],
			   N_REGS * sizeof(*list))) {
		pr_err("h265: copy_from_user failed1\n");
		return -EFAULT;
	}
	for (i = 0; i < N_REGS; i++)
		if (list[i].write) {
			shadow[i] = list[i].val;
			dirty[i] = 1;
		}
	memcpy(now, job, sizeof(now));
	now[OUT_LEN] = now[OUT_JPEG_LEN] = now[3] = 0;
	if (!job[JOB_LINES]) {
		writel(0, line + L_ERR_EN);
	} else {
		writel(job[JOB_LINES], line + L_LINES);
		writel(0, line + 0xa0);
		writel(0xf, line + L_ERR_EN);
		writel(0x3000000, line + 0x04);
		writel(2, line + 0x08);
		if (joint_sensor_num <= 0) {
			writel(0, line + L_JOINT_EN);
			writel(0, line + L_JOINT_FNUM);
			writel(0, line + L_JOINT_VNUM);
		} else {
			writel(1, line + L_JOINT_EN);
			if (joint_sensor_num > 1)
				writel(1, line + L_JOINT_FNUM);
			writel(job[JOB_LINES] / joint_sensor_num,
			       line + L_JOINT_VNUM);
		}
	}
	hevc_begin();
	return 0;
}

static long hevc_wait(void __user *arg)
{
	if (!done) {
		if (err)
			return -1;
		if (wait_event_interruptible(waitq, done || err))
			return -ERESTARTSYS;
	}
	if (err)
		return -1;
	/* the result stays pending until it reached the caller */
	if (arg && copy_to_user(arg, now, sizeof(now))) {
		pr_err("hevc: copy_to_user failed\n");
		return -EFAULT;
	}
	done = false;
	return 0;
}

static long hevc_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	void __user *uarg = (void __user *)arg;
	u32 key;

	switch (cmd) {
	case HEVC_RESET:
		if (copy_from_user(&key, uarg, 4))
			return -EFAULT;
		if (key != HEVC_KEY) {
			pr_err("hevc: Version mismatch\n");
			return -1;
		}
		hevc_reset();
		return 0;
	case HEVC_LOCK:
		if (down_interruptible(&engine))
			return -ERESTARTSYS;
		holder = file;
		return 0;
	case HEVC_UNLOCK:
		/* only the holder gives the engine back: a stray unlock would
		 * let two encoders in */
		if (holder != file)
			return -EPERM;
		holder = NULL;
		up(&engine);
		writel(1, line + L_JOINT_EN);
		writel(joint_sensor_num - 1, line + L_JOINT_FNUM);
		return 0;
	case HEVC_JOB:
		return hevc_job(uarg);
	case HEVC_WAIT:
		return hevc_wait(uarg);
	}
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
	if (clk_reset)
		iounmap(clk_reset);
	if (vrst)
		iounmap(vrst);
	if (line)
		iounmap(line);
	if (regs)
		iounmap(regs);
	kfree(list);
}

static int __init xm232_hevc_init(void)
{
	int ret;

	sema_init(&engine, 1);
	list = kmalloc(N_REGS * sizeof(*list), GFP_KERNEL);
	regs = ioremap(HEVC_BASE, 0x10000);
	line = ioremap(HEVC_LINE_BASE, 0x100);
	vrst = ioremap(HEVC_VRST_BASE, 0x100);
	clk_reset = ioremap(HEVC_CLK_RESET, 4);
	if (!list || !regs || !line || !vrst || !clk_reset) {
		hevc_unmap();
		return -ENOMEM;
	}
	/* at load the engine is reset through the clock block */
	hevc_halt();
	writel(0, clk_reset);
	udelay(100);
	writel(7, clk_reset);
	hevc_forget();
	/* named as XM232's hevc.ko names it */
	ret = request_irq(HEVC_IRQ, hevc_irq, 0, "h265", NULL);
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
	pr_info("hevc: openxiongmai xm232_hevc init ok, joint_sensor_num %d\n",
		joint_sensor_num);
	return 0;
}

static void __exit xm232_hevc_exit(void)
{
	misc_deregister(&hevc_misc);
	free_irq(HEVC_IRQ, NULL);
	hevc_unmap();
}

module_init(xm232_hevc_init);
module_exit(xm232_hevc_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("XM232 H.264/H.265 engine, compatible with its hevc.ko");
