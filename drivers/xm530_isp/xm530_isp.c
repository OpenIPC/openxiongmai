// SPDX-License-Identifier: GPL-2.0
/*
 * xm530_isp: the XM530/XM550 ISP device, a drop-in replacement for the
 * vendor's xm530_isp.ko with the same /dev/isp ABI.
 *
 * The ISP's work is done in user space, by the vendor ISP library linked into
 * the streamer. The kernel gives it:
 *  - register access: ioctl 1 writes, 2 and 3 read one word ({addr, val}) in
 *    the ISP block (0x30000000, 2 MiB) or the GPIO block (0x10020000, 1 MiB);
 *  - waits for the ISP's frame interrupts (ioctl 4: 0 Bayer statistics,
 *    1 luma statistics) and the audio one (ioctl 5);
 *  - a 16 KiB block shared by whoever maps /dev/isp;
 *  - AWB zone statistics: write() sets {base, count, div}, read() reads
 *    count zones from the block at base into three u16 arrays;
 *  - the MIPI lane-delay sweep (ioctl 6 starts it, 7 runs it), with the
 *    Bayer-H interrupt watching the MIPI status for changes, which only a
 *    diagnostic mode of the ISP application uses.
 * /proc/umap/isp has the interrupt rates and counts.
 *
 * What differs from the vendor module:
 *  - ioctl 4's event number is checked (the vendor module uses it as an index
 *    into its own data, and writes there);
 *  - a wait that times out fails with -ETIMEDOUT, one interrupted by a
 *    signal the usual way (the vendor module returns -512 to user space for
 *    both);
 *  - a read outside both blocks returns 0, not whatever was on the stack;
 *  - ioctl 6 stops a sweep still running before it starts another (the
 *    vendor module starts a thread every time and loses the earlier ones),
 *    and the thread stays until then, so ioctl 7 can run the sweep again;
 *  - the sweep sets each delay before it counts that delay's frames (the
 *    vendor module judges each delay by the frames of the one before), and
 *    takes the presets from each ioctl 6;
 *  - a mapping is 16 KiB at most (the vendor module maps any length, and
 *    with it the memory after the block);
 *  - the AWB statistics' base, count and divisor are checked (the vendor
 *    module writes to an unchecked address, overruns its arrays with a large
 *    count and divides by a zero divisor), and zones past the count read
 *    as 0;
 *  - the rates come from the monotonic clock;
 *  - the block is freed at the order it was allocated (the vendor module
 *    frees one page of the four).
 */

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/gfp.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kthread.h>
#include <linux/ktime.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/proc_fs.h>
#include <linux/ratelimit.h>
#include <linux/sched.h>
#include <linux/seq_file.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>
#include <linux/wait.h>

#define ISP_REG_BASE 0x30000000
#define ISP_REG_SIZE 0x200000
#define GPIO_REG_BASE 0x10020000
#define GPIO_REG_SIZE 0x100000

#define IRQ_BAYER 57
#define IRQ_LUM 58
#define IRQ_BAYER_H 59
#define IRQ_AUDIO 62

/* register offsets from ISP_REG_BASE */
#define REG_IRQ_EN_A 0xf0040
#define REG_BAYER_STATUS 0xf004c
#define REG_BAYER_H_EN 0xf0050
#define REG_MIPI_DELAY 0xf0058
#define REG_BAYER_H_STATUS 0xf005c
#define REG_IRQ_EN_B 0xf0060
#define REG_LUM_STATUS 0xf006c
#define REG_AUDIO_STATUS 0x890c8
#define REG_MIPI_LANE_DELAY 0xc0010
#define REG_MIPI_RESET 0xc002c
#define REG_MIPI_STAT0 0xc0040
#define REG_MIPI_STAT1 0xc0044

#define ISP_IOC_WRITE _IOW('I', 1, struct isp_reg)
#define ISP_IOC_READ _IOW('I', 2, struct isp_reg) /* read, written back */
#define ISP_IOC_READ2 _IOW('I', 3, struct isp_reg)
#define ISP_IOC_WAIT _IOW('I', 4, struct isp_reg)
#define ISP_IOC_WAIT_AUDIO _IO('I', 5)
#define ISP_IOC_MIPI_INIT _IOW('I', 6, struct isp_mipi)
#define ISP_IOC_MIPI_RUN _IO('I', 7)

#define ISP_EVENTS 2 /* ioctl 4: 0 Bayer, 1 luma */
#define ISP_WAIT_JIFFIES 200
#define SHMEM_ORDER 2 /* 16 KiB */
#define AWB_ZONES 512
#define MIPI_CAP 500
#define MIPI_STEP_MS 200

struct isp_reg {
	u32 addr;
	u32 val;
	u32 arg; /* ioctl 4: the event */
};

struct isp_mipi {
	u32 delay, frames; /* capped at MIPI_CAP */
	u32 preset0, preset1; /* MIPI status to compare with, both or none */
	u32 go;
};

struct isp_event {
	u32 way; /* as /proc shows it: 0 until the first interrupt, as in the
		  * vendor module */
	u32 way_id;
	bool flag; /* happened since the last wait */
	ktime_t last;
	u32 rate; /* audio: the last interval in us */
	u32 cnt;
	wait_queue_head_t wq;
};

static void __iomem *isp_regs;
static void __iomem *gpio_regs;
static DEFINE_SPINLOCK(isp_lock); /* events and the MIPI state */
static struct isp_event events[ISP_EVENTS], audio_event;

/* the shared block */
static DEFINE_MUTEX(shmem_lock);
static struct page *shmem;

/* AWB statistics: write() sets these, read() reads into the arrays */
static DEFINE_MUTEX(awb_lock);
static struct {
	u32 base, count, div;
} awb;
static u16 awb_stat[3][AWB_ZONES];

static struct proc_dir_entry *isp_proc;

/* The MIPI sweep, as the vendor module keeps it */
static DEFINE_MUTEX(mipi_lock); /* the thread's start and stop */
static struct task_struct *mipi_thread;
static struct {
	bool preset;
	u32 preset0, preset1;
	u32 prev0, prev1; /* the MIPI status at the previous frame, ~0 unset */
	u32 cur0, cur1;
	u32 cnt; /* frames since the delay was set */
	u32 frames; /* to wait before judging a delay */
	bool err; /* the status changed */
	bool restart, done;
	u8 delay, end, start;
	u16 mid;
	u16 rst_cnt, settle;
} mipi = {
	.prev0 = ~0u,
	.prev1 = ~0u,
};

static inline void isp_write(u32 off, u32 val)
{
	writel(val, isp_regs + off);
}

static inline u32 isp_read(u32 off)
{
	return readl(isp_regs + off);
}

/* The register an address names, NULL if in neither block or not a word */
static void __iomem *isp_reg(u32 addr)
{
	if (addr & 3)
		return NULL;
	if (addr - ISP_REG_BASE < ISP_REG_SIZE)
		return isp_regs + (addr - ISP_REG_BASE);
	if (addr - GPIO_REG_BASE < GPIO_REG_SIZE)
		return gpio_regs + (addr - GPIO_REG_BASE);
	return NULL;
}

/* An interrupt's rate and count; the lock held */
static void event_stamp(struct isp_event *e, bool raw_interval)
{
	ktime_t now = ktime_get();
	s64 us = ktime_us_delta(now, e->last); /* the first: since boot */

	if (raw_interval)
		e->rate = us <= 0 ? 1 : min_t(s64, us, 0x0fffffff);
	else if (us <= 0)
		e->rate = 1000000;
	else if (us > 10000000)
		e->rate = 0; /* under 0.1 per second rounds to 0 */
	else
		e->rate = (10000000 / (u32)us + 5) / 10;
	e->cnt = e->cnt < 0xffff ? e->cnt + 1 : 1;
	e->last = now;
	e->way = e->way_id;
}

static irqreturn_t isp_irq_stat(int irq, void *dev_id)
{
	struct isp_event *e = dev_id;
	u32 status = e == &events[0] ? REG_BAYER_STATUS : REG_LUM_STATUS;
	unsigned long flags;

	spin_lock_irqsave(&isp_lock, flags);
	if (isp_read(status)) {
		isp_write(status, 0);
		event_stamp(e, false);
		e->flag = true;
	}
	spin_unlock_irqrestore(&isp_lock, flags);
	wake_up_interruptible(&e->wq);
	return IRQ_HANDLED;
}

static irqreturn_t isp_irq_audio(int irq, void *dev_id)
{
	unsigned long flags;

	spin_lock_irqsave(&isp_lock, flags);
	if (isp_read(REG_AUDIO_STATUS)) {
		audio_event.flag = true;
		isp_write(REG_AUDIO_STATUS, 0);
		event_stamp(&audio_event, true);
	}
	spin_unlock_irqrestore(&isp_lock, flags);
	wake_up_interruptible(&audio_event.wq);
	return IRQ_HANDLED;
}

/* Bayer-H: whether the MIPI status stays put from frame to frame, for the
 * sweep */
static irqreturn_t isp_irq_bayer_h(int irq, void *dev_id)
{
	unsigned long flags;

	spin_lock_irqsave(&isp_lock, flags);
	if (!isp_read(REG_BAYER_H_STATUS))
		goto out;
	isp_write(REG_BAYER_H_STATUS, 0);
	if (mipi.preset) {
		mipi.prev0 = mipi.preset1;
		mipi.prev1 = mipi.preset0;
	} else {
		if (mipi.prev0 == ~0u || !mipi.cnt)
			mipi.prev0 = isp_read(REG_MIPI_STAT0);
		if (mipi.prev1 == ~0u || !mipi.cnt)
			mipi.prev1 = isp_read(REG_MIPI_STAT1);
	}
	mipi.cur0 = isp_read(REG_MIPI_STAT0);
	mipi.cur1 = isp_read(REG_MIPI_STAT1);
	mipi.cnt = mipi.cnt + 1 > 0xfffffff0 ? 0xfffffff0 : mipi.cnt + 1;
	if (mipi.cur0 != mipi.prev0 || mipi.cur1 != mipi.prev1) {
		mipi.prev0 = mipi.prev1 = ~0u;
		mipi.cnt = 0xfffffff0;
		mipi.err = true;
	}
out:
	spin_unlock_irqrestore(&isp_lock, flags);
	return IRQ_HANDLED;
}

/* The good window found, its middle printed as the vendor module does */
static void mipi_report(u8 start, u8 end)
{
	u8 delt = end - start;

	mipi.mid = start + delt / 2;
	pr_info("start: %2x  end: %2x  delt: %2x  r_md0: %2x  r_md1: %2x \n",
		start, end, delt, mipi.mid, mipi.mid);
}

/* One delay, for all four lanes; Bayer-H is left off */
static void mipi_set_delay(u8 d)
{
	u32 v = d | d << 8 | d << 16 | d << 24;

	isp_write(REG_BAYER_H_EN, 0);
	mdelay(10);
	isp_write(REG_MIPI_LANE_DELAY, v);
	isp_write(REG_MIPI_RESET, 0);
	mdelay(1);
	isp_write(REG_MIPI_RESET, 4);
	mdelay(100);
}

/* Counts the frames of the delay just set from 0, then turns Bayer-H on */
static void mipi_arm(void)
{
	unsigned long flags;

	spin_lock_irqsave(&isp_lock, flags);
	mipi.prev0 = mipi.prev1 = ~0u;
	mipi.err = false;
	mipi.cnt = 0;
	isp_write(REG_BAYER_H_EN, 1);
	spin_unlock_irqrestore(&isp_lock, flags);
}

/*
 * The sweep, as the vendor module's: each delay is set, given mipi.frames
 * frames, and judged by whether the Bayer-H interrupt saw the MIPI status
 * change. A run of good delays is reported with its middle. Unlike the
 * vendor's, a delay is set before its frames are counted, and the thread
 * stays, idle, until it is stopped, so ioctl 7 can run the sweep again.
 */
static int mipi_check_th(void *unused)
{
	unsigned long flags;
	bool start, apply, err = false;
	u8 d = 0;

	while (!kthread_should_stop()) {
		start = apply = false;
		spin_lock_irqsave(&isp_lock, flags);
		if (mipi.restart) {
			if (++mipi.rst_cnt > 4) {
				mipi.rst_cnt = 0;
				mipi.restart = false;
				mipi.settle = 1;
				start = true;
			}
		} else if (mipi.settle <= 1) {
			/* the first frames after the start are not counted */
			mipi.settle++;
			mipi.err = false;
			mipi.cnt = 0;
		} else if (!mipi.done && mipi.cnt >= mipi.frames) {
			apply = true;
			d = mipi.delay;
			err = mipi.err;
		}
		spin_unlock_irqrestore(&isp_lock, flags);

		if (start) {
			mipi_set_delay(0);
			mipi_arm();
		} else if (apply) {
			pr_info("add:%2x\t%2x\n", d, d);
			if (err) {
				if (mipi.end > mipi.start)
					mipi_report(mipi.start, mipi.end);
				mipi.start = d == 0xff ? 0xff : d + 1;
			} else {
				mipi.end = d;
				if (d == 0xff && d > mipi.start)
					mipi_report(mipi.start, d);
			}
			if (d == 0xff) {
				spin_lock_irqsave(&isp_lock, flags);
				mipi.done = true;
				isp_write(REG_BAYER_H_EN, 0);
				spin_unlock_irqrestore(&isp_lock, flags);
				pr_info("finish\n");
			} else {
				mipi_set_delay(d + 1);
				spin_lock_irqsave(&isp_lock, flags);
				mipi.delay = d + 1;
				spin_unlock_irqrestore(&isp_lock, flags);
				mipi_arm();
			}
		}
		schedule_timeout_interruptible(msecs_to_jiffies(MIPI_STEP_MS));
	}
	return 0;
}

static void mipi_stop(void)
{
	if (mipi_thread) {
		kthread_stop(mipi_thread);
		mipi_thread = NULL;
		isp_write(REG_BAYER_H_EN, 0);
	}
}

/* A sweep from delay 0, at the thread's next step; the lock held */
static void mipi_restart(void)
{
	mipi.rst_cnt = 4;
	mipi.delay = mipi.end = mipi.start = 0;
	mipi.done = false;
	mipi.restart = true;
}

static int mipi_init(const struct isp_mipi *m)
{
	struct task_struct *t;
	unsigned long flags;
	int ret = 0;

	if (m->go != 1)
		return 0;
	mutex_lock(&mipi_lock);
	mipi_stop();
	spin_lock_irqsave(&isp_lock, flags);
	mipi.frames = min_t(u32, m->frames, MIPI_CAP);
	mipi.preset = m->preset0 && m->preset1;
	mipi.preset0 = m->preset0;
	mipi.preset1 = m->preset1;
	mipi_restart();
	spin_unlock_irqrestore(&isp_lock, flags);
	isp_write(REG_MIPI_DELAY, min_t(u32, m->delay, MIPI_CAP));
	t = kthread_run(mipi_check_th, NULL, "xm530MipiDelay");
	if (IS_ERR(t)) {
		pr_err("create mipi check thread failed!!\n");
		ret = PTR_ERR(t);
	} else {
		mipi_thread = t;
	}
	mutex_unlock(&mipi_lock);
	return ret;
}

/* Runs the sweep again from delay 0, after a 4-step pause as the vendor's */
static void mipi_run(void)
{
	unsigned long flags;

	spin_lock_irqsave(&isp_lock, flags);
	mipi_restart();
	mipi.rst_cnt = 0;
	spin_unlock_irqrestore(&isp_lock, flags);
	pr_info("Go!\n");
}

/* Until the event has happened since the last wait */
static long isp_wait(struct isp_event *e)
{
	unsigned long flags;
	long left;

	left = wait_event_interruptible_timeout(e->wq, e->flag,
						ISP_WAIT_JIFFIES);
	if (left < 0)
		return left;
	spin_lock_irqsave(&isp_lock, flags);
	if (!e->flag) {
		spin_unlock_irqrestore(&isp_lock, flags);
		return -ETIMEDOUT;
	}
	e->flag = false;
	spin_unlock_irqrestore(&isp_lock, flags);
	return 0;
}

static long isp_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	void __user *uarg = (void __user *)arg;
	struct isp_mipi m;
	struct isp_reg r;
	void __iomem *reg;

	switch (cmd) {
	case ISP_IOC_WRITE:
		if (copy_from_user(&r, uarg, sizeof(r)))
			goto fault;
		reg = isp_reg(r.addr);
		if (!reg) {
			pr_err_ratelimited(
				"The WriteReg Addr is outside, cmd %d error!\n",
				_IOC_NR(cmd));
			return 0;
		}
		writel(r.val, reg);
		return 0;
	case ISP_IOC_READ:
	case ISP_IOC_READ2:
		if (copy_from_user(&r, uarg, sizeof(r)))
			goto fault;
		reg = isp_reg(r.addr);
		if (reg) {
			r.val = readl(reg);
		} else {
			pr_err_ratelimited(
				"The ReadReg Addr is outside, cmd %d error!\n",
				_IOC_NR(cmd));
			r.val = 0;
		}
		if (copy_to_user(uarg, &r, sizeof(r))) {
			pr_err("IspRegRead : copy_to_user error.\n");
			return -EFAULT;
		}
		return 0;
	case ISP_IOC_WAIT:
		if (copy_from_user(&r, uarg, sizeof(r)))
			goto fault;
		if (r.arg >= ISP_EVENTS)
			return -EINVAL;
		return isp_wait(&events[r.arg]);
	case ISP_IOC_WAIT_AUDIO:
		return isp_wait(&audio_event);
	case ISP_IOC_MIPI_INIT:
		if (copy_from_user(&m, uarg, sizeof(m)))
			goto fault;
		return mipi_init(&m);
	case ISP_IOC_MIPI_RUN:
		mipi_run();
		return 0;
	default:
		pr_err_ratelimited("IspKo:Err: command %d error!\n",
				   _IOC_NR(cmd));
		return 0;
	}
fault:
	pr_err("Isp %d: copy_from_user error.\n", _IOC_NR(cmd));
	return -EFAULT;
}

/* write(): {base, count, div} of the AWB statistics, 12 bytes at most */
static ssize_t isp_awb_write(struct file *file, const char __user *buf,
			     size_t len, loff_t *pos)
{
	size_t n = min_t(size_t, len, sizeof(awb));
	ssize_t ret = n;

	mutex_lock(&awb_lock);
	if (copy_from_user(&awb, buf, n))
		ret = -EFAULT;
	mutex_unlock(&awb_lock);
	return ret;
}

/* read(): count zones from the statistics block at base, as three u16
 * arrays of AWB_ZONES; at most their size */
static ssize_t isp_awb_read(struct file *file, char __user *buf, size_t len,
			    loff_t *pos)
{
	size_t n = min_t(size_t, len, sizeof(awb_stat));
	u32 off, i, r0, r1, r2;
	ssize_t ret = n;

	mutex_lock(&awb_lock);
	off = awb.base - ISP_REG_BASE;
	if ((awb.base & 3) || off >= ISP_REG_SIZE - 0x10 ||
	    awb.count > AWB_ZONES || !awb.div) {
		mutex_unlock(&awb_lock);
		return -EINVAL;
	}
	memset(awb_stat, 0, sizeof(awb_stat));
	isp_write(off, 1);
	for (i = 0; i < awb.count; i++) {
		r0 = isp_read(off + 8);
		r1 = isp_read(off + 0xc);
		r2 = isp_read(off + 0x10);
		isp_write(off + 4, 1);
		awb_stat[0][i] = r0 / awb.div;
		awb_stat[1][i] = (r1 / awb.div) >> 1;
		awb_stat[2][i] = r2 / awb.div;
	}
	isp_write(off, 1);
	if (copy_to_user(buf, awb_stat, n))
		ret = -EFAULT;
	mutex_unlock(&awb_lock);
	return ret;
}

static void isp_vma_open(struct vm_area_struct *vma)
{
	pr_debug("%s()is called.\n", __func__);
}

static void isp_vma_close(struct vm_area_struct *vma)
{
	pr_debug("%s()is called.\n", __func__);
}

static const struct vm_operations_struct isp_vm_ops = {
	.open = isp_vma_open,
	.close = isp_vma_close,
};

/* The shared block, made on the first mapping */
static int isp_mmap(struct file *file, struct vm_area_struct *vma)
{
	unsigned long len = vma->vm_end - vma->vm_start;
	int ret;

	if (vma->vm_pgoff || len > PAGE_SIZE << SHMEM_ORDER)
		return -EINVAL;
	mutex_lock(&shmem_lock);
	if (!shmem) {
		shmem = alloc_pages(GFP_KERNEL | __GFP_ZERO, SHMEM_ORDER);
		if (!shmem) {
			mutex_unlock(&shmem_lock);
			return -ENOMEM;
		}
		pr_info("IspKo: [IspDrv]: Memery Init Buffer!Num[%d]\n",
			SHMEM_ORDER);
	}
	ret = remap_pfn_range(vma, vma->vm_start, page_to_pfn(shmem), len,
			      vma->vm_page_prot);
	mutex_unlock(&shmem_lock);
	if (ret) {
		pr_err("IspKo: [IspDrv] err: remap_pfn_range failed!\n");
		return -EAGAIN;
	}
	vma->vm_ops = &isp_vm_ops;
	return 0;
}

static int isp_seq_show(struct seq_file *s, void *unused)
{
	struct isp_event e[ISP_EVENTS + 1];
	unsigned long flags;
	u32 audio_rate;
	int i;

	spin_lock_irqsave(&isp_lock, flags);
	for (i = 0; i < ISP_EVENTS; i++)
		e[i] = events[i];
	e[ISP_EVENTS] = audio_event;
	spin_unlock_irqrestore(&isp_lock, flags);

	seq_puts(s, "[ISP] openxiongmai xm530_isp\n\n");
	seq_puts(s, "IspinfoWay\tIspinfoRate\tIspinfoCnt \n");
	for (i = 0; i < ISP_EVENTS; i++)
		seq_printf(s, "%d\t\t%d\t\t%d \n", e[i].way, e[i].rate,
			   e[i].cnt);
	seq_puts(s, "\nAudioWay  \tAudioRate  \tAudioCnt \n");
	audio_rate = e[ISP_EVENTS].rate ?
			     (1000000 + e[ISP_EVENTS].rate / 2) /
				     e[ISP_EVENTS].rate :
			     0;
	seq_printf(s, "%d\t\t%d\t\t%d\n", e[ISP_EVENTS].way, audio_rate,
		   e[ISP_EVENTS].cnt);
	return 0;
}

static int isp_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, isp_seq_show, NULL);
}

static const struct file_operations isp_proc_fops = {
	.owner = THIS_MODULE,
	.open = isp_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int isp_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int isp_release(struct inode *inode, struct file *file)
{
	return 0;
}

static const struct file_operations isp_fops = {
	.owner = THIS_MODULE,
	.read = isp_awb_read,
	.write = isp_awb_write,
	.unlocked_ioctl = isp_ioctl,
	.mmap = isp_mmap,
	.open = isp_open,
	.release = isp_release,
};

static struct miscdevice isp_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "isp",
	.fops = &isp_fops,
};

static const struct {
	int irq;
	irq_handler_t fn;
	const char *name;
	void *dev;
} isp_irqs[] = {
	{ IRQ_BAYER, isp_irq_stat, "isp_irq_bayer", &events[0] },
	{ IRQ_BAYER_H, isp_irq_bayer_h, "isp_irq_bayer_h", NULL },
	{ IRQ_LUM, isp_irq_stat, "isp_irq_lum", &events[1] },
	{ IRQ_AUDIO, isp_irq_audio, "audio_irq_fun", NULL },
};

static int __init xm530_isp_init(void)
{
	int i, ret;

	isp_regs = ioremap(ISP_REG_BASE, ISP_REG_SIZE);
	if (!isp_regs) {
		pr_err("IspKo:Err: ioremap gpIspRegAddr failed!\n");
		return -ENOMEM;
	}
	gpio_regs = ioremap(GPIO_REG_BASE, GPIO_REG_SIZE);
	if (!gpio_regs) {
		pr_err("IspKo:Err: ioremap gpGpioRegAddr failed!\n");
		ret = -ENOMEM;
		goto unmap_isp;
	}
	for (i = 0; i < ISP_EVENTS; i++) {
		events[i].way_id = i;
		init_waitqueue_head(&events[i].wq);
	}
	audio_event.way_id = 3;
	init_waitqueue_head(&audio_event.wq);
	isp_write(REG_IRQ_EN_A, 1);
	isp_write(REG_IRQ_EN_B, 1);

	for (i = 0; i < ARRAY_SIZE(isp_irqs); i++) {
		ret = request_irq(isp_irqs[i].irq, isp_irqs[i].fn, 0,
				  isp_irqs[i].name, isp_irqs[i].dev);
		if (ret) {
			pr_err("IspKo:Err: request_irq %s failed!\n",
			       isp_irqs[i].name);
			goto free_irqs;
		}
	}
	ret = misc_register(&isp_misc);
	if (ret) {
		pr_err("IspKo:Err: misc_register failed!\n");
		goto free_irqs;
	}
	/* /proc/umap is the MMZ module's */
	isp_proc = proc_create("umap/isp", 0444, NULL, &isp_proc_fops);
	if (!isp_proc)
		pr_warn("IspKo: no /proc/umap/isp, MMZ not loaded?\n");
	pr_info("IspKo: openxiongmai xm530_isp init ok\n");
	return 0;

free_irqs:
	while (--i >= 0)
		free_irq(isp_irqs[i].irq, isp_irqs[i].dev);
	isp_write(REG_IRQ_EN_A, 0);
	isp_write(REG_IRQ_EN_B, 0);
	iounmap(gpio_regs);
unmap_isp:
	iounmap(isp_regs);
	return ret;
}

static void __exit xm530_isp_exit(void)
{
	int i;

	mutex_lock(&mipi_lock);
	mipi_stop();
	mutex_unlock(&mipi_lock);
	if (isp_proc)
		remove_proc_entry("umap/isp", NULL);
	misc_deregister(&isp_misc);
	for (i = 0; i < ARRAY_SIZE(isp_irqs); i++)
		free_irq(isp_irqs[i].irq, isp_irqs[i].dev);
	isp_write(REG_IRQ_EN_A, 0);
	isp_write(REG_IRQ_EN_B, 0);
	iounmap(gpio_regs);
	iounmap(isp_regs);
	/* nothing maps it now: a mapping holds /dev/isp, and the module */
	if (shmem)
		__free_pages(shmem, SHMEM_ORDER);
	pr_info("IspKo: ispDev_exit OK!\n");
}

module_init(xm530_isp_init);
module_exit(xm530_isp_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("XM530/XM550 ISP device, compatible with xm530_isp.ko");
