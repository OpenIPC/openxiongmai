// SPDX-License-Identifier: GPL-2.0
/*
 * xm510_isp: the XM510 ISP device, a drop-in replacement for the vendor's
 * xm510_isp.ko with the same /dev/isp ABI.
 *
 * The ISP's work (exposure, white balance and the rest) is done in user space
 * by the vendor isp daemon. The kernel gives it three things:
 *  - register access: ioctl 0x101 writes and 0x103 reads one word, on a
 *    record { u32 addr; u32 val; }, in the ISP/VI block (0x40000000, 1 MiB)
 *    or the GPIO block (0x10020000, 1 MiB);
 *  - a page of memory shared by whoever maps /dev/isp, zeroed when first
 *    mapped and kept until the module goes;
 *  - SIGIO, to descriptors that ask for it (fasync), at the start of every
 *    frame: the VI frame-start interrupt (IRQ 16), shared with the VI driver,
 *    which clears it.
 * /proc/isp has the frame rate and a frame count.
 *
 * What differs from the vendor module:
 *  - the record is copied from and to user space;
 *  - an address outside both blocks, or not word-aligned, is refused and
 *    releases the lock (the vendor module leaves it held, and every later
 *    ioctl then waits forever);
 *  - a mapping is one page at most (the vendor module maps whatever length
 *    it is asked for, and with it the memory after the page);
 *  - the page is freed as a page on unload (the vendor module passes it to
 *    free_pages() as an address, frees part of mem_map instead and logs
 *    "Bad page state").
 */

#include <linux/fs.h>
#include <linux/gfp.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/proc_fs.h>
#include <linux/ratelimit.h>
#include <linux/sched.h>
#include <linux/seq_file.h>
#include <linux/spinlock.h>
#include <linux/time.h>
#include <linux/uaccess.h>

#define ISP_REG_BASE 0x40000000
#define GPIO_REG_BASE 0x10020000
#define ISP_WIN_SIZE 0x100000
#define ISP_IRQ 16 /* VI frame start, shared */

/* ioctl numbers as the vendor daemon issues them: not _IO()-encoded */
#define ISP_IOC_WRITE 0x101
#define ISP_IOC_READ 0x103

struct isp_reg {
	u32 addr;
	u32 val;
};

static void __iomem *isp_regs;
static void __iomem *gpio_regs;
static DEFINE_MUTEX(isp_lock);
static struct page *shared_page;
static struct fasync_struct *isp_async;

static DEFINE_SPINLOCK(stat_lock);
static struct timeval last_frame;
static int frame_rate;
static u32 frame_cnt;

/* the shared interrupt needs a dev_id of its own */
static int isp_irq_dev;

/* The register an address names, NULL if it is in neither block or not a
 * whole word */
static void __iomem *isp_reg(u32 addr)
{
	if (addr & 3)
		return NULL;
	if (addr - ISP_REG_BASE < ISP_WIN_SIZE)
		return isp_regs + (addr - ISP_REG_BASE);
	if (addr - GPIO_REG_BASE < ISP_WIN_SIZE)
		return gpio_regs + (addr - GPIO_REG_BASE);
	return NULL;
}

static long isp_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct isp_reg __user *uarg = (struct isp_reg __user *)arg;
	struct isp_reg r;
	void __iomem *reg;
	long ret = 0;

	if (cmd != ISP_IOC_WRITE && cmd != ISP_IOC_READ) {
		pr_err_ratelimited("IspKo:Err: command error!\n");
		return 0;
	}
	if (copy_from_user(&r, uarg, sizeof(r)))
		return -EFAULT;
	reg = isp_reg(r.addr);
	if (!reg) {
		/* the vendor module returns 0 here too */
		pr_err_ratelimited("IspKo:Err: u32RegAddr :%d Err!\n", r.addr);
		return 0;
	}

	mutex_lock(&isp_lock);
	if (cmd == ISP_IOC_WRITE) {
		writel(r.val, reg);
	} else {
		r.val = readl(reg);
		if (put_user(r.val, &uarg->val))
			ret = -EFAULT;
	}
	mutex_unlock(&isp_lock);
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

/* The shared page, made on the first mapping */
static int isp_mmap(struct file *file, struct vm_area_struct *vma)
{
	unsigned long len = vma->vm_end - vma->vm_start;
	int ret;

	if (vma->vm_pgoff || len > PAGE_SIZE)
		return -EINVAL;
	mutex_lock(&isp_lock);
	if (!shared_page) {
		shared_page = alloc_page(GFP_KERNEL | __GFP_ZERO);
		if (!shared_page) {
			mutex_unlock(&isp_lock);
			return -ENOMEM;
		}
		pr_info("IspKo: [IspDrv]: Share Memery Init Buffer!\n");
	}
	ret = remap_pfn_range(vma, vma->vm_start, page_to_pfn(shared_page),
			      len, vma->vm_page_prot);
	mutex_unlock(&isp_lock);
	if (ret) {
		pr_err("IspKo: [IspDrv] err: remap_pfn_range failed!\n");
		return -EAGAIN;
	}
	vma->vm_ops = &isp_vm_ops;
	return 0;
}

static int isp_fasync(int fd, struct file *file, int on)
{
	return fasync_helper(fd, file, on, &isp_async);
}

static int isp_open(struct inode *inode, struct file *file)
{
	return 0;
}

static int isp_release(struct inode *inode, struct file *file)
{
	fasync_helper(-1, file, 0, &isp_async);
	return 0;
}

/*
 * Frame start: SIGIO to the descriptors that asked, and the frame rate. The
 * VI driver owns the interrupt's status and clears it; this one only listens,
 * and says IRQ_NONE, as the vendor module does.
 */
static irqreturn_t isp_irq(int irq, void *dev_id)
{
	struct timeval now;
	long us;

	kill_fasync(&isp_async, SIGIO, POLL_IN);
	do_gettimeofday(&now);
	spin_lock(&stat_lock);
	us = (now.tv_sec - last_frame.tv_sec) * 1000000 + now.tv_usec -
	     last_frame.tv_usec;
	if (us > 0)
		frame_rate = (10000000 / us + 5) / 10;
	frame_cnt = frame_cnt < 0xffff ? frame_cnt + 1 : 1;
	last_frame = now;
	spin_unlock(&stat_lock);
	return IRQ_NONE;
}

static int isp_seq_show(struct seq_file *s, void *unused)
{
	unsigned long flags;
	int rate;
	u32 cnt;

	spin_lock_irqsave(&stat_lock, flags);
	rate = frame_rate;
	cnt = frame_cnt;
	spin_unlock_irqrestore(&stat_lock, flags);
	seq_puts(s, "FrameRate\tFrameCnt \n");
	seq_printf(s, "%d\t\t%d \n", rate, cnt);
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

static const struct file_operations isp_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = isp_ioctl,
	.mmap = isp_mmap,
	.open = isp_open,
	.release = isp_release,
	.fasync = isp_fasync,
};

static struct miscdevice isp_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "isp",
	.fops = &isp_fops,
};

static int __init xm510_isp_init(void)
{
	int ret;

	isp_regs = ioremap(ISP_REG_BASE, ISP_WIN_SIZE);
	if (!isp_regs) {
		pr_err("IspKo:Err: ioremap gpIspRegAddr failed!\n");
		return -ENOMEM;
	}
	gpio_regs = ioremap(GPIO_REG_BASE, ISP_WIN_SIZE);
	if (!gpio_regs) {
		pr_err("IspKo:Err: ioremap gpGpioRegAddr failed!\n");
		ret = -ENOMEM;
		goto unmap_isp;
	}
	ret = request_irq(ISP_IRQ, isp_irq, IRQF_SHARED, "isp_irq",
			  &isp_irq_dev);
	if (ret) {
		pr_err("IspKo:Err: request_irq VI_FRONT_V_IRQ failed!\n");
		goto unmap_gpio;
	}
	ret = misc_register(&isp_misc);
	if (ret) {
		pr_err("IspKo:Err: misc_register failed!\n");
		goto free_irq;
	}
	proc_create("isp", 0444, NULL, &isp_proc_fops);
	pr_info("IspKo: openxiongmai xm510_isp init ok\n");
	return 0;

free_irq:
	free_irq(ISP_IRQ, &isp_irq_dev);
unmap_gpio:
	iounmap(gpio_regs);
unmap_isp:
	iounmap(isp_regs);
	return ret;
}

static void __exit xm510_isp_exit(void)
{
	remove_proc_entry("isp", NULL);
	misc_deregister(&isp_misc);
	free_irq(ISP_IRQ, &isp_irq_dev);
	iounmap(gpio_regs);
	iounmap(isp_regs);
	/* nothing maps it now: a mapping holds /dev/isp, and the module */
	if (shared_page)
		__free_page(shared_page);
	pr_info("IspKo: ispDev_exit OK!\n");
}

module_init(xm510_isp_init);
module_exit(xm510_isp_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("XM510 ISP device, compatible with xm510_isp.ko");
