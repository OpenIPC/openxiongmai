// SPDX-License-Identifier: GPL-2.0
/*
 * The XM533's ISP (/dev/isp): a drop-in for the vendor isp.ko. User space
 * (the sensor library) sets the ISP up and hands it parameter blocks;
 * isp.ko writes them to the ISP's registers once a frame, from its BPU
 * thread, woken by the VICU's frame interrupt (IRQ 57). It also passes
 * register reads and writes through, gives the AWB statistics (read()),
 * and maps 8 pages user space shares with itself (mmap()).
 *
 *   0x400c4901 / 02  {addr, val, -}: a register written / read
 *   0x400c490c   {-, pages, -}: the pages mmap() may map
 *   0x4908       {chn, ...} 28 bytes: the channel's frame timing
 *   0x490a       wait for a frame (2 s): the frames since the last call
 *   0x490b       {chn}: the frame's statistics and VPSS work done now
 *   0x490e       52 bytes: the channel (vicu_chn_init, bpu_chn_init)
 *   0x4958 / 59  {chn << 24 | v}: the BPU on / off, the WDR mode
 *   0x495a       36 bytes: dump a raw frame to {addr, size}
 *   0x495c       60 bytes: frame rates back
 *   0x495d       40 bytes: the library's version in (word 0 = 0) or out
 *   0x495e       {chn}: the VICU started
 *   0x495f       reset;  0x4963 {held}: reset, the VPSS group held
 *   0x4960       {chn, on}: wait for the parameters before frames
 *   0x4961       16 bytes: the channel's bindings
 *   0x4962       20 bytes: w, h, mode | blocks << 8 | channels << 16 | ...
 *   0x4980       {chn, x | y << 16, w | h << 16}: the VICU's crop
 *   0x4964..     the parameter blocks (iw.c, vpss_isp.c)
 *
 * mode= as isp.ko's (the product, shown in /proc/umap/isp).
 */

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/gfp.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/kthread.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/sched.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#include "isp.h"

#define VICU_IRQ 57
#define ISP_PAGES_ORDER 3 /* 8 pages */

static int mode;
module_param(mode, int, 0444);

u8 isp_comm[COMM_SIZE] __aligned(8);
DEFINE_MUTEX(bpu_lock);
DEFINE_MUTEX(iwctrl_lock);
DEFINE_MUTEX(iwctrl_vpss_lock);
DEFINE_MUTEX(vpss_limitup_lock);
DECLARE_WAIT_QUEUE_HEAD(dump_wq);
DECLARE_WAIT_QUEUE_HEAD(vsync_wq);
DECLARE_WAIT_QUEUE_HEAD(bpu_thread_wq);
int isp_vpss_grp;

/* The register windows user space and the frame work may write */
static const u32 win_start[4] = { 0x30000000, 0x30020000, 0x30040000,
				  0x300f0000 };
static const u32 win_end[4] = { 0x3001a000, 0x30035000, 0x3006e000,
				0x300f0400 };
static void __iomem *win[4];
static void __iomem *pll;

static struct page *isp_pages;
static u8 isp_pages_max = 8; /* 0x400c490c */
static u32 awb_chn;          /* write(): the channel read() gives */
static atomic_t isp_users;
static struct task_struct *bpu_task;
static struct proc_dir_entry *proc;
static DEFINE_SPINLOCK(awb_lock);

int isp_write_reg(u32 addr, u32 val)
{
	int i;

	for (i = 0; i < 4; i++)
		if (addr >= win_start[i] && addr < win_end[i])
			break;
	if (i == 4) {
		pr_info("The WriteReg Addr 0x%x is outside, cmd error!\n", addr);
		return -1;
	}
	if (!win[i])
		return -1;
	writel(val, win[i] + (addr - win_start[i]));
	return 0;
}

u32 isp_read_reg(u32 addr)
{
	int i;

	for (i = 0; i < 4; i++)
		if (addr >= win_start[i] && addr < win_end[i])
			break;
	if (i == 4) {
		pr_info("The ReadReg Addr 0x%x is outside, cmd error!\n", addr);
		return -1;
	}
	if (!win[i])
		return -1;
	return readl(win[i] + (addr - win_start[i]));
}

int pll_write_reg(u32 addr, u32 val)
{
	if (addr - 0x20000000 > 0x3ff) {
		pr_info("The WriteReg Addr 0x%x is outside, cmd error!\n", addr);
		return -1;
	}
	if (!pll)
		return -1;
	writel(val, pll + (addr - 0x20000000));
	return 0;
}

u32 pll_read_reg(u32 addr)
{
	if (addr - 0x20000000 > 0x3ff) {
		pr_info("The WriteReg Addr 0x%x is outside, cmd error!\n", addr);
		return -1;
	}
	if (!pll)
		return -1;
	return readl(pll + (addr - 0x20000000));
}

static void isp_info_init(void)
{
	memset(CP(0xc8), 0, 0x50);   /* the VICU's counters */
	memset(CP(0x140), 0, 0x38);  /* the BPU's */
	memset(CP(CM_CHN_ATTR), 0, 0x1c);
	memset(CP(0x178), 0, 0x38);
}

static void isp_reset(void)
{
	vicu_reset();
	bpu_reset();
	vpss_isp_reset(isp_vpss_grp);
	isp_info_init();
}

static void isp_exit(void)
{
	unsigned long flags;

	C32(CM_IW_WAIT) = 0xff;
	C32(CM_VPSS_HOLD) = 0xff;
	local_irq_save(flags);
	C32(CM_RUN) = 0;
	local_irq_restore(flags);
	isp_reset();
	C8(CM_BPU_BUSY) = 0;
}

static ssize_t isp_awb_read(struct file *f, char __user *buf, size_t count,
			    loff_t *pos)
{
	static u8 stat[0xc00];
	unsigned long flags;

	if (awb_chn != 0)
		return -1;
	if (count < sizeof(stat))
		pr_err("IspKo:Err: isp_awb_read count:%zu size:%d\n", count,
		       (int)sizeof(stat));
	else
		count = sizeof(stat);
	spin_lock_irqsave(&awb_lock, flags);
	memcpy(stat, CP(CM_AWB_STAT + awb_chn * sizeof(stat)), count);
	spin_unlock_irqrestore(&awb_lock, flags);
	if (copy_to_user(buf, stat, count))
		return -1;
	return count;
}

static ssize_t isp_awb_write(struct file *f, const char __user *buf,
			     size_t count, loff_t *pos)
{
	awb_chn = 0;
	if (count > 4)
		count = 4;
	if (copy_from_user(&awb_chn, buf, count))
		return -1;
	return count;
}

static void isp_vma_open(struct vm_area_struct *vma)
{
}

static void isp_vma_close(struct vm_area_struct *vma)
{
}

static const struct vm_operations_struct isp_vm_ops = {
	.open = isp_vma_open,
	.close = isp_vma_close,
};

static int isp_mmap(struct file *f, struct vm_area_struct *vma)
{
	unsigned long size = vma->vm_end - vma->vm_start;

	if (!isp_pages || isp_pages_max > 8 ||
	    size > PAGE_SIZE << ISP_PAGES_ORDER) {
		pr_err("IspKo:Err: isp_mmap failed! [need:%d get:%d]\n",
		       isp_pages_max, 8);
		return -1;
	}
	if (remap_pfn_range(vma, vma->vm_start, page_to_pfn(isp_pages), size,
			    vma->vm_page_prot)) {
		pr_err("IspKo: [IspDrv] err: remap_pfn_range failed!\n");
		return -1;
	}
	vma->vm_ops = &isp_vm_ops;
	return 0;
}

/* the BPU thread works while /dev/isp is open */
int isp_users_open(void)
{
	return atomic_read(&isp_users);
}

static int isp_open(struct inode *inode, struct file *f)
{
	atomic_inc(&isp_users);
	return 0;
}

static int isp_release(struct inode *inode, struct file *f)
{
	atomic_dec(&isp_users);
	return 0;
}

#define GET(n)                                          \
	do {                                            \
		if (copy_from_user(w, arg, (n)))        \
			return -1;                      \
	} while (0)

/* 0x490a: wait for a frame, at most 200 jiffies */
static long isp_wait_frame(void)
{
	long t = 200, ret;

	if ((int)C32(CM_VSYNC_CNT) < 1) {
		t = wait_event_interruptible_timeout(
			vsync_wq, (int)C32(CM_VSYNC_CNT) > 0, t);
		if (!t)
			return -ERESTARTSYS; /* isp.ko's -512 on a timeout */
	}
	ret = C32(CM_VSYNC_CNT);
	C32(CM_VSYNC_CNT) = 0;
	return ret;
}

/* 0x495a: dump the next raw frame to w[3] (w[5] bytes) */
static long isp_dump(void __user *arg, u32 *w)
{
	unsigned long flags;
	long ret, t;

	if ((u8)w[0] != 0) {
		ret = -1;
	} else if (C8(CM_DUMP) != 0) {
		C8(CM_DUMP) = 0;
		ret = -1;
	} else {
		C16(CM_DUMP_W) = w[0] >> 16;
		w[0] &= 0xffff0000;
		local_irq_save(flags);
		C8(CM_DUMP_REQ) = 0;
		C8(CM_DUMP) = 1;
		C16(CM_DUMP_H) = w[1];
		C32(CM_DUMP_ADDR) = w[3];
		C32(CM_DUMP_SIZE) = w[5];
		local_irq_restore(flags);
		t = wait_event_interruptible_timeout(dump_wq,
						     C8(CM_DUMP) & 0x30, 50);
		if (!t || (C8(CM_DUMP) & 0x20)) {
			ret = -1;
			w[0] = (w[0] & ~0xff00) | 0x1100;
		} else {
			w[2] = (w[1] & 0xffff) * (((w[0] >> 16) + 9) / 10) *
			       0x10;
			ret = 0;
		}
		C8(CM_DUMP) = 0;
	}
	if (copy_to_user(arg, w, 0x24)) {
		pr_err("IspKo:Err: IspRegRead : copy_to_user error.\n");
		return -1;
	}
	return ret;
}

/* 0x495c: the frame rates */
static long isp_get_fps(void __user *arg, u32 *w)
{
	static u32 last_cnt;
	static u8 same;
	u16 a, b, c;

	memset(&w[7], 0, 8 * 4);
	a = b = c = C16(CM_FPS_CTRL) & 1;
	if (C16(CM_FPS_CTRL) & 1) {
		if (last_cnt == C32(0x154)) {
			a = 0;
			if (same < 0xf0 && ++same < 3)
				a = C32(0x150);
		} else {
			last_cnt = C32(0x154);
			same = 0;
			a = C32(0x150);
		}
		b = C32(0xe18);
		c = C32(0xe1c);
	}
	w[4] = (w[4] & 0xffff) | a << 16;
	w[0] = C16(CM_FPS_CTRL) | b << 16;
	w[2] = (w[2] & 0xffff) | c << 16;
	if (copy_to_user(arg, w, 0x3c)) {
		pr_err("IspKo:Err: CMD_ISP_GETFPS : copy_to_user error.\n");
		return -1;
	}
	return 0;
}

/* 0x495d: the library's version, in or out */
static long isp_version(void __user *arg, u32 *w)
{
	if (w[0]) {
		w[1] = C32(0x175c);
		w[2] = C32(0x1760);
		w[3] = C32(0x1764);
		w[4] = C32(0x1768);
		w[5] = C32(0x176c);
		w[6] = C32(0x1774);
		w[7] = C32(0x1778);
		w[8] = C32(0x177c);
		w[9] = C32(0x1770);
		if (copy_to_user(arg, w, 0x28)) {
			pr_err("IspKo:Err: CMD_ISP_VERSION copy_to_user error.\n");
			return -1;
		}
		return 0;
	}
	C32(0x175c) = w[1];
	C32(0x1760) = w[2];
	C32(0x1764) = w[3];
	C32(0x1768) = w[4];
	C32(0x176c) = w[5];
	C32(0x1774) = w[6];
	C32(0x1778) = w[7];
	C32(0x177c) = w[8];
	C32(0x1780) = w[9]; /* (read back from 0x1770, as isp.ko) */
	return 0;
}

/* 0x4962: the ISP set up for a sensor's picture */
static long isp_comm_init(const u32 *w)
{
	u32 pool[3];
	int ret;

	C32(CM_WIDTH) = w[0];
	C32(CM_HEIGHT) = w[1];
	C32(CM_MODE) = w[2];
	C32(0x68) = w[3];
	C32(0x6c) = w[4];
	pr_info("IspKo: %d*%d blkNum[%d]\n", w[0], w[1], C8(CM_BLKNUM));
	pool[0] = 1;
	pool[1] = ((C32(CM_WIDTH) + 9) / 10) * C32(CM_HEIGHT) * 0x10;
	pool[2] = C8(CM_BLKNUM);
	ret = ib_init(pool, 0, 0);
	isp_reset();
	vicu_init_queue();
	bpu_init_queue();
	vpss_init_queue();
	C32(0xdcc) = 0;
	C32(CM_IW_WAIT) = 0;
	C32(CM_BPU_GO) = 0;
	C32(CM_VPSS_HOLD) = 0;
	C32(CM_VSYNC_CNT) = 0;
	mutex_lock(&vpss_limitup_lock);
	C8(CM_FLAGS) |= 0x20;
	mutex_unlock(&vpss_limitup_lock);
	C32(CM_REPAIR) = 0xffffffff;
	bpu_init();
	C32(CM_RUN) = 1;
	isp_vpss_grp = 0;
	return ret;
}

static long isp_ioctl(struct file *f, unsigned int cmd, unsigned long a)
{
	void __user *arg = (void __user *)a;
	u32 w[0x3c / 4];
	int chn, ret;

	switch (cmd) {
	case 0x400c4901:
		GET(0xc);
		isp_write_reg(w[0], w[1]);
		return 0;
	case 0x400c4902:
		if (copy_from_user(w, arg, 0xc)) {
			pr_err("IspKo:Err: Isp %d: copy_from_user error.\n", cmd);
			return -1;
		}
		w[1] = isp_read_reg(w[0]);
		if (copy_to_user(arg, w, 0xc)) {
			pr_err("IspKo:Err: IspRegRead : copy_to_user error.\n");
			return -1;
		}
		return 0;
	case 0x400c4903:
		pr_err("IspKo:Err: CMD_REG_READS : not support!\n");
		return 0;
	case 0x400c490c:
		GET(0xc);
		isp_pages_max = w[1];
		return 0;
	case 0x4908:
		if (copy_from_user(w, arg, 0x1c)) {
			pr_err("Isp %d: copy_from_user error.\n", cmd);
			return -1;
		}
		chn = w[0] & 0xf;
		if (chn > 1)
			return 0;
		memcpy(CP(CM_CHN_ATTR + chn * 0x1c), w, 0x1c);
		return 0;
	case 0x490a:
		if (!C8(CM_CHNNUM)) {
			pr_err("IspKo:Err: %s line%d: ChnNum = %d\n", "isp_ioctl",
			       0x193, 0);
			return -1;
		}
		GET(0xc);
		return isp_wait_frame();
	case 0x490b:
		GET(4);
		if (w[0] != 0)
			return 0;
		bpu_post_deal(0);
		vpss_isp_post_deal(0, C32(CM_VICU_SIZE) & 0xffff,
				   C32(CM_VICU_SIZE) >> 16);
		return 0;
	case 0x490e:
		GET(0x34);
		ret = vicu_chn_init((u8 *)w);
		bpu_chn_init((u8 *)w);
		return ret;
	case 0x4958:
		GET(4);
		return bpu_set_enable(w[0] >> 24, w[0] & 0xff);
	case 0x4959:
		GET(4);
		return bpu_set_wdr_mode(w[0] >> 24, w[0] & 0xff);
	case 0x495a:
		if (copy_from_user(w, arg, 0x24)) {
			pr_err("IspKo:Err: Isp %d: copy_from_user error.\n", cmd);
			return -1;
		}
		return isp_dump(arg, w);
	case 0x495b:
		if (copy_from_user(w, arg, 0x24)) {
			pr_err("IspKo:Err: Isp %d: copy_from_user error.\n", cmd);
			return -1;
		}
		return 0;
	case 0x495c:
		if (copy_from_user(w, arg, 0x3c)) {
			pr_err("IspKo:Err: Isp %d: copy_from_user error.\n", cmd);
			return -1;
		}
		return isp_get_fps(arg, w);
	case 0x495d:
		GET(0x28);
		return isp_version(arg, w);
	case 0x495e:
		GET(4);
		vicu_run(w[0]);
		return 0;
	case 0x495f:
		isp_exit();
		return 0;
	case 0x4960:
		GET(8);
		if (!w[1]) {
			C32(CM_IW_WAIT) = 0;
			C32(CM_VPSS_HOLD) = 0;
			return 0;
		}
		if (w[0] > 0) /* one channel */
			return 0;
		C32(CM_IW_WAIT) = 1 << (w[0] & 0xff);
		C32(CM_VPSS_HOLD) = 1 << (C32(0xde0 + w[0] * 0x980) & 0xff);
		return 0;
	case 0x4961:
		GET(0x10);
		if ((int)w[0] > 0)
			return -1;
		C32(0xde4 + w[0] * 0x980) = w[3];
		C32(0xddc + w[0] * 0x980) = w[1];
		C32(0xde0 + w[0] * 0x980) = w[2];
		return 0;
	case 0x4962:
		if (copy_from_user(w, arg, 0x14)) {
			pr_err("IspKo:Err: Isp %d: copy_from_user error.\n", cmd);
			return -1;
		}
		return isp_comm_init(w);
	case 0x4963:
		GET(4);
		isp_vpss_grp = w[0] != 0;
		isp_exit();
		return 0;
	case 0x4980:
		GET(0xc);
		if (w[0] != 0)
			return -1;
		C32(0x70) = w[1];
		C32(0x74) = w[2];
		C32(0xb4) = 1;
		C32(0xb8) = w[1] & 0xffff;
		C32(0xbc) = w[1] >> 16;
		C32(0xc0) = w[2] & 0xffff;
		C32(0xc4) = w[2] >> 16;
		return 0;
	}
	if (!C8(CM_CHNNUM)) {
		pr_err("IspKo:Err: %s line%d: ChnNum = %d\n", "isp_ioctl", 0x27b,
		       0);
		return -1;
	}
	if (iw_ioctl(cmd, arg) == 0)
		return 0;
	ret = iw_vpss_ioctl(cmd, arg);
	if (ret) {
		pr_err("IspKo:Err: command = %c 0x%x error!\n",
		       (cmd & 0xffff) >> 8, cmd & 0xff);
		return ret;
	}
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

static const char *wdr_name(u32 m)
{
	return m == 2 ? "hdr" : m == 1 ? "build-in" : m == 0 ? "linear" : "---";
}

static int isp_proc_show(struct seq_file *m, void *v)
{
	int n = 0;
	u32 a, b;

	seq_printf(m, "[%s] BUILD TIME:[%s %s] [V%d.%d.%d.%d.%d.%d.%d]\n", "ISP",
		   "open_isp", "", C32(0x175c), C32(0x1760), C32(0x176c),
		   C32(0x1768), C32(0x1770), C32(0x1764), C32(0x1774));
	seq_printf(m, "Mode(Product):%d\n", C32(CM_PRODUCT));
	if (!C32(CM_RUN))
		return 0;
	switch (C8(CM_MODE)) {
	case 0:
		seq_puts(m, "BpuMode:[Online]\n");
		break;
	case 1:
		seq_printf(m, "BpuMode:[Offline] [%s]\n",
			   C8(CM_OPT) & 1 ? "MemorySaver" : "Normal");
		break;
	case 2:
		seq_puts(m, "BpuMode:[HDR(line by line)]\n");
		break;
	case 3:
		seq_puts(m, "BpuMode:[HDR(build-in)]\n");
		break;
	default:
		seq_puts(m, "BpuMode:[UNKOWN]\n");
	}
	seq_printf(m, "VpssMode:[%s]\n",
		   C8(CM_VPSS_MODE) == 1 ? "Offline" : "Online");
	vpss_isp_proc_show(m, &n);
	seq_printf(m, "[%d.VICU_info]:\n", n++);
	seq_puts(m, "chn\tfps\tcount\tib_fail\tib_success\tsns_id\tmode\n");
	seq_printf(m, "%d\t%d.%d\t%d\t%d\t%d\t\t%d\t%s\n", C32(0xc8),
		   C32(CM_VICU_FPS) / 10, C32(CM_VICU_FPS) % 10,
		   C32(CM_VICU_CNT), C32(0xf8), C32(0xfc), C32(CM_VICU_MODE),
		   wdr_name(C32(CM_WDR_MODE)));
	if (C8(CM_MODE) == 1) {
		seq_printf(m, "\n[%d.BPU_Info]:\n", n++);
		seq_puts(m, "chn\tfps\tcount\ttime\ttimeout\tvb_fail\tvb_success\tmode\n");
		seq_printf(m, "%d\t%d.%d\t%d\t%lld\t%d\t%d\t%d\t\t%s\n",
			   C32(0x140), C32(0x150) / 10, C32(0x150) % 10,
			   C32(0x154), *(s64 *)CP(0x158), C32(0x170),
			   C32(0x174), C32(0xe14), wdr_name(C32(0xdf8)));
		seq_puts(m, "all(chn):\n");
		seq_printf(m, "*\t%d.%d\t%d \n", C32(0x188) / 10,
			   C32(0x188) % 10, C32(0x18c));
	}
	seq_printf(m, "\n[%d.VI_info]:\nchn\tTotalSize\tActiveSize\n", n++);
	a = isp_read_reg(0x30010090);
	b = isp_read_reg(0x30010094);
	seq_printf(m, "%d\t%4d*%4d\t%4d*%4d\n", 0, b & 0xffff, b >> 16,
		   a & 0xffff, a >> 16);
	seq_printf(m, "\n[%d.MIPI_info]:\nID\tTotalSize(mipi)\tActiveSize(mipi)\tTotalSize(pix)\tActiveSize(pix)\tRepair\n",
		   n++);
	seq_printf(m, "0x%2x\t%4d*%4d\t%4d*%4d\t\t%4d*%4d\t%4d*%4d\t%4d\n",
		   isp_read_reg(0x30000050), isp_read_reg(0x30000048),
		   isp_read_reg(0x3000004c), isp_read_reg(0x30000040),
		   isp_read_reg(0x30000044), isp_read_reg(0x30000114),
		   isp_read_reg(0x30000118), isp_read_reg(0x3000010c),
		   isp_read_reg(0x30000110), 0);
	seq_printf(m, "\t%4d*%4d\t%4d*%4d\t\t%4d*%4d\t%4d*%4d\n",
		   isp_read_reg(0x30000180), isp_read_reg(0x30000188),
		   isp_read_reg(0x3000017c), isp_read_reg(0x30000184),
		   isp_read_reg(0x30000124), isp_read_reg(0x30000128),
		   isp_read_reg(0x3000011c), isp_read_reg(0x30000120));
	seq_printf(m, "\n[%d.Cut_info]:\nMod\tchn\tFlag\tX\tY\tWidth\tHeight\tNote\n",
		   n++);
	seq_printf(m, "vicu\t%d\t%d\t%d\t%d\t%d\t%d\t%s\n", 0, C32(0xb4),
		   C32(0xb8), C32(0xbc), C32(0xc0), C32(0xc4),
		   C32(0xb4) ? "Use" : "UnUse");
	seq_printf(m, "vicu\t%d\t%d\t%d\t%d\t%d\t%d\t%s\n", 0, 1,
		   C32(0x70) & 0xffff, C32(0x70) >> 16, C32(0x74) & 0xffff,
		   C32(0x74) >> 16, "  ");
	seq_printf(m, "h_d\t%d\t%d\t \t \t%d\t%d\t%s\n", 0, 1,
		   C32(0x78) & 0xffff, C32(0x78) >> 16, "  ");
	seq_printf(m, "hv_gen\t%d\t%d\t%d\t%d\t%d\t%d\t%s\n", 0, 1,
		   C32(0x7c) & 0xffff, C32(0x7c) >> 16, C32(0x80) & 0xffff,
		   C32(0x80) >> 16, "  ");
	seq_printf(m, "bpu_cut\t%d\t%d\t%d\t%d\t%d\t%d\t%s\n", 0, C32(0xe20),
		   C32(0x88) & 0xffff, C32(0x88) >> 16, C32(0x8c) & 0xffff,
		   C32(0x8c) >> 16, C32(0xe20) ? "Use" : "UnUse");
	seq_printf(m, "vpss_in\t%d\t%d\t \t \t%d\t%d\t%s\n", 0, 1,
		   C32(CM_VICU_SIZE) & 0xffff, C32(CM_VICU_SIZE) >> 16, "  ");
	seq_printf(m, "drv\tall\t0x%x\t%d\t%d\t%d\t%d\t%s\n", C8(CM_FLAGS),
		   C32(0x1784), C32(0x1788), C32(0x178c), C32(0x1790),
		   C8(CM_MODE) != 1 ? "Use" : "UnUse");
	ib_proc_show(m, &n);
	return 0;
}

static int isp_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, isp_proc_show, NULL);
}

static const struct file_operations isp_proc_fops = {
	.owner = THIS_MODULE,
	.open = isp_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int __init isp_init(void)
{
	int i, ret;

	memset(isp_comm, 0, sizeof(isp_comm));
	for (i = 0; i < 4; i++) {
		win[i] = ioremap(win_start[i], win_end[i] - win_start[i]);
		if (!win[i]) {
			pr_err("IspKo:Err: ioremap isp_regaddr %d failed!\n", i);
			ret = -ENOMEM;
			goto unmap;
		}
	}
	pll = ioremap(0x20000000, 0x400);
	if (!pll) {
		pr_err("IspKo:Err: ioremap pll_regaddr failed!\n");
		ret = -ENOMEM;
		goto unmap;
	}
	C32(0x1764) = 0xffffffff;
	C32(0x1768) = 0xffffffff;
	C32(0x176c) = 0xffffffff;
	C32(0x1770) = 0;
	C32(CM_PRODUCT) = mode;
	C32(CM_RUN) = 0;
	C8(CM_FLAGS) = 1;
	C32(CM_REPAIR) = 0xffffffff;
	proc = proc_create("umap/isp", 0444, NULL, &isp_proc_fops);
	isp_info_init();
	isp_pages = alloc_pages(GFP_KERNEL, ISP_PAGES_ORDER);
	if (!isp_pages) {
		pr_err("IspKo:Err: alloc_pages failed!\n");
		ret = -EIO;
		goto proc;
	}
	memset(page_address(isp_pages), 0, PAGE_SIZE << ISP_PAGES_ORDER);
	ret = request_irq(VICU_IRQ, vicu_irq_0, 0, "vicu_bayer_irq0", NULL);
	if (ret) {
		pr_err("IspKo:Err: vicu_bayer_irq failed!\n");
		ret = -EIO;
		goto pages;
	}
	bpu_task = kthread_run(bpu_thread, NULL, "bpu");
	if (IS_ERR(bpu_task))
		bpu_task = NULL;
	ret = misc_register(&isp_misc);
	if (ret) {
		pr_err("IspKo:Err: misc_register failed!\n");
		ret = -EPERM;
		goto thread;
	}
	iw_isp_init();
	pr_info("IspKo: build: open_isp\n");
	return 0;
thread:
	if (bpu_task)
		kthread_stop(bpu_task);
	free_irq(VICU_IRQ, NULL);
pages:
	__free_pages(isp_pages, ISP_PAGES_ORDER);
proc:
	if (proc)
		remove_proc_entry("umap/isp", NULL);
unmap:
	for (i = 0; i < 4; i++)
		if (win[i])
			iounmap(win[i]);
	if (pll)
		iounmap(pll);
	return ret;
}

static void __exit isp_cleanup(void)
{
	int i;

	if (bpu_task)
		kthread_stop(bpu_task);
	misc_deregister(&isp_misc);
	free_irq(VICU_IRQ, NULL);
	isp_exit();
	for (i = 0; i < 4; i++)
		iounmap(win[i]);
	iounmap(pll);
	if (proc)
		remove_proc_entry("umap/isp", NULL);
	__free_pages(isp_pages, ISP_PAGES_ORDER);
	pr_info("IspKo: rmmod ispDev OK!\n");
}

module_init(isp_init);
module_exit(isp_cleanup);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("XM533 ISP (open isp.ko)");
