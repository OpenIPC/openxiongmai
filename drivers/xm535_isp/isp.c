// SPDX-License-Identifier: GPL-2.0
/*
 * The XM535D1/D2's ISP (/dev/isp): a drop-in for the vendor isp.ko. Up to
 * three channels on two VICUs; the BPU works online, or offline from ISP
 * buffers (ib.c) the VICU stores, its output straight into vpss.ko's
 * buffers. The sensor library hands it parameter blocks (iw.c, vpss_isp.c)
 * that go to the registers once a frame.
 *
 *   0x400c4901 / 02  {addr, val, -}: a register written / read
 *   0x400c490c   {-, pages, -}: the pages mmap() may map
 *   0x4908       28 bytes: a channel's frame timing (channels 0, 1)
 *   0x490a       wait for a frame (2 s): the channels with one
 *   0x490b       {chn}: the frame's statistics now
 *   0x490e       76 bytes: the channels (vicu_chn_init, bpu_chn_init)
 *   0x4914       {chn, on}: the channel's VICU on
 *   0x4958 / 59  {chn << 24 | v}: the BPU on / off, the WDR mode
 *   0x495a       36 bytes: dump a raw frame;  0x495c frame rates
 *   0x495d       40 bytes: the library's version in or out
 *   0x495e       {chn}: the VICU started;  0x495f / 0x4963 reset
 *   0x4960       {chn, on}: wait for the parameters before frames
 *   0x4961       16 bytes: the channel's bindings
 *   0x4962       56 bytes: w, h, mode | channels << 8 | ..., ISP buffers
 *   0x4980       48 bytes: the channel's cmos interface windows
 *   0x4964..     the parameter blocks
 *
 * mode= as isp.ko's (the product).
 */

#include <linux/fs.h>
#include <linux/gfp.h>
#include <linux/io.h>
#include <linux/kthread.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/sched.h>
#include <linux/uaccess.h>

#include "isp.h"

#define VICU_IRQ0 87
#define VICU_IRQ1 88
#define BPU_IRQ 62
#define NR3D_IRQ 69
#define TIMER_IRQ 38
#define ISP_PAGES_ORDER 4

static int mode;
module_param(mode, int, 0444);

u8 isp_comm[COMM_SIZE] __aligned(8);
struct mutex grp_lock[N_CHN] = { __MUTEX_INITIALIZER(grp_lock[0]),
				 __MUTEX_INITIALIZER(grp_lock[1]),
				 __MUTEX_INITIALIZER(grp_lock[2]) };
DEFINE_MUTEX(iwctrl_lock);
DEFINE_MUTEX(iwctrl_vpss_lock);
DECLARE_WAIT_QUEUE_HEAD(dump_wq);
DECLARE_WAIT_QUEUE_HEAD(vsync_wq);
DECLARE_WAIT_QUEUE_HEAD(bpu_thread_wq);
int isp_vpss_grp;
DECLARE_TASKLET(isp_irq0_tasklet, vicu_frame, 0);
DECLARE_TASKLET(isp_irq1_tasklet, vicu_stored, 0);
DECLARE_TASKLET(isp_bpu_tasklet, bpu_offline_deal_in, 0);

static const u32 win_start[4] = { 0x30000000, 0x32000000, 0x320f0000,
				  0x34000000 };
static const u32 win_end[4] = { 0x30083000, 0x32081000, 0x32101000,
				0x34090000 };
static void __iomem *win[4];
static void __iomem *pll;
static void __iomem *timer;

static struct page *isp_pages;
static u8 isp_pages_max = 0x10;
static u32 awb_chn;
static atomic_t isp_users;
static struct task_struct *bpu_task;
static struct proc_dir_entry *proc;
static bool nr3d_irq_on, timer_irq_on;
static u8 fps_same[N_CHN];
static u32 fps_last[N_CHN];

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
	if (addr - 0x20000000 > 0x3ff || !pll) {
		pr_info("The WriteReg Addr 0x%x is outside, cmd error!\n", addr);
		return -1;
	}
	writel(val, pll + (addr - 0x20000000));
	return 0;
}

u32 pll_read_reg(u32 addr)
{
	if (addr - 0x20000000 > 0x3ff || !pll) {
		pr_info("The WriteReg Addr 0x%x is outside, cmd error!\n", addr);
		return -1;
	}
	return readl(pll + (addr - 0x20000000));
}

/* timer 4/5, for the time-shared VICU */
int timer_write_reg(u32 addr, u32 val)
{
	if (addr - 0x100d1000 > 0x3ff || !timer) {
		pr_info("The WriteReg Addr 0x%x is outside, cmd error!\n", addr);
		return -1;
	}
	writel(val, timer + (addr - 0x100d1000));
	return 0;
}

u32 timer_read_reg(u32 addr)
{
	if (addr - 0x100d1000 > 0x3ff || !timer) {
		pr_info("The WriteReg Addr 0x%x is outside, cmd error!\n", addr);
		return -1;
	}
	return readl(timer + (addr - 0x100d1000));
}

int isp_users_open(void)
{
	return atomic_read(&isp_users);
}

static void isp_info_init(void)
{
	int i;

	for (i = 0; i < N_CHN; i++) {
		memset(CP(CM_VICU(i) + 0x50), 0, 0x48);
		memset(CP(CM_BPUSTAT(i)), 0, 0x30);
		memset(CP(CM_CHN_ATTR(i)), 0, 0x1c);
		memset(CP(CM_TIME(i)), 0, 0x80);
	}
	memset(CP(CM_BPUALL), 0, 0x30);
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
	ISP_LOCK(flags);
	C32(CM_RUN) = 0;
	ISP_UNLOCK(flags);
	isp_reset();
	ib_exit();
	C8(CM_BPU_BUSY) = 0;
}

static ssize_t isp_awb_read(struct file *f, char __user *buf, size_t count,
			    loff_t *pos)
{
	static u8 stat[0xc00];
	unsigned long flags;
	u32 c = awb_chn;

	if (c >= N_CHN)
		return -1;
	if (count < sizeof(stat))
		pr_err("IspKo:Err: isp_awb_read count:%zu size:%d\n", count,
		       (int)sizeof(stat));
	else
		count = sizeof(stat);
	ISP_LOCK(flags);
	memcpy(stat, CP(CM_AWB_STAT + c * sizeof(stat)), count);
	ISP_UNLOCK(flags);
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

	if (!isp_pages || isp_pages_max > 0x10 ||
	    size > PAGE_SIZE << ISP_PAGES_ORDER) {
		pr_err("IspKo:Err: isp_mmap failed! [need:%d get:%d]\n",
		       isp_pages_max, 0x10);
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

static long chnnum_err(int line)
{
	pr_err("IspKo:Err: %s line%d: ChnNum = %d\n", "isp_ioctl", line, 0);
	return -1;
}

static long isp_wait_frame(void)
{
	unsigned long flags;
	u32 ret = 0, c;
	long t;

	if ((int)C32(CM_VSYNC_CNT) < 1) {
		t = wait_event_interruptible_timeout(
			vsync_wq, (int)C32(CM_VSYNC_CNT) > 0, 200);
		if (t <= 0)
			return -ERESTARTSYS;
	}
	C32(CM_VSYNC_CNT) = 0;
	for (c = 0; c < N_CHN; c++) {
		ISP_LOCK(flags);
		ret |= (C32(CM_VB(c)) & 1) << c;
		C32(CM_VB(c)) = 0;
		ISP_UNLOCK(flags);
	}
	return ret;
}

static long isp_dump(void __user *arg, u32 *w)
{
	unsigned long flags;
	u32 chn = w[0] & 0xff;
	long ret = -1, t;

	if (chn < N_CHN) {
		if (C8(CM_DUMP) == 0) {
			w[0] = (w[0] & 0xffff0000) | chn;
			ISP_LOCK(flags);
			C8(CM_DUMP_CHN) = chn;
			C8(CM_DUMP) = 1;
			C16(CM_DUMP_W) = w[0] >> 16;
			C16(CM_DUMP_H) = w[1];
			C32(CM_DUMP_ADDR) = w[3];
			C32(CM_DUMP_SIZE) = w[5];
			ISP_UNLOCK(flags);
			t = 1;
			if (!(C8(CM_DUMP) & 0x30))
				t = wait_event_interruptible_timeout(
					dump_wq, C8(CM_DUMP) & 0x30, 50);
			if (t <= 0 || (C8(CM_DUMP) & 0x20)) {
				w[0] = (w[0] & ~0xff00) | 0x1100;
			} else {
				w[2] = (w[1] & 0xffff) *
				       (((w[0] >> 16) + 9) / 10) * 0x10;
				ret = 0;
			}
		}
		C8(CM_DUMP) = 0;
	}
	if (copy_to_user(arg, w, 0x24)) {
		pr_err("IspKo:Err: IspRegRead : copy_to_user error.\n");
		return -1;
	}
	return ret;
}

static long isp_get_fps(void __user *arg, u32 *w)
{
	u16 *h = (u16 *)w;
	u16 a, b, c;
	int i;

	h[0] = C8(CM_FPS_MASK);
	memset(&w[7], 0, 8 * 4);
	for (i = 0; i < N_CHN; i++) {
		a = b = c = 0;
		if (C8(CM_FPS_MASK) >> i & 1) {
			if (fps_last[i] == C32(CM_BPUSTAT(i) + 0xc)) {
				if (fps_same[i] < 0xf0 && ++fps_same[i] < 3)
					a = C32(CM_BPUSTAT(i) + 8);
			} else {
				fps_last[i] = C32(CM_BPUSTAT(i) + 0xc);
				fps_same[i] = 0;
				a = C32(CM_BPUSTAT(i) + 8);
			}
			b = C16(CM_GRP(i) + 0x4c);
			c = C16(CM_GRP(i) + 0x50);
		}
		h[9 + i] = a;
		h[1 + i] = b;
		h[5 + i] = c;
	}
	if (copy_to_user(arg, w, 0x3c)) {
		pr_err("IspKo:Err: CMD_ISP_GETFPS : copy_to_user error.\n");
		return -1;
	}
	return 0;
}

static long isp_version(void __user *arg, u32 *w)
{
	if (w[0]) {
		w[1] = C32(CM_VER);
		w[2] = C32(CM_VER + 4);
		w[3] = C32(CM_VER + 8);
		w[4] = C32(CM_VER + 0xc);
		w[5] = C32(CM_VER + 0x10);
		w[6] = C32(CM_VER + 0x18);
		w[7] = C32(CM_VER + 0x1c);
		w[8] = C32(CM_VER + 0x20);
		w[9] = C32(CM_VER + 0x14);
		if (copy_to_user(arg, w, 0x28)) {
			pr_err("IspKo:Err: CMD_ISP_VERSION copy_to_user error.\n");
			return -1;
		}
		return 0;
	}
	C32(CM_VER) = w[1];
	C32(CM_VER + 4) = w[2];
	C32(CM_VER + 0xc) = w[4];
	C32(CM_VER + 0x10) = w[5];
	C32(CM_VER + 8) = w[3];
	C32(CM_VER + 0x18) = w[6];
	C32(CM_VER + 0x1c) = w[7];
	C32(CM_VER + 0x20) = w[8];
	C32(CM_VER + 0x24) = w[9]; /* (read back from 0x14, as isp.ko) */
	return 0;
}

static long isp_comm_init(const u32 *w)
{
	u32 pools[7] = { 0 };
	int i, ret;

	memcpy(CP(CM_INIT), w, 0x38);
	pools[0] = C32(0x7c) > 2 ? 3 : C32(0x7c);
	for (i = 0; i < pools[0]; i++) {
		pools[1 + i * 2] = C32(0x80 + i * 8);
		pools[2 + i * 2] = C32(0x84 + i * 8);
	}
	ret = ib_init(pools, 0, 0, C8(CM_OPT) & 3);
	isp_reset();
	C32(CM_BPU_RUN) = 0;
	C32(CM_IW_WAIT) = 0;
	C32(CM_BPU_GO) = 0;
	C32(CM_VPSS_HOLD) = 0;
	C32(CM_VSYNC_CNT) = 0;
	bpu_init();
	C32(CM_RUN) = 1;
	isp_vpss_grp = 0;
	if (C8(CM_VICU_MODE) != 2)
		return ret;
	/* the time-shared VICU's timer: 20 ms at 24 MHz */
	timer_write_reg(0x100d1008, 99);
	timer_write_reg(0x100d1000, 480000);
	timer_write_reg(0x100d1018, 480000);
	if (!timer_irq_on) {
		if (request_irq(TIMER_IRQ, timer45_interrupt, 0, "vicu_t", NULL))
			pr_err("IspKo:Err: vicu_t_irq failed!\n");
		else
			timer_irq_on = true;
	}
	return ret;
}

/* 0x4980: the cmos interface's windows, and the channel's copy of them */
static long isp_cmos_if(const u32 *w)
{
	u8 p[0x18] __aligned(4);
	u32 c = w[0];

	if (c > 2)
		return -1;
	memset(p, 0, sizeof(p));
	p[0] = 1;
	p[1] = 1;
	W16(p, 2) = w[4];
	W16(p, 4) = w[4] + w[6];
	W16(p, 6) = w[5];
	W16(p, 8) = w[5] + w[7];
	W32(p, 0xc) = w[3];
	W32(p, 0x10) = w[2];
	W32(p, 0x14) = w[1];
	if (iw_cmos_if(c, (const u32 *)p))
		return 0;
	memcpy(CP(CM_VICU(c) + 0x24), &w[1], 11 * 4);
	return 0;
}

static long isp_ioctl(struct file *f, unsigned int cmd, unsigned long a)
{
	void __user *arg = (void __user *)a;
	u32 w[0x4c / 4];
	int ret;

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
		if ((w[0] & 0xf) < 2)
			memcpy(CP(CM_CHN_ATTR(w[0] & 0xf)), w, 0x1c);
		return 0;
	case 0x490a:
		if (!C8(CM_CHNNUM))
			return chnnum_err(0x1f5);
		GET(0xc);
		return isp_wait_frame();
	case 0x490b:
		GET(4);
		if (w[0] < N_CHN)
			bpu_post_deal(w[0], C8(CM_MODE));
		return 0;
	case 0x490e:
		GET(0x4c);
		ret = vicu_chn_init((u8 *)w);
		bpu_chn_init((u8 *)w);
		return ret;
	case 0x4912:
	case 0x4913:
		return 0;
	case 0x4914:
		GET(8);
		return vicu_chn_enable(w[0], w[1]);
	case 0x4958:
		GET(4);
		return bpu_set_enable(w[0] >> 24, w[0] & 0xff);
	case 0x4959: /* the WDR mode: none on the XM535 */
		GET(4);
		return 0;
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
		if (w[0] >= N_CHN)
			return 0;
		C32(CM_IW_WAIT) = 1 << (w[0] & 0xff);
		C32(CM_VPSS_HOLD) = 1 << (C32(CM_GRP(w[0]) + 0x14) & 0xff);
		return 0;
	case 0x4961:
		GET(0x10);
		if ((int)w[0] > 2 || (int)w[0] < 0)
			return -1;
		C32(CM_GRP(w[0]) + 0x10) = w[1];
		C32(CM_GRP(w[0]) + 0x18) = w[3];
		C32(CM_GRP(w[0]) + 0x14) = w[2];
		return 0;
	case 0x4962:
		if (copy_from_user(w, arg, 0x38)) {
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
		if (copy_from_user(w, arg, 0x30)) {
			pr_err("IspKo:Err: %s %d ~~~~~~~~~~~~~~~~~~~~~~~~#\n",
			       "isp_ioctl", 0x2e7);
			return -1;
		}
		return isp_cmos_if(w);
	}
	if (!C8(CM_CHNNUM))
		return chnnum_err(0x2f0);
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

/* the 3D noise reduction's interrupt: isp.ko only acknowledges it */
static irqreturn_t nr3d_irq(int irq, void *dev)
{
	if (!isp_read_reg(0x3400f04c))
		return IRQ_NONE;
	isp_write_reg(0x3400f04c, 0);
	return IRQ_HANDLED;
}

static int isp_proc_show(struct seq_file *m, void *v)
{
	int n = 0, i;

	seq_printf(m, "[%s] BUILD TIME:[%s %s] [V%d.%d.%d.%d.%d.%d.%d]\n", "ISP",
		   "open_isp", "", C32(CM_VER), C32(CM_VER + 4),
		   C32(CM_VER + 0x10), C32(CM_VER + 0xc), C32(CM_VER + 0x14),
		   C32(CM_VER + 8), C32(CM_VER + 0x18));
	seq_printf(m, "Mode(Product):%d\n", C32(CM_PRODUCT));
	if (!C32(CM_RUN))
		return 0;
	seq_printf(m, "BpuMode:[%s]\n", C8(CM_MODE) ? "Offline" : "Online");
	seq_printf(m, "[%d.VICU_info]:\n", n++);
	seq_puts(m, "chn\tphy\tfps\tcount\tibFail\tibSucc\n");
	for (i = 0; i < N_CHN; i++)
		seq_printf(m, "%d\t%d\t%d.%d\t%d\t%d\t%d\n", i, C8(CM_VICU(i)),
			   C32(CM_VICU(i) + 0x58) / 10, C32(CM_VICU(i) + 0x58) % 10,
			   C32(CM_VICU(i) + 0x5c), C32(CM_VICU(i) + 0x78),
			   C32(CM_VICU(i) + 0x7c));
	seq_printf(m, "\n[%d.BPU_Info]:\n", n++);
	seq_puts(m, "chn\tfps\tcount\n");
	for (i = 0; i < N_CHN; i++)
		seq_printf(m, "%d\t%d.%d\t%d\n", i, C32(CM_BPUSTAT(i) + 8) / 10,
			   C32(CM_BPUSTAT(i) + 8) % 10, C32(CM_BPUSTAT(i) + 0xc));
	seq_printf(m, "all(chn):\n*\t%d.%d\t%d\n", C32(CM_BPUALL + 8) / 10,
		   C32(CM_BPUALL + 8) % 10, C32(CM_BPUALL + 0xc));
	seq_printf(m, "\n[%d.test_info]:\n", n++);
	seq_printf(m, "listNum: %d; err:%d\n", C8(CM_PEND_N), C16(CM_PEND_LOST));
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
	C32(CM_PRODUCT) = mode;
	C32(CM_VER + 0x14) = 3;
	C32(CM_VER + 0xc) = 0xffffffff;
	C32(CM_VER + 0x10) = 0xffffffff;
	C32(CM_VER + 8) = 0xffffffff;
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
	timer = ioremap(0x100d1000, 0x400);
	if (!timer) {
		pr_err("IspKo:Err: ioremap timer_regaddr failed!\n");
		ret = -ENOMEM;
		goto unmap;
	}
	C32(CM_RUN) = 0;
	proc = proc_create("umap/isp", 0444, NULL, &isp_proc_fops);
	isp_info_init();
	isp_pages = alloc_pages(GFP_KERNEL, ISP_PAGES_ORDER);
	if (!isp_pages) {
		pr_err("IspKo:Err: alloc_pages failed!\n");
		ret = -EIO;
		goto proc;
	}
	memset(page_address(isp_pages), 0, PAGE_SIZE << ISP_PAGES_ORDER);
	ret = request_irq(VICU_IRQ0, vicu_irq_0, 0, "vicu_bayer_irq0", NULL);
	if (ret)
		goto irq0;
	ret = request_irq(VICU_IRQ1, vicu_irq_1, 0, "vicu_bayer_irq1", NULL);
	if (ret)
		goto irq1;
	ret = request_irq(BPU_IRQ, bpu_irq, 0, "bpu_hvgen_irq", NULL);
	if (ret) {
		pr_err("IspKo:Err: Bpu_bayer_Irq_V failed!\n");
		goto irq2;
	}
	if (request_irq(NR3D_IRQ, nr3d_irq, 0, "nr3d_irq", NULL))
		pr_err("IspKo:Err: nr3d_irq failed!\n");
	else
		nr3d_irq_on = true;
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
	if (nr3d_irq_on)
		free_irq(NR3D_IRQ, NULL);
	free_irq(BPU_IRQ, NULL);
irq2:
	free_irq(VICU_IRQ1, NULL);
irq1:
	free_irq(VICU_IRQ0, NULL);
irq0:
	if (ret != -EPERM)
		pr_err("IspKo:Err: vicu_bayer_irq failed!\n");
	ret = -EIO;
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
	if (timer)
		iounmap(timer);
	return ret;
}

static void __exit isp_cleanup(void)
{
	int i;

	if (bpu_task)
		kthread_stop(bpu_task);
	misc_deregister(&isp_misc);
	free_irq(BPU_IRQ, NULL);
	free_irq(VICU_IRQ1, NULL);
	free_irq(VICU_IRQ0, NULL);
	if (timer_irq_on)
		free_irq(TIMER_IRQ, NULL);
	if (nr3d_irq_on)
		free_irq(NR3D_IRQ, NULL);
	tasklet_kill(&isp_irq0_tasklet);
	tasklet_kill(&isp_irq1_tasklet);
	tasklet_kill(&isp_bpu_tasklet);
	isp_exit();
	for (i = 0; i < 4; i++)
		iounmap(win[i]);
	vpss_isp_exit();
	iounmap(pll);
	iounmap(timer);
	if (proc)
		remove_proc_entry("umap/isp", NULL);
	__free_pages(isp_pages, ISP_PAGES_ORDER);
	pr_info("IspKo: rmmod ispDev OK!\n");
}

module_init(isp_init);
module_exit(isp_cleanup);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("XM535D1/D2 ISP (open isp.ko)");
