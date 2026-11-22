// SPDX-License-Identifier: GPL-2.0
/*
 * The XM530V200's ISP (/dev/isp): a drop-in for the vendor isp.ko. Up to
 * three channels on two VICUs; the BPU works online, or offline from ISP
 * buffers (ib.c) the VICU stores, its output in VB buffers for the VPSS.
 * The sensor library hands it parameter blocks (iw.c, vpss_isp.c) that
 * go to the registers once a frame.
 *
 *   0x400c4901 / 02  {addr, val, -}: a register written / read
 *   0x400c490c   {-, pages, -}: the pages mmap() may map
 *   0x4908       28 bytes: a channel's frame timing (channels 0, 1)
 *   0x4909       wait for the time-shared VICU's next sensor
 *   0x490a       wait for a frame (2 s): the channels with one
 *   0x490b       {chn}: the frame's statistics and VPSS work now
 *   0x490e       52 bytes: the channels (vicu_chn_init, bpu_chn_init)
 *   0x4912       28 bytes: the time-shared VICU's sensors
 *   0x4913       {chn, n}: frames the BPU's VB output is held back
 *   0x4932/33    autofocus windows in, statistics out
 *   0x4950       {chn, en | 2 to read, x, y, w, h}: the BPU's crop
 *   0x4958 / 59  {chn << 24 | v}: the BPU on / off, the WDR mode
 *   0x495a       36 bytes: dump a raw frame;  0x495c frame rates
 *   0x495d       40 bytes: the library's version in or out
 *   0x495e       {chn}: the VICU started;  0x495f / 0x4963 reset
 *   0x4960       {chn, on}: wait for the parameters before frames
 *   0x4961       16 bytes: the channel's bindings
 *   0x4962       56 bytes: w, h, mode | channels << 8 | ..., ISP buffers
 *   0x4980       {chn, x | y << 16, w | h << 16}: the VICU's crop
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
#define BPU_IRQ 63
#define ISP_PAGES_ORDER 3

static int mode;
module_param(mode, int, 0444);

u8 isp_comm[COMM_SIZE] __aligned(8);
spinlock_t awb_lock[N_CHN] = { __SPIN_LOCK_UNLOCKED(awb_lock[0]),
			       __SPIN_LOCK_UNLOCKED(awb_lock[1]),
			       __SPIN_LOCK_UNLOCKED(awb_lock[2]) };
DEFINE_SPINLOCK(dvp_lock);
DEFINE_SPINLOCK(bpu_irq_lock);
DEFINE_SPINLOCK(bpu_lock2);
struct mutex grp_lock[N_CHN] = { __MUTEX_INITIALIZER(grp_lock[0]),
				 __MUTEX_INITIALIZER(grp_lock[1]),
				 __MUTEX_INITIALIZER(grp_lock[2]) };
DEFINE_MUTEX(iwctrl_lock);
DEFINE_MUTEX(iwctrl_vpss_lock);
DEFINE_MUTEX(vpss_limitup_lock);
DECLARE_WAIT_QUEUE_HEAD(dump_wq);
DECLARE_WAIT_QUEUE_HEAD(vsync_wq);
DECLARE_WAIT_QUEUE_HEAD(dvp_wq);
DECLARE_WAIT_QUEUE_HEAD(bpu_thread_wq);
int isp_vpss_grp;

static const u32 win_start[4] = { 0x30000000, 0x32000000, 0x320f0000,
				  0x34000000 };
static const u32 win_end[4] = { 0x30083000, 0x32081000, 0x32101000,
				0x34090000 };
static void __iomem *win[4];
static void __iomem *pll;

static struct page *isp_pages;
static u8 isp_pages_max = 8;
static u32 awb_chn;
static atomic_t isp_users;
static struct task_struct *bpu_task;
static struct proc_dir_entry *proc;
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

int isp_users_open(void)
{
	return atomic_read(&isp_users);
}

static void isp_info_init(void)
{
	int i;

	for (i = 0; i < N_CHN; i++) {
		memset(CP(0x140 + i * 0xd0), 0, 0x78);
		memset(CP(CM_BPUSTAT + i * 0x58), 0, 0x58);
		memset(CP(CM_CHN_ATTR(i)), 0, 0x1c);
		memset(CP(CM_CUT(i)), 0, 0x14);
	}
	memset(CP(0x488), 0, 0x58);
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
	spin_lock_irqsave(&bpu_irq_lock, flags);
	C32(CM_RUN) = 0;
	spin_unlock_irqrestore(&bpu_irq_lock, flags);
	isp_reset();
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
	spin_lock_irqsave(&awb_lock[c], flags);
	memcpy(stat, CP(CM_AWB_STAT + c * sizeof(stat)), count);
	spin_unlock_irqrestore(&awb_lock[c], flags);
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
	long t;
	u32 ret;

	if ((int)C32(CM_VSYNC_CNT) < 1) {
		t = wait_event_interruptible_timeout(
			vsync_wq, (int)C32(CM_VSYNC_CNT) > 0, 200);
		if (!t)
			return -ERESTARTSYS;
	}
	ret = C32(CM_VSYNC_CNT) & 0xff;
	C32(CM_VSYNC_CNT) = 0;
	return (C32(CM_DVP_STATE) & 0xff) << 8 | ret;
}

static long isp_wait_dvpsel(void)
{
	long t;

	if ((int)C32(CM_DVP_STATE) >= 0) {
		t = wait_event_interruptible_timeout(
			dvp_wq, (int)C32(CM_DVP_STATE) < 0, 200);
		if (!t) {
			pr_err("IspKo:Err: %s %d: 0x%x\n", "isp_get_dvpsel",
			       0xef, C32(CM_DVP_STATE));
			return -1;
		}
	}
	C32(CM_DVP_STATE) &= 0x7fffffff;
	return (C32(CM_DVP_STATE) & 0xff) << 8;
}

static long isp_dump(void __user *arg, u32 *w)
{
	unsigned long flags;
	u32 chn = w[0] & 0xff;
	long ret = -1, t;

	if (chn < N_CHN) {
		if (C8(CM_DUMP) == 0) {
			w[0] = (w[0] & 0xffff0000) | chn;
			spin_lock_irqsave(&awb_lock[chn], flags);
			C8(CM_DUMP) = 1;
			C8(CM_DUMP_CHN) = chn;
			C16(CM_DUMP_W) = w[0] >> 16;
			C16(CM_DUMP_H) = w[1];
			C32(CM_DUMP_ADDR) = w[3];
			C32(CM_DUMP_SIZE) = w[5];
			spin_unlock_irqrestore(&awb_lock[chn], flags);
			t = 1;
			if (!(C8(CM_DUMP) & 0x30))
				t = wait_event_interruptible_timeout(
					dump_wq, C8(CM_DUMP) & 0x30, 50);
			if (!t || (C8(CM_DUMP) & 0x20)) {
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
			if (fps_last[i] == C32(0x38c + i * 0x58)) {
				if (fps_same[i] < 0xf0 && ++fps_same[i] < 3)
					a = C32(0x388 + i * 0x58);
			} else {
				fps_last[i] = C32(0x38c + i * 0x58);
				fps_same[i] = 0;
				a = C32(0x388 + i * 0x58);
			}
			b = C16(CM_GRP(i) + 0x58);
			c = C16(CM_GRP(i) + 0x5c);
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
		w[1] = C32(0x61b8);
		w[2] = C32(0x61bc);
		w[3] = C32(0x61c0);
		w[4] = C32(0x61c4);
		w[5] = C32(0x61c8);
		w[6] = C32(0x61d0);
		w[7] = C32(0x61d4);
		w[8] = C32(0x61d8);
		w[9] = C32(0x61cc);
		if (copy_to_user(arg, w, 0x28)) {
			pr_err("IspKo:Err: CMD_ISP_VERSION copy_to_user error.\n");
			return -1;
		}
		return 0;
	}
	C32(0x61d0) = w[6];
	C32(0x61b8) = w[1];
	C32(0x61bc) = w[2];
	C32(0x61c4) = w[4];
	C32(0x61c8) = w[5];
	C32(0x61c0) = w[3];
	C32(0x61d4) = w[7];
	C32(0x61d8) = w[8];
	C32(0x61dc) = w[9]; /* (read back from 0x61cc, as isp.ko) */
	return 0;
}

static long isp_comm_init(const u32 *w)
{
	u32 pools[7] = { 0 }, v;
	int i, ret;

	memcpy(CP(CM_INIT), w, 0x38);
	pools[0] = C32(0x7c) > 2 ? 3 : C32(0x7c);
	for (i = 0; i < pools[0]; i++) {
		pools[1 + i * 2] = C32(0x80 + i * 8);
		pools[2 + i * 2] = C32(0x84 + i * 8);
	}
	ret = ib_init(pools, 0, 0, C8(CM_OPT) & 3);
	isp_reset();
	C32(CM_IW_WAIT) = 0;
	C32(CM_BPU_GO) = 0;
	C32(CM_VPSS_HOLD) = 0;
	C32(CM_BPU_RUN) = 0;
	C32(CM_VSYNC_CNT) = 0;
	C32(CM_DVP_STATE) = 0;
	mutex_lock(&vpss_limitup_lock);
	C8(CM_FLAGS) |= 0x20;
	mutex_unlock(&vpss_limitup_lock);
	C32(CM_REPAIR) = 0xffffffff;
	bpu_init();
	isp_vpss_grp = 0;
	C32(CM_RUN) = 1;
	v = (C8(CM_OPT) & 3) < 2 ? 0 : 3;
	for (i = 0; i < N_CHN; i++)
		C32(CM_VB_HOLD(i)) = v;
	return ret;
}

static long isp_ioctl(struct file *f, unsigned int cmd, unsigned long a)
{
	void __user *arg = (void __user *)a;
	unsigned long flags;
	u32 w[0x3c / 4];
	u32 chn;
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
	case 0x4909:
		if (!C8(CM_CHNNUM))
			return chnnum_err(0x1d9);
		GET(4);
		return isp_wait_dvpsel();
	case 0x490a:
		if (!C8(CM_CHNNUM))
			return chnnum_err(0x1e5);
		GET(0xc);
		return isp_wait_frame();
	case 0x490b:
		GET(4);
		if (w[0] < N_CHN) {
			chn = w[0];
			bpu_post_deal(chn, C8(CM_MODE), C32(CM_GRP(chn) + 0x38));
			vpss_isp_post_deal(chn, C32(0xb8 + chn * 0x28) & 0xffff,
					   C32(0xb8 + chn * 0x28) >> 16);
		}
		return 0;
	case 0x490e:
		GET(0x34);
		ret = vicu_chn_init((u8 *)w);
		bpu_chn_init((u8 *)w);
		return ret;
	case 0x4912:
		GET(0x1c);
		spin_lock_irqsave(&dvp_lock, flags);
		memcpy(CP(CM_DVPSEL), w, 0x1c);
		spin_unlock_irqrestore(&dvp_lock, flags);
		return 0;
	case 0x4913:
		GET(8);
		if (w[0] < N_CHN)
			C32(CM_VB_HOLD(w[0])) = w[1];
		return 0;
	case 0x4932:
		return bpu_af_attr_set(arg);
	case 0x4933:
		return bpu_af_stat_get(arg);
	case 0x4950:
		GET(0x18);
		if (w[0] >= N_CHN)
			return -1;
		if (w[1] < 2)
			return bpu_set_cut(w[0], w[1], w[2], w[3], w[4], w[5]);
		bpu_get_cut(w[0], &w[1], &w[2], &w[3], &w[4], &w[5]);
		if (copy_to_user(arg, w, 0x18))
			return -1;
		return 0;
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
		if (w[0] >= N_CHN)
			return 0;
		C32(CM_IW_WAIT) = 1 << (w[0] & 0xff);
		C32(CM_VPSS_HOLD) = 1 << (C32(CM_GRP(w[0]) + 0x14) & 0xff);
		return 0;
	case 0x4961:
		GET(0x10);
		if ((int)w[0] > 2 || (int)w[0] < 0)
			return -1;
		C32(CM_GRP(w[0]) + 0x18) = w[3];
		C32(CM_GRP(w[0]) + 0x10) = w[1];
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
		GET(0xc);
		if (w[0] > 2)
			return -1;
		chn = w[0];
		C32(CM_CHN + chn * 0x28) = w[1];
		C32(CM_CHN + 4 + chn * 0x28) = w[2];
		C32(0x13c + chn * 0xd0) = w[2] >> 16;
		C32(0x138 + chn * 0xd0) = w[2] & 0xffff;
		C32(0x130 + chn * 0xd0) = w[1] & 0xffff;
		C32(0x134 + chn * 0xd0) = w[1] >> 16;
		C32(0x12c + chn * 0xd0) = 1;
		return 0;
	}
	if (!C8(CM_CHNNUM))
		return chnnum_err(0x2d8);
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

static int isp_proc_show(struct seq_file *m, void *v)
{
	int n = 0, i;

	seq_printf(m, "[%s] BUILD TIME:[%s %s] [V%d.%d.%d.%d.%d.%d.%d]\n", "ISP",
		   "open_isp", "", C32(0x61b8), C32(0x61bc), C32(0x61c8),
		   C32(0x61c4), C32(0x61cc), C32(0x61c0), C32(0x61d0));
	seq_printf(m, "Mode(Product):%d\n", C32(CM_PRODUCT));
	if (!C32(CM_RUN))
		return 0;
	seq_printf(m, "BpuMode:[%s]\n", C8(CM_MODE) == 0 ? "Online" :
		   C8(CM_MODE) == 1 ? "Offline" : "HDR");
	vpss_isp_proc_show(m, &n);
	seq_printf(m, "[%d.VICU_info]:\n", n++);
	seq_puts(m, "chn\tfps\tcount\tib_fail\tib_success\n");
	for (i = 0; i < N_CHN; i++)
		seq_printf(m, "%d\t%d.%d\t%d\t%d\t%d\n", i,
			   C32(0x148 + i * 0xd0) / 10, C32(0x148 + i * 0xd0) % 10,
			   C32(0x14c + i * 0xd0), C32(0x168 + i * 0xd0),
			   C32(0x16c + i * 0xd0));
	seq_printf(m, "\n[%d.BPU_Info]:\n", n++);
	seq_puts(m, "chn\tfps\tcount\n");
	for (i = 0; i < N_CHN; i++)
		seq_printf(m, "%d\t%d.%d\t%d\n", i, C32(0x388 + i * 0x58) / 10,
			   C32(0x388 + i * 0x58) % 10, C32(0x38c + i * 0x58));
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
	C32(0x61cc) = 4;
	C32(0x61c4) = 0xffffffff;
	C32(0x61c8) = 0xffffffff;
	C32(0x61c0) = 0xffffffff;
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
	C8(CM_FLAGS) = 1;
	C32(CM_RUN) = 0;
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
	isp_exit();
	for (i = 0; i < 4; i++)
		iounmap(win[i]);
	vpss_isp_exit();
	iounmap(pll);
	if (proc)
		remove_proc_entry("umap/isp", NULL);
	__free_pages(isp_pages, ISP_PAGES_ORDER);
	pr_info("IspKo: rmmod ispDev OK!\n");
}

module_init(isp_init);
module_exit(isp_cleanup);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("XM530V200 ISP (open isp.ko)");
