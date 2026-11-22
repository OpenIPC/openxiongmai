// SPDX-License-Identifier: GPL-2.0
/*
 * The XM232's ISP (/dev/isp): a drop-in for its vendor isp.ko
 * ("XM210V200_ISP"). One VICU, its input switched between up to three
 * sensors: the mux timer (IRQ 90) marks each sensor's frame start and
 * vi_mu.ko picks the next (isp_set_next_ch), which writes the BPU and VPSS
 * side's registers from the parameter blocks the sensor library hands in
 * (iw.c, vpss_isp.c). Online and offline BPU modes are there as isp.ko has
 * them.
 *
 *   0x400c4901 / 02 / 03  {addr, val, -}: a register written / read, in
 *                the ISP window, the pads or the system controller
 *   0x400c490c   {-, pages, -}: the pages mmap() may map
 *   0x4900       {chn, n}: a debug print
 *   0x4908       28 bytes: a MIPI receiver's info (receivers 0, 1)
 *   0x4909       {flags | vicu << 16 | on}: the VICU's input on (bit 31 reads)
 *   0x490a       {-, -, chn}: wait for the channel's frame (2 s)
 *   0x490e       52 bytes: a channel (vicu_chn_init, bpu_chn_init)
 *   0x4912       28 bytes: the sensor select and its pads
 *   0x4915       44 bytes: frame counts before a power-down; 0x4916 IR cut
 *   0x4959       {chn}: the channel running
 *   0x495a / 5b  36 bytes: dump a raw frame / stop
 *   0x495d       40 bytes: the library's version in or out
 *   0x495e       {cmd}: suspend / resume / stop a sensor (isp_suspend)
 *   0x495f / 63  stop / reset
 *   0x4960       {chn, on}: hold the parameters after the first frame
 *   0x4961       16 bytes: the channel's bindings (its VPSS group)
 *   0x4962       44 bytes: modes, channels and the ISP buffer pools
 *   0x4982 / 83  wait for a sensor power request / acknowledge it
 *   0x4964..0x4980, 0x49d2..0x49ec  the parameter blocks
 *
 * mode= as isp.ko's: 1 for the offline VICU interrupts, 2 without the BPU
 * interrupt.
 */

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/gfp.h>
#include <linux/io.h>
#include <linux/kthread.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#include "isp.h"
#include "suspend_regs.h"

#define VICU_IRQ 64
#define VICU_IRQ0 85
#define VICU_IRQ1 86
#define BPU_IRQ 88
#define MUX_IRQ 90
#define ISP_PAGES_ORDER 4

#define ISP_WIN 0x30000000
#define ISP_WIN_SIZE 0x4400000
#define GPIO_WIN 0x10020000
#define SYS_WIN 0x20000000
#define DDR_WIN 0x20050000
#define IRCUT_WIN 0x100e2000

static int mode;
module_param(mode, int, 0444);

u8 isp_comm[COMM_SIZE] __aligned(8);
void __iomem *isp_base;
static void __iomem *gpio_base, *pll_base, *ddr_base, *ircut_base;
struct mutex grp_lock[N_CHN];
struct mutex iwctrl_lock;
struct mutex iwctrl_vpss_lock;
struct mutex sns_pw_lock;
wait_queue_head_t dump_wq, bpu_thread_wq, sns_pw_wq;
wait_queue_head_t vicu_wq[N_CHN], mux_wq[N_CHN];
DECLARE_TASKLET(isp_bpu_tasklet, bpu_offline_deal_in, 0xff);
struct isp_ops isp_ops;
void __iomem *stat_va[2][N_CHN][2];
u32 *isp_regs_buf;
atomic_t isp_users;

static struct page *isp_pages;
static u8 page_num = 0x10;
static u32 awb_chn;
static u32 mipi_info[2][7];
static u8 frame_sync;
static u32 frame_cnt_now[N_CHN], frame_cnt_attr[N_CHN];
static u64 ircut_pts;
static u8 irq_mark;
static struct task_struct *bpu_task;

/* a register in one of the three windows 0x400c4901..03 reach */
static void __iomem *win_reg(u32 addr)
{
	if (addr - ISP_WIN < ISP_WIN_SIZE)
		return isp_base + (addr - ISP_WIN);
	if (addr - GPIO_WIN < 0x1000)
		return gpio_base + (addr - GPIO_WIN);
	if (addr - SYS_WIN < 0x1000)
		return pll_base + (addr - SYS_WIN);
	return NULL;
}

static void win_write(u32 addr, u32 val)
{
	void __iomem *r = win_reg(addr);

	if (!r) {
		pr_err("The WriteReg Addr 0x%x is outside, cmd error!\n", addr);
		return;
	}
	dsb();
	writel_relaxed(val, r);
}

static u32 win_read(u32 addr)
{
	void __iomem *r = win_reg(addr);
	u32 v;

	if (!r) {
		pr_err("The ReadReg Addr 0x%x is outside, cmd error!\n", addr);
		return 0;
	}
	v = readl_relaxed(r);
	dsb();
	return v;
}

static int isp_stop_clear(void)
{
	wr(0x214, 1);
	wr(0x1214, 1);
	wr(0x2038, 1);
	wr(0x3038, 1);
	return 0;
}

static int isp_open(struct inode *inode, struct file *file)
{
	atomic_inc(&isp_users);
	return 0;
}

static int isp_release(struct inode *inode, struct file *file)
{
	atomic_dec(&isp_users);
	return 0;
}

static int isp_pm_set_mode(int m)
{
	if (isp_ops.pm_set_mode)
		return isp_ops.pm_set_mode(m);
	return 0;
}

/* vi_mu.ko's callbacks: the power mode, then three isp.ko calls offline */
int isp_register_ops(struct isp_ops ops)
{
	isp_ops = ops;
	pr_info("IK: %s\n", "isp_register_ops");
	return 0;
}
EXPORT_SYMBOL(isp_register_ops);

/* a sensor power request for user space (0x4982); 0xff: all sensors */
static int isp_set_sns_pw_ctrl(u32 sns, int on)
{
	u32 v, v2;
	u8 m = C8(CM_BPU_MODE);

	if (m == 1 || (m == 3 && (s8)C8(CM_OPT + 1) < 0)) {
		if (sns != 0xff && sns != C8(CM_FCNT_SNS))
			return 0;
		v = 0xf;
	} else if (m != 3) {
		v = 1;
	} else if (sns == 0xff) {
		v = 0xf;
	} else {
		v = BIT(sns);
	}
	mutex_lock(&sns_pw_lock);
	v2 = 0;
	if (on) {
		v2 = v;
		if (sns == 0xff)
			C32(CM_PW_MASK) = 0;
	}
	C32(CM_PW_VAL) = v2 | v << 8;
	C32(CM_PW_EV) = 1;
	mutex_unlock(&sns_pw_lock);
	wake_up(&sns_pw_wq);
	pr_info("IK: PW_0x%x_0x%x\n", v, v2);
	return 0;
}

/* mux_timer_irq: a sensor's frame start */
static irqreturn_t mux_timer(int irq, void *dev)
{
	u32 i = 0, next, ch, bit;

	for (;;) {
		if (!rd(0x2108 + i * 0x1000)) {
			next = i + 1;
		} else {
			wr(0x2100 + i * 0x1000, 0);
			wr(0x2104 + i * 0x1000, 0);
			ch = 1;
			bit = BIT(i);
			if (!(bit & C32(CM_RUN))) {
				next = i + 1;
			} else {
				if (C8(CM_CHNNUM) < 3) {
					next = i + 1;
					ch = i;
				} else if (i == 0) {
					if (C8(CM_FCNT_SNS) < 2) {
						i = C8(CM_FCNT_SNS);
						bit = BIT(i);
						next = i + 1;
						ch = i;
					} else {
						next = 3;
						ch = 2;
						bit = 4;
					}
				} else {
					next = 2;
					bit = 2;
				}
				C32(CM_MUX(ch)) = bit;
				wake_up(&mux_wq[ch]);
			}
		}
		i = 1;
		if (next != 1)
			return IRQ_HANDLED;
	}
}

/*
 * vi_mu.ko: the VICU's next sensor. The finished channel's statistics are
 * taken, the next one's registers written, and the frame before it
 * signalled (0x490a).
 */
int isp_set_next_ch(u32 ch, u32 state, u32 timeout, u32 rsvd)
{
	s32 prev = CS32(CM_CH_NOW), now;
	u32 g, v40, v50, v;
	int ret = 0;

	if (C8(CM_BPU_MODE) != 1 || ch > 2 || !(C32(CM_RUN) & BIT(ch)))
		return -1;
	C32(CM_CH_PREV) = C32(CM_CH_NOW);
	C32(CM_CH_NOW) = C32(CM_CH_CUR);
	C32(CM_CH_CUR) = ch;
	if (prev >= 0)
		ret = bpu_post_deal(prev) >> 31;
	if (bpu_deal(ch) < 0) {
		ret = -1;
		pr_err("IK:Err: bpu_deal err!\n");
	}
	now = CS32(CM_CH_NOW);
	if (now >= 0) {
		C32(CM_VSYNC(now)) |= BIT(now);
		wake_up(&vicu_wq[now]);
		v = C32(CM_VICU(now) + 0x48) + 1;
		C32(CM_VICU(now) + 0x48) = v > 0xffff ? 0 : v;
	}
	if (ch != 1 && C8(CM_CHNNUM) > 2) {
		g = (C32(CM_SNS_GPIO) & 0xf) << 8;
		if (C32(CM_SNS_GPIO) & 0x10000) {
			v40 = g | 0x50;
			v50 = g | 0x40;
		} else {
			v40 = g | 0x40;
			v50 = g | 0x50;
		}
		wr(0x10008, ch ? v40 : v50);
		return ret;
	}
	wr(0x10008, ch);
	return ret;
}
EXPORT_SYMBOL(isp_set_next_ch);

static void isp_frame_cnt_sync(void)
{
	mutex_lock(&sns_pw_lock);
	if (!frame_sync) {
		memcpy(frame_cnt_now, frame_cnt_attr, sizeof(frame_cnt_now));
		frame_sync = 1;
	}
	mutex_unlock(&sns_pw_lock);
}

/* a frame done: the sensor's count before its power-down (0x4915) */
int isp_deal_frame_cnt(u32 ch)
{
	u8 dual = C8(CM_FCNT_DUAL);
	u32 n;

	if (ch > 2)
		return -1;
	if (ch == C8(CM_FCNT_CH))
		isp_frame_cnt_sync();
	mutex_lock(&sns_pw_lock);
	n = frame_cnt_now[ch];
	if (n - 1 < 0xfffe) {
		frame_cnt_now[ch] = n - 1;
		mutex_unlock(&sns_pw_lock);
		if (n == 1) {
			if (dual != BIT(ch))
				isp_suspend(ch << 24 | 0x31);
			isp_set_sns_pw_ctrl(ch, 0);
		}
	} else {
		mutex_unlock(&sns_pw_lock);
	}
	return 0;
}

/* vi_mu.ko: the channel's frame done */
int isp_set_complete(u32 ch)
{
	if (C8(CM_BPU_MODE) == 1 && (C32(CM_RUN) & BIT(ch)))
		isp_deal_frame_cnt(C32(CM_CH_NOW));
	return 0;
}
EXPORT_SYMBOL(isp_set_complete);

static int isp_reset(void)
{
	int c;

	vicu_reset();
	bpu_reset();
	vpss_isp_reset();
	vicu_init_queue();
	bpu_init_queue();
	init_waitqueue_head(&sns_pw_wq);
	mutex_init(&sns_pw_lock);
	C32(CM_BPU_OFF) = 0;
	C32(CM_BPU_RUN) = 0;
	C32(CM_VPSS_OFF) = 0;
	C32(CM_BPU_GO) = 0;
	C32(CM_STATE) = 0;
	C32(CM_PW_MASK) = 0;
	C32(CM_PW_VAL) = 0;
	C32(CM_PW_EV) = 0;
	for (c = 0; c < N_CHN; c++) {
		C32(CM_VSYNC(c)) = 0;
		C32(CM_MUX(c)) = 0;
		init_waitqueue_head(&mux_wq[c]);
	}
	return 0;
}

static int isp_exit(void)
{
	unsigned long flags;

	C32(CM_BPU_OFF) = 0xff;
	C32(CM_VPSS_OFF) = 0xff;
	ISP_LOCK(flags);
	C32(CM_RUN) = 0;
	ISP_UNLOCK(flags);
	vicu_reset();
	bpu_reset();
	vpss_isp_reset();
	return 0;
}

static void suspend_lock(int lock)
{
	int c;

	for (c = 0; c < N_CHN; c++) {
		if (lock)
			mutex_lock(&grp_lock[c]);
		else
			mutex_unlock(&grp_lock[c]);
		vpss_isp_mutex_lock(c, lock);
	}
}

/* sensor c's input stopped; with wait, until its frame start shows */
static int isp_stop_sensor(u32 cmd)
{
	u32 c = cmd >> 24, d = c ? 0x1000 : 0;
	int i;
	long t;
	u64 pts;

	wr(0x2100 + d, 0);
	wr(0x2104 + d, 0);
	C32(CM_MUX(c)) = 0;
	wr(0x202c + d, 0x5c);
	wr(0x2100 + d, 1);
	wr(0x2024 + d, 0x1b);
	wr(0x2028 + d, 1);
	if (!(cmd & 0x20))
		return 0;
	if (c == C8(CM_FCNT_SNS) && C8(CM_BPU_MODE) == 1)
		isp_pm_set_mode(0);
	if ((rd(0x2028 + d) & 3) != 2) {
		schedule_timeout_interruptible(msecs_to_jiffies(10));
		for (i = 0x31; (rd(0x2028 + d) & 3) != 2; ) {
			schedule_timeout_interruptible(msecs_to_jiffies(10));
			if (!--i) {
				pr_err("IK:Err: ch%d isp_stop timeout_0! %d\n", c,
				       0x33);
				return -1;
			}
		}
	}
	clk_get_pts(&pts);
	if (CS32(CM_MUX(c)) < 1) {
		t = wait_event_interruptible_timeout(mux_wq[c],
						     CS32(CM_MUX(c)) > 0, 100);
		if (t < 1) {
			pr_err("IK:Err: isp_stop timeout_1! 0x%x\n", C32(CM_MUX(c)));
			C32(CM_MUX(c)) = 0;
			return -1;
		}
	}
	C32(CM_MUX(c)) = 0;
	return 0;
}

/* the registers before a power-down (cmd bit 0) */
static int isp_save(void)
{
	u32 mask = C8(CM_CHN_MASK), dual = C8(CM_FCNT_DUAL), *b, v;
	unsigned long flags;
	int i;

	for (i = 0x3c; i; i--) {
		if ((C32(CM_PW_MASK) & mask & ~dual) == (mask & ~dual))
			break;
		schedule_timeout_interruptible(msecs_to_jiffies(20));
	}
	if (!i)
		pr_err("IK:Err: %s line%d timerout!    0x%x\n", "isp_suspend",
		       0x8cb, C32(CM_PW_MASK));
	if (C8(CM_BPU_MODE) == 3) {
		for (i = 1; C8(CM_PEND_N) || C32(CM_BPU_DEALING); i++) {
			schedule_timeout_interruptible(msecs_to_jiffies(20));
			if (i == 0x3c) {
				pr_err("IK:Err: %s %d timerout!\n", "isp_suspend",
				       0x8d8);
				break;
			}
			pr_info("IK: wait bpu_dealing %d\n", i);
		}
	}
	C32(CM_STATE) = 0x11;
	if (!isp_regs_buf) {
		isp_regs_buf = kmalloc(0x1300, GFP_KERNEL);
		if (!isp_regs_buf) {
			pr_err("IK:Err: isp_suspend kmalloc failed\n");
			return -1;
		}
	}
	b = isp_regs_buf;
	suspend_lock(1);
	local_irq_save(flags);
	/* the frame controls, last first, each saved then cleared */
	for (i = ARRAY_SIZE(suspend_tail) - 1; i >= 0; i--) {
		b[924 + i] = win_read(suspend_tail[i]);
		win_write(suspend_tail[i], suspend_tail[i] == 0x30042044);
	}
	for (i = 0; i < ARRAY_SIZE(suspend_regs); i++)
		b[i] = win_read(suspend_regs[i]);
	if (rd(0x46000))
		for (i = 0; i < ARRAY_SIZE(suspend_lsc); i++)
			b[943 + i] = win_read(suspend_lsc[i]);
	local_irq_restore(flags);
	suspend_lock(0);
	v = 0;
	pr_info("IK: S1S OR!\n");
	return v;
}

/* the PLL at 0x20000000: off, its value, on, then its lock (bit 31) */
static void isp_restore_pll(u32 val)
{
	u32 ctl = readl_relaxed(pll_base + 0x10);
	int i;

	dsb();
	dsb();
	writel_relaxed(ctl & 0x7effffff, pll_base + 0x10);
	udelay(250);
	dsb();
	writel_relaxed((ctl & 0x7effffff) | 0x1000000, pll_base + 0x10);
	win_write(SYS_WIN, val);
	ctl = readl_relaxed(pll_base + 0x10);
	dsb();
	for (i = 499; i; i--) {
		udelay(250);
		if ((s32)ctl < 0)
			break;
		ctl = readl_relaxed(pll_base + 0x10);
		dsb();
	}
	if (!i)
		pr_err("IK:Err: PLL failed!\n");
	dsb();
	writel_relaxed(1, ddr_base + 0x490);
}

/* the registers back after the sensors power up */
static int isp_restore(void)
{
	u32 *b = isp_regs_buf, a;
	unsigned long flags;
	int i;

	if (!b) {
		pr_err("IK:Err: p_regs_buf not init!\n");
		return -1;
	}
	isp_stop_clear();
	suspend_lock(1);
	local_irq_save(flags);
	dsb();
	writel_relaxed(1, pll_base + 0x11c);
	wr(0x2038, 1);
	wr(0x3038, 1);
	for (i = 0; i < ARRAY_SIZE(suspend_regs); i++) {
		a = suspend_regs[i];
		if (a == 0x20000010) {
			dsb();
			writel_relaxed(1, pll_base);
			win_write(a, b[i] & 0x7fffffff);
		} else if (a == SYS_WIN) {
			isp_restore_pll(b[i]);
		} else if (a == 0x3001000c && C8(CM_BPU_MODE) == 1) {
			win_write(a, 3);
		} else {
			win_write(a, b[i]);
		}
	}
	if (rd(0x46000))
		for (i = 0; i < ARRAY_SIZE(suspend_lsc); i++)
			win_write(suspend_lsc[i], b[943 + i]);
	if (rd(0x6c01c) && isp_ldc_buf && C8(CM_BPU_MODE) != 3) {
		wr(0x6c01c, 1);
		wr(0x6c020, 0);
		for (i = 0; i < 0x400; i++)
			wr(0x6d000 + i * 4, ((u32 *)isp_ldc_buf)[i]);
		wr(0x6c020, 1);
		wr(0x6c01c, 1);
	}
	wr(0x264, 1);
	wr(0x1264, 1);
	for (i = 0; i < ARRAY_SIZE(suspend_tail); i++)
		win_write(suspend_tail[i], b[924 + i]);
	local_irq_restore(flags);
	suspend_lock(0);
	C32(CM_STATE) = 0x10;
	CS32(CM_CH_CUR) = -1;
	if (C8(CM_BPU_MODE) == 1)
		isp_pm_set_mode(1);
	pr_info("IK: S1R OR!\n");
	return 0;
}

/*
 * 0x495e: bits 4-7 clear: bit 0 save the registers before the sensors
 * power down, else restore them; bits 4-7 set: stop sensor cmd >> 24's
 * input (bit 5: and wait for it).
 */
int isp_suspend(u32 cmd)
{
	if (!(cmd & 0xf0))
		return cmd & 1 ? isp_save() : isp_restore();
	if (cmd & 0x40) {
		pr_info("IK: %s %d ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~\n",
			"isp_vi_pm_set_mode", 0x33e);
		return 0;
	}
	return isp_stop_sensor(cmd);
}

static ssize_t isp_awb_read(struct file *f, char __user *buf, size_t count,
			    loff_t *pos)
{
	size_t n = count;

	if (count < 0xc00)
		pr_err("IK:Err: isp_awb_read count:%d size:%d\n", count, 0xc00);
	else
		n = 0xc00;
	if (awb_chn >= N_CHN)
		return -1;
	if (copy_to_user(buf, CP(CM_AWB_STAT(awb_chn)), n))
		return -1;
	return n;
}

static ssize_t isp_awb_write(struct file *f, const char __user *buf,
			     size_t count, loff_t *pos)
{
	size_t n = min_t(size_t, count, 4);

	awb_chn = 0;
	if (copy_from_user(&awb_chn, buf, n)) {
		awb_chn = 0;
		return -1;
	}
	return n;
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

/* the 16 shared pages, as many of them as 0x400c490c allows */
static int isp_mmap(struct file *file, struct vm_area_struct *vma)
{
	unsigned long size = vma->vm_end - vma->vm_start;

	if (!isp_pages || page_num > 0x10 ||
	    size > (PAGE_SIZE << ISP_PAGES_ORDER)) {
		pr_err("IK:Err: isp_mmap failed! [need:%d get:%d]\n", page_num,
		       0x10);
		return -1;
	}
	if (remap_pfn_range(vma, vma->vm_start, page_to_pfn(isp_pages), size,
			    vma->vm_page_prot)) {
		pr_err("IK: [IspDrv] err: remap_pfn_range failed!\n");
		return -1;
	}
	vma->vm_ops = &isp_vm_ops;
	return 0;
}

#define ISP_GET(n)                                                        \
	do {                                                              \
		if (copy_from_user(w, arg, (n)))                          \
			return -1;                                        \
	} while (0)

static long isp_ioctl_sys(unsigned int cmd, void __user *arg, u32 *w,
			  bool *done)
{
	unsigned long flags;
	u32 c, hi, m, i;
	long t;
	int ret;

	*done = true;
	switch (cmd) {
	case 0x495d:
		ISP_GET(0x28);
		if (w[0]) {
			for (i = 0; i < 5; i++)
				w[1 + i] = C32(CM_VER + i * 4);
			for (i = 0; i < 3; i++)
				w[6 + i] = C32(CM_VER + 0x18 + i * 4);
			w[9] = C32(CM_VER + 0x14);
			if (copy_to_user(arg, w, 0x28)) {
				pr_err("IK:Err: CMD_ISP_VERSION copy_to_user error.\n");
				return -1;
			}
			return 0;
		}
		for (i = 0; i < 5; i++)
			C32(CM_VER + i * 4) = w[1 + i];
		for (i = 0; i < 3; i++)
			C32(CM_VER + 0x18 + i * 4) = w[6 + i];
		C32(CM_VER + 0x24) = w[9];
		return 0;
	case 0x4963:
		isp_reset();
		C32(CM_RUN) = 0;
		return 0;
	case 0x4960:
		ISP_GET(8);
		if (!w[1]) {
			C32(CM_BPU_OFF) = 0;
			C32(CM_VPSS_OFF) = 0;
		} else if (w[0] < N_CHN) {
			C32(CM_BPU_OFF) = BIT(w[0]);
			C32(CM_VPSS_OFF) = BIT(C32(CM_GRP(w[0]) + GRP_VPSS) & 0x1f);
		}
		return 0;
	case 0x495e:
		ISP_GET(8);
		return isp_suspend(w[0]);
	case 0x495f:
		isp_exit();
		return 0;
	case 0x4961:
		ISP_GET(0x10);
		if ((s32)w[0] > 2 || (s32)w[0] < 0)
			return -1;
		C32(CM_GRP(w[0]) + GRP_BIND) = w[1];
		C32(CM_GRP(w[0]) + GRP_BIND + 4) = w[2];
		C32(CM_GRP(w[0]) + GRP_BIND + 8) = w[3];
		return 0;
	case 0x4962: {
		u32 pools[1 + 64] = { 0 };
		u8 n;

		if (copy_from_user(w, arg, 0x2c)) {
			pr_err("IK:Err: Isp %d: copy_from_user error.\n", 0x4962);
			return -1;
		}
		memcpy(CP(CM_INIT), w, 0x2c);
		pools[0] = min_t(u32, w[4], 0x20);
		for (i = 0; i < pools[0]; i++) {
			pools[1 + i * 2] = C32(CM_POOLS + 4 + i * 8);
			pools[2 + i * 2] = C32(CM_POOLS + 8 + i * 8);
		}
		ret = ib_init(pools, 0, 0);
		isp_reset();
		bpu_init(CP(CM_INIT));
		n = C8(CM_CHNNUM);
		C8(CM_FCNT_CH) = 1;
		C8(CM_FCNT_SNS) = 0;
		C8(CM_FCNT_DUAL) = n > 2;
		return ret;
	}
	case 0x400c4901:
		ISP_GET(0xc);
		win_write(w[0], w[1]);
		return 0;
	case 0x400c4902:
	case 0x400c4903:
		if (copy_from_user(w, arg, 0xc)) {
			pr_err("IK:Err: Isp %d: copy_from_user error.\n", cmd);
			return -1;
		}
		w[1] = win_read(w[0]);
		if (copy_to_user(arg, w, 0xc)) {
			pr_err("IK:Err: IspRegRead : copy_to_user error.\n");
			return -1;
		}
		return 0;
	case 0x4982:
		if (CS32(CM_PW_EV) < 1) {
			t = wait_event_interruptible_timeout(sns_pw_wq,
							     CS32(CM_PW_EV) > 0,
							     200);
			if (t < 1)
				return -ERESTARTSYS;
			if (!C32(CM_PW_EV))
				return -1;
		}
		mutex_lock(&sns_pw_lock);
		ret = C32(CM_PW_VAL);
		C32(CM_PW_EV) = 0;
		mutex_unlock(&sns_pw_lock);
		return ret;
	case 0x4983:
		ISP_GET(4);
		hi = (w[0] & 0xffff) >> 8;
		if (hi && (w[0] & 1)) {
			if (C8(CM_BPU_MODE) == 1)
				isp_pm_set_mode(2);
			return 0;
		}
		m = C32(CM_PW_MASK);
		for (i = 0; i < 3; i++)
			if ((BIT(i) & hi) && !(w[0] & 0xff & BIT(i)))
				m |= BIT(i);
		C32(CM_PW_MASK) = m;
		return 0;
	case 0x400c490c:
		ISP_GET(0xc);
		page_num = w[1];
		return 0;
	case 0x4912:
		ISP_GET(0x1c);
		ISP_LOCK(flags);
		memcpy(CP(CM_SNS_SEL), w, 0x1c);
		if (C32(CM_SNS_SEL + 4) == 1)
			wr(0x10008, C32(CM_SNS_SEL + 8));
		for (i = 0; i < ARRAY_SIZE(sns_sw_gpio); i++) {
			if (sns_sw_gpio[i][1] == (C32(CM_SNS_SEL + 0x10) & 0xff))
				C32(CM_SNS_GPIO) = (C32(CM_SNS_SEL + 0x10) & 0x10000) |
					0x80000000 | sns_sw_gpio[i][0];
			if (sns_sw_gpio[i][2] == (C32(CM_SNS_SEL + 0x14) & 0xff))
				C32(CM_SNS_GPIO + 4) = (C32(CM_SNS_SEL + 0x14) & 0x10000) |
					0x80000000 | sns_sw_gpio[i][0];
		}
		ISP_UNLOCK(flags);
		return 0;
	case 0x4959:
		if (copy_from_user(w, arg, 4)) {
			pr_err("IK:Err: Isp %d: copy_from_user error.\n", 0x4959);
			return -1;
		}
		if (w[0] > 2)
			return -1;
		C32(CM_RUN) |= BIT(w[0]);
		return 0;
	case 0x4915:
		ISP_GET(0x2c);
		pr_info("IK: FS\n");
		mutex_lock(&sns_pw_lock);
		frame_sync = 0;
		memcpy(frame_cnt_attr, w, sizeof(frame_cnt_attr));
		mutex_unlock(&sns_pw_lock);
		if (C32(CM_PW_MASK))
			isp_frame_cnt_sync();
		if (w[6] == 1) {
			isp_set_sns_pw_ctrl(0xff, 1);
		} else if (w[6] == 2 && C32(CM_PW_MASK)) {
			C32(CM_STATE) = 0x10;
			CS32(CM_CH_CUR) = -1;
			if (C8(CM_BPU_MODE) == 1)
				isp_pm_set_mode(1);
			isp_stop_clear();
			isp_set_sns_pw_ctrl(0xff, 1);
		}
		return 0;
	case 0x4916:
		ISP_GET(4);
		if (w[0])
			clk_get_pts(&ircut_pts);
		return 0;
	case 0x495a:
		if (copy_from_user(w, arg, 0x24)) {
			pr_err("IK:Err: Isp %d: copy_from_user error.\n", 0x495a);
			return -1;
		}
		ret = -1;
		if (!C8(CM_DUMP)) {
			C8(CM_DUMP_CHN) = w[0];
			w[0] = (w[0] & 0xffff0000) | (u8)w[0];
			ISP_LOCK(flags);
			C8(CM_DUMP) = 1;
			C16(CM_DUMP_W) = w[0] >> 16;
			C16(CM_DUMP_H) = w[1];
			C32(CM_DUMP_ADDR) = w[3];
			ISP_UNLOCK(flags);
			t = wait_event_interruptible_timeout(dump_wq,
							     C8(CM_DUMP) & 0x30, 0x32);
			if (t < 1 || (C8(CM_DUMP) & 0x20)) {
				w[0] = (w[0] & 0xffff0000) | 0x1100 | (u8)w[0];
			} else {
				w[2] = (w[1] & 0xffff) * (((w[0] >> 16) + 9) / 10) * 16;
				ret = 0;
			}
		}
		C8(CM_DUMP) = 0;
		if (copy_to_user(arg, w, 0x24)) {
			pr_err("IK:Err: IspRegRead : copy_to_user error.\n");
			return -1;
		}
		return ret;
	case 0x495b:
		if (copy_from_user(w, arg, 0x24)) {
			pr_err("IK:Err: Isp %d: copy_from_user error.\n", 0x495b);
			return -1;
		}
		C8(CM_DUMP) = 0;
		return 0;
	case 0x4909:
		if (copy_from_user(w, arg, 4)) {
			pr_err("IK:Err: Isp %d: copy_from_user error.\n", 0x4909);
			return -1;
		}
		c = (w[0] & 0xffffff) >> 16;
		if (c > 2) /* the three VICU inputs, as 0x490a checks */
			return -1;
		if (C8(CM_BPU_MODE) != 3) {
			if ((s32)w[0] < 0)
				w[0] = (rd(0xf0060 + c * 0x1000) & 0xff) |
				       (w[0] & 0xffffff00);
			else
				wr(0xf0060 + c * 0x1000, (w[0] & 0xf) != 0);
		}
		if (copy_to_user(arg, w, 4)) {
			pr_err("IK:Err: IspRegRead : copy_to_user error.\n");
			return -1;
		}
		return 0;
	case 0x490a:
		if (!C8(CM_CHNNUM)) {
			pr_err("IK:Err: %s line%d: ChnNum = %d\n", "isp_ioctl",
			       0xa6f, 0);
			return -1;
		}
		ISP_GET(0xc);
		c = w[2];
		if (c > 2)
			return -1;
		if (CS32(CM_VSYNC(c)) < 1) {
			t = wait_event_interruptible_timeout(vicu_wq[c],
							     CS32(CM_VSYNC(c)) > 0,
							     200);
			if (t < 1)
				return -ERESTARTSYS;
		}
		ret = C32(CM_VSYNC(c));
		C32(CM_VSYNC(c)) = 0;
		C32(CM_VICU(c) + 0x34) = 0;
		return ret;
	case 0x490e:
		ISP_GET(0x34);
		ret = vicu_chn_init((u8 *)w);
		bpu_chn_init((u8 *)w);
		return ret;
	case 0x4900:
		if (copy_from_user(w, arg, 8)) {
			pr_err("IK:Err: Isp %d: copy_from_user error.\n", 0x4900);
			return -1;
		}
		if (w[1] <= 2)
			pr_info("IK: P_%d_%d\n", w[0], w[1]);
		return 0;
	case 0x4908:
		ret = 0;
		if (copy_from_user(w, arg, 0x1c)) {
			pr_err("IK:Err: Isp %d: copy_from_user error.\n", 0x4908);
			return -1;
		}
		if (w[0] < 2)
			memcpy(mipi_info[w[0]], w, sizeof(mipi_info[0]));
		return ret;
	}
	*done = false;
	return 0;
}

static long isp_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	u32 w[0x34 / 4];
	bool done;
	long ret;

	ret = isp_ioctl_sys(cmd, (void __user *)arg, w, &done);
	if (done)
		return ret;
	if (!C8(CM_CHNNUM)) {
		pr_err("IK:Err: %s line%d: ChnNum = %d\n", "isp_ioctl", 0xb4a, 0);
		return -1;
	}
	if (!iw_ioctl(cmd, (void __user *)arg))
		return 0;
	ret = iw_vpss_ioctl(cmd, (void __user *)arg);
	if (ret)
		pr_err("IK:Err: command = %c 0x%x error!\n", (cmd & 0xffff) >> 8,
		       cmd & 0xff);
	return ret;
}

static int isp_proc_show(struct seq_file *m, void *v)
{
	int k = 1, c;
	u32 a, i, mo;

	seq_printf(m, "[ISP232] openxiongmai [V%d.%d.%d.%d.%d.%d.%d] Dev:%d[0x%x 0x%x] Buf:[0x%x] Mode:%d\n",
		   C32(CM_VER), C32(CM_VER + 4), C32(CM_VER + 0x10),
		   C32(CM_VER + 0xc), C32(CM_VER + 0x14), C32(CM_VER + 8),
		   C32(CM_VER + 0x18), C8(CM_CHNNUM), C8(CM_CHN_MASK),
		   C32(CM_RUN), C8(CM_OPT), mode);
	if (!C32(CM_RUN))
		return 0;
	mo = C8(CM_BPU_MODE);
	if (mo == 1)
		seq_puts(m, "BpuMode :[Online(Mul)]\n");
	else if (mo == 0)
		seq_puts(m, "BpuMode :[Online]\n");
	else if (mo == 3)
		seq_printf(m, "BpuMode :[Offline2] [%d]\n", C8(CM_OPT + 1));
	else
		seq_puts(m, "BpuMode :[UNKOWN]\n");
	seq_printf(m, "VpssMode:[%s]\n", C8(CM_VPSS_MODE) ? "Offline" : "Online");
	seq_printf(m, "[%d.VICU info]:\n", k++);
	seq_puts(m, "chn\tfps\tFrmCnt\tibFail\tibSucc\tsnsId\tRsvCnt\tTimInt\tIntMin\tIntMax\twbIdxR\twbIdxW\n");
	for (c = 0; c < N_CHN; c++) {
		a = CM_VICU(c);
		/* the AWB, then the MD statistics' buffer indices (two
		 * more than the header names, as the vendor prints) */
		seq_printf(m, "%d\t%d.%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\n",
			   C32(a + 0x30), C32(a + 0x40) / 10, C32(a + 0x40) % 10,
			   C32(a + 0x44), C32(a + 0x58), C32(a + 0x5c),
			   C32(a + 0x14), C32(a + 0x48), C32(a + 0x4c),
			   C32(a + 0x54), C32(a + 0x50), C8(CM_AWB(c) + ST_R),
			   C8(CM_AWB(c) + ST_W), C8(CM_MD(c) + ST_R),
			   C8(CM_MD(c) + ST_W));
		C32(a + 0x54) = 0xffffffff;
		C32(a + 0x50) = 0;
	}
	seq_puts(m, "\nen\top_type\tchn\t\tsw0_io\tsw0\tpol\tsw1_io\tsw1\tpol\n");
	seq_printf(m, "%d\t%d\t%d [%d %d %d]\t%d\t%d\t%d\t%d\t%d\t%d\n",
		   C32(CM_SNS_SEL), C32(CM_SNS_SEL + 4), C32(CM_SNS_SEL + 8),
		   CS32(CM_CH_PREV), CS32(CM_CH_NOW), CS32(CM_CH_CUR),
		   C32(CM_SNS_SEL + 0x10) & 0xff, C32(CM_SNS_GPIO) & 0xff,
		   (C32(CM_SNS_GPIO) & 0xffffff) >> 16,
		   C32(CM_SNS_SEL + 0x14) & 0xff, C32(CM_SNS_GPIO + 4) & 0xff,
		   (C32(CM_SNS_GPIO + 4) & 0xffffff) >> 16);
	if (mo == 3) {
		seq_printf(m, "\n[%d.BPU_Info]:\n", k++);
		seq_puts(m, "chn\tfps\tcount\ttime\ttimeout\tvbFail\tvbSucc\n");
		for (c = 0; c < N_CHN; c++) {
			a = CM_BPUSTAT(c);
			seq_printf(m, "%d\t%d.%d\t%d\t%lld\t%d\t%d\t%d\n",
				   C32(a), C32(a + 8) / 10, C32(a + 8) % 10,
				   C32(a + 0xc), *(u64 *)CP(a + 0x18),
				   C32(CM_GRP(c) + 0x2c), C32(a + 0x10),
				   C32(a + 0x14));
		}
		seq_puts(m, "all(chn):\n");
		seq_printf(m, "*\t%d.%d\t%d \n", C32(CM_BPUALL + 4) / 10,
			   C32(CM_BPUALL + 4) % 10, C32(CM_BPUALL + 8));
	}
	seq_printf(m, "\n[%d.Capture info]:\nchn\t TotalSize\t ActiveSize\n", k++);
	for (c = 0; c < 2; c++) {
		u32 t = rd(0x2094 + c * 0x1000), s = rd(0x2090 + c * 0x1000);

		seq_printf(m, "%d\t%5d*%5d\t%5d*%5d\n", c, t & 0xffff, t >> 16,
			   s & 0xffff, s >> 16);
	}
	seq_printf(m, "\n[%d.Mipi info]:\nID\tTotalSize(mipi)\tActiveSize(mipi)\tTotalSize(pix)\tActiveSize(pix)\tRepair\n",
		   k++);
	for (c = 0; c < 2; c++) {
		u32 o = 0x50 + c * 0x1000;

		seq_printf(m, "0x%2x\t%4d*%4d\t%4d*%4d\t\t%4d*%4d\t%4d*%4d\t%4d\n",
			   rd(o), rd(o - 8), rd(o - 4), rd(o - 0x10), rd(o - 0xc),
			   rd(o + 0xc4), rd(o + 0xc8), rd(o + 0xbc), rd(o + 0xc0),
			   mipi_info[c][1]);
	}
	seq_printf(m, "\n[%d.Cut_info]:\nMod\tchn\tFlag\tX\tY\tWidth\tHeight\tNote\n",
		   k++);
	for (i = 0; i < C8(CM_CHNNUM) && i < N_CHN; i++) {
		a = CM_CHN(i);
		seq_printf(m, "vicu\t%d\t%d\t%d\t%d\t%d\t%d\t%s\n", i, 1,
			   C16(a), C16(a + 2), C16(a + 4), C16(a + 6), "");
		seq_printf(m, "h_d\t%d\t%d\t \t \t%d\t%d\t%s\n", i, 1,
			   C16(a + 8), C16(a + 0xa), "");
		seq_printf(m, "hv_gen\t%d\t%d\t%d\t%d\t%d\t%d\t[%d %d]\n", i, 1,
			   C16(a + 0xc), C16(a + 0xe), C16(a + 0x10),
			   C16(a + 0x12), C16(a + 0x14), C16(a + 0x16));
		seq_printf(m, "bpu_cut\t%d\t%d\t%d\t%d\t%d\t%d\t%s\n", i, 0,
			   C16(a + 0x18), C16(a + 0x1a), C16(a + 0x1c),
			   C16(a + 0x1e), "UnUse");
		seq_printf(m, "vpss_in\t%d\t%d\t \t \t%d\t%d\t%s\n", i, 1,
			   C16(a + 0x20), C16(a + 0x22), "");
		seq_puts(m, "\n");
	}
	ib_proc_show(m, &k);
	return 0;
}

static int isp_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, isp_proc_show, NULL);
}

static const struct file_operations isp_proc_ops = {
	.owner = THIS_MODULE,
	.open = isp_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static const struct file_operations isp_fops = {
	.owner = THIS_MODULE,
	.open = isp_open,
	.release = isp_release,
	.read = isp_awb_read,
	.write = isp_awb_write,
	.unlocked_ioctl = isp_ioctl,
	.mmap = isp_mmap,
};

static struct miscdevice isp_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "isp",
	.fops = &isp_fops,
};

static void isp_unmap(void)
{
	if (isp_base)
		iounmap(isp_base);
	if (gpio_base)
		iounmap(gpio_base);
	if (pll_base)
		iounmap(pll_base);
	if (ddr_base)
		iounmap(ddr_base);
	if (ircut_base)
		iounmap(ircut_base);
	isp_base = gpio_base = pll_base = ddr_base = ircut_base = NULL;
}

static void isp_free_irqs(void)
{
	if (irq_mark & 1)
		free_irq(VICU_IRQ, NULL);
	if (irq_mark & 2)
		free_irq(VICU_IRQ0, NULL);
	if (irq_mark & 4)
		free_irq(VICU_IRQ1, NULL);
	if (irq_mark & 8)
		free_irq(BPU_IRQ, NULL);
	if (irq_mark & 0x10)
		free_irq(MUX_IRQ, NULL);
	irq_mark = 0;
}

static int __init isp_init(void)
{
	int c, ret = -EIO;

	memset(isp_comm, 0, sizeof(isp_comm));
	C32(CM_VER + 0x14) = 0;
	C32(CM_VER + 0xc) = 0xffffffff;
	C32(CM_VER + 0x10) = 0xffffffff;
	C32(CM_VER + 8) = 0xffffffff;
	for (c = 0; c < N_CHN; c++) {
		mutex_init(&grp_lock[c]);
		init_waitqueue_head(&vicu_wq[c]);
		init_waitqueue_head(&mux_wq[c]);
	}
	mutex_init(&iwctrl_lock);
	mutex_init(&iwctrl_vpss_lock);
	mutex_init(&sns_pw_lock);
	init_waitqueue_head(&dump_wq);
	init_waitqueue_head(&bpu_thread_wq);
	init_waitqueue_head(&sns_pw_wq);
	isp_base = ioremap(ISP_WIN, ISP_WIN_SIZE);
	if (!isp_base) {
		pr_err("IK:Err: ioremap isp_regaddr failed!\n");
		return -ENOMEM;
	}
	gpio_base = ioremap(GPIO_WIN, 0x1000);
	pll_base = ioremap(SYS_WIN, 0x1000);
	ddr_base = ioremap(DDR_WIN, 0x1000);
	ircut_base = ioremap(IRCUT_WIN, 0x2000);
	if (!gpio_base || !pll_base || !ddr_base || !ircut_base) {
		pr_err("IK:Err: ioremap gpio/pll/ddr/ircut_regaddr failed!\n");
		ret = -ENOMEM;
		goto unmap;
	}
	memset(&isp_ops, 0, sizeof(isp_ops));
	proc_create("umap/isp", 0644, NULL, &isp_proc_ops);
	for (c = 0; c < N_CHN; c++)
		C32(CM_VICU(c) + 0x54) = 0xffffffff;
	memset(mipi_info, 0, sizeof(mipi_info));
	isp_pages = alloc_pages(GFP_KERNEL, ISP_PAGES_ORDER);
	if (!isp_pages) {
		pr_err("IK:Err: alloc_pages failed!\n");
		ret = -EIO;
		goto proc;
	}
	memset(page_address(isp_pages), 0, PAGE_SIZE << ISP_PAGES_ORDER);
	if (request_irq(VICU_IRQ, vicu_irq, 0, "vicu_bayer_irq", NULL)) {
		pr_err("IK:Err: vicu_bayer_irq failed!\n");
		goto pages;
	}
	irq_mark |= 1;
	if (mode == 1) {
		if (request_irq(VICU_IRQ0, vicu_irq_0, 0, "vicu_irq0", NULL)) {
			pr_err("IK:Err: vicu_irq0 failed!\n");
			goto irqs;
		}
		irq_mark |= 2;
		if (request_irq(VICU_IRQ1, vicu_irq_1, 0, "vicu_irq1", NULL)) {
			pr_err("IK:Err: vicu_irq1 failed!\n");
			goto irqs;
		}
		irq_mark |= 4;
	}
	if (mode != 2) {
		if (request_irq(BPU_IRQ, bpu_irq, 0, "bpu_hvgen_irq", NULL)) {
			pr_err("IK:Err: Bpu_bayer_Irq_V failed!\n");
			goto irqs;
		}
		irq_mark |= 8;
	}
	if (request_irq(MUX_IRQ, mux_timer, 0, "mux_timer_irq", NULL)) {
		pr_err("IK:Err: mux_timer_irq failed!\n");
		goto irqs;
	}
	irq_mark |= 0x10;
	bpu_task = kthread_run(bpu_thread, NULL, "bpu");
	if (IS_ERR(bpu_task))
		bpu_task = NULL;
	ret = misc_register(&isp_dev);
	if (ret) {
		pr_err("IK:Err: misc_register failed!\n");
		goto thread;
	}
	pr_info("IK: openxiongmai xm232_isp [%d]\n", mode);
	iw_isp_init();
	return 0;
thread:
	if (bpu_task)
		kthread_stop(bpu_task);
	bpu_task = NULL;
irqs:
	isp_free_irqs();
	ret = -EIO;
pages:
	__free_pages(isp_pages, ISP_PAGES_ORDER);
	isp_pages = NULL;
proc:
	remove_proc_entry("umap/isp", NULL);
unmap:
	isp_unmap();
	return ret;
}

static void __exit isp_cleanup(void)
{
	int c;

	if (bpu_task)
		kthread_stop(bpu_task);
	bpu_task = NULL;
	misc_deregister(&isp_dev);
	isp_free_irqs();
	tasklet_kill(&isp_bpu_tasklet);
	isp_exit();
	for (c = 0; c < N_CHN; c++) {
		if (stat_va[1][c][0])
			iounmap(stat_va[1][c][0]);
		stat_va[1][c][0] = stat_va[1][c][1] = NULL;
	}
	isp_unmap();
	remove_proc_entry("umap/isp", NULL);
	if (isp_pages)
		__free_pages(isp_pages, ISP_PAGES_ORDER);
	isp_pages = NULL;
	kfree(isp_regs_buf);
	isp_regs_buf = NULL;
	memset(&isp_ops, 0, sizeof(isp_ops));
	pr_info("IK: rmmod ispDev OK!\n");
}

module_init(isp_init);
module_exit(isp_cleanup);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("XM210V200_ISP");
MODULE_AUTHOR("openxiongmai");
