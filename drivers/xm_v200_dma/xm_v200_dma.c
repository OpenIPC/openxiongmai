// SPDX-License-Identifier: GPL-2.0
/*
 * The XM530V200's memory-to-memory DMA (/dev/dma): a drop-in for the
 * vendor xm_dma.ko. It runs the DMAC, an ARM PL330 at 0x20020000 whose
 * event 0 is IRQ 50, on channel 0 with microcode written here.
 *
 * ioctls, on physical addresses:
 *   0x400c4401  {src, dst, len}: len bytes, a multiple of 8
 *   0x40144402  {src, dst, stride, width, height}: height rows of width
 *               bytes (a multiple of 8, 32 KiB at most) stride bytes apart,
 *               the same in both
 *
 * Neither may touch the kernel's own RAM (xm_dma.ko does not check).
 *
 * The microcode moves 16 beats of 8 bytes a burst, then the rest a beat
 * at a time; a 2D copy repeats that per row and steps both addresses on by
 * the gap (DMAADDH). It ends with DMASEV 0, which interrupts.
 */

#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/miscdevice.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/sched.h>
#include <linux/uaccess.h>
#include <linux/wait.h>
#include <asm/memory.h>

#define DMAC_BASE	0x20020000
#define XM_DMA_IRQ	50
#define INTEN		0x020
#define INTMIS		0x028
#define INTCLR		0x02c
#define FSRC		0x034
#define FTR(c)		(0x040 + 4 * (c))
#define CSR(c)		(0x100 + 8 * (c))
#define DBGSTATUS	0xd00
#define DBGCMD		0xd04
#define DBGINST0	0xd08
#define DBGINST1	0xd0c

#define DMA_COPY	0x400c4401
#define DMA_COPY_2D	0x40144402

#define PROG_SIZE	0x1000
#define BURST		16	/* beats of 8 bytes */
#define MAX_WIDTH	32768

/* source and destination: incrementing, 8-byte beats, n a burst, AXI
 * prot 7 (privileged, non-secure, as xm_dma.ko's) */
#define CCR(n)		(1 | 3 << 1 | ((n) - 1) << 4 | 7 << 8 | \
			 1 << 14 | 3 << 15 | ((n) - 1) << 18 | 7 << 22)

static void __iomem *regs;
static u8 *prog;
static dma_addr_t prog_phys;
static DEFINE_MUTEX(dma_lock);
static DECLARE_WAIT_QUEUE_HEAD(dma_wq);
static int dma_done;

struct mc {
	u8 *p;
	int n;
};

static void emit(struct mc *m, const u8 *b, int n)
{
	if (m->n + n <= PROG_SIZE)
		memcpy(m->p + m->n, b, n);
	m->n += n;
}

static void mov(struct mc *m, int rd, u32 v)
{
	u8 b[6] = { 0xbc, rd, v, v >> 8, v >> 16, v >> 24 };

	emit(m, b, 6);
}

#define SAR 0
#define CCR_REG 1
#define DAR 2

static int lp(struct mc *m, int lc, u32 iter)
{
	u8 b[2] = { 0x20 | lc << 1, iter - 1 };

	emit(m, b, 2);
	return m->n;
}

static void lpend(struct mc *m, int lc, int start)
{
	u8 b[2] = { 0x38 | lc << 2, m->n - start };

	emit(m, b, 2);
}

static void ldst(struct mc *m)
{
	static const u8 b[2] = { 0x04, 0x08 };

	emit(m, b, 2);
}

/* ra: 0 SAR, 1 DAR (not DMAMOV's numbering) */
static void addh(struct mc *m, int ra, u16 v)
{
	u8 b[3] = { 0x54 | ra << 1, v, v >> 8 };

	emit(m, b, 3);
}

/* len bytes: bursts in loops, then the beats left */
static void copy_run(struct mc *m, u32 len, int lc_free)
{
	u32 bursts = len / (BURST * 8), beats = len % (BURST * 8) / 8;
	int a, b;

	if (bursts)
		mov(m, CCR_REG, CCR(BURST));
	/* 256 x 256 bursts a block, with the outer counter if it is ours */
	while (lc_free && bursts >= 256) {
		u32 outer = min(bursts / 256, 256u);

		a = lp(m, 1, outer);
		b = lp(m, 0, 256);
		ldst(m);
		lpend(m, 0, b);
		lpend(m, 1, a);
		bursts -= outer * 256;
	}
	while (bursts) {
		u32 n = min(bursts, 256u);

		b = lp(m, 0, n);
		ldst(m);
		lpend(m, 0, b);
		bursts -= n;
	}
	if (beats) {
		mov(m, CCR_REG, CCR(beats));
		ldst(m);
	}
}

static void finish(struct mc *m)
{
	static const u8 b[4] = { 0x13, 0x34, 0x00, 0x00 }; /* WMB, SEV 0, END */

	emit(m, b, 4);
}

static irqreturn_t dma_irq(int irq, void *dev)
{
	if (!(readl(regs + INTMIS) & 1))
		return IRQ_NONE;
	writel(1, regs + INTCLR);
	dma_done = 1;
	wake_up(&dma_wq);
	return IRQ_HANDLED;
}

static void dbg_exec(u32 inst0, u32 inst1)
{
	int n = 1000;

	while ((readl(regs + DBGSTATUS) & 1) && --n)
		udelay(1);
	writel(inst0, regs + DBGINST0);
	writel(inst1, regs + DBGINST1);
	writel(0, regs + DBGCMD);
}

static int run(struct mc *m)
{
	if (m->n > PROG_SIZE) {
		pr_err("xm_dma: program too long\n");
		return -EINVAL;
	}
	wmb();
	dma_done = 0;
	/* DMAGO channel 0, non-secure, from the manager thread */
	dbg_exec(0xa2 << 16, prog_phys);
	if (!wait_event_timeout(dma_wq, dma_done, 2 * HZ) ||
	    (readl(regs + FSRC) & 1)) {
		pr_err("xm_dma: channel 0 %s (csr %#x ftr %#x)\n",
		       dma_done ? "fault" : "timeout",
		       readl(regs + CSR(0)), readl(regs + FTR(0)));
		/* DMAKILL on channel 0 */
		dbg_exec(0x01 << 16 | 0 << 8 | 1, 0);
		return -EIO;
	}
	return 0;
}

/* not the kernel's RAM */
static bool allowed(u32 addr, u64 len)
{
	u64 ram0 = PHYS_OFFSET, ram1 = __pa(high_memory - 1) + 1;

	return len && addr + len <= 0x100000000ull &&
	       (addr + len <= ram0 || addr >= ram1);
}

static long dma_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	u32 r[5];
	struct mc m = { prog, 0 };
	u32 row, rows, gap;
	int a, ret;

	switch (cmd) {
	case DMA_COPY:
		if (copy_from_user(r, (void __user *)arg, 12))
			return -EFAULT;
		if (r[2] & 7 || !allowed(r[0], r[2]) || !allowed(r[1], r[2]))
			return -EINVAL;
		mutex_lock(&dma_lock);
		mov(&m, SAR, r[0]);
		mov(&m, DAR, r[1]);
		copy_run(&m, r[2], 1);
		break;
	case DMA_COPY_2D:
		if (copy_from_user(r, (void __user *)arg, 20))
			return -EFAULT;
		/* src, dst, stride, width, height */
		if (r[3] & 7 || !r[3] || r[3] > MAX_WIDTH || !r[4] ||
		    r[2] < r[3] || r[2] - r[3] > 0xffff ||
		    !allowed(r[0], (u64)r[2] * (r[4] - 1) + r[3]) ||
		    !allowed(r[1], (u64)r[2] * (r[4] - 1) + r[3]))
			return -EINVAL;
		gap = r[2] - r[3];
		mutex_lock(&dma_lock);
		mov(&m, SAR, r[0]);
		mov(&m, DAR, r[1]);
		if (!gap) {
			copy_run(&m, r[3] * r[4], 1);
			break;
		}
		for (row = 0; row < r[4]; row += rows) {
			rows = min(r[4] - row, 256u);
			a = lp(&m, 1, rows);
			copy_run(&m, r[3], 0);
			addh(&m, 0, gap);
			addh(&m, 1, gap);
			lpend(&m, 1, a);
		}
		break;
	default:
		return -ENOTTY;
	}
	finish(&m);
	ret = run(&m);
	mutex_unlock(&dma_lock);
	return ret;
}

static const struct file_operations dma_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = dma_ioctl,
};

static struct miscdevice dma_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "dma",
	.fops = &dma_fops,
};

static int __init xm_v200_dma_init(void)
{
	int ret;

	regs = ioremap(DMAC_BASE, 0x1000);
	if (!regs)
		return -ENOMEM;
	prog = dma_alloc_coherent(NULL, PROG_SIZE, &prog_phys, GFP_KERNEL);
	if (!prog) {
		ret = -ENOMEM;
		goto unmap;
	}
	ret = request_irq(XM_DMA_IRQ, dma_irq, 0, "dma-irq", NULL);
	if (ret)
		goto free;
	writel(1, regs + INTCLR);
	writel(1, regs + INTEN);
	ret = misc_register(&dma_misc);
	if (ret)
		goto irq;
	return 0;
irq:
	writel(0, regs + INTEN);
	free_irq(XM_DMA_IRQ, NULL);
free:
	dma_free_coherent(NULL, PROG_SIZE, prog, prog_phys);
unmap:
	iounmap(regs);
	return ret;
}

static void __exit xm_v200_dma_exit(void)
{
	misc_deregister(&dma_misc);
	writel(0, regs + INTEN);
	free_irq(XM_DMA_IRQ, NULL);
	dma_free_coherent(NULL, PROG_SIZE, prog, prog_phys);
	iounmap(regs);
}

module_init(xm_v200_dma_init);
module_exit(xm_v200_dma_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("XM530V200 memory-to-memory DMA, compatible with xm_dma.ko");
