// SPDX-License-Identifier: GPL-2.0
/*
 * The XM232's BPU: its defaults, the per-frame register writes from a
 * channel's block, the AWB statistics, the offline queue and its tasklet,
 * the BPU interrupt and thread.
 *
 * With the mux (mode 1, the XM232's two sensors on one VICU) vi_mu.ko drives
 * it: isp_set_next_ch() takes the finished channel's statistics
 * (bpu_post_deal) and writes the next one's blocks (bpu_deal). Online
 * (mode 0) the BPU thread does that on the VICU's frame interrupt. Offline
 * (mode 3) the VICU stores raw frames, and the tasklet starts the BPU on
 * each in turn.
 */

#include <linux/delay.h>
#include <linux/kthread.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/string.h>

#include "isp.h"

#define BLK(c) ((u8 *)CP(CM_BLK(c)))
#define IB_BPU 0x10

/* isp.ko's g_bpu_tmp: frames counted for the all-channel rate, and the
 * channels' registers held (0x4960) */
static u32 bpu_frames;
static u8 bpu_held;
static u16 stat_r[512], stat_g[512], stat_b[512];

static u32 rate(u64 now, u64 then)
{
	u64 d;

	if (now <= then)
		return 0xff;
	d = now - then;
	if (d >> 32)
		d = 0xffffffff;
	if (!d)
		return 0xff;
	return (100000000 / (u32)d + 5) / 10;
}

static u32 next_count(u32 n)
{
	return n + 1 > 0xfffffff ? 0 : n + 1;
}

/*
 * One channel's blocks to the registers. Held channels (CM_BPU_OFF) only
 * get their statistics buffers after the first frame.
 */
static int bpu_update_regs(u32 chn, int mode, const u8 *p)
{
	u32 md = chn == 1 ? 0x92000 : 0x82000;
	u32 v0, v1, v2;
	int i;

	if (C32(CM_BPU_OFF) == 0xff)
		return 0;
	if (C32(CM_BPU_OFF) == 0) {
		bpu_held = 0;
	} else if (bpu_held) {
		wr(0x4b118, P32(p, 0x41c));
		wr(0x4b11c, P32(p, 0x420));
		wr(md + 0x118, P32(p, 0x4f8));
		wr(md + 0x11c, P32(p, 0x4fc));
		goto tail;
	}
	wr(0x35000, P16(p, 0xa4));
	wr(0x35004, P16(p, 0xa6));
	wr(0x3500c, P16(p, 0xa8));
	wr(0x35010, P16(p, 0xaa));
	wr(0x35014, P16(p, 0xac));
	for (i = 0; i < 8; i++)
		wr(0x41010 + i * 4, P8(p, 0x606 + i));
	for (i = 0; i < 6; i++)
		wr(0x41030 + i * 4, P16(p, 0x60e + i * 2));
	wr(0x41004, P8(p, 0x605));
	wr(0x41000, P8(p, 0x604));
	wr(0x4901c, P32(p, 0x2bc));
	wr(0x49020, P32(p, 0x2c0));
	wr(0x40010, P16(p, 0x318));
	wr(0x40014, P16(p, 0x31a));
	wr(0x40018, P16(p, 0x31c));
	wr(0x40004, P16(p, 0x31e));
	wr(0x40008, P16(p, 0x320));
	wr(0x4000c, P16(p, 0x322));
	wr(0x40028, P16(p, 0x324));
	wr(0x4002c, P8(p, 0x315));
	wr(0x40030, P8(p, 0x316));
	for (i = 0; i < 81; i++)
		wr(0x40050 + i * 4, P8(p, 0x2c4 + i));
	wr(0x40000, P16(p, 0x326));
	for (i = 0; i < 12; i++)
		wr(0x50040 + i * 4, P32(p, 0x500 + i * 4));
	for (i = 0; i < 97; i++)
		wr(0x51000 + i * 4, P16(p, 0x530 + i * 2));
	for (i = 0; i < 3; i++) {
		wr(0x52000 + i * 4, P16(p, 0x860 + i * 2) & 0x1fff);
		wr(0x5200c + i * 4, P16(p, 0x868 + i * 2) & 0x1fff);
		wr(0x52018 + i * 4, P16(p, 0x870 + i * 2) & 0x1fff);
		wr(0x52024 + i * 4, P8(p, 0x866 + i * 8));
	}
	wr(0x47000, P32(p, 0x338));
	v0 = min_t(u32, P16(p, 0x2b4), 0x3ff);
	v1 = min_t(u32, P16(p, 0x2b6), 0x3ff);
	v2 = min_t(u32, P16(p, 0x2b8), 0x3ff);
	wr(0x45000, v0);
	wr(0x45004, v1);
	wr(0x45008, v2);
	wr(0x4a000, P32(p, 0x33c));
	wr(0x4a004, P32(p, 0x340));
	wr(0x4a008, P32(p, 0x344));
	/* the AWB statistics' zone grid: 33 column and 17 row edges */
	for (i = 0; i < 33; i++)
		wr(0x4b000 + i * 4, P32(p, 0x34c + i * 4));
	for (i = 0; i < 17; i++)
		wr(0x4b0c0 + i * 4, P32(p, 0x3d0 + i * 4));
	if (P32(p, 0x414) && (!P32(p, 0x41c) || !P32(p, 0x420))) {
		pr_err("IK:Err: %s %d~ch%d\n", "bpu_update_regs", 0x1fc, chn);
		P32(p, 0x414) = 0;
	}
	wr(0x4b114, P32(p, 0x418));
	wr(0x4b118, P32(p, 0x41c));
	wr(0x4b11c, P32(p, 0x420));
	wr(0x4b110, P32(p, 0x414));
	if (mode == 3)
		wr(0x46000, 0);
	for (i = 0; i < 14; i++)
		wr(0x46004 + i * 4, P16(p, 0xb0 + i * 2));
	if (P16(p, 0xae)) {
		for (i = 0; i < 61; i++) {
			wr(0x46040 + i * 4, P16(p, 0xcc + i * 2));
			wr(0x46134 + i * 4, P16(p, 0x146 + i * 2));
			wr(0x46228 + i * 4, P16(p, 0x1c0 + i * 2));
			wr(0x4631c + i * 4, P16(p, 0x23a + i * 2));
		}
	}
	wr(0x46000, P16(p, 0xae));
	/* the motion statistics, the sensor's own block */
	if (C32(CM_MD(chn) + ST_PHYS) && C32(CM_MD(chn) + ST_PHYS + 4)) {
		for (i = 0; i < 33; i++)
			wr(md + i * 4, P32(p, 0x428 + i * 4));
		for (i = 0; i < 17; i++)
			wr(md + 0xc0 + i * 4, P32(p, 0x4ac + i * 4));
		wr(md + 0x114, P32(p, 0x4f4));
		wr(md + 0x118, P32(p, 0x4f8));
		wr(md + 0x11c, P32(p, 0x4fc));
	}
tail:
	if (mode == 3) {
		wr(0x42008, P32(p, 0x7fc));
		wr(0x4200c, P32(p, 0x800));
		wr(0x42010, P32(p, 0x804));
		wr(0x42014, P32(p, 0x808));
		wr(0x42000, P32(p, 0x80c));
		wr(0x42004, P32(p, 0x810));
		wr(0x42040, P32(p, 0x814));
		wr(0x43000, P32(p, 0x81c));
		wr(0x43004, P32(p, 0x820));
		wr(0x43008, P32(p, 0x824));
		wr(0x4300c, P32(p, 0x828));
		wr(0x43014, P32(p, 0x830));
		wr(0x43010, P32(p, 0x82c));
		wr(0x43018, 0);
	}
	if (C32(CM_BPU_OFF) & BIT(chn))
		bpu_held = 1;
	return 0;
}

void bpu_init_queue(void)
{
	init_waitqueue_head(&bpu_thread_wq);
}

void bpu_reset(void)
{
	wr(0x4b110, 0);
	bpu_frames = 0;
	bpu_held = 0;
}

int bpu_init(const u8 *init)
{
	int i;

	if (!init)
		pr_err("IK:Err: %s %d failed!\n", "bpu_init", 0x7a);
	memset(CP(CM_GRP(0)), 0, N_CHN * 0x8b0);
	C32(CM_BPU_DEALING) = 0;
	for (i = 0; i < N_CHN; i++)
		mutex_init(&grp_lock[i]);
	mutex_init(&iwctrl_lock);
	wr(0x10000, 0x0a500a50);
	wr(0x10004, 0x0a50001b);
	wr(0x10018, 2);
	wr(0x1001c, 0xffff00cb);
	if (init[8] == 1)
		wr(0x1000c, 3);
	wr(0x4001c, 0x80);
	wr(0x40030, 0);
	wr(0x4101c, 0x5a);
	wr(0x41020, 3);
	wr(0x4901c, 2);
	wr(0x49020, 0);
	wr(0x35018, 0);
	wr(0x52000, 0x418);
	wr(0x52004, 0x809);
	wr(0x52008, 0x18f);
	wr(0x5200c, 0x1da3);
	wr(0x52010, 0x1b5d);
	wr(0x52014, 0x700);
	wr(0x52018, 0x700);
	wr(0x5201c, 0x1a23);
	wr(0x52020, 0x1edd);
	wr(0x52024, 0x10);
	wr(0x52028, 0x80);
	wr(0x5202c, 0x80);
	if (init[8] == 3)
		wr(0x42024, 1);
	return 0;
}

/* 0x490e's 52 bytes: chn, sensor, ..., crops and sizes from 8 */
int bpu_chn_init(const u8 *attr)
{
	u32 c = attr[0], grp, i;

	if (c >= N_CHN)
		return -1;
	grp = CM_GRP(c);
	C32(grp + GRP_SNS) = attr[1];
	C32(grp + GRP_ON2) = 1;
	C32(grp + GRP_ON) = 1;
	C8(grp + 0x24) = attr[6];
	for (i = 0; i < 10; i++)
		C32(CM_CHN(c) + i * 4) = *(const u32 *)(attr + 8 + i * 4);
	return 0;
}

/*
 * The channel's AWB statistics: 512 zones of {R, G, B} sums, each divided by
 * a quarter of the zone's pixels (the block's 0x348, from 0x496d), G
 * halved. read() returns them.
 */
int awb_stat_get(u32 chn)
{
	const u32 __iomem *s;
	u32 div, a, b;
	int i;

	if (chn >= N_CHN || !(C32(CM_RUN) & BIT(chn)))
		return -1;
	div = P32(BLK(chn), 0x348);
	s = stat_va[0][chn][C8(CM_AWB(chn) + ST_R)];
	if (!div)
		div = 1;
	if (!s) {
		pr_err("IK:Err: %s %d failed!\n", "awb_stat_get", 0xb8);
	} else {
		for (i = 0; i < 512; i++, s += 4) {
			dsb();
			a = readl_relaxed(s + 1);
			dsb();
			b = readl_relaxed(s + 2);
			dsb();
			stat_r[i] = (readl_relaxed(s) << 2) / div;
			stat_g[i] = ((a << 2) / div) >> 1;
			stat_b[i] = (b << 2) / div;
		}
	}
	memcpy(CP(CM_AWB_STAT(chn)), stat_r, sizeof(stat_r));
	memcpy(CP(CM_AWB_STAT(chn) + 0x400), stat_g, sizeof(stat_g));
	memcpy(CP(CM_AWB_STAT(chn) + 0x800), stat_b, sizeof(stat_b));
	return 0;
}

int bpu_post_deal(u32 chn)
{
	awb_stat_get(chn);
	vpss_get_stat(chn);
	return 0;
}

/* a statistics buffer pair's next: its start and end >> 3 for the BPU */
static void stat_next(u32 st, u8 *blk, u32 o)
{
	u32 w = (C8(st + ST_W) + 1) & 1, phys, start, end;

	C8(st + ST_W) = w;
	C8(st + ST_R) = w ^ 1;
	phys = C32(st + ST_PHYS + w * 4);
	if (phys <= 0x80000000) {
		start = 0;
		end = 0x400;
	} else {
		start = (phys - 0x80000000) >> 3;
		end = start + 0x400;
	}
	P32(blk, o) = start;
	P32(blk, o + 4) = end;
}

static int bpu_set_val(u32 chn, u8 *blk)
{
	u32 c = CM_CHN(chn);

	P32(blk, 0x7fc) = C16(c + 0x0c);
	P32(blk, 0x800) = C16(c + 0x10) + C16(c + 0x0c);
	P32(blk, 0x804) = C16(c + 0x0e);
	P32(blk, 0x808) = C16(c + 0x12) + C16(c + 0x0e);
	P32(blk, 0x80c) = C16(c + 0x14) - 1;
	P32(blk, 0x810) = C16(c + 0x16) - 1;
	stat_next(CM_AWB(chn), blk, 0x41c);
	stat_next(CM_MD(chn), blk, 0x4f8);
	if (C8(CM_BPU_MODE) == 3) {
		P32(blk, 0x814) = 1;
		P32(blk, 0x818) = 1;
		P32(blk, 0x82c) = 1;
	} else {
		P32(blk, 0x814) = 0;
		P32(blk, 0x818) = 0;
		P32(blk, 0x82c) = 0;
	}
	P32(blk, 0x830) = 0;
	P32(blk, 0x84c) = 0;
	P32(blk, 0x850) = 0;
	return 0;
}

/* offline: the next stored channel to the BPU */
void bpu_offline_deal_in(unsigned long unused)
{
	u32 chn, grp, vgrp, size, w, h, stride, base, clk;
	unsigned long flags, f2;
	void *buf = NULL;
	u8 mode;
	u64 now;

	if ((C8(CM_OPT + 1) & 3) < 2 || C32(CM_STATE) == 0x11)
		return;
	mode = C8(CM_BPU_MODE);
	ISP_LOCK(flags);
	if (C32(CM_BPU_RUN)) {
		ISP_UNLOCK(flags);
		return;
	}
	C32(CM_BPU_RUN) |= 0x80000000;
	ISP_LOCK(f2);
	if (!C8(CM_PEND_N)) {
		chn = 0xffffffff;
	} else {
		chn = C8(CM_PEND + C8(CM_PEND_HEAD));
		C8(CM_PEND_N)--;
		C32(CM_BPU_DEALING) = 1;
		C8(CM_PEND_HEAD) = C8(CM_PEND_HEAD) + 1 < 5 ?
			C8(CM_PEND_HEAD) + 1 : 0;
	}
	ISP_UNLOCK(f2);
	if (chn >= N_CHN) {
		pr_err("IK:Err: %s line%d ch%d~~~~~~~~~~~~~~~~~~~~~~~~~%d\n",
		       "bpu_offline_deal_in", 0x29e, chn, C8(CM_PEND_N));
		goto out;
	}
	clk_get_pts(&now);
	*(u64 *)CP(CM_TIME(chn) + 0x18) = now;
	ib_get_buf(chn, IB_BPU, 0, &buf, 1000);
	if (!buf)
		goto out;
	grp = CM_GRP(chn);
	*(u64 *)CP(CM_TIME(chn)) = *(u64 *)((u8 *)buf + 0x40);
	C32(CM_TIME(chn) + 0x08) = P32(buf, 0x3c);
	C32(CM_TIME(chn) + 0x0c) = P32(buf, 0x38);
	size = C32(CM_CHN(chn) + 0x10);
	if (!C32(grp + GRP_ON) || !C32(grp + GRP_ON2) ||
	    (size & 0xffff) < P32(buf, 0x18) || (size >> 16) < P32(buf, 0x1c))
		pr_err("IK:Err: %s line[%d] ch%d:%d %d [%d*%d] [%d*%d]\n",
		       "bpu_offline_deal_in", 700, chn, C32(grp + GRP_ON),
		       C32(grp + GRP_ON2), size & 0xffff, size >> 16,
		       P32(buf, 0x18), P32(buf, 0x1c));
	C32(grp + GRP_CNT)++;
	w = P32(buf, 0x18);
	h = P32(buf, 0x1c);
	C32(grp + GRP_W) = w;
	C32(grp + GRP_H) = h;
	base = (P32(buf, 0x10) + 0x80000000) >> 3;
	stride = ((w + 5) / 6) * 6;
	C32(grp + 0x854) = base;
	C32(grp + 0x858) = (h - 1) * (stride / 6) + base;
	C32(grp + 0x85c) = stride / 6;
	C32(grp + 0x860) = stride / 6;
	clk = 6;
	bpu_set_val(chn, BLK(chn));
	sys_get_clk(0x50, &clk);
	sys_set_clk(0x50, clk | 0x100);
	bpu_update_regs(chn, mode, BLK(chn));
	vgrp = C32(grp + GRP_VPSS);
	size = C32(CM_CHN(chn) + 0x20);
	vpss_isp_set_val_v2(vgrp, size & 0xffff, size >> 16, 1);
	if (isp_ops.vpss_set)
		isp_ops.vpss_set(vgrp, stride, w, h, P32(buf, 0x3c));
	wr(0x42020, C32(grp + 0x850));
	if (isp_ops.vpss_start)
		isp_ops.vpss_start(vgrp, 1);
	ISP_LOCK(f2);
	C32(CM_BPU_RUN) |= BIT(chn);
	C32(CM_BPU_GO) |= BIT(chn);
	ISP_UNLOCK(f2);
	wake_up(&bpu_thread_wq);
out:
	C32(CM_BPU_RUN) &= 0x7fffffff;
	ISP_UNLOCK(flags);
}

/* offline: queue a stored channel (chn >= 3: none), start the tasklet */
int bpu_offline_deal(u32 chn)
{
	unsigned long flags;
	u8 n;

	ISP_LOCK(flags);
	if (chn < N_CHN) {
		clk_get_pts(CP(CM_TIME(chn) + 0x10));
		n = C8(CM_PEND_N);
		if (n < 5) {
			C8(CM_PEND + C8(CM_PEND_TAIL)) = chn;
			C8(CM_PEND_N) = ++n;
			C8(CM_PEND_TAIL) = C8(CM_PEND_TAIL) + 1 < 5 ?
				C8(CM_PEND_TAIL) + 1 : 0;
		} else if (C16(CM_PEND_LOST) != 0xffff) {
			C16(CM_PEND_LOST)++;
		}
	} else {
		n = C8(CM_PEND_N);
	}
	if (!C32(CM_BPU_RUN) && n)
		tasklet_schedule(&isp_bpu_tasklet);
	ISP_UNLOCK(flags);
	return 0;
}

/* online: the channel's blocks to the registers, then the VPSS side */
int bpu_deal(u32 chn)
{
	u32 vgrp, size;

	if (chn >= N_CHN || !(C32(CM_RUN) & BIT(chn)) ||
	    C32(CM_STATE) == 0x11)
		return -1;
	mutex_lock(&grp_lock[chn]);
	bpu_set_val(chn, BLK(chn));
	bpu_update_regs(chn, C8(CM_BPU_MODE), BLK(chn));
	mutex_unlock(&grp_lock[chn]);
	if (C8(CM_VPSS_MODE))
		return 0;
	vgrp = C32(CM_GRP(chn) + GRP_VPSS);
	size = C32(CM_CHN(chn) + 0x20);
	vpss_isp_set_val_v2(vgrp, size & 0xffff, size >> 16, 0);
	vpss_isp_post_deal(vgrp);
	return 0;
}

extern atomic_t isp_users;

int bpu_thread(void *unused)
{
	u32 go, bit, chn;
	long t;
	u8 mode;

	C8(CM_BPU_RUNNING) = 1;
	while (!kthread_should_stop()) {
		mode = C8(CM_BPU_MODE);
		if (mode != 0 && mode != 3) {
			schedule_timeout_interruptible(msecs_to_jiffies(100));
			continue;
		}
		if (!atomic_read(&isp_users) || !C32(CM_RUN)) {
			schedule_timeout_interruptible(msecs_to_jiffies(100));
			continue;
		}
		if (CS32(CM_BPU_GO) < 1) {
			t = wait_event_interruptible_timeout(bpu_thread_wq,
							     CS32(CM_BPU_GO) > 0,
							     100);
			if (t < 1)
				continue;
		}
		go = C32(CM_BPU_GO);
		if (!go)
			continue;
		C32(CM_BPU_GO) = 0;
		if (go & 1) {
			bit = 1;
			chn = 0;
		} else if (go & 2) {
			bit = 2;
			chn = 1;
		} else {
			bit = 4;
			chn = 2;
		}
		if (!(bit & C32(CM_RUN)))
			continue;
		if (C8(CM_BPU_MODE) == 0)
			bpu_deal(chn);
		isp_deal_frame_cnt(chn);
	}
	C8(CM_BPU_RUNNING) = 0;
	return 0;
}

/*
 * The BPU's frame end (bpu_hvgen_irq): acknowledged; offline, the finished
 * channel's statistics and rates, then the next stored channel. Like
 * isp.ko's it reports the interrupt handled only when a channel finished.
 */
irqreturn_t bpu_irq(int irq, void *dev)
{
	irqreturn_t ret = IRQ_NONE;
	unsigned long flags;
	u32 run, chn, st, clk;
	u64 now, then, d;

	if (!rd(0x42044))
		goto out;
	wr(0x42044, 0);
	if (C32(CM_RUN)) {
		ISP_LOCK(flags);
		run = C32(CM_BPU_RUN) & 0xff;
		C32(CM_BPU_RUN) &= 0xffffff00;
		if (run) {
			clk_get_pts(&now);
			for (chn = 0; chn < N_CHN; chn++)
				if (run & BIT(chn))
					break;
			if (chn < N_CHN) {
				st = CM_BPUSTAT(chn);
				C32(st + 0x08) = rate(now, *(u64 *)CP(st + 0x20));
				*(u64 *)CP(st + 0x20) = now;
				C32(st + 0x0c) = next_count(C32(st + 0x0c));
				C32(st + 0x00) = chn;
				C32(st + 0x04) = 1;
			}
			if (++bpu_frames > 1000)
				bpu_frames = 1000;
			then = *(u64 *)CP(CM_BPUALL + 0x1c);
			if (now > then) {
				d = now - then;
				if (d >> 32)
					d = 0xffffffff;
				d = (u32)d / 1000;
				if (d > 1999) {
					C32(CM_BPUALL + 0x04) =
						(bpu_frames * 100000 / (u32)d + 5) / 10;
					bpu_frames = 0;
					*(u64 *)CP(CM_BPUALL + 0x1c) = now;
				}
			}
			C32(CM_BPUALL + 0x08) = next_count(C32(CM_BPUALL + 0x08));
			C32(CM_BPUALL) = 1;
			/* isp.ko goes on with channel 3 when only higher bits
			 * are set, past its tables; this stops there */
			if (chn < N_CHN) {
				awb_stat_get(chn);
				vpss_get_stat(chn);
				then = *(u64 *)CP(CM_TIME(chn) + 0x18);
				*(u64 *)CP(CM_TIME(chn) + 0x20) = now;
				*(u64 *)CP(CM_BPUSTAT(chn) + 0x18) =
					now > then ? now - then : 0;
				if (isp_ops.bpu_done)
					isp_ops.bpu_done(C32(CM_GRP(chn) + GRP_VPSS));
			}
			ret = IRQ_HANDLED;
		}
		C32(CM_BPU_DEALING) = 0;
		ISP_UNLOCK(flags);
	}
	sys_get_clk(0x50, &clk);
	sys_set_clk(0x50, clk & ~0x100);
out:
	bpu_offline_deal(0xff);
	return ret;
}
