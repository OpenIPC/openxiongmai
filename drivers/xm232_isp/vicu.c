// SPDX-License-Identifier: GPL-2.0
/*
 * The XM232's VICU: channel set-up, the frame interrupt (vicu_bayer_irq)
 * and its rates, the raw-frame dump, and the offline mode's store
 * interrupts (vicu_irq0/1, mode=1 only), which put raw frames into ISP
 * buffers (ib.c) for the BPU.
 *
 * With the mux (mode 1) the frame interrupt only counts: vi_mu.ko switches
 * the sensors (isp_set_next_ch). Online (mode 0) it picks the channel the
 * sensor select names, takes its statistics and wakes the BPU thread.
 */

#include <linux/module.h>
#include <linux/sched.h>
#include <linux/time.h>

#include "isp.h"

#define IB_FREE 0
#define IB_BPU 0x10

/* isp.ko's g_vicu_tmp: per channel, the buffer being stored (6..8) and
 * the one before (3..5) */
static void *vicu_buf[9];
static u8 dump_flag; /* s_flag: 0 idle, 1 storing, 2 stored */

void vicu_init_queue(void)
{
	int c;

	for (c = 0; c < N_CHN; c++)
		init_waitqueue_head(&vicu_wq[c]);
}

/* The time-shared VICU's next sensor (offline, three channels) */
static u32 vicu_chn_ext_switch(int vicu, u32 chn)
{
	u32 g, v40, v50;

	if (C8(CM_CHNNUM) <= 2)
		return chn;
	if (vicu == 0) {
		C8(CM_EXT) = C8(CM_EXT + 1);
		C8(CM_EXT + 1) = C8(CM_EXT + 2);
		C8(CM_EXT + 2) = C8(CM_EXT + 2) ? 0 : 2;
		wr(0x1000c, 3);
		g = (C32(CM_SNS_GPIO) & 0xf) << 8;
		if (C32(CM_SNS_GPIO) & 0x10000) {
			v40 = g | 0x50;
			v50 = g | 0x40;
		} else {
			v40 = g | 0x40;
			v50 = g | 0x50;
		}
		wr(0x10008, C8(CM_EXT + 2) ? v40 : v50);
		wr(0x10014, 1);
	}
	return C8(CM_EXT + 1);
}

/* 0x490e's 52 bytes: chn, ..., the raw size at 0x18, at 0x30 a word kept */
int vicu_chn_init(const u8 *attr)
{
	u32 c = attr[0], v, size, d;

	if (c >= N_CHN)
		return -1;
	v = CM_VICU(c);
	size = *(const u32 *)(attr + 0x18);
	C32(v + 0x00) = c;
	C32(v + 0x14) = *(const u32 *)(attr + 0x30);
	C32(v + 0x8c) = (((size & 0xffff) + 5) / 6) * (size >> 16) * 8;
	if (!C8(CM_CHNNUM)) {
		pr_err("IK:Err: %s %d failed! [%d %d]\n", "vicu_reg_init", 0x2f,
		       0, c);
	} else {
		C32(v + 0x90) = 1;
		wr(0xf0064, 1);
		wr(0xf001c, 1);
		if (C8(CM_VPSS_MODE)) {
			wr(0xf0018, 2);
			wr(0xf0028, 1);
		}
		if (C8(CM_BPU_MODE) == 3) {
			d = c == 1 ? 0x10000 : 0;
			wr(0xf0018, 0);
			wr(0xf001c, 1);
			wr(0xf0020, 1);
			wr(0x37050 - d, 0);
			wr(0x3704c - d, 2);
			wr(0x37044 - d, 0);
			wr(0x37004 - d, 1);
			wr(0x36044 - d, 2);
			wr(0x36040 - d, 1);
			wr(0x36054 - d, *(const u16 *)(attr + 0xe) * 15 >> 4);
			wr(0x36050 - d, 1);
		}
		wr(0xf002c, !(C8(CM_OPT) & 4));
		wr(0x4000, 0);
	}
	init_waitqueue_head(&dump_wq);
	return 0;
}

void vicu_reset(void)
{
	int c;

	wr(0x37000, 0);
	wr(0x27000, 0);
	wr(0xf0060, 0);
	wr(0x36040, 0);
	wr(0x36050, 0);
	wr(0x26040, 0);
	wr(0x26050, 0);
	for (c = 0; c < N_CHN; c++) {
		vicu_buf[c] = vicu_buf[c + 3] = vicu_buf[c + 6] = NULL;
		if (stat_va[0][c][0])
			iounmap(stat_va[0][c][0]);
		stat_va[0][c][0] = stat_va[0][c][1] = NULL;
		C32(CM_AWB(c) + ST_PHYS) = 0;
		C32(CM_AWB(c) + ST_PHYS + 4) = 0;
		C8(CM_AWB(c) + ST_W) = 0;
		C8(CM_AWB(c) + ST_R) = 0;
	}
	C32(CM_SNS_SEL) = 0;
	C32(CM_SNS_SEL + 4) = 1;
	C32(CM_SNS_SEL + 8) = 1;
	CS32(CM_CH_CUR) = -1;
	CS32(CM_CH_NOW) = -1;
	CS32(CM_CH_PREV) = -1;
}

/*
 * 0x495a's raw-frame dump, a frame at a time: the store set up on one
 * frame, checked on the next. Returns 1 while it owns the channel's frame.
 */
static int vicu_bayerdump(u32 chn)
{
	u8 st = C8(CM_DUMP);
	u32 addr = C32(CM_DUMP_ADDR), w = C16(CM_DUMP_W), h = C16(CM_DUMP_H);
	u32 d = chn == 1 ? 0x10000 : 0, s;
	bool stored;

	if (C8(CM_DUMP_CHN) != chn || !(st & 1))
		return 0;
	if (dump_flag == 1) {
		stored = rd(0x37008 - d) != rd(0x37084 - d);
		if (stored)
			dump_flag = 2;
		else
			st = 0x20;
		wr(0x37000 - d, 0);
		wr(0x37050 - d, 0);
		if (stored)
			return 1;
		st &= 0xf0;
	} else if (dump_flag == 0) {
		wr(0x37000 - d, 0);
		wr(0x37050 - d, 0);
		wr(0x37004 - d, 3);
		wr(0x37008 - d, (addr + 0x80000000) >> 3);
		s = w / 6;
		wr(0x37018 - d, s);
		wr(0x3701c - d, s);
		wr(0x37020 - d, h * s);
		wr(0x37044 - d, 0);
		wr(0x3704c - d, 2);
		wr(0x37000 - d, 3);
		dump_flag = 1;
		return 1;
	} else if (dump_flag != 2) {
		return 1;
	} else {
		st = 0x10;
	}
	C8(CM_DUMP) = st;
	dump_flag = 0;
	wake_up(&dump_wq);
	return 1;
}

/* a channel's frame: its interval, rate and count */
static void vicu_count(u32 chn, struct timeval *tv)
{
	u32 v = CM_VICU(chn), us, ms;

	us = (tv->tv_usec - C32(v + 0x3c)) + (tv->tv_sec - C32(v + 0x38)) * 1000000;
	ms = us / 1000;
	C32(v + 0x4c) = ms;
	if (C32(v + 0x50) < ms)
		C32(v + 0x50) = ms;
	if (ms < C32(v + 0x54))
		C32(v + 0x54) = ms;
	C32(v + 0x40) = us ? (100000000 / us + 5) / 10 : 0xff;
	C32(v + 0x38) = tv->tv_sec;
	C32(v + 0x3c) = tv->tv_usec;
	C32(v + 0x44) = C32(v + 0x44) + 1 > 0xffff ? 0 : C32(v + 0x44) + 1;
	C32(v + 0x30) = chn;
	C32(v + 0x34) = 1;
}

/* vicu_bayer_irq: a sensor's frame is in */
irqreturn_t vicu_irq(int irq, void *dev)
{
	struct timeval tv;
	u32 mask, cur, next, c;
	bool mux, any = false;
	int dump;

	if (!rd(0xf006c))
		return IRQ_HANDLED;
	wr(0xf006c, 0);
	cur = 0;
	if (C8(CM_BPU_MODE) == 1) {
		if (CS32(CM_CH_CUR) < 0)
			return IRQ_HANDLED;
		mask = BIT(C32(CM_CH_CUR));
		mux = true;
	} else {
		if (!C32(CM_SNS_SEL)) {
			wr(0x10008, 0);
			mask = 1;
			next = 1;
		} else if (C32(CM_SNS_SEL + 4) == 1) {
			cur = readl_relaxed(isp_base + 0x10008);
			dsb();
			mask = BIT(cur);
			next = cur + 1;
			C32(CM_SNS_SEL + 8) = cur;
		} else if (CS32(CM_CH_CUR) < 0) {
			next = 1;
			mask = 1;
		} else {
			cur = C32(CM_CH_CUR);
			mask = BIT(cur);
			next = cur + 1;
		}
		C32(CM_CH_NOW) = cur;
		C32(CM_CH_CUR) = next;
		if (((C32(CM_SNS_SEL + 0xc) & 0xffff) >> 8) < C32(CM_CH_CUR))
			C32(CM_CH_CUR) = C32(CM_SNS_SEL + 0xc) & 0xff;
		mux = false;
	}
	if (!C32(CM_RUN) || !mask)
		return IRQ_HANDLED;
	do_gettimeofday(&tv);
	for (c = 0; c < N_CHN; c++) {
		if (!(mask & BIT(c)))
			continue;
		if (!C8(CM_BPU_RUNNING))
			break;
		vicu_count(c, &tv);
		dump = vicu_bayerdump(c);
		if (mux)
			continue;
		if (!dump) {
			any = true;
			bpu_post_deal(c);
			C32(CM_BPU_GO) = BIT(c);
			C32(CM_VSYNC(c)) |= BIT(c);
			wake_up(&vicu_wq[c]);
		} else {
			C32(CM_VSYNC(c)) |= BIT(c);
			wake_up(&vicu_wq[c]);
			if (!any)
				continue;
		}
		wake_up(&bpu_thread_wq);
	}
	return IRQ_HANDLED;
}

/* offline (mode=1): a channel's frame and rate, the store's buffer back */
irqreturn_t vicu_irq_0(int irq, void *dev)
{
	u32 mask = 0, c, v, d, bit;
	bool stored, go;
	void *b;
	u64 now;

	if (rd(0x3604c)) {
		if (rd(0x36008))
			mask = BIT(vicu_chn_ext_switch(0, 0));
		wr(0x3604c, 0);
	}
	if (rd(0x2604c)) {
		if (rd(0x26008))
			mask |= 2;
		wr(0x2604c, 0);
	}
	if (!mask || !C32(CM_RUN))
		return IRQ_HANDLED;
	clk_get_pts(&now);
	for (c = 0; c < N_CHN; c++) {
		if (!(mask & BIT(c)) || !C8(CM_BPU_RUNNING))
			continue;
		bit = BIT(c);
		go = false;
		if (!vicu_bayerdump(c) && C8(CM_BPU_MODE) == 3) {
			b = vicu_buf[c + 6];
			vicu_buf[c + 6] = NULL;
			vicu_buf[c + 3] = b;
			if (b) {
				d = c == 1 ? 0x10000 : 0;
				P32(b, 0x38) = now >> 32;
				P32(b, 0x3c) = (u32)now;
				stored = rd(0x37008 - d) != rd(0x37084 - d);
				P32(b, 0x2c) = stored;
				ib_put_buf(1, stored ? IB_BPU : IB_FREE, b);
			}
			go = true;
			C32(CM_BPU_GO) = bit;
		}
		v = CM_VICU(c);
		C32(v + 0x40) = 0xff;
		if (now > *(u64 *)CP(v + 0x68)) {
			u64 dt = now - *(u64 *)CP(v + 0x68);

			if (dt >> 32)
				dt = 0xffffffff;
			if (dt)
				C32(v + 0x40) = (100000000 / (u32)dt + 5) / 10;
		}
		*(u64 *)CP(v + 0x70) = *(u64 *)CP(v + 0x68);
		*(u64 *)CP(v + 0x68) = now;
		C32(v + 0x44) = C32(v + 0x44) + 1 > 0xffff ? 0 : C32(v + 0x44) + 1;
		C32(v + 0x30) = c;
		C32(v + 0x34) = 1;
		*(u64 *)CP(v + 0x60) = now;
		*(u64 *)CP(v + 0x78) = now;
		C32(CM_VSYNC(c)) |= bit;
		wake_up(&vicu_wq[c]);
		if (go)
			wake_up(&bpu_thread_wq);
	}
	return IRQ_HANDLED;
}

/* offline (mode=1): the next frame's store into an ISP buffer */
irqreturn_t vicu_irq_1(int irq, void *dev)
{
	u32 mask = 0, c, c2, v, reg, d, sz, w, h, s;
	void *b;
	u64 now;

	if (rd(0x3605c)) {
		if (rd(0x36008))
			mask = BIT(vicu_chn_ext_switch(1, 0));
		wr(0x3605c, 0);
	}
	if (rd(0x2605c)) {
		if (rd(0x26008))
			mask |= 2;
		wr(0x2605c, 0);
	}
	if (!mask || !C32(CM_RUN))
		return IRQ_HANDLED;
	clk_get_pts(&now);
	for (c = 0; c < N_CHN; c++) {
		if (!(mask & BIT(c)) || !C8(CM_BPU_RUNNING) ||
		    C8(CM_BPU_MODE) != 3)
			continue;
		c2 = c;
		if (C8(CM_CHNNUM) > 2)
			c2 = c == 0 ? 2 : c != 2;
		if (!(C8(CM_DUMP) & 1) && !vicu_buf[c2 + 6]) {
			v = CM_VICU(c2);
			reg = c2 == 1 ? 0x27000 : 0x37000;
			d = c2 == 1 ? 0x10000 : 0;
			b = NULL;
			if (C32(v + 0x90)) {
				if (!ib_get_buf(c2, IB_FREE, C32(v + 0x8c),
						&vicu_buf[c2 + 6], 0))
					C32(v + 0x5c)++;
				else
					C32(v + 0x58)++;
				b = vicu_buf[c2 + 6];
			}
			if (b) {
				sz = C32(CM_CHN(c2) + 8);
				w = sz & 0xffff;
				h = sz >> 16;
				s = (w + 5) / 6;
				wr(0x37008 - d, (P32(b, 0x10) + 0x80000000) >> 3);
				wr(0x37018 - d, s);
				wr(0x3701c - d, s);
				wr(0x37020 - d, h * s);
				wr(0x37004 - d, 3);
				if (!P32(b, 0x10)) {
					pr_err("IK:Err: %s %d~ch%d\n",
					       "vicu_irq_1", 0x222, c2);
					wr(reg, 0);
				} else {
					wr(reg, 3);
				}
				P32(b, 0x18) = w;
				P32(b, 0x40) = (u32)now;
				P32(b, 0x44) = now >> 32;
				P32(b, 0x1c) = h;
				P32(b, 0x20) = 3;
				P32(b, 0x24) = c2;
				P32(b, 0x28) = c2;
				P32(b, 0x30) = 0;
				P32(b, 0x2c) = 0;
			} else {
				wr(reg, 0);
			}
		}
		bpu_offline_deal(c);
	}
	return IRQ_HANDLED;
}
