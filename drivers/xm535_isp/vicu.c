// SPDX-License-Identifier: GPL-2.0
/*
 * The VICUs (the 0x30040000 and 0x30050000 windows): the frame interrupt
 * (IRQ 87) writes the channel's VICU block to its registers, takes the AWB
 * statistics and wakes the BPU thread; offline, the store interrupt (IRQ
 * 88) points the VICU at the next ISP buffer and queues the stored one for
 * the BPU (bpu_offline_deal). Interrupts hand the second VICU's work to
 * tasklets.
 *
 * With three sensors (CM_NSNS 3, CM_VICU_MODE 2) the second VICU's
 * receiver alternates between channels 0 and 1 (CM_TS_CUR), and timer 4/5
 * (IRQ 38) runs the frame work once the switched sensor's frame is in.
 *
 * A channel's VICU state (CM_VICU) holds, at: 0x00 its VICU, 0x04/0x08
 * from 0x490e, 0x1c/0x20 its frame lengths, 0x24.. 0x4980's window, 0x50..
 * the frame rate, 0x78/0x7c buffers missed and got, 0x98 the buffer size,
 * 0x9c the flags (bit 0 store, bit 4 on).
 */

#include <linux/kernel.h>
#include <linux/sched.h>

#include "isp.h"

#define VI(c) CM_VICU(c)
#define T(c) CM_TIME(c)

static void *stored[N_CHN]; /* the last stored buffer of a channel */
static void *cur[N_CHN];    /* the buffer being stored */
static u8 ts_skip;          /* frames the time-shared VICU drops */
static u8 ts_pend;          /* timer 4/5's work: bit 0 VICU 0, bit 1 VICU 1 */
static u8 dump_state;

static bool ts_mode(void)
{
	return C8(CM_MODE) == 1 && C8(CM_VICU_MODE) == 2;
}

/* the channel of VICU v (1) or of chn; with the time-shared VICU,
 * next picks the sensor after the current one */
static u32 chn_of(int v, int chn, int next)
{
	if (C8(CM_MODE) && C8(CM_NSNS) >= 3) {
		if (v != 1 && chn != 2)
			return next ? (C8(CM_TS_CUR) + 1) & 1 : C8(CM_TS_CUR);
		return 2;
	}
	return v == 1 || chn == 1;
}

static u32 phy_ofs(u32 c, u32 ofs)
{
	return C8(VI(c)) ? ofs : 0;
}

static void timer45_start(int on)
{
	timer_write_reg(0x100d1008, on ? 0xe3 : 0x62);
}

static void crtline(u32 c, int type, u32 v)
{
	u32 o = CM_LINES + c * 4;

	if (c >= N_CHN)
		return;
	switch (type) {
	case 0:
		C32(o) = v;
		if (v > C32(o + 0xc))
			C32(o + 0xc) = v;
		if (v < C32(o + 0x18))
			C32(o + 0x18) = v;
		break;
	case 1:
		C32(o + 0x24) = v;
		if (v > C32(o + 0x30))
			C32(o + 0x30) = v;
		if (v < C32(o + 0x3c))
			C32(o + 0x3c) = v;
		break;
	case 2:
		C32(o + 0xc) = 0;
		C32(o + 0x30) = 0;
		C32(o + 0x18) = 0xffffffff;
		C32(o + 0x3c) = 0xffffffff;
		break;
	}
}

void vicu_cmos_if(u32 chn, const void *w)
{
	u32 o = C8(VI(chn & 0xff)) ? 0x10000 : 0;

	if (!W8(w, 0))
		return;
	isp_write_reg(o + 0x30040098, W16(w, 2));
	isp_write_reg(o + 0x3004009c, W16(w, 4));
	isp_write_reg(o + 0x300400a0, W16(w, 6));
	isp_write_reg(o + 0x300400a4, W16(w, 8));
	isp_write_reg(o + 0x300400c0, W32(w, 0x10));
	isp_write_reg(o + 0x300400c4, W32(w, 0x14));
	isp_write_reg(o + 0x30040008, W32(w, 0xc));
}

/* 0x490e's 76 bytes: the channels of a[1]'s VICU mask */
int vicu_chn_init(const u8 *a)
{
	u32 c, v, wh, r44, r14, r10, r0c, r18, off, off2;

	for (c = 0; c < N_CHN; c++) {
		C32(CM_VB(c)) = 0;
		if (a[1] >> c & 1) {
			v = VI(c);
			C8(v) = a[8];
			C32(v + 8) = a[0];
			C32(v + 4) = a[7];
			C32(v + 0x1c) = *(const u32 *)(a + 0x34);
			C32(v + 0x20) = *(const u32 *)(a + 0x38);
			wh = *(const u32 *)(a + 0x1c);
			C32(v + 0x98) = ((wh & 0xffff) + 9) / 10 * (wh >> 16) * 0x10;
			if (!C8(CM_CHNNUM)) {
				pr_err("IspKo:Err: not config vicu num\n");
			} else if (CS8(CM_OPT) >= 0) {
				if (c == 1)
					isp_write_reg(0x30001000,
						      isp_read_reg(0x30001000) | 2);
				off = phy_ofs(c, 0x1000);
				off2 = phy_ofs(c, 0x10000);
				r44 = off2 + 0x30048044;
				r18 = off2 + 0x30047018;
				r0c = off2 + 0x3004700c;
				r10 = off2 + 0x30047010;
				r14 = off2 + 0x30047014;
				isp_write_reg(r18, 0);
				isp_write_reg(r0c, 0);
				isp_write_reg(r10, 0xfff);
				isp_write_reg(r14, 0x100);
				C32(v + 0x9c) = 0x11;
				if (C8(CM_MODE) == 1) {
					isp_write_reg(off + 0x30010048, 0);
					isp_write_reg(off + 0x30010038, 0);
					isp_write_reg(off + 0x30010040, 0);
					isp_write_reg(off + 0x30010054, 3);
					isp_write_reg(r44, 2);
					isp_write_reg(off2 + 0x30048054,
						      *(const u16 *)(a + 0x12) +
						      *(const u16 *)(a + 0xe) + 1);
					if (C8(CM_VICU_MODE) == 2) {
						if (!off2)
							goto next;
						isp_write_reg(r44, C16(CM_TS_LINES));
						isp_write_reg(off2 + 0x30048054,
							      C16(CM_TS_LINES + 2));
					}
					isp_write_reg(off2 + 0x30048040, 1);
					isp_write_reg(off2 + 0x30048050, 1);
				} else {
					isp_write_reg(r44, 1);
					isp_write_reg(off2 + 0x30048040, 1);
					isp_write_reg(0x30058044, 1);
					isp_write_reg(0x30058040, 1);
				}
			}
		}
next:
		C32(CM_LINES + c * 4) = 0;
		C32(CM_LINES + 0xc + c * 4) = 0;
		C32(CM_LINES + 0x24 + c * 4) = 0;
		C32(CM_LINES + 0x30 + c * 4) = 0;
		C32(CM_LINES + 0x18 + c * 4) = 0xffffffff;
		C32(CM_LINES + 0x3c + c * 4) = 0xffffffff;
	}
	return 0;
}

/* 0x4914 */
int vicu_chn_enable(u32 chn, int en)
{
	if (chn >= N_CHN)
		return -1;
	if (en)
		C32(VI(chn) + 0x9c) |= 0x10;
	else
		C32(VI(chn) + 0x9c) &= ~0x10;
	return 0;
}

/* 0x495e: the channel's VICUs started */
int vicu_run(u32 chn)
{
	u32 m, k;

	if (chn >= N_CHN)
		return -1;
	m = C32(CM_GRP(chn) + 8);
	for (k = 0; k < N_CHN; k++) {
		if (!(m >> k & 1))
			continue;
		if (k) {
			isp_write_reg(0x30058040, 1);
			if (ts_mode() || C8(CM_MODE) == 1)
				isp_write_reg(0x30058050, 1);
		} else if (!ts_mode()) {
			isp_write_reg(0x30048040, 1);
			if (C8(CM_MODE) == 1)
				isp_write_reg(0x30048050, 1);
		}
	}
	return 0;
}

void vicu_reset(void)
{
	u32 i, o;

	for (i = 0; i < N_CHN; i++) {
		o = i ? 0x10000 : 0;
		isp_write_reg(o + 0x300400c0, 0x110050);
		isp_write_reg(o + 0x300400c4, 1);
		isp_write_reg(o + 0x30048040, 0);
		isp_write_reg(o + 0x30048050, 0);
		isp_write_reg(C8(VI(i)) ? 0x30011034 : 0x30010034, 0);
		stored[i] = NULL;
		cur[i] = NULL;
		isp_write_reg(o + 0x30043004, 0xffff);
		memset(CP(CM_VB(i) + 0x2f8), 0, 0x18);
	}
	ts_pend = 0;
	ts_skip = 0;
	C8(CM_VICU_HELD) = 0;
	C8(CM_TS_CUR) = 0;
}

/* the time-shared VICU's receiver to sensor sel */
static void ts_switch(u32 sel)
{
	if (!C8(CM_MODE) || C8(CM_NSNS) < 3)
		return;
	sel &= 1;
	isp_write_reg(0x30048050, 0);
	isp_write_reg(0x30000000, (isp_read_reg(0x30000000) & 0xc) | (sel + 2));
	isp_write_reg(0x3004805c, 0);
	if (C8(CM_VICU_MODE) != 2)
		isp_write_reg(0x30048050, 1);
	C8(CM_TS_CUR) = sel;
	vicu_cmos_if(sel, CP(CM_VB(sel) + 0x2f8));
}

/*
 * VICU v's store done (offline): the next buffer to store into, and the
 * stored one queued for the BPU. line is the time-shared VICU's line count.
 */
static void vicu_stored_in(u32 v, int lock, u32 line)
{
	u32 v1 = v != 0, c, cn, o, reg, wh, w, h, n, l, e;
	unsigned long flags = 0;
	bool skip0;
	void *b;
	u64 now;

	clk_get_pts(&now);
	c = chn_of(v1, 0xff, 0);
	cn = chn_of(v1, 0xff, 1);
	if (lock)
		ISP_LOCK(flags);
	if (!(C8(CM_DUMP) & 1) && !cur[c]) {
		o = phy_ofs(c, 0x1000);
		reg = o + 0x30010034;
		if (!(C32(VI(c) + 0x9c) & 1)) {
			isp_write_reg(reg, 0);
		} else {
			if (ib_get_buf(c, 0, C32(VI(c) + 0x98), &cur[c], 0) == 0)
				C32(VI(c) + 0x7c)++;
			else
				C32(VI(c) + 0x78)++;
			b = cur[c];
			if (b) {
				wh = C32(CM_CHN + c * 0x28 + 8);
				w = wh & 0xffff;
				h = wh >> 16;
				n = (w + 9) / 10;
				isp_write_reg(o + 0x30010000,
					      (W32(b, 0x10) + 0x80000000) >> 4);
				isp_write_reg(o + 0x30010080,
					      (n * (h - 1) * 0x10 + W32(b, 0x10) -
					       0x80000000) >> 4);
				isp_write_reg(o + 0x30010020, n);
				isp_write_reg(o + 0x30010024, n);
				isp_write_reg(o + 0x30010058, (h + 1) * n);
				isp_write_reg(o + 0x30010054, 3);
				isp_write_reg(reg, 1);
				W32(b, 0x1c) = h;
				W32(b, 0x18) = w;
				W32(b, 0x24) = c;
				W32(b, 0x28) = c;
				W32(b, 0x40) = now;
				W32(b, 0x44) = now >> 32;
				W32(b, 0x20) = 3; /* to the BPU */
				W32(b, 0x30) = 0;
				W32(b, 0x2c) = 0;
			} else {
				isp_write_reg(reg, 0);
			}
		}
	}
	l = isp_read_reg(v1 * 0x10000 + 0x30048080);
	if (l > 99998)
		l = 99999;
	C32(T(c) + 0x74) = l;
	if (l > C32(T(c) + 0x78))
		C32(T(c) + 0x78) = l;
	if (C8(CM_NSNS) >= 3 && c <= 1) {
		/* the time-shared VICU: a frame from the wrong sensor */
		skip0 = !ts_skip;
		if (!skip0)
			ts_skip--;
		if (C8(CM_VICU_MODE) == 2) {
			if (c == 1) {
				e = C32(CM_TS_LINES) & 0xffff;
				if (line < e || line > e + 0xf)
					goto bad;
			} else if (line < C32(CM_TS_LINES) >> 16) {
				goto bad;
			}
			if (skip0)
				goto out;
			goto drop;
		}
		e = isp_read_reg(0x30048054) + 0xf;
		if (C32(T(c) + 0x74) < e && e >> 1 <= C32(T(c) + 0x74) && skip0)
			goto out;
		goto drop;
bad:
		ts_skip = 1;
drop:
		cur[c] = NULL;
		C32(T(c) + 0x70)++;
	}
out:
	if (stored[cn])
		bpu_offline_deal(cn);
	if (lock)
		ISP_UNLOCK(flags);
}

void vicu_stored(unsigned long v)
{
	vicu_stored_in(v, 0, 0);
}

/* the VICU block to the registers of channel c's VICU */
static void vicu_frame_regs(u32 c, u8 mode)
{
	u32 vb = CM_VB(c), base, pb, r, g, b;
	int i, k;

	base = phy_ofs(c, 0x10000) + 0x30042000;
	for (i = 0; i < 15; i++)
		isp_write_reg(base + i * 4, C16(vb + 0xe + i * 2));
	for (i = 0; i < 61; i++) {
		isp_write_reg(base + 0x40 + i * 4, C16(vb + 0x2c + i * 2));
		isp_write_reg(base + 0x140 + i * 4, C16(vb + 0xa6 + i * 2));
		isp_write_reg(base + 0x240 + i * 4, C16(vb + 0x120 + i * 2));
		isp_write_reg(base + 0x340 + i * 4, C16(vb + 0x19a + i * 2));
	}
	base = phy_ofs(c, 0x10000) + 0x30047000;
	isp_write_reg(base, C16(vb + 4));
	isp_write_reg(base + 4, C16(vb + 6));
	r = C16(vb + 0x214);
	g = C16(vb + 0x216);
	b = C16(vb + 0x218);
	if (mode == 1) {
		pb = phy_ofs(c, 0x10000) + 0x30040000;
		isp_write_reg(pb + 0x3000, C32(vb + 0x21c));
		k = 1;
	} else {
		pb = 0x30040000;
		isp_write_reg(0x30043000, C32(vb + 0x21c));
		isp_write_reg(0x30053000, C32(vb + 0x21c));
		isp_write_reg(0x30043004, 0xffff);
		isp_write_reg(0x30053004, 0xffff);
		k = 2; /* online: both VICUs */
	}
	for (; k; k--, pb += 0x10000) {
		isp_write_reg(pb + 0x1000, r);
		isp_write_reg(pb + 0x1004, g);
		isp_write_reg(pb + 0x1008, b);
		isp_write_reg(pb + 0x4000, C32(vb + 0x220));
		isp_write_reg(pb + 0x4004, C32(vb + 0x224));
		isp_write_reg(pb + 0x4008, C32(vb + 0x228));
		for (i = 0; i < 33; i++)
			isp_write_reg(pb + 0x5000 + i * 4, C32(vb + 0x230 + i * 4));
		for (i = 0; i < 17; i++)
			isp_write_reg(pb + 0x50c0 + i * 4, C32(vb + 0x2b4 + i * 4));
	}
	if (mode == 1 && (c == 2 || C8(CM_NSNS) < 3))
		vicu_cmos_if(c, CP(vb + 0x2f8));
}

/* a raw frame into the dump buffer (0x495a), the frame after it was asked */
static void vicu_dump(u32 c)
{
	u32 o = phy_ofs(c, 0x1000), n;
	u8 res;

	if (c != C8(CM_DUMP_CHN))
		return;
	switch (dump_state) {
	case 0:
		isp_write_reg(o + 0x30010048, 0);
		isp_write_reg(o + 0x30010000, (C32(CM_DUMP_ADDR) + 0x80000000) >> 4);
		n = C16(CM_DUMP_W) / 10;
		isp_write_reg(o + 0x30010020, n);
		isp_write_reg(o + 0x30010024, n);
		isp_write_reg(o + 0x30010058, n * (C16(CM_DUMP_H) + 1));
		isp_write_reg(o + 0x30010034, 1);
		isp_write_reg(0x30000014, C32(CM_DUMP_SIZE));
		dump_state = 1;
		return;
	case 1:
		if (isp_read_reg(o + 0x30010100) != isp_read_reg(o + 0x30010108)) {
			dump_state = 2;
			isp_write_reg(o + 0x30010034, 0);
			return;
		}
		isp_write_reg(o + 0x30010034, 0);
		res = 0x20;
		break;
	case 2:
		res = 0x10;
		break;
	default:
		return;
	}
	C8(CM_DUMP) = res;
	dump_state = 0;
	isp_write_reg(0x30000014, 0);
	wake_up(&dump_wq);
}

/* VICU v's frame interrupt */
static void vicu_frame_in(u32 v, int lock)
{
	u32 c, k, o, mask, d, n;
	unsigned long flags = 0;
	u8 mode = C8(CM_MODE);
	bool wake = false;
	void *s;
	u64 now;

	clk_get_pts(&now);
	c = chn_of(v, 0xff, 1);
	if (lock)
		ISP_LOCK(flags);
	mask = 1 << c;
	if (!C32(CM_IW_WAIT)) {
		C8(CM_VICU_HELD) = 0;
		vicu_frame_regs(c, mode);
	} else if (!C8(CM_VICU_HELD)) {
		if (mask & C32(CM_IW_WAIT))
			C8(CM_VICU_HELD) = 1;
		vicu_frame_regs(c, mode);
	}
	if (C8(CM_DUMP) & 1) {
		vicu_dump(c);
	} else {
		if (mode == 1) {
			k = chn_of(0xff, c, 0);
			s = cur[k];
			cur[k] = NULL;
			stored[k] = s;
			if (s) {
				o = phy_ofs(k, 0x1000);
				W32(s, 0x38) = now;
				W32(s, 0x34) = now >> 32;
				if (isp_read_reg(o + 0x30010100) ==
				    isp_read_reg(o + 0x30010108)) {
					W32(s, 0x2c) = 0;
					ib_put_buf(1, 0, s);
				} else {
					W32(s, 0x2c) = 1;
					ib_put_buf(1, 0x10, s);
				}
			}
			awb_stat_get(0, c);
		} else if (bpu_post_deal(c, mode)) {
			goto stats;
		}
		C32(CM_BPU_GO) |= mask;
		wake = true;
	}
stats:
	o = VI(c);
	if (now <= PTS(o + 0x90)) {
		d = 0xff;
	} else {
		d = now - PTS(o + 0x90) > 0xffffffff ? 0xffffffff : now - PTS(o + 0x90);
		d = d ? (100000000 / d + 5) / 10 : 0xff;
	}
	C32(o + 0x58) = d;
	PTS(o + 0x90) = now;
	n = C32(o + 0x5c);
	C32(o + 0x5c) = n == 0xfffffffe ? 0 : n + 1;
	C32(o + 0x50) = c;
	C32(o + 0x54) = 1;
	C32(CM_VSYNC_CNT) |= mask;
	C32(CM_VB(c)) = 1;
	wake_up(&vsync_wq);
	if (wake)
		wake_up(&bpu_thread_wq);
	if (lock)
		ISP_UNLOCK(flags);
}

void vicu_frame(unsigned long v)
{
	vicu_frame_in(v, 0);
}

irqreturn_t timer45_interrupt(int irq, void *dev)
{
	u32 line;
	u8 p;

	if (!timer_read_reg(0x100d1014))
		return IRQ_NONE;
	timer_write_reg(0x100d100c, 1);
	timer45_start(0);
	p = ts_pend;
	ts_pend = 0;
	line = isp_read_reg(0x30058080);
	if (p & 1) {
		vicu_frame_in(0, 1);
		crtline(chn_of(0, 0xff, 0), 0, line);
	}
	if (p & 2) {
		isp_irq0_tasklet.data = 1;
		tasklet_schedule(&isp_irq0_tasklet);
		crtline(2, 0, line);
	}
	return IRQ_HANDLED;
}

static bool vicu_live(void)
{
	return C32(CM_RUN) == 1 && C8(CM_BPU_BUSY);
}

irqreturn_t vicu_irq_0(int irq, void *dev)
{
	u32 st, v, line;
	u64 now;

	if (!ts_mode()) {
		if (isp_read_reg(0x3004804c))
			v = 0;
		else if (isp_read_reg(0x3005804c))
			v = 1;
		else
			return IRQ_HANDLED;
		st = isp_read_reg(v * 0x10000 + 0x30048008);
		isp_write_reg(v * 0x10000 + 0x3004804c, 0);
		if (!st || !vicu_live())
			return IRQ_HANDLED;
		if (C8(CM_MODE) && !v) {
			vicu_frame_in(0, 1);
		} else {
			isp_irq0_tasklet.data = v;
			tasklet_schedule(&isp_irq0_tasklet);
		}
		return IRQ_HANDLED;
	}
	/* the time-shared VICU: sensor 1's frame in, sensor 0 next */
	if (isp_read_reg(0x3005804c)) {
		isp_write_reg(0x3005804c, 0);
		clk_get_pts(&now);
		if (isp_read_reg(0x30058008)) {
			ts_switch(1);
			timer45_start(1);
			ts_pend |= 1;
			if (vicu_live()) {
				line = isp_read_reg(0x30058080);
				vicu_stored_in(0, 1, line);
				crtline(0, 1, line);
				isp_irq1_tasklet.data = 1;
				tasklet_schedule(&isp_irq1_tasklet);
				crtline(2, 1, line);
				return IRQ_HANDLED;
			}
		}
	}
	pr_err("IspKo:Err: ~~~~~~~~~~~~~~~~~~~~~~~ %s %d\n", "vicu_irq_0_mode2",
	       0x3f8);
	return IRQ_HANDLED;
}

irqreturn_t vicu_irq_1(int irq, void *dev)
{
	u32 line;
	bool v1;
	u64 now;

	if (ts_mode()) {
		if (isp_read_reg(0x3005805c)) {
			isp_write_reg(0x3005805c, 0);
			clk_get_pts(&now);
			if (isp_read_reg(0x30058008)) {
				ts_switch(0);
				timer45_start(1);
				ts_pend |= 3;
				if (vicu_live() && C8(CM_MODE) == 1) {
					line = isp_read_reg(0x30058080);
					vicu_stored_in(0, 1, line);
					crtline(1, 1, line);
					return IRQ_HANDLED;
				}
			}
		}
		pr_err("IspKo:Err: ~~~~~~~~~~~~~~~~~~~~~~~ %s %d\n",
		       "vicu_irq_1_mode2", 0x43e);
		return IRQ_HANDLED;
	}
	if (isp_read_reg(0x3004805c)) {
		if (!isp_read_reg(0x30048008)) {
			isp_write_reg(0x3004805c, 0);
			return IRQ_HANDLED;
		}
		ts_switch(C8(CM_TS_CUR) + 1);
		isp_write_reg(0x3004805c, 0);
		v1 = false;
	} else if (isp_read_reg(0x3005805c)) {
		if (!isp_read_reg(0x30058008)) {
			isp_write_reg(0x3005805c, 0);
			return IRQ_HANDLED;
		}
		isp_write_reg(0x3005805c, 0);
		v1 = true;
	} else {
		return IRQ_HANDLED;
	}
	if (!vicu_live() || C8(CM_MODE) != 1)
		return IRQ_HANDLED;
	if (v1) {
		isp_irq1_tasklet.data = 1;
		tasklet_schedule(&isp_irq1_tasklet);
	} else {
		vicu_stored_in(0, 1, 0);
	}
	return IRQ_HANDLED;
}
