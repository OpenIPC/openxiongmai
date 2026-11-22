// SPDX-License-Identifier: GPL-2.0
/*
 * The BPU (Bayer processing): its defaults, the statistics it leaves (AWB,
 * histogram), and how a frame gets to it. The control is the XM530V200
 * isp.ko's, the BPU and its parameter block the XM535D1/D2's.
 *
 * Online (CM_MODE 0/2) the VICU feeds it directly; the thread writes
 * channel 0's parameter block (iw.c) once a frame, woken by the VICU's
 * frame tasklet. Offline (CM_MODE 1) the VICU stores raw frames in ISP
 * buffers (ib.c) and the BPU reads them back:
 *  - CM_OPT & 3 < 2: the thread takes a stored frame, writes the block,
 *    starts the BPU and waits for it;
 *  - otherwise (the memory saver) bpu_offline_deal, from the VICU's store
 *    interrupt and the BPU's own, starts the next pending channel from a
 *    snapshot of its block that the thread takes when woken.
 * With CM_MEMSAVE the BPU's output goes into a VB buffer, else straight
 * into vi.ko's (vpss_thread_*).
 *
 * A channel's group (CM_GRP) holds, at:
 *   0x00 on, 0x04 enabled (0x4958), 0x08 its VICUs, 0x0c a snapshot taken,
 *   0x10/0x14 the bind (id, chn) of its output, 0x28 the raw bits, 0x2c the
 *   WDR mode, 0x44 the frames, 0x48 the late ones, 0x4c/0x50 the input
 *   size, 0x54.. the crop (on, x, y, w, h), 0x68 the parameter block, 0x664
 *   the histogram, 0x7d4 the block's snapshot.
 */

#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/sched.h>
#include <linux/spinlock.h>

#include "isp.h"

#define ST(c) (CM_BPUSTAT + (c) * 0x58) /* the channel's statistics */
#define VI(c) (CM_VICU + (c) * 0xd0)

static u8 held;     /* the parameters waited for (CM_IW_WAIT) and set */
static u8 nr_on;    /* the channels with the 3D noise reduction on */
static u32 irq_cnt; /* BPU frames, for the frame rate in CM_BPUALL */

static void inc16(u32 o)
{
	if (C32(o) < 0xffff)
		C32(o)++;
}

/* now - then, 10 s when then is not before now */
static u64 elapsed(u64 now, u64 then)
{
	return now > then ? now - then : 10000000;
}

static void set_max(u32 o, u64 v)
{
	if (v > PTS(o))
		PTS(o) = v;
}

void bpu_reset(void)
{
	isp_write_reg(0x32000020, 0);
	isp_write_reg(0x32001040, 0);
	isp_write_reg(0x32080034, 0);
	isp_write_reg(0x32001040, 0);
	isp_write_reg(0x32002040, 0);
	isp_write_reg(0x32003040, 0);
	isp_write_reg(0x32022068, 0);
	irq_cnt = 0;
	held = 0;
}

static const u16 nr_curve[33] = {
	0x28, 0x2d, 0x32, 0x37, 0x3c, 0x41, 0x46, 0x4b, 0x50, 0x55, 0x5a,
	0x5f, 0x64, 0x69, 0x6e, 0x73, 0x78, 0x7d, 0x7d, 0x7d, 0x7d, 0x7d,
	0x7d, 0x7d, 0x7d, 0x7d, 0x7d, 0x82, 0x8c, 0x96, 0xa0, 0x96, 0x8c,
};

static const u32 bpu_defaults1[][2] = {
	{ 0x32080048, 0 },     { 0x32080040, 0 },     { 0x32080054, 3 },
	{ 0x32000020, 0 },     { 0x30047018, 0 },     { 0x32022034, 0x40 },
	{ 0x32022038, 0xc8 },  { 0x3202203c, 0xa },   { 0x32022040, 0x31 },
	{ 0x32022044, 0x340 }, { 0x32022048, 0x7a },  { 0x3202204c, 0x1d },
};

static const u32 bpu_defaults2[][2] = {
	{ 0x32032000, 0x418 },  { 0x32032004, 0x809 }, { 0x32032008, 0x18f },
	{ 0x3203200c, 0x1da3 }, { 0x32032010, 0x1b5d }, { 0x32032014, 0x700 },
	{ 0x32032018, 0x700 },  { 0x3203201c, 0x1a23 }, { 0x32032020, 0x1edd },
	{ 0x32032024, 0x10 },   { 0x32032028, 0x80 },  { 0x3203202c, 0x80 },
	{ 0x32033000, 1 },      { 0x32033004, 0xe6 },  { 0x32033008, 0xe6 },
	{ 0x3203300c, 0xe6 },   { 0x32033010, 0xe6 },  { 0x320f0004, 3 },
	{ 0x320f0008, 1 },      { 0x32012000, 0x100 }, { 0x32012004, 0xffff },
	{ 0x32021090, 0xdac },  { 0x32021094, 0x9c4 }, { 0x32021098, 0x1f4 },
	{ 0x3202109c, 0xdac },  { 0x320210a0, 0x54 },  { 0x320210a4, 0x10 },
	{ 0x32025000, 1 },      { 0x32025004, 2 },     { 0x32025008, 0 },
	{ 0x32025010, 0x1ff },  { 0x32025020, 0x52 },  { 0x32025030, 0xb88 },
	{ 0x32000018, 1 },      { 0x32036090, 0 },     { 0x32036094, 0xff },
	{ 0x32036098, 0xc8 },   { 0x3203609c, 0x40 },  { 0x320360a0, 0xc },
	{ 0x320360a4, 0xc },    { 0x320360a8, 0x40 },  { 0x320360ac, 0x1d0 },
	{ 0x320360b0, 0x1d0 },
};

#define WR_CONST(t)                                       \
	do {                                              \
		int _i;                                   \
		for (_i = 0; _i < ARRAY_SIZE(t); _i++)    \
			isp_write_reg(t[_i][0], t[_i][1]); \
	} while (0)

int bpu_init(void)
{
	int i;

	C32(CM_VB_OUT) = 0;
	memset(CP(CM_GRP(0)), 0, 3 * 0xf40);
	if (CS8(CM_OPT) < 0)
		return 0;
	WR_CONST(bpu_defaults1);
	isp_write_reg(0x32100008, 3);
	for (i = 0; i < 33; i++)
		isp_write_reg(0x32011034 + i * 4, nr_curve[i]);
	WR_CONST(bpu_defaults2);
	if (C8(CM_MODE) == 1) {
		isp_write_reg(0x320f0000, 0);
		isp_write_reg(0x32000024, 1);
		isp_write_reg(0x320f0004, 3);
		isp_write_reg(0x32100000, 1);
	} else {
		isp_write_reg(0x32000024, 0);
		isp_write_reg(0x320f0004, 3);
	}
	return 0;
}

/* 0x490e's 52 bytes: chn, VICU mask, .., raw bits, WDR, .., the crop */
int bpu_chn_init(const u8 *a)
{
	u32 c = a[0], g;

	if (c >= N_CHN)
		return -1;
	g = CM_GRP(c);
	C32(g + 8) = a[1];
	C32(g + 4) = 1;
	C32(g + 0) = 1;
	C8(g + 0x28) = a[6];
	C32(g + 0x2c) = a[7];
	C32(g + 0x54) = 1;
	C32(g + 0x58) = *(const u16 *)(a + 0x20);
	C32(g + 0x5c) = *(const u16 *)(a + 0x22);
	C32(g + 0x60) = *(const u16 *)(a + 0x24);
	C32(g + 0x64) = *(const u16 *)(a + 0x26);
	if (C32(g + 0x64) > 0x3ffe && C32(g + 0x60) > 0x3ffe)
		C32(g + 0x54) = 0;
	memcpy(CP(CM_CHN + c * 0x28), a + 8, 0x28);
	return 0;
}

/* 512 zones of R, G, B sums, divided by the VICU block's count (0x496d) */
int awb_stat_get(int wdr, u32 chn)
{
	u32 base, div, r, g, b, o;
	int i;

	if (chn >= N_CHN || wdr)
		return -1;
	div = C32(CM_VB(chn) + 0x22c) ?: 1;
	base = chn ? 0x30055140 : 0x30045140;
	isp_write_reg(base, 1);
	for (i = 0; i < 0x200; i++) {
		isp_write_reg(base + 4, 1);
		r = isp_read_reg(base + 8);
		g = isp_read_reg(base + 0xc);
		b = isp_read_reg(base + 0x10);
		o = CM_AWB_STAT + (chn * 0x600 + i) * 2;
		C16(o) = r / div;
		C16(o + 0x400) = (g / div) >> 1;
		C16(o + 0x800) = b / div;
	}
	isp_write_reg(base, 1);
	return 0;
}

/* the 64-bin histogram: max, mean, mean x bins over the threshold, ... */
static void bpu_hist(u32 chn)
{
	u32 d[64], a, b, max = 0, sa = 0, sd = 0, thr, avg, n = 0, s = 0, o;
	int i;

	for (i = 0; i < 64; i++) {
		a = isp_read_reg(0x32022200 + i * 4);
		b = isp_read_reg(0x32022300 + i * 4);
		sa += a;
		sd += b - a;
		d[i] = b - a;
		if (b > max)
			max = b;
	}
	thr = (sd + 0x20) >> 6;
	avg = (sa + 0x20) >> 6;
	for (i = 0; i < 64; i++) {
		if (d[i] >= thr) {
			s += d[i];
			n++;
		}
	}
	o = CM_GRP(chn);
	C32(o + 0x664) = max;
	C32(o + 0x668) = avg;
	C32(o + 0x670) = thr;
	C32(o + 0x66c) = avg * n;
	C32(o + 0x674) = s;
	C32(o + 0x75c) = 0xf0e;
	C32(o + 0x760) = 0;
	C32(o + 0x764) = 0;
	C32(o + 0x768) = 0;
	C32(o + 0x76c) = 0;
	C32(o + 0x770) = isp_read_reg(0x320360c8);
}

int bpu_post_deal(u32 chn, int mode)
{
	if (mode == 2) {
		awb_stat_get(0, 0);
		awb_stat_get(0, 1);
		chn = 0;
	} else if (mode != 1) {
		awb_stat_get(0, chn);
	}
	if (chn >= N_CHN)
		return -1;
	bpu_hist(chn);
	return 0;
}

int bpu_set_wdr_mode(u32 chn, u32 mode)
{
	unsigned long f0, f1;
	u32 m;

	if (chn >= N_CHN)
		return -1;
	m = C32(CM_GRP(chn) + 8);
	spin_lock_irqsave(&awb_lock[0], f0);
	spin_lock_irqsave(&awb_lock[1], f1);
	C32(CM_GRP(chn) + 0x2c) = mode;
	if (m & 1)
		C32(VI(0)) = mode;
	if (m & 2)
		C32(VI(1)) = mode;
	if (m & 4)
		C32(VI(2)) = mode;
	spin_unlock_irqrestore(&awb_lock[1], f1);
	spin_unlock_irqrestore(&awb_lock[0], f0);
	return 0;
}

int bpu_set_enable(u32 chn, int en)
{
	u8 old;

	if (chn >= N_CHN)
		return -1;
	old = C8(CM_GRP(chn) + 4);
	C32(CM_GRP(chn) + 4) = en != 0;
	return old != 0;
}

/* {register, offset in the block, how} */
enum { R32, R16, RS16, R8, RK };
struct breg {
	u32 reg;
	u16 off;
	u8 how;
};

static void bpu_write(const struct breg *t, int n, const u8 *b)
{
	u32 v;
	int i;

	for (i = 0; i < n; i++) {
		switch (t[i].how) {
		case R32:
			v = W32(b, t[i].off);
			break;
		case R16:
			v = W16(b, t[i].off);
			break;
		case RS16:
			v = WS16(b, t[i].off);
			break;
		case R8:
			v = W8(b, t[i].off);
			break;
		default:
			v = t[i].off;
			break;
		}
		isp_write_reg(t[i].reg, v);
	}
}

#define BW(t, b) bpu_write(t, ARRAY_SIZE(t), b)

static const struct breg bpu_regs_nr[] = {
	{ 0x32011000, 0xa5, R8 },   { 0x32011004, 0xa8, RS16 },
	{ 0x32011008, 0xaa, RS16 }, { 0x3201100c, 0xac, RS16 },
	{ 0x32011010, 0xae, R16 },  { 0x32011014, 0xb0, R16 },
	{ 0x32011018, 0xb2, R16 },  { 0x3201101c, 0xb4, R16 },
	{ 0x32011020, 0xb6, R16 },  { 0x32011024, 0xb8, R16 },
	{ 0x32011028, 0xa6, R8 },   { 0x3201102c, 0xba, R16 },
	{ 0x32011030, 0xbc, R16 },
};

static const struct breg bpu_regs_dpc[] = {
	{ 0x32010004, 0x1f9, R8 }, { 0x32010008, 0x1fa, R8 },
	{ 0x3201000c, 0x1fb, R8 }, { 0x32010010, 0x1fc, R8 },
	{ 0x32010014, 0x1fd, R8 }, { 0x32010018, 0x1fe, R8 },
	{ 0x3201001c, 0x1ff, R8 }, { 0x32010000, 0x1f8, R8 },
	{ 0x32010020, 0x200, R8 }, { 0x32010024, 0x201, R8 },
	{ 0x32010028, 0x202, R8 }, { 0x3201002c, 0x203, R8 },
	{ 0x32010030, 0x204, R8 }, { 0x32010034, 0x206, R16 },
	{ 0x32010038, 0x205, R8 },
};

static const struct breg bpu_regs_nr3d[] = {
	{ 0x320360c0, 0x6e0, R32 }, { 0x32036020, 0x718, R32 },
	{ 0x32036024, 0x71c, R32 }, { 0x32036038, 0x720, R32 },
	{ 0x3203603c, 0x724, R32 }, { 0x32036028, 0x728, R32 },
	{ 0x3203602c, 0x72c, R32 }, { 0x32036030, 0x730, R32 },
	{ 0x32036034, 0x734, R32 }, { 0x32036008, 0x6ec, R32 },
	{ 0x3203600c, 0x6ec, R32 }, { 0x32036010, 0x6f0, R32 },
	{ 0x32036014, 0x6f0, R32 },
};

static const struct breg bpu_regs_sh[] = {
	{ 0x32035004, 0x638, R32 }, { 0x32035008, 0x638, R32 },
	{ 0x32035010, 0x6bc, R32 }, { 0x32035014, 0x6c0, R32 },
	{ 0x32035028, 0x6c4, R32 }, { 0x3203502c, 0x6c8, R32 },
	{ 0x32035018, 0x6cc, R32 }, { 0x3203501c, 0x6d0, R32 },
	{ 0x32035020, 0x6d4, R32 }, { 0x32035024, 0x6d8, R32 },
	{ 0x32035038, 0x624, R32 }, { 0x3203503c, 0x628, R32 },
	{ 0x32035040, 0x62c, R32 }, { 0x320350e0, 0x630, R32 },
	{ 0x320350e4, 0x634, R32 },
};

static const struct breg bpu_regs_win[] = {
	{ 0x32000008, 0x558, R32 }, { 0x3200000c, 0x55c, R32 },
	{ 0x32000010, 0x560, R32 }, { 0x32000014, 0x564, R32 },
	{ 0x32000000, 0x568, R32 }, { 0x32000004, 0x56c, R32 },
	{ 0x32000040, 0x570, R32 },
};

/* offline: the raw frame in, and the YUV frame out */
static const struct breg bpu_regs_offline[] = {
	{ 0x32001000, 0x578, R32 }, { 0x32001004, 0x57c, R32 },
	{ 0x32001008, 0x580, R32 }, { 0x3200100c, 0x584, R32 },
	{ 0x32001040, 0x588, R32 }, { 0x32001028, 0, RK },
	{ 0x32080000, 0x594, R32 }, { 0x32080010, 0x598, R32 },
	{ 0x32080020, 0x59c, R32 }, { 0x32080024, 0x5a0, R32 },
	{ 0x32080028, 0x5a4, R32 }, { 0x3208002c, 0x5a8, R32 },
	{ 0x32080030, 0x5ac, R32 }, { 0x32080038, 0x5b4, R32 },
	{ 0x32080080, 0x5b8, R32 }, { 0x32080090, 0x5bc, R32 },
	{ 0x32080058, 0x5c0, R32 }, { 0x3208005c, 0x5c4, R32 },
	{ 0x32080034, 0x5b0, R32 },
};

/*
 * A parameter block (b, the live one or a snapshot) to the registers.
 * With CM_IW_WAIT the full set goes once, then only what changes per frame
 * until the waited-for parameters are released.
 */
static int bpu_update_regs(u32 chn, int mode, u8 *b)
{
	u32 dx = W32(b, 0x55c) - W32(b, 0x558);
	u32 dy = W32(b, 0x564) - W32(b, 0x560);
	u32 bw, bh, lw, lh, v, bit = 1 << chn;
	int i, on;

	if (C32(CM_IW_WAIT) == 0xff) {
		isp_write_reg(0x32000020, 0);
		isp_write_reg(0x32001040, 0);
		isp_write_reg(0x32080034, 0);
		isp_write_reg(0x32022068, 0);
		return 0;
	}
	if (C32(CM_IW_WAIT) == 0)
		held = 0;
	if (held) {
		isp_write_reg(0x32022060, W32(b, 0x5f8));
		isp_write_reg(0x32022064, W32(b, 0x5f8));
		isp_write_reg(0x32035004, W32(b, 0x638));
		isp_write_reg(0x32035008, W32(b, 0x638));
		isp_write_reg(0x32036008, W32(b, 0x6ec));
		isp_write_reg(0x3203600c, W32(b, 0x6ec));
		isp_write_reg(0x32036010, W32(b, 0x6f0));
		isp_write_reg(0x32036014, W32(b, 0x6f0));
		if (mode)
			isp_write_reg(0x320360c4,
				      max_t(u32, W32(b, 0x708), 0x7ff) | 0x1000);
		goto frame;
	}
	BW(bpu_regs_nr, b);
	for (i = 0; i < 33; i++)
		isp_write_reg(0x32011034 + i * 4, W16(b, 0xbe + i * 2));
	for (i = 0; i < 12; i++)
		isp_write_reg(0x32030040 + i * 4, WS16(b, 0x10c + i * 2));
	for (i = 0; i < 97; i++)
		isp_write_reg(0x32031000 + i * 4, W16(b, 0x124 + i * 2));
	BW(bpu_regs_dpc, b);
	/* the shading grid: 32 x 32 blocks, the last ones what is left */
	bw = (dx + 0x10) >> 5;
	bh = (dy + 0x10) >> 5;
	lw = dx > bw * 31 ? dx - bw * 31 : 0;
	lh = dy > bh * 31 ? dy - bh * 31 : 0;
	if (bw * bh * lw * lh == 0) {
		pr_err("IspKo:Err: %s line%d: ch:%d: size=0[%d %d %d %d]\n",
		       "bpu_update_regs", 0x206, chn, bw, bh, lw, lh);
		return -1;
	}
	W32(b, 0x5c8) = bw - 1;
	W32(b, 0x5cc) = bh - 1;
	W32(b, 0x5d0) = 0x100000 / bw;
	W32(b, 0x5d4) = 0x100000 / bh;
	W32(b, 0x5d8) = 0x100000 / (bw * bh);
	W32(b, 0x5dc) = 0x100000 / (bh * lw);
	W32(b, 0x5e0) = 0x100000 / (bw * lh);
	W32(b, 0x5e4) = 0x100000 / (lw * lh);
	isp_write_reg(0x32022020, W32(b, 0x5ec));
	isp_write_reg(0x32022024, W32(b, 0x5f0));
	isp_write_reg(0x3202202c, W32(b, 0x5f4));
	for (i = 0; i < 8; i++)
		isp_write_reg(0x32022000 + i * 4, W32(b, 0x5c8 + i * 4));
	isp_write_reg(0x32022060, W32(b, 0x5f8));
	isp_write_reg(0x32022064, W32(b, 0x5f8));
	isp_write_reg(0x32022068, W32(b, 0x5e8) != 0);
	/* the 3D noise reduction's grid: the same */
	memcpy(b + 0x718, b + 0x5c8, 8 * 4);
	BW(bpu_regs_nr3d, b);
	on = W32(b, 0x6dc) != 0;
	if (mode == 0) {
		if (on == (nr_on >> chn & 1)) {
			isp_write_reg(0x32036018, on);
			isp_write_reg(0x32036000, on);
		} else {
			isp_write_reg(0x32036004, on);
		}
		nr_on = (nr_on & ~bit) | on << chn;
	} else {
		isp_write_reg(0x320360c4,
			      max_t(u32, W32(b, 0x708), 0x7ff) | 0x1000);
		isp_write_reg(0x32036004, on);
		isp_write_reg(0x32036018, on);
		isp_write_reg(0x32036000, on);
	}
	/* and the sharpening's */
	memcpy(b + 0x6bc, b + 0x718, 8 * 4);
	BW(bpu_regs_sh, b);
	for (i = 0; i < 16; i++) {
		isp_write_reg(0x32035100 + i * 4, W32(b, 0x63c + i * 4));
		isp_write_reg(0x32035140 + i * 4, W32(b, 0x67c + i * 4));
	}
	isp_write_reg(0x3203500c, W32(b, 0x620) != 0);
	isp_write_reg(0x32035000, W32(b, 0x620) != 0);
	for (i = 0; i < 104; i++)
		isp_write_reg(0x32021200 + i * 4, W32(b, 0x28c + i * 4));
	BW(bpu_regs_win, b);
	for (i = 0; i < 3; i++) {
		isp_write_reg(0x32032000 + i * 4, W16(b, 0x738 + i * 2) & 0x1fff);
		isp_write_reg(0x3203200c + i * 4, W16(b, 0x740 + i * 2) & 0x1fff);
		isp_write_reg(0x32032018 + i * 4, W16(b, 0x748 + i * 2) & 0x1fff);
		isp_write_reg(0x32032024 + i * 4, W8(b, 0x73e + i * 8));
	}
	if (W32(b, 0x760) < 0x3fff) {
		isp_write_reg(0x32034000, W32(b, 0x75c));
		isp_write_reg(0x32034004, W32(b, 0x760));
	}
	if (W32(b, 0x768) < 0x3fff) {
		isp_write_reg(0x32034008, W32(b, 0x764));
		isp_write_reg(0x3203400c, W32(b, 0x768));
	}
frame:
	if (mode == 1) {
		BW(bpu_regs_offline, b);
	} else if (!held) {
		v = isp_read_reg(0x30001000);
		isp_write_reg(0x30001000, v | (W32(b, 0x750) & 0xfff) >> 8);
		isp_write_reg(0x30000018, W32(b, 0x750) & 0xf);
		isp_write_reg(0x320f0000, W8(b, 0x754));
		isp_write_reg(0x32100000, (W32(b, 0x754) & 0xfff) >> 8);
	}
	if (C32(CM_IW_WAIT) & bit)
		held = 1;
	return 0;
}

/* the output window (into b, 0x75c..0x768) and size: the crop, else the input */
static void bpu_crop(u32 g, u8 *b, u32 *w, u32 *h)
{
	if (!C32(g + 0x54)) {
		W32(b, 0x75c) = 0;
		W32(b, 0x764) = 0;
		W32(b, 0x760) = 0x3fff;
		W32(b, 0x768) = 0x3fff;
	} else {
		W32(b, 0x75c) = C32(g + 0x58);
		W32(b, 0x760) = C32(g + 0x60) + C32(g + 0x58);
		W32(b, 0x764) = C32(g + 0x5c);
		W32(b, 0x768) = C32(g + 0x64) + C32(g + 0x5c);
	}
	*w = C32(g + 0x60) > 0x3ffe ? C32(g + 0x4c) : C32(g + 0x60);
	*h = C32(g + 0x64) > 0x3ffe ? C32(g + 0x50) : C32(g + 0x64);
}

/*
 * The YUV output, w x h at the stride rounded to 16: into the VB buffer
 * vb, or (vb NULL) at 0 in vi.ko's buffer.
 */
static void bpu_set_out(u8 *b, void *vb, void *ib, u32 w, u32 h)
{
	u32 w16 = (w + 0xf) >> 4, uv = w16 * 0x10 * h, y = 0;

	if (vb) {
		W32(vb, 0x20) = h;
		W32(vb, 0x18) = w16 * 0x10;
		W32(vb, 0x1c) = w;
		W32(vb, 0x28) = W32(ib, 0x40);
		W32(vb, 0x2c) = W32(ib, 0x3c);
		y = W32(vb, 0x10) + 0x80000000;
	}
	W32(b, 0x594) = y >> 4;
	W32(b, 0x598) = (uv + y) >> 4;
	W32(b, 0x59c) = w >> 4;
	W32(b, 0x5a0) = w >> 4;
	W32(b, 0x5a4) = w >> 4;
	W32(b, 0x5a8) = w >> 4;
	W32(b, 0x5ac) = 0;
	W32(b, 0x5b0) = vb != NULL;
	W32(b, 0x5b4) = 0;
	W32(b, 0x5bc) = (w * ((h >> 1) - 1) + uv + y) >> 4;
	W32(b, 0x5b8) = (w * (h - 1) + y) >> 4;
	W32(b, 0x5c4) = ((w16 + 1) >> 1) * h;
	W32(b, 0x5c0) = w16 * h;
}

/*
 * The raw frame in, stored at 10 bits a pixel. The memory saver's line
 * pitch is isp.ko's: 10 bytes per 10 pixels (the VICU stores 16).
 */
static u32 bpu_set_in(u32 chn, u8 *b, void *ib, bool saver)
{
	u32 a = CM_CHN + chn * 0x28, n = (W32(ib, 0x18) + 9) / 10;
	u32 x = C16(a + 0xc), y = C16(a + 0xe), pitch = saver ? n * 10 : n * 0x10;

	W32(b, 0x558) = x;
	W32(b, 0x55c) = C16(a + 0x10) + x;
	W32(b, 0x560) = y;
	W32(b, 0x564) = C16(a + 0x12) + y;
	W32(b, 0x568) = C16(a + 0x14) - 1;
	W32(b, 0x56c) = C16(a + 0x16) - 1;
	W32(b, 0x570) = 1;
	W32(b, 0x574) = 1;
	W32(b, 0x578) = (W32(ib, 0x10) + 0x80000000) >> 4;
	W32(b, 0x57c) = ((W32(ib, 0x1c) - 1) * pitch + W32(ib, 0x10) -
			 0x80000000) >> 4;
	W32(b, 0x580) = n;
	W32(b, 0x584) = n;
	W32(b, 0x588) = 1;
	W32(b, 0x590) = 0;
	if (!saver)
		W32(b, 0x58c) = n * (W32(ib, 0x1c) + 1);
	return n * 10;
}

/* a VB buffer done with: to VPSS (bind id 8) or back to the pool */
static void bpu_vb_done(u32 chn, void *vb, int list)
{
	u32 g = CM_GRP(chn);

	W32(vb, 0x30) = C32(g + 0x10);
	W32(vb, 0x34) = C32(g + 0x14);
	W32(vb, 0x38) = 0;
	vb_put_buf(list < 0 ? C32(g + 0x10) == 8 : list, vb);
}

/* the BPU's clock gate (sys.ko 0x50 bit 10), unless the VICU says not */
static void bpu_clk(bool on)
{
	u32 clk;

	if (isp_read_reg(0x300400c8))
		return;
	sys_get_clk(0x50, &clk);
	sys_set_clk(0x50, on ? clk | 0x400 : clk & ~0x400);
}

/*
 * The memory saver: chn (0..2) has stored a frame, or 0xff the BPU is
 * done with cur. Starts the BPU on the next pending channel, if idle.
 */
int bpu_offline_deal(u32 chn, u32 cur)
{
	u8 mode = C8(CM_MODE);
	void *ib = NULL, *vb = NULL, *old;
	u32 k, bit, p, g, wh, w, h, n10, grp;
	unsigned long flags, f2;
	u64 now, start;
	u8 *b;

	if ((C8(CM_OPT) & 3) < 2)
		return 0;
	if (chn != 0xff && chn >= N_CHN) {
		pr_err("IspKo:Err: %s %d\n", __func__, 0x49f);
		return -1;
	}
	spin_lock_irqsave(&bpu_lock2, flags);
	clk_get_pts(&now);
	if (chn == 0xff) {
		if (C32(CM_BPU_CUR) != cur) {
			pr_err("IspKo:Err: %s %d\n", __func__, 0x4ac);
			if (C32(CM_BPU_CUR))
				goto out;
		} else {
			C32(CM_BPU_CUR) = 0;
		}
	} else {
		bit = 1 << chn;
		if ((C32(CM_BPU_PEND) | C32(CM_BPU_CUR)) & bit &&
		    C32(ST(chn) + 0x24) < 0xffff) {
			/* the last one not taken yet: dropped, or the BPU stuck */
			C32(ST(chn) + 0x24)++;
			start = PTS(VI(chn) + 0x90);
			if (start > now || now - start > 199999) {
				inc16(ST(chn) + 0x28);
				C32(CM_BPU_CUR) = 0;
			}
		}
		C32(CM_BPU_PEND) |= bit;
		if (C32(CM_BPU_CUR))
			goto out;
	}
	p = C32(CM_BPU_PEND);
	if (p & 1)
		k = 0;
	else if (p & 2)
		k = 1;
	else if (p & 4)
		k = 2;
	else
		goto out;
	C32(CM_BPU_CUR) = 1 << k;
	C32(CM_BPU_PEND) = p & ~(1 << k);
	PTS(VI(k) + 0x90) = now;
	ib_get_buf(k, 0x10, 0, &ib, 1000);
	g = CM_GRP(k);
	b = CP(g + GRP_SNAP);
	if (!ib || !C32(g + 0xc))
		goto idle;
	if (k && C32(CM_DVPSEL + 0x2c)) { /* frames VICU 1 drops after a switch */
		C32(CM_DVPSEL + 0x2c)--;
		goto idle;
	}
	if (PTS(VI(k) + 0x90) <= PTS(VI(k) + 0x88))
		inc16(ST(k) + 0x24);
	wh = C32(CM_CHN + k * 0x28 + 0x10);
	if (!C32(g) || !C32(g + 4) || W32(ib, 0x18) > (wh & 0xffff) ||
	    W32(ib, 0x1c) > wh >> 16)
		pr_err("IspKo:Err: %s line[%d]:%d %d %d %d %d %d\n", __func__,
		       0x4fb, C32(g), C32(g + 4), wh & 0xffff, W32(ib, 0x18),
		       wh >> 16, W32(ib, 0x1c));
	C32(g + 0x44)++;
	C32(g + 0x4c) = W32(ib, 0x18);
	C32(g + 0x50) = W32(ib, 0x1c);
	bpu_crop(g, b, &w, &h);
	if (C8(CM_MEMSAVE) == 1) {
		if (vb_get_buf(0, ((w + 0xf) & ~0xf) * h * 3 >> 1, &vb, 0)) {
			C32(ST(k) + 0x10)++;
			goto idle;
		}
		C32(ST(k) + 0x14)++;
	}
	bpu_set_out(b, vb, ib, w, h);
	n10 = bpu_set_in(k, b, ib, true);
	spin_lock_irqsave(&bpu_irq_lock, f2);
	old = (void *)C32(CM_VB_OUT);
	if (old) { /* the BPU never finished the last one */
		bpu_vb_done(k, old, -1);
		C32(CM_VB_OUT) = 0;
		inc16(ST(k) + 0x24);
	}
	C32(CM_VB_OUT) = (u32)vb;
	spin_unlock_irqrestore(&bpu_irq_lock, f2);
	bpu_clk(true);
	bpu_update_regs(k, mode, b);
	if (!C8(CM_MEMSAVE)) {
		grp = C32(g + 0x14);
		wh = C32(CM_CHN + k * 0x28 + 0x20);
		vpss_isp_set_val_v2(grp, wh & 0xffff, wh >> 16, 1);
		vpss_thread_set_val(grp, n10, w, h, W32(ib, 0x40), W32(ib, 0x3c));
		isp_write_reg(0x32000020, W32(b, 0x574));
		vpss_thread_middle_deal(grp, 1);
	} else {
		isp_write_reg(0x32000020, W32(b, 0x574));
	}
	C32(CM_BPU_RUN) |= 1 << k;
	goto done;
idle:
	C32(CM_BPU_CUR) = 0;
done:
	clk_get_pts(&now);
	PTS(VI(k) + 0x98) = now;
	set_max(ST(k) + 0x50, elapsed(now, PTS(VI(k) + 0x90)));
out:
	spin_unlock_irqrestore(&bpu_lock2, flags);
	return 0;
}

irqreturn_t bpu_irq(int irq, void *dev)
{
	irqreturn_t ret = IRQ_NONE;
	u32 run, c, n, s, d, list;
	unsigned long flags;
	u64 now, last;
	void *vb;

	spin_lock_irqsave(&bpu_irq_lock, flags);
	if (!isp_read_reg(0x32000044)) {
		spin_unlock_irqrestore(&bpu_irq_lock, flags);
		goto next;
	}
	isp_write_reg(0x32000044, 0);
	ret = IRQ_HANDLED;
	run = C32(CM_BPU_RUN);
	if (C32(CM_RUN) != 1 || !run)
		goto out;
	clk_get_pts(&now);
	for (c = 0; c < N_CHN; c++) {
		if (!(run >> c & 1))
			continue;
		s = ST(c);
		last = PTS(s + 0x30);
		if (now <= last) {
			d = 0xff;
		} else {
			d = now - last > 0xffffffff ? 0xffffffff : now - last;
			d = (100000000 / d + 5) / 10;
		}
		C32(s + 8) = d;
		PTS(s + 0x30) = now;
		n = C32(s + 0xc) + 1;
		C32(s + 0xc) = n > 0xfffffff ? 0 : n;
		C32(s) = c;
		C32(s + 4) = 1;
		break;
	}
	if (hweight8(run & 7) > 1)
		pr_err("IspKo:Err: %s line[%d]: 0x%x\n", __func__, 0x30b, run);
	if (++irq_cnt > 1000)
		irq_cnt = 1000;
	last = PTS(CM_BPUALL + 0x30);
	if (now > last) {
		d = now - last > 0xffffffff ? 0xffffffff : now - last;
		if (d / 1000 >= 2000) {
			C32(CM_BPUALL + 8) = (irq_cnt * 100000 / (d / 1000) + 5) / 10;
			irq_cnt = 0;
			PTS(CM_BPUALL + 0x30) = now;
		}
	}
	C32(CM_BPUALL + 4) = 1;
	n = C32(CM_BPUALL + 0xc) + 1;
	C32(CM_BPUALL + 0xc) = n > 0xfffffff ? 0 : n;
	C32(CM_BPU_RUN) = 0;
	if ((C8(CM_OPT) & 3) < 2) {
		wake_up(&bpu_thread_wq);
		goto out;
	}
	if (c == N_CHN)
		goto out;
	bpu_post_deal(c, C8(CM_MODE));
	vb = (void *)C32(CM_VB_OUT);
	if (vb) {
		/* the first frames of a memory saver go back to the pool */
		list = -1;
		if (C32(CM_VB_HOLD(c != 0))) {
			C32(CM_VB_HOLD(c != 0))--;
			list = 0;
		}
		bpu_vb_done(c, vb, list);
		C32(CM_VB_OUT) = 0;
	}
	PTS(VI(c) + 0xa0) = now;
	if (!C8(CM_MEMSAVE))
		vpss_thread_post_deal(C32(CM_GRP(c) + 0x14), 1);
	set_max(ST(c) + 0x40, PTS(ST(c) + 0x38) = elapsed(now, PTS(VI(c) + 0x90)));
	last = PTS(VI(c) + 0x88);
	if (now <= last) {
		inc16(CM_GRP(c) + 0x48);
	} else {
		set_max(ST(c) + 0x48, now - last);
		if (now - last > 79000)
			inc16(CM_GRP(c) + 0x48);
	}
out:
	spin_unlock_irqrestore(&bpu_irq_lock, flags);
	bpu_clk(false);
next:
	bpu_offline_deal(0xff, C32(CM_BPU_CUR));
	return ret;
}

/* online: channel 0's block straight to the registers */
static void bpu_online(void)
{
	u32 g = CM_GRP(0), b = g + GRP_BLOCK, a = CM_CHN, wdr, wh;

	mutex_lock(&grp_lock[0]);
	wdr = C32(g + 0x2c);
	if (!C32(g + 0x54)) {
		C32(b + 0x75c) = 0;
		C32(b + 0x764) = 0;
		C32(b + 0x760) = 0x3fff;
		C32(b + 0x768) = 0x3fff;
	} else {
		C32(b + 0x75c) = C32(g + 0x58);
		C32(b + 0x764) = C32(g + 0x5c);
		C32(b + 0x760) = C32(g + 0x60) + C32(g + 0x58);
		C32(b + 0x768) = C32(g + 0x64) + C32(g + 0x5c);
	}
	C32(b + 0x558) = C16(a + 0xc);
	C32(b + 0x55c) = C16(a + 0x10) + C16(a + 0xc);
	C32(b + 0x560) = C16(a + 0xe);
	C32(b + 0x564) = C16(a + 0x12) + C16(a + 0xe);
	C32(b + 0x568) = C16(a + 0x14) - 1;
	C32(b + 0x56c) = C16(a + 0x16) - 1;
	switch (wdr) {
	case 0:
	case 1:
		C32(b + 0x570) = 0;
		C32(b + 0x574) = 0;
		C32(b + 0x588) = 0;
		C32(b + 0x590) = 0;
		C32(b + 0x5ac) = 0;
		C32(b + 0x5b0) = 0;
		C32(b + 0x750) = 0x100;
		C32(b + 0x754) = 1;
		C32(b + 0x758) = wdr ? 0x100 : 0x10;
		break;
	case 2:
		C32(b + 0x570) = 0;
		C32(b + 0x588) = 0;
		C32(b + 0x590) = 0;
		C32(b + 0x5ac) = 0;
		C32(b + 0x574) = 1;
		C32(b + 0x5b0) = 1;
		C32(b + 0x750) = 0x301;
		C32(b + 0x754) = 0x100;
		C32(b + 0x758) = 0x100;
		break;
	default:
		pr_err("IspKo:Err: wdr_mode: %d not support!\n", wdr);
		break;
	}
	bpu_update_regs(0, C8(CM_MODE), CP(b));
	mutex_unlock(&grp_lock[0]);
	wh = C32(a + 0x20);
	vpss_isp_set_val_v2(C32(g + 0x14), wh & 0xffff, wh >> 16, 0);
}

/* offline: a snapshot of each channel's block whose frame came */
static void bpu_snapshot(u32 go)
{
	unsigned long flags;
	u32 c, g;

	for (c = 0; c < N_CHN; c++) {
		if (!((int)go >> c & 1))
			continue;
		g = CM_GRP(c);
		mutex_lock(&grp_lock[c]);
		spin_lock_irqsave(&awb_lock[c], flags);
		C32(CM_BPU_GO) &= ~(1 << c);
		memcpy(CP(g + GRP_SNAP), CP(g + GRP_BLOCK), GRP_BLOCK_SIZE);
		C32(g + 0xc) = 1;
		spin_unlock_irqrestore(&awb_lock[c], flags);
		mutex_unlock(&grp_lock[c]);
		vpss_isp_sync_para(C32(g + 0x14));
	}
}

static int bpu_wait_go(void)
{
	return wait_event_interruptible_timeout(bpu_thread_wq,
						(int)C32(CM_BPU_GO) > 0, 100) > 0;
}

static bool bpu_idle(u32 c)
{
	return !isp_read_reg(0x32000020) && !(C32(CM_BPU_RUN) >> c & 1);
}

/* offline, a frame at a time: a stored frame through the BPU */
static void bpu_from_ib(void)
{
	void *ib = NULL, *vb = NULL;
	u32 c, g, wh, w, h, s, grp;
	u64 t0, t1;
	u8 *b;
	long r;

	if (ib_get_buf(1, 0x10, 0, &ib, 1000)) {
		msleep(10);
		return;
	}
	if (!ib) {
		pr_err("IspKo:Err: %s line%d: bind_id support ISP_MOD_ID_BPU! NULL\n",
		       "bpu_thread", 0x3bd);
		return;
	}
	if (W32(ib, 0x20) != 3) {
		pr_err("IspKo:Err: %s line%d: bind_id support ISP_MOD_ID_BPU! now_id:[%d]\n",
		       "bpu_thread", 0x3c2, W32(ib, 0x20));
		goto put;
	}
	c = W32(ib, 0x24);
	if (c >= N_CHN)
		goto put;
	g = CM_GRP(c);
	s = ST(c);
	wh = C32(CM_CHN + c * 0x28 + 0x10);
	if (!C32(g) || !C32(g + 4) || (wh & 0xffff) < W32(ib, 0x18) ||
	    wh >> 16 < W32(ib, 0x1c)) {
		pr_err("IspKo:Err: %s line[%d]:%d %d %d %d %d %d\n", "bpu_thread",
		       0x3dc, C32(g), C32(g + 4), wh & 0xffff, W32(ib, 0x18),
		       wh >> 16, W32(ib, 0x1c));
		goto put;
	}
	mutex_lock(&grp_lock[c]);
	b = CP(g + GRP_BLOCK);
	C32(g + 0x44)++;
	C32(g + 0x4c) = W32(ib, 0x18);
	C32(g + 0x50) = W32(ib, 0x1c);
	bpu_crop(g, b, &w, &h);
	if (C8(CM_MEMSAVE) == 1) {
		if (vb_get_buf(0, ((w + 0xf) & ~0xf) * h * 3 >> 1, &vb, 0)) {
			C32(s + 0x10)++;
			mutex_unlock(&grp_lock[c]);
			goto put;
		}
		C32(s + 0x14)++;
	}
	bpu_set_out(b, vb, ib, w, h);
	bpu_set_in(c, b, ib, false);
	bpu_update_regs(c, 1, b);
	grp = C32(g + 0x14);
	if (!C8(CM_MEMSAVE)) {
		wh = C32(CM_CHN + c * 0x28 + 0x20);
		vpss_isp_set_val_v2(grp, wh & 0xffff, wh >> 16, 0);
		vpss_thread_set_val(grp, ((w + 0xf) >> 4) * 0x10, w, h,
				    W32(ib, 0x40), W32(ib, 0x3c));
		isp_write_reg(0x32000020, W32(b, 0x574));
		vpss_thread_middle_deal(grp, 1);
	} else {
		isp_write_reg(0x32000020, W32(b, 0x574));
	}
	clk_get_pts(&t0);
	C32(CM_BPU_RUN) |= 1 << c;
	r = 1;
	if (isp_read_reg(0x32000020) || C32(CM_BPU_RUN) >> c & 1)
		r = wait_event_interruptible_timeout(bpu_thread_wq, bpu_idle(c),
						     100);
	clk_get_pts(&t1);
	if (r <= 0) {
		C32(CM_BPU_RUN) &= ~(1 << c);
		inc16(g + 0x48);
		if (vb)
			vb_put_buf(0, vb);
	} else {
		bpu_post_deal(c, 1);
		PTS(s + 0x38) = elapsed(t1, t0);
		if (vb)
			bpu_vb_done(c, vb, -1);
	}
	if (!C8(CM_MEMSAVE))
		vpss_thread_post_deal(grp, 1);
	mutex_unlock(&grp_lock[c]);
put:
	memset((u8 *)ib + 0x20, 0, 0xc);
	ib_put_buf(4, 0, ib);
}

int bpu_thread(void *unused)
{
	u32 go;

	C8(CM_BPU_BUSY) = 0;
	while (!kthread_should_stop()) {
		if (!isp_users_open() || !C32(CM_RUN)) {
			schedule_timeout_interruptible(msecs_to_jiffies(100));
			continue;
		}
		C8(CM_BPU_BUSY) = 1;
		if (C8(CM_MODE) == 1 && (C8(CM_OPT) & 3) < 2) {
			bpu_from_ib();
			continue;
		}
		if ((int)C32(CM_BPU_GO) < 1 && !bpu_wait_go())
			continue;
		if (C8(CM_MODE) == 1) {
			bpu_snapshot(C32(CM_BPU_GO));
			continue;
		}
		go = C32(CM_BPU_GO);
		C32(CM_BPU_GO) = 0; /* isp.ko leaves other channels' bits, and spins */
		if (go & 1)
			bpu_online();
	}
	return 0;
}
