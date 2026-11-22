// SPDX-License-Identifier: GPL-2.0
/*
 * The BPU (Bayer processing): its defaults, the statistics it leaves (AWB,
 * histogram, AF), and how a frame gets to it.
 *
 * Online (CM_MODE 0/2) the VICU feeds it directly; the thread writes the
 * channel's parameter block (iw.c) once a frame, woken by the VICU
 * interrupt. Offline (CM_MODE 1) the VICU stores raw frames in ISP buffers
 * (ib.c) and the BPU reads them back:
 *  - CM_OPT & 3 < 2: the thread takes a stored frame, writes the block,
 *    starts the BPU and waits for it;
 *  - otherwise (the memory saver) bpu_offline_deal, from the VICU's store
 *    interrupt and the BPU's own, starts the next pending channel from a
 *    snapshot of its block that the thread takes when woken.
 * With CM_MEMSAVE the BPU's output goes into a VB buffer handed on to VPSS.
 *
 * A channel's group (CM_GRP) holds, at:
 *   0x00 on, 0x04 enabled (0x4950), 0x08 its VICUs, 0x0c a snapshot taken,
 *   0x10/0x14 the bind (id, chn) of its output, 0x34 the raw bits,
 *   0x38 the WDR mode, 0x50 the frames, 0x54 the late ones, 0x58/0x5c the
 *   input size, 0x60.. the crop (on, x, y, w, h), 0x74 the parameter
 *   block, 0x9a4 its snapshot.
 */

#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>

#include "isp.h"

#define ST(c) (CM_BPUSTAT + (c) * 0x58) /* the channel's statistics */
#define VI(c) (CM_VICU + (c) * 0xd0)
#define PTS(o) (*(u64 *)CP(o))

#define AF_ATTR 0x68 /* {chn, on, 20 registers, ...} */
#define AF_STAT 0x9c /* {chn, 34 registers} */

static u8 held;      /* the parameters waited for (CM_IW_WAIT) and set */
static u32 irq_cnt;  /* BPU frames, for the frame rate in 0x488 */
static u32 af_attr[N_CHN][AF_ATTR / 4];
static u32 af_stat[N_CHN][AF_STAT / 4];

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
	isp_write_reg(0x3202b068, 0);
	irq_cnt = 0;
	held = 0;
}

static const u32 bpu_defaults[][2] = {
	{ 0x32080048, 0 },      { 0x32080040, 0 },      { 0x32080054, 1 },
	{ 0x32000020, 0 },      { 0x3202b030, 0x9b },   { 0x3202b034, 0x40 },
	{ 0x3202b038, 0xc8 },   { 0x3202b03c, 0xa },    { 0x3202b040, 0x31 },
	{ 0x3202b044, 0x340 },  { 0x3202b048, 0x9b },   { 0x3202b04c, 0x25 },
	{ 0x32100008, 3 },      { 0x32030000, 0x3f },   { 0x32030020, 0x3f },
	{ 0x32031010, 0x100 },  { 0x32031014, 0x100 },  { 0x32031018, 0x200 },
	{ 0x3203101c, 0x100 },  { 0x32031058, 6 },      { 0x32031060, 0 },
	{ 0x32031000, 1 },      { 0x32063004, 0x100 },  { 0x32063008, 0x50 },
	{ 0x3206300c, 0 },      { 0x32063010, 0 },      { 0x32063014, 0x40 },
	{ 0x32063018, 0 },      { 0x32063000, 0 },      { 0x32062000, 0x418 },
	{ 0x32062004, 0x809 },  { 0x32062008, 0x18f },  { 0x3206200c, 0x1da3 },
	{ 0x32062010, 0x1b5d }, { 0x32062014, 0x700 },  { 0x32062018, 0x700 },
	{ 0x3206201c, 0x1a23 }, { 0x32062020, 0x1edd }, { 0x32062024, 0x10 },
	{ 0x32062028, 0x80 },   { 0x3206202c, 0x80 },   { 0x3200900c, 0x100 },
	{ 0x32009040, 0xbb8 },  { 0x32009044, 0xcb8 },  { 0x32009048, 0xdb8 },
	{ 0x3200904c, 0x100 },  { 0x32009050, 0x12c },  { 0x32009054, 0x16c },
	{ 0x32009058, 0 },      { 0x32009064, 0xffff }, { 0x32009068, 0x10 },
	{ 0x3200906c, 0x258 },  { 0x32009070, 6 },      { 0x32009080, 0xc },
	{ 0x32009084, 0x100 },  { 0x32009088, 0xfff },  { 0x3200908c, 0 },
	{ 0x32009090, 0x10 },   { 0x32009094, 0x10 },   { 0x32009098, 0 },
	{ 0x320090a4, 1 },      { 0x320090a8, 2 },      { 0x320090ac, 0x10 },
	{ 0x320090b0, 0x10 },   { 0x320090b4, 0 },      { 0x320090b8, 4 },
	{ 0x320090bc, 4 },      { 0x320090c0, 0xa0 },   { 0x320090c4, 0x80 },
	{ 0x320090c8, 0xa0 },   { 0x320090cc, 0xa0 },   { 0x320090d0, 0x80 },
	{ 0x320090d4, 0xa0 },   { 0x320090d8, 0 },      { 0x320090dc, 0 },
	{ 0x320090e0, 0 },      { 0x320090e4, 0 },      { 0x320090e8, 0 },
	{ 0x320090ec, 1 },      { 0x320f0004, 3 },      { 0x320f0008, 1 },
	{ 0x32021000, 0x10 },   { 0x3202a0a4, 0x10 },   { 0x32021004, 0xffff },
	{ 0x3202a050, 0x1e },   { 0x3202a054, 5 },      { 0x3202a058, 0x1f4 },
	{ 0x3202a05c, 0x834 },  { 0x3202a060, 4 },      { 0x3202a064, 0x1e },
	{ 0x3202a068, 5 },      { 0x3202a06c, 0x1f4 },  { 0x3202a070, 0x834 },
	{ 0x3202a074, 4 },      { 0x32050000, 0xfff },  { 0x32050004, 0xfff },
	{ 0x32050008, 0 },      { 0x3205000c, 0x40 },   { 0x32050010, 0xff },
	{ 0x32050014, 0 },      { 0x32050018, 0x200 },  { 0x3205001c, 0x200 },
	{ 0x32050020, 0x400 },  { 0x32050024, 0x180 },  { 0x32050028, 0xc0 },
	{ 0x3205002c, 0xc0 },   { 0x32050030, 0x60 },   { 0x32050034, 0x60 },
	{ 0x32000018, 1 },
};

int bpu_init(void)
{
	int i;

	C32(CM_VB_OUT) = 0;
	memset(CP(CM_GRP(0)), 0, CM_IW_WAIT - CM_GRP(0));
	memset(af_attr, 0, sizeof(af_attr));
	memset(af_stat, 0, sizeof(af_stat));
	if (CS8(CM_OPT) < 0)
		return 0;
	for (i = 0; i < ARRAY_SIZE(bpu_defaults); i++)
		isp_write_reg(bpu_defaults[i][0], bpu_defaults[i][1]);
	if (C8(CM_MODE) == 1) {
		isp_write_reg(0x320f0000, 0);
		isp_write_reg(0x32000024, 1);
		isp_write_reg(0x320f0004, 3);
		isp_write_reg(0x32021000, 0x10);
		isp_write_reg(0x32100000, 1);
		isp_write_reg(0x32009000, 0);
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
	C8(g + 0x34) = a[6];
	C32(g + 0x38) = a[7];
	C32(g + 0x60) = 1;
	C32(g + 0x64) = *(const u16 *)(a + 0x20);
	C32(g + 0x68) = *(const u16 *)(a + 0x22);
	C32(g + 0x6c) = *(const u16 *)(a + 0x24);
	C32(g + 0x70) = *(const u16 *)(a + 0x26);
	if (C32(g + 0x70) > 0x3ffe && C32(g + 0x6c) > 0x3ffe)
		C32(g + 0x60) = 0;
	memcpy(CP(CM_CHN + c * 0x28), a + 8, 0x28);
	return 0;
}

/* 512 zones of R, G, B sums, divided by the block's count (0x496d) */
int awb_stat_get(int wdr, u32 chn)
{
	u32 base, div, r, g, b, o;
	int i;

	if (chn >= N_CHN)
		return -1;
	div = C32(CM_GRP(chn) + 0x38c) ?: 1;
	base = chn && !wdr ? 0x30058140 : 0x32011140;
	isp_write_reg(base, 1);
	for (i = 0; i < 0x200; i++) {
		r = isp_read_reg(base + 8);
		g = isp_read_reg(base + 0xc);
		b = isp_read_reg(base + 0x10);
		isp_write_reg(base + 4, 1);
		o = CM_AWB_STAT + (chn * 0x600 + i) * 2;
		C16(o) = r / div;
		C16(o + 0x400) = (g / div) >> 1;
		C16(o + 0x800) = b / div;
	}
	isp_write_reg(base, 1);
	return 0;
}

int bpu_af_attr_set(void __user *arg)
{
	u32 a[AF_ATTR / 4];

	if (copy_from_user(a, arg, sizeof(a)) || a[0] >= N_CHN)
		return -1;
	mutex_lock(&grp_lock[a[0]]);
	memcpy(af_attr[a[0]], a, sizeof(a));
	mutex_unlock(&grp_lock[a[0]]);
	return 0;
}

int bpu_af_stat_get(void __user *arg)
{
	u32 a[AF_STAT / 4];

	if (copy_from_user(a, arg, sizeof(a)) || a[0] >= N_CHN)
		return -1;
	af_stat[a[0]][0] = a[0];
	if (copy_to_user(arg, af_stat[a[0]], sizeof(a)))
		return -1;
	return 0;
}

/* the 64-bin histogram: max, mean, mean x bins over the threshold, ... */
static void bpu_hist(u32 chn)
{
	u32 d[64], a, b, max = 0, sa = 0, sd = 0, thr, avg, n = 0, s = 0, o;
	int i;

	for (i = 0; i < 64; i++) {
		a = isp_read_reg(0x3202b200 + i * 4);
		b = isp_read_reg(0x3202b300 + i * 4);
		sa += a;
		d[i] = b - a;
		if (b > max)
			max = b;
		sd += b - a;
	}
	thr = (sd + 0x20) >> 6;
	avg = (sa + 0x20) >> 6;
	for (i = 0; i < 64; i++) {
		if (d[i] >= thr) {
			s += d[i];
			n++;
		}
	}
	o = CM_GRP(chn) + 0x94c;
	C32(o) = max;
	C32(o + 4) = avg;
	C32(o + 8) = avg * n;
	C32(o + 0xc) = thr;
	C32(o + 0x10) = s;
}

static void bpu_af(u32 chn)
{
	static const u16 grp[][3] = { /* first register, words, where */
		{ 0x80, 12, 1 }, { 0xc0, 12, 13 }, { 0x100, 7, 25 }, { 0x140, 3, 32 },
	};
	u32 v;
	int i, j;

	if (!af_attr[0][1] && !af_attr[1][1] && !af_attr[2][1]) {
		/* the sensor library's AF windows: kept from one frame */
		v = isp_read_reg(0x32032010);
		if ((v & 0xa58) != 0xa58 || (v & 3) != chn)
			return;
		v = isp_read_reg(0x32032080);
		isp_write_reg(0x32032014, v & 0xfff);
		isp_write_reg(0x32032018, (v >> 12) & 0xfff);
		isp_write_reg(0x3203201c, v >> 24);
		v = isp_read_reg(0x32032088);
		isp_write_reg(0x32032020, v & 0xfff);
		isp_write_reg(0x32032024, (v >> 12) & 0xfff);
		isp_write_reg(0x32032028, v >> 24);
		v = isp_read_reg(0x32032090);
		isp_write_reg(0x3203202c, v & 0xfff);
		isp_write_reg(0x32032030, (v >> 12) & 0xfff);
		isp_write_reg(0x32032034, v >> 24);
		v = isp_read_reg(0x320320a0);
		isp_write_reg(0x32032038, v & 0xfff);
		isp_write_reg(0x3203203c, (v >> 12) & 0xfff);
		isp_write_reg(0x32032040, v >> 24);
		v = isp_read_reg(0x32032094);
		isp_write_reg(0x32032044, v & 0xfff);
		v = isp_read_reg(0x320320a4);
		isp_write_reg(0x32032048, v & 0xfff);
		isp_write_reg(0x32032010, 0xa55);
		return;
	}
	if (chn >= N_CHN)
		return;
	for (i = 0; i < ARRAY_SIZE(grp); i++)
		for (j = 0; j < grp[i][1]; j++)
			af_stat[chn][grp[i][2] + j] =
				isp_read_reg(0x32032000 + grp[i][0] + j * 4);
}

int bpu_post_deal(u32 chn, int mode, int wdr)
{
	if (mode == 2) {
		awb_stat_get(0, 0);
		awb_stat_get(0, 1);
		chn = 0;
	} else if (mode != 1) {
		awb_stat_get(wdr == 1, chn);
	}
	bpu_hist(chn);
	bpu_af(chn);
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
	C32(CM_GRP(chn) + 0x38) = mode;
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

int bpu_set_cut(u32 chn, u32 en, u32 x, u32 y, u32 w, u32 h)
{
	unsigned long flags;
	u32 o = CM_CUT(chn);

	if (chn >= N_CHN)
		return -1;
	spin_lock_irqsave(&bpu_lock2, flags);
	C32(o) = en;
	C32(o + 4) = x;
	C32(o + 8) = y;
	C32(o + 0xc) = w;
	C32(o + 0x10) = h;
	spin_unlock_irqrestore(&bpu_lock2, flags);
	return 0;
}

int bpu_get_cut(u32 chn, u32 *en, u32 *x, u32 *y, u32 *w, u32 *h)
{
	u32 o = CM_CUT(chn);

	if (chn >= N_CHN)
		return -1;
	*en = C32(o);
	*x = C32(o + 4);
	*y = C32(o + 8);
	*w = C32(o + 0xc);
	*h = C32(o + 0x10);
	return 0;
}

/*
 * The output window into the live block (0x920..0x92c) and the output
 * size: the 0x4958 cut, else the channel's crop, else the input.
 */
static void bpu_crop(u32 chn, u32 g, u32 *w, u32 *h)
{
	u32 c = CM_CUT(chn), b = g + GRP_BLOCK;
	u32 cw = C32(g + 0x6c), ch = C32(g + 0x70);

	if (C32(c)) {
		C32(b + 0x920) = C32(c + 4);
		C32(b + 0x924) = C32(c + 0xc) + C32(c + 4);
		C32(b + 0x928) = C32(c + 8);
		C32(b + 0x92c) = C32(c + 0x10) + C32(c + 8);
	} else if (!C32(g + 0x60)) {
		C32(b + 0x920) = 0;
		C32(b + 0x928) = 0;
		C32(b + 0x924) = 0x3fff;
		C32(b + 0x92c) = 0x3fff;
	} else {
		C32(b + 0x920) = C32(g + 0x64);
		C32(b + 0x928) = C32(g + 0x68);
		C32(b + 0x924) = cw + C32(g + 0x64);
		C32(b + 0x92c) = ch + C32(g + 0x68);
	}
	*w = cw > 0x3ffe ? C32(g + 0x58) : cw;
	*h = ch > 0x3ffe ? C32(g + 0x5c) : ch;
}

/* {register, offset in the block, how} */
enum { R32, R16, RS16, RK, RCHN };
struct breg {
	u32 reg;
	u16 off;
	u8 how;
};

static void bpu_write(const struct breg *t, int n, const u8 *b, u32 chn)
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
		case RK:
			v = t[i].off;
			break;
		default:
			v = chn != 0;
			break;
		}
		isp_write_reg(t[i].reg, v);
	}
}

static const struct breg bpu_regs_full[] = {
	{ 0x32030008, 0x2c0, R32 },  { 0x32030004, 0xfff, RK },
	{ 0x3203000c, 0xfff, RK },   { 0x32030010, 0x2c8, R32 },
	{ 0x32030014, 0x2cc, R32 },  { 0x32030018, 0x2d0, R32 },
	{ 0x32030028, 0x2e0, R32 },  { 0x32030024, 0xfff, RK },
	{ 0x3203002c, 0xfff, RK },   { 0x32030030, 0x2e8, R32 },
	{ 0x32030034, 0x2ec, R32 },  { 0x32030038, 0x2f0, R32 },
	{ 0x32050018, 0x2fc, R16 },  { 0x3205001c, 0x2fe, R16 },
	{ 0x32050020, 0x300, R16 },  { 0x32050024, 0x302, R16 },
	{ 0x32050028, 0x304, R16 },  { 0x3205002c, 0x306, R16 },
	{ 0x32060040, 0x3e4, RS16 }, { 0x32060044, 0x3e6, RS16 },
	{ 0x32060048, 0x3e8, RS16 }, { 0x3206004c, 0x3ea, RS16 },
	{ 0x32060050, 0x3ec, RS16 }, { 0x32060054, 0x3ee, RS16 },
	{ 0x32060058, 0x3f0, RS16 }, { 0x3206005c, 0x3f2, RS16 },
	{ 0x32060060, 0x3f4, RS16 }, { 0x32060064, 0x3f6, RS16 },
	{ 0x32060068, 0x3f8, RS16 }, { 0x3206006c, 0x3fa, RS16 },
};

static const struct breg bpu_regs_full2[] = {
	{ 0x32063004, 0x4c4, R32 }, { 0x32063008, 0x4c8, R32 },
	{ 0x32063014, 0x4cc, R32 }, { 0x32063018, 0x4c0, R32 },
	{ 0x32031018, 0x4d0, R16 }, { 0x3203101c, 0x4d2, R16 },
};

static const struct breg bpu_regs_win[] = {
	{ 0x32000008, 0x824, R32 }, { 0x3200000c, 0x828, R32 },
	{ 0x32000010, 0x82c, R32 }, { 0x32000014, 0x830, R32 },
	{ 0x32000000, 0x834, R32 }, { 0x32000004, 0x838, R32 },
	{ 0x32000040, 0x83c, R32 },
};

/* offline: the raw frame in, and the YUV frame out */
static const struct breg bpu_regs_offline[] = {
	{ 0x32001000, 0x844, R32 }, { 0x32001004, 0x848, R32 },
	{ 0x32001008, 0x84c, R32 }, { 0x3200100c, 0x850, R32 },
	{ 0x32001044, 0x858, R32 }, { 0x32001040, 0x854, R32 },
	{ 0x32001028, 0, RCHN },    { 0x32080000, 0x85c, R32 },
	{ 0x32080010, 0x860, R32 }, { 0x32080020, 0x864, R32 },
	{ 0x32080024, 0x868, R32 }, { 0x32080028, 0x86c, R32 },
	{ 0x3208002c, 0x870, R32 }, { 0x32080030, 0x874, R32 },
	{ 0x32080038, 0x87c, R32 }, { 0x32080080, 0x880, R32 },
	{ 0x32080090, 0x884, R32 }, { 0x32080058, 0x888, R32 },
	{ 0x3208005c, 0x88c, R32 }, { 0x32080034, 0x878, R32 },
	{ 0x32000020, 0x840, R32 },
};

static const struct breg bpu_regs_tail[] = {
	{ 0x30000014, 0x7c, R32 },  { 0x30060000, 0x4, R32 },
	{ 0x30060080, 0xc, R32 },   { 0x30060020, 0x14, R32 },
	{ 0x30060024, 0x1c, R32 },  { 0x30060038, 0x34, R32 },
	{ 0x30060054, 0x3c, R32 },  { 0x30060030, 0x24, R32 },
	{ 0x30060058, 0x44, R32 },  { 0x30060034, 0x2c, R32 },
	{ 0x32001000, 0x4c, R32 },  { 0x32001004, 0x54, R32 },
	{ 0x32001008, 0x5c, R32 },  { 0x3200100c, 0x64, R32 },
	{ 0x32001044, 0x74, R32 },  { 0x32001040, 0x6c, R32 },
	{ 0x32009000, 0x7d0, R32 }, { 0x3200900c, 0x7d4, R32 },
	{ 0x32009040, 0x7d8, R32 }, { 0x32009044, 0x7dc, R32 },
	{ 0x32009048, 0x7e0, R32 }, { 0x3200904c, 0x7e4, R32 },
	{ 0x32009050, 0x7e8, R32 }, { 0x32009054, 0x7ec, R32 },
	{ 0x3200906c, 0x7f0, R32 }, { 0x32009070, 0x7f4, R32 },
	{ 0x32009080, 0x7f8, R32 }, { 0x32009084, 0x7fc, R32 },
	{ 0x320090b8, 0x800, R32 }, { 0x320090bc, 0x804, R32 },
	{ 0x320090c0, 0x808, R32 }, { 0x320090c8, 0x808, R32 },
	{ 0x320090cc, 0x80c, R32 }, { 0x320090d4, 0x80c, R32 },
	{ 0x32009098, 0x810, R32 },
};

#define BW(t, b, c) bpu_write(t, ARRAY_SIZE(t), b, c)

/*
 * A parameter block (b, the live one or a snapshot) to the registers.
 * With CM_IW_WAIT the full set goes once, then only what changes per frame
 * until the waited-for parameters are released.
 */
static int bpu_update_regs(u32 chn, int mode, int wdr, u8 *b)
{
	u32 sh = C8(CM_GRP(chn) + 0x34) == 4 ? 0 : 4;
	u32 dx, dy, bw, bh, lw, lh, v, r, gr, bl;
	const u32 *af = af_attr[chn < N_CHN ? chn : 0];
	int i;

	if (C32(CM_IW_WAIT) == 0xff) {
		isp_write_reg(0x32000020, 0);
		isp_write_reg(0x32001040, 0);
		isp_write_reg(0x32080034, 0);
		isp_write_reg(0x3202b068, 0);
		return 0;
	}
	if (C32(CM_IW_WAIT) == 0)
		held = 0;
	if (held) {
		v = W32(b, 0x8d4);
		W32(b, 0x8c0) = v;
		isp_write_reg(0x3202b060, v);
		isp_write_reg(0x3202b064, v);
		goto frame;
	}
	if (af[1])
		for (i = 0; i < 20; i++)
			isp_write_reg(0x32032000 + i * 4, af[2 + i]);
	BW(bpu_regs_full, b, chn);
	for (i = 0; i < 97; i++)
		isp_write_reg(0x32061000 + i * 4, W16(b, 0x3fc + i * 2));
	BW(bpu_regs_full2, b, chn);
	/* the shading grid: 32 x 32 blocks, the last ones what is left */
	dx = W32(b, 0x828) - W32(b, 0x824);
	dy = W32(b, 0x830) - W32(b, 0x82c);
	bw = (dx + 0x10) >> 5;
	bh = (dy + 0x10) >> 5;
	lw = dx > bw * 31 ? dx - bw * 31 : 0;
	lh = dy > bh * 31 ? dy - bh * 31 : 0;
	if (bw * bh * lw * lh == 0) {
		pr_err("IspKo:Err: %s line%d: ch:%d: size=0[%d %d %d %d]\n",
		       "bpu_update_regs", 0x2d1, chn, bw, bh, lw, lh);
		return -1;
	}
	W32(b, 0x8a4) = bh - 1;
	W32(b, 0x894) = W32(b, 0x8c8);
	W32(b, 0x898) = W32(b, 0x8cc);
	W32(b, 0x8a0) = bw - 1;
	W32(b, 0x89c) = W32(b, 0x8d0);
	W32(b, 0x8c0) = W32(b, 0x8d4);
	W32(b, 0x8a8) = 0x100000 / bw;
	W32(b, 0x8ac) = 0x100000 / bh;
	W32(b, 0x8b0) = 0x100000 / (bw * bh);
	W32(b, 0x8b4) = 0x100000 / (bh * lw);
	W32(b, 0x8b8) = 0x100000 / (bw * lh);
	W32(b, 0x8bc) = 0x100000 / (lw * lh);
	isp_write_reg(0x3202b020, W32(b, 0x894));
	isp_write_reg(0x3202b024, W32(b, 0x898));
	isp_write_reg(0x3202b02c, W32(b, 0x89c));
	for (i = 0; i < 8; i++)
		isp_write_reg(0x3202b000 + i * 4, W32(b, 0x8a0 + i * 4));
	isp_write_reg(0x3202b060, W32(b, 0x8c0));
	isp_write_reg(0x3202b064, W32(b, 0x8c0));
	isp_write_reg(0x3202b068, W32(b, 0x8c4) != 0);
	for (i = 0; i < 128; i++)
		isp_write_reg(0x3202a200 + i * 4, W32(b, 0x558 + i * 4));
	BW(bpu_regs_win, b, chn);
	for (i = 0; i < 3; i++) {
		isp_write_reg(0x32062000 + i * 4, W16(b, 0x8fc + i * 2) & 0x1fff);
		isp_write_reg(0x3206200c + i * 4, W16(b, 0x904 + i * 2) & 0x1fff);
		isp_write_reg(0x32062018 + i * 4, W16(b, 0x90c + i * 2) & 0x1fff);
		isp_write_reg(0x32062024 + i * 4, W8(b, 0x902 + i * 8));
	}
	if (W32(b, 0x924) < 0x3fff) {
		isp_write_reg(0x32064000, W32(b, 0x920));
		isp_write_reg(0x32064004, W32(b, 0x924));
	}
	if (W32(b, 0x92c) < 0x3fff) {
		isp_write_reg(0x32064008, W32(b, 0x928));
		isp_write_reg(0x3206400c, W32(b, 0x92c));
	}
frame:
	r = W16(b, 0x2b4);
	gr = W16(b, 0x2b6);
	bl = W16(b, 0x2b8);
	if (mode == 1) {
		BW(bpu_regs_offline, b, chn);
	} else if (!held) {
		if (wdr == 1) {
			isp_write_reg(0x320f0004, 0);
			isp_write_reg(0x32021000, W32(b, 0x308));
			isp_write_reg(0x32020000, min_t(u32, r << sh, 0xffff));
			isp_write_reg(0x32020004, min_t(u32, gr << sh, 0xffff));
			isp_write_reg(0x32020008, min_t(u32, bl << sh, 0xffff));
			isp_write_reg(0x32028000, W32(b, 0x30c));
			isp_write_reg(0x32028004, W32(b, 0x310));
			isp_write_reg(0x32028008, W32(b, 0x314));
			for (i = 0; i < 33; i++)
				isp_write_reg(0x32011000 + i * 4, W32(b, 0x31c + i * 4));
			for (i = 0; i < 17; i++)
				isp_write_reg(0x320110c0 + i * 4, W32(b, 0x3a0 + i * 4));
		} else {
			isp_write_reg(0x32021000, W32(b, 0x91c));
		}
		v = isp_read_reg(0x30001000);
		isp_write_reg(0x30001000, v | (W32(b, 0x914) & 0xfff) >> 8);
		isp_write_reg(0x30000018, W32(b, 0x914) & 0xf);
		isp_write_reg(0x320f0000, W8(b, 0x918));
		isp_write_reg(0x32100000, (W32(b, 0x918) & 0xfff) >> 8);
		isp_write_reg(0x3200a060, W32(b, 0x764) == 0);
		for (i = 0; i < 8; i++) {
			isp_write_reg(0x3200a000 + i * 4, W32(b, 0x76c + i * 4));
			isp_write_reg(0x3200a020 + i * 4, W32(b, 0x78c + i * 4));
			isp_write_reg(0x3200a040 + i * 4, W32(b, 0x7ac + i * 4));
		}
		BW(bpu_regs_tail, b, chn);
	}
	if (C32(CM_IW_WAIT) & 1 << chn)
		held = 1;
	return 0;
}

/*
 * The YUV output into a VB buffer (vb) of w x h at the stride rounded to
 * 16, and the raw input from the ISP buffer (ib): into the block b.
 */
static void bpu_set_out(u8 *b, void *vb, void *ib, u32 w, u32 h)
{
	u32 st = (w + 0xf) & ~0xf, y = W32(vb, 0x10), uv = y + st * h;
	u32 w16 = (w + 0xf) >> 4;

	W32(vb, 0x20) = h;
	W32(vb, 0x1c) = w;
	W32(vb, 0x18) = st;
	W32(vb, 0x28) = W32(ib, 0x40);
	W32(vb, 0x2c) = W32(ib, 0x3c);
	W32(b, 0x85c) = (y + 0x80000000) >> 4;
	W32(b, 0x860) = (uv + 0x80000000) >> 4;
	W32(b, 0x864) = w >> 4;
	W32(b, 0x868) = w >> 4;
	W32(b, 0x86c) = w >> 4;
	W32(b, 0x870) = w >> 4;
	W32(b, 0x874) = 0;
	W32(b, 0x87c) = 0;
	W32(b, 0x878) = 1;
	W32(b, 0x888) = h * w16;
	W32(b, 0x884) = (w * ((h >> 1) - 1) + uv + 0x80000000) >> 4;
	W32(b, 0x880) = (w * (h - 1) + y + 0x80000000) >> 4;
	W32(b, 0x88c) = h * ((w16 + 1) >> 1);
}

/* the raw frame in, stored by the VICU at 10 (VICU 0) or 16 bits a pixel */
static void bpu_set_in(u32 chn, u8 *b, void *ib)
{
	u32 a = CM_CHN + chn * 0x28, div = chn ? 0x10 : 10, n;
	u32 x = C16(a + 0xc), y = C16(a + 0xe);

	W32(b, 0x824) = x;
	W32(b, 0x828) = C16(a + 0x10) + x;
	W32(b, 0x82c) = y;
	W32(b, 0x830) = C16(a + 0x12) + y;
	W32(b, 0x834) = C16(a + 0x14) - 1;
	W32(b, 0x838) = C16(a + 0x16) - 1;
	W32(b, 0x83c) = 1;
	W32(b, 0x840) = 1;
	n = (W32(ib, 0x18) - 1 + div) / div;
	W32(b, 0x844) = (W32(ib, 0x10) + 0x80000000) >> 4;
	W32(b, 0x848) = ((W32(ib, 0x1c) - 1) * div * n + W32(ib, 0x10) -
			 0x80000000) >> 4;
	W32(b, 0x84c) = div * n / div;
	W32(b, 0x850) = W32(b, 0x84c);
	W32(b, 0x854) = 1;
	W32(b, 0x858) = 0;
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

/*
 * The memory saver: chn (0..2) has stored a frame, or 0xff the BPU is
 * done with cur. Starts the BPU on the next pending channel, if idle.
 */
int bpu_offline_deal(u32 chn, u32 cur)
{
	u8 mode = C8(CM_MODE);
	void *ib = NULL, *vb = NULL, *old;
	unsigned long flags, f2;
	u32 k, bit, p, g, wh, w, h;
	u64 now, start;
	u8 *b;

	if ((C8(CM_OPT) & 3) < 2)
		return 0;
	if (chn != 0xff && chn >= N_CHN) {
		pr_err("IspKo:Err: %s %d\n", __func__, 0x552);
		return -1;
	}
	spin_lock_irqsave(&bpu_lock2, flags);
	clk_get_pts(&now);
	if (chn == 0xff) {
		if (C32(CM_BPU_CUR) != cur) {
			pr_err("IspKo:Err: %s %d\n", __func__, 0x55f);
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
				pr_err("IspKo:Err: %s %d [%lld %lld]ch%d\n", __func__,
				       0x573, start, now, chn);
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
	if (k && C32(0x6368)) { /* frames VICU 1 drops after a switch */
		C32(0x6368)--;
		goto idle;
	}
	if (PTS(VI(k) + 0x90) <= PTS(VI(k) + 0x88))
		inc16(ST(k) + 0x24);
	wh = C32(CM_CHN + k * 0x28 + 0x10);
	if (!C32(g) || !C32(g + 4) || W32(ib, 0x18) > (wh & 0xffff) ||
	    W32(ib, 0x1c) > wh >> 16) {
		pr_err("IspKo:Err: %s line[%d]:%d %d %d %d %d %d\n", __func__,
		       0x5ae, C32(g), C32(g + 4), wh & 0xffff, W32(ib, 0x18),
		       wh >> 16, W32(ib, 0x1c));
		goto idle;
	}
	C32(g + 0x50)++;
	C32(g + 0x58) = W32(ib, 0x18);
	C32(g + 0x5c) = W32(ib, 0x1c);
	bpu_crop(k, g, &w, &h);
	if (C8(CM_MEMSAVE) == 1) {
		if (vb_get_buf(0, ((w + 0xf) & ~0xf) * h * 3 >> 1, &vb, 0)) {
			C32(ST(k) + 0x10)++;
			goto idle;
		}
		C32(ST(k) + 0x14)++;
		bpu_set_out(b, vb, ib, w, h);
	} else {
		W32(b, 0x878) = 0;
	}
	bpu_set_in(k, b, ib);
	spin_lock_irqsave(&bpu_irq_lock, f2);
	old = (void *)C32(CM_VB_OUT);
	if (old) { /* the BPU never finished the last one */
		bpu_vb_done(k, old, -1);
		C32(CM_VB_OUT) = 0;
		inc16(ST(k) + 0x24);
	}
	C32(CM_VB_OUT) = (u32)vb;
	spin_unlock_irqrestore(&bpu_irq_lock, f2);
	bpu_update_regs(k, mode, 0, b);
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
		pr_err("IspKo:Err: %s line[%d]: 0x%x\n", __func__, 0x3d3, run);
	if (++irq_cnt > 1000)
		irq_cnt = 1000;
	last = PTS(0x4b8);
	if (now > last) {
		d = now - last > 0xffffffff ? 0xffffffff : now - last;
		if (d / 1000 >= 2000) {
			C32(0x490) = (irq_cnt * 100000 / (d / 1000) + 5) / 10;
			irq_cnt = 0;
			PTS(0x4b8) = now;
		}
	}
	C32(0x48c) = 1;
	n = C32(0x494) + 1;
	C32(0x494) = n > 0xfffffff ? 0 : n;
	C32(CM_BPU_RUN) = 0;
	if ((C8(CM_OPT) & 3) < 2) {
		wake_up(&bpu_thread_wq);
		goto out;
	}
	if (c == N_CHN)
		goto out;
	bpu_post_deal(c, C8(CM_MODE), 0);
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
	set_max(ST(c) + 0x40, PTS(ST(c) + 0x38) = elapsed(now, PTS(VI(c) + 0x90)));
	last = PTS(VI(c) + 0x88);
	if (now <= last) {
		inc16(CM_GRP(c) + 0x54);
	} else {
		set_max(ST(c) + 0x48, now - last);
		if (now - last > 79000)
			inc16(CM_GRP(c) + 0x54);
	}
out:
	spin_unlock_irqrestore(&bpu_irq_lock, flags);
next:
	bpu_offline_deal(0xff, C32(CM_BPU_CUR));
	return IRQ_HANDLED;
}

static int bpu_wait_go(void)
{
	return wait_event_interruptible_timeout(bpu_thread_wq,
						(int)C32(CM_BPU_GO) > 0, 100) > 0;
}

/* online: the block of channel 0 straight to the registers */
static void bpu_online(void)
{
	u32 g = CM_GRP(0), b = g + GRP_BLOCK, a = CM_CHN, wdr, grp;

	if (!(C32(CM_BPU_GO) & 1)) {
		C32(CM_BPU_GO) = 0; /* isp.ko spins on these */
		return;
	}
	C32(CM_BPU_GO) = 0;
	mutex_lock(&grp_lock[0]);
	wdr = C32(g + 0x38);
	if (!C32(g + 0x60)) {
		C32(b + 0x920) = 0;
		C32(b + 0x928) = 0;
		C32(b + 0x924) = 0x3fff;
		C32(b + 0x92c) = 0x3fff;
	} else {
		C32(b + 0x920) = C32(g + 0x64);
		C32(b + 0x924) = C32(g + 0x6c) + C32(g + 0x64);
		C32(b + 0x928) = C32(g + 0x68);
		C32(b + 0x92c) = C32(g + 0x70) + C32(g + 0x68);
	}
	C32(b + 0x824) = C16(a + 0xc);
	C32(b + 0x828) = C16(a + 0x10) + C16(a + 0xc);
	C32(b + 0x82c) = C16(a + 0xe);
	C32(b + 0x830) = C16(a + 0x12) + C16(a + 0xe);
	C32(b + 0x834) = C16(a + 0x14) - 1;
	C32(b + 0x838) = C16(a + 0x16) - 1;
	switch (wdr) {
	case 0:
	case 1:
		C32(b + 0x83c) = 0;
		C32(b + 0x840) = 0;
		C32(b + 0x854) = 0;
		C32(b + 0x858) = 0;
		C32(b + 0x874) = 0;
		C32(b + 0x878) = 0;
		C32(b + 0x914) = 0x100;
		C32(b + 0x918) = 5;
		C32(b + 0x91c) = wdr ? 0x100 : 0x10;
		break;
	case 2:
		C32(b + 0x83c) = 0;
		C32(b + 0x854) = 0;
		C32(b + 0x858) = 0;
		C32(b + 0x874) = 0;
		C32(b + 0x840) = 1;
		C32(b + 0x878) = 1;
		C32(b + 0x914) = 0x301;
		C32(b + 0x918) = 0x106;
		C32(b + 0x91c) = 0x100;
		break;
	default:
		pr_err("IspKo:Err: wdr_mode: %d not support!\n", wdr);
		break;
	}
	bpu_update_regs(0, C8(CM_MODE), wdr, CP(b));
	mutex_unlock(&grp_lock[0]);
	grp = C32(g + 0x14);
	vpss_isp_set_val(grp, C32(a + 0x20) & 0xffff, C32(a + 0x20) >> 16);
	vpss_isp_post_deal(grp, C32(a + 0x20) & 0xffff, C32(a + 0x20) >> 16);
}

/* the memory saver: a snapshot of the next channel's block */
static void bpu_snapshot(void)
{
	unsigned long flags;
	u32 c, g;

	for (c = 0; c < N_CHN; c++)
		if ((int)C32(CM_BPU_GO) >> c & 1)
			break;
	if (c == N_CHN)
		return;
	g = CM_GRP(c);
	mutex_lock(&grp_lock[c]);
	spin_lock_irqsave(&awb_lock[c], flags);
	C32(CM_BPU_GO) &= ~(1 << c);
	memcpy(CP(g + GRP_SNAP), CP(g + GRP_BLOCK), 0x930);
	C32(g + 0xc) = 1;
	spin_unlock_irqrestore(&awb_lock[c], flags);
	mutex_unlock(&grp_lock[c]);
}

static bool bpu_idle(u32 c)
{
	return !isp_read_reg(0x32000020) && !(C32(CM_BPU_RUN) >> c & 1);
}

/* offline, a frame at a time: a stored frame through the BPU */
static void bpu_from_ib(void)
{
	void *ib = NULL, *vb = NULL;
	u32 c, g, wh, w, h, s;
	u64 t0, t1;
	u8 *b;
	long r;

	if (ib_get_buf(1, 0x10, 0, &ib, 1000)) {
		msleep(10);
		return;
	}
	if (!ib) {
		pr_err("IspKo:Err: %s line%d: bind_id support ISP_MOD_ID_BPU! NULL\n",
		       "bpu_thread", 0x4a4);
		return;
	}
	if (W32(ib, 0x20) != 3) {
		pr_err("IspKo:Err: %s line%d: bind_id support ISP_MOD_ID_BPU! now_id:[%d]\n",
		       "bpu_thread", 0x4a9, W32(ib, 0x20));
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
		       0x4c3, C32(g), C32(g + 4), wh & 0xffff, W32(ib, 0x18),
		       wh >> 16, W32(ib, 0x1c));
		goto put;
	}
	mutex_lock(&grp_lock[c]);
	b = CP(g + GRP_BLOCK);
	C32(g + 0x50)++;
	C32(g + 0x58) = W32(ib, 0x18);
	C32(g + 0x5c) = W32(ib, 0x1c);
	bpu_crop(c, g, &w, &h);
	if (C8(CM_MEMSAVE) == 1) {
		if (vb_get_buf(0, ((w + 0xf) & ~0xf) * h * 3 >> 1, &vb, 0)) {
			C32(s + 0x10)++;
			mutex_unlock(&grp_lock[c]);
			goto put;
		}
		C32(s + 0x14)++;
		bpu_set_out(b, vb, ib, w, h);
	} else {
		W32(b, 0x878) = 0;
	}
	bpu_set_in(c, b, ib);
	bpu_update_regs(c, 1, 0, b);
	clk_get_pts(&t0);
	C32(CM_BPU_RUN) |= 1 << c;
	r = 1;
	if (isp_read_reg(0x32000020) || C32(CM_BPU_RUN) >> c & 1)
		r = wait_event_interruptible_timeout(bpu_thread_wq, bpu_idle(c),
						     100);
	clk_get_pts(&t1);
	if (r <= 0) {
		C32(CM_BPU_RUN) &= ~(1 << c);
		inc16(g + 0x54);
		if (vb)
			vb_put_buf(0, vb);
	} else {
		bpu_post_deal(c, 1, 0);
		PTS(s + 0x38) = elapsed(t1, t0);
		if (vb)
			bpu_vb_done(c, vb, -1);
	}
	mutex_unlock(&grp_lock[c]);
put:
	memset((u8 *)ib + 0x20, 0, 0xc);
	ib_put_buf(4, 0, ib);
}

int bpu_thread(void *unused)
{
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
		if (C8(CM_MODE) == 1)
			bpu_snapshot();
		else
			bpu_online();
	}
	return 0;
}
