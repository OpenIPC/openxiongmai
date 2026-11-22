// SPDX-License-Identifier: GPL-2.0
/*
 * The BPU (Bayer processing) side: its defaults, the AWB statistics, and
 * the thread that writes the parameter blocks (iw.c) to the registers once
 * a frame, woken by the VICU interrupt.
 *
 * The blocks, at their g_comm_para offsets, and where they go:
 *   0x0e34  0x4970: 0xed8..0xee0 the black level (0x30018000..)
 *   0x0ee2  0x4971: 15 + 4 x 61 gamma/curve points (0x30013000..)
 *   0x10e8  0x4967: the white balance gains R, G, B (0x30012000/0x30022000)
 *   0x10ee  0x4968: 0x30020000.. and 0x30029018.. (demosaic, sharpening)
 *   0x1128  0x4964: the exposure (0x30014000)
 *   0x112c  0x4965: again, dgain, ispgain (0x30015000/0x30025000)
 *   0x1138  0x496d: the AWB statistics windows (0x30016000..)
 *   0x1204  0x4966: the colour matrix, signed (0x30030040..)
 *   0x121c  0x4969: 97 points (0x30031000..)
 *   0x12e0  0x496a: 0x30033004.., 0x12f0 0x496b: 0x3002100c..
 *   0x1374  0x496e: 0x30025040.. (lens shading)
 *   0x16e8  0x497f: the shading grid's settings
 *   0x171c  0x4976: the RGB -> YUV matrix (0x30032000..)
 */

#include <linux/delay.h>
#include <linux/kernel.h>
#include <linux/kthread.h>
#include <linux/sched.h>
#include <linux/spinlock.h>

#include "isp.h"

static u8 bpu_held; /* the parameters waited for (CM_IW_WAIT) and set */
DEFINE_SPINLOCK(awb_stat_lock);

void bpu_init_queue(void)
{
}

void bpu_reset(void)
{
	bpu_held = 0;
}

static const u32 bpu_defaults[][2] = {
	{ 0x3002000c, 0x800 }, { 0x3002001c, 0x400 }, { 0x3002002c, 0x800 },
	{ 0x30020050, 0x800 }, { 0x30020058, 0x800 }, { 0x30020060, 0x800 },
	{ 0x30020074, 0x10 },  { 0x30020078, 0x10 },  { 0x3002007c, 0x10 },
	{ 0x30020080, 0x40 },  { 0x30020084, 0x40 },  { 0x30020088, 0x40 },
	{ 0x30020008, 0x10 },  { 0x30020018, 0 },     { 0x30020028, 0x10 },
	{ 0x300260c0, 0 },     { 0x3002608c, 0xffa }, { 0x30026040, 0xff },
	{ 0x30026088, 0x60 },  { 0x30026094, 0x96 },  { 0x30026098, 0x50 },
	{ 0x300260ac, 0x16 },  { 0x300260b0, 0x16 },
};

static const u32 bpu_defaults2[][2] = {
	{ 0x30018018, 0 },      { 0x30029000, 0 },      { 0x30029004, 0 },
	{ 0x30029008, 0x20 },   { 0x3002900c, 0x1ff },  { 0x30029010, 0x20 },
	{ 0x30029030, 0x400 },  { 0x30029034, 0x400 },  { 0x30033004, 0x100 },
	{ 0x30033008, 0x50 },   { 0x3003300c, 0 },      { 0x30033010, 0 },
	{ 0x30033014, 0x40 },   { 0x30033018, 0 },      { 0x30033000, 0 },
	{ 0x30032000, 0x418 },  { 0x30032004, 0x809 },  { 0x30032008, 399 },
	{ 0x3003200c, 0x1da3 }, { 0x30032010, 0x1b5d }, { 0x30032014, 0x700 },
	{ 0x30032018, 0x700 },  { 0x3003201c, 0x1a23 }, { 0x30032020, 0x1edd },
	{ 0x30032024, 0x10 },   { 0x30032028, 0x80 },   { 0x3003202c, 0x80 },
};

int bpu_init(void)
{
	int i;

	memset(CP(CM_GRP), 0, 0x980);
	if (CS8(CM_OPT) < 0)
		return 0;
	for (i = 0; i < ARRAY_SIZE(bpu_defaults); i++)
		isp_write_reg(bpu_defaults[i][0], bpu_defaults[i][1]);
	isp_write_reg(0x300f0000, isp_read_reg(0x300f0000) | 4);
	for (i = 0; i < ARRAY_SIZE(bpu_defaults2); i++)
		isp_write_reg(bpu_defaults2[i][0], bpu_defaults2[i][1]);
	return 0;
}

/* 0x490e's 52 bytes, as bpu_chn_init reads them */
int bpu_chn_init(const u8 *a)
{
	u32 chn = a[0], g = CM_GRP + chn * 0x980, u;

	if (chn) /* one channel (isp.ko writes past its block) */
		return 0;
	C32(g + 8) = a[1];
	C32(g + 4) = 1;
	C32(g + 0) = 1;
	C8(g + 0x24) = a[6];
	C32(g + 0x50) = 1;
	C32(g + 0x28) = a[7];
	C32(g + 0x54) = *(u16 *)(a + 0x20);
	C32(g + 0x58) = *(u16 *)(a + 0x22);
	C32(g + 0x5c) = *(u16 *)(a + 0x24);
	C32(g + 0x60) = u = *(u16 *)(a + 0x26);
	if (u > 0x3ffe)
		u = C32(g + 0x5c);
	if (u > 0x3ffe)
		C32(g + 0x50) = 0;
	memcpy(CP(0x70 + chn * 0x28), a + 8, 0x28);
	return 0;
}

/* The AWB statistics, 0x200 blocks of R, G, B sums, into CM_AWB_STAT */
static int awb_stat_get(int mode, u32 chn)
{
	unsigned long flags;
	u32 div, r, g, b;
	int i;

	if (chn)
		return -1;
	div = C32(0x1138) ?: 1;
	if (mode != 0) {
		pr_err("IspKo:Err: %s %d failed!\n", "awb_stat_get", 0xbb);
		return -1;
	}
	spin_lock_irqsave(&awb_stat_lock, flags);
	isp_write_reg(0x30016140, 1);
	for (i = 0; i < 0x200; i++) {
		r = isp_read_reg(0x30016148);
		g = isp_read_reg(0x3001614c);
		b = isp_read_reg(0x30016150);
		isp_write_reg(0x30016144, 1);
		C16(CM_AWB_STAT + i * 2) = r / div;
		C16(CM_AWB_STAT + 0x400 + i * 2) = (g / div) >> 1;
		C16(CM_AWB_STAT + 0x800 + i * 2) = b / div;
	}
	isp_write_reg(0x30016140, 1);
	spin_unlock_irqrestore(&awb_stat_lock, flags);
	return 0;
}

int bpu_post_deal(u32 chn)
{
	awb_stat_get(0, chn);
	return 0;
}

int bpu_set_wdr_mode(int chn, u32 mode)
{
	unsigned long flags;

	if (chn != 0)
		return -1;
	local_irq_save(flags);
	C32(0xdf8) = mode;
	if (C32(0xdd8) & 1)
		C32(CM_WDR_MODE) = mode;
	local_irq_restore(flags);
	return 0;
}

int bpu_set_enable(int chn, int en)
{
	u8 old;

	if (chn != 0)
		return -1;
	old = C8(0xdd4);
	C32(0xdd4) = en != 0;
	return old != 0;
}

/*
 * The shading grid: the picture in 32-pixel blocks, the reciprocals of the
 * block counts and the edge blocks' sizes (0x100000 / n), and the gains.
 */
static int bpu_lsc(void)
{
	u32 w = C32(0x1648) - C32(0x1644), h = C32(0x1650) - C32(0x164c);
	u32 nx = (w + 0x10) >> 5, ny = (h + 0x10) >> 5;
	u32 ex = 0, ey = 0, k;

	if (nx * 31 < w)
		ex = w - nx * 31;
	if (ny * 31 < h)
		ey = h - ny * 31;
	if (!ex || !ey || nx * ny * ex * ey == 0) {
		pr_err("IspKo:Err: %s line%d: ch:%d: size=0[%d %d %d %d]\n",
		       "bpu_update_regs", 0x1ac, 0, nx, ny, ex, ey);
		return -1;
	}
	C32(0x16c4) = ny - 1;
	C32(0x16b4) = C32(0x16e8);
	C32(0x16b8) = C32(0x16ec);
	C32(0x16bc) = C32(0x16f0);
	C32(0x16c0) = nx - 1;
	C32(0x16e0) = C32(0x16f4);
	C32(0x16c8) = 0x100000 / nx;
	C32(0x16cc) = 0x100000 / ny;
	C32(0x16d0) = 0x100000 / (nx * ny);
	C32(0x16d4) = 0x100000 / (ny * ex);
	C32(0x16d8) = 0x100000 / (nx * ey);
	C32(0x16dc) = 0x100000 / (ex * ey);
	isp_write_reg(0x300260c0, C32(0x16e8));
	isp_write_reg(0x30026020, C32(0x16c0));
	isp_write_reg(0x30026024, C32(0x16c4));
	isp_write_reg(0x30026038, C32(0x16c8));
	isp_write_reg(0x3002603c, C32(0x16cc));
	isp_write_reg(0x30026028, C32(0x16d0));
	isp_write_reg(0x3002602c, C32(0x16d4));
	isp_write_reg(0x30026030, C32(0x16d8));
	isp_write_reg(0x30026034, C32(0x16dc));
	for (k = 0; k < 104; k++)
		isp_write_reg(0x30025040 + k * 4, C32(0x1378 + k * 4));
	return 0;
}

/* The frame's registers from the parameter blocks */
static void bpu_update_regs(void)
{
	u32 mode = C8(CM_MODE), r, g, b, k, base;

	if (C32(0xe20) == 0) {
		C32(0x1740) = 0;
		C32(0x1748) = 0;
		C32(0x1744) = 0x3fff;
		C32(0x174c) = 0x3fff;
	} else {
		C32(0x1740) = C32(0xe24);
		C32(0x1744) = C32(0xe2c) + C32(0xe24);
		C32(0x1748) = C32(0xe28);
		C32(0x174c) = C32(0xe30) + C32(0xe28);
	}
	C32(0x1644) = C16(0x7c);
	C32(0x1648) = C16(0x80) + C16(0x7c);
	C32(0x164c) = C16(0x7e);
	C32(0x1650) = C16(0x82) + C16(0x7e);
	C32(0x1654) = C16(0x84) - 1;
	C32(0x1658) = C16(0x86) - 1;
	switch (C32(0xdf8)) {
	case 1:
	case 0:
		C32(0x165c) = 0;
		C32(0x1660) = 0;
		C32(0x1674) = 0;
		C32(0x1678) = 0;
		C32(0x1694) = 0;
		C32(0x1698) = 0;
		C32(0x1734) = 0x100;
		C32(0x1738) = 5;
		C32(0x173c) = C32(0xdf8) == 1 ? 0x100 : 0x10;
		break;
	case 2:
		C32(0x165c) = 0;
		C32(0x1674) = 0;
		C32(0x1678) = 0;
		C32(0x1694) = 0;
		C32(0x1660) = 1;
		C32(0x1698) = 1;
		C32(0x1734) = 0x301;
		C32(0x1738) = 0x102;
		C32(0x173c) = 0x100;
		break;
	default:
		pr_err("IspKo:Err: wdr_mode: %d not support!\n", C32(0xdf8));
	}
	if (C32(CM_IW_WAIT) == 0xff)
		return;
	if (C32(CM_IW_WAIT) == 0) {
		bpu_held = 0;
	} else if (bpu_held) {
		C32(0x16e0) = C32(0x16f4);
		return;
	}
	/* curves */
	for (k = 0; k < 15; k++)
		isp_write_reg(0x30013000 + k * 4, C16(0xee2 + k * 2));
	for (k = 0; k < 61; k++) {
		isp_write_reg(0x30013040 + k * 4, C16(0xf00 + k * 2));
		isp_write_reg(0x30013140 + k * 4, C16(0xf7a + k * 2));
		isp_write_reg(0x30013240 + k * 4, C16(0xff4 + k * 2));
		isp_write_reg(0x30013340 + k * 4, C16(0x106e + k * 2));
	}
	/* black level */
	isp_write_reg(0x30018000, C16(0xed8));
	isp_write_reg(0x30018004, C16(0xeda));
	isp_write_reg(0x3001800c, C16(0xedc));
	isp_write_reg(0x30018010, C16(0xede));
	isp_write_reg(0x30018014, C16(0xee0));
	/* 0x4968 */
	isp_write_reg(0x30020000, C16(0x1100));
	isp_write_reg(0x30020010, C16(0x1102));
	isp_write_reg(0x30020020, C16(0x1104));
	isp_write_reg(0x30020004, C16(0x1106));
	isp_write_reg(0x30020014, C16(0x1108));
	isp_write_reg(0x30020024, C16(0x110a));
	isp_write_reg(0x30020054, C16(0x1112));
	isp_write_reg(0x3002005c, C16(0x1114));
	isp_write_reg(0x30020064, C16(0x1116));
	isp_write_reg(0x30020068, C16(0x110c));
	isp_write_reg(0x3002006c, C16(0x110e));
	isp_write_reg(0x30020070, C16(0x1110));
	isp_write_reg(0x3002008c, C16(0x10ee));
	for (k = 0; k < 8; k++)
		isp_write_reg(0x30020030 + k * 4, C16(0x10f0 + k * 2));
	isp_write_reg(0x30020100, C16(0x111a));
	for (k = 0; k < 6; k++)
		isp_write_reg(0x30029018 + k * 4, C16(0x111c + k * 2));
	/* the colour matrix */
	for (k = 0; k < 12; k++)
		isp_write_reg(0x30030040 + k * 4, (s32)CS16(0x1204 + k * 2));
	for (k = 0; k < 97; k++)
		isp_write_reg(0x30031000 + k * 4, C16(0x121c + k * 2));
	isp_write_reg(0x30033004, C32(0x12e4));
	isp_write_reg(0x30033008, C32(0x12e8));
	isp_write_reg(0x30033014, C32(0x12ec));
	isp_write_reg(0x30033018, C32(0x12e0));
	isp_write_reg(0x3002100c, C8(0x12f0));
	isp_write_reg(0x3002101c, C8(0x12f1));
	isp_write_reg(0x30021020, C8(0x12f2));
	isp_write_reg(0x30021024, C8(0x12f3));
	if (bpu_lsc())
		return;
	/* RGB -> YUV */
	for (k = 0; k < 3; k++) {
		isp_write_reg(0x30032000 + k * 4, C16(0x171c + k * 2) & 0x1fff);
		isp_write_reg(0x3003200c + k * 4, C16(0x1724 + k * 2) & 0x1fff);
		isp_write_reg(0x30032018 + k * 4, C16(0x172c + k * 2) & 0x1fff);
		isp_write_reg(0x30032024 + k * 4, C8(0x1722 + k * 8));
	}
	if (C32(0x1744) < 0x3fff) {
		isp_write_reg(0x30034000, C32(0x1740));
		isp_write_reg(0x30034004, C32(0x1744));
	}
	if (C32(0x174c) < 0x3fff) {
		isp_write_reg(0x30034008, C32(0x1748));
		isp_write_reg(0x3003400c, C32(0x174c));
	}
	r = C16(0x10e8);
	g = C16(0x10ea);
	b = C16(0x10ec);
	if ((mode == 0 || mode == 2) && !bpu_held) {
		isp_write_reg(0x30014000, C32(0x1128));
		isp_write_reg(0x30014004, 0xfff);
		for (base = 0x30012000; base != 0x30032000; base += 0x10000) {
			isp_write_reg(base, r);
			isp_write_reg(base + 4, g);
			isp_write_reg(base + 8, b);
			isp_write_reg(base + 0x3000, C32(0x112c));
			isp_write_reg(base + 0x3004, C32(0x1130));
			isp_write_reg(base + 0x3008, C32(0x1134));
			for (k = 0; k < 33; k++)
				isp_write_reg(0x30016000 + k * 4,
					      C32(0x113c + k * 4));
			for (k = 0; k < 17; k++)
				isp_write_reg(0x300160c0 + k * 4,
					      C32(0x11c0 + k * 4));
		}
	}
	if (C32(CM_IW_WAIT) & 1)
		bpu_held = 1;
}

int bpu_thread(void *unused)
{
	long t;

	C8(CM_BPU_BUSY) = 0;
	while (!kthread_should_stop()) {
		if (!isp_users_open() || !C32(CM_RUN)) {
			schedule_timeout_interruptible(msecs_to_jiffies(100));
			continue;
		}
		if (C8(CM_MODE) == 1) { /* offline: isp.ko does no work here */
			cond_resched();
			continue;
		}
		C8(CM_BPU_BUSY) = 1;
		if ((int)C32(CM_BPU_GO) < 1) {
			t = wait_event_interruptible_timeout(
				bpu_thread_wq, (int)C32(CM_BPU_GO) > 0, 100);
			if (!t)
				continue;
		}
		if (!C32(CM_BPU_GO))
			continue;
		C32(CM_BPU_GO) = 0;
		mutex_lock(&bpu_lock);
		bpu_update_regs();
		mutex_unlock(&bpu_lock);
		vpss_isp_set_val(C32(0xde0), C32(CM_VICU_SIZE) & 0xffff,
				 C32(CM_VICU_SIZE) >> 16);
		vpss_isp_post_deal(C32(0xde0), C32(CM_VICU_SIZE) & 0xffff,
				   C32(CM_VICU_SIZE) >> 16);
	}
	return 0;
}
