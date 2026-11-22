// SPDX-License-Identifier: GPL-2.0
/*
 * The ISP's VPSS side (the 0x34000000 window: noise reduction, sharpening,
 * gamma, the lens correction (LDC), the limit-up scaler): parameter blocks
 * per group (0x49d2..0x49f0), and once a frame (from vpss.ko, or the BPU
 * thread when online) a copy of them into isp.ko's register image, which
 * then goes to the registers. Exports vpss_isp_init, vpss_isp_set_attr,
 * vpss_isp_set_val and vpss_isp_post_deal for vpss.ko.
 */

#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/uaccess.h>

#include "isp.h"

#define VP_SIZE 0x7c8
#define VS_SIZE 0x840

/* g_vpss_isp_para[grp]: the blocks at isp.ko's offsets (0..0x18 its lock) */
static u8 vp[N_CHN][VP_SIZE] __aligned(4);
static struct mutex vp_lock[N_CHN] = { __MUTEX_INITIALIZER(vp_lock[0]),
					__MUTEX_INITIALIZER(vp_lock[1]),
					__MUTEX_INITIALIZER(vp_lock[2]) };
#define P32(c, o) (*(u32 *)(vp[c] + (o)))
#define P16(c, o) (*(u16 *)(vp[c] + (o)))

/* isp.ko's register image of the frame */
static u8 vs[VS_SIZE] __aligned(4);
#define S32(o) (*(u32 *)(vs + (o)))

static void __iomem *ldc_buf[N_CHN]; /* 0x49e9's LDC table, 4 KiB */
static u8 ldc_new;     /* the groups with a new LDC table */
static u8 vs_held;     /* the parameters waited for (CM_VPSS_HOLD) and set */
static u8 vs_en[N_CHN] = { 0xff, 0xff, 0xff };
static u8 cut_cnt[N_CHN]; /* frames left to set the crop */
static u8 lu_mode[N_CHN] = { 0xfe, 0, 0 }; /* 0xfe: not yet set */
static int repair_wait = -50;

/* the image from a block: {image, block} */
static const u16 vs_copy[][2] = {
	{ 0x4, 0x38 },     { 0xc, 0x34 },     { 0x38, 0xa0 },    { 0x8, 0x3c },
	{ 0x40, 0xa8 },    { 0x48, 0xb0 },    { 0x10, 0x28 },    { 0x50, 0xb8 },
	{ 0x14, 0x2c },    { 0x0, 0x148 },    { 0x18, 0x30 },    { 0x3c, 0xa4 },
	{ 0x44, 0xac },    { 0x34, 0x9c },    { 0x4c, 0xb4 },    { 0x54, 0xbc },
	{ 0x58, 0xc0 },    { 0x78, 0xe0 },    { 0x80, 0xe8 },    { 0x68, 0xd0 },
	{ 0x70, 0xd8 },    { 0x88, 0xf0 },    { 0x5c, 0xc4 },    { 0x7c, 0xe4 },
	{ 0x84, 0xec },    { 0x90, 0xf8 },    { 0x60, 0xc8 },    { 0x64, 0xcc },
	{ 0x6c, 0xd4 },    { 0x74, 0xdc },    { 0x8c, 0xf4 },    { 0x98, 0x100 },
	{ 0x94, 0xfc },    { 0x1c, 0x40 },    { 0x20, 0x48 },    { 0x24, 0x44 },
	{ 0x28, 0x4c },    { 0x2c, 0x50 },    { 0x30, 0x54 },    { 0xf4, 0x94 },
	{ 0xf8, 0x5c },    { 0x104, 0x68 },   { 0xfc, 0x60 },    { 0x100, 0x64 },
	{ 0x108, 0x6c },   { 0x10c, 0x70 },   { 0x110, 0x98 },   { 0x114, 0x74 },
	{ 0x118, 0x80 },   { 0x120, 0x7c },   { 0x15c, 0x184 },  { 0x128, 0x88 },
	{ 0x124, 0x84 },   { 0x130, 0x90 },   { 0x12c, 0x8c },   { 0x138, 0x154 },
	{ 0x134, 0x150 },  { 0x140, 0x15c },  { 0x13c, 0x158 },  { 0x148, 0x164 },
	{ 0x144, 0x160 },  { 0x11c, 0x78 },   { 0x150, 0x16c },  { 0x14c, 0x168 },
	{ 0x160, 0x188 },  { 0x154, 0x170 },  { 0x158, 0x174 },  { 0x168, 0x190 },
	{ 0x164, 0x18c },  { 0x174, 0x19c },  { 0x170, 0x198 },  { 0x17c, 0x1a4 },
	{ 0x178, 0x1a0 },  { 0x184, 0x1ac },  { 0x180, 0x1a8 },  { 0x18c, 0x1b4 },
	{ 0x188, 0x1b0 },  { 0x16c, 0x194 },  { 0x194, 0x1bc },  { 0x190, 0x1b8 },
	{ 0x198, 0x1c0 },  { 0x19c, 0x1c4 },  { 0x1a4, 0x1cc },  { 0x1a0, 0x1c8 },
	{ 0x1a8, 0x1d0 },  { 0x1ac, 0x1d4 },  { 0x1b0, 0x1d8 },  { 0x1b4, 0x1dc },
	{ 0x1b8, 0x1e0 },  { 0x1bc, 0x1e4 },  { 0x2c4, 0x2ec },  { 0x470, 0x404 },
	{ 0x500, 0x494 },  { 0x4f8, 0x48c },  { 0x4fc, 0x490 },  { 0x474, 0x408 },
	{ 0x504, 0x498 },  { 0x514, 0x4a8 },  { 0x508, 0x49c },  { 0x50c, 0x4a0 },
	{ 0x510, 0x4a4 },  { 0x518, 0x4ac },  { 0x528, 0x4bc },  { 0x51c, 0x4b0 },
	{ 0x520, 0x4b4 },  { 0x524, 0x4b8 },  { 0x52c, 0x4c0 },  { 0x534, 0x4c8 },
	{ 0x538, 0x4cc },  { 0x53c, 0x4d0 },  { 0x530, 0x4c4 },  { 0x6c4, 0x4d8 },
	{ 0x540, 0x4d4 },  { 0x6c8, 0x4dc },  { 0x6cc, 0x4e0 },  { 0x6d0, 0x664 },
	{ 0x6d4, 0x668 },  { 0x6d8, 0x66c },  { 0x798, 0x670 },  { 0x79c, 0x674 },
	{ 0x7a0, 0x678 },  { 0x7a4, 0x67c },  { 0x7c8, 0x680 },
};

/* {register, image} */
static const u32 vs_held_regs[][2] = {
	{ 0x34002370, 0x10 },  { 0x340022d8, 0x1c },  { 0x34002388, 0x28 },
	{ 0x3400c000, 0xf8 },  { 0x3400c008, 0xfc },  { 0x3400c00c, 0x100 },
	{ 0x3400c010, 0x104 }, { 0x3400c018, 0x108 }, { 0x3400c01c, 0x10c },
	{ 0x3400d000, 0x114 }, { 0x3400d010, 0x118 }, { 0x3400d020, 0x11c },
	{ 0x3400d024, 0x120 }, { 0x3400d028, 0x124 }, { 0x3400d02c, 0x128 },
	{ 0x3400e060, 0x7c8 }, { 0x3400e064, 0x7c8 }, { 0x34009004, 0x6e0 },
	{ 0x34009008, 0x6e0 },
};

static const u32 vs_regs1[][2] = {
	{ 0x34002370, 0x10 },  { 0x34002374, 0x18 },  { 0x340022d8, 0x1c },
	{ 0x340022dc, 0x30 },  { 0x34002388, 0x28 },  { 0x3400c000, 0xf8 },
	{ 0x3400c008, 0xfc },  { 0x3400c00c, 0x100 }, { 0x3400c010, 0x104 },
	{ 0x3400c018, 0x108 }, { 0x3400c01c, 0x10c }, { 0x3400d000, 0x114 },
	{ 0x3400d010, 0x118 }, { 0x3400d020, 0x11c }, { 0x3400d024, 0x120 },
	{ 0x3400d028, 0x124 }, { 0x3400d02c, 0x128 }, { 0x3400237c, 0x14 },
	{ 0x340022e4, 0x20 },  { 0x3400d058, 0x12c }, { 0x3400d05c, 0x130 },
	{ 0x3400202c, 0x34 },  { 0x34002030, 0x38 },  { 0x340020b0, 0x3c },
	{ 0x340020b4, 0x3c },  { 0x340020bc, 0x40 },  { 0x340020c0, 0x44 },
	{ 0x340020c4, 0x44 },  { 0x340021e8, 0x48 },  { 0x34002234, 0x4c },
	{ 0x34002240, 0x50 },  { 0x340022a4, 0x54 },  { 0x340022a8, 0x58 },
	{ 0x340022ac, 0x5c },  { 0x340022c0, 0x60 },  { 0x340022c4, 0x64 },
	{ 0x340022c8, 0x68 },  { 0x340022d0, 0x6c },  { 0x3400239c, 0x70 },
	{ 0x340023a0, 0x74 },  { 0x340023a4, 0x78 },  { 0x340023a8, 0x7c },
	{ 0x34002404, 0x80 },  { 0x34002408, 0x84 },  { 0x3400240c, 0x88 },
	{ 0x34002410, 0x8c },  { 0x34002470, 0x90 },  { 0x34002474, 0x94 },
	{ 0x34002478, 0x98 },  { 0x34002480, 0xe0 },  { 0x34002484, 0xe4 },
	{ 0x34002488, 0xe8 },  { 0x3400248c, 0xec },  { 0x34002490, 0xf0 },
};

static const u32 vs_regs2[][2] = {
	{ 0x34001030, 0x138 }, { 0x34001034, 0x13c }, { 0x34001048, 0x140 },
	{ 0x3400104c, 0x144 }, { 0x34001090, 0x148 }, { 0x340010b0, 0x14c },
	{ 0x34001104, 0x150 }, { 0x34001108, 0x154 }, { 0x3400110c, 0x158 },
	{ 0x34001000, 0x134 }, { 0x34005000, 0x15c }, { 0x34005004, 0x160 },
	{ 0x34005008, 0x164 }, { 0x34006000, 0x168 }, { 0x34006200, 0x16c },
	{ 0x34006204, 0x170 }, { 0x34006208, 0x174 }, { 0x3400620c, 0x178 },
	{ 0x34006210, 0x17c }, { 0x34006214, 0x180 }, { 0x34006218, 0x184 },
	{ 0x3400621c, 0x188 }, { 0x34006220, 0x18c }, { 0x34006224, 0x190 },
	{ 0x34006228, 0x194 }, { 0x3400622c, 0x198 }, { 0x34006230, 0x19c },
	{ 0x34006234, 0x1a0 }, { 0x34006240, 0x1a4 }, { 0x34006244, 0x1a8 },
	{ 0x34006248, 0x1ac }, { 0x3400624c, 0x1b0 }, { 0x34006250, 0x1b4 },
	{ 0x34006254, 0x1b8 }, { 0x34006258, 0x1bc },
};

static const u32 vs_regs3[][2] = {
	{ 0x34003030, 0x4fc }, { 0x34003038, 0x504 }, { 0x340030d0, 0x508 },
	{ 0x340030d4, 0x50c }, { 0x340030d8, 0x510 }, { 0x340030dc, 0x514 },
	{ 0x340030e0, 0x518 }, { 0x340030e4, 0x51c }, { 0x34003180, 0x520 },
	{ 0x34003184, 0x524 }, { 0x34003188, 0x528 }, { 0x3400318c, 0x52c },
	{ 0x34003190, 0x530 }, { 0x34003194, 0x534 }, { 0x34003248, 0x538 },
	{ 0x3400324c, 0x53c },
};

static const u32 vs_regs4[][2] = {
	{ 0x34003000, 0x6cc }, { 0x3400a000, 0x6d0 }, { 0x3400a004, 0x6d4 },
	{ 0x3400a008, 0x6d8 }, { 0x34009004, 0x6e0 }, { 0x34009008, 0x6e0 },
	{ 0x34009010, 0x6e4 }, { 0x34009014, 0x6e8 }, { 0x34009028, 0x6ec },
	{ 0x3400902c, 0x6f0 }, { 0x34009018, 0x6f4 }, { 0x3400901c, 0x6f8 },
	{ 0x34009020, 0x6fc }, { 0x34009024, 0x700 }, { 0x34009038, 0x704 },
	{ 0x3400903c, 0x708 }, { 0x34009040, 0x70c }, { 0x340090e4, 0x714 },
};

static const u32 vs_regs5[][2] = {
	{ 0x3400e020, 0x79c }, { 0x3400e024, 0x7a0 }, { 0x3400e02c, 0x7a4 },
	{ 0x3400e000, 0x7a8 }, { 0x3400e004, 0x7ac }, { 0x3400e008, 0x7b0 },
	{ 0x3400e00c, 0x7b4 }, { 0x3400e010, 0x7b8 }, { 0x3400e014, 0x7bc },
	{ 0x3400e018, 0x7c0 }, { 0x3400e01c, 0x7c4 }, { 0x3400e060, 0x7c8 },
	{ 0x3400e064, 0x7c8 },
};

static const u32 vs_regs6[][2] = {
	{ 0x3400e06c, 0x798 }, { 0x34020000, 0x818 }, { 0x34020004, 0x81c },
	{ 0x34020008, 0x820 }, { 0x3402000c, 0x824 }, { 0x34020010, 0x828 },
	{ 0x34020014, 0x82c }, { 0x34020018, 0x830 },
};

/* the limit-up scaler */
static const u32 vs_regs_lu[][2] = {
	{ 0x3408e008, 0x7d0 }, { 0x3408e00c, 0x7d4 }, { 0x3408f010, 0x7e4 },
	{ 0x3408f01c, 0x7f0 }, { 0x3408f020, 0x7f4 }, { 0x3408f028, 0x7fc },
	{ 0x3408f004, 0x7dc }, { 0x3408f00c, 0x7e0 }, { 0x3408f024, 0x7f8 },
	{ 0x3408f02c, 0x800 },
};

#define WR_IMG(t)                                         \
	do {                                              \
		int _i;                                   \
		for (_i = 0; _i < ARRAY_SIZE(t); _i++)    \
			isp_write_reg(t[_i][0], S32(t[_i][1])); \
	} while (0)
#define WR_CONST(t)                                       \
	do {                                              \
		int _i;                                   \
		for (_i = 0; _i < ARRAY_SIZE(t); _i++)    \
			isp_write_reg(t[_i][0], t[_i][1]); \
	} while (0)

static const u32 vpss_defaults1[][2] = {
	{ 0x34003004, 0 },    { 0x34003300, 0 },    { 0x34003304, 0xff },
	{ 0x34003308, 0x14 }, { 0x3400330c, 0x1c }, { 0x34003310, 0x40 },
	{ 0x34003300, 0 },    { 0x34003304, 0xff }, { 0x34003018, 0x10 },
	{ 0x34003024, 6 },    { 0x34003244, 2 },
};

static const u32 vpss_defaults2[][2] = {
	{ 0x34003034, 0xff },      { 0x34002018, 0 },      { 0x34002020, 0x20 },
	{ 0x34002050, 0x40 },      { 0x34002060, 0x30 },   { 0x340021e8, 0xff },
	{ 0x34002264, 1 },         { 0x34002278, 1 },      { 0x3400227c, 1 },
	{ 0x34002280, 3 },         { 0x34002288, 0 },      { 0x3400228c, 0x70 },
	{ 0x34002290, 0x60 },      { 0x34002294, 0 },      { 0x340022b0, 0x28 },
	{ 0x340022b4, 0xe },       { 0x340022b8, 2 },      { 0x340022bc, 0x30 },
	{ 0x340022d4, 0xff },      { 0x340022f0, 0 },      { 0x340022f4, 8 },
	{ 0x340022f8, 0x40 },      { 0x340022fc, 0x20 },   { 0x34002300, 0 },
	{ 0x34002304, 5 },         { 0x3400200c, 0x5a5f }, { 0x34002390, 0 },
	{ 0x34002394, 0xcc },      { 0x34002398, 0xcc },   { 0x34002400, 1 },
	{ 0x34002024, 0xb1e3150 }, { 0x34002010, 0 },      { 0x3400c04c, 0 },
	{ 0x3400d074, 0 },         { 0x3400d054, 3 },      { 0x3400205c, 0x20 },
	{ 0x34002480, 4 },         { 0x34002484, 6 },      { 0x34002488, 1 },
	{ 0x3400248c, 0xf },       { 0x34002490, 0xa },    { 0x34001004, 4 },
	{ 0x34001008, 7 },         { 0x34001040, 2 },      { 0x34001044, 2 },
	{ 0x340010b4, 2 },
};

static const u32 vpss_defaults3[][2] = {
	{ 0x3408f008, 0x64 },  { 0x3408f014, 3 },     { 0x3408f018, 0x20 },
	{ 0x34009030, 0 },     { 0x34009034, 1 },     { 0x34009050, 0xff },
	{ 0x34009054, 0x64 },  { 0x34009058, 0x16 },  { 0x3400905c, 4 },
	{ 0x34009060, 0x28 },  { 0x34009064, 0x82 },  { 0x34009068, 0x894 },
	{ 0x3400906c, 0x1b7 }, { 0x340090e0, 0x100 },
};

static const u32 vpss_defaults4[][2] = {
	{ 0x3400e030, 0x9b },  { 0x3400e034, 0x40 },  { 0x3400e038, 0xc8 },
	{ 0x3400e03c, 0xa },   { 0x3400e040, 0x31 },  { 0x3400e044, 0x340 },
	{ 0x3400e048, 0x9b },  { 0x3400e04c, 0x25 },
};

static const u16 nr_tab[2][16] = {
	{ 0x40, 0x40, 0x40, 0x40, 0x40, 0x60, 0x80, 0x90, 0xa0, 0x120, 0x120,
	  0xd0, 0xe0, 0xf0, 0xf0, 0xf0 },
	{ 0x30, 0x30, 0x30, 0x30, 0x40, 0x60, 0x80, 0x90, 0x100, 0x140, 0x1ff,
	  0xf0, 0xf0, 0xf0, 0xf0, 0xf0 },
};

static const u8 sharp_tab[16] = {
	0xc0, 0xa0, 0x80, 0x40, 0x20, 0x10, 0x10, 0x10,
	0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10,
};

static const u8 gamma_tab[21] = {
	0x3c, 0x91, 0xd2, 0xf0, 0xf8, 0xfb, 0xfe, 0x20, 0x40, 0x60, 0x80,
	0xa0, 0xc0, 0xe0, 0x10, 0x1c, 0x1e, 0x37, 0x54, 0x78, 0xb4,
};

static void vpss_isp_defaults(void)
{
	int i;

	WR_CONST(vpss_defaults1);
	for (i = 0; i < 16; i++) {
		isp_write_reg(0x34003260 + i * 4, nr_tab[0][i]);
		isp_write_reg(0x340032a0 + i * 4, nr_tab[1][i]);
	}
	WR_CONST(vpss_defaults2);
	for (i = 0; i < 16; i++)
		isp_write_reg(0x34001050 + i * 4, sharp_tab[i]);
	WR_CONST(vpss_defaults3);
	for (i = 0; i < 21; i++)
		isp_write_reg(0x34009080 + i * 4, gamma_tab[i]);
	WR_CONST(vpss_defaults4);
}

int vpss_isp_init(void *regs)
{
	return 0;
}
EXPORT_SYMBOL(vpss_isp_init);

/* the 64-bin histogram after the VPSS: max, mean, ... (0x49d7) */
int vpss_isp_post_deal(u32 grp, u32 w, u32 h)
{
	u32 d[64], a, b, max = 0, sa = 0, sd = 0, thr, avg, n = 0, s = 0;
	int i;

	if (grp >= N_CHN)
		return -1;
	for (i = 0; i < 64; i++) {
		a = isp_read_reg(0x3400e200 + i * 4);
		b = isp_read_reg(0x3400e300 + i * 4);
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
	P32(grp, 0x684) = max;
	P32(grp, 0x688) = avg;
	P32(grp, 0x68c) = avg * n;
	P32(grp, 0x690) = thr;
	P32(grp, 0x694) = s;
	return 0;
}
EXPORT_SYMBOL(vpss_isp_post_deal);

/* vpss.ko: the scaling-down flag (attr[3]) and the crop (attr[4..7]) */
int vpss_isp_set_attr(u32 grp, u32 w, u32 h, u32 *attr)
{
	if (!attr)
		return -1;
	mutex_lock(&vpss_limitup_lock);
	C8(CM_FLAGS) = 0xc0 | (attr[3] != 0); /* isp.ko stores the byte whole */
	C32(CM_LIMITUP) = attr[4];
	C32(CM_LIMITUP + 4) = attr[5];
	C32(CM_LIMITUP + 8) = attr[6];
	C32(CM_LIMITUP + 12) = attr[7];
	mutex_unlock(&vpss_limitup_lock);
	return 0;
}
EXPORT_SYMBOL(vpss_isp_set_attr);

void vpss_isp_reset(int grp)
{
	if (grp == 0) {
		isp_write_reg(0x34010020, 0);
		isp_write_reg(0x34011040, 0);
		isp_write_reg(0x34002008, 0);
		isp_write_reg(0x3400d034, 0);
		isp_write_reg(0x3400c040, 0);
		isp_write_reg(0x34002000, 0);
		isp_write_reg(0x3400e06c, 0);
		isp_write_reg(0x3400e068, 0);
		isp_write_reg(0x3400900c, 0);
		isp_write_reg(0x34009000, 0);
	}
	vs_held = 0;
	memset(vs_en, 0xff, sizeof(vs_en));
}

void vpss_isp_exit(void)
{
	int c;

	for (c = 0; c < N_CHN; c++) {
		if (ldc_buf[c])
			iounmap(ldc_buf[c]);
		ldc_buf[c] = NULL;
	}
}

/* the image of group c's blocks, w x h; under its lock */
static int vs_fill(u32 c, u32 w, u32 h, u32 ah, u8 mode)
{
	u32 bw = (w + 0x10) >> 5, bh = (h + 0x10) >> 5, lw, lh, v, d;
	int i;

	for (i = 0; i < ARRAY_SIZE(vs_copy); i++)
		S32(vs_copy[i][0]) = P32(c, vs_copy[i][1]);
	P32(c, 0x34) = 0; /* once */
	v = P32(c, 0x14c);
	S32(0xe0) = v & 0xf;
	S32(0xe4) = (v >> 8) & 0xf;
	S32(0xe8) = (v >> 15) & 1;
	S32(0xec) = (v >> 16) & 0xff;
	S32(0xf0) = v >> 24;
	for (i = 0; i < 17; i++)
		S32(0x9c + i * 4) = P32(c, 0x104 + i * 4);
	for (i = 0; i < 65; i++) {
		S32(0x1c0 + i * 4) = P32(c, 0x1e8 + i * 4);
		S32(0x2c8 + i * 4) = P16(c, 0x2f0 + i * 2);
	}
	for (i = 0; i < 9; i++)
		S32(0x3cc + i * 4) = P16(c, 0x372 + i * 2);
	for (i = 0; i < 16; i++) {
		S32(0x3f0 + i * 4) = P32(c, 0x384 + i * 4);
		S32(0x430 + i * 4) = P32(c, 0x3c4 + i * 4);
		S32(0x478 + i * 4) = P32(c, 0x40c + i * 4);
		S32(0x4b8 + i * 4) = P32(c, 0x44c + i * 4);
	}
	for (i = 0; i < 0x60; i++)
		S32(0x544 + i * 4) = P32(c, 0x4e4 + i * 4);

	/* the shading grid: 32 x 32 blocks, the last ones what is left */
	lw = w > bw * 31 ? w - bw * 31 : 0;
	lh = h > bh * 31 ? h - bh * 31 : 0;
	if (lw * lh * bh * bw == 0) {
		pr_err("IspKo:Err: %s line%d:size=0[%d %d %d %d]\n",
		       "vpss_isp_set_val", 0x72a, bw, bh, lw, lh);
		return -1;
	}
	S32(0x7a8) = bw - 1;
	S32(0x7ac) = bh - 1;
	S32(0x7b0) = 0x100000 / bw;
	S32(0x7b4) = 0x100000 / bh;
	S32(0x7b8) = 0x100000 / (bh * bw);
	S32(0x7bc) = 0x100000 / (bh * lw);
	S32(0x7c0) = 0x100000 / (bw * lh);
	S32(0x7c4) = 0x100000 / (lh * lw);
	for (i = 0; i < 7; i++) /* 0x6e4.. as 0x7a8.. */
		S32(0x6e4 + i * 4) = S32(0x7a8 + i * 4);
	S32(0x700) = S32(0x7c4);
	S32(0x6dc) = P32(c, 0x6a8);
	S32(0x6e0) = P32(c, 0x6c0);
	for (i = 0; i < 5; i++)
		S32(0x704 + i * 4) = P32(c, 0x6ac + i * 4);
	for (i = 0; i < 16; i++) {
		S32(0x718 + i * 4) = P32(c, 0x6c4 + i * 4);
		S32(0x758 + i * 4) = P32(c, 0x704 + i * 4);
	}

	/* the limit-up scaler (0x49e7: P 0x744..) */
	if (!(P32(c, 0x744) & 1)) {
		S32(0x7cc) = 1;
		S32(0x7d0) = 0x400;
		S32(0x7d4) = 0;
	} else {
		v = P32(c, 0x764);
		S32(0x7cc) = 0;
		S32(0x7d0) = (P32(c, 0x750) << 10) / (v ?: 1);
		S32(0x804) = P32(c, 0x778);
		S32(0x810) = P32(c, 0x784);
		S32(0x808) = P32(c, 0x77c);
		S32(0x80c) = P32(c, 0x780);
		S32(0x7d4) = v;
	}
	S32(0x814) = P32(c, 0x788);
	if (!(P32(c, 0x744) & 0x10)) {
		S32(0x7d8) = 1;
		S32(0x7dc) = 0x400;
		for (i = 0; i < 9; i++)
			S32(0x7e0 + i * 4) = 0;
	} else {
		v = isp_read_reg(c ? 0x30050094 : 0x30040094);
		if (!P32(c, 0x758))
			P32(c, 0x758) = 1;
		v = P32(c, 0x76c) * (v & 0xffff) / P32(c, 0x758);
		if (!P32(c, 0x75c))
			P32(c, 0x75c) = 1;
		d = P32(c, 0x770) * ah / P32(c, 0x75c);
		S32(0x7f0) = P32(c, 0x768);
		S32(0x7d8) = 0;
		S32(0x7fc) = P32(c, 0x760) - 1;
		S32(0x7f4) = d < 0x3fff ? d : 0x3fff;
		S32(0x7e4) = v > 0x3ffe ? 0x3fff : v;
		S32(0x7dc) = (P32(c, 0x754) << 10) / (S32(0x7f0) ?: 1);
		S32(0x7e0) = P32(c, 0x750);
		S32(0x800) = P32(c, 0x74c);
		S32(0x810) = P32(c, 0x784);
		S32(0x808) = P32(c, 0x77c);
		S32(0x80c) = P32(c, 0x780);
		S32(0x7f8) = P32(c, 0x760) != S32(0x800);
		if (!S32(0x7f8))
			S32(0x800) = 0;
		S32(0x804) = P32(c, 0x778);
		S32(0x7e8) = P32(c, 0x774) & 0xffff;
		S32(0x7ec) = P32(c, 0x774) >> 16;
	}

	/* the LDC (0x49e9: P 0x794..) */
	S32(0x828) = 0x10001;
	S32(0x834) = P32(c, 0x794);
	S32(0x81c) = P32(c, 0x7a0);
	S32(0x818) = P16(c, 0x798) | P32(c, 0x79c) << 16;
	S32(0x830) = P32(c, 0x7ac);
	S32(0x820) = P32(c, 0x7a4);
	S32(0x824) = P32(c, 0x7a8);
	S32(0x82c) = P32(c, 0x7b0);
	S32(0x83c) = 0;
	S32(0x838) = 0;
	if (S32(0x834))
		S32(0x83c) = 1; /* ldc_buf[c] */
	if (mode == 1) {
		S32(0x838) = 1;
	} else if (S32(0x834) && ldc_new >> c & 1) {
		S32(0x838) = 1;
		ldc_new &= ~(1 << c);
	}
	return 0;
}

static void vs_write(u32 c)
{
	u32 v;
	int i;

	WR_IMG(vs_regs1);
	for (i = 0; i < 17; i++)
		isp_write_reg(0x340021f0 + i * 4, S32(0x9c + i * 4));
	vs_en[c] = S32(0);
	if (!(S32(0) & 0xff)) {
		isp_write_reg(0x34002008, 0);
		isp_write_reg(0x3400d034, 0);
		isp_write_reg(0x3400c040, 0);
	} else {
		isp_write_reg(0x3400d034, S32(0x110));
		isp_write_reg(0x3400c040, S32(0xf4));
		isp_write_reg(0x34002008, 1);
	}
	isp_write_reg(0x34002000, S32(0) & 0xff);
	isp_write_reg(0x34002600, (int)S32(0) < 0 ? (S32(0) & 0x7fffffff) >> 24 : 0);
	WR_IMG(vs_regs2);
	for (i = 0; i < 65; i++)
		isp_write_reg(0x340060e4 + i * 4, S32(0x1c0 + i * 4));
	isp_write_reg(0x34006018, S32(0x2c4));
	for (i = 0; i < 65; i++)
		isp_write_reg(0x34007000 + i * 4, S32(0x2c8 + i * 4));
	for (i = 0; i < 9; i++)
		isp_write_reg(0x34007800 + i * 4, S32(0x3cc + i * 4));
	WR_IMG(vs_regs3);
	v = S32(0x540) ? (S32(0x540) & 0xff) | 0x200 : 0;
	isp_write_reg(0x32050014, v);
	for (i = 0; i < 32; i++) {
		isp_write_reg(0x3400303c + i * 4, S32(0x544 + i * 4));
		isp_write_reg(0x340030e8 + i * 4, S32(0x5c4 + i * 4));
		isp_write_reg(0x340031b0 + i * 4, S32(0x644 + i * 4));
	}
	WR_IMG(vs_regs4);
	for (i = 0; i < 16; i++) {
		isp_write_reg(0x34009100 + i * 4, S32(0x718 + i * 4));
		isp_write_reg(0x34009140 + i * 4, S32(0x758 + i * 4));
	}
	isp_write_reg(0x3400900c, S32(0x6dc) != 0);
	isp_write_reg(0x34009000, S32(0x6dc) != 0);
	WR_IMG(vs_regs5);
	isp_write_reg(0x3400e068, S32(0x798) != 0);
	WR_IMG(vs_regs6);
	if (S32(0x83c) && S32(0x834) && S32(0x838) && ldc_buf[c]) {
		isp_write_reg(0x3402001c, 1);
		isp_write_reg(0x34020020, 0);
		for (i = 0; i < 0x400; i++)
			isp_write_reg(0x34021000 + i * 4, readl(ldc_buf[c] + i * 4));
		isp_write_reg(0x34020020, 1);
	}
	isp_write_reg(0x3402001c, S32(0x834));
}

/* the limit-up scaler and the crop vpss.ko asked for (vpss_isp_set_attr) */
static void vs_limitup(u32 c)
{
	u32 x = 0, y = 0, w = 0xffffff, h = 0xffffff, a, t;
	u8 f, mode = 0, en = 1, k;
	bool changed;

	if (lu_mode[0] == 0xfe) {
		memset(lu_mode, 0xff, sizeof(lu_mode));
		memset(cut_cnt, 0, sizeof(cut_cnt));
	}
	mutex_lock(&vpss_limitup_lock);
	f = C8(CM_FLAGS);
	if (f & 0x80) {
		en = f & 1;
		x = C32(CM_LIMITUP);
		y = C32(CM_LIMITUP + 4);
		w = C32(CM_LIMITUP + 8);
		h = C32(CM_LIMITUP + 12);
		a = C32(CM_CHN + c * 0x28 + 0x20);
		if (a >> 16 >= h) {
			t = w + x;
			S32(0x7d8) = 1;
			if ((a & 0xffff) < t && t) {
				S32(0x7cc) = 0;
				S32(0x7d0) = ((a & 0xffff) << 10) / t;
				S32(0x7d4) = t;
				mode = 1;
			} else {
				S32(0x7cc) = 1;
				mode = 2;
			}
		}
	}
	changed = lu_mode[c] != mode;
	lu_mode[c] = mode;
	C8(CM_FLAGS) = f & 0x9f;
	mutex_unlock(&vpss_limitup_lock);
	if (C8(CM_MODE) == 1) {
		isp_write_reg(0x3408e000, 1);
		isp_write_reg(0x3408f000, 1);
		return;
	}
	if (!en) {
		S32(0x7cc) = 1;
		S32(0x7d8) = 1;
	}
	WR_IMG(vs_regs_lu);
	if (S32(0x7e8))
		isp_write_reg(0x3408f014, S32(0x7e8));
	if (S32(0x7ec))
		isp_write_reg(0x3408f018, S32(0x7ec));
	if (!changed && !(f & 0x60)) {
		/* the crop a frame after the scaler */
		k = cut_cnt[c];
		if (k == 1 && C32(CM_PRODUCT) != 1) {
			isp_write_reg(0x34081000, x);
			isp_write_reg(0x34081004, w + x);
			isp_write_reg(0x34081008, y);
			isp_write_reg(0x3408100c, h + y);
			pr_info("IspKo: set cut 0x%x 0x%x 0x%x 0x%x\n", x, w + x, y,
				h + y);
		}
		if (k)
			cut_cnt[c] = k - 1;
		return;
	}
	if (C32(CM_PRODUCT) != 1) {
		isp_write_reg(0x34081008, 0);
		isp_write_reg(0x34081000, 0);
		isp_write_reg(0x3408100c, 0xffffffff);
		isp_write_reg(0x34081004, 0xffffffff);
		pr_info("IspKo: clean cut!\n");
	}
	isp_write_reg(0x3408e000, S32(0x7cc));
	isp_write_reg(0x3408f000, S32(0x7d8));
	cut_cnt[c] = 1;
}

/* the limit-up scaler stuck: a pixel clock kick, or its own reset */
static void vs_repair(u32 c)
{
	u32 a, b, d1, d2, v, n;
	bool bad;

	if (C32(CM_REPAIR) == 0xffffffff) {
		C32(CM_REPAIR) = 0;
		repair_wait = -50;
	}
	if (isp_read_reg(0x3408f000) || !P32(c, 0x788))
		return;
	if (repair_wait < 0) {
		repair_wait++;
		return;
	}
	a = isp_read_reg(0x3408f020);
	b = isp_read_reg(0x3408f01c);
	d1 = isp_read_reg(0x3408e014);
	d2 = isp_read_reg(0x3408e01c);
	d1 = a < d1 ? d1 - a : a - d1;
	d2 = b < d2 ? d2 - b : b - d2;
	a = a / 20 ?: 1;
	b = b / 20 ?: 1;
	bad = d1 > a || d2 > b;
	if (!bad && d2 < 5)
		return;
	if (repair_wait + 1 <= 10) {
		repair_wait++;
		return;
	}
	repair_wait = -50;
	if (bad) {
		pll_write_reg(0x20000000, 1);
		v = pll_read_reg(0x20000050);
		n = ((v >> 16) & 0xff) * 2 + 1;
		if (n > 0xfe)
			n = 0xff;
		pll_write_reg(0x20000050, (v & 0xff00ffff) | n << 16);
		schedule_timeout_interruptible(msecs_to_jiffies(100));
		pll_write_reg(0x20000050, v);
		pll_write_reg(0x20000000, 0);
		if ((C32(CM_REPAIR) & 0xffff) < 0xfffe)
			C32(CM_REPAIR)++;
	} else {
		isp_write_reg(0x3408f034, 1);
		if (C32(CM_REPAIR) < 0xfffe0000)
			C32(CM_REPAIR) += 0x10000;
	}
}

/* vpss.ko, once a frame of group grp, w x h */
int vpss_isp_set_val(u32 grp, u32 w, u32 h)
{
	u8 mode = C8(CM_MODE);
	u32 i, ah;
	int ret;

	if (C32(CM_RUN) != 1)
		return -1;
	for (i = 0; i < N_CHN; i++)
		if (C8(CM_VPSS_MASK) >> i & 1 && C32(CM_GRP(i) + 0x14) == grp)
			break;
	if (i == N_CHN || grp >= N_CHN)
		return -2;
	ah = C32(CM_CHN_ATTR(grp) + 8);
	mutex_lock(&vp_lock[grp]);
	ret = vs_fill(grp, w, h, ah, mode);
	mutex_unlock(&vp_lock[grp]);
	if (ret)
		return ret;
	if (C32(CM_VPSS_HOLD) == 0xff) {
		isp_write_reg(0x3400e068, 0);
		isp_write_reg(0x3400d034, 0);
		isp_write_reg(0x3400c040, 0);
		isp_write_reg(0x34002008, 0);
		return 0;
	}
	if (C32(CM_VPSS_HOLD) == 0) {
		vs_held = 0;
	} else if (vs_held) {
		if (C8(CM_MODE) == 1)
			WR_IMG(vs_held_regs);
		return 0;
	}
	if (C8(CM_MODE) == 1)
		vpss_isp_defaults();
	vs_write(grp);
	vs_limitup(grp);
	if (C8(CM_MODE) != 1)
		vs_repair(grp);
	if (C32(CM_VPSS_HOLD) & 1 << grp)
		vs_held = 1;
	return 0;
}
EXPORT_SYMBOL(vpss_isp_set_val);

/*
 * Not in isp.ko: whether the BPU runs online (HDR) for group grp, its
 * output then going straight to VPSS's scalers and store, not to a vb.ko
 * buffer; the picture's height << 16 | width if so, else 0. The open VPSS
 * takes the frames from its store then (vpss.ko has no such path).
 */
u32 vpss_isp_online(u32 grp)
{
	if (C32(CM_RUN) != 1 || C8(CM_MODE) == 1 ||
	    C32(CM_GRP(0) + 0x14) != grp)
		return 0;
	return C32(CM_CHN + 0x20);
}
EXPORT_SYMBOL(vpss_isp_online);

/* {cmd, size, offset in the block, bytes} of the plain copies */
static const struct {
	u16 cmd, size, dst, len;
} vp_copy[] = {
	{ 0x49db, 0x38, 0x150, 0x34 },  { 0x49dc, 0x13c, 0x18, 0x138 },
	{ 0x49de, 0x10, 0x184, 0xc },   { 0x49df, 0x88, 0x2f0, 0x82 },
	{ 0x49e0, 0x18, 0x372, 0x12 },  { 0x49e2, 0x10, 0x664, 0xc },
	{ 0x49e3, 0x18, 0x670, 0x14 },  { 0x49e4, 0x164, 0x190, 0x160 },
	{ 0x49e5, 0x110, 0x384, 0x10c }, { 0x49e6, 0x58, 0x490, 0x54 },
	{ 0x49e7, 0x54, 0x744, 0x50 },  { 0x49e8, 0xa0, 0x6a8, 0x9c },
	{ 0x49f0, 0x184, 0x4e4, 0x180 },
};

static void vp_init(void)
{
	int c, i;

	memset(vp, 0, sizeof(vp));
	for (c = 0; c < N_CHN; c++) {
		for (i = 0; i < 9; i++)
			P16(c, 0x372 + i * 2) = 0x100;
		P32(c, 0x184) = 0x100;
		P32(c, 0x188) = 0x100;
		P32(c, 0x18c) = 0x100;
		P32(c, 0x664) = 0x100;
		P32(c, 0x668) = 0x100;
		P32(c, 0x66c) = 0x100;
		P32(c, 0x744) = 0;
	}
	vpss_isp_exit();
	ldc_new = 0;
	if (CS8(CM_OPT) >= 0)
		vpss_isp_defaults();
}

/* 0x49e9: the LDC's 11 words, and its 4 KiB table at word 9 */
static void vp_ldc(u32 c, const u32 *w)
{
	memcpy(vp[c] + 0x794, &w[1], 11 * 4);
	if (!w[1] || !w[9] || P32(c, 0x7c0))
		return;
	P32(c, 0x7c0) = w[9];
	if (ldc_buf[c])
		iounmap(ldc_buf[c]);
	ldc_buf[c] = ioremap_cached(w[9], 0x1000);
	if (!ldc_buf[c])
		pr_err("IspKo:Err: ioremap_cached ldc_buf failed!\n");
	ldc_new |= 1 << c;
}

static int vp_do(unsigned int cmd, void __user *arg, u32 *w)
{
	int i;

	switch (cmd) {
	case 0x49d7: /* the histogram back */
		if (copy_from_user(w, arg, 0x28))
			return -1;
		if (w[0] < N_CHN)
			memcpy(&w[1], vp[w[0]] + 0x684, 9 * 4);
		return copy_to_user(arg, w, 0x28) ? -1 : 0;
	case 0x49e9:
		if (copy_from_user(w, arg, 0x30))
			return -1;
		if (w[0] >= N_CHN)
			return 0;
		mutex_lock(&vp_lock[w[0]]);
		vp_ldc(w[0], w);
		mutex_unlock(&vp_lock[w[0]]);
		return 0;
	}
	for (i = 0; i < ARRAY_SIZE(vp_copy); i++) {
		if (vp_copy[i].cmd != cmd)
			continue;
		if (copy_from_user(w, arg, vp_copy[i].size))
			return -1;
		if (w[0] >= N_CHN)
			return 0;
		mutex_lock(&vp_lock[w[0]]);
		memcpy(vp[w[0]] + vp_copy[i].dst, &w[1], vp_copy[i].len);
		mutex_unlock(&vp_lock[w[0]]);
		return 0;
	}
	return -1;
}

int iw_vpss_ioctl(unsigned int cmd, void __user *arg)
{
	static u32 w[0x184 / 4];
	int ret;

	if (cmd == 0x49d2) {
		vp_init();
		return 0;
	}
	mutex_lock(&iwctrl_vpss_lock);
	ret = vp_do(cmd, arg, w);
	mutex_unlock(&iwctrl_vpss_lock);
	return ret;
}

static const struct {
	u8 type;
	const char *name;
} lu_names[] = {
	{ 0x10, "1M->2M" },       { 0x11, "1M->3M" },       { 0x12, "1M->4M" },
	{ 0x13, "1M->5M" },       { 0x20, "2M->3M" },       { 0x21, "2M->4M" },
	{ 0x22, "2M->5M" },       { 0x23, "2M->4K" },       { 0x30, "3M->4M" },
	{ 0x31, "3M->5M(16:9)" }, { 0x32, "3M->4K" },       { 0x40, "4M->5M(16:9)" },
	{ 0x41, "4M->4K" },       { 0x50, "5M(4:3)->4K" },  { 0x51, "5M(16:9)->4K" },
};

void vpss_isp_proc_show(struct seq_file *m, int *n)
{
	u32 t = P32(0, 0x748);
	int i;

	seq_printf(m, "[%d.LimitUp_info]:\n", (*n)++);
	if (!t) {
		seq_puts(m, "LimitUp:[Disable]\n");
	} else {
		for (i = 0; i < ARRAY_SIZE(lu_names); i++)
			if (lu_names[i].type == t)
				break;
		if (i < ARRAY_SIZE(lu_names))
			seq_printf(m, "LimitUp:[Enable](%s ", lu_names[i].name);
		else
			seq_printf(m, "LimitUp:[Enable](Unknow 0x%x ", t);
	}
	seq_printf(m, "[%d*%d]->[%d*%d])\n", P32(0, 0x750), P32(0, 0x754),
		   P32(0, 0x764), P32(0, 0x768));
	if (P32(0, 0x744)) {
		seq_puts(m, "chn\tTotalSize\tActiveSize\n");
		seq_printf(m, "%d\t%4d*%4d\t%4d*%4d\n", 0,
			   isp_read_reg(0x3408f040), isp_read_reg(0x3408f044),
			   isp_read_reg(0x3408f048), isp_read_reg(0x3408f04c));
		seq_printf(m, "%d\t%4d*%4d\t%4d*%4d\n", 0,
			   isp_read_reg(0x3408e010), isp_read_reg(0x3408e014),
			   isp_read_reg(0x3408e018), isp_read_reg(0x3408e01c));
		seq_printf(m, "%d\t%4d*%4d\t%4d*%4d\n", 0,
			   isp_read_reg(0x34081014), isp_read_reg(0x3408101c),
			   isp_read_reg(0x34081010), isp_read_reg(0x34081018));
		seq_printf(m, "Repair: En:%d\tCnt:0x%08x\n", P32(0, 0x788),
			   C32(CM_REPAIR));
	}
	seq_puts(m, "\n");
}
