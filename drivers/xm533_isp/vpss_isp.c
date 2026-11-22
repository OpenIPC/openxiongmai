// SPDX-License-Identifier: GPL-2.0
/*
 * The ISP's VPSS side (the 0x30040000 window: noise reduction, sharpening,
 * the limit-up scaler): parameter blocks per channel (0x49d2..0x49f0), and
 * once a frame (from the BPU thread) a copy of them into isp.ko's register
 * image, which then goes to the registers. Exports vpss_isp_init,
 * vpss_isp_set_attr, vpss_isp_set_val and vpss_isp_post_deal for vi.ko.
 */

#include <linux/delay.h>
#include <linux/module.h>
#include <linux/sched.h>
#include <linux/uaccess.h>

#include "isp.h"

#define VP_SIZE 0x788
#define VS_SIZE 0x81c

/* g_vpss_isp_para[chn]: the blocks at isp.ko's offsets (0..0xc its lock) */
static u8 vp[2][VP_SIZE] __aligned(4);
static struct mutex vp_lock[2] = { __MUTEX_INITIALIZER(vp_lock[0]),
				    __MUTEX_INITIALIZER(vp_lock[1]) };
#define P32(c, o) (*(u32 *)(vp[c] + (o)))
#define P16(c, o) (*(u16 *)(vp[c] + (o)))

/* isp.ko's register image of the frame */
static u8 vs[VS_SIZE] __aligned(4);
#define S32(o) (*(u32 *)(vs + (o)))

static u8 vs_frames;     /* frames since the parameters came (to 0xff) */
static u8 vs_hold;       /* the VPSS group held (CM_VPSS_HOLD) */
static u8 vs_en[2] = { 0xff, 0xff };
static u8 cut_cnt[2];    /* frames left to set the crop */
static u8 lu_mode[2] = { 0xfe, 0 }; /* 0xfe: not yet set */
static int repair_wait = -50;

/* the image from the blocks: {image, block} */
static const u16 vs_copy1[][2] = {
	{ 0x03c, 0x090 }, { 0x00c, 0x028 }, { 0x014, 0x020 }, { 0x040, 0x094 },
	{ 0x004, 0x02c }, { 0x044, 0x098 }, { 0x04c, 0x0a0 }, { 0x038, 0x08c },
	{ 0x018, 0x024 }, { 0x050, 0x0a4 }, { 0x008, 0x030 }, { 0x054, 0x0a8 },
	{ 0x010, 0x01c }, { 0x034, 0x088 }, { 0x048, 0x09c }, { 0x000, 0x134 },
	{ 0x058, 0x0ac }, { 0x06c, 0x0c0 }, { 0x07c, 0x0d0 }, { 0x074, 0x0c8 },
	{ 0x070, 0x0c4 }, { 0x078, 0x0cc }, { 0x05c, 0x0b0 }, { 0x08c, 0x0e0 },
	{ 0x080, 0x0d4 }, { 0x084, 0x0d8 }, { 0x088, 0x0dc }, { 0x060, 0x0b4 },
	{ 0x094, 0x0e8 }, { 0x064, 0x0b8 }, { 0x068, 0x0bc }, { 0x090, 0x0e4 },
	{ 0x098, 0x0ec },
};

static const u16 vs_copy2[][2] = {
	{ 0x108, 0x060 }, { 0x030, 0x048 }, { 0x024, 0x038 }, { 0x0fc, 0x054 },
	{ 0x028, 0x040 }, { 0x114, 0x068 }, { 0x104, 0x05c }, { 0x10c, 0x064 },
	{ 0x01c, 0x034 }, { 0x020, 0x03c }, { 0x02c, 0x044 }, { 0x100, 0x058 },
	{ 0x118, 0x074 }, { 0x0f4, 0x138 }, { 0x11c, 0x06c }, { 0x0f8, 0x050 },
	{ 0x120, 0x070 }, { 0x12c, 0x080 }, { 0x140, 0x150 }, { 0x148, 0x158 },
	{ 0x150, 0x160 }, { 0x110, 0x13c }, { 0x128, 0x07c }, { 0x134, 0x144 },
	{ 0x158, 0x168 }, { 0x124, 0x078 }, { 0x138, 0x148 }, { 0x13c, 0x14c },
	{ 0x144, 0x154 }, { 0x14c, 0x15c }, { 0x154, 0x164 }, { 0x130, 0x084 },
	{ 0x174, 0x190 }, { 0x15c, 0x178 }, { 0x17c, 0x198 }, { 0x178, 0x194 },
	{ 0x160, 0x17c }, { 0x180, 0x19c }, { 0x184, 0x1a0 }, { 0x168, 0x184 },
	{ 0x188, 0x1a4 }, { 0x18c, 0x1a8 }, { 0x16c, 0x188 }, { 0x170, 0x18c },
	{ 0x190, 0x1ac }, { 0x194, 0x1b0 }, { 0x164, 0x180 }, { 0x198, 0x1b4 },
	{ 0x19c, 0x1b8 }, { 0x1a0, 0x1bc }, { 0x1a4, 0x1c0 }, { 0x1b0, 0x1cc },
	{ 0x1b4, 0x1d0 }, { 0x1ac, 0x1c8 }, { 0x1a8, 0x1c4 }, { 0x1b8, 0x1d4 },
	{ 0x1bc, 0x1d8 },
};

static const u16 vs_copy3[][2] = {
	{ 0x500, 0x488 }, { 0x504, 0x48c }, { 0x508, 0x490 }, { 0x4fc, 0x484 },
	{ 0x50c, 0x494 }, { 0x518, 0x4a0 }, { 0x510, 0x498 }, { 0x51c, 0x4a4 },
	{ 0x514, 0x49c }, { 0x520, 0x4a8 }, { 0x524, 0x4ac }, { 0x528, 0x4b0 },
	{ 0x52c, 0x4b4 }, { 0x53c, 0x4c4 }, { 0x534, 0x4bc }, { 0x540, 0x4c8 },
	{ 0x6c4, 0x4cc }, { 0x530, 0x4b8 }, { 0x538, 0x4c0 }, { 0x6c8, 0x4d0 },
};

static const u16 vs_copy4[][2] = {
	{ 0x798, 0x664 }, { 0x7a0, 0x66c }, { 0x7a4, 0x670 }, { 0x6cc, 0x4d4 },
	{ 0x79c, 0x668 }, { 0x6d0, 0x658 }, { 0x6d4, 0x65c }, { 0x6d8, 0x660 },
	{ 0x7c8, 0x674 },
};

/* the image to the registers: {register, image} */
static const u32 vs_regs1[][2] = {
	{ 0x30042370, 0x010 }, { 0x30042374, 0x018 }, { 0x300422d8, 0x01c },
	{ 0x300422dc, 0x030 }, { 0x30042388, 0x028 }, { 0x30045000, 0x0f8 },
	{ 0x30045008, 0x0fc }, { 0x3004500c, 0x100 }, { 0x30045010, 0x104 },
	{ 0x30045018, 0x108 }, { 0x3004501c, 0x10c }, { 0x30044000, 0x114 },
	{ 0x30044010, 0x118 }, { 0x30044020, 0x11c }, { 0x30044024, 0x120 },
	{ 0x30044028, 0x124 }, { 0x3004402c, 0x128 }, { 0x3004237c, 0x014 },
	{ 0x300422e4, 0x020 }, { 0x30044058, 0x12c }, { 0x3004405c, 0x130 },
	{ 0x3004202c, 0x034 }, { 0x30042030, 0x038 }, { 0x300420b0, 0x03c },
	{ 0x300420b4, 0x03c }, { 0x300420bc, 0x040 }, { 0x300420c0, 0x044 },
	{ 0x300420c4, 0x044 }, { 0x300421e8, 0x048 }, { 0x30042234, 0x04c },
	{ 0x30042240, 0x050 }, { 0x300422a4, 0x054 }, { 0x300422a8, 0x058 },
	{ 0x300422ac, 0x05c }, { 0x300422c0, 0x060 }, { 0x300422c4, 0x064 },
	{ 0x300422c8, 0x068 }, { 0x300422d0, 0x06c }, { 0x3004239c, 0x070 },
	{ 0x300423a0, 0x074 }, { 0x300423a4, 0x078 }, { 0x300423a8, 0x07c },
	{ 0x30042404, 0x080 }, { 0x30042408, 0x084 }, { 0x3004240c, 0x088 },
	{ 0x30042410, 0x08c }, { 0x30042470, 0x090 }, { 0x30042474, 0x094 },
	{ 0x30042478, 0x098 }, { 0x30042480, 0x0e0 }, { 0x30042484, 0x0e4 },
	{ 0x30042488, 0x0e8 }, { 0x3004248c, 0x0ec }, { 0x30042490, 0x0f0 },
};

static const u32 vs_regs2[][2] = {
	{ 0x30046030, 0x138 }, { 0x30046034, 0x13c }, { 0x30046048, 0x140 },
	{ 0x3004604c, 0x144 }, { 0x30046090, 0x148 }, { 0x300460b0, 0x14c },
	{ 0x30046104, 0x150 }, { 0x30046108, 0x154 }, { 0x3004610c, 0x158 },
	{ 0x30046000, 0x134 }, { 0x30048000, 0x15c }, { 0x3004801c, 0x160 },
	{ 0x30049000, 0x168 }, { 0x30049200, 0x16c }, { 0x30049204, 0x170 },
	{ 0x30049208, 0x174 }, { 0x3004920c, 0x178 }, { 0x30049210, 0x17c },
	{ 0x30049214, 0x180 }, { 0x30049218, 0x184 }, { 0x3004921c, 0x188 },
	{ 0x30049220, 0x18c }, { 0x30049224, 0x190 }, { 0x30049228, 0x194 },
	{ 0x3004922c, 0x198 }, { 0x30049230, 0x19c }, { 0x30049234, 0x1a0 },
	{ 0x30049240, 0x1a4 }, { 0x30049244, 0x1a8 }, { 0x30049248, 0x1ac },
	{ 0x3004924c, 0x1b0 }, { 0x30049250, 0x1b4 }, { 0x30049254, 0x1b8 },
	{ 0x30049258, 0x1bc },
};

static const u32 vs_regs3[][2] = {
	{ 0x30047030, 0x4fc }, { 0x30047034, 0x500 }, { 0x30047038, 0x504 },
	{ 0x300470d0, 0x508 }, { 0x300470d4, 0x50c }, { 0x300470d8, 0x510 },
	{ 0x300470dc, 0x514 }, { 0x300470e0, 0x518 }, { 0x300470e4, 0x51c },
	{ 0x30047180, 0x520 }, { 0x30047184, 0x524 }, { 0x30047188, 0x528 },
	{ 0x3004718c, 0x52c }, { 0x30047190, 0x530 }, { 0x30047194, 0x534 },
	{ 0x30047248, 0x538 }, { 0x3004724c, 0x53c },
};

static const u32 vs_regs4[][2] = {
	{ 0x30047000, 0x6cc }, { 0x30040010, 0x6e4 }, { 0x30040014, 0x6e8 },
	{ 0x30040028, 0x6ec }, { 0x3004002c, 0x6f0 }, { 0x30040018, 0x6f4 },
	{ 0x3004001c, 0x6f8 }, { 0x30040020, 0x6fc }, { 0x30040024, 0x700 },
	{ 0x30040038, 0x704 }, { 0x3004003c, 0x708 }, { 0x30040040, 0x70c },
	{ 0x300400e0, 0x710 }, { 0x300400e4, 0x714 },
};

static const u32 vs_regs5[][2] = {
	{ 0x30041020, 0x7a8 }, { 0x30041024, 0x7ac }, { 0x30041038, 0x7b0 },
	{ 0x3004103c, 0x7b4 }, { 0x30041028, 0x7b8 }, { 0x3004102c, 0x7bc },
	{ 0x30041030, 0x7c0 }, { 0x30041034, 0x7c4 },
};

/* the limit-up scaler */
static const u32 vs_regs_lu[][2] = {
	{ 0x3006c008, 0x7d0 }, { 0x3006c00c, 0x7d4 }, { 0x3006d010, 0x7e4 },
	{ 0x3006d01c, 0x7f0 }, { 0x3006d020, 0x7f4 }, { 0x3006d028, 0x7fc },
	{ 0x3006d004, 0x7dc }, { 0x3006d00c, 0x7e0 }, { 0x3006d024, 0x7f8 },
	{ 0x3006d02c, 0x800 },
};

#define WR_TABLE(t)                                                \
	do {                                                       \
		int _i;                                            \
		for (_i = 0; _i < ARRAY_SIZE(t); _i++)             \
			isp_write_reg((t)[_i][0], S32((t)[_i][1])); \
	} while (0)
#define CP_TABLE(t, c)                                             \
	do {                                                       \
		int _i;                                            \
		for (_i = 0; _i < ARRAY_SIZE(t); _i++)             \
			S32((t)[_i][0]) = P32(c, (t)[_i][1]);      \
	} while (0)

static const u16 nr_curve1[16] = { 0x60, 0x70, 0x80, 0x80, 0x90, 0x90,
				    0xa0, 0xa0, 0xa0, 0xb0, 0xc0, 0xd0,
				    0xe0, 0xf0, 0xff, 0xff };
static const u16 nr_curve2[16] = { 0x60, 0x60, 0x70, 0x70, 0x80, 0x80,
				    0x90, 0x90, 0xa0, 0xb0, 0xc0, 0xd0,
				    0xe0, 0xf0, 0xff, 0xff };
static const u8 sharp_tab[16] = { 4, 6, 8, 0x10, 0x10, 0x10, 0x10, 0x10,
				   0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10,
				   0x10 };
static const u8 gamma_tab[21] = { 0x3c, 0x91, 0xd2, 0xf0, 0xf8, 0xfb, 0xfe,
				   0x20, 0x40, 0x60, 0x80, 0xa0, 0xc0, 0xe0,
				   0x10, 0x1c, 0x1e, 0x37, 0x54, 0x78, 0xb4 };

static const u32 vpss_defaults1[][2] = {
	{ 0x30047004, 1 },    { 0x30047308, 0x14 }, { 0x3004730c, 0x1c },
	{ 0x30047310, 0x40 }, { 0x30047300, 0x16 }, { 0x30047304, 0xff },
	{ 0x30047018, 0x10 }, { 0x30047024, 6 },    { 0x30047244, 0 },
	{ 0x300472f0, 0xcf }, { 0x300472f4, 0xaf },
};

static const u32 vpss_defaults2[][2] = {
	{ 0x30042018, 0 },    { 0x30042020, 0x40 },  { 0x30042050, 0x40 },
	{ 0x30042060, 0x30 }, { 0x300421e8, 0xff },  { 0x30042264, 1 },
	{ 0x30042278, 0 },    { 0x3004227c, 0 },     { 0x30042280, 2 },
	{ 0x30042288, 0 },    { 0x3004228c, 0x70 },  { 0x30042290, 0x60 },
	{ 0x30042294, 0 },    { 0x300422b0, 0x28 },  { 0x300422b4, 0xe },
	{ 0x300422b8, 2 },    { 0x300422bc, 0x30 },  { 0x300422d4, 0x20 },
	{ 0x300422f0, 0 },    { 0x300422f4, 4 },     { 0x300422f8, 0x60 },
	{ 0x300422fc, 0x10 }, { 0x30042300, 0 },     { 0x30042304, 5 },
	{ 0x3004200c, 0x5a5f }, { 0x30042390, 0 },   { 0x30042394, 0xcc },
	{ 0x30042398, 0xcc }, { 0x30042400, 0 },     { 0x30042024, 0xb1e3150 },
	{ 0x3004504c, 0 },    { 0x30044074, 0 },     { 0x30044054, 3 },
	{ 0x3004205c, 0x20 }, { 0x30042480, 4 },     { 0x30042484, 6 },
	{ 0x30042488, 1 },    { 0x3004248c, 10 },    { 0x30042490, 10 },
	{ 0x30046004, 4 },    { 0x30046008, 7 },     { 0x30046040, 0 },
	{ 0x30046044, 0 },    { 0x300460b4, 0x10 },
};

static const u32 vpss_defaults3[][2] = {
	{ 0x3006d008, 100 },  { 0x3006d014, 3 },     { 0x3006d018, 0x20 },
	{ 0x30040030, 0 },    { 0x30040034, 1 },     { 0x30040038, 0x3c },
	{ 0x3004003c, 0x3c }, { 0x30040050, 0xff },  { 0x30040054, 100 },
	{ 0x30040058, 0x16 }, { 0x3004005c, 4 },     { 0x30040060, 0x28 },
	{ 0x30040064, 0x82 }, { 0x30040068, 0x7b8 }, { 0x3004006c, 0x1b7 },
	{ 0x300400e0, 0x104 }, { 0x300400e4, 0xa0 },
};

static const u32 vpss_defaults4[][2] = {
	{ 0x30041040, 0x60 },  { 0x30041088, 0x60 },  { 0x30041094, 0x96 },
	{ 0x30041098, 0x50 },  { 0x3004109c, 0x40 },  { 0x300410a0, 0xc },
	{ 0x300410a4, 0xc },   { 0x300410a8, 0x40 },  { 0x300410ac, 0x157 },
	{ 0x300410b0, 0x157 }, { 0x30041004, 0 },     { 0x30041000, 0 },
};

#define WR_CONST(t)                                                \
	do {                                                       \
		int _i;                                            \
		for (_i = 0; _i < ARRAY_SIZE(t); _i++)             \
			isp_write_reg((t)[_i][0], (t)[_i][1]);     \
	} while (0)

static void vpss_isp_defaults(void)
{
	int k;

	WR_CONST(vpss_defaults1);
	for (k = 0; k < 16; k++) {
		isp_write_reg(0x30047260 + k * 4, nr_curve1[k]);
		isp_write_reg(0x300472a0 + k * 4, nr_curve2[k]);
	}
	for (k = 0; k < ARRAY_SIZE(vpss_defaults2); k++) {
		isp_write_reg(vpss_defaults2[k][0], vpss_defaults2[k][1]);
		if (vpss_defaults2[k][0] == 0x300460b4) {
			int j;

			for (j = 0; j < 16; j++)
				isp_write_reg(0x30046050 + j * 4, sharp_tab[j]);
		}
	}
	WR_CONST(vpss_defaults3);
	for (k = 0; k < 21; k++)
		isp_write_reg(0x30040080 + k * 4, gamma_tab[k]);
	WR_CONST(vpss_defaults4);
}

int vpss_isp_init(void *regs)
{
	return 0;
}
EXPORT_SYMBOL(vpss_isp_init);

int vpss_isp_post_deal(u32 grp, u32 w, u32 h)
{
	return 0;
}
EXPORT_SYMBOL(vpss_isp_post_deal);

void vpss_init_queue(void)
{
}

/* vi.ko: channel 0's scaling-down flag (attr[3]) and crop (attr[4..7]) */
int vpss_isp_set_attr(u32 grp, u32 w, u32 h, u32 *attr)
{
	u32 down;

	if (!attr)
		return -1;
	down = attr[3] != 0;
	mutex_lock(&vpss_limitup_lock);
	C8(CM_FLAGS) = 0xc0 | down; /* isp.ko stores the byte whole */
	C32(CM_LIMITUP) = attr[4];
	C32(CM_LIMITUP + 4) = attr[5];
	C32(CM_LIMITUP + 8) = attr[6];
	C32(CM_LIMITUP + 12) = attr[7];
	pr_info("IspKo: vpss_isp_set_attr:%d %d %d %d %d\n", down, attr[4],
		attr[5], attr[6], attr[7]);
	mutex_unlock(&vpss_limitup_lock);
	return 0;
}
EXPORT_SYMBOL(vpss_isp_set_attr);

void vpss_isp_reset(int grp)
{
	if (grp == 0) {
		isp_write_reg(0x30042008, 0);
		isp_write_reg(0x30044034, 0);
		isp_write_reg(0x30045040, 0);
		isp_write_reg(0x30042000, 0);
		isp_write_reg(0x30041000, 0);
		isp_write_reg(0x30041004, 0);
	}
	vs_frames = 0;
	vs_hold = 0;
	vs_en[0] = 0xff;
	vs_en[1] = 0xff;
}

/* the image from channel c's blocks, w x h the picture: 0 or -1 */
static int vs_fill(u32 c, u32 w, u32 h, u32 frame_lines, u32 g)
{
	u32 nx = (w + 0x10) >> 5, ny = (h + 0x10) >> 5;
	u32 ex = 0, ey = 0, u, k, d, v;

	CP_TABLE(vs_copy1, c);
	P32(c, 0x28) = 0; /* taken once */
	u = P32(c, 0x140);
	S32(0xe0) = u & 0xf;
	S32(0xe4) = (u & 0xfff) >> 8;
	S32(0xe8) = (u & 0xffff) >> 15;
	S32(0xec) = (u & 0xffffff) >> 16;
	S32(0xf0) = u >> 24;
	for (k = 0; k < 17; k++)
		S32(0x9c + k * 4) = P32(c, 0xf0 + k * 4);
	CP_TABLE(vs_copy2, c);
	for (k = 0; k < 65; k++)
		S32(0x1c0 + k * 4) = P32(c, 0x1dc + k * 4);
	S32(0x2c4) = P32(c, 0x2e0);
	for (k = 0; k < 65; k++)
		S32(0x2c8 + k * 4) = P16(c, 0x2e4 + k * 2);
	for (k = 0; k < 9; k++)
		S32(0x3cc + k * 4) = P16(c, 0x366 + k * 2);
	CP_TABLE(vs_copy3, c);
	for (k = 0; k < 32; k++) {
		S32(0x544 + k * 4) = P32(c, 0x4d8 + k * 4);
		S32(0x5c4 + k * 4) = P32(c, 0x558 + k * 4);
		S32(0x644 + k * 4) = P32(c, 0x5d8 + k * 4);
	}
	CP_TABLE(vs_copy4, c);
	if (nx * 31 < w)
		ex = w - nx * 31;
	if (ny * 31 < h)
		ey = h - ny * 31;
	if (!ex || !ey || ex * ey * ny * nx == 0) {
		pr_err("IspKo:Err: %s line%d:size=0[%d %d %d %d]\n",
		       "vpss_isp_set_val", 0x520, nx, ny, ex, ey);
		return -1;
	}
	/* the noise reduction's grid, as the BPU's shading grid */
	S32(0x7ac) = ny - 1;
	S32(0x7a8) = nx - 1;
	S32(0x7b0) = 0x100000 / nx;
	S32(0x7b4) = 0x100000 / ny;
	S32(0x7b8) = 0x100000 / (ny * nx);
	S32(0x7bc) = 0x100000 / (ny * ex);
	S32(0x7c0) = 0x100000 / (nx * ey);
	S32(0x7c4) = 0x100000 / (ey * ex);
	S32(0x6e0) = P32(c, 0x6b4);
	S32(0x6e4) = nx - 1;
	S32(0x6e8) = ny - 1;
	S32(0x6ec) = S32(0x7b0);
	S32(0x6f0) = S32(0x7b4);
	S32(0x6f4) = S32(0x7b8);
	S32(0x6dc) = P32(c, 0x69c);
	S32(0x700) = S32(0x7c4);
	S32(0x6f8) = S32(0x7bc);
	S32(0x704) = P32(c, 0x6a0);
	S32(0x708) = P32(c, 0x6a4);
	S32(0x70c) = P32(c, 0x6a8);
	S32(0x6fc) = S32(0x7c0);
	S32(0x710) = P32(c, 0x6ac);
	S32(0x714) = P32(c, 0x6b0);
	for (k = 0; k < 16; k++) {
		S32(0x718 + k * 4) = P32(c, 0x6b8 + k * 4);
		S32(0x758 + k * 4) = P32(c, 0x6f8 + k * 4);
	}
	/* limit-up: the scaler's input and output */
	if (!(P32(c, 0x738) & 1)) {
		S32(0x7d4) = 0;
		S32(0x7cc) = 1;
		S32(0x7d0) = 0x400;
	} else {
		v = P32(c, 0x758);
		S32(0x7cc) = 0;
		S32(0x7d0) = (P32(c, 0x744) << 10) / (v ?: 1);
		S32(0x804) = P32(c, 0x76c);
		S32(0x808) = P32(c, 0x770);
		S32(0x80c) = P32(c, 0x774);
		S32(0x810) = P32(c, 0x778);
		S32(0x7d4) = v;
	}
	S32(0x814) = P32(c, 0x77c);
	if (!(P32(c, 0x738) & 0x10)) {
		S32(0x7e4) = 0;
		S32(0x7f0) = 0;
		S32(0x7f4) = 0;
		S32(0x7fc) = 0;
		S32(0x7e0) = 0;
		S32(0x7f8) = 0;
		S32(0x800) = 0;
		S32(0x7e8) = 0;
		S32(0x7ec) = 0;
		S32(0x7d8) = 1;
		S32(0x7dc) = 0x400;
	} else {
		u = isp_read_reg(g * 0x10000 + 0x30010094);
		if (!P32(c, 0x74c))
			P32(c, 0x74c) = 1;
		v = P32(c, 0x760) * (u & 0xffff) / P32(c, 0x74c);
		if (v > 0x3ffe)
			v = 0x3fff;
		if (!P32(c, 0x750))
			P32(c, 0x750) = 1;
		d = P32(c, 0x764) * frame_lines / P32(c, 0x750);
		S32(0x7f0) = P32(c, 0x75c);
		S32(0x7d8) = 0;
		S32(0x7fc) = P32(c, 0x754) - 1;
		S32(0x7f4) = d < 0x3fff ? d : 0x3fff;
		S32(0x7e4) = v;
		S32(0x7dc) = (P32(c, 0x748) << 10) / (S32(0x7f0) ?: 1);
		S32(0x800) = P32(c, 0x740);
		S32(0x7e0) = P32(c, 0x744);
		S32(0x804) = P32(c, 0x76c);
		S32(0x808) = P32(c, 0x770);
		S32(0x7f8) = P32(c, 0x754) != S32(0x800);
		S32(0x80c) = P32(c, 0x774);
		S32(0x810) = P32(c, 0x778);
		if (!S32(0x7f8))
			S32(0x800) = 0;
		S32(0x7e8) = P32(c, 0x768) & 0xffff;
		S32(0x7ec) = P32(c, 0x768) >> 16;
	}
	return 0;
}

static void vs_write(u32 c)
{
	u32 k, v;

	WR_TABLE(vs_regs1);
	for (k = 0; k < 17; k++)
		isp_write_reg(0x300421f0 + k * 4, S32(0x9c + k * 4));
	vs_en[c] = (u8)S32(0);
	if (!vs_en[c]) {
		isp_write_reg(0x30042008, 0);
		isp_write_reg(0x30044034, 0);
		isp_write_reg(0x30045040, 0);
	} else {
		isp_write_reg(0x30044034, S32(0x110));
		isp_write_reg(0x30045040, S32(0xf4));
		isp_write_reg(0x30042008, 1);
	}
	isp_write_reg(0x30042000, (u8)S32(0));
	v = (s32)S32(0) < 0 ? (S32(0) & 0x7fffffff) >> 24 : 0;
	isp_write_reg(0x30042600, v);
	WR_TABLE(vs_regs2);
	for (k = 0; k < 65; k++)
		isp_write_reg(0x300490e4 + k * 4, S32(0x1c0 + k * 4));
	isp_write_reg(0x30049018, S32(0x2c4));
	for (k = 0; k < 65; k++)
		isp_write_reg(0x3004a000 + k * 4, S32(0x2c8 + k * 4));
	for (k = 0; k < 9; k++)
		isp_write_reg(0x3004a800 + k * 4, S32(0x3cc + k * 4));
	WR_TABLE(vs_regs3);
	v = S32(0x540) ? (S32(0x540) & 0xff) | 0x200 : 0;
	isp_write_reg(0x30029014, v);
	for (k = 0; k < 32; k++) {
		isp_write_reg(0x3004703c + k * 4, S32(0x544 + k * 4));
		isp_write_reg(0x300470e8 + k * 4, S32(0x5c4 + k * 4));
		isp_write_reg(0x300471b0 + k * 4, S32(0x644 + k * 4));
	}
	WR_TABLE(vs_regs4);
	for (k = 0; k < 16; k++) {
		isp_write_reg(0x30040100 + k * 4, S32(0x718 + k * 4));
		isp_write_reg(0x30040140 + k * 4, S32(0x758 + k * 4));
	}
	isp_write_reg(0x30040000, S32(0x6dc) != 0);
	WR_TABLE(vs_regs5);
	isp_write_reg(0x30041004, 1);
	isp_write_reg(0x300410c0, S32(0x79c));
	if (vs_frames > 1)
		isp_write_reg(0x30041000, S32(0x798));
}

/* the limit-up scaler and channel 0's crop (vpss_isp_set_attr) */
static void vs_limitup(u32 c)
{
	u32 x = 0, y = 0, w = 0xffffff, h = 0xffffff, sz, cr;
	u8 f, en, cnt;

	if (lu_mode[0] == 0xfe) {
		lu_mode[0] = 0xff;
		lu_mode[1] = 0xff;
		cut_cnt[0] = 0;
		cut_cnt[1] = 0;
	}
	mutex_lock(&vpss_limitup_lock);
	f = C8(CM_FLAGS);
	if (f & 0x80) {
		x = C32(CM_LIMITUP);
		y = C32(CM_LIMITUP + 4);
		w = C32(CM_LIMITUP + 8);
		h = C32(CM_LIMITUP + 12);
	}
	C8(CM_FLAGS) = f & 0x9f;
	mutex_unlock(&vpss_limitup_lock);
	sz = (f >> 6) & 1;
	cr = (f >> 5) & 1;
	if (C8(CM_MODE) == 1) {
		isp_write_reg(0x3006c000, 1);
		isp_write_reg(0x3006d000, 1);
		return;
	}
	if ((f & 0x80) && !(f & 1)) {
		/* a new crop, no scaling down: the scaler bypassed */
		S32(0x7cc) = 1;
		S32(0x7d8) = 1;
		en = 0;
	} else {
		en = S32(0x7cc) ? 0x10 : 0x11;
		if (S32(0x7d8))
			en &= 0xef;
	}
	WR_TABLE(vs_regs_lu);
	if (S32(0x7e8))
		isp_write_reg(0x3006d014, S32(0x7e8));
	if (S32(0x7ec))
		isp_write_reg(0x3006d018, S32(0x7ec));
	if (cr == 0 && sz == 0) {
		cnt = cut_cnt[c];
		if (cnt == 1) {
			if (C32(CM_PRODUCT) == 1)
				goto done;
			isp_write_reg(0x30061000, x);
			isp_write_reg(0x30061004, w + x);
			isp_write_reg(0x30061008, y);
			isp_write_reg(0x3006100c, h + y);
			pr_info("IspKo: set cut 0x%x 0x%x 0x%x 0x%x\n", x, w + x,
				y, h + y);
			cnt = cut_cnt[c];
		}
		if (cnt == 0)
			return;
	} else {
		lu_mode[c] = en;
		if (C32(CM_PRODUCT) != 1) {
			isp_write_reg(0x30061008, 0);
			isp_write_reg(0x30061000, 0);
			isp_write_reg(0x3006100c, 0xffffffff);
			isp_write_reg(0x30061004, 0xffffffff);
			pr_info("IspKo: clean cut!\n");
		}
		isp_write_reg(0x3006c000, S32(0x7cc));
		isp_write_reg(0x3006d000, S32(0x7d8));
		cnt = 2;
	}
done:
	cut_cnt[c] = cnt - 1;
}

/* the limit-up scaler stuck: kick it, or the PLL behind it */
static void vs_repair(u32 c)
{
	u32 a, b, ra, rb, da, db, u, m;

	if (C32(CM_REPAIR) == 0xffffffff) {
		C32(CM_REPAIR) = 0;
		repair_wait = -50;
	}
	if (isp_read_reg(0x3006d000) != 0 || !P32(c, 0x77c))
		return;
	if (repair_wait < 0) {
		repair_wait++;
		return;
	}
	a = isp_read_reg(0x3006d020);
	b = isp_read_reg(0x3006d01c);
	ra = isp_read_reg(0x3006c014);
	rb = isp_read_reg(0x3006c01c);
	da = ra > a ? ra - a : a - ra;
	db = rb > b ? rb - b : b - rb;
	a = a / 20 ?: 1;
	b = b / 20 ?: 1;
	if (da <= a && db <= b) {
		if (db < 5)
			return;
		if (repair_wait + 1 > 10) {
			repair_wait = -50;
			isp_write_reg(0x3006d034, 1);
			if (C32(CM_REPAIR) < 0xfffe0000)
				C32(CM_REPAIR) += 0x10000;
			return;
		}
	} else if (repair_wait + 1 > 10) {
		repair_wait = -50;
		pll_write_reg(0x20000000, 1);
		u = pll_read_reg(0x20000050);
		m = ((u & 0xffffff) >> 16) * 2 + 1;
		if (m > 0xfe)
			m = 0xff;
		pll_write_reg(0x20000050, (u & 0xff00ffff) | m << 16);
		schedule_timeout_interruptible(msecs_to_jiffies(100));
		pll_write_reg(0x20000050, u);
		pll_write_reg(0x20000000, 0);
		if ((C32(CM_REPAIR) & 0xffff) < 0xfffe)
			C32(CM_REPAIR)++;
		return;
	}
	repair_wait++;
}

/* once a frame, from the BPU thread: 0, -1 not set up, -2 not this group */
int vpss_isp_set_val(u32 grp, u32 w, u32 h)
{
	u32 g;

	if (C32(CM_RUN) != 1)
		return -1;
	g = C32(0xde0);
	if (!(C8(CM_VPSS_EN) & 1) || g != grp || (int)g > 1)
		return -2;
	mutex_lock(&vp_lock[g]);
	if (vs_fill(g, w, h, C32(CM_CHN_ATTR + g * 0x1c + 8), g)) {
		mutex_unlock(&vp_lock[g]);
		return -1;
	}
	mutex_unlock(&vp_lock[g]);
	if (C32(CM_VPSS_HOLD) == 0xff) {
		isp_write_reg(0x30044034, 0);
		isp_write_reg(0x30045040, 0);
		isp_write_reg(0x30042008, 0);
		return 0;
	}
	if (vs_frames != 0xff)
		vs_frames++;
	if (C32(CM_VPSS_HOLD) == 0)
		vs_hold = 0;
	else if (vs_hold)
		return 0;
	if (C8(CM_MODE) == 1)
		vpss_isp_defaults();
	vs_write(g);
	vs_limitup(g);
	if (C8(CM_MODE) != 1)
		vs_repair(g);
	if (C32(CM_VPSS_HOLD) & (1 << g))
		vs_hold = 1;
	return 0;
}
EXPORT_SYMBOL(vpss_isp_set_val);

/* plain copies after the channel word: {cmd, size, block offset, bytes} */
static const struct {
	u16 cmd, size, dst, len;
} vp_copy[] = {
	{ 0x49db, 0x38, 0x144, 0x34 },  { 0x49dc, 0x13c, 0x00c, 0x138 },
	{ 0x49de, 0x10, 0x178, 0xc },   { 0x49df, 0x88, 0x2e4, 0x82 },
	{ 0x49e0, 0x18, 0x366, 0x12 },  { 0x49e2, 0x10, 0x658, 0xc },
	{ 0x49e3, 0x18, 0x664, 0x14 },  { 0x49e4, 0x164, 0x184, 0x160 },
	{ 0x49e6, 0x1d8, 0x484, 0x54 }, { 0x49e7, 0x54, 0x738, 0x50 },
	{ 0x49e8, 0xa0, 0x69c, 0x9c },  { 0x49f0, 0x184, 0x4d8, 0x180 },
};

/* 0x49d2: the blocks to their defaults (and the registers, once) */
static int vp_init(void)
{
	int c, k;

	memset(vp, 0, sizeof(vp));
	for (c = 0; c < 2; c++) {
		for (k = 0; k < 9; k++)
			P16(c, 0x366 + k * 2) = 0x100;
		P32(c, 0x178) = 0x100;
		P32(c, 0x17c) = 0x100;
		P32(c, 0x180) = 0x100;
		P32(c, 0x658) = 0x100;
		P32(c, 0x65c) = 0x100;
		P32(c, 0x660) = 0x100;
		P32(c, 0x738) = 0;
	}
	if (CS8(CM_OPT) >= 0)
		vpss_isp_defaults();
	return 0;
}

int iw_vpss_ioctl(unsigned int cmd, void __user *arg)
{
	static u32 w[0x1d8 / 4];
	const u8 *b = (const u8 *)w;
	int i, ret = -1;

	if (cmd == 0x49d2)
		return vp_init();
	mutex_lock(&iwctrl_vpss_lock);
	if (cmd == 0x49d7) { /* 9 words back */
		if (copy_from_user(w, arg, 0x28))
			goto out;
		if (w[0] < 2)
			memcpy(&w[1], vp[w[0]] + 0x678, 0x24);
		if (copy_to_user(arg, w, 0x28))
			goto out;
		ret = 0;
		goto out;
	}
	for (i = 0; i < ARRAY_SIZE(vp_copy); i++) {
		if (vp_copy[i].cmd != cmd)
			continue;
		if (copy_from_user(w, arg, vp_copy[i].size))
			goto out;
		ret = 0;
		if (w[0] >= 2) /* isp.ko takes 0x49db for any channel */
			goto out;
		mutex_lock(&vp_lock[w[0]]);
		memcpy(vp[w[0]] + vp_copy[i].dst, b + 4, vp_copy[i].len);
		mutex_unlock(&vp_lock[w[0]]);
		goto out;
	}
out:
	mutex_unlock(&iwctrl_vpss_lock);
	return ret;
}

static const char *const lu_name[] = {
	[0x10] = "1M->2M", [0x11] = "1M->3M", [0x12] = "1M->4M",
	[0x13] = "1M->5M", [0x20] = "2M->3M", [0x21] = "2M->4M",
	[0x22] = "2M->5M", [0x23] = "2M->4K", [0x30] = "3M->4M",
	[0x31] = "3M->5M(16:9)", [0x32] = "3M->4K", [0x40] = "4M->5M(16:9)",
	[0x41] = "4M->4K", [0x50] = "5M(4:3)->4K", [0x51] = "5M(16:9)->4K",
};

void vpss_isp_proc_show(struct seq_file *m, int *n)
{
	u32 t = P32(0, 0x73c);

	seq_printf(m, "[%d.LimitUp_info]:\n", (*n)++);
	if (t == 0)
		seq_puts(m, "LimitUp:[Disable]\n");
	else if (t < ARRAY_SIZE(lu_name) && lu_name[t])
		seq_printf(m, "LimitUp:[Enable](%s ", lu_name[t]);
	else
		seq_printf(m, "LimitUp:[Enable](Unknow 0x%x ", t);
	seq_printf(m, "[%d*%d]->[%d*%d])\n", P32(0, 0x744), P32(0, 0x748),
		   P32(0, 0x758), P32(0, 0x75c));
	if (P32(0, 0x738)) {
		seq_puts(m, "chn\tTotalSize\tActiveSize\n");
		seq_printf(m, "%d\t%4d*%4d\t%4d*%4d\n", 0,
			   isp_read_reg(0x3006d040), isp_read_reg(0x3006d044),
			   isp_read_reg(0x3006d048), isp_read_reg(0x3006d04c));
		seq_printf(m, "%d\t%4d*%4d\t%4d*%4d\n", 0,
			   isp_read_reg(0x3006c010), isp_read_reg(0x3006c014),
			   isp_read_reg(0x3006c018), isp_read_reg(0x3006c01c));
		seq_printf(m, "%d\t%4d*%4d\t%4d*%4d\n", 0,
			   isp_read_reg(0x30061014), isp_read_reg(0x3006101c),
			   isp_read_reg(0x30061010), isp_read_reg(0x30061018));
		seq_printf(m, "Repair: En:%d\tCnt:0x%08x\n", P32(0, 0x77c),
			   C32(CM_REPAIR));
	}
	seq_puts(m, "\n");
}
