// SPDX-License-Identifier: GPL-2.0
/*
 * The XM232's VPSS side of the ISP (0x60000..0x6dfff): its parameter blocks
 * (0x49d2..0x49ec), kept per group in isp.ko's layout (0x81c bytes, the
 * group's lock first), and each frame their register image (g_regs_val):
 * the scaler's zone steps from the output size, noise reduction, the LTM,
 * sharpening, the lens correction table. The first frame (or each frame
 * while the group's parameters are not held, 0x4960) writes all of it, the
 * rest only what moves.
 */

#include <linux/io.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#include "isp.h"

#define VP_SIZE 0x81c
#define NR_RV (2376 / 4)

static u8 vpss_para[N_CHN][VP_SIZE] __aligned(4);
static struct mutex vpss_lock[N_CHN];
static u32 rv[NR_RV]; /* isp.ko's g_regs_val */
static u8 ldc_tab_set;
/* isp.ko's g_vpss_tmp: the registers all written once; per group a
 * frame counter */
static u8 vpss_all_set;
static u8 vpss_delay[N_CHN];
u32 isp_ldc_buf;

#define VP(c, o) P32(vpss_para[c], o)
#define VP16(c, o) P16(vpss_para[c], o)
#define VP8(c, o) P8(vpss_para[c], o)
#define RV(o) rv[(o) / 4]

struct reg_val {
	u32 reg, val;
};

static const struct reg_val vpss_defaults[] = {
	{ 0x6101c, 0x190 }, { 0x61020, 0x16 }, { 0x61038, 0x28 },
	{ 0x6103c, 0x36 }, { 0x61040, 0x40 }, { 0x61044, 0x40 },
	{ 0x61048, 0x40 }, { 0x6104c, 0x40 }, { 0x61050, 0x40 },
	{ 0x61054, 0x40 }, { 0x61058, 0x40 }, { 0x6105c, 0x3a },
	{ 0x61060, 0x34 }, { 0x61064, 0x32 }, { 0x61068, 0x30 },
	{ 0x6106c, 0x2e }, { 0x61070, 0x2c }, { 0x61074, 0x2a },
	{ 0x61078, 0x28 }, { 0x6107c, 0x26 }, { 0x61080, 0x24 },
	{ 0x61084, 0x22 }, { 0x61088, 0x20 }, { 0x6108c, 0x1e },
	{ 0x61090, 0x1c }, { 0x61094, 0x1a }, { 0x61098, 0x18 },
	{ 0x6109c, 0x16 }, { 0x610a0, 0x14 }, { 0x610a4, 0x12 },
	{ 0x610a8, 0x10 }, { 0x610ac, 0xe }, { 0x610b0, 0xc },
	{ 0x610d0, 0x80 }, { 0x610d4, 0x80 }, { 0x610d8, 0x80 },
	{ 0x610dc, 0x80 }, { 0x610e0, 0x80 }, { 0x610e4, 0x80 },
	{ 0x610e8, 0x80 }, { 0x610ec, 0x80 }, { 0x610f0, 0x80 },
	{ 0x610f4, 0x80 }, { 0x610f8, 0x80 }, { 0x610fc, 0x80 },
	{ 0x61100, 0x80 }, { 0x61104, 0x80 }, { 0x61108, 0x80 },
	{ 0x6110c, 0x80 }, { 0x61110, 0x80 }, { 0x61114, 0x80 },
	{ 0x61118, 0x80 }, { 0x6111c, 0x80 }, { 0x61120, 0x80 },
	{ 0x61124, 0x80 }, { 0x61128, 0x80 }, { 0x6112c, 0x80 },
	{ 0x61130, 0x80 }, { 0x61134, 0x80 }, { 0x61138, 0x80 },
	{ 0x6113c, 0x80 }, { 0x61140, 0x80 }, { 0x61144, 0x80 },
	{ 0x61148, 0x80 }, { 0x6114c, 0x80 }, { 0x61150, 0x80 },
	{ 0x611d0, 0x73 }, { 0x611d4, 0x15 }, { 0x611d8, 0x28 },
	{ 0x611dc, 0x63 }, { 0x611e0, 0x15 }, { 0x611e4, 0x28 },
	{ 0x611e8, 0x0 }, { 0x611ec, 0x10 }, { 0x611f0, 0x73 },
	{ 0x611f4, 0x15 }, { 0x611f8, 0x28 }, { 0x611fc, 0xb4 },
	{ 0x61200, 0x15 }, { 0x61204, 0x28 }, { 0x61208, 0x0 },
	{ 0x6120c, 0x10 }, { 0x61210, 0xa0 }, { 0x61214, 0x15 },
	{ 0x61218, 0x28 }, { 0x6121c, 0x5a }, { 0x61220, 0x15 },
	{ 0x61224, 0x28 }, { 0x61228, 0x0 }, { 0x6122c, 0x10 },
	{ 0x61230, 0x65 }, { 0x61234, 0x15 }, { 0x61238, 0x28 },
	{ 0x6123c, 0x96 }, { 0x61240, 0x15 }, { 0x61244, 0x28 },
	{ 0x61248, 0x0 }, { 0x6124c, 0x10 }, { 0x61258, 0x2d8 },
	{ 0x6125c, 0x2d8 }, { 0x61260, 0x298 }, { 0x61264, 0x298 },
	{ 0x61268, 0x60 }, { 0x6126c, 0x60 },
};

static const struct reg_val vpss_defaults2[] = {
	{ 0x66008, 0x10 }, { 0x6600c, 0x10 }, { 0x66010, 0x5 },
	{ 0x66014, 0xa }, { 0x6601c, 0x2 }, { 0x66070, 0x8 },
	{ 0x66074, 0x4 }, { 0x66078, 0x3 }, { 0x66094, 0xff },
	{ 0x6609c, 0x1 }, { 0x660a4, 0x60 }, { 0x660a8, 0x60 },
	{ 0x66400, 0x1 }, { 0x66410, 0x80 }, { 0x66414, 0x1 },
	{ 0x66c84, 0x10 }, { 0x66c88, 0x8 }, { 0x66c90, 0x0 },
	{ 0x66084, 0x1 }, { 0x60008, 0x2 }, { 0x60014, 0x20 },
	{ 0x60018, 0x14 }, { 0x6001c, 0x2 }, { 0x60070, 0x8 },
	{ 0x60074, 0x4 }, { 0x60078, 0x3 }, { 0x60088, 0x4 },
	{ 0x6008c, 0x32 }, { 0x60094, 0xff }, { 0x600a0, 0x1 },
	{ 0x6040c, 0x0 }, { 0x60410, 0x0 }, { 0x60800, 0x1 },
	{ 0x600a4, 0x70 }, { 0x60808, 0x0 },
};

/* the LTM's curve (vpss_isp_init_ltm) */
static const struct reg_val vpss_ltm[] = {
	{ 0x62044, 0x60 }, { 0x62050, 0xfa }, { 0x62054, 0x23 },
	{ 0x62058, 0x64 }, { 0x6205c, 0x14 }, { 0x62060, 0x14 },
	{ 0x62064, 0x64 }, { 0x62068, 0x2ad }, { 0x6206c, 0x2ad },
	{ 0x62080, 0x5a }, { 0x62084, 0x91 }, { 0x62088, 0xc8 },
	{ 0x6208c, 0xdc }, { 0x62090, 0xe6 }, { 0x62094, 0xf0 },
	{ 0x62098, 0xfa }, { 0x6209c, 0x20 }, { 0x620a0, 0x40 },
	{ 0x620a4, 0x60 }, { 0x620a8, 0x80 }, { 0x620ac, 0xa0 },
	{ 0x620b0, 0xc0 }, { 0x620b4, 0xe0 }, { 0x620b8, 0x10 },
	{ 0x620bc, 0x1c }, { 0x620c0, 0x25 }, { 0x620c4, 0x37 },
	{ 0x620c8, 0x54 }, { 0x620cc, 0x78 }, { 0x620d0, 0xb4 },
};

static void wr_table(const struct reg_val *t, int n)
{
	int i;

	for (i = 0; i < n; i++)
		wr(t[i].reg, t[i].val);
}

int vpss_isp_init(void)
{
	return 0;
}
EXPORT_SYMBOL(vpss_isp_init);

int vpss_isp_set_attr(void)
{
	return 0;
}
EXPORT_SYMBOL(vpss_isp_set_attr);

/* the defaults; with on set only while the VPSS side is offline */
static int vpss_isp_regs_init(int on)
{
	if (on && C8(CM_VPSS_MODE) == 0)
		return 0;
	isp_ldc_buf = 0;
	wr_table(vpss_defaults, ARRAY_SIZE(vpss_defaults));
	if (C8(CM_BPU_MODE))
		wr(0x66c08, 0);
	wr_table(vpss_defaults2, ARRAY_SIZE(vpss_defaults2));
	wr_table(vpss_ltm, ARRAY_SIZE(vpss_ltm));
	wr(0x6401c, 0x180);
	wr(0x6501c, 0x40);
	return 0;
}

int vpss_isp_reset(void)
{
	wr(0x6200c, 0);
	wr(0x66800, 0);
	wr(0x69000, 0);
	wr(0x6a010, 0);
	wr(0x66c00, 0);
	vpss_all_set = 0;
	memset(vpss_delay, 0, sizeof(vpss_delay));
	return 0;
}

/* the frame's VPSS statistics: isp.ko keeps fixed values, and the one the
 * engine counts */
void vpss_get_stat(u32 c)
{
	if (c >= N_CHN)
		return;
	VP(c, 0x72c) = 0xf0e;
	VP(c, 0x730) = 0;
	VP(c, 0x734) = 0;
	VP(c, 0x738) = 0;
	VP(c, 0x73c) = 0;
	VP(c, 0x740) = 0x300;
	VP(c, 0x7e4) = rd(0x64a64);
}

/* the group's channel, or -1 */
static int vpss_find(u32 grp)
{
	int i;

	for (i = 0; i < N_CHN; i++)
		if ((C8(CM_CHN_MASK) >> i & 1) &&
		    C32(CM_GRP(i) + GRP_VPSS) == grp)
			return i;
	return -1;
}

int vpss_isp_post_deal(u32 grp)
{
	if (!C8(CM_VPSS_MODE))
		return -1;
	if (vpss_find(grp) < 0 || grp >= N_CHN)
		return -2;
	if (!(C32(CM_RUN) & BIT(grp)))
		return -1;
	vpss_get_stat(grp);
	return 0;
}
EXPORT_SYMBOL(vpss_isp_post_deal);

void vpss_isp_mutex_lock(u32 grp, int lock)
{
	if (grp >= N_CHN)
		return;
	if (lock)
		mutex_lock(&vpss_lock[grp]);
	else
		mutex_unlock(&vpss_lock[grp]);
}

void vpss_isp_set_delay(u32 grp, u8 v)
{
	if (grp < N_CHN)
		vpss_delay[grp] = v;
}

/* the group's blocks into the register image */
static void vpss_image(u32 c, u32 w, u32 h, s32 *zx, s32 *zy)
{
	u32 sx, sy;
	int i;

	RV(60) = VP(c, 0x1c);
	VP(c, 0x1c) = 0;
	RV(48) = VP(c, 0x158);
	RV(56) = VP(c, 0x24);
	RV(120) = VP(c, 0x70);
	RV(124) = VP(c, 0x74);
	RV(212) = VP(c, 0xcc);
	for (i = 0; i < 17; i++)
		RV(0x80 + i * 4) = VP(c, 0x78 + i * 4);
	RV(200) = VP(c, 0xc0);
	RV(196) = VP(c, 0xbc);
	RV(208) = VP(c, 0xc8);
	RV(224) = VP(c, 0xd8);
	RV(204) = VP(c, 0xc4);
	RV(232) = VP(c, 0xe0);
	RV(220) = VP(c, 0xd4);
	RV(228) = VP(c, 0xdc);
	RV(216) = VP(c, 0xd0);
	RV(236) = VP(c, 0xe4);
	for (i = 0; i < 17; i++)
		RV(0xf0 + i * 4) = VP(c, 0xe8 + i * 4);
	RV(308) = VP(c, 0x12c);
	RV(320) = VP(c, 0x138);
	RV(72) = VP(c, 0x30);
	RV(316) = VP(c, 0x134);
	RV(356) = VP(c, 0x168);
	RV(312) = VP(c, 0x130);
	RV(324) = VP(c, 0x13c);
	RV(328) = VP(c, 0x140);
	RV(336) = VP(c, 0x148);
	RV(332) = VP(c, 0x144);
	RV(344) = VP(c, 0x150);
	RV(340) = VP(c, 0x14c);
	RV(68) = VP(c, 0x2c);
	RV(352) = VP(c, 0x164);
	RV(360) = VP(c, 0x16c);
	RV(348) = VP(c, 0x154);
	RV(364) = VP(c, 0x170);
	RV(64) = VP(c, 0x28);
	RV(368) = VP(c, 0x174);
	RV(416) = VP(c, 0x1a4);
	RV(372) = VP(c, 0x178);
	RV(376) = VP(c, 0x17c);
	RV(380) = VP(c, 0x180);
	RV(384) = VP(c, 0x184);
	RV(388) = VP(c, 0x188);
	RV(392) = VP(c, 0x18c);
	RV(400) = VP(c, 0x194);
	RV(396) = VP(c, 0x190);
	RV(404) = VP(c, 0x198);
	RV(408) = VP(c, 0x19c);
	RV(412) = VP(c, 0x1a0);
	/* the scaler's steps: 6-pixel phases */
	if (w) {
		sx = (w - 1) % 6 + 1;
		RV(108) = 0x1000 / (sx * 6) + 1;
	} else {
		sx = 1;
		RV(108) = 0x2ab;
	}
	if (h) {
		sy = (h - 1) % 6 + 1;
		RV(112) = 0x1000 / (sy * 6) + 1;
	} else {
		sy = 1;
		RV(112) = 0x2ab;
	}
	RV(116) = 0x1000 / (sx * sy) + 1;
	RV(420) = VP(c, 0x15c);
	RV(440) = VP(c, 0x48);
	RV(432) = VP(c, 0x40);
	RV(456) = VP(c, 0x50);
	RV(452) = VP(c, 0x160);
	RV(472) = VP(c, 0x60);
	RV(436) = VP(c, 0x44);
	RV(428) = VP(c, 0x3c);
	RV(444) = VP(c, 0x4c);
	RV(460) = VP(c, 0x5c);
	RV(468) = VP(c, 0x58);
	RV(464) = VP(c, 0x54);
	RV(476) = VP(c, 0x64);
	RV(480) = VP(c, 0x68);
	RV(492) = VP(c, 0x1f0);
	RV(424) = VP(c, 0x38);
	RV(484) = VP(c, 0x6c);
	RV(488) = VP(c, 0x1ec);
	RV(500) = VP(c, 0x1f8);
	RV(504) = VP(c, 0x1fc);
	RV(448) = h;
	for (i = 0; i < 17; i++)
		RV(0x1fc + i * 4) = VP(c, 0x200 + i * 4);
	RV(596) = VP(c, 0x258);
	RV(608) = VP(c, 0x260);
	RV(576) = VP(c, 0x244);
	RV(616) = VP(c, 0x264);
	RV(496) = VP(c, 0x1f4);
	RV(600) = VP(c, 0x26c);
	RV(588) = VP(c, 0x250);
	RV(584) = VP(c, 0x24c);
	RV(624) = VP(c, 0x6a0);
	RV(628) = VP(c, 0x680);
	RV(580) = VP(c, 0x248);
	RV(604) = VP(c, 0x25c);
	RV(592) = VP(c, 0x254);
	RV(632) = VP(c, 0x684);
	RV(760) = VP(c, 0x688);
	RV(764) = VP(c, 0x68c);
	RV(768) = VP(c, 0x690);
	RV(772) = VP(c, 0x694);
	for (i = 0; i < 31; i++)
		RV(0x27c + i * 4) = VP(c, 0x6a4 + i * 4);
	RV(776) = VP8(c, 0x530);
	RV(780) = VP8(c, 0x531);
	RV(784) = VP8(c, 0x532);
	RV(788) = VP8(c, 0x533);
	RV(792) = VP8(c, 0x534);
	RV(796) = VP8(c, 0x535);
	RV(800) = VP8(c, 0x536);
	RV(804) = VP8(c, 0x537);
	RV(808) = VP8(c, 0x538);
	RV(812) = VP8(c, 0x539);
	RV(816) = VP8(c, 0x53a);
	RV(820) = VP16(c, 0x53c);
	RV(824) = VP8(c, 0x53e);
	RV(828) = VP(c, 0x270);
	RV(832) = VP(c, 0x274);
	RV(836) = VP(c, 0x278);
	RV(840) = VP(c, 0x280);
	RV(844) = VP(c, 0x27c);
	RV(848) = VP(c, 0x288);
	RV(852) = VP(c, 0x28c);
	RV(856) = VP(c, 0x284);
	for (i = 0; i < 20; i++)
		RV(860 + i * 4) = VP(c, 0x290 + i * 4);
	for (i = 0; i < 65; i++)
		RV(0x3ac + i * 4) = VP(c, 0x2e0 + i * 4);
	RV(1200) = VP(c, 0x3e4);
	for (i = 0; i < 65; i++)
		RV(0x4b4 + i * 4) = VP(c, 0x3e8 + i * 4);
	for (i = 0; i < 9; i++)
		RV(0x5b8 + i * 4) = VP16(c, 0x4ec + i * 2);
	for (i = 0; i < 15; i++)
		RV(1768 + i * 4) = VP(c, 0x540 + i * 4);
	for (i = 0; i < 31; i++)
		RV(0x724 + i * 4) = VP(c, 0x580 + i * 4);
	for (i = 0; i < 33; i++)
		RV(0x7a0 + i * 4) = VP(c, 0x5fc + i * 4);
	RV(2088) = VP(c, 0x720);
	RV(2084) = VP(c, 0x57c);
	RV(2092) = VP(c, 0x724);
	RV(2096) = VP(c, 0x728);
}

/* the zones: 32-pixel ones, 31 a row and column, the last ones' rest */
static void vpss_zones(u32 c, u32 w, u32 h, u32 zx, u32 zy, u32 rx, u32 ry)
{
	int i;

	RV(2248) = zx - 1;
	RV(2252) = zy - 1;
	RV(2256) = 0x100000 / zx;
	RV(2260) = 0x100000 / zy;
	RV(2264) = 0x100000 / (zy * zx);
	RV(2268) = 0x100000 / (zy * rx);
	RV(2272) = 0x100000 / (zx * ry);
	RV(96) = 0x100000 / (ry * rx);
	RV(2116) = VP(c, 0x760);
	RV(2112) = VP(c, 0x75c);
	RV(2104) = VP(c, 0x758);
	RV(2108) = VP(c, 0x754);
	RV(2276) = RV(96);
	for (i = 0; i < 32; i++)
		RV(0x848 + i * 4) = VP(c, 0x764 + i * 4);
	RV(2328) = w - 1;
	RV(2332) = h - 1;
	RV(2364) = VP(c, 0x7e8);
	RV(2336) = (VP(c, 0x7ec) & 0xffff) | VP(c, 0x7f0) << 16;
	RV(2100) = VP(c, 0x750);
	RV(2300) = VP(c, 0x740);
	RV(2348) = VP(c, 0x7fc);
	RV(2340) = VP(c, 0x7f4);
	RV(2316) = 1;
	RV(2356) = VP(c, 0x804);
	RV(2344) = VP(c, 0x7f8);
	RV(2360) = VP(c, 0x800);
	RV(2352) = 0x10001;
	RV(2320) = 0;
	RV(2324) = 0;
	RV(2372) = 0;
	/* the lens correction: its table loaded once after 0x49e9 maps it,
	 * or every frame offline */
	if (RV(2364))
		RV(2372) = VP(c, 0x818);
	if (C8(CM_BPU_MODE) == 3) {
		RV(2368) = 1;
	} else {
		RV(2368) = 0;
		if (RV(2364) && (ldc_tab_set >> c & 1)) {
			ldc_tab_set &= ~BIT(c);
			RV(2368) = 1;
		}
	}
	RV(76) = RV(2248);
	RV(80) = RV(2252);
	RV(84) = RV(2264);
	RV(88) = RV(2268);
	RV(92) = RV(2272);
	RV(100) = RV(2256);
	RV(104) = RV(2260);
}

static void vpss_zone_regs(void)
{
	wr(0x62010, RV(2248));
	wr(0x62014, RV(2252));
	wr(0x62028, RV(2256));
	wr(0x6202c, RV(2260));
	wr(0x62018, RV(2264));
	wr(0x6201c, RV(2268));
	wr(0x62020, RV(2272));
	wr(0x62024, RV(2276));
	wr(0x62004, RV(2104));
	wr(0x62008, RV(2104));
}

static void vpss_nr_regs(void)
{
	wr(0x6a000, RV(424));
	wr(0x6a040, RV(436));
	wr(0x69008, RV(456));
	wr(0x69024, RV(460));
	wr(0x69020, RV(480));
	wr(0x6903c, RV(484));
	wr(0x6a008, RV(428));
	wr(0x6a00c, RV(432));
	wr(0x6a048, RV(440));
	wr(0x6a04c, RV(444));
	wr(0x69018, RV(464));
	wr(0x6901c, RV(468));
	wr(0x69034, RV(472));
	wr(0x69038, RV(476));
}

static void vpss_all_regs(u32 c, u32 ltm)
{
	bool on = false;
	int i;

	vpss_isp_regs_init(1);
	wr(0x66804, RV(64));
	wr(0x6680c, RV(68));
	wr(0x66808, RV(72));
	wr(0x66004, RV(120));
	wr(0x66008, RV(124));
	for (i = 0; i < 17; i++)
		wr(0x66020 + i * 4, RV(0x80 + i * 4));
	wr(0x66064, RV(196));
	wr(0x66068, RV(200));
	wr(0x6606c, RV(204));
	wr(0x6607c, RV(208));
	wr(0x66084, RV(212));
	wr(0x660a8, RV(216));
	wr(0x66098, RV(220));
	wr(0x6609c, RV(224));
	wr(0x66404, RV(228));
	wr(0x66408, RV(232));
	wr(0x6640c, RV(236));
	for (i = 0; i < 17; i++)
		wr(0x66c20 + i * 4, RV(0xf0 + i * 4));
	wr(0x66c0c, RV(308));
	wr(0x66c64, RV(312));
	wr(0x66c68, RV(316));
	wr(0x66c6c, RV(320));
	wr(0x66c70, RV(324));
	wr(0x66c74, RV(328));
	wr(0x66c78, RV(332));
	wr(0x66c80, RV(336));
	wr(0x66c94, RV(344));
	wr(0x66c98, RV(348));
	wr(0x66c8c, RV(340));
	wr(0x66c10, RV(352));
	wr(0x66c14, RV(356));
	wr(0x66c84, RV(360));
	wr(0x66c88, RV(364));
	wr(0x66c90, RV(368));
	for (i = 0; i < 10; i++)
		wr(0x66ca0 + i * 4, RV(372 + i * 4));
	wr(0x66410, RV(412));
	wr(0x66414, RV(416));
	/* noise reduction: on when the block says so (bit 4) and its
	 * parameters are there; with the LTM (opt bit 2) left as it was */
	if (RV(48) & 0x10) {
		if (!RV(64) || !RV(68) || !RV(424) || !RV(456)) {
			pr_err("IK:Err: %s %d~ch%d\n", "vpss_isp_set_val_v2",
			       0x34c, c);
		} else {
			on = true;
		}
	}
	if (on && ltm)
		goto nr_on;
	if (!ltm) {
		if (on || RV(424)) {
			wr(0x6a000, RV(424));
			wr(0x6a008, RV(428));
			wr(0x6a00c, RV(432));
			wr(0x6a040, RV(436));
			wr(0x6a048, RV(440));
			wr(0x6a04c, RV(444));
			wr(0x6a018, 1);
		}
		if (RV(456)) {
			wr(0x69008, RV(456));
			wr(0x69024, RV(460));
			wr(0x69018, RV(464));
			wr(0x6901c, RV(468));
			wr(0x69034, RV(472));
			wr(0x69038, RV(476));
			wr(0x69020, RV(480));
			wr(0x6903c, RV(484));
			wr(0x69004, 3);
		}
		wr(0x69000, on && RV(452) ? 3 : 0);
		wr(0x6a010, on && RV(420) ? 3 : 0);
		if (on)
			goto nr_on;
	}
	wr(0x66800, 0);
	wr(0x66c00, 0);
	goto tail;
nr_on:
	wr(0x66c00, (s32)RV(48) < 0 ? 5 : 1);
	wr(0x66800, 3);
tail:
	wr(0x66000, RV(48) & 1);
	wr(0x66400, (RV(48) & 0x1ff) >> 8);
	vpss_zone_regs();
	wr(0x620e0, RV(2112));
	wr(0x620e4, RV(2116));
	wr(0x62038, RV(2108));
	for (i = 0; i < 32; i++)
		wr(0x62100 + i * 4, RV(0x848 + i * 4));
	if (RV(2100) && !RV(2104)) {
		pr_err("IK:Err: %s %d~ch%d\n", "vpss_isp_set_val_v2", 0x390, c);
		RV(2100) = 0;
	}
	wr(0x6200c, RV(2100) != 0);
	wr(0x62000, RV(2100));
	wr(0x60000, RV(488));
	wr(0x60004, RV(492));
	wr(0x60008, RV(496));
	wr(0x60088, RV(600));
	wr(0x6000c, RV(500));
	wr(0x60010, RV(504));
	for (i = 0; i < 16; i++)
		wr(0x60020 + i * 4, RV(0x1fc + i * 4));
	wr(0x60064, RV(576));
	wr(0x60068, RV(580));
	wr(0x6006c, RV(584));
	wr(0x6007c, RV(588));
	wr(0x60080, RV(592));
	wr(0x60084, RV(596));
	wr(0x60098, RV(604));
	wr(0x6009c, RV(608));
	wr(0x600a8, RV(616));
	wr(0x60804, RV(628));
	wr(0x60808, RV(632));
	wr(0x60910, RV(760));
	wr(0x60914, RV(764));
	wr(0x6091c, RV(768));
	wr(0x60920, RV(772));
	wr(0x60800, RV(624));
	for (i = 0; i < 12; i++)
		wr(0x60404 + i * 4, RV(780 + i * 4));
	wr(0x60400, RV(776));
	wr(0x64000, RV(828));
	wr(0x64004, RV(832));
	wr(0x64008, RV(836));
	wr(0x6401c, RV(840));
	wr(0x6400c, RV(844));
	wr(0x6501c, RV(856));
	wr(0x65018, RV(1200));
	if (RV(848)) {
		wr(0x65000, RV(848));
		wr(0x65200, RV(852));
		for (i = 0; i < 13; i++)
			wr(0x65204 + i * 4, RV(860 + i * 4));
		for (i = 0; i < 7; i++)
			wr(0x65240 + i * 4, RV(912 + i * 4));
		for (i = 0; i < 65; i++)
			wr(0x650e4 + i * 4, RV(0x3ac + i * 4));
	}
	for (i = 0; i < 65; i++)
		wr(0x63000 + i * 4, RV(0x4b4 + i * 4));
	for (i = 0; i < 9; i++)
		wr(0x63800 + i * 4, RV(0x5b8 + i * 4));
	wr(0x6101c, RV(1768));
	wr(0x61020, RV(1772));
	wr(0x611d0, RV(1776));
	wr(0x611dc, RV(1780));
	wr(0x611ec, RV(1784));
	wr(0x61230, RV(1788));
	wr(0x6123c, RV(1792));
	wr(0x6124c, RV(1796));
	wr(0x61258, RV(1812));
	wr(0x6125c, RV(1816));
	wr(0x61268, RV(1820));
	wr(0x6126c, RV(1824));
	wr(0x61294, RV(1800));
	wr(0x61298, RV(1804));
	wr(0x6129c, RV(1808));
	for (i = 0; i < 31; i++)
		wr(0x61038 + i * 4, RV(0x724 + i * 4));
	for (i = 0; i < 33; i++)
		wr(0x610d0 + i * 4, RV(0x7a0 + i * 4));
	wr(0x61000, RV(2084));
	wr(0x64a44, RV(2320));
	wr(0x64a48, RV(2324));
	wr(0x64a4c, RV(2328));
	wr(0x64a50, RV(2332));
	wr(0x64a6c, 1);
	wr(0x64a40, 0xf);
	if (RV(2372) && RV(2364) && RV(2368)) {
		const u32 *t = (const u32 *)RV(2372);

		wr(0x6c000, RV(2336));
		wr(0x6c004, RV(2340));
		wr(0x6c008, RV(2344));
		wr(0x6c00c, RV(2348));
		wr(0x6c010, RV(2352));
		wr(0x6c014, RV(2356));
		wr(0x6c018, RV(2360));
		wr(0x6c01c, 1);
		wr(0x6c020, 0);
		for (i = 0; i < 0x400; i++)
			wr(0x6d000 + i * 4, t[i]);
		wr(0x6c020, 1);
		wr(0x6c01c, RV(2364));
		isp_ldc_buf = RV(2372);
	}
	wr(0x6c020, RV(2364) != 0);
}

/*
 * A frame's VPSS registers for group grp (its channel's number) at the
 * output size w * h; locked: the caller holds the group's lock.
 */
int vpss_isp_set_val_v2(u32 grp, u32 w, u32 h, int locked)
{
	u32 zx, zy, rx, ry, ltm, off;
	int c = vpss_find(grp);

	if (c < 0 || grp >= N_CHN)
		return -2;
	if (!(C32(CM_RUN) & BIT(grp)))
		return -1;
	c = grp;
	if (!locked)
		mutex_lock(&vpss_lock[c]);
	vpss_image(c, w, h, NULL, NULL);
	zx = (w + 0x10) >> 5;
	zy = (h + 0x10) >> 5;
	rx = w > zx * 31 ? w - zx * 31 : 0;
	ry = h > zy * 31 ? h - zy * 31 : 0;
	if (!rx || !ry || !zx || !zy) {
		pr_err("IK:Err: %s line%d:size=0[%d %d %d %d, grp_id = %d]\n",
		       "vpss_isp_set_val_v2", 0x588, zx, zy, rx, ry, grp);
		if (!locked)
			mutex_unlock(&vpss_lock[c]);
		return -1;
	}
	vpss_zones(c, w, h, zx, zy, rx, ry);
	if (!locked)
		mutex_unlock(&vpss_lock[c]);
	if (vpss_delay[c] != 0xff)
		vpss_delay[c]++;
	off = C32(CM_VPSS_OFF);
	ltm = (C8(CM_OPT) & 7) >> 2;
	if (off == 0xff) {
		wr(0x69000, 0);
		wr(0x6a010, 0);
		wr(0x66800, 0);
		return 0;
	}
	if (!off) {
		vpss_all_set = 0;
	} else if (vpss_all_set) {
		wr(0x66804, RV(64));
		wr(0x6680c, RV(68));
		if (!ltm)
			vpss_nr_regs();
		vpss_zone_regs();
		return 0;
	}
	vpss_all_regs(c, ltm);
	if (BIT(grp) & C32(CM_VPSS_OFF))
		vpss_all_set = 1;
	return 0;
}

int vpss_isp_set_val(u32 grp, u32 w, u32 h, int locked)
{
	return vpss_isp_set_val_v2(grp, w, h, locked);
}
EXPORT_SYMBOL(vpss_isp_set_val);

struct vpss_cmd {
	u16 cmd, size, dst, len;
};

static const struct vpss_cmd vpss_cmds[] = {
	{ 0x49db, 0x88, 0x1ec, 0x84 },
	{ 0x49dc, 0x1e4, 0x00c, 0x1e0 },
	{ 0x49de, 0x1c, 0x270, 0x18 },
	{ 0x49df, 0x108, 0x3e8, 0x104 },
	{ 0x49e0, 0x18, 0x4ec, 0x12 },
	{ 0x49e2, 0x10, 0x720, 0x0c },
	{ 0x49e4, 0x164, 0x288, 0x160 },
	{ 0x49e6, 0x44, 0x540, 0x40 },
	{ 0x49e7, 0x104, 0x580, 0x100 },
	{ 0x49e8, 0x98, 0x750, 0x94 },
	{ 0x49e9, 0x30, 0x7e8, 0x2c },
	{ 0x49ea, 0x28, 0x680, 0x24 },
	{ 0x49eb, 0x80, 0x6a4, 0x7c },
	{ 0x49ec, 0x14, 0x530, 0x10 },
};

static void vpss_init_para(void)
{
	int c, i;

	memset(vpss_para, 0, sizeof(vpss_para));
	for (c = 0; c < N_CHN; c++) {
		mutex_init(&vpss_lock[c]);
		for (i = 0; i < 9; i++)
			VP16(c, 0x4ec + i * 2) = 0x100;
		VP(c, 0x270) = 0x100;
		VP(c, 0x274) = 0x100;
		VP(c, 0x278) = 0x100;
		VP(c, 0x280) = 0x100;
		VP(c, 0x720) = 0x100;
		VP(c, 0x724) = 0x100;
		VP(c, 0x728) = 0x100;
		VP(c, 0x27c) = 0;
		VP(c, 0x284) = 0x40;
	}
	ldc_tab_set = 0;
	mutex_init(&iwctrl_vpss_lock);
	vpss_isp_regs_init(0);
}

/* 0x49e9's lens correction table: mapped once, cached */
static void vpss_ldc(u32 c, const u32 *w)
{
	void __iomem *va;

	if (!w[0] || !w[8] || VP(c, 0x814))
		return;
	VP(c, 0x814) = w[8];
	if (VP(c, 0x818))
		iounmap((void __iomem *)VP(c, 0x818));
	va = ioremap_cached(w[8], 0x1000);
	VP(c, 0x818) = (u32)va;
	if (!va)
		pr_err("IK:Err: ioremap_cached ldc_buf failed!\n");
	ldc_tab_set |= BIT(c);
}

int iw_vpss_ioctl(unsigned int cmd, void __user *arg)
{
	const struct vpss_cmd *v = NULL;
	u32 buf[0x1e4 / 4], c;
	int i, ret = 0;

	if (cmd == 0x49d2) {
		vpss_init_para();
		return 0;
	}
	if (cmd == 0x49e3) {
		pr_err("IK:Err: CMD_VPSS_IW_ENHANCE not support!\n");
		return -1;
	}
	for (i = 0; i < ARRAY_SIZE(vpss_cmds); i++)
		if (vpss_cmds[i].cmd == cmd)
			v = &vpss_cmds[i];
	if (!v && cmd != 0x49d7 && cmd != 0x49d9)
		return -1;
	mutex_lock(&iwctrl_vpss_lock);
	if (cmd == 0x49d7 || cmd == 0x49d9) {
		u32 n = cmd == 0x49d7 ? 0x28 : 8;

		if (copy_from_user(buf, arg, n)) {
			ret = -1;
			goto out;
		}
		c = buf[0];
		if (c < N_CHN) {
			if (cmd == 0x49d7)
				memcpy(&buf[1], &vpss_para[c][0x72c], 0x24);
			else
				buf[1] = VP(c, 0x7e4);
		}
		if (copy_to_user(arg, buf, n))
			ret = -1;
		goto out;
	}
	if (copy_from_user(buf, arg, v->size)) {
		ret = -1;
		goto out;
	}
	c = buf[0];
	if (c >= N_CHN)
		goto out; /* isp.ko ignores it */
	mutex_lock(&vpss_lock[c]);
	memcpy(&vpss_para[c][v->dst], &buf[1], v->len);
	if (cmd == 0x49e9)
		vpss_ldc(c, &buf[1]);
	mutex_unlock(&vpss_lock[c]);
	if (cmd == 0x49ea) {
		/* the sharpening's two words, into the BPU group too */
		mutex_lock(&grp_lock[c]);
		C32(CM_GRP(c) + 0x2f4) = buf[7];
		C32(CM_GRP(c) + 0x2f8) = buf[8];
		mutex_unlock(&grp_lock[c]);
	}
out:
	mutex_unlock(&iwctrl_vpss_lock);
	return ret;
}
