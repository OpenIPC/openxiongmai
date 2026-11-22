// SPDX-License-Identifier: GPL-2.0
/*
 * The BPU's parameter blocks (0x4964..0x497f): {chn, data...}, copied into
 * the channel's BPU group under its lock; the BPU (bpu.c) writes them to
 * the registers. A block for a channel past the third is taken and
 * ignored, as isp.ko does.
 */

#include <linux/uaccess.h>

#include "isp.h"

void iw_isp_init(void)
{
	int c;

	for (c = 0; c < N_CHN; c++) {
		C32(CM_GRP(c) + 0x37c) = 0x100; /* 0x2cac: exposure */
		C32(CM_GRP(c) + 0x380) = 0x100;
		C32(CM_GRP(c) + 0x384) = 0x100;
		C32(CM_GRP(c) + 0x388) = 0x100;
	}
}

/* {cmd, size, offset (channel 0's), bytes, from} */
static const struct {
	u16 cmd, size, dst, len, from;
} iw_copy[] = {
	{ 0x4964, 8, 0x2cac, 4, 4 },
	{ 0x4965, 0x10, 0x2cb0, 12, 4 },
	{ 0x4969, 200, 0x2da0, 0xc2, 4 },   /* 97 points */
	{ 0x496a, 0x14, 0x2e64, 0x10, 4 },
	{ 0x496b, 8, 0x2e74, 4, 4 },
	{ 0x496c, 0x84, 0x2e78, 0x80, 4 },
	{ 0x496d, 0xd0, 0x2cbc, 0xcc, 4 },
	{ 0x496e, 0x214, 0x2ef8, 0x210, 4 },
	{ 0x496f, 0x6c, 0x3108, 0x68, 4 },
	{ 0x4971, 0x20c, 0x2a52, 0x206, 4 },
	{ 0x4975, 0x10, 0x2ca0, 12, 4 },
	{ 0x4976, 0x20, 0x32a0, 24, 6 },
	{ 0x4977, 0x5c, 0x3170, 0x58, 4 },
	{ 0x497f, 0x18, 0x3268, 20, 4 },
};

static int iw_do(unsigned int cmd, void __user *arg, u32 *w)
{
	const u8 *b = (const u8 *)w;
	u32 c, g;
	int i;

	switch (cmd) {
	case 0x4966: /* 12 words, kept as 16 bits */
		if (copy_from_user(w, arg, 0x34))
			return -EFAULT;
		if (w[0] >= N_CHN)
			return 0;
		g = w[0] * 0x12d4;
		mutex_lock(&grp_lock[w[0]]);
		for (i = 0; i < 12; i++)
			C16(0x2d88 + g + i * 2) = w[1 + i];
		mutex_unlock(&grp_lock[w[0]]);
		return 0;
	case 0x4967: /* R | G << 16, B */
		if (copy_from_user(w, arg, 0xc))
			return -EFAULT;
		if (w[0] >= N_CHN)
			return 0;
		g = w[0] * 0x12d4;
		mutex_lock(&grp_lock[w[0]]);
		C16(0x2c58 + g) = w[1];
		C16(0x2c5a + g) = w[1] >> 16;
		C16(0x2c5c + g) = w[2];
		mutex_unlock(&grp_lock[w[0]]);
		return 0;
	case 0x4968: /* words 1..6 and 9..14 */
		if (copy_from_user(w, arg, 0x44))
			return -EFAULT;
		if (w[0] >= N_CHN)
			return 0;
		g = w[0] * 0x12d4;
		mutex_lock(&grp_lock[w[0]]);
		memcpy(CP(0x2c60 + g), &w[1], 6 * 4);
		memcpy(CP(0x2c80 + g), &w[9], 6 * 4);
		mutex_unlock(&grp_lock[w[0]]);
		return 0;
	case 0x4970: /* the whole block, to every channel */
		if (copy_from_user(w, arg, 0xa4))
			return -EFAULT;
		for (c = 0; c < N_CHN; c++)
			memcpy(CP(CM_GRP(c) + GRP_BLOCK), w, 0xa4);
		return 0;
	case 0x4973: /* 9 words back */
		if (copy_from_user(w, arg, 0x28))
			return -EFAULT;
		if (w[0] < N_CHN) {
			mutex_lock(&grp_lock[w[0]]);
			memcpy(&w[1], CP(0x327c + w[0] * 0x12d4), 0x24);
			mutex_unlock(&grp_lock[w[0]]);
		}
		if (copy_to_user(arg, w, 0x28))
			return -EFAULT;
		return 0;
	case 0x4974:
		if (copy_from_user(w, arg, 0x10))
			return -EFAULT;
		if (w[0] >= N_CHN)
			return 0;
		g = w[0] * 0x12d4;
		mutex_lock(&grp_lock[w[0]]);
		C32(0x2a48 + g) = w[1];
		C32(0x2a4c + g) = w[2];
		C16(0x2a50 + g) = w[3];
		mutex_unlock(&grp_lock[w[0]]);
		return 0;
	}
	for (i = 0; i < ARRAY_SIZE(iw_copy); i++) {
		if (iw_copy[i].cmd != cmd)
			continue;
		if (copy_from_user(w, arg, iw_copy[i].size))
			return -EFAULT;
		if (w[0] >= N_CHN)
			return 0;
		mutex_lock(&grp_lock[w[0]]);
		memcpy(CP(iw_copy[i].dst + w[0] * 0x12d4), b + iw_copy[i].from,
		       iw_copy[i].len);
		mutex_unlock(&grp_lock[w[0]]);
		return 0;
	}
	return -1;
}

int iw_ioctl(unsigned int cmd, void __user *arg)
{
	static u32 w[0x214 / 4];
	int ret;

	mutex_lock(&iwctrl_lock);
	ret = iw_do(cmd, arg, w);
	mutex_unlock(&iwctrl_lock);
	return ret == -EFAULT ? -1 : ret;
}
