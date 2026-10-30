// SPDX-License-Identifier: GPL-2.0
/*
 * The parameter blocks (0x4964..0x49e8): {chn, data...}. The BPU's go into
 * the channel's BPU group under its lock, the bpu.c thread writing them to
 * the registers; the VICU's (exposure, gains, white balance, the AWB
 * windows, the black level and curves) into its VICU block with interrupts
 * masked, the frame interrupt writing them. A block for a channel past the
 * third is taken and ignored, as isp.ko does.
 */

#include <linux/uaccess.h>

#include "isp.h"

void iw_isp_init(void)
{
	int c;

	for (c = 0; c < N_CHN; c++) {
		C32(CM_VB(c) + 0x21c) = 0x100; /* 0x57e4: exposure */
		C32(CM_VB(c) + 0x220) = 0x100;
		C32(CM_VB(c) + 0x224) = 0x100;
		C32(CM_VB(c) + 0x228) = 0x100;
	}
}

enum { IW_BPU, IW_VICU };

/* {cmd, size, offset (channel 0's), bytes, from, where} */
static const struct {
	u16 cmd, size, dst, len;
	u8 from, where;
} iw_copy[] = {
	{ 0x4964, 8, CM_VB(0) + 0x21c, 4, 4, IW_VICU },
	{ 0x4965, 0x10, CM_VB(0) + 0x220, 12, 4, IW_VICU },
	{ 0x4967, 0xc, CM_VB(0) + 0x214, 6, 4, IW_VICU },
	{ 0x496d, 0xd0, CM_VB(0) + 0x22c, 0xcc, 4, IW_VICU },
	{ 0x4971, 0x20c, CM_VB(0) + 0xe, 0x206, 4, IW_VICU },
	{ 0x4969, 200, CM_GRP(0) + 0x18c, 0xc2, 4, IW_BPU }, /* 97 points */
	{ 0x496a, 0x14, CM_GRP(0) + 0x250, 0x10, 4, IW_BPU },
	{ 0x496b, 0x14, CM_GRP(0) + 0x260, 0x10, 4, IW_BPU },
	{ 0x496c, 0x84, CM_GRP(0) + 0x270, 0x80, 4, IW_BPU },
	{ 0x496e, 0x214, CM_GRP(0) + 0x2f0, 0x210, 4, IW_BPU },
	{ 0x496f, 0x6c, CM_GRP(0) + 0x500, 0x68, 4, IW_BPU },
	{ 0x4975, 0x10, CM_GRP(0) + 0x168, 12, 4, IW_BPU },
	{ 0x4976, 0x20, CM_GRP(0) + 0x7a0, 24, 6, IW_BPU },
	{ 0x4977, 0x5c, CM_GRP(0) + 0x568, 0x58, 4, IW_BPU },
	{ 0x497f, 0x18, CM_GRP(0) + 0x650, 20, 4, IW_BPU },
	{ 0x49e3, 0x1c, CM_GRP(0) + 0x744, 24, 4, IW_BPU },
	{ 0x49e8, 0xa0, CM_GRP(0) + 0x688, 0x9c, 4, IW_BPU },
};

/* 0x4973 the BPU's histogram, 0x49d7 its 3D noise reduction's: 9 words */
static int iw_read9(void __user *arg, u32 *w, u32 off)
{
	if (copy_from_user(w, arg, 0x28))
		return -EFAULT;
	if (w[0] >= N_CHN)
		return 0;
	mutex_lock(&grp_lock[w[0]]);
	memcpy(&w[1], CP(CM_GRP(w[0]) + off), 9 * 4);
	mutex_unlock(&grp_lock[w[0]]);
	if (copy_to_user(arg, w, 0x28))
		return -EFAULT;
	return 0;
}

static int iw_do(unsigned int cmd, void __user *arg, u32 *w)
{
	const u8 *b = (const u8 *)w;
	unsigned long flags;
	u32 c, g, o;
	int i;

	switch (cmd) {
	case 0x4966: /* 12 words, kept as 16 bits */
		if (copy_from_user(w, arg, 0x34))
			return -EFAULT;
		if (w[0] >= N_CHN)
			return 0;
		g = CM_GRP(w[0]);
		mutex_lock(&grp_lock[w[0]]);
		for (i = 0; i < 12; i++)
			C16(g + 0x174 + i * 2) = w[1 + i];
		mutex_unlock(&grp_lock[w[0]]);
		return 0;
	case 0x4968: /* 3 bytes, then 0x58 */
		if (copy_from_user(w, arg, 0x60))
			return -EFAULT;
		if (w[0] >= N_CHN)
			return 0;
		g = CM_GRP(w[0]);
		mutex_lock(&grp_lock[w[0]]);
		memcpy(CP(g + 0x10c), b + 4, 3);
		memcpy(CP(g + 0x110), b + 8, 0x58);
		mutex_unlock(&grp_lock[w[0]]);
		return 0;
	case 0x4970: /* the whole block, to every channel */
		if (copy_from_user(w, arg, 0xa4))
			return -EFAULT;
		for (c = 0; c < N_CHN; c++)
			memcpy(CP(CM_GRP(c) + GRP_BLOCK), w, 0xa4);
		return 0;
	case 0x4973:
		return iw_read9(arg, w, 0x664);
	case 0x49d7:
		return iw_read9(arg, w, 0x75c);
	case 0x4974:
		if (copy_from_user(w, arg, 0x10))
			return -EFAULT;
		if (w[0] >= N_CHN)
			return 0;
		o = CM_VB(w[0]);
		spin_lock_irqsave(&awb_lock[w[0]], flags);
		C32(o + 4) = w[1];
		C32(o + 8) = w[2];
		C16(o + 0xc) = w[3];
		spin_unlock_irqrestore(&awb_lock[w[0]], flags);
		return 0;
	}
	for (i = 0; i < ARRAY_SIZE(iw_copy); i++) {
		if (iw_copy[i].cmd != cmd)
			continue;
		if (copy_from_user(w, arg, iw_copy[i].size))
			return -EFAULT;
		if (w[0] >= N_CHN)
			return 0;
		if (iw_copy[i].where == IW_VICU) {
			o = iw_copy[i].dst + w[0] * 0x2f4;
			spin_lock_irqsave(&awb_lock[w[0]], flags);
			memcpy(CP(o), b + iw_copy[i].from, iw_copy[i].len);
			spin_unlock_irqrestore(&awb_lock[w[0]], flags);
		} else {
			o = iw_copy[i].dst + w[0] * 0xf40;
			mutex_lock(&grp_lock[w[0]]);
			memcpy(CP(o), b + iw_copy[i].from, iw_copy[i].len);
			mutex_unlock(&grp_lock[w[0]]);
		}
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
