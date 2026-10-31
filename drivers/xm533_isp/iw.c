// SPDX-License-Identifier: GPL-2.0
/*
 * The BPU's parameter blocks (0x4964..0x497f): {chn, data...}, copied into
 * g_comm_para for the BPU thread (bpu.c says where each goes). Channel 0
 * only; a block for another channel is taken and ignored, as isp.ko does.
 */

#include <linux/uaccess.h>

#include "isp.h"

void iw_isp_init(void)
{
	C32(0x1128) = 0x100;
	C32(0x112c) = 0x100;
	C32(0x1130) = 0x100;
	C32(0x1134) = 0x100;
}

/* plain copies of the block after its channel word */
static const struct {
	u16 cmd, size, dst, len;
} iw_copy[] = {
	{ 0x4969, 200, 0x121c, 0xc2 }, /* 97 points */
	{ 0x496a, 0x14, 0x12e0, 0x10 },
	{ 0x496b, 8, 0x12f0, 4 },
	{ 0x496c, 0x84, 0x12f4, 0x80 },
	{ 0x496d, 0xd0, 0x1138, 0xcc },
	{ 0x496e, 0x214, 0x1374, 0x210 },
	{ 0x496f, 0x6c, 0x1584, 0x68 },
	{ 0x4971, 0x20c, 0x0ee2, 0x206 },
	{ 0x4975, 0x10, 0x111c, 12 },
	{ 0x497f, 0x18, 0x16e4, 20 },
};

static int iw_do(unsigned int cmd, void __user *arg, u32 *w)
{
	const u8 *b = (const u8 *)w;
	int i;

	switch (cmd) {
	case 0x4964:
		if (copy_from_user(w, arg, 8))
			return -EFAULT;
		if (w[0])
			return 0;
		mutex_lock(&bpu_lock);
		C32(0x1128) = w[1];
		mutex_unlock(&bpu_lock);
		return 0;
	case 0x4965:
		if (copy_from_user(w, arg, 0x10))
			return -EFAULT;
		if (w[0])
			return 0;
		mutex_lock(&bpu_lock);
		C32(0x112c) = w[1];
		C32(0x1130) = w[2];
		C32(0x1134) = w[3];
		mutex_unlock(&bpu_lock);
		return 0;
	case 0x4966: /* 12 words, kept as 16 bits */
		if (copy_from_user(w, arg, 0x34))
			return -EFAULT;
		if (w[0])
			return 0;
		mutex_lock(&bpu_lock);
		for (i = 0; i < 12; i++)
			C16(0x1204 + i * 2) = w[1 + i];
		mutex_unlock(&bpu_lock);
		return 0;
	case 0x4967: /* R | G << 16, B */
		if (copy_from_user(w, arg, 0xc))
			return -EFAULT;
		if (w[0])
			return 0;
		mutex_lock(&bpu_lock);
		C16(0x10e8) = w[1];
		C16(0x10ea) = w[1] >> 16;
		C16(0x10ec) = w[2];
		mutex_unlock(&bpu_lock);
		return 0;
	case 0x4968:
		if (copy_from_user(w, arg, 0x34))
			return -EFAULT;
		if (w[0])
			return 0;
		mutex_lock(&bpu_lock);
		memcpy(CP(0x10ee), b + 4, 42);
		C16(0x111a) = *(u16 *)(b + 48);
		mutex_unlock(&bpu_lock);
		return 0;
	case 0x4970: /* the whole block, its channel word too */
		if (copy_from_user(w, arg, 0xa4))
			return -EFAULT;
		memcpy(CP(0xe34), w, 0xa4);
		return 0;
	case 0x4973: /* 9 words back */
		if (copy_from_user(w, arg, 0x28))
			return -EFAULT;
		if (w[0] == 0) {
			mutex_lock(&bpu_lock);
			memcpy(&w[1], CP(0x16f8), 0x24);
			mutex_unlock(&bpu_lock);
		}
		if (copy_to_user(arg, w, 0x28))
			return -EFAULT;
		return 0;
	case 0x4974:
		if (copy_from_user(w, arg, 0x10))
			return -EFAULT;
		if (w[0])
			return 0;
		mutex_lock(&bpu_lock);
		C32(0xed8) = w[1];
		C32(0xedc) = w[2];
		C16(0xee0) = w[3];
		mutex_unlock(&bpu_lock);
		return 0;
	case 0x4976:
		if (copy_from_user(w, arg, 0x20))
			return -EFAULT;
		if (w[0])
			return 0;
		mutex_lock(&bpu_lock);
		memcpy(CP(0x171c), b + 6, 24);
		mutex_unlock(&bpu_lock);
		return 0;
	case 0x4977:
		/* (isp.ko returns here with iwctrl_lock held) */
		return -1;
	}
	for (i = 0; i < ARRAY_SIZE(iw_copy); i++) {
		if (iw_copy[i].cmd != cmd)
			continue;
		if (copy_from_user(w, arg, iw_copy[i].size))
			return -EFAULT;
		if (w[0]) /* channel 0 only */
			return 0;
		mutex_lock(&bpu_lock);
		memcpy(CP(iw_copy[i].dst), b + 4, iw_copy[i].len);
		mutex_unlock(&bpu_lock);
		return 0;
	}
	return -1;
}

/* 0: taken; -1: not one of these (iw_vpss_ioctl's then) */
int iw_ioctl(unsigned int cmd, void __user *arg)
{
	static u32 w[0x214 / 4];
	int ret;

	mutex_lock(&iwctrl_lock);
	ret = iw_do(cmd, arg, w);
	mutex_unlock(&iwctrl_lock);
	return ret == -EFAULT ? -1 : ret;
}
