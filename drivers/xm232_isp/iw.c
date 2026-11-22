// SPDX-License-Identifier: GPL-2.0
/*
 * The parameter blocks (0x4964..0x4980) into the channels' BPU groups.
 * Each command but 0x4970 is {chn, data}; the data lands at a fixed offset
 * of the channel's group, under its lock, which the per-frame register
 * writes (bpu.c) read. 0x4970 sets every channel's block head at once.
 * 0x496d and 0x4978 also name the AWB and MD statistics' buffer pair (the
 * physical address of the first, the second 8 KiB on), mapped here so the
 * driver can read them.
 */

#include <linux/io.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#include "isp.h"

struct iw_cmd {
	u16 cmd;
	u16 size; /* bytes from user space */
	u16 dst;  /* the group's offset the data goes to */
	u16 len;  /* bytes copied */
	u8 from;  /* the data's offset in the record */
};

/* isp.ko's blocks: offsets from the group (0x2840 + chn * 0x8b0) */
static const struct iw_cmd iw_cmds[] = {
	{ 0x4964, 0x08, 0x370, 0x04, 4 },
	{ 0x4965, 0x10, 0x374, 0x0c, 4 },
	{ 0x4966, 0x34, 0x538, 0x30, 4 },
	{ 0x4967, 0x0c, 0x2ec, 0x06, 4 },
	{ 0x4968, 0x68, 0x2fc, 0x64, 4 },
	{ 0x4969, 0xc8, 0x568, 0xc2, 4 },
	{ 0x496a, 0x14, 0x62c, 0x10, 4 },
	{ 0x496b, 0x08, 0x652, 0x04, 4 },
	{ 0x496c, 0x84, 0x658, 0x80, 4 },
	{ 0x496d, 0xd4, 0x380, 0xcc, 4 },
	{ 0x496e, 0xf8, 0x6d8, 0xf4, 4 },
	{ 0x4971, 0x20c, 0x0e6, 0x206, 4 },
	{ 0x4974, 0x10, 0x0dc, 0x0a, 4 },
	{ 0x4976, 0x20, 0x898, 0x18, 6 },
	{ 0x4978, 0xd4, 0x45c, 0xcc, 4 },
	{ 0x4980, 0x1c, 0x63c, 0x16, 4 },
};

void iw_isp_init(void)
{
	int c;

	for (c = 0; c < N_CHN; c++) {
		C32(CM_GRP(c) + 0x370) = 0x100;
		C32(CM_GRP(c) + 0x374) = 0x100;
		C32(CM_GRP(c) + 0x378) = 0x100;
		C32(CM_GRP(c) + 0x37c) = 0x100;
	}
}

/* the AWB (0x496d) or MD (0x4978) buffer pair at phys: 0x4000 bytes */
static void iw_stat_buf(int md, u32 chn, u32 phys)
{
	u32 st = md ? CM_MD(chn) : CM_AWB(chn);
	u32 grp = CM_GRP(chn) + (md ? 0x45c : 0x380);
	void __iomem *va;

	C32(st + ST_PHYS) = phys;
	C32(st + ST_PHYS + 4) = phys + 0x2000;
	C32(grp + 0xd0) = 0x40;
	C32(grp + 0xcc) = 1;
	if (stat_va[md][chn][0]) {
		iounmap(stat_va[md][chn][0]);
		stat_va[md][chn][0] = NULL;
		stat_va[md][chn][1] = NULL;
	}
	va = ioremap(phys, 0x4000);
	if (!va) {
		pr_err("IK:Err: %s %d ioremap failed!\n",
		       md ? "iw_md_wdw" : "iw_awb_wdw", md ? 0xfa : 200);
		return;
	}
	stat_va[md][chn][0] = va;
	stat_va[md][chn][1] = va + 0x2000;
}

int iw_ioctl(unsigned int cmd, void __user *arg)
{
	const struct iw_cmd *c = NULL;
	u8 *buf;
	u32 chn, phys;
	int i, ret = 0;

	for (i = 0; i < ARRAY_SIZE(iw_cmds); i++)
		if (iw_cmds[i].cmd == cmd)
			c = &iw_cmds[i];
	if (!c && cmd != 0x4970)
		return -1;
	buf = kmalloc(0x20c, GFP_KERNEL);
	if (!buf)
		return -1;
	mutex_lock(&iwctrl_lock);
	if (cmd == 0x4970) {
		/* every channel's block head, no channel number */
		if (copy_from_user(buf, arg, 0xa4)) {
			ret = -1;
			goto out;
		}
		for (i = 0; i < N_CHN; i++)
			memcpy(CP(CM_BLK(i)), buf, 0xa4);
		goto out;
	}
	if (copy_from_user(buf, arg, c->size)) {
		ret = -1;
		goto out;
	}
	chn = *(u32 *)buf;
	if (chn >= N_CHN)
		goto out; /* isp.ko ignores it */
	mutex_lock(&grp_lock[chn]);
	memcpy(CP(CM_GRP(chn) + c->dst), buf + c->from, c->len);
	if (cmd == 0x496d || cmd == 0x4978) {
		phys = *(u32 *)(buf + 0xd0);
		if (phys > 0x80000000)
			iw_stat_buf(cmd == 0x4978, chn, phys);
	}
	mutex_unlock(&grp_lock[chn]);
out:
	mutex_unlock(&iwctrl_lock);
	kfree(buf);
	return ret;
}
