// SPDX-License-Identifier: GPL-2.0
/*
 * The VICU (the sensor's frames into the ISP): started by 0x490e/0x495e,
 * its frame interrupt (IRQ 57, status 0x300f004c) counts frames and their
 * rate, takes the AWB statistics, wakes the BPU thread and 0x490a, and
 * runs a raw-frame dump (0x495a) over the dump engine at 0x30019000.
 */

#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/time.h>

#include "isp.h"

void clk_get_pts(u64 *pts);

static u8 dump_state; /* 0 idle, 1 started, 2 its frame seen */

void vicu_init_queue(void)
{
}

/* 0x490e's 52 bytes, as vicu_chn_init reads them */
int vicu_chn_init(const u8 *a)
{
	if (!(a[1] & 1))
		return 0;
	C32(CM_SNS_ID) = a[0];
	C32(CM_WDR_MODE) = a[7];
	C32(CM_VICU_MODE) = *(u32 *)(a + 0x30);
	if (!C8(CM_CHNNUM)) {
		pr_err("IspKo:Err: not config vicu num\n");
	} else if (CS8(CM_OPT) >= 0 && C8(CM_MODE) == 0) {
		C32(0x128) = 1;
		isp_write_reg(0x300f0044, 1);
		isp_write_reg(0x300f0040, 1);
	}
	return 0;
}

int vicu_run(int chn)
{
	if (chn != 0)
		return -1;
	if (C32(0xdd8) & 1)
		isp_write_reg(0x300f0040, 1);
	return 0;
}

void vicu_reset(void)
{
	isp_write_reg(0x300f0040, 0);
	isp_write_reg(0x30019034, 0);
}

/* The dump: armed on one frame, checked on the next; 1 to wake its waiter */
static void vicu_dump(void)
{
	u8 res;

	if (C8(CM_DUMP_REQ))
		return;
	switch (dump_state) {
	case 0:
		isp_write_reg(0x30019034, 0);
		isp_write_reg(0x30019048, 0);
		isp_write_reg(0x30019054, 0);
		isp_write_reg(0x30019000, (C32(CM_DUMP_ADDR) + 0x80000000) >> 4);
		isp_write_reg(0x30019020, C16(CM_DUMP_W) / 10);
		isp_write_reg(0x30019024, C16(CM_DUMP_W) / 10);
		isp_write_reg(0x30019034, 1);
		dump_state = 1;
		return;
	case 1:
		if (isp_read_reg(0x30019100) != isp_read_reg(0x30019108)) {
			dump_state = 2;
			isp_write_reg(0x30019034, 0);
			isp_write_reg(0x30019048, 0);
			return;
		}
		isp_write_reg(0x30019034, 0);
		isp_write_reg(0x30019048, 0);
		res = 0x20; /* failed */
		break;
	case 2:
		res = 0x10; /* done */
		break;
	default:
		return;
	}
	C8(CM_DUMP) = res;
	dump_state = 0;
	wake_up(&dump_wq);
}

irqreturn_t vicu_irq_0(int irq, void *dev)
{
	struct timeval tv;
	u64 pts;
	u32 dt, cnt;
	bool wake = false;

	if (!isp_read_reg(0x300f004c))
		return IRQ_HANDLED;
	isp_write_reg(0x300f004c, 0);
	if (C32(CM_RUN) != 1)
		return IRQ_HANDLED;
	do_gettimeofday(&tv);
	clk_get_pts(&pts);
	if (!C8(CM_BPU_BUSY))
		return IRQ_HANDLED;
	if (!(C8(CM_DUMP) & 1)) {
		if (C8(CM_MODE) != 1 && bpu_post_deal(0) == 0) {
			C32(CM_BPU_GO) = 1;
			wake = true;
		}
	} else {
		vicu_dump();
	}
	dt = (tv.tv_usec - C32(0xd4)) + (tv.tv_sec - C32(0xd0)) * 1000000;
	C32(CM_VICU_FPS) = dt ? (100000000 / dt + 5) / 10 : 0xff;
	C32(0xd0) = tv.tv_sec;
	C32(0xd4) = tv.tv_usec;
	cnt = C32(CM_VICU_CNT) + 1;
	C32(0xcc) = 1;
	C32(0xc8) = 0;
	if (cnt > 0xffff)
		cnt = 0;
	C32(CM_VICU_CNT) = cnt;
	C32(0x110) = (u32)pts;
	C32(0x114) = (u32)(pts >> 32);
	if (wake) {
		C32(CM_VSYNC_CNT) |= 1;
		wake_up(&vsync_wq);
		wake_up(&bpu_thread_wq);
	}
	return IRQ_HANDLED;
}
