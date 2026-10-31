// SPDX-License-Identifier: GPL-2.0
/*
 * The two VICUs (the sensors' frames into the ISP), IRQ 87 at the frame's
 * start and IRQ 88 at its stored end. At the start the frame's VICU-side
 * registers go out (curves, black level; offline also the white balance
 * and statistics windows), the AWB statistics come in, and offline the
 * buffer the VICU stored last is handed to the BPU. At the end (offline
 * only) the VICU gets the next ISP buffer to store into. The second VICU
 * can be time-shared by sensors (0x4912, "dvpsel").
 */

#include <linux/interrupt.h>
#include <linux/sched.h>

#include "isp.h"

/* the buffers: [0..2] stored, [3..5] storing, [6..8] next, per channel */
static void *vbuf[9];
static u8 held;      /* the parameters waited for (CM_IW_WAIT) and set */
static u8 dvp_same;  /* frames the time-shared sensor stayed */
static u8 dump_state;
static u8 dvp_last = 0xff; /* isp.ko's .data 0x130: the last sensor */
static u8 dvp_awb = 1;     /* .data 0x138: the channel of its last frame */
static u32 frame_no = 1;   /* .data 0x134 */

#define VB(n, o) W32(vbuf[n], o)

int vicu_chn_init(const u8 *a)
{
	u32 i, u, vo, r40, r44, ra, rb, rc, rd, off;

	for (i = 0; i < N_CHN; i++) {
		if (!(a[1] >> i & 1))
			continue;
		vo = i * 0xd0;
		C32(CM_VICU + 4 + vo) = a[0];
		C32(CM_VICU + vo) = a[7];
		C32(0x128 + vo) = *(u32 *)(a + 0x30);
		u = *(u32 *)(a + 0x18);
		C32(0x1c4 + vo) = (((u & 0xffff) + 9) / 10) * (u >> 16) * 0x10;
		if (!C8(CM_CHNNUM)) {
			pr_err("IspKo:Err: not config vicu num\n");
			continue;
		}
		if (CS8(CM_OPT) < 0)
			continue;
		if (i == 1) {
			isp_write_reg(0x30001000, isp_read_reg(0x30001000) | 6);
			isp_write_reg(0x30000004, 2);
		}
		if (i == 0) {
			r40 = 0x30047040, r44 = 0x30047044;
			ra = 0x30043014, rb = 0x30043010, rc = 0x3004300c;
			rd = 0x30043018, off = 0;
		} else {
			r40 = 0x30057040, r44 = 0x30057044;
			ra = 0x30053014, rb = 0x30053010, rc = 0x3005300c;
			rd = 0x30053018, off = 0x10000;
		}
		isp_write_reg(rd, 0);
		isp_write_reg(rc, 0);
		isp_write_reg(rb, 0xfff);
		isp_write_reg(ra, 0x100);
		if (C8(CM_MODE) == 1) {
			isp_write_reg(0x30001000, isp_read_reg(0x30001000) | 4);
			if (i == 0) {
				isp_write_reg(0x30060048, 0);
				isp_write_reg(0x30060038, 0);
				isp_write_reg(0x30060040, 0);
				isp_write_reg(0x30060054, 1);
			} else {
				isp_write_reg(0x30082048, 0);
				isp_write_reg(0x30082038, 0);
				isp_write_reg(0x30082040, 0);
				isp_write_reg(0x30082054, 1);
			}
			C32(0x1c8 + vo) = 1;
			isp_write_reg(r44, 2);
			isp_write_reg(r40, 1);
			isp_write_reg(off + 0x30047054, *(u16 *)(a + 0xe) * 15 >> 4);
			isp_write_reg(off + 0x30047050, 1);
		} else {
			C32(0x1c8 + vo) = 1;
			isp_write_reg(r44, 1);
			isp_write_reg(r40, 1);
			/* (isp.ko: the second VICU's, whichever channel) */
			isp_write_reg(0x30057044, 1);
			isp_write_reg(0x30057040, 1);
		}
	}
	return 0;
}

int vicu_run(u32 chn)
{
	u32 m, k;

	if (chn >= N_CHN)
		return -1;
	m = C32(CM_GRP(chn) + 8);
	for (k = 0; k < N_CHN; k++) {
		if (!(m & 1 << k))
			continue;
		isp_write_reg(k ? 0x30057040 : 0x30047040, 1);
		if (C8(CM_MODE) == 1)
			isp_write_reg(k ? 0x30057050 : 0x30047050, 1);
	}
	return 0;
}

void vicu_reset(void)
{
	int k;

	for (k = 0; k < N_CHN; k++) {
		if (k == 0) {
			isp_write_reg(0x30047040, 0);
			isp_write_reg(0x30047050, 0);
			isp_write_reg(0x30044020, 1);
			isp_write_reg(0x30060034, 0);
		} else {
			isp_write_reg(0x30057040, 0);
			isp_write_reg(0x30057050, 0);
			isp_write_reg(0x30054020, 1);
			isp_write_reg(0x30082034, 0);
		}
		vbuf[k + 3] = vbuf[k] = vbuf[k + 6] = NULL;
		isp_write_reg(k ? 0x30052004 : 0x30042004, 0xffff);
	}
	held = 0;
	C32(CM_DVPSEL) = 0;
	C32(0x6340) = 1;
	C32(0x6344) = 1;
	C32(0x6360) = 0;
	C32(0x6358) = 1;
	C32(0x635c) = 1;
	C32(0x6364) = 0;
	C32(0x6368) = 0;
}

/* the white balance and the statistics windows' gains, VICU side */
static void vicu_wb(u32 g, u32 r, u32 gg, u32 b, u32 base, u32 st1, u32 st2)
{
	int k;

	isp_write_reg(base, r);
	isp_write_reg(base + 4, gg);
	isp_write_reg(base + 8, b);
	isp_write_reg(base + 0x9000, C32(0x2cb0 + g));
	isp_write_reg(base + 0x9004, C32(0x2cb4 + g));
	isp_write_reg(base + 0x9008, C32(0x2cb8 + g));
	for (k = 0; k < 33; k++)
		isp_write_reg(st1 + k * 4, C32(0x2cc0 + g + k * 4));
	for (k = 0; k < 17; k++)
		isp_write_reg(st2 + k * 4, C32(0x2d44 + g + k * 4));
}

/* channel i's frame registers from its parameter blocks */
static void vicu_frame_regs(u32 i, u32 mode, u32 wdr)
{
	u32 g = i * 0x12d4, base = i ? 0x3005b000 : 0x3004b000;
	u32 r, gg, b, sh, k;

	for (k = 0; k < 15; k++)
		isp_write_reg(base + k * 4, C16(0x2a52 + g + k * 2));
	for (k = 0; k < 61; k++) {
		isp_write_reg(base + 0x40 + k * 4, C16(0x2a70 + g + k * 2));
		isp_write_reg(base + 0x140 + k * 4, C16(0x2aea + g + k * 2));
		isp_write_reg(base + 0x240 + k * 4, C16(0x2b64 + g + k * 2));
		isp_write_reg(base + 0x340 + k * 4, C16(0x2bde + g + k * 2));
	}
	isp_write_reg(i ? 0x30053000 : 0x30043000, C16(0x2a48 + g));
	isp_write_reg(i ? 0x30053004 : 0x30043004, C16(0x2a4a + g));
	sh = C8(0x2964 + g) == 4 ? 0 : 4;
	r = min_t(u32, C16(0x2c58 + g) << sh, 0xffff);
	gg = min_t(u32, C16(0x2c5a + g) << sh, 0xffff);
	b = min_t(u32, C16(0x2c5c + g) << sh, 0xffff);
	if (mode == 1) {
		if (i == 0) {
			isp_write_reg(0x30042000, C32(0x2cac));
			vicu_wb(0, r, gg, b, 0x30041000, 0x32011000, 0x320110c0);
		} else {
			isp_write_reg(0x30052000, C32(0x2cac + g));
			vicu_wb(g, r, gg, b, 0x30051000, 0x30058000, 0x300580c0);
		}
	} else if (wdr == 1) {
		isp_write_reg(0x30042000, 0x100);
		isp_write_reg(0x30052000, 0x100);
		isp_write_reg(0x30042004, 0xffff);
		isp_write_reg(0x30052004, 0xffff);
		isp_write_reg(0x30041000, 0);
		isp_write_reg(0x30041004, 0);
		isp_write_reg(0x30041008, 0);
		isp_write_reg(0x3004a000, 0x100);
		isp_write_reg(0x3004a004, 0x100);
		isp_write_reg(0x3004a008, 0x100);
	} else {
		isp_write_reg(0x30042000, C32(0x2cac + g));
		isp_write_reg(0x30052000, C32(0x2cac + g));
		isp_write_reg(0x30042004, 0xffff);
		isp_write_reg(0x30052004, 0xffff);
		vicu_wb(g, r, gg, b, 0x30041000, 0x32011000, 0x320110c0);
		vicu_wb(g, r, gg, b, 0x30051000, 0x30058000, 0x300580c0);
	}
}

/* the time-shared VICU's sensor for this frame; 1: not counted as one */
static int vicu_dvpsel(bool *new_sensor)
{
	unsigned long flags;
	u32 flag = 0, v;
	u8 nw;
	int skip = 0;

	*new_sensor = false;
	spin_lock_irqsave(&dvp_lock, flags);
	if (C32(CM_DVPSEL) == 0) {
		C32(0x6358) = 1;
		C32(0x6360) = 0;
		C32(0x6364) = 0;
		dvp_last = 0xff;
	} else if (C32(0x6340) == 1) {
		if (C32(0x6344) == 0) {
			C32(0x6358) = 1;
			v = 2;
		} else {
			C32(0x6358) = C32(0x6344) & 0xff;
			v = (C32(0x6344) & 0xff) == 2 ? 5 : 2;
		}
		C32(0x6360) = v;
		C32(0x6364) = 0;
		nw = (C32(0x6358) & 0xf) | 0x10;
		if (dvp_last == nw && dvp_same > 1) {
			dvp_last = nw;
			goto out;
		}
		if (dvp_last == nw) {
			dvp_same++;
		} else {
			dvp_same = 1;
			*new_sensor = true;
		}
		dvp_last = nw;
		C32(0x6364) = 0x11;
		C32(0x6368) = 3;
		flag = 0x10;
		skip = 1;
	} else {
		C32(0x6358) = C32(0x6360) / 3 + 1;
		C32(0x6364) = C32(0x6360) % 3 == 1 ? 0x10 : 0x11;
		v = C32(0x6360)++;
		skip = v % 3 != 2;
		if (v + 1 > 5)
			C32(0x6360) = 0;
		flag = skip << 4;
		dvp_last = C32(0x6358) & 0xf;
	}
out:
	C32(CM_DVP_STATE) = (C32(0x6358) & 0xf) | 0x80000000 | flag;
	wake_up(&dvp_wq);
	spin_unlock_irqrestore(&dvp_lock, flags);
	return skip;
}

/* offline: the buffer stored last to the BPU, the next one to the VICU */
static void vicu_offline(u32 i, bool new_sensor, u32 *awb)
{
	u32 vo = i * 0xd0, off = i ? 0x22000 : 0, a;
	void *cur = vbuf[i + 3], *nxt = vbuf[i + 6], *stored = NULL;

	vbuf[i] = cur;
	vbuf[i + 3] = nxt;
	if (!cur && !nxt) {
		C32(0x170 + vo) = 0;
		C32(0x174 + vo) = 0;
		goto next;
	}
	if (cur) {
		if (W32(cur, 0x2c) == 1) { /* stored whole */
			if (C8(CM_DVP) == i && ++frame_no > 0xffff)
				frame_no = 1;
			W32(cur, 0x30) = frame_no;
			W32(cur, 0x34) = C32(CM_VICU + vo);
			C32(0x1c0 + vo) = i;
			C32(0x1b8 + vo) = 3;
			C32(0x1bc + vo) = C32(CM_VICU + 4 + vo);
			W32(cur, 0x20) = 3; /* bound to the BPU */
			W32(cur, 0x24) = C32(0x1bc + vo);
			W32(cur, 0x28) = i;
			W32(cur, 0x18) = C16(0xa0 + i * 0x28);
			W32(cur, 0x1c) = C16(0xa2 + i * 0x28);
			W32(cur, 0x38) = 0;
			ib_put_buf(1, 0x10, cur);
			C32(0x164 + vo)++;
		} else {
			ib_put_buf(3, 0, cur);
		}
	}
	if (vbuf[i + 3]) {
		stored = vbuf[i + 3];
		if (new_sensor) {
			W32(stored, 0x2c) = 0;
		} else {
			a = isp_read_reg(off + 0x30060100);
			W32(stored, 0x2c) =
				a != isp_read_reg(off + 0x30060108);
		}
	}
	C32(0x170 + vo) = vbuf[i] ? W32(vbuf[i], 0x10) : 0;
	C32(0x174 + vo) = stored ? W32(stored, 0x10) : 0;
next:
	C32(0x178 + vo) = vbuf[i + 6] ? W32(vbuf[i + 6], 0x10) : 0;
	vbuf[i + 6] = NULL;
	if (i) {
		*awb = dvp_awb;
		dvp_awb = i;
	} else {
		*awb = 0;
	}
}

/* the raw-frame dump on channel i's VICU: 1 when its waiter is woken */
static void vicu_dump(u32 i)
{
	u32 off = i ? 0x22000 : 0, v;
	u8 res;

	if (i != C8(CM_DUMP_CHN))
		return;
	switch (dump_state) {
	case 0:
		isp_write_reg(off + 0x30060048, 0);
		isp_write_reg(off + 0x30060000,
			      (C32(CM_DUMP_ADDR) + 0x80000000) >> 4);
		v = C16(CM_DUMP_W) / (i ? 0x10 : 10);
		isp_write_reg(off + 0x30060020, v);
		isp_write_reg(off + 0x30060024, v);
		isp_write_reg(off + 0x30060034, 1);
		isp_write_reg(0x30000014, C32(CM_DUMP_SIZE));
		dump_state = 1;
		return;
	case 1:
		if (isp_read_reg(off + 0x30060100) !=
		    isp_read_reg(off + 0x30060108)) {
			dump_state = 2;
			isp_write_reg(off + 0x30060034, 0);
			return;
		}
		isp_write_reg(off + 0x30060034, 0);
		res = 0x20;
		break;
	case 2:
		res = 0x10;
		break;
	default:
		return;
	}
	C8(CM_DUMP) = res;
	dump_state = 0;
	isp_write_reg(0x30000014, 0);
	wake_up(&dump_wq);
}

static void vicu_fps(u32 ch, u64 pts)
{
	u32 o = ch * 0xd0;
	u64 prev = *(u64 *)CP(0x188 + o), dt;
	u32 fps = 0xff;

	if (pts > prev) {
		dt = pts - prev;
		if (dt >> 32)
			dt = 0xffffffff;
		if (dt)
			fps = (100000000 / (u32)dt + 5) / 10;
	}
	C32(0x148 + o) = fps;
	*(u64 *)CP(0x190 + o) = prev;
	*(u64 *)CP(0x188 + o) = pts;
}

irqreturn_t vicu_irq_0(int irq, void *dev)
{
	u32 st = 0, i = 0, ch, next, mode, wdr, mask, awb, cnt, o;
	unsigned long flags;
	bool new_sensor = false, wake;
	int skip = 0;
	u64 pts;

	if (isp_read_reg(0x3004704c)) {
		st = isp_read_reg(0x30047008) != 0;
		isp_write_reg(0x3004704c, 0);
	}
	if (isp_read_reg(0x3005704c)) {
		if (isp_read_reg(0x30057008))
			st |= 2;
		isp_write_reg(0x3005704c, 0);
	}
	if (C32(CM_RUN) != 1 || !st)
		return IRQ_HANDLED;
	clk_get_pts(&pts);
	while (i < 2) {
		if (!(st >> i & 1)) {
			i++;
			continue;
		}
		if (!C8(CM_BPU_BUSY))
			return IRQ_HANDLED;
		if (i == 0) {
			ch = 0;
			next = 1;
		} else {
			skip = vicu_dvpsel(&new_sensor);
			ch = C32(0x635c) & 0xff;
			next = ch + 1;
			if (ch >= N_CHN) { /* as isp.ko: past the channels */
				i = next;
				continue;
			}
		}
		spin_lock_irqsave(&awb_lock[ch], flags);
		mode = C8(CM_MODE);
		mask = 1 << ch;
		wdr = C32(CM_GRP(ch) + 0x38);
		if (C32(CM_IW_WAIT) == 0)
			held = 0;
		if (!held && (mode != 2 || ch == 0)) {
			vicu_frame_regs(ch, mode, wdr);
			if (mask & C32(CM_IW_WAIT))
				held = 1;
		}
		mode = C8(CM_MODE);
		wake = false;
		if (C8(CM_DUMP) & 1) {
			vicu_dump(ch);
		} else if (mode != 1) {
			if (bpu_post_deal(ch, C8(CM_MODE),
					  C32(CM_GRP(ch) + 0x38)) == 0) {
				wake = true;
				C32(CM_BPU_GO) = mask;
			}
		} else {
			vicu_offline(ch, new_sensor, &awb);
			awb_stat_get(0, awb);
			if ((C8(CM_OPT) & 3) >= 2) {
				wake = true;
				C32(CM_BPU_GO) = 1 << awb;
			}
		}
		if (!skip)
			vicu_fps(ch, pts);
		o = ch * 0xd0;
		cnt = C32(0x14c + o) + 1;
		C32(0x14c + o) = cnt > 0xffff ? 0 : cnt;
		C32(0x140 + o) = ch;
		C32(0x144 + o) = 1;
		*(u64 *)CP(0x180 + o) = pts;
		*(u64 *)CP(0x198 + o) = pts;
		C32(CM_VSYNC_CNT) |= mask;
		wake_up(&vsync_wq);
		if (wake)
			wake_up(&bpu_thread_wq);
		spin_unlock_irqrestore(&awb_lock[ch], flags);
		i = next;
	}
	return IRQ_HANDLED;
}

irqreturn_t vicu_irq_1(int irq, void *dev)
{
	u32 st = 0, j = 0, ch, flag = 0, vo, reg, off, wd, div, v;
	unsigned long flags;
	void *n;
	u64 pts;

	if (isp_read_reg(0x3004705c)) {
		st = isp_read_reg(0x30047008) != 0;
		isp_write_reg(0x3004705c, 0);
	}
	if (isp_read_reg(0x3005705c)) {
		if (isp_read_reg(0x30057008))
			st |= 2;
		isp_write_reg(0x3005705c, 0);
	}
	if (C32(CM_RUN) != 1 || !st || C8(CM_MODE) != 1)
		return IRQ_HANDLED;
	clk_get_pts(&pts);
	while (j < 2) {
		if (!(st >> j & 1)) {
			j++;
			continue;
		}
		if (!C8(CM_BPU_BUSY))
			return IRQ_HANDLED;
		if (j == 0) {
			j = 1;
			ch = 0;
		} else {
			ch = C32(0x635c);
			flag = C32(0x6364) & 0xff;
			C32(0x635c) = C32(0x6358);
			if ((C8(CM_OPT) & 3) < 2)
				ch = C32(0x635c);
			ch &= 0xff;
			j = ch + 1;
			if (ch >= N_CHN)
				continue;
		}
		spin_lock_irqsave(&awb_lock[ch], flags);
		if (!(C8(CM_DUMP) & 1) && !vbuf[ch + 6]) {
			if (!(flag & 0x10) && (C8(CM_OPT) & 3) == 1)
				flag = C32(0x14c + ch * 0xd0) & 1;
			vo = ch * 0xd0;
			reg = ch ? 0x30082034 : 0x30060034;
			off = ch ? 0x22000 : 0;
			n = NULL;
			if (C32(0x1c8 + vo) && !(flag & 1)) {
				if (ib_get_buf(ch, 0, C32(0x1c4 + vo), &vbuf[ch + 6],
					       0) == 0)
					C32(0x16c + vo)++;
				else
					C32(0x168 + vo)++;
				n = vbuf[ch + 6];
			}
			if (n) {
				wd = C32(0xa0 + ch * 0x28) & 0xffff;
				isp_write_reg(off + 0x30060000,
					      (W32(n, 0x10) + 0x80000000) >> 4);
				isp_write_reg(off + 0x30060080,
					      (wd * ((C32(0xa0 + ch * 0x28) >> 16) - 1) +
					       W32(n, 0x10) - 0x80000000) >> 4);
				div = ch ? 0x10 : 10;
				v = (wd - 1 + div) / div;
				isp_write_reg(off + 0x30060020, v);
				isp_write_reg(off + 0x30060024, v);
				isp_write_reg(reg, 1);
				W32(n, 0x3c) = C32(0x184 + vo);
				W32(n, 0x40) = C32(0x180 + vo);
			} else {
				isp_write_reg(reg, 0);
			}
		}
		bpu_offline_deal(ch, 0);
		spin_unlock_irqrestore(&awb_lock[ch], flags);
	}
	return IRQ_HANDLED;
}
