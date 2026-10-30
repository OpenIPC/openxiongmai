// SPDX-License-Identifier: GPL-2.0
/*
 * The two VICUs (the sensors' frames into the ISP), IRQ 87 at the frame's
 * start and IRQ 88 at its stored end. At the start (in a tasklet) the
 * channel's VICU block goes out (curves, black level; offline also the
 * white balance and statistics windows), the AWB statistics come in, and
 * offline the buffer the VICU stored last is handed to the BPU. At the end
 * (offline only) the VICU gets the next ISP buffer to store into. The
 * second VICU can be time-shared by sensors (0x4912, "dvpsel").
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
	u32 i, u, vo, r40, r44, off;

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
			isp_write_reg(0x30001000, isp_read_reg(0x30001000) | 2);
			isp_write_reg(0x30000004, 2);
		}
		off = i ? 0x10000 : 0;
		r40 = off + 0x30048040;
		r44 = off + 0x30048044;
		isp_write_reg(off + 0x30047018, 0);
		isp_write_reg(off + 0x3004700c, 0);
		isp_write_reg(off + 0x30047010, 0xfff);
		isp_write_reg(off + 0x30047014, 0x100);
		C32(0x1c8 + vo) = 1;
		if (C8(CM_MODE) == 1) {
			u = i ? 0x1000 : 0;
			isp_write_reg(u + 0x30010048, 0);
			isp_write_reg(u + 0x30010038, 0);
			isp_write_reg(u + 0x30010040, 0);
			isp_write_reg(u + 0x30010054, 3);
			isp_write_reg(r44, 2);
			isp_write_reg(r40, 1);
			isp_write_reg(off + 0x30048054, *(u16 *)(a + 0xe) * 15 >> 4);
			isp_write_reg(off + 0x30048050, 1);
		} else {
			isp_write_reg(r44, 1);
			isp_write_reg(r40, 1);
			/* (isp.ko: the second VICU's, whichever channel) */
			isp_write_reg(0x30058044, 1);
			isp_write_reg(0x30058040, 1);
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
		isp_write_reg(k ? 0x30058040 : 0x30048040, 1);
		if (C8(CM_MODE) == 1)
			isp_write_reg(k ? 0x30058050 : 0x30048050, 1);
	}
	return 0;
}

void vicu_reset(void)
{
	int k;

	for (k = 0; k < N_CHN; k++) {
		isp_write_reg(k ? 0x30058040 : 0x30048040, 0);
		isp_write_reg(k ? 0x30058050 : 0x30048050, 0);
		isp_write_reg(k ? 0x30011034 : 0x30010034, 0);
		vbuf[k + 3] = vbuf[k] = vbuf[k + 6] = NULL;
		isp_write_reg(k ? 0x30053004 : 0x30043004, 0xffff);
	}
	held = 0;
	C32(CM_DVPSEL) = 0;
	C32(CM_DVPSEL + 4) = 1;
	C32(CM_DVPSEL + 8) = 1;
	C32(CM_DVPSEL + 0x24) = 0;
	C32(CM_DVPSEL + 0x1c) = 1;
	C32(CM_DVPSEL + 0x20) = 1;
	C32(CM_DVPSEL + 0x28) = 0;
	C32(CM_DVPSEL + 0x2c) = 0;
}

/* channel i's VICU block to VICU v's registers (online: both VICUs) */
static void vicu_wb(u32 vb, u32 pb, u32 r, u32 g, u32 b)
{
	int k;

	isp_write_reg(pb + 0x1000, r);
	isp_write_reg(pb + 0x1004, g);
	isp_write_reg(pb + 0x1008, b);
	isp_write_reg(pb + 0x4000, C32(vb + 0x220));
	isp_write_reg(pb + 0x4004, C32(vb + 0x224));
	isp_write_reg(pb + 0x4008, C32(vb + 0x228));
	for (k = 0; k < 33; k++)
		isp_write_reg(pb + 0x5000 + k * 4, C32(vb + 0x230 + k * 4));
	for (k = 0; k < 17; k++)
		isp_write_reg(pb + 0x50c0 + k * 4, C32(vb + 0x2b4 + k * 4));
}

/* channel i's frame registers from its VICU block */
static void vicu_frame_regs(u32 i, u32 mode, u32 wdr)
{
	u32 vb = CM_VB(i), base = i ? 0x30052000 : 0x30042000;
	u32 r, g, b, k;

	for (k = 0; k < 15; k++)
		isp_write_reg(base + k * 4, C16(vb + 0xe + k * 2));
	for (k = 0; k < 61; k++) {
		isp_write_reg(base + 0x40 + k * 4, C16(vb + 0x2c + k * 2));
		isp_write_reg(base + 0x140 + k * 4, C16(vb + 0xa6 + k * 2));
		isp_write_reg(base + 0x240 + k * 4, C16(vb + 0x120 + k * 2));
		isp_write_reg(base + 0x340 + k * 4, C16(vb + 0x19a + k * 2));
	}
	isp_write_reg(i ? 0x30057000 : 0x30047000, C16(vb + 4));
	isp_write_reg(i ? 0x30057004 : 0x30047004, C16(vb + 6));
	r = C16(vb + 0x214);
	g = C16(vb + 0x216);
	b = C16(vb + 0x218);
	if (mode == 1) {
		isp_write_reg(i ? 0x30053000 : 0x30043000, C32(vb + 0x21c));
		vicu_wb(vb, i ? 0x30050000 : 0x30040000, r, g, b);
	} else if (wdr == 1) {
		isp_write_reg(0x30043000, 0x100);
		isp_write_reg(0x30053000, 0x100);
		isp_write_reg(0x30043004, 0xffff);
		isp_write_reg(0x30053004, 0xffff);
		isp_write_reg(0x30041000, 0);
		isp_write_reg(0x30041004, 0);
		isp_write_reg(0x30041008, 0);
		isp_write_reg(0x30044000, 0x100);
		isp_write_reg(0x30044004, 0x100);
		isp_write_reg(0x30044008, 0x100);
	} else {
		isp_write_reg(0x30043000, C32(vb + 0x21c));
		isp_write_reg(0x30053000, C32(vb + 0x21c));
		isp_write_reg(0x30043004, 0xffff);
		isp_write_reg(0x30053004, 0xffff);
		vicu_wb(vb, 0x30040000, r, g, b);
		vicu_wb(vb, 0x30050000, r, g, b);
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
		C32(CM_DVPSEL + 0x1c) = 1;
		C32(CM_DVPSEL + 0x24) = 0;
		C32(CM_DVPSEL + 0x28) = 0;
		dvp_last = 0xff;
	} else if (C32(CM_DVPSEL + 4) == 1) {
		if (C32(CM_DVPSEL + 8) == 0) {
			C32(CM_DVPSEL + 0x1c) = 1;
			v = 2;
		} else {
			C32(CM_DVPSEL + 0x1c) = C32(CM_DVPSEL + 8) & 0xff;
			v = (C32(CM_DVPSEL + 8) & 0xff) == 2 ? 5 : 2;
		}
		C32(CM_DVPSEL + 0x24) = v;
		C32(CM_DVPSEL + 0x28) = 0;
		nw = (C32(CM_DVPSEL + 0x1c) & 0xf) | 0x10;
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
		C32(CM_DVPSEL + 0x28) = 0x11;
		C32(CM_DVPSEL + 0x2c) = 3;
		flag = 0x10;
		skip = 1;
	} else {
		C32(CM_DVPSEL + 0x1c) = C32(CM_DVPSEL + 0x24) / 3 + 1;
		C32(CM_DVPSEL + 0x28) = C32(CM_DVPSEL + 0x24) % 3 == 1 ? 0x10 : 0x11;
		v = C32(CM_DVPSEL + 0x24)++;
		skip = v % 3 != 2;
		if (v + 1 > 5)
			C32(CM_DVPSEL + 0x24) = 0;
		flag = skip << 4;
		dvp_last = C32(CM_DVPSEL + 0x1c) & 0xf;
	}
out:
	C32(CM_DVP_STATE) = (C32(CM_DVPSEL + 0x1c) & 0xf) | 0x80000000 | flag;
	wake_up(&dvp_wq);
	spin_unlock_irqrestore(&dvp_lock, flags);
	return skip;
}

/* offline: the buffer stored last to the BPU, the next one to the VICU */
static void vicu_offline(u32 i, bool new_sensor, u32 *awb)
{
	u32 vo = i * 0xd0, off = i ? 0x1000 : 0, a;
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
			W32(cur, 0x18) = C16(CM_CHN + 8 + i * 0x28);
			W32(cur, 0x1c) = C16(CM_CHN + 0xa + i * 0x28);
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
			a = isp_read_reg(off + 0x30010100);
			W32(stored, 0x2c) =
				a != isp_read_reg(off + 0x30010108);
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
	u32 off = i ? 0x1000 : 0, v;
	u8 res;

	if (i != C8(CM_DUMP_CHN))
		return;
	switch (dump_state) {
	case 0:
		isp_write_reg(off + 0x30010048, 0);
		isp_write_reg(off + 0x30010000,
			      (C32(CM_DUMP_ADDR) + 0x80000000) >> 4);
		v = C16(CM_DUMP_W) / 10;
		isp_write_reg(off + 0x30010020, v);
		isp_write_reg(off + 0x30010024, v);
		isp_write_reg(off + 0x30010058, v * (C16(CM_DUMP_H) + 1));
		isp_write_reg(off + 0x30010034, 1);
		isp_write_reg(0x30000014, C32(CM_DUMP_SIZE));
		dump_state = 1;
		return;
	case 1:
		if (isp_read_reg(off + 0x30010100) !=
		    isp_read_reg(off + 0x30010108)) {
			dump_state = 2;
			isp_write_reg(off + 0x30010034, 0);
			return;
		}
		isp_write_reg(off + 0x30010034, 0);
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

static DECLARE_TASKLET(isp_tasklet, vicu_frame, 0);

/* the frame interrupt: the frame work in a tasklet */
irqreturn_t vicu_irq_0(int irq, void *dev)
{
	u32 st = 0;

	if (isp_read_reg(0x3004804c)) {
		st = isp_read_reg(0x30048008) != 0;
		isp_write_reg(0x3004804c, 0);
	}
	if (isp_read_reg(0x3005804c)) {
		if (isp_read_reg(0x30058008))
			st |= 2;
		isp_write_reg(0x3005804c, 0);
	}
	isp_tasklet.data = st;
	tasklet_schedule(&isp_tasklet);
	return IRQ_HANDLED;
}

void vicu_kill(void)
{
	tasklet_kill(&isp_tasklet);
}

/* the frame work of the VICUs in st */
void vicu_frame(unsigned long st)
{
	u32 i = 0, ch, next, mode, wdr, mask, awb, cnt, o;
	unsigned long flags;
	bool new_sensor = false, wake;
	int skip = 0;
	u64 pts;

	if (C32(CM_RUN) != 1 || !st)
		return;
	clk_get_pts(&pts);
	while (i < 2) {
		if (!(st >> i & 1)) {
			i++;
			continue;
		}
		if (!C8(CM_BPU_BUSY))
			return;
		if (i == 0) {
			ch = 0;
			next = 1;
		} else {
			skip = vicu_dvpsel(&new_sensor);
			ch = C32(CM_DVPSEL + 0x20) & 0xff;
			next = ch + 1;
			if (ch >= N_CHN) { /* as isp.ko: past the channels */
				i = next;
				continue;
			}
		}
		spin_lock_irqsave(&awb_lock[ch], flags);
		mode = C8(CM_MODE);
		mask = 1 << ch;
		wdr = C32(CM_GRP(ch) + 0x2c);
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
			if (bpu_post_deal(ch, C8(CM_MODE)) == 0) {
				wake = true;
				C32(CM_BPU_GO) = mask;
			}
		} else {
			vicu_offline(ch, new_sensor, &awb);
			awb_stat_get(0, awb);
			if ((C8(CM_OPT) & 3) >= 2) {
				wake = true;
				C32(CM_BPU_GO) |= 1 << awb;
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
}

irqreturn_t vicu_irq_1(int irq, void *dev)
{
	u32 st = 0, j = 0, ch, flag = 0, vo, reg, off, wd, ht, v;
	unsigned long flags;
	void *n;
	u64 pts;

	if (isp_read_reg(0x3004805c)) {
		st = isp_read_reg(0x30048008) != 0;
		isp_write_reg(0x3004805c, 0);
	}
	if (isp_read_reg(0x3005805c)) {
		if (isp_read_reg(0x30058008))
			st |= 2;
		isp_write_reg(0x3005805c, 0);
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
			ch = C32(CM_DVPSEL + 0x20);
			flag = C32(CM_DVPSEL + 0x28) & 0xff;
			C32(CM_DVPSEL + 0x20) = C32(CM_DVPSEL + 0x1c);
			if ((C8(CM_OPT) & 3) < 2)
				ch = C32(CM_DVPSEL + 0x20);
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
			off = ch ? 0x1000 : 0;
			reg = off + 0x30010034;
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
				wd = C32(CM_CHN + 8 + ch * 0x28);
				ht = wd >> 16;
				v = ((wd & 0xffff) + 9) / 10;
				isp_write_reg(off + 0x30010000,
					      (W32(n, 0x10) + 0x80000000) >> 4);
				isp_write_reg(off + 0x30010080,
					      (v * (ht - 1) * 0x10 + W32(n, 0x10) -
					       0x80000000) >> 4);
				isp_write_reg(off + 0x30010020, v);
				isp_write_reg(off + 0x30010024, v);
				isp_write_reg(off + 0x30010058, (ht + 1) * v);
				isp_write_reg(off + 0x30010054, 3);
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
