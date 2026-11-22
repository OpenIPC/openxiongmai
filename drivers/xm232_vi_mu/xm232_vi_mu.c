// SPDX-License-Identifier: GPL-2.0
/*
 * The XM232's video input (/dev/vi): a drop-in for the vendor vi_mu.ko.
 *
 * One capture path is time-multiplexed between up to two sensors: isp.ko
 * switches the VICU between them every frame (isp_set_next_ch) and the
 * store writes the VPSS's three outputs of the sensor captured to memory,
 * into vb.ko's buffers. A frame period is one pass of the thread here:
 *
 *   - vi_sel (IRQ 89, raw +0x10020, masked +0x10028 through +0x10024)
 *     wakes it; with a VICU end event (0x10101) it waits for the VICU's
 *     own end flag (+0x7104C, polled each jiffy, at most vic_timeout_num)
 *   - the buffers the store was given for the frame just captured are
 *     done: handed on (vb.ko list 0, or 2 for a buffer bound to VENC) or
 *     queued for user space; in the split modes (1, 3) the sensors share
 *     one buffer, spliced top to bottom, and it goes on once every
 *     sensor's part is in (+0x4c's bits)
 *   - the next sensor goes to isp.ko, then each channel that takes the
 *     next frame gets a buffer and the register image is written to the
 *     store (vi_update_regs), latched by +0x10014
 *
 * Modes (mode, sensor_max): 0 one sensor's picture each; 1 split, the
 * sensors one above the other; 2 channel 0 into a line buffer of "line"
 * lines (online VENC); 3 split with channel 0 into the line buffer.
 * memcomp: the line buffer from mmz.ko's compressed blocks; rst: reset the
 * VI at init (SYS +0x11C).
 *
 * The ioctls, records and /proc/umap/vi are vi_mu.ko's:
 *   0x5600      init
 *   0x5632/33   accepted, nothing done
 *   0x40045603  enable {chn} / 0x40045604 disable
 *   0x40505601  a device's attributes (ignored)
 *   0x40505602  a channel's attributes: words 4..14 capture x, y, w, h,
 *               pixel format, output w, h, mirror, flip, source and
 *               output frame rates
 *   0x40505605  bind: words 4..6 copied to each buffer's +0x40..+0x48
 *   0x40505606  the channel's queue depth for user space (word 4)
 *   0x40505607  a picture from the queue, waiting word 2 ms: words 4..14
 *               luma, chroma, -, format, stride, w, h, -, pts, buffer
 *   0x40505608  that picture back (word 14, the buffer)
 *   0x4a18560e  a channel's OSD background (0xa18 bytes, word 0 the
 *               channel) / 0x4a18560f read back
 *   0x80245610  the version
 * Exports vi_get_sensor_type; registers its power hook with isp.ko.
 *
 * The state is kept at vi_mu.ko's offsets: VI() is its g_vi_info, a
 * channel record of 0xf8 bytes from +0x58 (index sensor * 3 + output),
 * the store's register image at +0xbe0 (a word for each of the three
 * store channels a register), the thread's state at +0xdbc.. .
 */

#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/kthread.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/proc_fs.h>
#include <linux/sched.h>
#include <linux/seq_file.h>
#include <linux/uaccess.h>
#include <linux/wait.h>

#include "frtc.h"

#define VI_BASE 0x30000000
#define VI_SIZE 0x100000
#define SYS_BASE 0x20000000
#define VI_RESET 0x11c /* SYS: the VI's reset */
#define VI_SEL_IRQ 89
#define VI_VER 0x2321001

#define R_SEL_GO 0x10014
#define R_SEL_RAW 0x10020
#define R_SEL_EN 0x10024
#define R_SEL_ST 0x10028
#define R_FRONT_LINE(n) (0x211c + (n) * 0x1000)
#define R_VIC_CTRL0 0x71040
#define R_VIC_CTRL1 0x71044
#define R_VIC_END 0x7104c
#define R_VIC_LINE 0x71080
#define R_ST_DONE(c) (0x73090 + (c) * 0x4000)
#define R_ST_ACK(c) (0x73094 + (c) * 0x4000)
#define SEL_MASK 0x3554515
#define VICU_END 0x10101

#define N_STORE 3
#define N_CHN 12 /* vi_mu.ko's tables: four sensors, three outputs each */
#define DDR 0x80000000u

/* g_vi_info, 0xe50 bytes */
#define INFO_SIZE 0xe50
static u8 vi_info[INFO_SIZE];
#define VI(o) (*(u32 *)(vi_info + (o)))
#define CH(i, o) VI((i) * 0xf8 + (o))
/* a channel, at its record */
#define CH_CAP_X 0x58
#define CH_CAP_Y 0x5c
#define CH_CAP_W 0x60
#define CH_CAP_H 0x64
#define CH_PIXFMT 0x68
#define CH_DST_W 0x6c
#define CH_DST_H 0x70
#define CH_MIRROR 0x74
#define CH_FLIP 0x78
#define CH_SRC_FPS 0x7c
#define CH_DST_FPS 0x80
#define CH_CROP 0x84 /* enable, x, y, w, h */
#define CH_PTS 0x98  /* the last frame's, 64 bits */
#define CH_FRMTIME 0xa0
#define CH_VB_OK 0xa4
#define CH_VB_FAIL 0xa8
#define CH_VB_DUP 0xac
#define CH_INT 0xb0
#define CH_SENT 0xb4
#define CH_LAST 0xb8
#define CH_NOW 0xbc
#define CH_NEXT 0xc0
#define CH_SCALE 0xc8 /* 15 words, the scaler's settings, to 0x100 */
#define CH_BIND 0x104 /* 3 words, each buffer's +0x40..0x48 */
#define CH_BUFSZ 0x110
#define CH_ENABLE 0x114
#define CH_LIST 0x11c /* the queue for user space */
#define CH_QLEN 0x124
#define CH_DEPTH 0x128
#define CH_WQ 0x12c
/* the store's register image: a word for each store channel */
#define IMG 0xbe0
#define IM(o, c) VI((o) + (c) * 4)
#define CUR_SNS 0xdbc
#define START_WQ 0xdc0
#define STATE 0xdc8
#define INT_CNT 0xdcc /* 26 words: vi_sel's events */
#define SW_TIMEOUT 0xe4c

#define LIST(i) ((struct list_head *)(vi_info + (i) * 0xf8 + CH_LIST))
#define CHWQ(i) ((wait_queue_head_t *)(vi_info + (i) * 0xf8 + CH_WQ))
#define STARTWQ ((wait_queue_head_t *)(vi_info + START_WQ))

/* vb.ko's buffer descriptor (80 bytes on the XM232): a list head first */
#define B(b, o) (*(u32 *)((u8 *)(b) + (o)))
#define B_ADDR 0x10
#define B_COMP 0x18 /* nonzero: not a buffer this store can use */
#define B_STRIDE 0x1c
#define B_W 0x20
#define B_H 0x24
#define B_LINES 0x28
#define B_PTS 0x30
#define B_MOD 0x40 /* the bound module: 7 VENC */
#define B_SPLICE 0x4c /* the sensors whose part is in */

static int g_line_depth;
module_param_named(line, g_line_depth, int, 0444);
static int sensor_max = 2;
module_param(sensor_max, int, 0444);
static int rst = 1;
module_param(rst, int, 0444);
static int memcomp = 1;
module_param(memcomp, int, 0444);
static int mode;
module_param(mode, int, 0444);

static int vic_timeout_num = 2;
static int vi_dev_open;
static void __iomem *reg_vi_base_va;
static void __iomem *sys_base;
static struct task_struct *ptask;
static void *now[N_CHN], *next[N_CHN], *last[N_CHN], *yc3d[4];
static struct {
	void *buf;
	u32 pad[7];
} g_vi_splice[N_STORE];
static u8 g_line_vb[0x50]; /* a buffer descriptor for the line buffer */
#define BG_SIZE 0xa18
static u8 g_bg_info[N_CHN][BG_SIZE];
static u8 bd_info[BG_SIZE];
static u32 g_vi_save;
/*
 * The pictures handed to user space (0x40505607), which alone 0x40505608
 * takes back: vi_mu.ko frees whatever pointer it is given.
 */
#define N_GIVEN 32
static void *given[N_GIVEN];

static void given_add(void *b)
{
	int k;

	for (k = 0; k < N_GIVEN; k++)
		if (!given[k]) {
			given[k] = b;
			return;
		}
}

static bool given_take(void *b)
{
	int k;

	for (k = 0; k < N_GIVEN; k++)
		if (b && given[k] == b) {
			given[k] = NULL;
			return true;
		}
	return false;
}

/* vb.ko, mmz.ko, isp.ko */
int vb_get_buf(u32 list, u32 size, void **out, int ms);
int vb_put_buf(u32 list, void *buf);
int vb_add_buf_reference(void *buf);
int mmz_alloc(const char *name, u32 size, u32 *phys);
int mmz_alloc_comp(const char *name, u32 size, u32 *phys, u32 *comp);
void clk_get_pts(u64 *pts);
struct isp_ops {
	int (*pm_set_mode)(int mode);
	void *rsvd[3];
};
int isp_register_ops(struct isp_ops ops);
int isp_set_next_ch(u32 ch, u32 state, u32 timeout, u32 rsvd);
int isp_set_complete(u32 ch);
int kthread_set_prio(struct task_struct *t, int prio);

static inline u32 rd(u32 off)
{
	u32 v = readl_relaxed(reg_vi_base_va + off);

	dsb();
	return v;
}

static inline void wr(u32 off, u32 v)
{
	dsb();
	writel_relaxed(v, reg_vi_base_va + off);
}

int vi_get_sensor_type(int *max, int *m)
{
	if (max)
		*max = sensor_max;
	if (m)
		*m = mode;
	return 0;
}
EXPORT_SYMBOL(vi_get_sensor_type);

static bool split(void)
{
	return (mode & ~2) == 1;
}

/* to what the buffer is bound: VENC's list (2) or vb.ko's own (0) */
static void vi_put_buf(void *b)
{
	vb_put_buf(B(b, B_MOD) == 7 ? 2 : 0, b);
}

static int vi_set_frame_depth(u32 chn, u32 depth)
{
	unsigned long flags;

	if (split()) {
		if (chn > 2)
			pr_err("vi: vi_get_frame mode %d chn %d err\n", mode,
			       chn);
		chn += (sensor_max - 1) * 3;
	}
	local_irq_save(flags);
	while (CH(chn, CH_QLEN)) {
		struct list_head *e = LIST(chn)->next;

		list_del(e);
		CH(chn, CH_QLEN)--;
		vi_put_buf(e);
	}
	CH(chn, CH_DEPTH) = depth;
	local_irq_restore(flags);
	return 0;
}

static irqreturn_t vi_sel_int(int irq, void *dev)
{
	unsigned long flags;
	u32 st = rd(R_SEL_ST), raw;
	int i;

	if (!st)
		return IRQ_NONE;
	local_irq_save(flags);
	VI(STATE) = st;
	raw = rd(R_SEL_RAW);
	for (i = 0; i < 26; i++)
		if (raw >> i & 1)
			VI(INT_CNT + i * 4)++;
	wr(R_SEL_RAW, 0);
	local_irq_restore(flags);
	wake_up_interruptible(STARTWQ);
	return IRQ_HANDLED;
}

/* the store's registers from the image, for each of its channels */
static void vi_update_regs(const u8 *img, void __iomem *base)
{
	static const u16 scaler[] = {
		0x04, 0x38, 0x08, 0x44, 0x0c, 0x50, 0x10, 0x5c, 0x14, 0x68,
		0x18, 0x74, 0x1c, 0x80, 0x20, 0x8c, 0x24, 0x98, 0x28, 0xa4,
		0x2c, 0xb0, 0x30, 0xbc, 0x00, 0x2c, 0x34, 0xc8, 0x3c, 0xd4,
	};
	static const u32 store[] = {
		0x72000, 0xe0,	0x72004, 0xec,	0x72008, 0xf8,	0x7200c, 0x104,
		0x73018, 0x110, 0x7301c, 0x11c, 0x73034, 0x128, 0x73038, 0x134,
		0x73040, 0x140, 0x73044, 0x158, 0x73004, 0x164, 0x73008, 0x170,
		0x73024, 0x17c, 0x73020, 0x188, 0x7303c, 0x194, 0x7306c, 0x1a0,
		0x73070, 0x1ac, 0x73074, 0x1b8, 0x73078, 0x1c4, 0x7307c, 0x1d0,
	};
	int c, i;

	for (c = 0; c < N_STORE; c++) {
		const u8 *p = img + c * 4;

		dsb();
		writel_relaxed(*(u32 *)(p + 0x14c),
			       base + 0x73000 + c * 0x4000);
		for (i = 0; i < ARRAY_SIZE(scaler); i += 2) {
			dsb();
			writel_relaxed(*(u32 *)(p + scaler[i + 1]),
				       base + 0x70000 + c * 0x40 + scaler[i]);
		}
		for (i = 0; i < ARRAY_SIZE(store); i += 2) {
			dsb();
			writel_relaxed(*(u32 *)(p + store[i + 1]),
				       base + store[i] + c * 0x4000);
		}
	}
}

/* a frame of the channel done: on, or to its queue for user space */
static void vi_deliver(int i, void *b, u64 pts)
{
	*(u64 *)((u8 *)b + B_PTS) = pts;
	if (!CH(i, CH_DEPTH)) {
		vi_put_buf(b);
	} else {
		list_add_tail((struct list_head *)b, LIST(i));
		wake_up_interruptible(CHWQ(i));
		if (++CH(i, CH_QLEN) > CH(i, CH_DEPTH)) {
			struct list_head *e = LIST(i)->next;

			list_del(e);
			CH(i, CH_QLEN)--;
			vi_put_buf(e);
		}
	}
	CH(i, CH_SENT)++;
}

/* the buffer for the next frame of output c of sensor s */
static void *vi_next_buf(int s, int c, int i, bool take)
{
	void **nx = &next[s * 3 + c];
	void *b;
	int ret;

	if (!CH(i, CH_ENABLE) || !take || !vi_dev_open)
		return *nx;
	if (split()) {
		if (s != 0) {
			*nx = g_vi_splice[c].buf;
			return *nx;
		}
		if (c == 0 && mode == 3) {
			ret = vb_get_buf(0, 0, nx, 0);
			b = *nx;
			if (b && B(b, B_COMP)) {
				vb_put_buf(0, b);
				*nx = NULL;
				g_vi_splice[0].buf = NULL;
				CH(i, CH_VB_FAIL)++;
				return NULL;
			}
		} else {
			ret = vb_get_buf(0, sensor_max * CH(i, CH_BUFSZ), nx,
					 0);
			b = *nx;
		}
		g_vi_splice[c].buf = b;
	} else if (c != 0 || !g_line_depth) {
		ret = vb_get_buf(0, CH(i, CH_BUFSZ), nx, 0);
		b = *nx;
	} else {
		ret = vb_get_buf(0, 0, nx, 0);
		b = *nx;
		if (b && B(b, B_COMP)) {
			vb_put_buf(0, b);
			*nx = NULL;
			CH(i, CH_VB_FAIL)++;
			return NULL;
		}
	}
	if (ret)
		CH(i, CH_VB_FAIL)++;
	else
		CH(i, CH_VB_OK)++;
	return b;
}

/* program store channel c for the buffer b of sensor s's channel i */
static void vi_set_store(int s, int c, int i, void *b, u32 *size,
			 u32 *csize)
{
	u32 w, h, stride, y0, lines;
	bool crop = CH(i, CH_CROP);

	if (!split() || s == 0) {
		B(b, B_MOD) = CH(i, CH_BIND);
		B(b, B_MOD + 4) = CH(i, CH_BIND + 4);
		B(b, B_MOD + 8) = CH(i, CH_BIND + 8);
		B(b, B_SPLICE) = 0;
	}
	w = crop ? CH(i, CH_CROP + 12) : CH(i, CH_DST_W);
	h = crop ? CH(i, CH_CROP + 16) : CH(i, CH_DST_H);
	stride = ALIGN(w, 32);
	B(b, B_STRIDE) = stride;
	B(b, B_W) = w;
	switch (mode) {
	case 0:
		*size = h * stride;
		B(b, B_H) = h;
		B(b, B_LINES) = 0;
		break;
	case 1:
		*size = h * stride;
		B(b, B_H) = sensor_max * h;
		B(b, B_LINES) = 0;
		break;
	case 2:
		if (c == 0) {
			B(b, B_H) = h;
			*size = g_line_depth * stride;
			B(b, B_ADDR) = *(u32 *)(g_line_vb + B_ADDR);
			B(b, B_LINES) = g_line_depth;
		} else {
			*size = h * stride;
			B(b, B_H) = h;
			B(b, B_LINES) = 0;
		}
		break;
	case 3:
		*size = h * stride;
		B(b, B_H) = sensor_max * h;
		if (c == 0) {
			B(b, B_ADDR) = *(u32 *)(g_line_vb + B_ADDR);
			B(b, B_LINES) = g_line_depth;
		} else {
			B(b, B_LINES) = 0;
		}
		break;
	}
	*csize = *size >> 1;
	if (!crop) {
		IM(0xccc, c) = w;
		IM(0xcc0, c) = 0;
		y0 = 0;
	} else {
		IM(0xccc, c) = w + CH(i, CH_CROP + 4);
		IM(0xcc0, c) = CH(i, CH_CROP + 4);
		y0 = CH(i, CH_CROP + 8);
	}
	IM(0xd38, c) = 0;
	IM(0xd44, c) = 3;
	IM(0xcd8, c) = y0;
	IM(0xce4, c) = y0 + h;
	IM(0xcf0, c) = stride >> 3;
	IM(0xcfc, c) = stride >> 3;
	IM(0xd08, c) = stride >> 3;
	IM(0xd14, c) = stride >> 3;
	IM(0xd2c, c) = 3;
	IM(0xd80, c) = 0;
	IM(0xd68, c) = (s32)*size >> 3;
	IM(0xd20, c) = CH(i, CH_MIRROR) << 2 | CH(i, CH_FLIP) << 1 |
		       CH(i, CH_MIRROR);
	IM(0xd74, c) = (s32)*csize >> 3;
	if (c == 0 && mode == 3) {
		lines = g_line_depth * stride;
		VI(0xd50) = (*size * s + B(b, B_ADDR) - DDR) >> 3;
		VI(0xd5c) = (lines + *csize * s + B(b, B_ADDR) - DDR) >> 3;
		VI(0xd8c) = (B(b, B_ADDR) + DDR) >> 3;
		VI(0xd98) = (B(b, B_ADDR) - DDR + lines) >> 3;
		if (sensor_max - 1 == s) {
			u32 t = stride * (sensor_max * h - g_line_depth);
			u32 u = stride * (g_line_depth - s * h);

			VI(0xd80) = 1;
			VI(0xda4) = t >> 3;
			VI(0xdb0) = t >> 4;
			VI(0xd68) = u >> 3;
			VI(0xd74) = u >> 4;
		}
	} else if (!split()) {
		IM(0xd50, c) = (B(b, B_ADDR) + DDR) >> 3;
		IM(0xd5c, c) = (B(b, B_ADDR) - DDR + *size) >> 3;
	} else {
		IM(0xd50, c) = (s * *size + B(b, B_ADDR) - DDR) >> 3;
		IM(0xd5c, c) = (sensor_max * *size + *csize * s +
				B(b, B_ADDR) - DDR) >> 3;
	}
}

static int vi_thread(void *unused)
{
	u32 s = 0, size = 0, csize = 0;

	while (!kthread_should_stop()) {
		unsigned long flags, t;
		u32 state, line0, line1, hi, lo, prev;
		bool tmo = false;
		u64 pts;
		long ret;
		int c;

		if (!vi_dev_open) {
			set_current_state(TASK_INTERRUPTIBLE);
			schedule_timeout(1);
			continue;
		}
		t = msecs_to_jiffies(1000);
		if ((s32)VI(STATE) < 1)
			ret = wait_event_interruptible_timeout(
				*STARTWQ, (s32)VI(STATE) > 0, t);
		else
			ret = t;
		if (ret == 0) {
			VI(SW_TIMEOUT)++;
			set_current_state(TASK_INTERRUPTIBLE);
			schedule_timeout(1);
			continue;
		}
		if (VI(STATE) & VICU_END) {
			u32 n = 0;

			if (vic_timeout_num < 0) {
				tmo = true;
			} else {
				while (!rd(R_VIC_END)) {
					set_current_state(TASK_INTERRUPTIBLE);
					n++;
					schedule_timeout(1);
					if ((int)n > vic_timeout_num) {
						tmo = true;
						break;
					}
				}
				if (!tmo)
					wr(R_VIC_END, 0);
			}
			if (tmo) {
				u32 l = rd(R_VIC_LINE);

				line0 = rd(R_FRONT_LINE(0));
				line1 = rd(R_FRONT_LINE(1));
				pr_info("vi: wait vic timeout %d state: %x line:%d %d %d\n",
					n, VI(STATE), l, line0, line1);
			}
			vic_timeout_num = 2;
		}
		clk_get_pts(&pts);

		local_irq_save(flags);
		state = VI(STATE);
		VI(STATE) = 0;
		/* the frame just stored: this sensor's buffers are done */
		for (c = 0; c < N_STORE; c++) {
			int i = s * 3 + c;
			void *b = next[i];

			next[i] = NULL;
			now[i] = b;
			if (!b) {
				CH(i, CH_NOW) = 0;
				continue;
			}
			if (state & VICU_END) {
				if (rd(R_ST_DONE(c))) {
					wr(R_ST_ACK(c), 1);
					B(now[i], B_SPLICE) |= 1 << s;
				}
				if (tmo)
					B(now[i], B_SPLICE) = 0;
				if (c == 0)
					yc3d[s] = now[i];
			}
			if (c == 0 && g_line_depth) {
				b = now[i];
				if (mode == 3) {
					if (sensor_max - 1 == s) {
						if (B(b, B_SPLICE) ==
						    ~(~0u << sensor_max)) {
							*(u64 *)((u8 *)b +
								 B_PTS) = pts;
							vi_put_buf(b);
							CH(i, CH_SENT)++;
						} else {
							memset((u8 *)b + B_MOD,
							       0, 12);
							vb_put_buf(0, now[i]);
						}
					}
				} else if (!B(b, B_SPLICE)) {
					vb_put_buf(0, b);
				} else {
					*(u64 *)((u8 *)b + B_PTS) = pts;
					vi_put_buf(b);
					CH(i, CH_SENT)++;
				}
				now[i] = NULL;
				CH(i, CH_NOW) = 0;
				continue;
			}
			b = now[i];
			CH(i, CH_NOW) = b ? B(b, B_ADDR) : 0;
		}
		/* the other sensor's: on, once complete */
		prev = (s + sensor_max - 1) % sensor_max;
		for (c = 0; c < N_STORE; c++) {
			int i = prev * 3 + c;
			void *b = now[i];

			now[i] = NULL;
			last[i] = b;
			if (b) {
				if (split()) {
					if (s == 0) {
						if (B(b, B_SPLICE) ==
						    (1u << sensor_max) - 1) {
							vi_deliver(i, b, pts);
						} else {
							memset((u8 *)b + B_MOD,
							       0, 12);
							vb_put_buf(s, last[i]);
						}
						b = last[i];
					}
				} else if (!B(b, B_SPLICE)) {
					memset((u8 *)b + B_MOD, 0, 12);
					vb_put_buf(0, last[i]);
					b = last[i];
				} else {
					vi_deliver(i, b, pts);
					b = last[i];
				}
			}
			CH(prev * 3 + c, CH_LAST) = b ? B(b, B_ADDR) : 0;
			last[i] = NULL;
		}
		local_irq_restore(flags);
		s = (s + 1) % sensor_max;
		isp_set_next_ch(s, state, tmo, 0);

		/* a sensor of fewer lines: give the other's frame time */
		line0 = rd(R_FRONT_LINE(0));
		line1 = rd(R_FRONT_LINE(1));
		hi = (s32)line0 > (s32)line1 ? line0 : line1;
		lo = (s32)line0 > (s32)line1 ? line1 : line0;
		if ((s32)hi > (s32)(lo * 4)) {
			set_current_state(TASK_INTERRUPTIBLE);
			schedule_timeout(msecs_to_jiffies(20));
		} else if ((s32)hi > (s32)(lo * 2)) {
			set_current_state(TASK_INTERRUPTIBLE);
			schedule_timeout(msecs_to_jiffies(10));
		}

		local_irq_save(flags);
		VI(CUR_SNS) = s;
		for (c = 0; c < N_STORE; c++) {
			int i = s * 3 + c;
			u32 src = CH(i, CH_SRC_FPS), cnt;
			bool take = false;
			void *b;

			/* the scaler: horizontal 0xc8..0xe0, vertical 0xe4.. */
			IM(0xc60, c) = CH(i, 0xc8);
			IM(0xc6c, c) = CH(i, 0xcc);
			IM(0xc78, c) = CH(i, 0xd0);
			IM(0xc84, c) = CH(i, 0xd4);
			IM(0xc90, c) = CH(i, 0xd8);
			IM(0xc9c, c) = CH(i, 0xdc);
			IM(0xca8, c) = CH(i, 0xe0);
			IM(0xc18, c) = CH(i, 0xe4);
			IM(0xc24, c) = CH(i, 0xe8);
			IM(0xc30, c) = CH(i, 0xec);
			IM(0xc3c, c) = CH(i, 0xf0);
			IM(0xc48, c) = CH(i, 0xf4);
			IM(0xc54, c) = CH(i, 0xf8);
			IM(0xcb4, c) = CH(i, 0xfc);
			IM(0xcc0, c) = 0;
			IM(0xccc, c) = 0xffffffff;
			IM(0xc0c, c) = CH(i, 0x100);
			IM(0xcd8, c) = 0;
			IM(0xce4, c) = 0xffffffff;
			CH(i, CH_FRMTIME) = (u32)pts - CH(i, CH_PTS);
			CH(i, CH_PTS) = (u32)pts;
			CH(i, CH_PTS + 4) = pts >> 32;
			cnt = ++CH(i, CH_INT);
			if (src)
				take = frtc_table[CH(i, CH_DST_FPS) +
						  (src - 1) * 30 - 1] &
				       1u << (cnt % src);
			if (split()) {
				if (s == 0)
					g_vi_splice[c].buf = NULL;
				else
					take = true;
			}
			b = vi_next_buf(s, c, i, take);
			if (!b) {
				IM(0xd2c, c) = 0;
				CH(i, CH_NEXT) = 0;
				continue;
			}
			vi_set_store(s, c, i, b, &size, &csize);
			CH(i, CH_NEXT) = B(next[i], B_ADDR);
		}
		wr(R_VIC_END, 0);
		vi_update_regs(vi_info + IMG, reg_vi_base_va);
		wr(R_SEL_GO, 1);
		local_irq_restore(flags);
		isp_set_complete(s);
	}
	return 0;
}

static int vi_info_init(void)
{
	int i;

	if (rst == 1) {
		dsb();
		writel_relaxed(0x1590000, sys_base + VI_RESET);
		mdelay(2);
		udelay(500);
		dsb();
		writel_relaxed(1, sys_base + VI_RESET);
		mdelay(2);
		udelay(500);
	}
	memset(vi_info, 0, sizeof(vi_info));
	memset(g_line_vb, 0, sizeof(g_line_vb));
	memset(g_bg_info, 0, sizeof(g_bg_info));
	for (i = 0; i < N_CHN; i++) {
		INIT_LIST_HEAD(LIST(i));
		init_waitqueue_head(CHWQ(i));
	}
	init_waitqueue_head(STARTWQ);
	wr(R_SEL_EN, SEL_MASK);
	wr(R_VIC_CTRL1, 2);
	wr(R_VIC_CTRL0, 3);
	return 0;
}

/* the scaler's step for out of in: as vi_mu.ko, a reduced ratio scaled
 * to about 1024 */
static void vi_ratio(u32 in, u32 out, u32 *num, u32 *den)
{
	u32 a = in % out, g = out, t;

	while (a) {
		t = g % a;
		g = a;
		a = t;
	}
	*den = out / g;
	*num = in / g;
	if ((int)*num <= 0x400) {
		u32 k = 0x400 / *num;

		*num *= k;
		*den *= k;
		if (*num < 0x400) {
			*num *= 2;
			*den *= 2;
		}
	}
}

static int vi_set_chn_attr(const u32 *a)
{
	u32 c = a[1], i;
	u32 dst_w = a[9], dst_h = a[10], cap_w = a[6], cap_h = a[7];
	u32 stride = ALIGN(dst_w, 32), bufsz = stride * dst_h * 3 >> 1;
	u32 hden, hnum, hint, hfrac, hstep, hflag;
	u32 vden, vnum, vint, vfrac, vstep, vflag, vshift;
	unsigned long flags;

	if (c >= N_CHN)
		return -1;
	local_irq_save(flags);
	for (i = 0; i < 11; i++)
		CH(c, CH_CAP_X + i * 4) = a[4 + i];
	if (c == 0 && g_line_depth) {
		if (!*(u32 *)(g_line_vb + B_ADDR)) {
			u32 sz = g_line_depth * dst_w * 3 >> 1;
			u32 phys = 0, comp = 0;
			int ret;

			if (!memcomp)
				ret = mmz_alloc("vi_store", sz, &phys);
			else
				ret = mmz_alloc_comp("vi_store", sz, &phys,
						     &comp);
			if (ret) {
				local_irq_restore(flags);
				pr_err("VB: mmz malloc memory failed\n");
				return -1;
			}
			*(u32 *)(g_line_vb + B_ADDR) = memcomp ? comp : phys;
			*(u32 *)(g_line_vb + 0x18) = sz;
		}
		*(u32 *)(g_line_vb + B_W) = dst_w;
		*(u32 *)(g_line_vb + B_H) = dst_h;
		*(u32 *)(g_line_vb + B_LINES) = g_line_depth;
		*(u32 *)(g_line_vb + B_STRIDE) = stride;
	}
	if (!CH(c, CH_BUFSZ)) {
		CH(c, CH_BUFSZ) = bufsz;
	} else if (CH(c, CH_BUFSZ) < bufsz) {
		local_irq_restore(flags);
		pr_err("vi: too large size\n");
		return -1;
	}
	if (dst_w < cap_w) {
		if (cap_w >= dst_w << 4) {
			local_irq_restore(flags);
			pr_err("vi: not support destsize %d -> %d\n", cap_w,
			       dst_w);
			return -1;
		}
		vi_ratio(cap_w, dst_w, &hnum, &hden);
		hint = hnum / hden;
		hstep = ((hden >> 1) + 0x40000) / hden;
		hfrac = hnum - hden * hint;
		hflag = hden * hstep <= 0x40000;
	} else {
		hden = hnum = 0x400;
		hflag = hint = 1;
		hstep = 0x100;
		hfrac = 0;
	}
	if (dst_h < cap_h) {
		u32 d;

		if (cap_h >= dst_h << 4) {
			local_irq_restore(flags);
			pr_err("vi: not support destsize %d -> %d\n", cap_h,
			       dst_h);
			return -1;
		}
		if (cap_h < dst_h << 3) {
			vshift = cap_h >= dst_h << 2;
			d = vshift ? dst_h << 1 : dst_h;
		} else {
			d = dst_h << 2;
			vshift = 3;
		}
		vi_ratio(cap_h, d, &vnum, &vden);
		vint = vnum / vden;
		vstep = ((vden >> 1) + 0x40000) / vden;
		vfrac = vnum - vden * vint;
		vflag = vden * vstep <= 0x40000;
	} else {
		vflag = vint = 1;
		vnum = vden = 0x400;
		vfrac = vshift = 0;
		vstep = 0x100;
	}
	CH(c, 0xc8) = hden;
	CH(c, 0xcc) = hnum;
	CH(c, 0xd0) = hint;
	CH(c, 0xd4) = hfrac;
	CH(c, 0xd8) = hstep;
	CH(c, 0xdc) = hflag;
	CH(c, 0xe0) = 0;
	CH(c, 0xe4) = vden;
	CH(c, 0xe8) = vnum;
	CH(c, 0xec) = vint;
	CH(c, 0xf0) = vfrac;
	CH(c, 0xf4) = vstep;
	CH(c, 0xf8) = vflag;
	CH(c, 0xfc) = vshift;
	CH(c, 0x100) = dst_h < cap_h || dst_w < cap_w;
	local_irq_restore(flags);
	return 0;
}

/* a picture from the channel's queue, waiting up to ms */
static long vi_get_frame(u32 *a)
{
	u32 c = a[1];
	unsigned long flags;
	long t;
	struct list_head *e;
	void *b;

	if (split()) {
		if (c > 2)
			pr_err("vi: vi_get_frame mode %d chn %d err\n", mode,
			       c);
		c += (sensor_max - 1) * 3;
	}
	if (c >= N_CHN)
		return -1;
	t = msecs_to_jiffies(a[2]);
	if (!CH(c, CH_QLEN))
		t = wait_event_interruptible_timeout(*CHWQ(c), CH(c, CH_QLEN),
						     t);
	if (t == 0)
		return -512;
	local_irq_save(flags);
	if (!CH(c, CH_QLEN)) {
		local_irq_restore(flags);
		return -1;
	}
	e = LIST(c)->next;
	list_del(e);
	CH(c, CH_QLEN)--;
	b = e;
	if (CH(c, CH_BIND)) {
		vb_add_buf_reference(b);
		vi_put_buf(b);
	}
	given_add(b);
	local_irq_restore(flags);
	a[12] = B(b, B_PTS);
	a[13] = B(b, B_PTS + 4);
	a[7] = CH(c, CH_PIXFMT);
	a[4] = B(b, B_ADDR);
	a[5] = B(b, B_STRIDE) * B(b, B_H) + B(b, B_ADDR);
	a[6] = B(b, B_ADDR) + B(b, B_STRIDE) * B(b, B_H) * 2;
	a[9] = B(b, B_W);
	a[10] = B(b, B_H);
	a[8] = B(b, B_STRIDE);
	a[14] = (u32)b;
	return 0;
}

static long vi_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	void __user *up = (void __user *)arg;
	u32 a[20];
	unsigned long flags;
	long ret;
	u32 c;

	switch (cmd) {
	case 0x5600:
		vi_info_init();
		return 0;
	case 0x5632:
	case 0x5633:
		return 0;
	case 0x40045603:
	case 0x40045604:
		if (copy_from_user(&c, up, 4))
			goto efault;
		if (c >= N_CHN)
			return -1;
		local_irq_save(flags);
		CH(c, CH_ENABLE) = cmd == 0x40045603;
		local_irq_restore(flags);
		return 0;
	case 0x4a18560e:
		if (copy_from_user(bd_info, up, BG_SIZE))
			goto efault;
		c = *(u32 *)bd_info;
		if (c >= N_CHN)
			return -1;
		local_irq_save(flags);
		memcpy(g_bg_info[c], bd_info, BG_SIZE);
		local_irq_restore(flags);
		return 0;
	case 0x4a18560f:
		c = *(u32 *)bd_info; /* the channel last set, as vi_mu.ko */
		if (c >= N_CHN)
			return -1;
		local_irq_save(flags);
		memcpy(bd_info, g_bg_info[c], BG_SIZE);
		local_irq_restore(flags);
		if (copy_to_user(up, bd_info, BG_SIZE))
			goto efault_to;
		return 0;
	case 0x80245610: {
		u8 v[0x24];

		memset(v, 0, sizeof(v));
		*(u32 *)v = VI_VER;
		strlcpy(v + 4, "open " __DATE__, 16);
		strlcpy(v + 0x14, __TIME__, 16);
		if (copy_to_user(up, v, sizeof(v)))
			goto efault_to;
		return 0;
	}
	case 0x40505601:
	case 0x40505602:
	case 0x40505605:
	case 0x40505606:
	case 0x40505607:
	case 0x40505608:
		break;
	default:
		pr_err("vi_ioctl: cmd error\n");
		return 0;
	}
	if (copy_from_user(a, up, 0x50))
		goto efault;
	switch (cmd) {
	case 0x40505602:
		return vi_set_chn_attr(a);
	case 0x40505605:
		c = a[1];
		if (c >= N_CHN)
			return -1;
		if (CH(c, CH_DEPTH)) {
			pr_warn("vi: bind/unbind depth is %d will set depth 0\n",
				CH(c, CH_DEPTH));
			vi_set_frame_depth(c, 0);
		}
		local_irq_save(flags);
		CH(c, CH_BIND) = a[4];
		CH(c, CH_BIND + 4) = a[5];
		CH(c, CH_BIND + 8) = a[6];
		local_irq_restore(flags);
		return 0;
	case 0x40505606:
		if (a[1] >= N_CHN)
			return -1;
		return vi_set_frame_depth(a[1], a[4]);
	case 0x40505607:
		ret = vi_get_frame(a);
		if (copy_to_user(up, a, 0x50))
			goto efault_to;
		return ret;
	case 0x40505608:
		local_irq_save(flags);
		ret = given_take((void *)a[14]);
		local_irq_restore(flags);
		if (!ret) {
			pr_err("vi: release of a picture not given out\n");
			return -1;
		}
		vb_put_buf(0, (void *)a[14]);
		return 0;
	}
	return 0;
efault:
	pr_err("vi_ioctl: copy_from_user error.\n");
	return -1;
efault_to:
	pr_err("vi_ioctl: copy_to_user error.\n");
	return -1;
}

static int vi_pm_set_mode(int m)
{
	unsigned long flags;

	if (m == 0) {
		local_irq_save(flags);
		g_vi_save = rd(R_SEL_EN);
		wr(R_SEL_EN, 0);
		wr(R_SEL_RAW, 0);
		VI(STATE) = 0;
		local_irq_restore(flags);
	} else if (m == 1) {
		isp_set_next_ch(VI(CUR_SNS), 0xff, 0, 0);
		local_irq_save(flags);
		vi_update_regs(vi_info + IMG, reg_vi_base_va);
		wr(R_SEL_GO, 1);
		wr(R_VIC_CTRL1, 2);
		wr(R_VIC_CTRL0, 3);
		vic_timeout_num = 0x12;
		local_irq_restore(flags);
		isp_set_complete(VI(CUR_SNS));
	} else {
		local_irq_save(flags);
		wr(R_SEL_EN, g_vi_save);
		local_irq_restore(flags);
	}
	return 0;
}

static int vi_open(struct inode *inode, struct file *file)
{
	vi_dev_open++;
	return 0;
}

static int vi_release(struct inode *inode, struct file *file)
{
	vi_dev_open--;
	return 0;
}

static const char *const pixfmt_name[] = { "SP420", "UNK", "RGB888",
					   "RGB565" };

static int vi_seq_show(struct seq_file *m, void *v)
{
	int n = sensor_max * 3, i, k;

	seq_printf(m, "[VI] VER:[%08x]  BUILD TIME:[%s %s]\n\n", VI_VER,
		   __DATE__, __TIME__);
	seq_puts(m, "--------------------------VI DEV ATTR---------------------------------------------------------\n");
	seq_printf(m, "Mode:%d  Sensor:%d\n", mode, sensor_max);
	seq_puts(m, "--------------------------VI PHYCHN ATTR------------------------------------------------------\n");
	seq_puts(m, "PhyChn  CapX  CapY  CapW  CapH  DstW  DstH  Mirror  Flip  IntEn  StEn  PixFom  SrcRat  DstRat\n");
	for (i = 0; i < n && i < N_CHN; i++) {
		seq_printf(m, "%-6d  %-4d  %-4d  %-4d  %-4d  %-4d  %-4d  ", i,
			   CH(i, CH_CAP_X), CH(i, CH_CAP_Y), CH(i, CH_CAP_W),
			   CH(i, CH_CAP_H), CH(i, CH_DST_W), CH(i, CH_DST_H));
		seq_printf(m, "%-6s  %-4s  %-5s  %-4s  %-6s  %-6d  %-6d\n",
			   CH(i, CH_MIRROR) ? "Y" : "N",
			   CH(i, CH_FLIP) ? "Y" : "N", "Y",
			   CH(i, CH_ENABLE) ? "Y" : "N",
			   pixfmt_name[CH(i, CH_PIXFMT) & 3],
			   CH(i, CH_SRC_FPS), CH(i, CH_DST_FPS));
	}
	seq_puts(m, "--------------------------VI CHN STATUS-------------------------------------------------------\n");
	seq_puts(m, "ViChn  FrmTime  FrmRate  VBSucc    VBFail    VBDup     IntCnt    SendCnt   LastAddr  NowAddr   NextAddr\n");
	for (i = 0; i < n && i < N_CHN; i++) {
		u32 ft = CH(i, CH_FRMTIME);

		seq_printf(m, "%-5d  %-7d  %-7d  %-8d  %-8d  %-8d  %-8d  %-8d  %-8x  %-8x  %-8x\n",
			   i, ft, ft ? ((ft >> 1) + 1000000) / ft : 0,
			   CH(i, CH_VB_OK), CH(i, CH_VB_FAIL),
			   CH(i, CH_VB_DUP), CH(i, CH_INT), CH(i, CH_SENT),
			   CH(i, CH_LAST), CH(i, CH_NOW), CH(i, CH_NEXT));
	}
	seq_puts(m, "--------------------------VI SPLICE STATUS-------------------------------------------------------\n");
	seq_puts(m, "Id  VB        State\n");
	for (i = 0; i < N_STORE; i++)
		seq_printf(m, "%-2d  %-8p  %-5x\n", i, g_vi_splice[i].buf,
			   g_vi_splice[i].buf ?
				   B(g_vi_splice[i].buf, B_SPLICE) : 0);
	seq_puts(m, "--------------------------VI OSD BACKGROUND----------------------------------------------------\n");
	seq_puts(m, "ViChn  En  Id    X      Y      W      H      Lum\n");
	for (i = 0; i < n && i < N_CHN; i++) {
		u8 *bg = g_bg_info[i];
		u32 cnt = *(u32 *)(bg + 8);

		for (k = 0; k < cnt && k < 0x100; k++)
			seq_printf(m, "%-5d  %-2s  %-3d  %-4d  %-4d  %-4d  %-4d  %-3x\n",
				   i, *(u32 *)(bg + 4) ? "Y" : "N", k,
				   *(u16 *)(bg + 0x18 + k * 2),
				   *(u16 *)(bg + 0x218 + k * 2),
				   *(u16 *)(bg + 0x418 + k * 2),
				   *(u16 *)(bg + 0x618 + k * 2),
				   *(u16 *)(bg + 0x818 + k * 2));
	}
	seq_puts(m, "--------------------------VI INT INFO----------------------------------------------------\n");
	seq_printf(m, "switch timeout : %d\n", VI(SW_TIMEOUT));
	for (i = 0; i < 0x20 && i < 26; i++)
		if (VI(INT_CNT + i * 4))
			seq_printf(m, "%-2d  %-8d  \n", i, VI(INT_CNT + i * 4));
	return 0;
}

static void *vi_seq_start(struct seq_file *m, loff_t *pos)
{
	return *pos ? NULL : (void *)1;
}

static void *vi_seq_next(struct seq_file *m, void *v, loff_t *pos)
{
	++*pos;
	return NULL;
}

static void vi_seq_stop(struct seq_file *m, void *v)
{
}

static const struct seq_operations vi_seq_ops = {
	.start = vi_seq_start,
	.next = vi_seq_next,
	.stop = vi_seq_stop,
	.show = vi_seq_show,
};

static int vi_proc_open(struct inode *inode, struct file *file)
{
	return seq_open(file, &vi_seq_ops);
}

static const struct file_operations vi_proc_ops = {
	.owner = THIS_MODULE,
	.open = vi_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = seq_release,
};

static const struct file_operations xm_vi_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = vi_ioctl,
	.open = vi_open,
	.release = vi_release,
};

static struct miscdevice xm_vi_dev = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "vi",
	.fops = &xm_vi_fops,
};

static int __init xm_vi_init(void)
{
	struct isp_ops ops = { .pm_set_mode = vi_pm_set_mode };
	int ret;

	pr_info("vi: g_line_depth:%d mode:%d sensor_max:%d\n", g_line_depth,
		mode, sensor_max);
	if (sensor_max < 1 || sensor_max > 4) {
		pr_err("vi: sensor_max %d not supported\n", sensor_max);
		return -EINVAL;
	}
	reg_vi_base_va = ioremap(VI_BASE, VI_SIZE);
	sys_base = ioremap(SYS_BASE, 0x1000);
	if (!reg_vi_base_va || !sys_base) {
		pr_err("VI: ioremap video in base failed!\n");
		ret = -ENOMEM;
		goto unmap;
	}
	vi_info_init();
	proc_create_data("umap/vi", 0644, NULL, &vi_proc_ops, NULL);
	ret = request_irq(VI_SEL_IRQ, vi_sel_int, 0, "vi_sel", NULL);
	if (ret) {
		pr_err("VI: VI_SEL_IRQ request_irq  failed!\n");
		ret = -EIO;
		goto proc;
	}
	ptask = kthread_create(vi_thread, NULL, "vi");
	if (!IS_ERR_OR_NULL(ptask)) {
		kthread_set_prio(ptask, 99);
		wake_up_process(ptask);
	} else {
		ptask = NULL;
	}
	isp_register_ops(ops);
	ret = misc_register(&xm_vi_dev);
	if (ret) {
		pr_err("VI: register vi device failed!\n");
		ret = -EBUSY;
		goto thread;
	}
	return 0;
thread:
	if (ptask)
		kthread_stop(ptask);
	free_irq(VI_SEL_IRQ, NULL);
proc:
	remove_proc_entry("umap/vi", NULL);
unmap:
	if (sys_base)
		iounmap(sys_base);
	if (reg_vi_base_va)
		iounmap(reg_vi_base_va);
	return ret;
}

static void __exit xm_vi_exit(void)
{
	if (ptask)
		kthread_stop(ptask);
	ptask = NULL;
	misc_deregister(&xm_vi_dev);
	free_irq(VI_SEL_IRQ, NULL);
	iounmap(sys_base);
	iounmap(reg_vi_base_va);
	remove_proc_entry("umap/vi", NULL);
}

module_init(xm_vi_init);
module_exit(xm_vi_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("XM232 video input (vi_mu.ko drop-in)");
