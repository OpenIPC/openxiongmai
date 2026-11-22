/* SPDX-License-Identifier: GPL-2.0 */
/*
 * The XM535W1's isp.ko, shared state: g_comm_para (0x60b8 bytes) at
 * isp.ko's offsets, named where their use is known. It is the XM530V200's
 * isp.ko (its front part 4 bytes lower) around the XM535D1/D2's BPU and
 * VPSS side: up to three channels on two VICUs (the second time-shared,
 * 0x4912); each has a BPU group of 0xf40 bytes at 0x290c and a VICU block
 * of 0x2f4 at 0x56c8.
 */
#ifndef XM535W1_ISP_H
#define XM535W1_ISP_H

#include <linux/interrupt.h>
#include <linux/irqflags.h>
#include <linux/mutex.h>
#include <linux/seq_file.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/wait.h>

#define COMM_SIZE 0x60b8
#define N_CHN 3
extern u8 isp_comm[COMM_SIZE];

#define C8(o) (*(u8 *)(isp_comm + (o)))
#define CS8(o) (*(s8 *)(isp_comm + (o)))
#define C16(o) (*(u16 *)(isp_comm + (o)))
#define CS16(o) (*(s16 *)(isp_comm + (o)))
#define C32(o) (*(u32 *)(isp_comm + (o)))
#define CP(o) ((void *)(isp_comm + (o)))
#define PTS(o) (*(u64 *)CP(o))
/* a word of a block at p */
#define W32(p, o) (*(u32 *)((u8 *)(p) + (o)))
#define W16(p, o) (*(u16 *)((u8 *)(p) + (o)))
#define WS16(p, o) (*(s16 *)((u8 *)(p) + (o)))
#define W8(p, o) (*(u8 *)((u8 *)(p) + (o)))

/* g_comm_para */
#define CM_BPU_BUSY 0x0000  /* u8: the BPU thread running */
#define CM_DUMP_CHN 0x0038  /* u8 */
#define CM_DUMP 0x0039      /* u8: 1 dump; 0x10 done, 0x20 failed */
#define CM_DUMP_W 0x003a
#define CM_DUMP_H 0x003c
#define CM_DUMP_ADDR 0x0048
#define CM_DUMP_SIZE 0x004c
#define CM_INIT 0x005c      /* 0x4962's 14 words */
#define CM_MODE 0x0064      /* u8: BPU 0 online, 1 offline, 2 HDR */
#define CM_CHNNUM 0x0065    /* u8 */
#define CM_VPSS_MASK 0x0066 /* u8: the channels with a VPSS side */
#define CM_FPS_MASK 0x0069  /* u8 */
#define CM_MEMSAVE 0x006a   /* u8: 1 the BPU's output in VB buffers, 0 vi.ko's */
#define CM_DVP 0x006b       /* u8 */
#define CM_OPT 0x006d       /* s8: bit 7 no defaults; bits 0-1 the BPU's buffering */
#define CM_CHN 0x0094       /* 0x28 a channel: crop, size (bpu_chn_init) */
#define CM_VICU 0x0110      /* 0xd0 a channel: the VICU's state */
#define CM_BPUSTAT 0x0380   /* 0x58 a channel */
#define CM_BPUALL 0x0488
#define CM_AWB_STAT 0x04e0  /* 3 x 0xc00 */
#define CM_VSYNC_CNT 0x28fc /* the channels with a new frame (0x490a) */
#define CM_DVP_STATE 0x2900
#define CM_BPU_GO 0x2904    /* the channels for the BPU thread */
#define CM_BPU_RUN 0x2908   /* the channels the BPU works on */
#define CM_GRP(c) (0x290c + (c) * 0xf40)
#define GRP_BLOCK 0x68      /* the parameter blocks */
#define GRP_SNAP 0x7d4      /* their copy for the frame */
#define GRP_BLOCK_SIZE 0x76c
#define CM_VB(c) (0x56c8 + (c) * 0x2f4) /* the VICU's parameter blocks */
#define CM_IW_WAIT 0x5fa8
#define CM_VPSS_HOLD 0x5fac
#define CM_RUN 0x5fb0
#define CM_VER 0x5fb4
#define CM_FLAGS 0x5fdc     /* u8 */
#define CM_REPAIR 0x5ff0
#define CM_PRODUCT 0x5ff4
#define CM_VB_HOLD(c) (0x5ff8 + (c) * 4)
#define CM_CHN_ATTR(c) (0x6028 + (c) * 0x1c)
#define CM_DVPSEL 0x607c    /* 0x4912's 7 words */
#define CM_BPU_PEND 0x60ac
#define CM_BPU_CUR 0x60b0
#define CM_VB_OUT 0x60b4    /* the VB buffer the BPU writes */

int isp_write_reg(u32 addr, u32 val);
u32 isp_read_reg(u32 addr);
int pll_write_reg(u32 addr, u32 val);
u32 pll_read_reg(u32 addr);
int isp_users_open(void);
void clk_get_pts(u64 *pts);

extern spinlock_t awb_lock[N_CHN];  /* the frame interrupts */
extern spinlock_t dvp_lock;
extern spinlock_t bpu_irq_lock;
extern spinlock_t bpu_lock2;
extern struct mutex grp_lock[N_CHN];
extern struct mutex iwctrl_lock;
extern struct mutex iwctrl_vpss_lock;
extern struct mutex vpss_limitup_lock;
extern wait_queue_head_t dump_wq, vsync_wq, dvp_wq, bpu_thread_wq;
extern int isp_vpss_grp;

/* bpu.c */
void bpu_reset(void);
int bpu_init(void);
int bpu_chn_init(const u8 *attr);
int awb_stat_get(int wdr, u32 chn);
int bpu_post_deal(u32 chn, int mode);
int bpu_set_wdr_mode(u32 chn, u32 mode);
int bpu_set_enable(u32 chn, int en);
int bpu_offline_deal(u32 chn, u32 cur);
irqreturn_t bpu_irq(int irq, void *dev);
int bpu_thread(void *unused);

/* ib.c */
int ib_init(const u32 *pools, u32 w, u32 h, u8 mode);
void ib_exit(void);
void ib_proc_show(struct seq_file *m, int *n);
int ib_get_buf(u32 chn, int type, u32 size, void **out, int ms);
int ib_put_buf(u32 chn, int type, void *buf);

/* iw.c */
void iw_isp_init(void);
int iw_ioctl(unsigned int cmd, void __user *arg);

/* vicu.c */
int vicu_chn_init(const u8 *attr);
int vicu_run(u32 chn);
void vicu_reset(void);
void vicu_frame(unsigned long st);
void vicu_kill(void);
irqreturn_t vicu_irq_0(int irq, void *dev);
irqreturn_t vicu_irq_1(int irq, void *dev);

/* vpss_isp.c */
void vpss_isp_reset(int grp);
int vpss_isp_set_val_v2(u32 grp, u32 w, u32 h, int synced);
int vpss_isp_sync_para(u32 grp);
int iw_vpss_ioctl(unsigned int cmd, void __user *arg);
void vpss_isp_exit(void);

/* vb.ko, sys.ko */
int vb_get_buf(u32 list, u32 size, void **buf, int ms);
int vb_put_buf(u32 list, void *buf);
int sys_set_clk(u32 reg, u32 val);
int sys_get_clk(u32 reg, u32 *val);

/* vi.ko */
int vpss_thread_set_val(u32 grp, u32 src, u32 w, u32 h, u32 pts_lo,
			u32 pts_hi);
int vpss_thread_post_deal(u32 grp, int ok);
int vpss_thread_middle_deal(u32 grp, int ok);

#endif
