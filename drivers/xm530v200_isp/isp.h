/* SPDX-License-Identifier: GPL-2.0 */
/*
 * The XM530V200's isp.ko, shared state: g_comm_para (0x6380 bytes) at
 * isp.ko's offsets, named where their use is known. Up to three channels
 * (two VICUs); each has a BPU group of 0x12d4 bytes at 0x2930.
 */
#ifndef XM530V200_ISP_H
#define XM530V200_ISP_H

#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/seq_file.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/wait.h>

#define COMM_SIZE 0x6380
#define N_CHN 3
extern u8 isp_comm[COMM_SIZE];

#define C8(o) (*(u8 *)(isp_comm + (o)))
#define CS8(o) (*(s8 *)(isp_comm + (o)))
#define C16(o) (*(u16 *)(isp_comm + (o)))
#define CS16(o) (*(s16 *)(isp_comm + (o)))
#define C32(o) (*(u32 *)(isp_comm + (o)))
#define CP(o) ((void *)(isp_comm + (o)))
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
#define CM_DUMP_ADDR 0x004c
#define CM_DUMP_SIZE 0x0050
#define CM_INIT 0x0060      /* 0x4962's 14 words */
#define CM_MODE 0x0068      /* u8: BPU 0 online, 1 offline, 2 HDR */
#define CM_CHNNUM 0x0069    /* u8 */
#define CM_VPSS_MASK 0x006a /* u8: the channels with a VPSS side */
#define CM_FPS_MASK 0x006d  /* u8 */
#define CM_MEMSAVE 0x006e   /* u8: 1 the BPU's output in VB buffers */
#define CM_DVP 0x006f       /* u8 */
#define CM_OPT 0x0071       /* s8: bit 7 no defaults; bits 0-1 the BPU's buffering */
#define CM_CHN 0x0098       /* 0x28 a channel: crop, size (bpu_chn_init) */
#define CM_VICU 0x0110      /* 0xd0 a channel: the VICU's state */
#define CM_BPUSTAT 0x0380   /* 0x58 a channel */
#define CM_AWB_LOCK(c) (0x04e0 + (c) * 4)
#define CM_LOCK_DVP 0x04ec
#define CM_LOCK_BPU 0x04f0
#define CM_AWB_STAT 0x04f8  /* 3 x 0xc00 */
#define CM_DVP_WQ 0x28fc
#define CM_VSYNC_WQ 0x2908
#define CM_BPU_WQ 0x2914
#define CM_VSYNC_CNT 0x2920 /* the channels with a new frame (0x490a) */
#define CM_DVP_STATE 0x2924
#define CM_BPU_GO 0x2928    /* the channels for the BPU thread */
#define CM_BPU_RUN 0x292c   /* the channels the BPU works on */
#define CM_GRP(c) (0x2930 + (c) * 0x12d4)
#define GRP_BLOCK 0x74      /* the parameter blocks */
#define GRP_SNAP 0x9a4      /* their copy for the frame */
#define CM_IW_WAIT 0x61ac
#define CM_VPSS_HOLD 0x61b0
#define CM_RUN 0x61b4
#define CM_VER 0x61b8
#define CM_FLAGS 0x61e0     /* u8: limit-up/crop changes */
#define CM_LIMITUP 0x61e4
#define CM_CUT(c) (0x61f4 + (c) * 0x14)
#define CM_REPAIR 0x6230
#define CM_PRODUCT 0x6234
#define CM_VB_HOLD(c) (0x6238 + (c) * 4)
#define CM_CHN_ATTR(c) (0x62e8 + (c) * 0x1c)
#define CM_DVPSEL 0x633c    /* 0x4912's 7 words */
#define CM_BPU_PEND 0x636c
#define CM_BPU_CUR 0x6370
#define CM_BPU_LOCK2 0x6374
#define CM_VB_OUT 0x6378    /* the VB buffer the BPU writes */

int isp_write_reg(u32 addr, u32 val);
u32 isp_read_reg(u32 addr);
int pll_write_reg(u32 addr, u32 val);
u32 pll_read_reg(u32 addr);
int isp_users_open(void);
void clk_get_pts(u64 *pts);

extern spinlock_t awb_lock[N_CHN];  /* CM_AWB_LOCK: the frame interrupts */
extern spinlock_t dvp_lock;         /* CM_LOCK_DVP */
extern spinlock_t bpu_irq_lock;     /* CM_LOCK_BPU */
extern spinlock_t bpu_lock2;        /* CM_BPU_LOCK2 */
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
int bpu_af_attr_set(void __user *arg);
int bpu_af_stat_get(void __user *arg);
int bpu_post_deal(u32 chn, int mode, int wdr);
int bpu_set_wdr_mode(u32 chn, u32 mode);
int bpu_set_enable(u32 chn, int en);
int bpu_offline_deal(u32 chn, u32 cur);
irqreturn_t bpu_irq(int irq, void *dev);
int bpu_thread(void *unused);
int bpu_set_cut(u32 chn, u32 en, u32 x, u32 y, u32 w, u32 h);
int bpu_get_cut(u32 chn, u32 *en, u32 *x, u32 *y, u32 *w, u32 *h);

/* ib.c */
int ib_init(const u32 *pools, u32 w, u32 h, u8 mode);
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
irqreturn_t vicu_irq_0(int irq, void *dev);
irqreturn_t vicu_irq_1(int irq, void *dev);

/* vpss_isp.c */
void vpss_isp_reset(int grp);
int vpss_isp_set_val(u32 grp, u32 w, u32 h);
int vpss_isp_post_deal(u32 grp, u32 w, u32 h);
int iw_vpss_ioctl(unsigned int cmd, void __user *arg);
void vpss_isp_proc_show(struct seq_file *m, int *n);
void vpss_isp_exit(void);

/* vb.ko */
int vb_get_buf(u32 list, u32 size, void **buf, int ms);
int vb_put_buf(u32 list, void *buf);

#endif
