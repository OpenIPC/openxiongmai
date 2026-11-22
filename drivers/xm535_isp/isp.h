/* SPDX-License-Identifier: GPL-2.0 */
/*
 * The XM535D1/D2's isp.ko, shared state: g_comm_para (0x6198 bytes) at
 * isp.ko's offsets, named where their use is known. Up to three channels
 * on two VICUs (the first time-shared between two sensors when there are
 * three); each has a BPU group of 0xf40 bytes at 0x2808 and a VICU block
 * of 0x310 at 0x55c8.
 */
#ifndef XM535_ISP_H
#define XM535_ISP_H

#include <linux/interrupt.h>
#include <linux/irqflags.h>
#include <linux/mutex.h>
#include <linux/seq_file.h>
#include <linux/types.h>
#include <linux/wait.h>

#define COMM_SIZE 0x6198
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
#define CM_VICU_HELD 0x0001 /* u8: the waited-for parameters set (CM_IW_WAIT) */
#define CM_TS_CUR 0x0002    /* u8: the time-shared VICU's sensor (0, 1) */
#define CM_DUMP_CHN 0x003c  /* u8 */
#define CM_DUMP 0x003d      /* u8: 1 dump; 0x10 done, 0x20 failed */
#define CM_DUMP_W 0x003e
#define CM_DUMP_H 0x0040
#define CM_DUMP_ADDR 0x004c
#define CM_DUMP_SIZE 0x0050
#define CM_INIT 0x0060      /* 0x4962's 14 words */
#define CM_MODE 0x0068      /* u8: BPU 0 online, 1 offline */
#define CM_CHNNUM 0x0069    /* u8 */
#define CM_VPSS_MASK 0x006a /* u8: the channels with a VPSS side */
#define CM_VICU_MODE 0x006b /* u8: 2 the first VICU time-shared */
#define CM_NSNS 0x006c      /* u8: the sensors; 3 time-shares the first VICU */
#define CM_FPS_MASK 0x006d  /* u8 */
#define CM_MEMSAVE 0x006e   /* u8: 0 the BPU's output straight to vpss.ko */
#define CM_OPT 0x0071       /* s8: bit 7 no defaults, bit 6 BPU reset a frame,
			     * bits 0-1 the BPU's buffering */
#define CM_TS_LINES 0x0074  /* u16 x 2: the time-shared sensors' lines */
#define CM_CHN 0x0098       /* 0x28 a channel: crop, size (bpu_chn_init) */
#define CM_VICU(c) (0x0110 + (c) * 0xb0) /* the VICU's state */
#define CM_BPUSTAT(c) (0x0320 + (c) * 0x30)
#define CM_BPUALL 0x03b0
#define CM_AWB_STAT 0x03e0  /* 3 x 0xc00 */
#define CM_VSYNC_CNT 0x27fc /* the channels with a new frame */
#define CM_BPU_GO 0x2800    /* the channels for the BPU thread */
#define CM_BPU_RUN 0x2804   /* the channels the BPU works on; bit 31 busy */
#define CM_GRP(c) (0x2808 + (c) * 0xf40)
#define GRP_BLOCK 0x68      /* the parameter blocks */
#define GRP_SNAP 0x7d4      /* their copy for the frame */
#define GRP_BLOCK_SIZE 0x76c
#define CM_VB(c) (0x55c8 + (c) * 0x310) /* the VICU's parameter blocks */
#define CM_IW_WAIT 0x5ef8
#define CM_VPSS_HOLD 0x5efc
#define CM_RUN 0x5f00
#define CM_VER 0x5f04
#define CM_PRODUCT 0x5f40
#define CM_CHN_ATTR(c) (0x5f68 + (c) * 0x1c)
#define CM_TIME(c) (0x5fc0 + (c) * 0x80) /* a channel's frame times */
#define CM_PEND 0x6140      /* u8 x 5: the channels stored, for the BPU */
#define CM_PEND_N 0x6145
#define CM_PEND_TAIL 0x6146
#define CM_PEND_HEAD 0x6147
#define CM_PEND_LOST 0x6148 /* u16 */
#define CM_LINES 0x614c     /* the frame interrupts' line counts */

/*
 * The XM535 is uniprocessor and isp.ko only masks interrupts around its
 * shared state; so does this driver.
 */
#define ISP_LOCK(f) local_irq_save(f)
#define ISP_UNLOCK(f) local_irq_restore(f)

int isp_write_reg(u32 addr, u32 val);
u32 isp_read_reg(u32 addr);
int pll_write_reg(u32 addr, u32 val);
u32 pll_read_reg(u32 addr);
int timer_write_reg(u32 addr, u32 val);
u32 timer_read_reg(u32 addr);
int isp_users_open(void);
void clk_get_pts(u64 *pts);

extern struct mutex grp_lock[N_CHN];
extern struct mutex iwctrl_lock;
extern struct mutex iwctrl_vpss_lock;
extern wait_queue_head_t dump_wq, vsync_wq, bpu_thread_wq;
extern int isp_vpss_grp;
extern struct tasklet_struct isp_irq0_tasklet, isp_irq1_tasklet,
	isp_bpu_tasklet;

/* bpu.c */
void bpu_reset(void);
int bpu_init(void);
int bpu_chn_init(const u8 *attr);
int awb_stat_get(int wdr, u32 chn);
int bpu_post_deal(u32 chn, int mode);
int bpu_set_enable(u32 chn, int en);
int bpu_offline_deal(u32 chn);
void bpu_offline_deal_in(unsigned long unused);
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
int iw_cmos_if(u32 chn, const u32 *w);
int iw_ioctl(unsigned int cmd, void __user *arg);

/* vicu.c */
int vicu_chn_init(const u8 *attr);
int vicu_chn_enable(u32 chn, int en);
int vicu_run(u32 chn);
void vicu_reset(void);
void vicu_cmos_if(u32 chn, const void *w);
void vicu_frame(unsigned long vicu);
void vicu_stored(unsigned long vicu);
irqreturn_t vicu_irq_0(int irq, void *dev);
irqreturn_t vicu_irq_1(int irq, void *dev);
irqreturn_t timer45_interrupt(int irq, void *dev);

/* vpss_isp.c */
void vpss_isp_reset(int grp);
int vpss_isp_set_val_v2(u32 grp, u32 w, u32 h, int synced);
int vpss_isp_sync_para(u32 grp);
int iw_vpss_ioctl(unsigned int cmd, void __user *arg);
void vpss_isp_exit(void);

/* sys.ko */
int sys_set_clk(u32 reg, u32 val);
int sys_get_clk(u32 reg, u32 *val);

/* vpss.ko */
int vpss_thread_set_val(u32 grp, u32 src, u32 w, u32 h, u32 pts_lo,
			u32 pts_hi);
int vpss_thread_post_deal(u32 grp, int ok);
int vpss_thread_middle_deal(u32 grp, int ok);

#endif
