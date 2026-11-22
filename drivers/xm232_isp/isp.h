/* SPDX-License-Identifier: GPL-2.0 */
/*
 * The XM232's isp.ko ("XM210V200_ISP"), shared state. g_comm_para (0x44c0
 * bytes) is kept at isp.ko's offsets, named where their use is known; the
 * kernel objects isp.ko embeds in it (mutexes, wait queues) are kept apart,
 * the space they take left unused. Up to three channels: each has a BPU
 * group of 0x8b0 bytes at 0x2840, its register block at +0x38.
 */
#ifndef XM232_ISP_H
#define XM232_ISP_H

#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irqflags.h>
#include <linux/mutex.h>
#include <linux/seq_file.h>
#include <linux/types.h>
#include <linux/wait.h>

#define COMM_SIZE 0x44c0
#define N_CHN 3
extern u8 isp_comm[COMM_SIZE];

#define C8(o) (*(u8 *)(isp_comm + (o)))
#define CS8(o) (*(s8 *)(isp_comm + (o)))
#define C16(o) (*(u16 *)(isp_comm + (o)))
#define C32(o) (*(u32 *)(isp_comm + (o)))
#define CS32(o) (*(s32 *)(isp_comm + (o)))
#define CP(o) ((void *)(isp_comm + (o)))
/* a word of a block at p */
#define P32(p, o) (*(u32 *)((u8 *)(p) + (o)))
#define P16(p, o) (*(u16 *)((u8 *)(p) + (o)))
#define PS16(p, o) (*(s16 *)((u8 *)(p) + (o)))
#define P8(p, o) (*(u8 *)((u8 *)(p) + (o)))

/* g_comm_para */
#define CM_BPU_RUNNING 0x0000 /* u8: the BPU thread running */
#define CM_MD(c) (0x0018 + (c) * 0x14) /* the MD statistics' buffers */
#define CM_AWB(c) (0x0054 + (c) * 0x14) /* the AWB statistics' buffers */
#define ST_W 0x00 /* u8: the buffer the BPU writes */
#define ST_R 0x01 /* u8: the one read */
#define ST_PHYS 0x04 /* u32 x 2 */
#define CM_DUMP_CHN 0x0090 /* u8 */
#define CM_DUMP 0x0091 /* u8: 1 dump; 0x10 done, 0x20 failed */
#define CM_DUMP_W 0x0092
#define CM_DUMP_H 0x0094
#define CM_DUMP_ADDR 0x00a0
#define CM_INIT 0x00b4 /* 0x4962's 11 words */
#define CM_BPU_MODE 0x00bc /* u8: 0 online, 1 online with the mux, 3 offline */
#define CM_VPSS_MODE 0x00bd /* u8: 0 online */
#define CM_CHNNUM 0x00be /* u8 */
#define CM_CHN_MASK 0x00bf /* u8 */
#define CM_OPT 0x00c0 /* u8; c1 bits 0-1 the offline buffering */
#define CM_POOLS 0x00c4 /* the ISP buffer pools: count, then size, count */
#define CM_CHN(c) (0x00e0 + (c) * 0x28) /* crops and sizes (bpu_chn_init) */
#define CM_VICU(c) (0x0158 + (c) * 0xb0) /* the VICU channel's state */
#define CM_BPUSTAT(c) (0x0368 + (c) * 0x28)
#define CM_BPUALL 0x03e4
#define CM_AWB_STAT(c) (0x0408 + (c) * 0xc00) /* what read() returns */
#define CM_VSYNC(c) (0x282c + (c) * 4) /* a new frame on the channel */
#define CM_BPU_GO 0x2838 /* the channels for the BPU thread */
#define CM_BPU_RUN 0x283c /* the channels the BPU works on; bit 31 busy */
#define CM_GRP(c) (0x2840 + (c) * 0x8b0)
#define GRP_ON 0x00
#define GRP_ON2 0x04
#define GRP_SNS 0x08
#define GRP_BIND 0x0c /* 0x4961's three words: -, the VPSS group, - */
#define GRP_VPSS 0x10
#define GRP_CNT 0x28
#define GRP_W 0x30
#define GRP_H 0x34
#define GRP_BLK 0x38 /* the register block */
#define CM_BLK(c) (CM_GRP(c) + GRP_BLK)
#define CM_BPU_OFF 0x4250 /* 0x4960: the channels holding their registers */
#define CM_VPSS_OFF 0x4254
#define CM_RUN 0x425c /* the channels running (0x4959) */
#define CM_STATE 0x4260 /* 0x10 running, 0x11 suspended */
#define CM_BPU_DEALING 0x4264
#define CM_VER 0x4280 /* 0x495d */
#define CM_SNS_SEL 0x42a8 /* 0x4912's 7 words */
#define CM_SNS_GPIO 0x42c4 /* u32 x 2 */
#define CM_CH_CUR 0x42cc /* the channels: next, now, before */
#define CM_CH_NOW 0x42d0
#define CM_CH_PREV 0x42d4
#define CM_TIME(c) (0x42d8 + (c) * 0x80)
#define CM_PEND 0x4458 /* u8 x 5: the channels stored, for the BPU */
#define CM_PEND_N 0x445d
#define CM_PEND_TAIL 0x445e
#define CM_PEND_HEAD 0x445f
#define CM_PEND_LOST 0x4460 /* u16 */
#define CM_MUX(c) (0x4464 + (c) * 4) /* the mux timer's frame starts */
#define CM_PW_VAL 0x44ac
#define CM_PW_EV 0x44b0
#define CM_PW_MASK 0x44b4
#define CM_EXT 0x44b8 /* u8 x 3: the time-shared VICU's sensors */
#define CM_FCNT_CH 0x44bb /* u8 */
#define CM_FCNT_SNS 0x44bc /* u8 */
#define CM_FCNT_DUAL 0x44bd /* u8 */

#define ISP_LOCK(f) local_irq_save(f)
#define ISP_UNLOCK(f) local_irq_restore(f)

extern void __iomem *isp_base;
/* isp.ko's register access: a barrier, then the access */
static inline void wr(u32 off, u32 val)
{
	dsb();
	writel_relaxed(val, isp_base + off);
}

static inline u32 rd(u32 off)
{
	dsb();
	return readl_relaxed(isp_base + off);
}

struct isp_ops {
	int (*pm_set_mode)(int mode);
	void (*bpu_done)(u32 grp);
	void (*vpss_start)(u32 grp, int on);
	void (*vpss_set)(u32 grp, u32 stride, u32 w, u32 h, u32 pts);
};
extern struct isp_ops isp_ops;

extern struct mutex grp_lock[N_CHN];
extern struct mutex iwctrl_lock;
extern struct mutex iwctrl_vpss_lock;
extern struct mutex sns_pw_lock;
extern wait_queue_head_t dump_wq, bpu_thread_wq, sns_pw_wq;
extern wait_queue_head_t vicu_wq[N_CHN], mux_wq[N_CHN];
extern struct tasklet_struct isp_bpu_tasklet;
extern void __iomem *stat_va[2][N_CHN][2]; /* AWB, MD: the two buffers */
extern u32 *isp_regs_buf;
extern u32 isp_ldc_buf;

void clk_get_pts(u64 *pts);
int sys_set_clk(u32 reg, u32 val);
int sys_get_clk(u32 reg, u32 *val);
int mmz_alloc(const char *name, u32 size, u32 *phys);

/* isp.c */
int isp_deal_frame_cnt(u32 chn);
int isp_suspend(u32 cmd);

/* bpu.c */
void bpu_init_queue(void);
void bpu_reset(void);
int bpu_init(const u8 *init);
int bpu_chn_init(const u8 *attr);
int awb_stat_get(u32 chn);
int bpu_post_deal(u32 chn);
int bpu_deal(u32 chn);
int bpu_offline_deal(u32 chn);
void bpu_offline_deal_in(unsigned long unused);
irqreturn_t bpu_irq(int irq, void *dev);
int bpu_thread(void *unused);

/* ib.c */
int ib_init(const u32 *pools, u32 w, u32 h);
void ib_proc_show(struct seq_file *m, int *n);
int ib_get_buf(u32 chn, int type, u32 size, void **out, int ms);
int ib_put_buf(u32 chn, int type, void *buf);

/* iw.c */
void iw_isp_init(void);
int iw_ioctl(unsigned int cmd, void __user *arg);

/* vicu.c */
void vicu_init_queue(void);
int vicu_chn_init(const u8 *attr);
void vicu_reset(void);
irqreturn_t vicu_irq(int irq, void *dev);
irqreturn_t vicu_irq_0(int irq, void *dev);
irqreturn_t vicu_irq_1(int irq, void *dev);

/* vpss_isp.c */
int vpss_isp_reset(void);
void vpss_get_stat(u32 chn);
void vpss_isp_mutex_lock(u32 grp, int lock);
int vpss_isp_set_val_v2(u32 grp, u32 w, u32 h, int locked);
int vpss_isp_post_deal(u32 grp);
int iw_vpss_ioctl(unsigned int cmd, void __user *arg);
extern u32 vpss_ldc_va[N_CHN];

#endif
