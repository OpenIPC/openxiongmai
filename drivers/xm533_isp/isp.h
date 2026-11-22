/* SPDX-License-Identifier: GPL-2.0 */
/*
 * The XM533's isp.ko, shared state. isp.ko keeps one block of parameters
 * (vendor: g_comm_para, 0x1848 bytes) that the ioctls fill and the frame
 * work reads; its fields are kept at isp.ko's offsets, named where their
 * use is known.
 */
#ifndef XM533_ISP_H
#define XM533_ISP_H

#include <linux/mutex.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/interrupt.h>
#include <linux/seq_file.h>

#define COMM_SIZE 0x1848
extern u8 isp_comm[COMM_SIZE];

#define C8(o) (*(u8 *)(isp_comm + (o)))
#define CS8(o) (*(s8 *)(isp_comm + (o)))
#define C16(o) (*(u16 *)(isp_comm + (o)))
#define CS16(o) (*(s16 *)(isp_comm + (o)))
#define C32(o) (*(u32 *)(isp_comm + (o)))
#define CP(o) ((void *)(isp_comm + (o)))

/* g_comm_para fields */
#define CM_BPU_BUSY 0x0000  /* u8: the BPU thread running */
#define CM_FLAGS 0x0001     /* u8: 0x20 limit-up; 0x40/0x20 crop/size changes */
#define CM_DUMP_REQ 0x0038  /* u8: a dump is being set up */
#define CM_DUMP 0x0039      /* u8: 1 dump the next frame; 0x10 done, 0x20 failed */
#define CM_DUMP_W 0x003a    /* u16 */
#define CM_DUMP_H 0x003c    /* u16 */
#define CM_DUMP_ADDR 0x0048 /* the dump's physical address */
#define CM_DUMP_SIZE 0x004c
#define CM_WIDTH 0x005c     /* 0x4962: the sensor's picture */
#define CM_HEIGHT 0x0060
#define CM_MODE 0x0064      /* u8: BPU mode, 0 online, 1 offline, 2/3 HDR */
#define CM_BLKNUM 0x0065    /* u8: ISP buffers */
#define CM_CHNNUM 0x0066    /* u8: VICU channels configured */
#define CM_VPSS_EN 0x0067   /* u8: bit 0 the VPSS-side ISP */
#define CM_FPS_CTRL 0x006a  /* u16 */
#define CM_VPSS_MODE 0x006b /* u8: 1 offline */
#define CM_OPT 0x006e       /* s8: bit 7 no defaults; bit 0 memory saver */
#define CM_VICU_SIZE 0x0090 /* the VPSS input, w | h << 16 */
#define CM_WDR_MODE 0x0098
#define CM_SNS_ID 0x009c
#define CM_VICU_MODE 0x00b0
#define CM_VICU_FPS 0x00d8  /* tenths of a frame per second */
#define CM_VICU_CNT 0x00dc
#define CM_AWB_STAT 0x01b0  /* 3 x 0x200 u16: the AWB statistics */
#define CM_VSYNC_CNT 0x0dc4 /* frames for 0x490a */
#define CM_BPU_GO 0x0dc8    /* frames for the BPU thread */
#define CM_GRP 0x0dd0       /* the BPU group, 0x980 a channel */
#define CM_IW_WAIT 0x1750   /* 1 << chn: wait for the parameters */
#define CM_VPSS_HOLD 0x1754 /* 1 << the VPSS group, 0xff off */
#define CM_RUN 0x1758       /* 1 once 0x4962 has set things up */
#define CM_VER 0x175c       /* 0x495d: the user library's version */
#define CM_LIMITUP 0x1784   /* vpss_isp_set_attr: x, y, w, h */
#define CM_REPAIR 0x1794
#define CM_PRODUCT 0x1798
#define CM_CHN_ATTR 0x1828  /* 0x4908, 0x1c a channel */

/* the four register windows and the PLL block */
int isp_write_reg(u32 addr, u32 val);
u32 isp_read_reg(u32 addr);
int pll_write_reg(u32 addr, u32 val);
u32 pll_read_reg(u32 addr);
int isp_users_open(void);

extern struct mutex bpu_lock;          /* vendor: bpu_grp[0].bpu_lock */
extern struct mutex iwctrl_lock;       /* the iw parameter ioctls */
extern struct mutex iwctrl_vpss_lock;  /* the iw VPSS parameter ioctls */
extern struct mutex vpss_limitup_lock; /* CM_FLAGS and CM_LIMITUP */
extern wait_queue_head_t dump_wq;      /* CM_DUMP */
extern wait_queue_head_t vsync_wq;     /* CM_VSYNC_CNT */
extern wait_queue_head_t bpu_thread_wq; /* CM_BPU_GO */
extern int isp_vpss_grp;               /* the VPSS group isp_exit resets */

/* bpu.c */
void bpu_init_queue(void);
void bpu_reset(void);
int bpu_init(void);
int bpu_chn_init(const u8 *attr);
int bpu_post_deal(u32 chn);
int bpu_set_wdr_mode(int chn, u32 mode);
int bpu_set_enable(int chn, int en);
int bpu_thread(void *unused);

/* ib.c */
int ib_init(const u32 *pools, u32 w, u32 h);
void ib_proc_show(struct seq_file *m, int *n);

/* iw.c */
void iw_isp_init(void);
int iw_ioctl(unsigned int cmd, void __user *arg);

/* vicu.c */
void vicu_init_queue(void);
int vicu_chn_init(const u8 *attr);
int vicu_run(int chn);
void vicu_reset(void);
irqreturn_t vicu_irq_0(int irq, void *dev);

/* vpss_isp.c */
void vpss_init_queue(void);
void vpss_isp_reset(int grp);
int vpss_isp_set_val(u32 grp, u32 w, u32 h);
int vpss_isp_post_deal(u32 grp, u32 w, u32 h);
int iw_vpss_ioctl(unsigned int cmd, void __user *arg);
void vpss_isp_proc_show(struct seq_file *m, int *n);

#endif
