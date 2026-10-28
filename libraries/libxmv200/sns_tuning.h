/* SPDX-License-Identifier: MIT */
/*
 * A sensor mode's tuning, as tools/xmv200-capture/gen_tuning.py writes it:
 * the sensor's registers, and the ISP program one channel runs: isp.ko's
 * parameter blocks in order, with the steps libxmv200 does itself between
 * them.
 */
#ifndef XMV200_SNS_TUNING_H
#define XMV200_SNS_TUNING_H

#include <stdint.h>

struct xmv200_sns_reg {
  uint16_t reg;
  uint8_t val;
};

/* the ISP buffers a channel allocates, which blocks address */
enum {
  XMV200_BUF_NONE,
  XMV200_BUF_NR_ALPHA,
  XMV200_BUF_NR_IDX,
  XMV200_BUF_NR_IDX_MAX,
  XMV200_BUF_NR_FETCH,
  XMV200_BUF_DEFOG,
  XMV200_BUF_COUNT
};

/* one isp.ko command and its payload; each patch writes the address of a
 * channel buffer, (phys - 0x80000000) >> 4, plus an offset in it */
struct xmv200_isp_block {
  uint32_t cmd;
  const uint8_t *data;
  uint32_t size;
  struct {
    uint16_t off;
    uint8_t buf;
    uint32_t rel;
  } patch[9];
};

/* program steps: a block, or one of libxmv200's own */
#define XMV200_OP_BLOCK(n) (0x8000 | (n))
enum {
  XMV200_OP_NOP,
  XMV200_OP_SNS_INIT,     /* sensor init registers */
  XMV200_OP_SNS_MODE,     /* sensor mode registers (frame timing) */
  XMV200_OP_SNS_EXPOSURE, /* the starting exposure and gain */
  XMV200_OP_SNS_FLIP,     /* the board's mirror/flip for this lens */
  XMV200_OP_MIPI_CTRL,    /* the MIPI receiver's lane control */
  XMV200_OP_VI_SYNC,      /* the receiver's sync timeouts */
  XMV200_OP_VI_WINDOW,    /* the VI capture window */
  XMV200_OP_NR_ALLOC,     /* 3D noise reduction buffers */
  XMV200_OP_DEFOG_ALLOC,  /* defog statistics buffer */
  XMV200_OP_LDC,          /* lens distortion table */
  XMV200_OP_CHN_ATTR,     /* 0x4908: frame length */
  XMV200_OP_AE_INIT,      /* 0x4964/0x4965: the exposure isp.ko starts from */
  XMV200_OP_CHN_INIT,     /* 0x490e: VICU and BPU channel */
  XMV200_OP_CHN_SIZE,     /* 0x4980: the channel's window */
  XMV200_OP_VICU_IRQ0,    /* the VICU's interrupt mask off */
  XMV200_OP_VICU_IRQ1,    /* ... and on */
  XMV200_OP_VICU_RUN,     /* 0x495e: capture */
};
#define XMV200_OP_VICU_IRQ(on) ((on) ? XMV200_OP_VICU_IRQ1 : XMV200_OP_VICU_IRQ0)

struct xmv200_reg {
  uint32_t addr, val;
};

struct xmv200_sns_tuning {
  /* the mode: the sensor's output, the clocks and the MIPI receiver */
  uint32_t width, height;
  uint32_t sns_clk[2];   /* 0x20000074 / 0x20000078 */
  uint32_t pll[2][5];    /* the ISP PLL before and after the sensors start:
                          * 0x14, 0x44, 0x48, 0x50, 0x54 */
  const struct xmv200_reg *mipi;
  int n_mipi;
  uint32_t mipi_ctrl;    /* 0x30004000 */
  uint32_t vi_sync;      /* 0x3000430c / 0x30004310 */
  /* isp.ko's structural commands as the vendor ISP gives them for channel
   * 0 (libxmv200 sets the channel and the board's choices) */
  uint32_t comm[14];     /* 0x4962 */
  uint32_t chn_init[13]; /* 0x490e */
  uint32_t chn_attr[7];  /* 0x4908 */
  uint32_t dgain;        /* 0x4964 */
  uint32_t gain[4];      /* 0x4965 */
  uint32_t ldc[12];      /* 0x49e9 */
  uint32_t buf_size[XMV200_BUF_COUNT]; /* the channel buffers */
  const struct xmv200_sns_reg *init_regs;
  int n_init_regs;
  const struct xmv200_sns_reg *mode_regs;
  int n_mode_regs;
  const struct xmv200_sns_reg *exposure_regs; /* the starting exposure */
  int n_exposure_regs;
  const uint16_t *program;
  int n_program;
  const struct xmv200_isp_block *blocks;
  const uint8_t *page; /* isp.ko's shared parameter page, three channels */
  uint32_t page_size;
  const uint8_t *ldc_table;
  uint32_t ldc_size;
};

extern const struct xmv200_sns_tuning xmv200_tuning_sc2336;

#endif
