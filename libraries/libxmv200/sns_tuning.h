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

/* a buffer a channel allocates, which blocks address: its MMZ name (a
 * format taking the channel) and size */
struct xmv200_isp_buf {
  const char *name;
  uint32_t size;
};

/* one isp.ko command and its payload; each patch writes the address of a
 * channel buffer (1 + its index in the tuning's bufs) plus an offset in it,
 * shifted: 4 for (phys - 0x80000000) >> 4, 0 for phys */
struct xmv200_isp_block {
  uint32_t cmd;
  const uint8_t *data;
  uint32_t size;
  struct {
    uint16_t off;
    uint8_t buf, shift;
    uint32_t rel;
  } patch[16];
};

/* program steps: a block, a buffer, or one of libxmv200's own */
#define XMV200_OP_BLOCK(n) (0x8000 | (n))
#define XMV200_OP_ALLOC(n) (0x4000 | (n)) /* the tuning's buffer n */
enum {
  XMV200_OP_NOP,
  XMV200_OP_SNS_INIT,     /* sensor init registers */
  XMV200_OP_SNS_MODE,     /* sensor mode registers (frame timing) */
  XMV200_OP_SNS_EXPOSURE, /* the starting exposure and gain */
  XMV200_OP_SNS_FLIP,     /* the board's mirror/flip for this lens */
  XMV200_OP_MIPI_CTRL,    /* the MIPI receiver's lane control */
  XMV200_OP_VI_SYNC,      /* the receiver's sync timeouts */
  XMV200_OP_VI_WINDOW,    /* the VI capture window */
  XMV200_OP_LDC,          /* lens distortion table */
  XMV200_OP_CHN_ATTR,     /* 0x4908: frame length */
  XMV200_OP_AE_INIT,      /* 0x4964/0x4965: the exposure isp.ko starts from */
  XMV200_OP_CHN_INIT,     /* 0x490e: VICU and BPU channel */
  XMV200_OP_CHN_SIZE,     /* 0x4980: the channel's window */
  XMV200_OP_VICU_IRQ0,    /* the VICU's interrupt mask off */
  XMV200_OP_VICU_IRQ1,    /* ... and on */
  XMV200_OP_VICU_RUN,     /* 0x495e: capture */
  XMV200_OP_CHN_ENABLE0,  /* 0x4914: the channel off */
  XMV200_OP_CHN_ENABLE1,  /* ... and on */
};
#define XMV200_OP_VICU_IRQ(on) ((on) ? XMV200_OP_VICU_IRQ1 : XMV200_OP_VICU_IRQ0)
#define XMV200_OP_CHN_ENABLE(on)                                              \
  ((on) ? XMV200_OP_CHN_ENABLE1 : XMV200_OP_CHN_ENABLE0)

struct xmv200_reg {
  uint32_t addr, val;
};

struct xmv200_sns_tuning {
  /* the chip family it was captured on, whose isp.ko it is for:
   * "xm530v200" or "xm535" (XM535D1/D2) */
  const char *chip;
  /* the mode: the sensor's output, the clocks and the MIPI receiver */
  uint32_t width, height;
  uint32_t sns_clk[2];   /* 0x20000074 / 0x20000078 (XM530V200) */
  uint32_t pll[2][5];    /* the ISP PLL before and after the sensors start:
                          * 0x14 (XM530V200) or 0x10 (XM535), then 0x44,
                          * 0x48, 0x50, 0x54 */
  const struct xmv200_reg *mipi;
  int n_mipi;
  uint32_t mipi_ctrl;    /* 0x30004000 */
  uint32_t vi_sync;      /* 0x3000430c / 0x30004310 */
  /* isp.ko's structural commands as the vendor ISP gives them for channel
   * 0 (libxmv200 sets the channel and the board's choices) */
  uint32_t comm[14];     /* 0x4962 */
  uint32_t chn_init[20]; /* 0x490e: 13 words (XM530V200) or 19 (XM535) */
  int n_chn_init;
  uint32_t chn_attr[7];  /* 0x4908 */
  uint32_t chn_size[12]; /* 0x4980: 3 words (XM530V200) or 12 (XM535) */
  int n_chn_size;
  uint32_t dgain;        /* 0x4964 */
  uint32_t gain[4];      /* 0x4965 */
  uint32_t ldc[12];      /* 0x49e9 */
  const struct xmv200_isp_buf *bufs; /* the channel buffers */
  int n_bufs;
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

/* the sensors there are tunings for (sns_registry.c, generated), per chip
 * family (the tuning's chip) */
struct xmv200_sns {
  uint16_t chip_id; /* SmartSens 0x3107 << 8 | 0x3108 */
  const char *name;
  const struct xmv200_sns_tuning *tuning;
};
extern const struct xmv200_sns xmv200_sensors[];
extern const int xmv200_n_sensors;

extern const struct xmv200_sns_tuning xmv200_tuning_sc2336;
extern const struct xmv200_sns_tuning xmv200_tuning_sc2336_xm535;

#endif
