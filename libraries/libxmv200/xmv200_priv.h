/* SPDX-License-Identifier: MIT */
/* libxmv200 internals: how the pipeline is laid out for a board */
#ifndef XMV200_PRIV_H
#define XMV200_PRIV_H

#include <stdint.h>

struct xmv200_sns_tuning;

struct xmv200_isp_sensor {
  int bus;      /* /dev/i2cN */
  uint8_t addr; /* 7-bit */
  uint8_t flip; /* SmartSens 0x3221: 0x66 mirrors and flips */
};

/* one ISP channel: a sensor's capture, before VPSS (the capture size and
 * timing are the sensor mode's, in its tuning) */
struct xmv200_isp_chn {
  uint32_t chn;
  uint32_t mipi;                  /* XM530V200: the receiver a second
                                   * channel takes */
  uint32_t x0, y0;                /* the lens' crop in the sensor's output */
  uint32_t out_width, out_height; /* the picture VPSS makes of it */
  int ldc;                        /* lens distortion correction */
  /* XM535: the MIPI receiver (phy) the sensor is on, and the VI device the
   * channel is on with its index there (the X6E-WEQ's two sensors: phy 0
   * and 1, both on device 0) */
  uint32_t phy, vi_dev, vi_idx;
};

struct xmv200_isp_cfg {
  const struct xmv200_sns_tuning *tuning;
  int n_chn;
  struct xmv200_isp_chn chn[3];
  struct xmv200_isp_sensor sensor[3];
};

/* What differs between the chip families' kernel modules and clocks, for
 * the same steps (the tuning's chip picks one). */
struct xmv200_chip {
  const char *name;
  const char *engine;   /* the H.265 engine (xmv200_venc_set_engine) */
  int sys_clocks;       /* the ISP PLL through sys.ko (0x40085301), the
                         * sensor clocks left to the module loader */
  int vi_window;        /* VICU windows through /dev/mem before isp.ko */
  int vi_sync2;         /* the receiver's sync timeouts at 0x430c and 0x4310 */
  int frame_answer;     /* every frame wait answered with 0x4964/0x4965 */
  int vi_mode;          /* VI +0x00 per channel from its VI device */
  int chn_attr_idx;     /* 0x4908 word 0: chn | vi_idx << 4 */
  uint32_t hevc_reset;  /* hevc.ko 0x40044806 */
  int vb_mode;          /* vb.ko 0x40044205 {1} before the pools */
  int isp_pool;         /* a VB pool for the ISP's output to VPSS */
  int vb_count;         /* buffers per stream pool */
  int venc_v2;          /* 0x41044901 as the XM535's Sofia fills it */
};
const struct xmv200_chip *xmv200_chip_of(const struct xmv200_sns_tuning *t);

int xmv200_isp_start(const struct xmv200_isp_cfg *cfg);
void xmv200_isp_stop(void);
int xmv200_isp_frames_start(void);
/* program boards: the digital gain (isp.ko 0x4964, 0x100 = 1x) the frame
 * thread sends with every frame */
void xmv200_isp_set_dgain(uint32_t chn, uint32_t gain);

/* a sensor, for AE: its I2C, its ISP channel and where its picture is
 * (band of n_bands, top to bottom, in VENC channel stream) */
struct xmv200_ae_sensor {
  int bus;
  uint8_t addr;
  uint32_t isp_chn;
  uint32_t stream, band, n_bands;
};

/* a layout board's sensors, where[] by channel; handoff: the digital gain
 * goes to the frame thread (frame_answer) */
void xmv200_ae_init(const struct xmv200_isp_cfg *cfg,
                    const struct xmv200_ae_sensor *where, int handoff);
struct xmv200_prog;
void xmv200_ae_init_prog(const struct xmv200_ae_sensor *s, int n,
                         const struct xmv200_prog *p);

#endif
