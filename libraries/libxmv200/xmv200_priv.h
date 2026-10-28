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
  uint32_t mipi;                  /* the receiver a second channel takes */
  uint32_t x0, y0;                /* the lens' crop in the sensor's output */
  uint32_t out_width, out_height; /* the picture VPSS makes of it */
  int ldc;                        /* lens distortion correction */
};

struct xmv200_isp_cfg {
  const struct xmv200_sns_tuning *tuning;
  int n_chn;
  struct xmv200_isp_chn chn[2];
  struct xmv200_isp_sensor sensor[2];
};

int xmv200_isp_start(const struct xmv200_isp_cfg *cfg);
void xmv200_isp_stop(void);
int xmv200_isp_frames_start(void);
/* program boards: the digital gain (isp.ko 0x4964, 0x100 = 1x) the frame
 * thread sends with every frame */
void xmv200_isp_set_dgain(uint32_t chn, uint32_t gain);
void xmv200_ae_init(const struct xmv200_isp_cfg *cfg);

/* a program board's sensor, for AE: its I2C, its ISP channel and where its
 * picture is (band of n_bands, top to bottom, in VENC channel stream) */
struct xmv200_ae_sensor {
  int bus;
  uint8_t addr;
  uint32_t isp_chn;
  uint32_t stream, band, n_bands;
};

struct xmv200_prog;
void xmv200_ae_init_prog(const struct xmv200_ae_sensor *s, int n,
                         const struct xmv200_prog *p);

#endif
