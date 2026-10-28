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

/* one ISP channel: a sensor's capture, before VPSS */
struct xmv200_isp_chn {
  uint32_t chn;
  uint32_t mipi;            /* the receiver a second channel takes */
  uint32_t x0, y0;          /* crop inside the sensor's output */
  uint32_t width, height;   /* the capture */
  uint32_t hts, frame_lines; /* the sensor mode's line length and VTS */
  uint32_t exposure;        /* starting exposure, lines */
  uint32_t out_width, out_height; /* the picture VPSS makes of it */
  int ldc;                  /* lens distortion correction */
};

struct xmv200_isp_cfg {
  const struct xmv200_sns_tuning *tuning;
  int n_chn;
  struct xmv200_isp_chn chn[2];
  struct xmv200_isp_sensor sensor[2];
};

int xmv200_isp_start(const struct xmv200_isp_cfg *cfg);
void xmv200_isp_stop(void);

#endif
