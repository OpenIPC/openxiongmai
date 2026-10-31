/* SPDX-License-Identifier: MIT */
/* libxmv200 internals: how the pipeline is laid out for a board */
#ifndef XMV200_PRIV_H
#define XMV200_PRIV_H

#include "xmv200.h"
#include <stdint.h>

#include "sns.h"

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
  /* the sensors' driver and its mode, or NULL: the tuning's recorded runs */
  const struct xmv200_sns_drv *drv;
  const struct xmv200_sns_mode *mode;
  struct xmv200_sns_attr attr; /* the mode's, and the frame rate */
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
  uint32_t rx, vicu;    /* VI's MIPI receiver and first VICU, from VI */
  uint32_t vi_ctrl;     /* VI's input select register, from VI */
  int vi_ctrl_chn;      /* ... and the channel after it (+0x18) */
  int comm_n;           /* 0x4962: the byte with the ISP count (0: none) */
  int comm_ib;          /* 0x4962 word 9: the BPU's raw buffers */
  /* no VPSS: VI (vi.ko) scales and queues the streams for VENC, in this
   * pixel format; no VENC configuration */
  int vi_scaler;
  uint32_t vi_format;
};
const struct xmv200_chip *xmv200_chip_of(const struct xmv200_sns_tuning *t);

int xmv200_isp_start(const struct xmv200_isp_cfg *cfg);
void xmv200_isp_stop(void);
/* the digital gain (isp.ko 0x4964, 0x100 = 1x) the frame thread sends with
 * every frame, on the chips that answer each frame wait */
void xmv200_isp_set_dgain(uint32_t chn, uint32_t gain);
/* the white balance (0x4965 R, G, B) it sends with it, where AWB has none */
void xmv200_isp_set_wb(uint32_t chn, const uint32_t g[3]);

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
/* the longest exposure a channel's sensor driver allows for the frame rate
 * it set, or 0 */
uint32_t xmv200_isp_max_lines(uint32_t chn);
/* the AE limits a channel's sensor driver gave (cmos_get_ae_default and its
 * frame rate), or NULL without a driver */
const struct xmv200_sns_ae *xmv200_isp_sns_ae(uint32_t chn);
/* the frame length (lines) a channel's sensor driver set, or 0 */
uint32_t xmv200_isp_full_lines(uint32_t chn);
/* a channel's statistics (write() the channel, read() len bytes) on a
 * /dev/isp descriptor, serialised between AE and AWB; -1 on failure */
int xmv200_isp_stats(int fd, uint32_t chn, void *buf, size_t len);
/* an isp.ko command, the ISP running; -1 otherwise */
int xmv200_isp_cmd(uint32_t cmd, void *arg);
/* the running ISP's configuration, or NULL */
const struct xmv200_isp_cfg *xmv200_isp_running(void);
/* the last of the image blocks (0x49de luma/chroma gains, 0x49df tone
 * curve, 0x49e4 hue, 0x49f0 sharpening) a channel's program sent, or NULL */
#define XMV200_IMG_BLOCKS 4
#define XMV200_IMG_MAX 0x200
const uint8_t *xmv200_isp_image_block(uint32_t chn, uint32_t cmd,
                                      uint32_t *size);
/* AE's anti-flicker: mains hz (50, 60), 0 off */
void xmv200_ae_set_flicker(int hz);
/* the running layout's streams, for the covers: each stream's picture per
 * sensor (spliced top to bottom) and which sensors it has; vi: VI channel
 * n makes stream n (no VPSS) */
struct xmv200_geom {
  int n_sensors, n_streams, vi;
  struct {
    uint32_t width, height, sensors;
  } stream[XMV200_MAX_STREAMS];
};
/* 0, or -1 when no layout runs */
int xmv200_layout_geom(struct xmv200_geom *g);
/* md.c: a sensor's picture (its band of a frame) to compare; reset when a
 * layout starts */
void xmv200_md_sample(int sensor, const uint8_t *luma, uint32_t stride,
                      uint32_t width, uint32_t y0, uint32_t height);
void xmv200_md_reset(void);
/* md.c: blind detection's look at AE's 8x8 grid (8.8 brightness) */
void xmv200_bd_sample(int sensor, const uint32_t *grid8);
void xmv200_bd_reset(void);
/* mask.c: the masks forgotten (a layout started) */
void xmv200_mask_reset(void);
/* image.c: the controls back at the tuning's (a layout started) */
void xmv200_image_reset(void);
void xmv200_ae_init(const struct xmv200_isp_cfg *cfg,
                    const struct xmv200_ae_sensor *where, int handoff);
/* h264.c: H.264 on the Hantro H1 (venc.c drives it) */
struct h264enc;
struct xmv200_frame;
struct h264enc *xmv200_h264_create(uint32_t w, uint32_t h, uint32_t fps);
void xmv200_h264_destroy(struct h264enc *e);
void xmv200_h264_set_fps(struct h264enc *e, uint32_t fps);
/* Main profile (CABAC) from the next IDR, or back to Baseline */
void xmv200_h264_set_cabac(struct h264enc *e, int on);
int xmv200_h264_encode(struct h264enc *e, const struct xmv200_frame *f,
                       int idr, int qp, uint8_t *out, int cap);

#endif
