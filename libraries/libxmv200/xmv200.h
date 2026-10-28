/* SPDX-License-Identifier: MIT */
/*
 * libxmv200: the media pipeline of the Xiongmai V200 series (XM530V200,
 * XM533, XM535) on the vendor kernel modules (i2c, mmz, vb, sys, isp, vpss,
 * hevc), without the vendor userspace (Sofia, isp).
 *
 * The vendor modules do the register work. This library makes the calls the
 * vendor userspace makes: device ioctls, the few registers it writes through
 * /dev/mem (PLLs, MIPI receiver, VI windows), and the sensor over I2C.
 */
#ifndef XMV200_H
#define XMV200_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Registers through /dev/mem */
int xmv200_reg_write(uint32_t addr, uint32_t val);
int xmv200_reg_read(uint32_t addr, uint32_t *val);

/* Sensor I2C through i2c.ko (/dev/i2cN). dev is the 8-bit address. */
int xmv200_i2c_write(int bus, uint8_t dev, uint32_t reg, int reg_bytes,
                     uint32_t val, int val_bytes);
int xmv200_i2c_read(int bus, uint8_t dev, uint32_t reg, int reg_bytes,
                    uint32_t *val, int val_bytes);

/* A block of MMZ memory: physical address and its mapping */
struct xmv200_buf {
  uint32_t phys;
  uint8_t *virt;
  uint32_t size;
};

/* mmz.ko: a reset releases every block; nothing else allocates before it. */
int xmv200_mmz_reset(void);
int xmv200_mmz_alloc(struct xmv200_buf *b, const char *name, uint32_t size,
                     int cached);

/* vb.ko: the pools VPSS takes its output frames from */
#define XMV200_VB_MAX_POOLS 32
struct xmv200_vb_pool {
  uint32_t size;  /* bytes per block */
  uint32_t count; /* blocks */
};
int xmv200_vb_init(const struct xmv200_vb_pool *pools, int n);

/*
 * A board's whole pipeline: sensors, ISP, VPSS, the VENC channels. After it
 * the streams' pictures queue for xmv200_frame_get.
 */
struct xmv200_board_info {
  int n_streams;
  uint32_t fps;
  struct {
    uint32_t width, height; /* what VENC channel n gets */
  } stream[2];
};
int xmv200_board_start(const char *board, struct xmv200_board_info *info);

/*
 * H.265 encoding on the hevc.ko engine, one encoder per stream size. Frames
 * come from venc.ko's FIFO, which the VPSS channels bound to VENC fill.
 * qp 0 = 35, gop 0 = two seconds.
 */
struct xmv200_venc;
/* the chip whose encoder engine to drive ("xm530v200", "xm535d1", ...);
 * without it, the kernel's machine name */
void xmv200_venc_set_engine(const char *chip);
int xmv200_venc_create(struct xmv200_venc **v, uint32_t width,
                       uint32_t height, uint32_t fps, uint32_t qp,
                       uint32_t gop);
void xmv200_venc_destroy(struct xmv200_venc *v);
void xmv200_venc_request_idr(struct xmv200_venc *v);

/* A picture VPSS queued for a VENC channel (venc.ko's FIFO, every channel's
 * pictures in one queue). Each must be released. */
struct xmv200_frame {
  uint32_t chn; /* the VENC channel its VPSS channel is bound to */
  uint32_t y, c, width, height, stride;
  uint64_t pts;
  uint32_t priv[10];
};
int xmv200_frame_get(struct xmv200_frame *f); /* waits up to 2 s */
void xmv200_frame_release(struct xmv200_frame *f);

/* Auto exposure and white balance of the board's sensors: give it stream
 * 0's frames (it samples every third) before releasing them. */
void xmv200_ae_frame(const struct xmv200_frame *f);

/* Encodes a frame into out (Annex B, parameter sets before each IDR).
 * Returns the length, or -1. */
int xmv200_venc_encode(struct xmv200_venc *v, const struct xmv200_frame *f,
                       uint8_t *out, int cap);

#ifdef __cplusplus
}
#endif

#endif
