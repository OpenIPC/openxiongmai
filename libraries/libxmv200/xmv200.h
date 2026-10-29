/* SPDX-License-Identifier: MIT */
/*
 * libxmv200: the media pipeline of the Xiongmai V200 series (XM530V200,
 * XM533, XM535D1/D2/W1) on the vendor kernel modules (i2c, mmz, vb, sys,
 * isp, vpss or vi, venc, hevc), without the vendor userspace (Sofia, isp).
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
 * A board's whole pipeline: sensors, ISP, VPSS (or VI's scaler), the VENC
 * channels. After it the streams' pictures queue for xmv200_frame_get.
 *
 * A board is a layout: its sensors, how they are wired, and the streams to
 * make of them. It can come from a configuration file; the sensor's mode
 * (its registers and the ISP's tuning for it) comes from the tunings
 * libxmv200 has for the running chip.
 */
#define XMV200_MAX_STREAMS 6
#define XMV200_MAX_SENSORS 3

struct xmv200_layout_sensor {
  const char *model; /* "SC2336"; NULL: probe its chip ID */
  int bus;           /* /dev/i2cN */
  uint8_t addr;      /* 7-bit */
  uint8_t flip;      /* SmartSens 0x3221: 0x66 mirrors and flips */
  uint32_t x0, y0;   /* the lens' crop in the sensor's output */
  /* the ISP's picture of it, what VPSS or VI scales from (0: the mode's,
   * to whole lines of 8 and columns of 16) */
  uint32_t width, height;
  /* how it is wired: its MIPI receiver (phy), and on the XM535D1/D2 the VI
   * device its channel is on with its index there; on the XM530V200 the
   * receiver input a second sensor takes (mipi) */
  uint32_t phy, vi_dev, vi_idx, mipi;
  int ldc; /* lens distortion correction, where the tuning has it */
};

struct xmv200_layout_stream {
  uint32_t width, height; /* per sensor */
  uint32_t sensors;       /* mask: several are spliced top to bottom */
};

struct xmv200_layout {
  const char *chip; /* "xm535d1", ...; NULL: the running kernel's */
  int n_sensors;
  struct xmv200_layout_sensor sensor[XMV200_MAX_SENSORS];
  int n_streams; /* stream n is VENC channel n */
  struct xmv200_layout_stream stream[XMV200_MAX_STREAMS];
  uint32_t fps;
};

struct xmv200_board_info {
  int n_streams;
  uint32_t fps;
  struct {
    uint32_t width, height; /* what VENC channel n gets */
  } stream[XMV200_MAX_STREAMS];
};

int xmv200_layout_start(const struct xmv200_layout *l,
                        struct xmv200_board_info *info);
/* a known board's layout ("x6c-weq", ...), or "auto": the sensors found on
 * I2C, one picture each or two of a kind spliced, and a 640-wide stream */
int xmv200_board_layout(const char *board, struct xmv200_layout *l);
/* a layout from text (layout.c has the format), and back */
int xmv200_layout_parse(const char *text, struct xmv200_layout *l);
int xmv200_layout_format(const struct xmv200_layout *l, char *out, int cap);
/* xmv200_board_layout, then xmv200_layout_start */
int xmv200_board_start(const char *board, struct xmv200_board_info *info);
/* the running chip ("xm530v200", "xm535d1", ...), or NULL */
const char *xmv200_chip_name(void);

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

/* Auto exposure and white balance of the board's sensors: give it every
 * frame before releasing it (it takes the streams its sensors are in, and
 * samples every third of each). */
void xmv200_ae_frame(const struct xmv200_frame *f);

/* Encodes a frame into out (Annex B, parameter sets before each IDR).
 * Returns the length, or -1. */
int xmv200_venc_encode(struct xmv200_venc *v, const struct xmv200_frame *f,
                       uint8_t *out, int cap);

#ifdef __cplusplus
}
#endif

#endif
