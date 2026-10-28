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

#ifdef __cplusplus
}
#endif

#endif
