/* SPDX-License-Identifier: MIT */
/*
 * Sensor scripts: the XM510 ISP start-up for one sensor and mode as a list
 * of steps, and a fixed 3A state applied once after it. extract.py in
 * tools/xm-isp-trace generates them from a trace of the vendor daemon.
 */
#ifndef XMISP_SCRIPT_H
#define XMISP_SCRIPT_H

#include <stdint.h>

enum xmisp_op_type {
  XMISP_END,
  XMISP_DELAY_MS,   /* a: milliseconds */
  XMISP_MMZ_RESET,  /* free every MMZ block (_IO('M', 0)) */
  XMISP_VI_INIT,    /* _IO('V', 6) */
  XMISP_RGN_RESET,  /* _IO('R', 1) */
  XMISP_SYS_W,      /* a: system controller register, b: value */
  XMISP_I2C_W,      /* a: sensor register, b: value */
  XMISP_I2C_EXPECT, /* a: sensor register, b: the value it must read (ID) */
  XMISP_VI_IOW,     /* a: command, b: index in args[]: a count, then words */
  XMISP_ISP_BUF,    /* a: size of the ISP's MMZ block "Isp", allocated */
  XMISP_ISP_W,      /* a: ISP register, b: value */
  XMISP_ISP_W_BUF,  /* a: ISP register, b: offset in the "Isp" block; the
                       register takes the address in 8-byte units from DDR */
};

struct xmisp_op {
  uint32_t type;
  uint32_t a;
  uint32_t b;
};

struct xmisp;

/* What the 3A needs of a sensor, beyond its script. */
struct xmisp_sensor {
  uint32_t lines_min; /* exposure range in lines, within the frame */
  uint32_t lines_max;
  float gain_max; /* the most gain AE may use; 1.0 = none */
  /* White balance gains relative to G that the sensor's grey can need,
   * from warm light (high blue) to shade (high red). */
  float awb_r_min, awb_r_max;
  float awb_b_min, awb_b_max;
  /* The exposure the sensor has now. 0 or -1. */
  int (*get_exposure)(struct xmisp *isp, uint32_t *lines, float *gain);
  /* Sets it; *gain comes back as the step the sensor took. 0 or -1. */
  int (*set_exposure)(struct xmisp *isp, uint32_t lines, float *gain);
};

struct xmisp_script {
  const char *name;
  const struct xmisp_sensor *sensor;
  uint8_t i2c_dev; /* 7-bit sensor address; 8-bit registers and data */
  const struct xmisp_op *start;
  const struct xmisp_op *fixed;
  const uint32_t *args;
};

extern const struct xmisp_script xmisp_h62_720p;
extern const struct xmisp_sensor xmisp_sensor_h62;

#endif
