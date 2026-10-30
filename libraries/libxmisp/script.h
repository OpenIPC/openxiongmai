/* SPDX-License-Identifier: MIT */
/*
 * Sensor scripts: the XM510 ISP start-up as the SoC's list of steps with
 * slots for one sensor and mode's parts, the ISP values that sensor needs
 * other than the SoC's, and a fixed 3A state applied once after it.
 * extract.py in tools/xm-isp-trace generates both from traces of the
 * vendor daemon.
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
  XMISP_I2C_R,      /* a: sensor register, read; b: what the trace read */
  XMISP_VI_IOW,     /* a: command, b: index in args[]: a count, then words */
  XMISP_ISP_BUF,    /* a: size of the ISP's MMZ block "Isp", allocated */
  XMISP_ISP_W,      /* a: ISP register, b: value */
  XMISP_ISP_W_BUF,  /* a: ISP register, b: offset in the "Isp" block; the
                       register takes the address in 8-byte units from DDR */
  XMISP_SENSOR,     /* a: the script's part (xmisp_part) that goes here */
};

/* A sensor and mode's parts of the start-up, in order. */
enum xmisp_part {
  XMISP_PART_PROBE, /* reset and chip ID */
  XMISP_PART_PLL,   /* its clock, through the system controller */
  XMISP_PART_VI,    /* VI channel setup: window, sizes */
  XMISP_PART_INIT,  /* its register writes */
  XMISP_PART_POST,  /* its register writes after the ISP's */
  XMISP_PARTS
};

struct xmisp_op {
  uint32_t type;
  uint32_t a;
  uint32_t b;
};

struct xmisp;

/* What the 3A needs of a sensor, beyond its script. */
/* Noise reduction and sharpening by gain, for one sensor: its strength goes
 * from 0 at gain_lo to 1 at gain_hi, taking the ISP from the script's
 * values to high[] (XM510 order: 2D NR BASE, THDH, COEF, DELTA_THD, the 3D
 * NR strength, SHARPEN_KD, KM0, KH0), and easing the colour matrix towards
 * identity by up to ccm_ease with the Cb/Cr gain going to cbcr (0x100 = 1). */
struct xmisp_nr_tune {
  float gain_lo, gain_hi;
  uint32_t high[8];
  float ccm_ease;
  uint32_t cbcr;
};

struct xmisp_sensor {
  uint32_t lines_min; /* exposure range in lines, within the frame */
  uint32_t lines_max;
  float gain_max;   /* the most gain AE may use; 1.0 = none */
  float ae_target;  /* mean luma AE holds (8-bit, linear); 0: the SoC's */
  uint32_t line_ns; /* the mode's line time, for anti-flicker; 0 unknown */
  /* White balance gains relative to G that the sensor's grey can need,
   * from warm light (high blue) to shade (high red). */
  float awb_r_min, awb_r_max;
  float awb_b_min, awb_b_max;
  /* Colour correction from white-balanced camera RGB to sRGB, rows of
   * three in units of 256, each row summing to 256 so grey stays grey;
   * NULL keeps the script's. */
  const int16_t *ccm;
  /* Noise reduction and sharpening by gain; NULL for the SoC's, chosen on
   * the JX-H62. */
  const struct xmisp_nr_tune *nr;
  /* The exposure the sensor has now. 0 or -1. */
  int (*get_exposure)(struct xmisp *isp, uint32_t *lines, float *gain);
  /* Sets it; *gain comes back as the step the sensor took. 0 or -1. */
  int (*set_exposure)(struct xmisp *isp, uint32_t lines, float *gain);
};

/* An ISP register's value for this sensor, other than the SoC's: it takes
 * the place of the register's last write in the template (an offset in the
 * "Isp" block for an XMISP_ISP_W_BUF write). Where the template writes a
 * register twice, the first is a default the same for every sensor. */
struct xmisp_isp_val {
  uint32_t addr; /* 0 ends the list */
  uint32_t val;
};

struct xmisp_script {
  const char *name;
  const struct xmisp_sensor *sensor; /* NULL: the fixed state only */
  uint16_t width, height;            /* VI channel 0: the main picture */
  uint8_t i2c_dev;                   /* 7-bit sensor address */
  uint8_t reg_bytes;                 /* 1 or 2 */
  uint8_t data_bytes;                /* 1 or 2 */
  const struct xmisp_op *part[XMISP_PARTS];
  const struct xmisp_isp_val *isp;
  uint32_t isp_buf; /* the "Isp" block's size; 0: the template's */
  const struct xmisp_op *fixed;
  const uint32_t *args; /* XMISP_VI_IOW arguments */
};

/* The XM510's start-up, with XMISP_SENSOR slots. */
extern const struct xmisp_op xmisp_xm510_start[];

/* The steps of soc with script's parts in their slots and its ISP values,
 * into out (up to max, the XMISP_END included). The number of steps, END
 * included; more than max if out is too short. */
unsigned xmisp_expand(const struct xmisp_op *soc,
                      const struct xmisp_script *script, struct xmisp_op *out,
                      unsigned max);

extern const struct xmisp_script xmisp_h62_720p;
extern const struct xmisp_sensor xmisp_sensor_h62;
extern const struct xmisp_sensor xmisp_sensor_ar0130;

#endif
