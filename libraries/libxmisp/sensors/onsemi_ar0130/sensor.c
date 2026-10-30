// SPDX-License-Identifier: MIT
/*
 * AR0130 exposure and gain, as measured on a 53X13 XM510 board under xmispd
 * with the AE statistics, 4000 K bulbs (the low means, before the bright
 * part of the scene clips):
 * - exposure in lines, register 0x3012; the statistics double with it
 *   (50, 100, 200 lines: x2.05, x1.97);
 * - coarse analogue gain in register 0x30b0 bits 5:4, 1x, 2x, 4x and 8x
 *   (measured x2.13, x4.36, x8.38 over 1x: the black level's share); the
 *   other bits are kept;
 * - fine digital gain in register 0x305e, value / 32 (0x20 is 1.0,
 *   measured x1.56 at 0x30, x2.1 at 0x40, x4.3 at 0x80, x8.2 at 0xff); from
 *   0x100 the picture goes black, so 0xff is the most it takes;
 * - the 720p25 script sets 750 lines a frame (0x300a = 0x2ee, 0x300c =
 *   0x780), which VI counts too: a line is 1 s / (25 x 750) = 53.33 us.
 * Gain is analogue first, which adds the least noise, then digital.
 */
#include "script.h"
#include "xmisp_priv.h"

#define REG_LINES 0x3012
#define REG_COARSE 0x30b0
#define COARSE_MASK 0x30
#define REG_FINE 0x305e
#define FINE_ONE 0x20
#define FINE_MAX 0xff

static float gain_of(uint32_t coarse_reg, uint32_t fine) {
  return (float)(1u << ((coarse_reg & COARSE_MASK) >> 4)) * (float)fine /
         (float)FINE_ONE;
}

static int get_exposure(struct xmisp *isp, uint32_t *lines, float *gain) {
  uint32_t l, c, f;
  if (xmisp_sensor_read(isp, REG_LINES, &l) < 0 ||
      xmisp_sensor_read(isp, REG_COARSE, &c) < 0 ||
      xmisp_sensor_read(isp, REG_FINE, &f) < 0)
    return -1;
  *lines = l;
  *gain = gain_of(c, f ? f : FINE_ONE);
  return 0;
}

static int set_exposure(struct xmisp *isp, uint32_t lines, float *gain) {
  unsigned step = 0; /* 1x, 2x, 4x, 8x */
  while (step < 3 && *gain >= (float)(2u << step))
    step++;
  int fine = (int)(*gain / (float)(1u << step) * FINE_ONE + 0.5f);
  fine = fine < FINE_ONE ? FINE_ONE : fine > FINE_MAX ? FINE_MAX : fine;
  uint32_t c;
  if (xmisp_sensor_read(isp, REG_COARSE, &c) < 0)
    return -1;
  c = (c & ~(uint32_t)COARSE_MASK) | step << 4;
  if (xmisp_sensor_write(isp, REG_LINES, lines) < 0 ||
      xmisp_sensor_write(isp, REG_COARSE, c) < 0 ||
      xmisp_sensor_write(isp, REG_FINE, (uint32_t)fine) < 0)
    return -1;
  *gain = gain_of(c, (uint32_t)fine);
  return 0;
}

/* Calibrated on the 53X13 board against a 24-patch colour checker in the
 * lab's midday daylight (tools/xm-isp-tune/colorchecker.py, AWB on, greys
 * R/G 1.01, B/G 1.05, gain 16): mean dE 15.6, against 23.9 for the JX-H62
 * vendor matrix and Cb/Cr gain. A fit at another spot in the room, under
 * the 5000 K bulbs, scored 16.1: their LED spectrum limits it as much as
 * the matrix. */
static const int16_t ccm_ar0130[9] = {
    388,  -81,  -51, /* R */
    -110, 364,  2,   /* G */
    -14,  -197, 467, /* B */
};

const struct xmisp_sensor xmisp_sensor_ar0130 = {
    .lines_min = 1,
    .lines_max = 746, /* a few lines short of the 750-line frame */
    .line_ns = 53333,
    .gain_max = 16.0f, /* 8x analogue, 2x digital: noise over brightness */
    /* The vendor daemon holds this sensor brighter than the JX-H62: a mean
     * of 58 in the same statistics (4000 K bulbs and daylight, 746 lines,
     * digital gain 1.28), against 37.5. */
    .ae_target = 58.0f,
    /* The chart's neutral on this sensor, closed loop on its greys: R 1.08,
     * B 2.55 at 2700 K and 1.10, 2.54 at 3000 K (RGBCCT bulbs), 1.19, 2.30
     * in daylight, 1.27, 2.21 at 6000 K, 1.50, 2.13 under the 200 W LED lamp.
     * With some room for warmer and cooler light; no wider: the orange
     * chair in the lab looks grey at R 0.77, B 3.40, and a range that
     * reached there kept AWB there in dim daylight. */
    .awb_r_min = 0.95f,
    .awb_r_max = 1.8f,
    .awb_b_min = 1.8f,
    .awb_b_max = 2.9f,
    .ccm = ccm_ar0130,
    .get_exposure = get_exposure,
    .set_exposure = set_exposure,
};
