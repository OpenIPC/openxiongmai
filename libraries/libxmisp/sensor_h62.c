// SPDX-License-Identifier: MIT
/*
 * JX-H62 exposure and gain, as measured on the lab XM510 under xmispd:
 * - exposure in lines, register 0x02 (high) and 0x01 (low); the statistics
 *   grow linearly with it;
 * - analogue gain in register 0x00: 2^[6:4] x (1 + [3:0]/16), each step of
 *   bits 6:4 doubling the statistics; 0x7x measured as 0x6x, so the top step
 *   is 6;
 * - the 720p25 script sets 750 lines a frame (registers 0x23/0x22).
 */
#include <math.h>

#include "script.h"
#include "xmisp_priv.h"

#define REG_GAIN 0x00
#define REG_LINES_LO 0x01
#define REG_LINES_HI 0x02
#define COARSE_MAX 6

static float gain_of(uint32_t reg) {
  return (float)(1u << ((reg >> 4) & 7)) * (1.0f + (float)(reg & 15) / 16.0f);
}

/* The largest step not above the gain asked for. */
static uint32_t reg_of(float gain) {
  unsigned coarse = 0;
  while (coarse < COARSE_MAX && gain >= (float)(2u << coarse))
    coarse++;
  int fine = (int)floorf((gain / (float)(1u << coarse) - 1.0f) * 16.0f);
  fine = fine < 0 ? 0 : fine > 15 ? 15 : fine;
  return coarse << 4 | (uint32_t)fine;
}

static int get_exposure(struct xmisp *isp, uint32_t *lines, float *gain) {
  uint32_t lo, hi, g;
  if (xmisp_sensor_read(isp, REG_LINES_LO, &lo) < 0 ||
      xmisp_sensor_read(isp, REG_LINES_HI, &hi) < 0 ||
      xmisp_sensor_read(isp, REG_GAIN, &g) < 0)
    return -1;
  *lines = hi << 8 | lo;
  *gain = gain_of(g);
  return 0;
}

static int set_exposure(struct xmisp *isp, uint32_t lines, float *gain) {
  const uint32_t g = reg_of(*gain);
  if (xmisp_sensor_write(isp, REG_LINES_LO, lines & 0xff) < 0 ||
      xmisp_sensor_write(isp, REG_LINES_HI, lines >> 8) < 0 ||
      xmisp_sensor_write(isp, REG_GAIN, g) < 0)
    return -1;
  *gain = gain_of(g);
  return 0;
}

const struct xmisp_sensor xmisp_sensor_h62 = {
    .lines_min = 1,
    .lines_max = 746,  /* a few lines short of the 750-line frame */
    .gain_max = 16.0f, /* 24 dB: noise over brightness, for now */
    .get_exposure = get_exposure,
    .set_exposure = set_exposure,
};
