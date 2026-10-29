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
  unsigned coarse = (reg >> 4) & 7;
  if (coarse > COARSE_MAX)
    coarse = COARSE_MAX; /* 0x7x measured as 0x6x */
  return (float)(1u << coarse) * (1.0f + (float)(reg & 15) / 16.0f);
}

/* The step nearest the gain asked for. */
static uint32_t reg_of(float gain) {
  unsigned coarse = 0;
  while (coarse < COARSE_MAX && gain >= (float)(2u << coarse))
    coarse++;
  int fine = (int)roundf((gain / (float)(1u << coarse) - 1.0f) * 16.0f);
  if (fine > 15 && coarse < COARSE_MAX)
    return (coarse + 1) << 4; /* rounds up into the next doubling */
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
    /* The range the vendor daemon's AWB searches on this sensor (its
     * candidate gains, traced). Daylight grey on the lab chart is R 1.56,
     * B 2.19. */
    .awb_r_min = 0.75f,
    .awb_r_max = 2.8f,
    .awb_b_min = 1.1f,
    .awb_b_max = 3.6f,
    .get_exposure = get_exposure,
    .set_exposure = set_exposure,
};
