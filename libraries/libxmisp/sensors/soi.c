// SPDX-License-Identifier: MIT
/* The SOI JX-H sensors' exposure and gain (soi.h). */
#include <math.h>

#include "soi.h"
#include "xmisp_priv.h"

#define REG_GAIN 0x00
#define REG_LINES_LO 0x01
#define REG_LINES_HI 0x02

float xmisp_soi_gain(uint32_t reg, unsigned coarse_max) {
  unsigned coarse = (reg >> 4) & 7;
  if (coarse > coarse_max)
    coarse = coarse_max;
  return (float)(1u << coarse) * (1.0f + (float)(reg & 15) / 16.0f);
}

uint32_t xmisp_soi_gain_reg(float gain, unsigned coarse_max) {
  unsigned coarse = 0;
  while (coarse < coarse_max && gain >= (float)(2u << coarse))
    coarse++;
  int fine = (int)roundf((gain / (float)(1u << coarse) - 1.0f) * 16.0f);
  if (fine > 15 && coarse < coarse_max)
    return (coarse + 1) << 4; /* rounds up into the next doubling */
  fine = fine < 0 ? 0 : fine > 15 ? 15 : fine;
  return coarse << 4 | (uint32_t)fine;
}

int xmisp_soi_get(struct xmisp *isp, unsigned coarse_max, uint32_t *lines,
                  float *gain) {
  uint32_t lo, hi, g;
  if (xmisp_sensor_read(isp, REG_LINES_LO, &lo) < 0 ||
      xmisp_sensor_read(isp, REG_LINES_HI, &hi) < 0 ||
      xmisp_sensor_read(isp, REG_GAIN, &g) < 0)
    return -1;
  *lines = hi << 8 | lo;
  *gain = xmisp_soi_gain(g, coarse_max);
  return 0;
}

int xmisp_soi_set(struct xmisp *isp, unsigned coarse_max, uint32_t lines,
                  float *gain) {
  const uint32_t g = xmisp_soi_gain_reg(*gain, coarse_max);
  if (xmisp_sensor_write(isp, REG_LINES_LO, lines & 0xff) < 0 ||
      xmisp_sensor_write(isp, REG_LINES_HI, lines >> 8) < 0 ||
      xmisp_sensor_write(isp, REG_GAIN, g) < 0)
    return -1;
  *gain = xmisp_soi_gain(g, coarse_max);
  return 0;
}
