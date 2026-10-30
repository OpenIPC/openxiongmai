// SPDX-License-Identifier: MIT
/*
 * SC1135 exposure and gain, from the XM510 vendor daemon's 3A as traced in
 * qemu-bench; not yet run on hardware:
 * - exposure in 1/16 lines, register 0x3e01 (bits 11:4 of the lines) and
 *   0x3e02 (bits 3:0, shifted up 4);
 * - analogue gain in register 0x3e09: bits 6:4 a doubling each as they fill
 *   (0x00 1x, 0x10 2x, 0x30 4x, 0x70 8x) times 1 + [3:0]/16, the codes the
 *   vendor's dark ramp passes through (0x0e 1.875x, 0x1c 3.5x, 0x72 9x match
 *   the gains it steps through on the JX-H62 and AR0130); register 0x3e08 a
 *   further 2x and 4x (0x01, 0x03), which the vendor used only past 0x7f;
 * - 0x3630/0x3631 = 0xb8/0x82 at 1x coarse, 0x70/0x8e from 2x, and 0x8c in
 *   0x3631 once 0x3e08 is set: written before the gain, as the vendor does.
 * 720p: 750 lines a frame (0x320e/0x320f = 0x2ee), exposure to 746; 960p:
 * 1000 lines (0x3e8), exposure to 996, both at the daemon's 25 fps. The
 * white balance range is the one the vendor's AWB searches under emulation.
 */
#include <math.h>

#include "script.h"
#include "xmisp_priv.h"

#define REG_LINES_HI 0x3e01
#define REG_LINES_LO 0x3e02
#define REG_DIGITAL 0x3e08
#define REG_GAIN 0x3e09
#define COARSE_MAX 3 /* 0x70: 8x */

static float gain_of(uint32_t g, uint32_t digital) {
  unsigned k = 0;
  for (uint32_t c = (g >> 4) & 7; c & 1; c >>= 1)
    k++;
  for (digital &= 3; digital & 1; digital >>= 1)
    k++;
  return (float)(1u << k) * (1.0f + (float)(g & 15) / 16.0f);
}

/* The step nearest the gain asked for, analogue only. */
static uint32_t reg_of(float gain) {
  unsigned k = 0;
  while (k < COARSE_MAX && gain >= (float)(2u << k))
    k++;
  int fine = (int)roundf((gain / (float)(1u << k) - 1.0f) * 16.0f);
  if (fine > 15 && k < COARSE_MAX) {
    k++;
    fine = 0; /* rounds up into the next doubling */
  }
  fine = fine < 0 ? 0 : fine > 15 ? 15 : fine;
  return ((1u << k) - 1) << 4 | (uint32_t)fine;
}

static int get_exposure(struct xmisp *isp, uint32_t *lines, float *gain) {
  uint32_t hi, lo, g, d;
  if (xmisp_sensor_read(isp, REG_LINES_HI, &hi) < 0 ||
      xmisp_sensor_read(isp, REG_LINES_LO, &lo) < 0 ||
      xmisp_sensor_read(isp, REG_GAIN, &g) < 0 ||
      xmisp_sensor_read(isp, REG_DIGITAL, &d) < 0)
    return -1;
  *lines = hi << 4 | lo >> 4;
  *gain = gain_of(g, d);
  return 0;
}

static int set_exposure(struct xmisp *isp, uint32_t lines, float *gain) {
  const uint32_t g = reg_of(*gain);
  const int low = (g & 0x70) == 0;
  if (xmisp_sensor_write(isp, REG_LINES_HI, (lines >> 4) & 0xff) < 0 ||
      xmisp_sensor_write(isp, REG_LINES_LO, (lines << 4) & 0xf0) < 0 ||
      xmisp_sensor_write(isp, 0x3630, low ? 0xb8 : 0x70) < 0 ||
      xmisp_sensor_write(isp, 0x3631, low ? 0x82 : 0x8e) < 0 ||
      xmisp_sensor_write(isp, REG_GAIN, g) < 0 ||
      xmisp_sensor_write(isp, REG_DIGITAL, 0) < 0)
    return -1;
  *gain = gain_of(g, 0);
  return 0;
}

#define SC1135(mode, lines, ns)                                                \
  const struct xmisp_sensor xmisp_sensor_sc1135_##mode = {                     \
      .lines_min = 1,                                                          \
      .lines_max = lines,                                                      \
      .line_ns = ns,                                                           \
      .gain_max = 15.5f, /* 0x7f */                                            \
      .awb_r_min = 0.76f,                                                      \
      .awb_r_max = 2.42f,                                                      \
      .awb_b_min = 0.99f,                                                      \
      .awb_b_max = 2.75f,                                                      \
      .get_exposure = get_exposure,                                            \
      .set_exposure = set_exposure,                                            \
  }

SC1135(720p, 746, 53333);
SC1135(960p, 996, 40000);
