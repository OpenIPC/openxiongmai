// SPDX-License-Identifier: MIT
/*
 * OV9750 exposure and gain, from the XM510 vendor daemon's 3A as traced in
 * qemu-bench; not yet run on hardware:
 * - exposure in 1/16 lines, registers 0x3500 (bits 19:16 of lines x 16),
 *   0x3501 and 0x3502;
 * - analogue gain linear in 1/128ths, 0x3508 (bits 10:8) and 0x3509: the
 *   vendor's dark ramp goes 0x86, 0xb8, 0xfd, 0x15b, 0x1de, 0x291 ... 0x7ff,
 *   1.37x a step, as it steps the measured JX-H62's and AR0130's gain. It
 *   took the digital gains (0x3532-0x3537) only past 0x7ff, and they are
 *   left at the script's.
 * The 720p script sets 988 lines a frame (0x380e/0x380f = 0x3dc) and the
 * vendor's exposure goes to 984; at 25 fps a line is 40.49 us. The white
 * balance range is the one the vendor's AWB searches under emulation.
 */
#include <math.h>

#include "script.h"
#include "xmisp_priv.h"

#define GAIN_ONE 0x80
#define GAIN_MAX 0x7ff

static int get_exposure(struct xmisp *isp, uint32_t *lines, float *gain) {
  uint32_t e0, e1, e2, g0, g1;
  if (xmisp_sensor_read(isp, 0x3500, &e0) < 0 ||
      xmisp_sensor_read(isp, 0x3501, &e1) < 0 ||
      xmisp_sensor_read(isp, 0x3502, &e2) < 0 ||
      xmisp_sensor_read(isp, 0x3508, &g0) < 0 ||
      xmisp_sensor_read(isp, 0x3509, &g1) < 0)
    return -1;
  *lines = ((e0 & 0xf) << 16 | e1 << 8 | e2) >> 4;
  uint32_t g = (g0 & 7) << 8 | g1;
  *gain = (float)(g < GAIN_ONE ? GAIN_ONE : g) / (float)GAIN_ONE;
  return 0;
}

static int set_exposure(struct xmisp *isp, uint32_t lines, float *gain) {
  int g = (int)lroundf(*gain * (float)GAIN_ONE);
  g = g < GAIN_ONE ? GAIN_ONE : g > GAIN_MAX ? GAIN_MAX : g;
  const uint32_t e = lines << 4;
  if (xmisp_sensor_write(isp, 0x3502, e & 0xf0) < 0 ||
      xmisp_sensor_write(isp, 0x3501, (e >> 8) & 0xff) < 0 ||
      xmisp_sensor_write(isp, 0x3500, (e >> 16) & 0xf) < 0 ||
      xmisp_sensor_write(isp, 0x3509, (uint32_t)g & 0xff) < 0 ||
      xmisp_sensor_write(isp, 0x3508, (uint32_t)g >> 8) < 0)
    return -1;
  *gain = (float)g / (float)GAIN_ONE;
  return 0;
}

const struct xmisp_sensor xmisp_sensor_ov9750 = {
    .lines_min = 1,
    .lines_max = 984,
    .line_ns = 40486,
    .gain_max = (float)GAIN_MAX / GAIN_ONE,
    .awb_r_min = 0.81f,
    .awb_r_max = 2.71f,
    .awb_b_min = 1.08f,
    .awb_b_max = 3.9f,
    .get_exposure = get_exposure,
    .set_exposure = set_exposure,
};
