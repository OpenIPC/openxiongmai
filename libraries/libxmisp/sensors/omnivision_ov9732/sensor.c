// SPDX-License-Identifier: MIT
/*
 * OV9732 exposure and gain, from the XM510 vendor daemon's 3A as traced in
 * qemu-bench; not yet run on hardware:
 * - exposure in 1/16 lines, registers 0x3501 and 0x3502;
 * - analogue gain linear in 1/16ths, 0x350b: the vendor's dark ramp goes
 *   0x16, 0x1e, 0x28, 0x38, 0x4c, 0x68, 0x90, 0xc8, 0xf8, the gains it steps
 *   the measured JX-H62's and AR0130's through (1.375x ... 15.5x), written
 *   in a group hold (0x3208 = 0x00, then 0x10 and 0xa0 to launch it). It
 *   took the digital gains (0x5180-0x5185) only past 0xf8, and they are
 *   left at the script's.
 * The 720p script sets 750 lines a frame (0x380e/0x380f = 0x2ee) and the
 * vendor's exposure goes to 748; at 25 fps a line is 53.33 us. The white
 * balance range is the one the vendor's AWB searches under emulation.
 */
#include <math.h>

#include "script.h"
#include "xmisp_priv.h"

#define GAIN_ONE 0x10
#define GAIN_MAX 0xf8 /* the vendor's analogue top */

static int get_exposure(struct xmisp *isp, uint32_t *lines, float *gain) {
  uint32_t e1, e2, g;
  if (xmisp_sensor_read(isp, 0x3501, &e1) < 0 ||
      xmisp_sensor_read(isp, 0x3502, &e2) < 0 ||
      xmisp_sensor_read(isp, 0x350b, &g) < 0)
    return -1;
  *lines = (e1 << 8 | e2) >> 4;
  *gain = (float)(g < GAIN_ONE ? GAIN_ONE : g) / (float)GAIN_ONE;
  return 0;
}

static int set_exposure(struct xmisp *isp, uint32_t lines, float *gain) {
  int g = (int)lroundf(*gain * (float)GAIN_ONE);
  g = g < GAIN_ONE ? GAIN_ONE : g > GAIN_MAX ? GAIN_MAX : g;
  const uint32_t e = lines << 4;
  if (xmisp_sensor_write(isp, 0x3501, (e >> 8) & 0xff) < 0 ||
      xmisp_sensor_write(isp, 0x3502, e & 0xf0) < 0 ||
      xmisp_sensor_write(isp, 0x3208, 0x00) < 0 ||
      xmisp_sensor_write(isp, 0x350b, (uint32_t)g) < 0 ||
      xmisp_sensor_write(isp, 0x3208, 0x10) < 0 ||
      xmisp_sensor_write(isp, 0x3208, 0xa0) < 0)
    return -1;
  *gain = (float)g / (float)GAIN_ONE;
  return 0;
}

const struct xmisp_sensor xmisp_sensor_ov9732 = {
    .lines_min = 1,
    .lines_max = 748,
    .line_ns = 53333,
    .gain_max = (float)GAIN_MAX / GAIN_ONE,
    .awb_r_min = 0.75f,
    .awb_r_max = 2.34f,
    .awb_b_min = 1.06f,
    .awb_b_max = 3.29f,
    .get_exposure = get_exposure,
    .set_exposure = set_exposure,
};
