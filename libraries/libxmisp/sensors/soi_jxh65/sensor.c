// SPDX-License-Identifier: MIT
/*
 * JX-H65 exposure and gain: the JX-H62's registers and encoding (../soi.h).
 * The XM510 vendor daemon's 3A, traced in qemu-bench, writes them with the
 * same codes for the same steps as on the JX-H62, to 0x40 (16x), and sets
 * register 0x1f to 0x10 below 4x (0x1c and 0x1d seen) and to 0 from 4x
 * (0x23, 0x24), before the gain; not yet run on hardware.
 * 720p: 750 lines a frame (0x23/0x22 = 0x2ee), exposure to 746; 960p: 1000
 * (0x3e8), exposure to 996; at 25 fps. The white balance range is the one
 * the vendor's AWB searches under emulation, B to 4.95, cut to 3.9 to fit
 * the gain registers (0xfff) at the scripts' G gain.
 */
#include "script.h"
#include "sensors/soi.h"
#include "xmisp_priv.h"

#define COARSE_MAX 4 /* 0x40, 16x */
#define REG_LOW_GAIN 0x1f

static int get_exposure(struct xmisp *isp, uint32_t *lines, float *gain) {
  return xmisp_soi_get(isp, COARSE_MAX, lines, gain);
}

static int set_exposure(struct xmisp *isp, uint32_t lines, float *gain) {
  const uint32_t g = xmisp_soi_gain_reg(*gain, COARSE_MAX);
  if (xmisp_sensor_write(isp, REG_LOW_GAIN, g < 0x20 ? 0x10 : 0x00) < 0)
    return -1;
  return xmisp_soi_set(isp, COARSE_MAX, lines, gain);
}

#define H65(mode, lines, ns)                                                   \
  const struct xmisp_sensor xmisp_sensor_h65_##mode = {                        \
      .lines_min = 1,                                                          \
      .lines_max = lines,                                                      \
      .line_ns = ns,                                                           \
      .gain_max = 16.0f,                                                       \
      .awb_r_min = 0.72f,                                                      \
      .awb_r_max = 3.47f,                                                      \
      .awb_b_min = 1.23f,                                                      \
      .awb_b_max = 3.9f,                                                      \
      .get_exposure = get_exposure,                                            \
      .set_exposure = set_exposure,                                            \
  }

H65(720p, 746, 53333);
H65(960p, 996, 40000);
