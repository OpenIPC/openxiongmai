// SPDX-License-Identifier: MIT
/*
 * JX-H42 exposure and gain: the JX-H62's registers and encoding (../soi.h).
 * The XM510 vendor daemon's 3A, traced in qemu-bench, writes them with the
 * same codes for the same steps as on the JX-H62, to 0x4f (31x) and 748
 * lines; not yet run on hardware. The 720p script sets 750 lines a frame
 * (0x23/0x22 = 0x2ee), at the daemon's 25 fps a line is 53.33 us. The white
 * balance range is the one the vendor's AWB searches under emulation.
 */
#include "script.h"
#include "sensors/soi.h"

#define COARSE_MAX 4 /* the vendor's top step, 0x4x */

static int get_exposure(struct xmisp *isp, uint32_t *lines, float *gain) {
  return xmisp_soi_get(isp, COARSE_MAX, lines, gain);
}

static int set_exposure(struct xmisp *isp, uint32_t lines, float *gain) {
  return xmisp_soi_set(isp, COARSE_MAX, lines, gain);
}

const struct xmisp_sensor xmisp_sensor_h42 = {
    .lines_min = 1,
    .lines_max = 748,
    .line_ns = 53333,
    .gain_max = 16.0f, /* as the JX-H62's */
    .awb_r_min = 0.72f,
    .awb_r_max = 3.31f,
    .awb_b_min = 1.17f,
    .awb_b_max = 3.67f,
    .get_exposure = get_exposure,
    .set_exposure = set_exposure,
};
