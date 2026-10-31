// SPDX-License-Identifier: MIT
/*
 * SC2145H exposure and gain (../smartsens.h), from the XM510 vendor daemon's
 * 3A as traced in qemu-bench; not yet run on hardware. The 720p script sets
 * 750 lines a frame (0x320e/0x320f = 0x2ee), the daemon logs 25 fps, and
 * the vendor's exposure goes to 746 lines: a line is 53.33 us. The white
 * balance range is the one the vendor's AWB searches under emulation.
 */
#include "script.h"
#include "sensors/smartsens.h"

/* 0x3633, 0x3301, 0x3631, 0x3620 */
static const struct xmisp_ss_range analog[] = {
    {0.0f, {0x3633, 0x3301, 0x3631, 0x3620}, {0x57, 0x04, 0x84, 0x28}},
    {2.0f, {0x3633, 0x3301, 0x3631, 0x3620}, {0x43, 0x08, 0x86, 0x08}},
    {4.0f, {0x3633, 0x3301, 0x3631, 0x3620}, {0x53, 0x09, 0x86, 0x08}},
    {8.0f, {0x3633, 0x3301, 0x3631, 0x3620}, {0x53, 0x20, 0x86, 0x08}},
};

static const struct xmisp_ss ss = {3, analog, 4};

static int get_exposure(struct xmisp *isp, uint32_t *lines, float *gain) {
  return xmisp_ss_get(isp, lines, gain);
}
static int set_exposure(struct xmisp *isp, uint32_t lines, float *gain) {
  return xmisp_ss_set(isp, &ss, lines, gain);
}

const struct xmisp_sensor xmisp_sensor_sc2145h = {
    .lines_min = 1,
    .lines_max = 746,
    .line_ns = 53333,
    .gain_max = 15.5f, /* 8x coarse at the top fine step */
    .awb_r_min = 0.67f,
    .awb_r_max = 2.16f,
    .awb_b_min = 0.88f,
    .awb_b_max = 2.45f,
    .get_exposure = get_exposure,
    .set_exposure = set_exposure,
};
