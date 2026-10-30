// SPDX-License-Identifier: MIT
/*
 * SC2235 exposure and gain (../smartsens.h), from the XM510 vendor daemon's
 * 3A as traced in qemu-bench; not yet run on hardware. The 1080p script
 * leaves the frame length at the chip's; the vendor's exposure goes to 1121
 * lines, four short of a frame as on the other SmartSens modes, so 1125
 * lines, at the 25 fps the daemon logs: a line is 35.56 us. The white
 * balance range is the one the vendor's AWB searches under emulation.
 */
#include "script.h"
#include "sensors/smartsens.h"

/* 0x3631, 0x3301, 0x366f. The last was seen at 15.5x and not at 11.5x. */
static const struct xmisp_ss_range analog[] = {
    {0.0f, {0x3631, 0x3301, 0x366f}, {0x84, 0x05, 0x2f}},
    {2.0f, {0x3631, 0x3301, 0x366f}, {0x88, 0x15, 0x2f}},
    {8.0f, {0x3631, 0x3301, 0x366f}, {0x88, 0x19, 0x2f}},
    {15.5f, {0x3631, 0x3301, 0x366f}, {0x88, 0xff, 0x3c}},
};

static const struct xmisp_ss ss = {3, analog, 4};

static int get_exposure(struct xmisp *isp, uint32_t *lines, float *gain) {
  return xmisp_ss_get(isp, lines, gain);
}
static int set_exposure(struct xmisp *isp, uint32_t lines, float *gain) {
  return xmisp_ss_set(isp, &ss, lines, gain);
}

const struct xmisp_sensor xmisp_sensor_sc2235 = {
    .lines_min = 1,
    .lines_max = 1121,
    .line_ns = 35556,
    .gain_max = 15.5f, /* 8x coarse at the top fine step */
    .awb_r_min = 0.66f,
    .awb_r_max = 2.2f,
    .awb_b_min = 0.87f,
    .awb_b_max = 2.52f,
    .get_exposure = get_exposure,
    .set_exposure = set_exposure,
};
