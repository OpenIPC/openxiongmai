// SPDX-License-Identifier: MIT
/*
 * SC1235 exposure and gain (../smartsens.h), from the XM510 vendor daemon's
 * 3A as traced in qemu-bench; not yet run on hardware.
 * - 720p: 750 lines a frame (0x320e/0x320f = 0x2ee), and the vendor's
 *   exposure goes to 746 lines; at 25 fps a line is 53.33 us.
 * - 960p: the script leaves the frame length at the chip's, and the
 *   vendor's exposure goes to 996 lines; 1000 lines at 25 fps, 40 us a
 *   line, as SC1135's and JX-H65's 960p set.
 * The analogue settings differ between the modes. The white balance range
 * is the one the vendor's AWB searches under emulation, on a test picture
 * (on the JX-H62 and AR0130 that came out wider than on the hardware).
 */
#include "script.h"
#include "sensors/smartsens.h"

/* 0x3631, 0x3301, 0x3633 */
static const struct xmisp_ss_range analog_720p[] = {
    {0.0f, {0x3631, 0x3301, 0x3633}, {0x84, 0x04, 0x2f}},
    {2.0f, {0x3631, 0x3301, 0x3633}, {0x88, 0x10, 0x23}},
    {4.0f, {0x3631, 0x3301, 0x3633}, {0x88, 0xb0, 0x43}},
};
static const struct xmisp_ss_range analog_960p[] = {
    {0.0f, {0x3631, 0x3301, 0x3633}, {0x84, 0x05, 0x2f}},
    {2.0f, {0x3631, 0x3301, 0x3633}, {0x88, 0x1f, 0x23}},
    {4.0f, {0x3631, 0x3301, 0x3633}, {0x88, 0xff, 0x43}},
};
/* Both modes: 0x04, 0x18 at 9x; 0x02, 0x08 at 12.5x (720p) and 13x
 * (960p). */
static const struct xmisp_ss_range post[] = {
    {0.0f, {0x5781, 0x5785}, {0x04, 0x18}},
    {12.5f, {0x5781, 0x5785}, {0x02, 0x08}},
};

static const struct xmisp_ss ss_720p = {3, analog_720p, 3, post, 2};
static const struct xmisp_ss ss_960p = {3, analog_960p, 3, post, 2};

static int get_exposure(struct xmisp *isp, uint32_t *lines, float *gain) {
  return xmisp_ss_get(isp, lines, gain);
}
static int set_720p(struct xmisp *isp, uint32_t lines, float *gain) {
  return xmisp_ss_set(isp, &ss_720p, lines, gain);
}
static int set_960p(struct xmisp *isp, uint32_t lines, float *gain) {
  return xmisp_ss_set(isp, &ss_960p, lines, gain);
}

const struct xmisp_sensor xmisp_sensor_sc1235_720p = {
    .lines_min = 1,
    .lines_max = 746,
    .line_ns = 53333,
    .gain_max = 15.5f, /* 8x coarse at the top fine step */
    .awb_r_min = 0.65f,
    .awb_r_max = 2.3f,
    .awb_b_min = 0.92f,
    .awb_b_max = 2.66f,
    .get_exposure = get_exposure,
    .set_exposure = set_720p,
};

const struct xmisp_sensor xmisp_sensor_sc1235_960p = {
    .lines_min = 1,
    .lines_max = 996,
    .line_ns = 40000,
    .gain_max = 15.5f,
    .awb_r_min = 0.65f,
    .awb_r_max = 2.3f,
    .awb_b_min = 0.92f,
    .awb_b_max = 2.66f,
    .get_exposure = get_exposure,
    .set_exposure = set_960p,
};
