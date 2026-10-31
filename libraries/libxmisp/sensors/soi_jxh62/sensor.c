// SPDX-License-Identifier: MIT
/*
 * JX-H62 exposure and gain, as measured on the lab XM510 under xmispd:
 * - exposure in lines, register 0x02 (high) and 0x01 (low); the statistics
 *   grow linearly with it;
 * - analogue gain in register 0x00: 2^[6:4] x (1 + [3:0]/16), each step of
 *   bits 6:4 doubling the statistics; 0x7x measured as 0x6x, so the top step
 *   is 6;
 * - the 720p25 script sets 750 lines a frame (registers 0x23/0x22), which VI
 *   counts too (VI_VH_ALL 0x02ee0780: 750 lines of 1920 clocks), at 25 fps:
 *   a line is 1 s / (25 x 750) = 53.33 us.
 */
#include "script.h"
#include "sensors/soi.h"

/* 0x7x measured as 0x6x, so the top step is 6. */
#define COARSE_MAX 6

static int get_exposure(struct xmisp *isp, uint32_t *lines, float *gain) {
  return xmisp_soi_get(isp, COARSE_MAX, lines, gain);
}

static int set_exposure(struct xmisp *isp, uint32_t lines, float *gain) {
  return xmisp_soi_set(isp, COARSE_MAX, lines, gain);
}

/* Calibrated on the lab XM510 against a 24-patch colour checker in
 * daylight (tools/xm-isp-tune/colorchecker.py): native camera RGB through
 * an identity matrix, Cb/Cr gain 1, fitted for the least CIELAB error of
 * the picture as an sRGB display shows it. Mean dE 10.4-10.8 on the chart
 * over three runs; the vendor daemon's matrix and Cb/Cr gain (2.05x) as
 * traced in the same light give 16.9 and clip three patches. */
static const int16_t ccm_h62[9] = {
    375, -118, -1,  /* R */
    -77, 393,  -60, /* G */
    -12, -309, 577, /* B */
};

const struct xmisp_sensor xmisp_sensor_h62 = {
    .lines_min = 1,
    .lines_max = 746, /* a few lines short of the 750-line frame */
    .line_ns = 53333,
    .gain_max = 16.0f, /* 24 dB: noise over brightness, for now */
    /* The range the vendor daemon's AWB searches on this sensor (its
     * candidate gains, traced). Daylight grey on the lab chart is R 1.56,
     * B 2.19. */
    .awb_r_min = 0.75f,
    .awb_r_max = 2.8f,
    .awb_b_min = 1.1f,
    .awb_b_max = 3.6f,
    .ccm = ccm_h62,
    /* The dark level, with the exposure at 1-8 lines and the black level
     * swept (the AE statistics' mean against it, extrapolated to no
     * light): 0x3b at 1x, 0x32 at 16x; the script subtracts 0x45. At 16x a
     * night-like picture's darkest patches came up 3-4 levels (Y 37 to 41
     * on the chart's black), noise unchanged. */
    .black_high = 0x32,
    /* Daylight, the chart's top edges into its dark band and the chair's
     * mesh, KD/KM0/KH0 scaled together from the script's 0x3a/0x28/0x22:
     *   share  edge 10-90%  halo bright/dark  mesh noise
     *   1      1.3 px       14% / 67%         2.9
     *   0.75   1.3 px        9% / 54%         1.9
     *   0.5    1.3 px        2% / 40%         1.2
     *   0      3.3 px        1% / 14%         0.9
     * Half keeps the edges' sharpness and loses most of the halos (the
     * dark outlines around bright edges) and half the noise. */
    .sharpen = 0.5f,
    .get_exposure = get_exposure,
    .set_exposure = set_exposure,
};
