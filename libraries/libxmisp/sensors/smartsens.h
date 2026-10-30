/* SPDX-License-Identifier: MIT */
/*
 * Exposure and gain of the SmartSens SC1235, SC2145H and SC2235, as the
 * XM510 vendor daemon's 3A writes them (traced in qemu-bench; no hardware
 * measurement yet):
 * - exposure in 1/16 lines, register 0x3e01 (bits 11:4 of the lines) and
 *   0x3e02 (bits 3:0, shifted up 4);
 * - gain as a coarse doubling in register 0x3e08 (0x03 1x, 0x07 2x, 0x0f
 *   4x, 0x1f 8x, 0x3f, 0x7f) times register 0x3e09 / 16 (0x10-0x1f). With
 *   the picture dark, the vendor 3A steps its gain by about 1.37x a frame
 *   pair through the same gains on every sensor whose gain was measured
 *   (JX-H62, AR0130), and these codes are those gains exactly: 0x03/0x16
 *   1.375x, 0x07/0x14 2.5x, 0x0f/0x13 4.75x, 0x1f/0x19 12.5x;
 * - analogue settings by gain range, written between 0x3812 = 0x00 and
 *   0x3812 = 0x30 before the gain, and 0x3903 = 0x84, 0x04 before each
 *   update, as the vendor does. Which settings a gain gets is known only at
 *   the gains the vendor's ramp passed through: where they changed with
 *   the coarse step, a range starts at that step; elsewhere at the lowest
 *   of those gains that showed the change.
 */
#ifndef XMISP_SENSORS_SMARTSENS_H
#define XMISP_SENSORS_SMARTSENS_H

#include <stdint.h>

struct xmisp;

#define XMISP_SS_WRITES 6

/* Register writes for gains from `from` up to the next range's. */
struct xmisp_ss_range {
  float from;
  uint16_t reg[XMISP_SS_WRITES]; /* 0 ends the list */
  uint8_t val[XMISP_SS_WRITES];
};

struct xmisp_ss {
  unsigned coarse_max; /* the top 0x3e08 step: 3 is 8x (0x1f) */
  /* Between 0x3812 = 0 and 0x30, before the gain, by gain; in order of
   * from, the first from 0. */
  const struct xmisp_ss_range *analog;
  unsigned n_analog;
  /* After the gain, likewise; n_post 0 for none. */
  const struct xmisp_ss_range *post;
  unsigned n_post;
};

/* The gain of 0x3e08 and 0x3e09's values. */
float xmisp_ss_gain(uint32_t coarse, uint32_t fine);
/* The step nearest the gain asked for, as 0x3e08 and 0x3e09's values. */
void xmisp_ss_gain_regs(float gain, unsigned coarse_max, uint32_t *coarse,
                        uint32_t *fine);

int xmisp_ss_get(struct xmisp *isp, uint32_t *lines, float *gain);
/* Writes lines, the analogue settings and the gain step nearest *gain,
 * which comes back as that step. 0 or -1. */
int xmisp_ss_set(struct xmisp *isp, const struct xmisp_ss *ss, uint32_t lines,
                 float *gain);

#endif
