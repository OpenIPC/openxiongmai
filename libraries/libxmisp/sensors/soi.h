/* SPDX-License-Identifier: MIT */
/*
 * The SOI JX-H sensors' exposure and gain, measured on the JX-H62 (see
 * soi_jxh62/sensor.c) and shared by the JX-H42 and JX-H65, whose vendor
 * 3A writes the same registers with the same codes for the same gains:
 * - exposure in lines, register 0x02 (high) and 0x01 (low);
 * - gain in register 0x00, 2^[6:4] x (1 + [3:0]/16), up to the coarse
 *   step coarse_max.
 */
#ifndef XMISP_SENSORS_SOI_H
#define XMISP_SENSORS_SOI_H

#include <stdint.h>

struct xmisp;

/* The gain of register 0x00's value. */
float xmisp_soi_gain(uint32_t reg, unsigned coarse_max);
/* The step nearest the gain asked for. */
uint32_t xmisp_soi_gain_reg(float gain, unsigned coarse_max);

int xmisp_soi_get(struct xmisp *isp, unsigned coarse_max, uint32_t *lines,
                  float *gain);
/* Writes lines and the gain step nearest *gain, which comes back as that
 * step. 0 or -1. */
int xmisp_soi_set(struct xmisp *isp, unsigned coarse_max, uint32_t lines,
                  float *gain);

#endif
