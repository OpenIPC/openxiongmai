/* SPDX-License-Identifier: MIT */
/*
 * Tuning by gain, independent of the SoC: a strength from 0 to 1 that
 * rises with the sensor's gain on a log scale, and values interpolated
 * between a low-gain and a high-gain setting by it.
 */
#ifndef XMISP_TUNE_H
#define XMISP_TUNE_H

#include <stdint.h>

/* 0 up to gain_lo, 1 from gain_hi, log-linear between; quantised to
 * 1/steps, so a setting changes only every so often. */
float xmisp_strength(float gain, float gain_lo, float gain_hi, unsigned steps);

/* lo + (hi - lo) x strength, rounded. */
uint32_t xmisp_lerp(uint32_t lo, uint32_t hi, float strength);

#endif
