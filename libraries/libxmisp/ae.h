/* SPDX-License-Identifier: MIT */
/*
 * Auto-exposure, independent of the SoC and the sensor: it takes the mean
 * luma of a frame and gives the exposure (lines) and gain to aim for. The
 * SoC side measures, the sensor side encodes.
 */
#ifndef XMISP_AE_H
#define XMISP_AE_H

#include <stdint.h>

struct xmisp_ae_cfg {
  float target;       /* mean luma to hold, 8-bit, linear (before gamma) */
  float tolerance;    /* relative error left alone, e.g. 0.08 */
  uint32_t lines_min; /* exposure range of the sensor mode */
  uint32_t lines_max;
  float gain_max;    /* 1.0 = no gain */
  unsigned interval; /* frames from a change until its result is measured */
  /* Anti-flicker: lines in one period of the light's flicker (half the
   * mains period), 0 for none. The exposure is whole periods, at least
   * one, and gain makes up the rest; light too bright for one period is
   * overexposed, not banded. */
  float flicker_lines;
};

struct xmisp_ae {
  struct xmisp_ae_cfg cfg;
  uint32_t lines; /* the exposure the sensor has */
  float gain;
  float total; /* lines x gain */
  unsigned wait;
  int replan; /* the exposure rules changed: re-plan even within tolerance */
};

void xmisp_ae_init(struct xmisp_ae *ae, const struct xmisp_ae_cfg *cfg,
                   uint32_t lines, float gain);

/* One frame's mean luma. 1 if the exposure should change, with *lines and
 * *gain set; nothing changes until xmisp_ae_applied() says what the sensor
 * took (after a failed write, what it reads back). */
int xmisp_ae_update(struct xmisp_ae *ae, float mean, uint32_t *lines,
                    float *gain);

/* Anti-flicker on (lines in a flicker period) or off (0), from the next
 * update. */
void xmisp_ae_flicker(struct xmisp_ae *ae, float flicker_lines);

/* The exposure the sensor took, after its gain steps. */
void xmisp_ae_applied(struct xmisp_ae *ae, uint32_t lines, float gain);

#endif
