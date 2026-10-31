/* SPDX-License-Identifier: MIT */
/*
 * Day and night, independent of the SoC and the sensor: night when the
 * picture stays dark with the exposure used up, day when the gain stays
 * low, each for a while. The meaning follows majestic's automatic night
 * mode (nightMode.lightMonitor), whose keys set it.
 */
#ifndef XMISP_DAYNIGHT_H
#define XMISP_DAYNIGHT_H

#include <stdint.h>

struct xmisp_dn_cfg {
  /* Night when the gain reaches this; 0: when AE has run out (the gain at
   * its ceiling and the picture still below its target). */
  float night_gain;
  float day_gain;    /* day when the gain is at or below this */
  uint32_t night_ms; /* how long it must stay dark for night */
  uint32_t day_ms;   /* how long it must stay bright for day */
};

struct xmisp_dn {
  struct xmisp_dn_cfg cfg;
  int night;
  int waiting;       /* the other mode's condition holds */
  uint32_t since_ms; /* since when */
};

void xmisp_dn_init(struct xmisp_dn *dn, const struct xmisp_dn_cfg *cfg,
                   int night);

/* One frame: the time (ms, any origin, wrapping), the gain AE has set and
 * its ceiling, the picture's mean and AE's target. 1 when the mode changes
 * (dn->night says which), else 0. */
int xmisp_dn_update(struct xmisp_dn *dn, uint32_t now_ms, float gain,
                    float gain_max, float mean, float target);

#endif
