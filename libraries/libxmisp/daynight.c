// SPDX-License-Identifier: MIT
/* Day and night from the exposure (daynight.h). */
#include "daynight.h"

/* AE has run out: the gain this close to its ceiling and the picture this
 * far below the target. */
#define OUT_OF_GAIN 0.97f
#define BELOW_TARGET 0.8f

void xmisp_dn_init(struct xmisp_dn *dn, const struct xmisp_dn_cfg *cfg,
                   int night) {
  dn->cfg = *cfg;
  dn->night = night;
  dn->waiting = 0;
  dn->since_ms = 0;
}

int xmisp_dn_update(struct xmisp_dn *dn, uint32_t now_ms, float gain,
                    float gain_max, float mean, float target) {
  int other;
  if (dn->night)
    other = gain <= dn->cfg.day_gain;
  else if (dn->cfg.night_gain > 0)
    other = gain >= dn->cfg.night_gain;
  else
    other = gain >= gain_max * OUT_OF_GAIN && mean < target * BELOW_TARGET;
  if (!other) {
    dn->waiting = 0;
    return 0;
  }
  if (!dn->waiting) {
    dn->waiting = 1;
    dn->since_ms = now_ms;
  }
  if (now_ms - dn->since_ms < (dn->night ? dn->cfg.day_ms : dn->cfg.night_ms))
    return 0;
  dn->night = !dn->night;
  dn->waiting = 0;
  return 1;
}
