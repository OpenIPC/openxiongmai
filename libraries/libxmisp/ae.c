// SPDX-License-Identifier: MIT
/*
 * Auto-exposure: the product of exposure lines and gain is moved towards
 * target / mean, by the square root of that ratio each step, so a large
 * error closes in a few steps and a small one without overshoot. Lines are
 * used before gain, which adds noise. After a change the next one waits
 * cfg.interval frames, until the change shows in the statistics.
 */
#include <math.h>

#include "ae.h"

#define MEAN_MIN 0.25f /* a black frame still gives a finite ratio */
#define STEP_MAX 8.0f  /* most change in one step, either way */

static float clampf(float v, float lo, float hi) {
  return v < lo ? lo : v > hi ? hi : v;
}

void xmisp_ae_init(struct xmisp_ae *ae, const struct xmisp_ae_cfg *cfg,
                   uint32_t lines, float gain) {
  ae->cfg = *cfg;
  ae->wait = 0;
  xmisp_ae_applied(ae, lines, gain);
}

void xmisp_ae_applied(struct xmisp_ae *ae, uint32_t lines, float gain) {
  ae->total = (float)lines * gain;
}

int xmisp_ae_update(struct xmisp_ae *ae, float mean, uint32_t *lines,
                    float *gain) {
  const struct xmisp_ae_cfg *c = &ae->cfg;
  if (ae->wait) {
    ae->wait--;
    return 0;
  }
  const float ratio = c->target / (mean > MEAN_MIN ? mean : MEAN_MIN);
  if (fabsf(ratio - 1.0f) <= c->tolerance)
    return 0;
  const float lo = (float)c->lines_min, hi = (float)c->lines_max * c->gain_max;
  const float total = clampf(
      ae->total * clampf(sqrtf(ratio), 1.0f / STEP_MAX, STEP_MAX), lo, hi);
  if (total == ae->total)
    return 0; /* at a limit: too dark or too bright for the range */
  uint32_t l =
      total < (float)c->lines_max ? (uint32_t)(total + 0.5f) : c->lines_max;
  if (l < c->lines_min)
    l = c->lines_min;
  *lines = l;
  *gain = clampf(total / (float)l, 1.0f, c->gain_max);
  ae->total = total;
  ae->wait = c->interval;
  return 1;
}
