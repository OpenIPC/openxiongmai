// SPDX-License-Identifier: MIT
/*
 * Auto-exposure: the product of exposure lines and gain is moved towards
 * target / mean, by the square root of that ratio each step, so a large
 * error closes in a few steps and a small one without overshoot. Lines are
 * used before gain, which adds noise. After a change the next one waits
 * cfg.interval frames, until the change shows in the statistics. Lines
 * are whole; the fraction a total leaves goes to gain, the sensor taking
 * its nearest step.
 *
 * Only xmisp_ae_applied() changes the state, with what the sensor took: a
 * write that failed leaves the controller where the sensor is. A proposal
 * that rounds to the exposure already set is no change.
 */
#include <math.h>

#include "ae.h"

#define MEAN_MIN 0.25f        /* a black frame still gives a finite ratio */
#define STEP_MAX 8.0f         /* most change in one step, either way */
#define GAIN_SAME (1.0f / 32) /* below the finest gain step of a sensor */

static float clampf(float v, float lo, float hi) {
  return v < lo ? lo : v > hi ? hi : v;
}

void xmisp_ae_init(struct xmisp_ae *ae, const struct xmisp_ae_cfg *cfg,
                   uint32_t lines, float gain) {
  ae->cfg = *cfg;
  ae->lines = lines;
  ae->gain = gain;
  ae->total = (float)lines * gain;
  ae->wait = 0;
}

void xmisp_ae_applied(struct xmisp_ae *ae, uint32_t lines, float gain) {
  ae->lines = lines;
  ae->gain = gain;
  ae->total = (float)lines * gain;
  ae->wait = ae->cfg.interval;
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
  /* Whole lines, the fraction left made up by gain: 1.5 lines of light is
   * 1 line at 1.5x, not 2 lines. */
  uint32_t l = total < (float)c->lines_max ? (uint32_t)total : c->lines_max;
  if (l < c->lines_min)
    l = c->lines_min;
  const float g = clampf(total / (float)l, 1.0f, c->gain_max);
  if (l == ae->lines && fabsf(g / ae->gain - 1.0f) < GAIN_SAME)
    return 0; /* rounds to what is set */
  *lines = l;
  *gain = g;
  return 1;
}
