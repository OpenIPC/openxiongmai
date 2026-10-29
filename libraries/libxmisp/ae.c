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
 * Under anti-flicker the lines are whole flicker periods once the light
 * allows one, the rest made up by gain; in brighter light, shorter than a
 * period, no exposure can avoid the flicker, and lines are free.
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

/* The most lines the exposure may have: under anti-flicker, whole periods
 * within the sensor's range. */
static float lines_top(const struct xmisp_ae_cfg *c) {
  if (c->flicker_lines >= 1.0f && (float)c->lines_max >= c->flicker_lines)
    return floorf((float)c->lines_max / c->flicker_lines) * c->flicker_lines;
  return (float)c->lines_max;
}

/* The lines for a total of lines x gain: whole lines, the fraction left
 * made up by gain (1.5 lines of light is 1 line at 1.5x, not 2 lines);
 * under anti-flicker whole periods, once there is light for one. */
static uint32_t lines_for(const struct xmisp_ae_cfg *c, float total) {
  const float top = lines_top(c);
  float l = total < top ? total : top;
  if (c->flicker_lines >= 1.0f && l >= c->flicker_lines)
    l = floorf(l / c->flicker_lines) * c->flicker_lines + 0.5f;
  uint32_t n = (uint32_t)l;
  if (n > c->lines_max)
    n = c->lines_max;
  return n < c->lines_min ? c->lines_min : n;
}

void xmisp_ae_flicker(struct xmisp_ae *ae, float flicker_lines) {
  ae->cfg.flicker_lines = flicker_lines;
  ae->replan = 1;
  ae->wait = 0;
}

void xmisp_ae_init(struct xmisp_ae *ae, const struct xmisp_ae_cfg *cfg,
                   uint32_t lines, float gain) {
  ae->cfg = *cfg;
  ae->replan = cfg->flicker_lines > 0;
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
  const int replan = ae->replan;
  ae->replan = 0;
  if (fabsf(ratio - 1.0f) <= c->tolerance && !replan)
    return 0;
  const float lo = (float)c->lines_min, hi = lines_top(c) * c->gain_max;
  const float step = fabsf(ratio - 1.0f) <= c->tolerance
                         ? 1.0f /* re-planning only: the same light */
                         : clampf(sqrtf(ratio), 1.0f / STEP_MAX, STEP_MAX);
  const float total = clampf(ae->total * step, lo, hi);
  if (total == ae->total && !replan)
    return 0; /* at a limit: too dark or too bright for the range */
  const uint32_t l = lines_for(c, total);
  const float g = clampf(total / (float)l, 1.0f, c->gain_max);
  if (l == ae->lines && fabsf(g / ae->gain - 1.0f) < GAIN_SAME)
    return 0; /* rounds to what is set */
  *lines = l;
  *gain = g;
  return 1;
}
