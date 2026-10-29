// SPDX-License-Identifier: MIT
/*
 * Auto white balance: Cr follows the red gain and Cb the blue one, so each
 * step scales R by exp(-Cr / step) and B by exp(-Cb / step), within the
 * sensor's range. The window stays wide until both are near zero, so a
 * start far off still finds grey pixels, then narrows so that only grey
 * ones count; if the error grows again, or the grey pixels go, it widens. As
 * with AE, only xmisp_awb_applied() changes the gains, once they are set.
 */
#include <math.h>

#include "awb.h"

static float clampf(float v, float lo, float hi) {
  return v < lo ? lo : v > hi ? hi : v;
}

void xmisp_awb_init(struct xmisp_awb *awb, const struct xmisp_awb_cfg *cfg,
                    float r, float b) {
  awb->cfg = *cfg;
  awb->r = clampf(r, cfg->r_min, cfg->r_max);
  awb->b = clampf(b, cfg->b_min, cfg->b_max);
  awb->window = XMISP_AWB_WIDE;
  awb->wait = 0;
}

int xmisp_awb_update(struct xmisp_awb *awb, uint32_t points, float cb, float cr,
                     float *r, float *b) {
  const struct xmisp_awb_cfg *c = &awb->cfg;
  if (awb->wait) {
    awb->wait--;
    return 0;
  }
  if (points < c->points_min) {
    /* Nothing grey enough to go by. In the narrow window that may be a
     * change of light: look wide before holding. */
    if (awb->window == XMISP_AWB_NARROW) {
      awb->window = XMISP_AWB_WIDE;
      awb->wait = c->interval;
    }
    return 0;
  }
  const int near = fabsf(cb) < c->near && fabsf(cr) < c->near;
  const enum xmisp_awb_window window = near ? XMISP_AWB_NARROW : XMISP_AWB_WIDE;
  if (window != awb->window) {
    /* The next measurement is of other pixels: take it before moving. */
    awb->window = window;
    awb->wait = c->interval;
    return 0;
  }
  if (fabsf(cb) <= c->tolerance && fabsf(cr) <= c->tolerance)
    return 0;
  const float nr = clampf(awb->r * expf(-cr / c->step), c->r_min, c->r_max);
  const float nb = clampf(awb->b * expf(-cb / c->step), c->b_min, c->b_max);
  if (nr == awb->r && nb == awb->b)
    return 0; /* at the edge of the range */
  *r = nr;
  *b = nb;
  return 1;
}

void xmisp_awb_applied(struct xmisp_awb *awb, float r, float b) {
  awb->r = r;
  awb->b = b;
  awb->wait = awb->cfg.interval;
}
