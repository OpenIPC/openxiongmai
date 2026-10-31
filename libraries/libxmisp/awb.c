// SPDX-License-Identifier: MIT
/*
 * Auto white balance: Cr follows the red gain and Cb the blue one, so each
 * step scales R by exp(-Cr / step) and B by exp(-Cb / step), within the
 * sensor's range. The window stays wide until both are near zero, so a
 * start far off still finds grey pixels, then narrows so that only grey
 * ones count; if the error grows again, or the grey pixels go, it widens. As
 * with AE, only xmisp_awb_applied() changes the gains, once they are set.
 *
 * Grey-world can settle where some surfaces look grey under the wrong gains:
 * in the lab room, after a start in the dark, at far too blue gains where its
 * warm surfaces do. So where to start it from is found by a search: a grid
 * of gains over the sensor's range, three candidates a round in the spare
 * statistics channels, counted in the narrow window; the one with the most
 * grey pixels wins if it clearly beats the gains set. On bright pixels the
 * real grey has the most (JX-H62 under 4000 K bulbs, luma 0xa0: 2143 grey
 * pixels at the neutral, 413 at the warm surfaces' gains). It runs at
 * start, every SEARCH_EVERY frames, when the grey pixels go, and when the
 * wide window's mean looks grey but the narrow window finds none, the gains
 * between two populations, where grey-world alone stays put.
 */
#include <math.h>

#include "awb.h"

#define SEARCH_EVERY 250 /* frames: 10 s at 25 fps */
#define SEARCH_GAP 50    /* the least frames between searches */
#define SEARCH_WIN 1.25f /* how much more grey a candidate needs to win */
#define SEARCH_SIDE 5    /* a grid of 5 x 5 in R and B, log-spaced */
#define STARVED 2        /* narrow windows in a row without grey pixels */

static float clampf(float v, float lo, float hi) {
  return v < lo ? lo : v > hi ? hi : v;
}

/* The grid, log-spaced: over the H62's range a step is 1.39x in R and 1.34x
 * in B, about what the narrow window (some 1.2x either side of grey)
 * covers. */
static void search_points(struct xmisp_awb *awb) {
  const struct xmisp_awb_cfg *c = &awb->cfg;
  awb->n_search = 0;
  if (!(c->r_min > 0 && c->r_max > c->r_min && c->b_min > 0 &&
        c->b_max > c->b_min))
    return;
  const float lr = logf(c->r_max / c->r_min), lb = logf(c->b_max / c->b_min);
  for (unsigned i = 0; i < SEARCH_SIDE; i++)
    for (unsigned j = 0; j < SEARCH_SIDE; j++) {
      const unsigned k = i * SEARCH_SIDE + j;
      awb->sr[k] = c->r_min * expf(lr * (float)i / (SEARCH_SIDE - 1));
      awb->sb[k] = c->b_min * expf(lb * (float)j / (SEARCH_SIDE - 1));
    }
  awb->n_search = SEARCH_SIDE * SEARCH_SIDE;
}

static void next_round(struct xmisp_awb *awb) {
  awb->n_cand = 0;
  while (awb->n_cand < XMISP_AWB_CHANNELS - 1 && awb->next < awb->n_search)
    awb->cand[awb->n_cand++] = awb->next++;
  awb->wait = awb->cfg.interval;
}

static void search_start(struct xmisp_awb *awb) {
  awb->searching = 1;
  awb->next = 0;
  awb->best = -1;
  awb->best_points = 0;
  awb->window = XMISP_AWB_NARROW; /* the counts are of near-grey pixels */
  next_round(awb);
}

void xmisp_awb_init(struct xmisp_awb *awb, const struct xmisp_awb_cfg *cfg,
                    float r, float b) {
  awb->cfg = *cfg;
  awb->r = clampf(r, cfg->r_min, cfg->r_max);
  awb->b = clampf(b, cfg->b_min, cfg->b_max);
  awb->window = XMISP_AWB_WIDE;
  awb->wait = 0;
  awb->searching = 0;
  awb->n_cand = 0;
  awb->starved = 0;
  search_points(awb);
  awb->since = SEARCH_EVERY; /* search first */
}

void xmisp_awb_candidates(const struct xmisp_awb *awb,
                          float r[XMISP_AWB_CHANNELS - 1],
                          float b[XMISP_AWB_CHANNELS - 1]) {
  for (unsigned k = 0; k < XMISP_AWB_CHANNELS - 1; k++) {
    const int s = awb->searching && k < awb->n_cand;
    r[k] = s ? awb->sr[awb->cand[k]] : awb->r;
    b[k] = s ? awb->sb[awb->cand[k]] : awb->b;
  }
}

int xmisp_awb_update(struct xmisp_awb *awb,
                     const struct xmisp_awb_meas m[XMISP_AWB_CHANNELS],
                     float *r, float *b) {
  const struct xmisp_awb_cfg *c = &awb->cfg;
  const uint32_t points = m[0].points;
  const float cb = m[0].cb, cr = m[0].cr;
  awb->since++;
  if (awb->wait) {
    awb->wait--;
    return 0;
  }
  if (awb->searching) {
    for (unsigned k = 0; k < awb->n_cand; k++)
      if (m[k + 1].points > awb->best_points) {
        awb->best_points = m[k + 1].points;
        awb->best = (int)awb->cand[k];
      }
    if (awb->next < awb->n_search) {
      next_round(awb);
      return 0;
    }
    awb->searching = 0;
    awb->n_cand = 0;
    awb->since = 0;
    awb->starved = 0;
    if (awb->best >= 0 && awb->best_points >= c->points_min &&
        (float)awb->best_points > (float)points * SEARCH_WIN) {
      *r = awb->sr[awb->best];
      *b = awb->sb[awb->best];
      return 1; /* refined from there, in the narrow window */
    }
  } else if (awb->n_search &&
             (awb->since >= SEARCH_EVERY ||
              (awb->starved >= STARVED && awb->since >= SEARCH_GAP) ||
              (points < c->points_min && awb->window == XMISP_AWB_WIDE &&
               awb->since >= SEARCH_GAP))) {
    search_start(awb);
    return 0;
  }
  if (points < c->points_min) {
    /* Nothing grey enough to go by. In the narrow window that may be a
     * change of light: look wide before holding. */
    if (awb->window == XMISP_AWB_NARROW) {
      awb->starved++;
      awb->window = XMISP_AWB_WIDE;
      awb->wait = c->interval;
    }
    return 0;
  }
  if (awb->window == XMISP_AWB_NARROW)
    awb->starved = 0;
  /* Narrower once near, wider again only if well off: a search lands a
   * grid step from grey, and the wide window there may average two
   * populations into a false grey. */
  const float lim = awb->window == XMISP_AWB_NARROW ? 2.0f * c->near : c->near;
  const int near = fabsf(cb) < lim && fabsf(cr) < lim;
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
