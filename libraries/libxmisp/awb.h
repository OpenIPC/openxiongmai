/* SPDX-License-Identifier: MIT */
/*
 * Auto white balance, independent of the SoC: grey-world over the pixels
 * that look grey under the gains set. The SoC side measures, for those
 * pixels, how many there are and their mean Cb and Cr (8-bit, signed),
 * within a window around grey that this layer chooses: wide while the
 * gains are far off, narrow once they are close. Bright pixels only: in a
 * room of beige and brown, the dim ones pass as grey under the wrong gains.
 */
#ifndef XMISP_AWB_H
#define XMISP_AWB_H

#include <stdint.h>

struct xmisp_awb_cfg {
  float r_min, r_max; /* gain range relative to G, the sensor's locus */
  float b_min, b_max;
  uint32_t points_min; /* fewer grey pixels than this: hold the gains */
  float near;          /* |Cb|, |Cr| below which the window narrows */
  float tolerance;     /* |Cb|, |Cr| left alone */
  float step;          /* Cb or Cr per unit of ln(gain); a larger step
                          moves the gains less for the same error */
  unsigned interval;   /* frames from a change until its result shows */
};

enum xmisp_awb_window { XMISP_AWB_WIDE, XMISP_AWB_NARROW };

/* One statistics channel's grey pixels under its candidate gains. */
struct xmisp_awb_meas {
  uint32_t points;
  float cb, cr;
};

#define XMISP_AWB_CHANNELS 4 /* channel 0 the gains set, 1-3 candidates */
#define XMISP_AWB_SEARCH_MAX 25

struct xmisp_awb {
  struct xmisp_awb_cfg cfg;
  float r, b; /* gains relative to G */
  enum xmisp_awb_window window;
  unsigned wait;
  /* The search: a grid of candidates, three a round in channels 1-3; the
   * one with the most grey pixels in the narrow window wins. */
  float sr[XMISP_AWB_SEARCH_MAX], sb[XMISP_AWB_SEARCH_MAX];
  unsigned n_search, next, cand[XMISP_AWB_CHANNELS - 1], n_cand;
  int searching, best;
  uint32_t best_points;
  unsigned since;   /* frames since the last search */
  unsigned starved; /* narrow windows in a row with too few grey pixels */
};

void xmisp_awb_init(struct xmisp_awb *awb, const struct xmisp_awb_cfg *cfg,
                    float r, float b);

/* One frame's measurements: m[0] under the gains set, m[1..] under the
 * candidates xmisp_awb_candidates() gave. 1 if the gains should change, with
 * *r and *b set (relative to G); they change once xmisp_awb_applied() says
 * they were set. The window to measure the next frame with is
 * awb->window. */
int xmisp_awb_update(struct xmisp_awb *awb,
                     const struct xmisp_awb_meas m[XMISP_AWB_CHANNELS],
                     float *r, float *b);

/* The gains for channels 1-3 from now (the gains set, when not searching). */
void xmisp_awb_candidates(const struct xmisp_awb *awb,
                          float r[XMISP_AWB_CHANNELS - 1],
                          float b[XMISP_AWB_CHANNELS - 1]);

void xmisp_awb_applied(struct xmisp_awb *awb, float r, float b);

#endif
