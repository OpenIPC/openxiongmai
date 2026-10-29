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

struct xmisp_awb {
  struct xmisp_awb_cfg cfg;
  float r, b; /* gains relative to G */
  enum xmisp_awb_window window;
  unsigned wait;
};

void xmisp_awb_init(struct xmisp_awb *awb, const struct xmisp_awb_cfg *cfg,
                    float r, float b);

/* One frame's measurement under the current gains and window. 1 if the
 * gains should change, with *r and *b set (relative to G); they change once
 * xmisp_awb_applied() says they were set. The window to measure the next
 * frame with is awb->window. */
int xmisp_awb_update(struct xmisp_awb *awb, uint32_t points, float cb, float cr,
                     float *r, float *b);

void xmisp_awb_applied(struct xmisp_awb *awb, float r, float b);

#endif
