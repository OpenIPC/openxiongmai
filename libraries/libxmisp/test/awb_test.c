// SPDX-License-Identifier: MIT
/*
 * Host test of the AWB layer on a simulated scene: grey pixels whose mean
 * Cb and Cr are K x ln(gain / grey's gain), as the XM510 statistics behave
 * near grey, counted only inside the window. Run: make -C libraries/libxmisp
 * test
 */
#include <math.h>
#include <stdio.h>

#include "../awb.h"

static const struct xmisp_awb_cfg cfg = {
    .r_min = 0.75f,
    .r_max = 2.8f,
    .b_min = 1.1f,
    .b_max = 3.6f,
    .points_min = 300,
    .near = 3.0f,
    .tolerance = 0.5f,
    .step = 40.0f,
    .interval = 2,
};

static int failures;

static void check(int ok, const char *what, float r0, float b0) {
  if (!ok) {
    printf("FAIL: %s (grey at R %.2f B %.2f)\n", what, r0, b0);
    failures++;
  }
}

/* K: Cb or Cr per unit of ln(gain); the window passes |Cb|,|Cr| within
 * it. Returns the frame of the last change, or -1 if still changing. */
static int run(struct xmisp_awb *awb, float r0, float b0, float k,
               unsigned points, int frames) {
  int last = 0;
  for (int f = 0; f < frames; f++) {
    const float cb = k * logf(awb->b / b0), cr = k * logf(awb->r / r0);
    const float half = awb->window == XMISP_AWB_NARROW ? 8.0f : 38.0f;
    const unsigned n =
        fabsf(cb) + fabsf(cr) < half ? points : points / 50; /* edge: few */
    float r, b;
    if (xmisp_awb_update(awb, n, cb, cr, &r, &b)) {
      xmisp_awb_applied(awb, r, b);
      last = f;
    }
  }
  return last < frames - 100 ? last : -1;
}

int main(void) {
  const float greys[][2] = {{1.56f, 2.19f}, {1.0f, 3.2f}, {2.2f, 1.4f}};
  const float starts[][2] = {
      {0.75f, 1.1f}, {2.8f, 3.6f}, {0.75f, 3.6f}, {2.8f, 1.1f}, {1.3f, 2.4f}};
  for (unsigned i = 0; i < 3; i++)
    for (unsigned j = 0; j < 5; j++)
      for (int kk = 0; kk < 2; kk++) {
        const float r0 = greys[i][0], b0 = greys[i][1];
        const float k = kk ? 60.0f : 25.0f; /* steeper or flatter */
        struct xmisp_awb awb;
        xmisp_awb_init(&awb, &cfg, starts[j][0], starts[j][1]);
        const int settled = run(&awb, r0, b0, k, 20000, 500);
        const float er = logf(awb.r / r0) * k, eb = logf(awb.b / b0) * k;
        printf("grey R %.2f B %.2f, K %2.0f, from R %.2f B %.2f: settled at "
               "frame %3d, R %.3f B %.3f (Cr %5.2f Cb %5.2f) %s\n",
               r0, b0, k, starts[j][0], starts[j][1], settled, awb.r, awb.b, er,
               eb, awb.window == XMISP_AWB_NARROW ? "narrow" : "wide");
        check(settled >= 0, "keeps changing", r0, b0);
        check(settled < 150, "slow to settle", r0, b0);
        check(fabsf(er) <= cfg.tolerance + 0.3f &&
                  fabsf(eb) <= cfg.tolerance + 0.3f,
              "not at grey", r0, b0);
        check(awb.window == XMISP_AWB_NARROW, "window not narrowed", r0, b0);
      }

  /* Too few grey pixels: the gains hold. */
  struct xmisp_awb awb;
  xmisp_awb_init(&awb, &cfg, 1.3f, 2.4f);
  run(&awb, 1.56f, 2.19f, 40.0f, 200, 200);
  check(awb.r == 1.3f && awb.b == 2.4f, "moved without grey pixels", 1.56f,
        2.19f);

  /* Grey outside the range: it stops at the edge, without hunting. */
  xmisp_awb_init(&awb, &cfg, 1.3f, 2.4f);
  const int settled = run(&awb, 3.5f, 0.8f, 40.0f, 20000, 400);
  check(settled >= 0 && awb.r == cfg.r_max && awb.b == cfg.b_min,
        "not at the edge of the range", 3.5f, 0.8f);

  printf(failures ? "%d failed\n" : "ok\n", failures);
  return failures != 0;
}
