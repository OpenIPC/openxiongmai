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

/* A scene: grey pixels (count n) whose Cb and Cr are k x ln(gain / their
 * gain), counted fewer the farther from grey, none at twice the narrow
 * window, a tenth at least in the wide one (as on the camera, where the
 * wide window always finds some). */
struct pop {
  float r0, b0, k, n;
};

static void count(const struct pop *p, float r, float b,
                  enum xmisp_awb_window w, float *n, float *cb, float *cr) {
  const float half = w == XMISP_AWB_NARROW ? 8.0f : 38.0f;
  *cb = p->k * logf(b / p->b0);
  *cr = p->k * logf(r / p->r0);
  const float off = (fabsf(*cb) + fabsf(*cr)) / (2.0f * half);
  const float left = w == XMISP_AWB_NARROW ? (off < 1.0f ? 1.0f - off : 0.0f)
                                           : (off < 0.9f ? 1.0f - off : 0.1f);
  *n = p->n * left;
}

/* The populations mixed: counts add, Cb and Cr average by count. */
static struct xmisp_awb_meas measure(const struct pop *p, unsigned np, float r,
                                     float b, enum xmisp_awb_window w) {
  float n = 0, cb = 0, cr = 0;
  for (unsigned i = 0; i < np; i++) {
    float ni, cbi, cri;
    count(&p[i], r, b, w, &ni, &cbi, &cri);
    n += ni;
    cb += ni * cbi;
    cr += ni * cri;
  }
  struct xmisp_awb_meas m = {(uint32_t)n, n > 0 ? cb / n : 0,
                             n > 0 ? cr / n : 0};
  return m;
}

/* Runs frames; returns the frame of the last change of the gains set, or -1
 * if still changing in the last 100. */
static int run(struct xmisp_awb *awb, const struct pop *p, unsigned np,
               int frames) {
  int last = 0;
  for (int f = 0; f < frames; f++) {
    float cr[XMISP_AWB_CHANNELS - 1], cb[XMISP_AWB_CHANNELS - 1];
    xmisp_awb_candidates(awb, cr, cb); /* R and B gains, not Cr and Cb */
    struct xmisp_awb_meas m[XMISP_AWB_CHANNELS];
    m[0] = measure(p, np, awb->r, awb->b, awb->window);
    for (unsigned k = 0; k < XMISP_AWB_CHANNELS - 1; k++)
      m[k + 1] = measure(p, np, cr[k], cb[k], awb->window);
    float r, b;
    if (xmisp_awb_update(awb, m, &r, &b)) {
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
        const struct pop grey = {r0, b0, k, 20000};
        const int settled = run(&awb, &grey, 1, 500);
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

  /* The light changes after settling: in the narrow window nothing is grey
   * any more, so it widens and finds the new grey. */
  {
    struct xmisp_awb a;
    xmisp_awb_init(&a, &cfg, 1.3f, 2.4f);
    const struct pop day = {1.56f, 2.19f, 40.0f, 20000};
    run(&a, &day, 1, 200);
    check(a.window == XMISP_AWB_NARROW, "not narrow before the change", 1.56f,
          2.19f);
    const struct pop warm = {1.0f, 3.2f, 40.0f, 20000};
    const int s2 = run(&a, &warm, 1, 400);
    const float er = logf(a.r / 1.0f) * 40, eb = logf(a.b / 3.2f) * 40;
    printf("light changed to R 1.00 B 3.20: settled at frame %d, R %.3f B "
           "%.3f\n",
           s2, a.r, a.b);
    check(s2 >= 0 && fabsf(er) <= 0.8f && fabsf(eb) <= 0.8f,
          "stuck after a change of light", 1.0f, 3.2f);
  }

  /* As the camera measured at luma 0xa0 under 4000 K bulbs: 2143 grey
   * pixels at the neutral, 413 at the warm surfaces' too-blue point. From
   * near the trap grey-world alone stays there; the search finds grey. */
  {
    const struct pop scene[] = {{1.42f, 2.08f, 40.0f, 2143},
                                {0.84f, 3.35f, 40.0f, 413}};
    const float starts3[][2] = {
        {1.24f, 2.37f}, {0.9f, 3.0f}, {0.84f, 3.35f}, {2.0f, 1.5f}};
    for (unsigned j = 0; j < 4; j++) {
      struct xmisp_awb a;
      xmisp_awb_init(&a, &cfg, starts3[j][0], starts3[j][1]);
      run(&a, scene, 2, 600);
      printf("camera-like scene, from R %.2f B %.2f: R %.2f B %.2f\n",
             starts3[j][0], starts3[j][1], a.r, a.b);
      check(fabsf(logf(a.r / 1.42f)) < 0.05f &&
                fabsf(logf(a.b / 2.08f)) < 0.06f,
            "camera-like scene not at grey", 1.42f, 2.08f);
    }
  }

  /* Two populations alike in count, the gains halfway between: the wide
   * window's mean looks grey, the narrow one finds nothing, and grey-world
   * alone goes back and forth between the windows without moving. The
   * search settles on one of them. */
  {
    const struct pop scene[] = {{1.0f, 1.6f, 40.0f, 3000},
                                {2.0f, 3.0f, 40.0f, 3000}};
    struct xmisp_awb a;
    xmisp_awb_init(&a, &cfg, sqrtf(2.0f), sqrtf(4.8f));
    run(&a, scene, 2, 600);
    const int one = fabsf(logf(a.r / 1.0f)) < 0.05f &&
                    fabsf(logf(a.b / 1.6f)) < 0.06f;
    const int two = fabsf(logf(a.r / 2.0f)) < 0.05f &&
                    fabsf(logf(a.b / 3.0f)) < 0.06f;
    printf("between two populations: R %.2f B %.2f\n", a.r, a.b);
    check(one || two, "stalled between two populations", 1.0f, 1.6f);
  }

  /* Too few grey pixels: the gains hold. */
  struct xmisp_awb awb;
  xmisp_awb_init(&awb, &cfg, 1.3f, 2.4f);
  const struct pop few = {1.56f, 2.19f, 40.0f, 200};
  run(&awb, &few, 1, 200);
  check(awb.r == 1.3f && awb.b == 2.4f, "moved without grey pixels", 1.56f,
        2.19f);

  /* Grey outside the range: it stops at the edge, without hunting. */
  xmisp_awb_init(&awb, &cfg, 1.3f, 2.4f);
  const struct pop outside = {3.5f, 0.8f, 40.0f, 20000};
  const int settled = run(&awb, &outside, 1, 400);
  check(settled >= 0 && awb.r == cfg.r_max && awb.b == cfg.b_min,
        "not at the edge of the range", 3.5f, 0.8f);

  printf(failures ? "%d failed\n" : "ok\n", failures);
  return failures != 0;
}
