// SPDX-License-Identifier: MIT
/*
 * Host test of the AE layer on a simulated sensor: the mean is
 * scene x lines x gain, measured cfg.interval frames after a change, with
 * gain in 1/16 steps. Build and run: make -C libraries/libxmisp test
 */
#include <math.h>
#include <stdio.h>

#include "../ae.h"

static const struct xmisp_ae_cfg cfg = {
    .target = 37.5f,
    .tolerance = 0.08f,
    .lines_min = 1,
    .lines_max = 746,
    .gain_max = 16.0f,
    .interval = 2,
};

static int failures;

static void check(int ok, const char *what, double scene) {
  if (!ok) {
    printf("FAIL: %s (scene %g)\n", what, scene);
    failures++;
  }
}

/* Runs frames on a scene; returns the frame AE last changed, or -1 if it
 * still changed in the last 100. Leaves the exposure in *lines, *gain. */
static int run(struct xmisp_ae *ae, double scene, int frames, uint32_t *lines,
               float *gain, double *mean) {
  double pending[8] = {0};
  int last = 0;
  for (int f = 0; f < frames; f++) {
    /* The statistics show the exposure set interval frames before. */
    const double m = fmin(255.0, scene * *lines * *gain);
    pending[f % (cfg.interval + 1)] = m;
    const double seen = pending[(f + 1) % (cfg.interval + 1)];
    *mean = seen ? seen : m;
    uint32_t l;
    float g;
    if (xmisp_ae_update(ae, (float)*mean, &l, &g)) {
      g = floorf(g * 16.0f) / 16.0f;
      xmisp_ae_applied(ae, l, g);
      *lines = l;
      *gain = g;
      last = f;
    }
  }
  return last < frames - 100 ? last : -1;
}

int main(void) {
  const double scenes[] = {0.0005, 0.005, 0.05, 0.5, 5.0};
  for (unsigned i = 0; i < sizeof(scenes) / sizeof(scenes[0]); i++) {
    const double scene = scenes[i];
    const uint32_t starts[] = {1, 100, 746};
    for (unsigned j = 0; j < 3; j++) {
      struct xmisp_ae ae;
      uint32_t lines = starts[j];
      float gain = 1.0f;
      double mean;
      xmisp_ae_init(&ae, &cfg, lines, gain);
      const int settled = run(&ae, scene, 400, &lines, &gain, &mean);
      const double lo = cfg.lines_min * 1.0, hi = cfg.lines_max * cfg.gain_max;
      const double want = cfg.target / scene;
      printf("scene %-7g from %3u lines: settled at frame %3d, %3u lines, "
             "gain %5.2f, mean %6.2f\n",
             scene, (unsigned)starts[j], settled, (unsigned)lines, gain, mean);
      check(settled >= 0, "keeps changing", scene);
      check(settled < 60, "slow to settle", scene);
      if (want < lo)
        check(lines == cfg.lines_min && gain == 1.0f, "not at the bright limit",
              scene);
      else if (want > hi)
        check(lines == cfg.lines_max && gain >= cfg.gain_max - 1.0f / 16,
              "not at the dark limit", scene);
      else
        check(fabs(mean / cfg.target - 1.0) <= cfg.tolerance + 0.07,
              "mean off target", scene);
      check(gain == 1.0f || lines == cfg.lines_max, "gain before lines", scene);
    }
  }
  printf(failures ? "%d failed\n" : "ok\n", failures);
  return failures != 0;
}
