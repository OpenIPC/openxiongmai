// SPDX-License-Identifier: MIT
/*
 * Host test of the AE layer on a simulated sensor: the mean is
 * scene x lines x gain, measured cfg.interval frames after a change, with
 * gain in 1/16 steps (the nearest is taken). Build and run: make -C
 * libraries/libxmisp test
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
static int writes_same; /* proposals that rounded to the setting there was */

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
      g = roundf(g * 16.0f) / 16.0f;
      xmisp_ae_applied(ae, l, g);
      if (l != *lines || g != *gain)
        last = f;
      else
        writes_same++;
      *lines = l;
      *gain = g;
    }
  }
  return last < frames - 100 ? last : -1;
}

/* run() with another configuration of the same interval. */
#define runc(ae, c, ...) ((void)(c), run(ae, __VA_ARGS__))

int main(void) {
  /* 25 and 30 want 1.5 and 1.25 lines: between the steps there are. */
  const double scenes[] = {0.0005, 0.005, 0.05, 0.5, 5.0, 25.0, 30.0};
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
      /* Gain only at the most lines, or for the fraction of a line. */
      check(lines == cfg.lines_max || gain <= 1.0f + 1.0f / lines + 1.0f / 16,
            "gain before lines", scene);
      check(writes_same < 3, "rewrites the setting it has", scene);
      writes_same = 0;
    }
  }
  /* Anti-flicker, 50 Hz at 53.33 us a line: 187.5 lines a period. */
  {
    struct xmisp_ae_cfg fc = cfg;
    fc.flicker_lines = 187.5f;
    const double fscenes[] = {0.0005, 0.005, 0.05, 0.1, 0.5, 5.0};
    for (unsigned i = 0; i < sizeof(fscenes) / sizeof(fscenes[0]); i++) {
      struct xmisp_ae ae;
      uint32_t lines = 100;
      float gain = 1.0f;
      double mean;
      xmisp_ae_init(&ae, &fc, lines, gain);
      const int settled = runc(&ae, &fc, fscenes[i], 400, &lines, &gain, &mean);
      const double periods = lines / 187.5;
      const int whole = fabs(periods - round(periods)) * 187.5 <= 0.51;
      printf("anti-flicker scene %-6g: settled at frame %3d, %3u lines (%.2f "
             "periods), gain %5.2f, mean %6.2f\n",
             fscenes[i], settled, (unsigned)lines, periods, gain, mean);
      check(settled >= 0, "keeps changing (anti-flicker)", fscenes[i]);
      check(lines < 187 || whole, "not whole flicker periods", fscenes[i]);
      check(lines <= 563, "beyond the last whole period", fscenes[i]);
      const double want = fc.target / fscenes[i];
      if (want >= 1 && want <= 562.5 * fc.gain_max)
        check(fabs(mean / fc.target - 1.0) <= fc.tolerance + 0.07,
              "mean off target (anti-flicker)", fscenes[i]);
    }
    /* Turned on while settled at 300 lines: it moves to whole periods in
     * the same light. */
    struct xmisp_ae ae;
    uint32_t lines = 300;
    float gain = 1.0f;
    double mean;
    xmisp_ae_init(&ae, &cfg, lines, gain);
    runc(&ae, &cfg, cfg.target / 300.0, 50, &lines, &gain, &mean);
    xmisp_ae_flicker(&ae, 187.5f);
    struct xmisp_ae_cfg fc2 = cfg;
    fc2.flicker_lines = 187.5f;
    runc(&ae, &fc2, cfg.target / 300.0, 100, &lines, &gain, &mean);
    printf("anti-flicker turned on at 300 lines: %u lines, gain %.2f, mean "
           "%.2f\n",
           (unsigned)lines, gain, mean);
    check(lines == 188 && fabs(mean / cfg.target - 1.0) <= 0.15,
          "not re-planned when turned on", cfg.target / 300.0);
  }

  /* A sensor minimum between period boundaries: the exposure is still
   * whole periods, the first one above the minimum. */
  {
    struct xmisp_ae_cfg mc = cfg;
    mc.flicker_lines = 187.5f;
    mc.lines_min = 200;
    struct xmisp_ae ae;
    uint32_t lines = 300;
    float gain = 1.0f;
    double mean;
    xmisp_ae_init(&ae, &mc, lines, gain);
    runc(&ae, &mc, mc.target / 250.0, 300, &lines, &gain, &mean);
    printf("anti-flicker with a 200-line minimum: %u lines, gain %.2f\n",
           (unsigned)lines, gain);
    check(lines == 375, "minimum between periods broke alignment",
          mc.target / 250.0);
  }

  /* A write that fails while re-planning: AE goes on from what the sensor
   * has, and the re-plan stays pending until it has whole periods. */
  {
    struct xmisp_ae_cfg fc3 = cfg;
    struct xmisp_ae ae;
    xmisp_ae_init(&ae, &fc3, 300, 1.0f);
    xmisp_ae_flicker(&ae, 187.5f);
    uint32_t l;
    float g;
    int asked = xmisp_ae_update(&ae, cfg.target, &l, &g);
    xmisp_ae_applied(&ae, 300, 1.0f); /* the write failed: read back */
    check(asked && ae.replan, "re-plan lost on a failed write", 0);
    for (int f = 0; f < 4 && !xmisp_ae_update(&ae, cfg.target, &l, &g); f++)
      ;
    check(l == 188, "not retried after a failed write", 0);
  }

  printf(failures ? "%d failed\n" : "ok\n", failures);
  return failures != 0;
}
