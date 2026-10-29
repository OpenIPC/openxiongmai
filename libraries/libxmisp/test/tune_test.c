// SPDX-License-Identifier: MIT
/* Host test of the tuning-by-gain helpers. */
#include <math.h>
#include <stdio.h>

#include "../tune.h"

static int failures;

static void check(int ok, const char *what) {
  if (!ok) {
    printf("FAIL: %s\n", what);
    failures++;
  }
}

int main(void) {
  check(xmisp_strength(1, 4, 16, 8) == 0, "below gain_lo is 0");
  check(xmisp_strength(4, 4, 16, 8) == 0, "at gain_lo is 0");
  check(xmisp_strength(8, 4, 16, 8) == 0.5f, "half way in log is 0.5");
  check(xmisp_strength(16, 4, 16, 8) == 1, "at gain_hi is 1");
  check(xmisp_strength(31, 4, 16, 8) == 1, "above gain_hi is 1");
  check(xmisp_strength(9, 4, 16, 8) == 0.625f, "quantised to eighths");
  check(xmisp_strength(NAN, 4, 16, 8) == 0, "NaN gain is 0");
  check(xmisp_strength(8, 16, 4, 8) == 0, "empty range is 0");
  float last = 0;
  for (float g = 1; g < 40; g *= 1.05f) {
    const float s = xmisp_strength(g, 4, 16, 8);
    check(s >= last, "not monotonic");
    last = s;
  }
  check(xmisp_lerp(0x100, 0x82, 0) == 0x100, "lerp at 0");
  check(xmisp_lerp(0x100, 0x82, 1) == 0x82, "lerp at 1");
  check(xmisp_lerp(1, 6, 0.5f) == 4, "lerp rounds");
  check(xmisp_lerp(0x3a, 0x26, 0.5f) == 0x30, "lerp down");
  printf(failures ? "%d failed\n" : "ok\n", failures);
  return failures != 0;
}
