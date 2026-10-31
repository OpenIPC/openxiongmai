// SPDX-License-Identifier: MIT
/* Host test of the day/night decision. Run: make -C libraries/libxmisp test */
#include <stdio.h>

#include "../daynight.h"

static int failures;

static void check(int ok, const char *what) {
  if (!ok) {
    printf("FAIL: %s\n", what);
    failures++;
  }
}

/* Feeds frames at 25 fps from t (ms) for ms; the time into the run of the
 * first change, or -1. */
static long run(struct xmisp_dn *dn, uint32_t *t, uint32_t ms, float gain,
                float mean) {
  for (uint32_t f = 0; f < ms / 40; f++, *t += 40)
    if (xmisp_dn_update(dn, *t, gain, 16.0f, mean, 37.5f))
      return (long)f * 40;
  return -1;
}

int main(void) {
  const struct xmisp_dn_cfg cfg = {0, 2.0f, 15000, 60000};
  struct xmisp_dn dn;
  uint32_t t = 1000;
  xmisp_dn_init(&dn, &cfg, 0);

  /* Out of gain but the picture still at its target: not night. */
  check(run(&dn, &t, 30000, 16.0f, 37.0f) < 0, "bright at full gain");
  /* High gain short of the ceiling, dark: not night. */
  check(run(&dn, &t, 30000, 12.0f, 10.0f) < 0, "dark below the ceiling");
  /* Out of gain and dark: night after 15 s, not before. */
  const long n = run(&dn, &t, 30000, 16.0f, 5.0f);
  check(n >= 15000 && n < 15100, "night 15 s after it went dark");
  check(dn.night, "night set");
  /* A dark spell interrupted by a moment of light starts again. */
  xmisp_dn_init(&dn, &cfg, 0);
  check(run(&dn, &t, 10000, 16.0f, 5.0f) < 0, "10 s dark");
  check(run(&dn, &t, 40, 4.0f, 37.0f) < 0, "a flash");
  check(run(&dn, &t, 10000, 16.0f, 5.0f) < 0, "10 s dark again: still day");
  /* At night, headlights for 20 s (gain down to 1x): still night; gain at
   * 3x, above day_gain: still night. */
  xmisp_dn_init(&dn, &cfg, 1);
  check(run(&dn, &t, 20000, 1.0f, 37.0f) < 0, "20 s of headlights");
  check(run(&dn, &t, 120000, 3.0f, 37.0f) < 0, "gain 3x for 2 min");
  /* Gain at 2x for a minute: day. */
  const long d = run(&dn, &t, 90000, 2.0f, 37.0f);
  check(d >= 60000 && d < 60100, "day 60 s after the gain fell to 2x");
  check(!dn.night, "day set");
  /* With a night gain set, the gain alone decides. */
  const struct xmisp_dn_cfg by_gain = {8.0f, 2.0f, 15000, 60000};
  xmisp_dn_init(&dn, &by_gain, 0);
  check(run(&dn, &t, 20000, 8.0f, 37.0f) >= 0, "night at 8x");
  /* The clock wrapping around changes nothing. */
  xmisp_dn_init(&dn, &cfg, 0);
  t = 0xffffffffu - 5000;
  check(run(&dn, &t, 20000, 16.0f, 5.0f) >= 0, "night across the wrap");

  if (failures)
    printf("%d failures\n", failures);
  else
    printf("day/night: ok\n");
  return failures != 0;
}
