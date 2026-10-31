// SPDX-License-Identifier: MIT
#include <math.h>

#include "tune.h"

float xmisp_strength(float gain, float gain_lo, float gain_hi, unsigned steps) {
  if (!(gain > gain_lo) || !(gain_hi > gain_lo))
    return 0.0f;
  if (gain >= gain_hi)
    return 1.0f;
  const float s = log2f(gain / gain_lo) / log2f(gain_hi / gain_lo);
  return steps ? floorf(s * (float)steps + 0.5f) / (float)steps : s;
}

uint32_t xmisp_lerp(uint32_t lo, uint32_t hi, float strength) {
  return (uint32_t)lrintf((float)lo + ((float)hi - (float)lo) * strength);
}
