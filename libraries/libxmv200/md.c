// SPDX-License-Identifier: MIT
/*
 * Motion detection, the vendor isp daemon's way (its "vda"), on the
 * pictures AE samples (ae.c) instead of the BPU's statistics.
 *
 * A sensor's picture is an 8x8 grid of blocks, each its mean luma (as
 * 16-bit, mean << 8). Every INTERVAL samples the grid is compared with the
 * previous one:
 * - the darker of the two is first scaled by the ratio of their means, so a
 *   change of light is not motion;
 * - a block moved when it changed by more than its old value divided by the
 *   level's divisor (2, 4, ... 12 for levels 1..6), and by at least 0xc18
 *   over that divisor;
 * - motion is 2% of the watched blocks moving or more. Over 1.2x the light,
 *   or more than half the blocks at once, is a change of scene and
 *   reported as none.
 */
#include <string.h>

#include "xmv200.h"
#include "xmv200_priv.h"

#define GRID 8
#define MAX_SNS 3
#define INTERVAL 7    /* AE samples every third frame: ~20 frames */
#define MIN_PCT 2     /* the daemon's: 2% of the blocks */
#define FLOOR 0xc18

static const uint8_t divisor[XMV200_MD_LEVELS] = {2, 4, 6, 8, 10, 12};

static struct {
  struct xmv200_md cfg;
  uint16_t prev[GRID * GRID];
  int have_prev, count;
  struct xmv200_md_state st;
} md[MAX_SNS];

int xmv200_md_set(int sensor, const struct xmv200_md *m) {
  if (sensor < 0 || sensor >= MAX_SNS || m->level < 1 ||
      m->level > XMV200_MD_LEVELS)
    return -1;
  md[sensor].cfg = *m;
  if (!m->enable) {
    md[sensor].have_prev = 0;
    memset(&md[sensor].st, 0, sizeof(md[sensor].st));
  }
  return 0;
}

int xmv200_md_get(int sensor, struct xmv200_md_state *s) {
  if (sensor < 0 || sensor >= MAX_SNS || !md[sensor].cfg.enable)
    return -1;
  *s = md[sensor].st;
  return 0;
}

void xmv200_md_reset(void) { memset(md, 0, sizeof(md)); }

static void compare(int c, const uint16_t *cur) {
  const uint16_t *old = md[c].prev;
  uint32_t sc = 0, so = 0, ratio, thr_min;
  int moved = 0, watched = 0, pct = 0, cur_darker;
  uint8_t blocks[GRID] = {0};
  uint32_t div = divisor[md[c].cfg.level - 1];

  for (int i = 0; i < GRID * GRID; i++) {
    sc += cur[i];
    so += old[i];
  }
  sc /= GRID * GRID;
  so /= GRID * GRID;
  /* the brighter over the darker, in 1/128 */
  cur_darker = sc < so;
  ratio = cur_darker ? (so << 7) / (sc ? sc : 1) : (sc << 7) / (so ? so : 1);
  thr_min = FLOOR / div;
  for (int y = 0; y < GRID; y++)
    for (int x = 0; x < GRID; x++) {
      uint32_t a = old[y * GRID + x], b = cur[y * GRID + x];
      uint32_t thr = a / div, d;

      if (cur_darker)
        b = b * ratio >> 7 > 0xfe00 ? 0xfe00 : b * ratio >> 7;
      else
        a = a * ratio >> 7 > 0xfe00 ? 0xfe00 : a * ratio >> 7;
      if (thr < thr_min)
        thr = thr_min;
      d = a > b ? a - b : b - a;
      if (d > thr)
        blocks[y] |= 1u << x;
      if (md[c].cfg.region[y] & 1u << x) {
        watched++;
        if (blocks[y] & 1u << x)
          moved++;
      }
    }
  if (watched)
    pct = moved * 100 / watched;
  /* a change of light or of the whole scene: nothing */
  if (ratio > 0x9a || pct > 50) {
    memset(blocks, 0, sizeof(blocks));
    pct = 0;
  }
  memcpy(md[c].st.blocks, blocks, sizeof(blocks));
  md[c].st.percent = pct;
  md[c].st.motion = pct >= MIN_PCT;
  if (md[c].st.motion)
    md[c].st.events++;
}

void xmv200_md_sample(int c, const uint8_t *luma, uint32_t stride,
                      uint32_t width, uint32_t y0, uint32_t height) {
  uint16_t grid[GRID * GRID];
  uint32_t bw = width / GRID, bh = height / GRID;

  if (c < 0 || c >= MAX_SNS || !md[c].cfg.enable || bw < 8 || bh < 8 ||
      md[c].count++ % INTERVAL)
    return;
  for (int gy = 0; gy < GRID; gy++)
    for (int gx = 0; gx < GRID; gx++) {
      uint32_t sum = 0, n = 0;

      for (uint32_t y = gy * bh; y < (gy + 1) * bh; y += 8)
        for (uint32_t x = gx * bw; x < (gx + 1) * bw; x += 8) {
          sum += luma[(y0 + y) * stride + x];
          n++;
        }
      grid[gy * GRID + gx] = n ? (sum << 8) / n : 0;
    }
  if (md[c].have_prev)
    compare(c, grid);
  memcpy(md[c].prev, grid, sizeof(grid));
  md[c].have_prev = 1;
}
