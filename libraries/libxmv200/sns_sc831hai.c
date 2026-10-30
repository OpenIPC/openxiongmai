// SPDX-License-Identifier: MIT
/*
 * SmartSens SC831HAI: the XM530V200 vendor libsns's driver of it, as
 * functions. 4K MIPI at 30 fps, or at 15 fps (the vendor's tables,
 * sns_sc831hai_regs.c; it takes the second where channel 0's frame rate
 * setting reads 0x88).
 *
 * Exposure is 0x3e00..0x3e02 in half lines, the analog gain 0x3e08/0x3e09
 * (doublings, a fine part, and from 1.47x the high-gain bit 0x80 in
 * 0x3e08), the digital gain 0x3e06/0x3e07, line length 0x320c/0x320d,
 * frame length 0x320e/0x320f, mirror and flip 0x3221 (0x06, 0x60). The
 * vendor's gains are in 1/1024.
 *
 * As the vendor's, the frame length remembers one value for all channels.
 */

#include <string.h>
#include <unistd.h>

#include "sns.h"
#include "xmv200.h"

extern const struct xmv200_sns_reg sc831hai_4k_30fps[163],
    sc831hai_4k_15fps[164];

#define N(a) ((int)(sizeof(a) / sizeof((a)[0])))
#define MAX_CHN 3

static struct {
  int valid;
  uint32_t exposure, again, dgain;
  const struct xmv200_sns_reg *mode;
} st[MAX_CHN];
static int vts_valid;
static uint32_t vts_last;

static void w(const struct xmv200_sns_dev *d, uint16_t reg, uint32_t val) {
  xmv200_i2c_write(d->bus, d->addr, reg, 2, val & 0xff, 1);
}

static uint32_t r(const struct xmv200_sns_dev *d, uint16_t reg) {
  uint32_t v = 0;

  xmv200_i2c_read(d->bus, d->addr, reg, 2, &v, 1);
  return v & 0xff;
}

static void init(const struct xmv200_sns_dev *d,
                 const struct xmv200_sns_mode *m,
                 const struct xmv200_sns_attr *a) {
  if (d->chn < MAX_CHN) {
    memset(&st[d->chn], 0, sizeof(st[d->chn]));
    st[d->chn].mode = m->regs;
  }
  vts_valid = 0;
  for (int i = 0; i < m->n; i++)
    w(d, m->regs[i].reg, m->regs[i].val);
  usleep(20000);
}

static void ae_default(const struct xmv200_sns_attr *a,
                       struct xmv200_sns_ae *ae) {
  ae->full_lines = 0x8ca;
  ae->max_lines = 0x8c3;
  ae->again_min = 0x20;
  ae->again_max = 0xb600 >> 5;
  ae->dgain_min = 0x20;
  ae->dgain_max = 0x3f00 >> 5;
}

static void set_exposure(const struct xmv200_sns_dev *d, uint32_t lines) {
  if (d->chn < MAX_CHN) {
    if (st[d->chn].valid & 1 && st[d->chn].exposure == lines)
      return;
    st[d->chn].exposure = lines;
    st[d->chn].valid |= 1;
  }
  w(d, 0x3e02, (lines & 7) << 5);
  w(d, 0x3e01, (lines << 1 & 0xfff) >> 4);
  w(d, 0x3e00, (lines << 1 & 0xffff) >> 12);
}

static void clamp_exposure(const struct xmv200_sns_dev *d,
                           const struct xmv200_sns_ae *ae) {
  if (d->chn < MAX_CHN && st[d->chn].valid & 1 &&
      ae->max_lines < st[d->chn].exposure)
    set_exposure(d, ae->max_lines);
}

/* the 30 fps mode: 25, or 30; the 15 fps mode: 12, or 15. By line length,
 * the frame stays 2250 lines. */
static void set_fps(const struct xmv200_sns_dev *d,
                    const struct xmv200_sns_attr *a, uint32_t fps,
                    struct xmv200_sns_ae *ae) {
  int slow = d->chn < MAX_CHN && st[d->chn].mode == sc831hai_4k_15fps;
  uint32_t hts = fps == (slow ? 12u : 25u) ? 0x13b0 : 0x1068;

  ae->full_lines = 0x8ca;
  ae->max_lines = 0x8c3;
  clamp_exposure(d, ae);
  w(d, 0x320c, hts >> 9);
  w(d, 0x320d, hts >> 1 & 0xfc);
  w(d, 0x320e, 0x08);
  w(d, 0x320f, 0xca);
}

static void set_vts(const struct xmv200_sns_dev *d, uint32_t vts,
                    struct xmv200_sns_ae *ae) {
  if (vts_valid && vts_last == vts)
    return;
  vts_last = vts;
  vts_valid = 1;
  ae->full_lines = vts;
  ae->max_lines = vts - 7;
  clamp_exposure(d, ae);
  w(d, 0x320e, vts >> 8);
  w(d, 0x320f, vts & 0xff);
}

/* gain in 1/32; the vendor's in 1/1024. From 0x5e0 (1.47x) on, the
 * sensor's high gain, and the rest of it after that factor. The value:
 * bit 31 the high gain, bits 8.. the doublings as ones, bits 7..0 the fine
 * part (1/32). */
static void again_calc(uint32_t gain, uint32_t *idx, uint32_t *real) {
  uint32_t g = gain << 5, bits = 0, k = 0, v;
  int high = g > 0x5df;

  if (high)
    g = (uint32_t)(((uint64_t)g << 10) / 0x5e0);
  g >>= 5;
  if (g < 0x20)
    g = 0x20;
  while (k < 5 && g >= 0x40) {
    k++;
    g >>= 1;
    bits = bits << 1 | 1;
  }
  *idx = bits << 8 | (uint32_t)high << 31 | g;
  v = g << (k + 5);
  if (high)
    v = (uint32_t)((uint64_t)v * 0x5e0 >> 10);
  *real = v >> 5;
}

/* gain in 1/32; the vendor's in 1/1024, taken in 1/128: the doublings in
 * the high byte */
static void dgain_calc(uint32_t gain, uint32_t *reg, uint32_t *real) {
  uint32_t g = (gain << 5) >> 3, bits = 0, k = 0;

  if (g < 0x80)
    g = 0x80;
  while (k < 4 && g >= 0x100) {
    k++;
    g >>= 1;
    bits = bits << 1 | 1;
  }
  *reg = (g & ~3u) + bits * 0x100;
  *real = ((g & ~3u) << (k + 3)) >> 5;
}

static void set_gain(const struct xmv200_sns_dev *d, uint32_t again,
                     uint32_t dgain) {
  if (d->chn < MAX_CHN) {
    if (st[d->chn].valid & 2 && st[d->chn].again == again &&
        st[d->chn].dgain == dgain)
      return;
    st[d->chn].again = again;
    st[d->chn].dgain = dgain;
    st[d->chn].valid |= 2;
  }
  w(d, 0x3e08, ((again & 0xffff) >> 8) | (again >> 31 ? 0x80 : 0));
  w(d, 0x3e09, again & 0xff);
  w(d, 0x3e06, (dgain & 0xffff) >> 8);
  w(d, 0x3e07, dgain & 0xff);
}

static void set_flip(const struct xmv200_sns_dev *d, int mirror, int flip) {
  uint32_t v = r(d, 0x3221);

  v = mirror ? v | 0x06 : v & ~0x06u;
  v = flip ? v | 0x60 : v & ~0x60u;
  w(d, 0x3221, v);
}

static void run(const struct xmv200_sns_dev *d, int n,
                const struct xmv200_sns_ctx *c);

/* the mode flag is not in xmv200_sns_attr: the 30 fps mode first */
static const struct xmv200_sns_mode modes[] = {
    {"4k mipi 30 fps", {0, 0, 0, 0}, sc831hai_4k_30fps,
     N(sc831hai_4k_30fps)},
    {"4k mipi 15 fps", {0, 0, 0, 0}, sc831hai_4k_15fps,
     N(sc831hai_4k_15fps)},
};

const struct xmv200_sns_drv xmv200_sns_sc831hai = {
    .name = "SC831HAI",
    .modes = modes,
    .n_modes = N(modes),
    .init = init,
    .ae_default = ae_default,
    .set_fps = set_fps,
    .set_vts = set_vts,
    .set_exposure = set_exposure,
    .again_calc = again_calc,
    .dgain_calc = dgain_calc,
    .set_gain = set_gain,
    .set_flip = set_flip,
    .run = run,
};

/*
 * The tuning's exposure is the register's count, in half lines. The
 * vendor's ISP starts this sensor at 25 fps, where it starts the SC2336 at
 * the board's 12 (its AE library chooses; not looked into).
 */
static void run(const struct xmv200_sns_dev *d, int n,
                const struct xmv200_sns_ctx *c) {
  struct xmv200_sns_ctx x = *c;

  if (n == 0) {
    xmv200_sns_init_step(&xmv200_sns_sc831hai, d, c);
  } else if (n == 1) {
    x.attr.fps = 25;
    xmv200_sns_fps_step(&xmv200_sns_sc831hai, d, &x);
  } else if (n == 2) {
    x.exp_lines = c->exp_lines / 2;
    xmv200_sns_ae_step(&xmv200_sns_sc831hai, d, &x);
  }
}
