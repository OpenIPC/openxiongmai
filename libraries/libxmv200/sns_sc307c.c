// SPDX-License-Identifier: MIT
/*
 * SmartSens SC307C (chip ID 0xcb07): the XM530V200 vendor libsns's driver
 * of it (in its isp daemon), as functions. One mode, 720p MIPI; its table
 * is the vendor's (sns_sc307c_regs.c, tools/xmv200-sns/extract.py).
 *
 * The vendor's analog gain is in 1/1024 (0x400 = 1x) from a table of 64
 * steps, its digital gain in 1/128; again_calc and dgain_calc take sns.h's
 * 1/32 and give it back. Exposure is in lines and goes to 0x3e00..0x3e02
 * in half lines (lines << 5): a tuning's exp_lines, read off those
 * registers as lines << 4, is twice it.
 *
 * Exposure is 0x3e00..0x3e02 (0x3314 follows it), the analog gain
 * 0x3e08/0x3e09 (0x3301/0x3632 follow it), the digital gain 0x3e06/0x3e07,
 * line length 0x320c/0x320d, frame length 0x320e/0x320f, mirror and flip
 * 0x3221 (0x06, 0x60). Each setter writes only what changed since the last
 * call, as the vendor's do.
 */

#include <string.h>
#include <unistd.h>

#include "sns.h"
#include "xmv200.h"

extern const struct xmv200_sns_reg sc307c_mipi[192];

#define N(a) ((int)(sizeof(a) / sizeof((a)[0])))
#define MAX_CHN 3

/* the analog gain's steps, 1/1024 (the vendor's table at 0x6e214) */
static const uint16_t again_val[64] = {
    0x400,  0x440,  0x480,  0x4c0,  0x500,  0x540,  0x580,  0x5c0,
    0x600,  0x640,  0x680,  0x6c0,  0x700,  0x740,  0x780,  0x7c0,
    0x800,  0x880,  0x900,  0x980,  0xa00,  0xa80,  0xb00,  0xb80,
    0xc00,  0xc80,  0xd00,  0xd80,  0xe00,  0xe80,  0xf00,  0xf80,
    0x1000, 0x1100, 0x1200, 0x1300, 0x1400, 0x1500, 0x1600, 0x1700,
    0x1800, 0x1900, 0x1a00, 0x1b00, 0x1c00, 0x1d00, 0x1e00, 0x1f00,
    0x2000, 0x2200, 0x2400, 0x2600, 0x2800, 0x2a00, 0x2c00, 0x2e00,
    0x3000, 0x3200, 0x3400, 0x3600, 0x3800, 0x3a00, 0x3c00, 0x3e00};
/* 0x3301 and 0x3632 per level of the total gain (the vendor's at 0x7f9be) */
static const uint8_t level_reg[5][2] = {
    {0x0f, 0x08}, {0x20, 0x08}, {0x28, 0x08}, {0x80, 0x08}, {0x80, 0x48}};

static struct {
  int valid;
  uint32_t exposure, again, dgain, vts;
  uint8_t level; /* the gain's level << 4, 0xff: none yet */
} st[MAX_CHN];
static uint8_t r3314 = 0xff; /* the vendor's keeps one for all sensors */

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
    st[d->chn].level = 0xff;
  }
  for (int i = 0; i < m->n; i++)
    w(d, m->regs[i].reg, m->regs[i].val);
}

static void ae_default(const struct xmv200_sns_attr *a,
                       struct xmv200_sns_ae *ae) {
  ae->full_lines = 750;
  ae->max_lines = 746;
  ae->again_min = 0x400 >> 5;
  ae->again_max = 0x3e00 >> 5;
  ae->dgain_min = 0x80 >> 2;
  ae->dgain_max = 0xf80 >> 2;
}

/* short exposures take 0x3314 = 0x14, long ones 0x04 */
static void set_exposure(const struct xmv200_sns_dev *d, uint32_t lines) {
  uint32_t v = lines << 1;
  uint8_t lv = 0;

  if (d->chn < MAX_CHN) {
    if (st[d->chn].valid & 1 && st[d->chn].exposure == lines)
      return;
    st[d->chn].exposure = lines;
    st[d->chn].valid |= 1;
  }
  if (lines < 0x250)
    lv = 0x14;
  else if (lines > 0x450)
    lv = 0x04;
  if (lv && r3314 != lv) {
    r3314 = lv;
    w(d, 0x3314, lv);
  }
  w(d, 0x3e02, (lines << 5) & 0xe0);
  w(d, 0x3e01, (v >> 4) & 0xff);
  w(d, 0x3e00, (v >> 12) & 0x0f);
}

static void clamp_exposure(const struct xmv200_sns_dev *d,
                           const struct xmv200_sns_ae *ae) {
  if (d->chn < MAX_CHN && st[d->chn].valid & 1 &&
      ae->max_lines < st[d->chn].exposure)
    set_exposure(d, ae->max_lines);
}

/* 25 and 30 fps, 26 and 31 the vendor's alternative 25 and 30 fps
 * timings; anything else 20 */
static void set_fps(const struct xmv200_sns_dev *d,
                    const struct xmv200_sns_attr *a, uint32_t fps,
                    struct xmv200_sns_ae *ae) {
  uint32_t hts, vts = 0x2ee;

  switch (fps) {
  case 25:
    w(d, 0x3f04, 0x04);
    w(d, 0x3f05, 0x8c);
    hts = 0x960;
    break;
  case 26:
    hts = 0x948, vts = 0x4e2;
    break;
  case 30:
    w(d, 0x3f04, 0x03);
    w(d, 0x3f05, 0xc4);
    hts = 2000;
    break;
  case 31:
    hts = 0x898, vts = 0x464;
    break;
  default:
    hts = 3000;
  }
  ae->full_lines = vts;
  ae->max_lines = vts - 4;
  clamp_exposure(d, ae);
  w(d, 0x320c, hts >> 8);
  w(d, 0x320d, hts & 0xf8);
  w(d, 0x320e, vts >> 8);
  w(d, 0x320f, vts & 0xee);
  if (d->chn < MAX_CHN)
    st[d->chn].vts = vts;
}

static void set_vts(const struct xmv200_sns_dev *d, uint32_t vts,
                    struct xmv200_sns_ae *ae) {
  if (d->chn < MAX_CHN) {
    if (st[d->chn].vts == vts)
      return;
    st[d->chn].vts = vts;
  }
  ae->full_lines = vts;
  ae->max_lines = vts - 4;
  clamp_exposure(d, ae);
  w(d, 0x320e, vts >> 8);
  w(d, 0x320f, vts & 0xff);
}

/* the step at or below the gain; the index is the vendor's 0..63 */
static void again_calc(uint32_t gain, uint32_t *idx, uint32_t *real) {
  uint32_t g = gain << 5, i = 63;

  if (g < again_val[63]) {
    for (i = 1; g >= again_val[i]; i++)
      ;
    i--;
  }
  *idx = i;
  *real = again_val[i] >> 5;
}

/* in 1/128 from 1x: doublings as ones in 0x3e06, the rest in 0x3e07 */
static void dgain_calc(uint32_t gain, uint32_t *reg, uint32_t *real) {
  uint32_t p = gain << 2, bits = 0, k = 0;

  while (k < 7 && p >= 0x100) {
    k++;
    p >>= 1;
    bits = bits << 1 | 1;
  }
  *reg = p + bits * 0x100;
  *real = (p << k) >> 2;
}

static void set_gain(const struct xmv200_sns_dev *d, uint32_t idx,
                     uint32_t dreg) {
  /* the digital gain from its register value, 1/128 */
  uint32_t dreal = (dreg & 0xff) << __builtin_popcount((dreg >> 8) & 0x7f);
  uint32_t total = (uint32_t)((uint64_t)again_val[idx & 63] * dreal >> 13);
  uint32_t lv = total < 0x20 ? 0 : total < 0x40 ? 1 : total < 0x80 ? 2
                                 : total < 0xf8 ? 3 : 4;
  uint32_t coarse = 3;

  if (d->chn < MAX_CHN && lv != (uint32_t)st[d->chn].level >> 4) {
    st[d->chn].level = lv << 4;
    w(d, 0x3812, 0x00); /* group hold */
    w(d, 0x3301, level_reg[lv][0]);
    w(d, 0x3632, level_reg[lv][1]);
    w(d, 0x3812, 0x30);
  }
  if (d->chn < MAX_CHN) {
    if (st[d->chn].valid & 2 && st[d->chn].again == idx &&
        st[d->chn].dgain == dreg)
      return;
    st[d->chn].again = idx;
    st[d->chn].dgain = dreg;
    st[d->chn].valid |= 2;
  }
  for (uint32_t k = idx >> 4; k; k--)
    coarse = coarse << 1 | 1;
  w(d, 0x3e08, coarse);
  w(d, 0x3e09, (idx & 0xf) | 0x10);
  w(d, 0x3e06, (dreg & 0xffff) >> 8);
  w(d, 0x3e07, dreg & 0xff);
}

/* no other way round for channel 1 on this one */
static void set_flip(const struct xmv200_sns_dev *d, int mirror, int flip) {
  uint32_t v = r(d, 0x3221);

  v = mirror ? v | 0x06 : v & ~0x06u;
  v = flip ? v | 0x60 : v & ~0x60u;
  w(d, 0x3221, v);
}

/*
 * The tuning's runs. The vendor's starts it at 25 fps, the one rate below
 * 30 its frame rate function has (another is 20 fps); and the exposure it
 * starts from is the tuning's exp_lines in half lines.
 */
static void run(const struct xmv200_sns_dev *d, int n,
                const struct xmv200_sns_ctx *c);

static const struct xmv200_sns_mode modes[] = {
    {"720p mipi", {0, 0, 1, 0}, sc307c_mipi, N(sc307c_mipi)},
};

const struct xmv200_sns_drv xmv200_sns_sc307c = {
    .name = "SC307C",
    .exp_shift = 1,
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

static void run(const struct xmv200_sns_dev *d, int n,
                const struct xmv200_sns_ctx *c) {
  const struct xmv200_sns_drv *drv = &xmv200_sns_sc307c;
  struct xmv200_sns_ctx h = *c;

  switch (n) {
  case 0:
    xmv200_sns_init_step(drv, d, c);
    break;
  case 1:
    if (h.attr.fps != 30 && h.attr.fps != 26 && h.attr.fps != 31)
      h.attr.fps = 25;
    xmv200_sns_fps_step(drv, d, &h);
    break;
  case 2:
    h.exp_lines = c->exp_lines / 2;
    xmv200_sns_ae_step(drv, d, &h);
    break;
  }
}
