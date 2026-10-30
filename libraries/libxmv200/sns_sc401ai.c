// SPDX-License-Identifier: MIT
/*
 * SmartSens SC401AI: the XM530V200 vendor libsns's driver of it, as
 * functions. One mode, MIPI, 2560x1440 at 1500 lines. The mode table is
 * the vendor's (sns_sc401ai_regs.c, tools/xmv200-sns/extract.py).
 *
 * Exposure is 0x3e00..0x3e02 in half lines (lines << 5), the analog gain
 * 0x3e08 (doublings, and from 1.47x the high conversion gain, 0x20) and
 * 0x3e09 (fine), the digital gain 0x3e06/0x3e07. Each setter writes only
 * what changed since the last call, as the vendor's do (the vendor keeps
 * one frame length for all channels; this keeps one a channel).
 */

#include <string.h>
#include <unistd.h>

#include "sns.h"
#include "xmv200.h"

extern const struct xmv200_sns_reg sc401ai_mipi[122];

#define N(a) ((int)(sizeof(a) / sizeof((a)[0])))
#define MAX_CHN 3

static struct {
  int valid;
  uint32_t exposure, again, dgain, vts;
} st[MAX_CHN];

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
  if (d->chn < MAX_CHN)
    memset(&st[d->chn], 0, sizeof(st[d->chn]));
  for (int i = 0; i < m->n; i++)
    w(d, m->regs[i].reg, m->regs[i].val);
  usleep(20000);
}

/* the vendor's limits, its gains in 1/1024 and 1/128, here in 1/32 */
static void ae_default(const struct xmv200_sns_attr *a,
                       struct xmv200_sns_ae *ae) {
  ae->full_lines = 1500;
  ae->max_lines = 1494;
  ae->again_min = 0x400 >> 5;
  ae->again_max = 0x5b18 >> 5;
  ae->dgain_min = 0x80 >> 2;
  ae->dgain_max = 0x1000 >> 2;
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

/* by the line length (0x320c/0x320d, the vendor's count halved): 25, 30,
 * else 20 fps, at 1500 lines */
static void set_fps(const struct xmv200_sns_dev *d,
                    const struct xmv200_sns_attr *a, uint32_t fps,
                    struct xmv200_sns_ae *ae) {
  uint32_t hts = fps == 25 ? 0xd20 : fps == 30 ? 0xaf0 : 0x1068;

  ae->full_lines = 1500;
  ae->max_lines = 1494;
  w(d, 0x320c, hts >> 9);
  w(d, 0x320d, hts >> 1 & 0xfc);
  w(d, 0x320e, 0x05);
  w(d, 0x320f, 0xdc);
}

static void set_vts(const struct xmv200_sns_dev *d, uint32_t vts,
                    struct xmv200_sns_ae *ae) {
  if (d->chn < MAX_CHN) {
    if (st[d->chn].valid & 4 && st[d->chn].vts == vts)
      return;
    st[d->chn].vts = vts;
    st[d->chn].valid |= 4;
  }
  ae->full_lines = vts;
  ae->max_lines = vts - 6;
  if (d->chn < MAX_CHN && st[d->chn].valid & 1 &&
      ae->max_lines < st[d->chn].exposure)
    set_exposure(d, ae->max_lines);
  w(d, 0x320e, vts >> 8);
  w(d, 0x320f, vts & 0xff);
}

/*
 * 1x .. 22.7x (the vendor's 0x400 .. 0x5b18): below 1.47x (0x5e0) the fine
 * gain (0x40 = 1x) alone; from there the high conversion gain (bit 31 here,
 * 0x20 in 0x3e08) and the rest divided by 1.47. Doublings as ones in bits
 * 8 and up.
 */
static void again_calc(uint32_t gain, uint32_t *idx, uint32_t *real) {
  uint32_t u = gain << 5, g, bits = 0, k = 0, hcg = u >= 0x5e0, v;

  g = hcg ? (u << 6) / 0x5e0 : u >> 4;
  if (g < 0x40)
    g = 0x40;
  while (k < 4 && g >= 0x80) {
    k++;
    g >>= 1;
    bits = bits << 1 | 1;
  }
  *idx = bits << 8 | hcg << 31 | g;
  v = g << (k + 4);
  if (hcg)
    v = (uint32_t)(((uint64_t)v * 0x5e0) >> 10);
  *real = v >> 5;
}

/* 1x .. 32x: doublings in 0x3e06, the rest in 0x3e07 (0x80 = 1x, even) */
static void dgain_calc(uint32_t gain, uint32_t *reg, uint32_t *real) {
  uint32_t g = gain << 2, bits = 0, k = 0;

  if (g < 0x80)
    g = 0x80;
  while (k < 5 && g >= 0x100) {
    k++;
    g >>= 1;
    bits = bits << 1 | 1;
  }
  *reg = (g & ~1u) + bits * 0x100;
  *real = ((g & ~1u) << k) >> 2;
}

static void set_gain(const struct xmv200_sns_dev *d, uint32_t areg,
                     uint32_t dreg) {
  if (d->chn < MAX_CHN) {
    if (st[d->chn].valid & 2 && st[d->chn].again == areg &&
        st[d->chn].dgain == dreg)
      return;
    st[d->chn].again = areg;
    st[d->chn].dgain = dreg;
    st[d->chn].valid |= 2;
  }
  w(d, 0x3e08, ((areg & 0xffff) >> 8) << 2 | (areg >> 31 ? 0x23 : 3));
  w(d, 0x3e09, areg & 0xff);
  w(d, 0x3e06, (dreg & 0xffff) >> 8);
  w(d, 0x3e07, dreg & 0xff);
}

static void set_flip(const struct xmv200_sns_dev *d, int mirror, int flip) {
  uint32_t v = r(d, 0x3221);

  v = mirror ? v | 0x06 : v & ~0x06u;
  v = flip ? v | 0x60 : v & ~0x60u;
  w(d, 0x3221, v);
}

static void run(const struct xmv200_sns_dev *d, int n,
                const struct xmv200_sns_ctx *c);

static const struct xmv200_sns_mode modes[] = {
    {"4 MP mipi", {0, 0, 0, 0}, sc401ai_mipi, N(sc401ai_mipi)},
};

const struct xmv200_sns_drv xmv200_sns_sc401ai = {
    .name = "SC401AI",
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

/* the tuning's exposure is in the register's half lines */
static void run(const struct xmv200_sns_dev *d, int n,
                const struct xmv200_sns_ctx *c) {
  const struct xmv200_sns_drv *drv = &xmv200_sns_sc401ai;
  uint32_t areg, dreg, real;

  switch (n) {
  case 0:
    xmv200_sns_init_step(drv, d, c);
    break;
  case 1:
    xmv200_sns_fps_step(drv, d, c);
    break;
  case 2:
    set_exposure(d, c->exp_lines / 2);
    again_calc(0x20, &areg, &real);
    dgain_calc(0x20, &dreg, &real);
    set_gain(d, areg, dreg);
    break;
  }
}

/* ---- XM535W1 ----
 *
 * The XM535W1 libsns's (Apr 2024), unverified (no tuning): the
 * XM530V200's functions, its own copy of the table.
 */

extern const struct xmv200_sns_reg xm535w1_sc401ai_mipi[122];

static void w1_run(const struct xmv200_sns_dev *d, int n,
                   const struct xmv200_sns_ctx *c);

static const struct xmv200_sns_mode w1_modes[] = {
    {"4 MP mipi", {0, 0, 0, 0}, xm535w1_sc401ai_mipi,
     N(xm535w1_sc401ai_mipi)},
};

const struct xmv200_sns_drv xmv200_sns_sc401ai_xm535w1 = {
    .name = "SC401AI",
    .family = "xm535w1",
    .exp_shift = 1,
    .modes = w1_modes,
    .n_modes = N(w1_modes),
    .init = init,
    .ae_default = ae_default,
    .set_fps = set_fps,
    .set_vts = set_vts,
    .set_exposure = set_exposure,
    .again_calc = again_calc,
    .dgain_calc = dgain_calc,
    .set_gain = set_gain,
    .set_flip = set_flip,
    .run = w1_run,
};

static void w1_run(const struct xmv200_sns_dev *d, int n,
                   const struct xmv200_sns_ctx *c) {
  uint32_t areg, dreg, real;

  switch (n) {
  case 0:
    xmv200_sns_init_step(&xmv200_sns_sc401ai_xm535w1, d, c);
    break;
  case 1:
    xmv200_sns_fps_step(&xmv200_sns_sc401ai_xm535w1, d, c);
    break;
  case 2:
    set_exposure(d, c->exp_lines / 2);
    again_calc(0x20, &areg, &real);
    dgain_calc(0x20, &dreg, &real);
    set_gain(d, areg, dreg);
    break;
  }
}

/* ---- XM535 ---- */

/*
 * The XM535D1's libsns (Aug 12 2025 d2db8e4): the XM530V200's functions
 * with its own table. The tuning recorded the init alone.
 */
extern const struct xmv200_sns_reg xm535_sc401ai_mipi[122];

static const struct xmv200_sns_mode xm535_modes[] = {
    {"4 MP mipi", {0, 0, 0, 0}, xm535_sc401ai_mipi, N(xm535_sc401ai_mipi)},
};

static void xm535_run(const struct xmv200_sns_dev *d, int n,
                      const struct xmv200_sns_ctx *c) {
  if (n == 0)
    init(d, c->mode, &c->attr);
}

const struct xmv200_sns_drv xmv200_sns_sc401ai_xm535 = {
    .name = "SC401AI",
    .family = "xm535",
    .exp_shift = 1,
    .modes = xm535_modes,
    .n_modes = N(xm535_modes),
    .init = init,
    .ae_default = ae_default,
    .set_fps = set_fps,
    .set_vts = set_vts,
    .set_exposure = set_exposure,
    .again_calc = again_calc,
    .dgain_calc = dgain_calc,
    .set_gain = set_gain,
    .set_flip = set_flip,
    .n_runs = 1,
    .run = xm535_run,
};

/* ---- XM533 ---- */

/* the XM533 libsns's: the same table and functions */
const struct xmv200_sns_drv xmv200_sns_sc401ai_xm533 = {
    .name = "SC401AI",
    .family = "xm533",
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
