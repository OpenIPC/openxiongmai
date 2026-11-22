// SPDX-License-Identifier: MIT
/*
 * SmartSens SC223AP: the XM530V200 vendor libsns's driver of it (in its isp
 * daemon), as functions. It reads chip ID 0xcb3e, as the SC223A does; the
 * vendor's probe takes it for the SC223AP where 0x801b reads non-zero. The
 * mode tables are the vendor's (sns_sc223ap_regs.c,
 * tools/xmv200-sns/extract.py).
 *
 * Exposure is 0x3e00..0x3e02 (lines << 4), the coarse analog gain 0x3e09
 * (and 0x3e13 for the short frame), the fine gain 0x3e06/0x3e07 (0x3e10/
 * 0x3e11), line length 0x320c/0x320d, frame length 0x320e/0x320f, mirror
 * and flip 0x3221 (0x06, 0xe0). Each setter writes only what changed since
 * the last call, as the vendor's do.
 */

#include <string.h>
#include <unistd.h>

#include "sns.h"
#include "xmv200.h"

extern const struct xmv200_sns_reg sc223ap_dvp[209], sc223ap_mipi[201],
    sc223ap_mipi_max[203];

#define N(a) ((int)(sizeof(a) / sizeof((a)[0])))
#define MAX_CHN 3

/* the analog gain's steps (1x .. 32x) and their 0x3e09 values */
static const uint32_t again_val[6] = {0x20, 0x40, 0x80, 0x100, 0x200, 0x400};
static const uint8_t again_reg[7] = {0x00, 0x40, 0x48, 0x49, 0x4b, 0x4f, 0x5f};

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

static void table(const struct xmv200_sns_dev *d,
                  const struct xmv200_sns_reg *t, int n) {
  for (int i = 0; i < n; i++)
    w(d, t[i].reg, t[i].val);
}

/* ---- init ---- */

static void init(const struct xmv200_sns_dev *d,
                 const struct xmv200_sns_mode *m,
                 const struct xmv200_sns_attr *a) {
  if (d->chn < MAX_CHN)
    memset(&st[d->chn], 0, sizeof(st[d->chn]));
  if (m->regs == sc223ap_dvp) {
    w(d, 0x0103, 0x01);
    usleep(20000);
    table(d, m->regs, m->n);
    if (d->chn == 2) { /* the third sensor slaves its sync */
      w(d, 0x3000, 0x00);
      w(d, 0x3001, 0x00);
      w(d, 0x3002, 0x00);
      w(d, 0x300a, 0x20);
    } else {
      w(d, 0x3000, 0x0f);
      w(d, 0x3001, 0xff);
      w(d, 0x3002, 0xf0);
      w(d, 0x300a, 0x24);
    }
    w(d, 0x3032, 0xa0);
    w(d, 0x3225, 0x30);
    w(d, 0x3217, 0x04);
    w(d, 0x322e, 0x04);
    w(d, 0x322f, 0x4a);
    w(d, 0x0100, 0x01);
    usleep(20000);
    return;
  }
  /* MIPI; "mipi max" (the vendor's attr +0x23 bit 0) is the table and its
   * last two writes, 0x3650/0x3651, after the pause */
  if (m->regs == sc223ap_mipi_max) {
    table(d, m->regs, m->n - 2);
    usleep(20000);
    table(d, m->regs + m->n - 2, 2);
  } else {
    table(d, m->regs, m->n);
    usleep(20000);
  }
}

/* ---- AE limits ---- */

static void ae_default(const struct xmv200_sns_attr *a,
                       struct xmv200_sns_ae *ae) {
  ae->full_lines = 1125;
  ae->max_lines = 1120;
  ae->again_min = 0x20;
  ae->again_max = 0x73e;
  ae->dgain_min = 0x20;
  ae->dgain_max = 0x40;
}

/* ---- exposure ---- */

static void set_exposure(const struct xmv200_sns_dev *d, uint32_t lines) {
  if (d->chn < MAX_CHN) {
    if (st[d->chn].valid & 1 && st[d->chn].exposure == lines)
      return;
    st[d->chn].exposure = lines;
    st[d->chn].valid |= 1;
  }
  w(d, 0x3e02, (lines & 0xf) << 4);
  w(d, 0x3e01, (lines & 0xfff) >> 4);
  w(d, 0x3e00, (lines & 0xffff) >> 12);
}

static void clamp_exposure(const struct xmv200_sns_dev *d,
                           const struct xmv200_sns_ae *ae) {
  if (d->chn < MAX_CHN && st[d->chn].valid & 1 &&
      ae->max_lines < st[d->chn].exposure)
    set_exposure(d, ae->max_lines);
}

/* ---- frame rate ---- */

/* 15 and 12 fps from 30 and 25 by the frame length; the vendor's leaves
 * the exposure as it is */
static void set_fps(const struct xmv200_sns_dev *d,
                    const struct xmv200_sns_attr *a, uint32_t fps,
                    struct xmv200_sns_ae *ae) {
  uint32_t hts, vts = 0x465;

  switch (fps) {
  case 15:
    hts = 0x898, vts = 0x8ca;
    break;
  case 12:
    hts = 0xa50, vts = 0x8ca;
    break;
  case 24:
    hts = 0xabe;
    break;
  case 30:
    hts = 0x898;
    break;
  default:
    hts = 66000 / (fps ? fps : 1);
  }
  ae->full_lines = vts;
  ae->max_lines = vts - 5;
  w(d, 0x320c, hts >> 8);
  w(d, 0x320d, hts & 0xff);
  w(d, 0x320e, vts >> 8);
  w(d, 0x320f, vts & 0xef);
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
  ae->max_lines = vts - 5;
  clamp_exposure(d, ae);
  w(d, 0x320e, vts >> 8);
  w(d, 0x320f, vts & 0xff);
}

/* ---- gain ---- */

static void again_calc(uint32_t gain, uint32_t *idx, uint32_t *real) {
  uint32_t i = 1;

  if (gain < 0x20)
    gain = 0x20;
  else if (gain > 0x3ff)
    gain = 0x400;
  while (i < 6 && gain >= again_val[i])
    i++;
  *idx = i - 1;
  *real = again_val[i - 1];
}

/* 1x .. 4x: doublings as ones from bit 8, the rest << 2 (0x80 = 1x) */
static void dgain_calc(uint32_t gain, uint32_t *reg, uint32_t *real) {
  uint32_t bits = 0, k = 0;

  if (gain < 0x20)
    gain = 0x20;
  else if (gain > 0x7f)
    gain = 0x80;
  while (k < 6 && gain >= 0x40) {
    k++;
    gain >>= 1;
    bits = bits << 1 | 1;
  }
  *reg = gain << 2 | bits << 8;
  *real = gain << k;
}

/*
 * The vendor's takes 0x3e06 (and 0x3e10) from bits 16..23 of the digital
 * gain, where its dgain_calc puts the doublings at bit 8: they never reach
 * the sensor, which so gets 1x .. 2x of digital gain at most. This does the
 * same.
 */
static void set_gain(const struct xmv200_sns_dev *d, uint32_t idx,
                     uint32_t dreg) {
  uint32_t a = again_reg[idx < 7 ? idx : 6], hi = (dreg & 0xffffff) >> 16;

  if (d->chn < MAX_CHN) {
    if (st[d->chn].valid & 2 && st[d->chn].again == idx &&
        st[d->chn].dgain == dreg)
      return;
    st[d->chn].again = idx;
    st[d->chn].dgain = dreg;
    st[d->chn].valid |= 2;
  }
  w(d, 0x3e09, a);
  w(d, 0x3e06, hi);
  w(d, 0x3e07, dreg & 0xff);
  w(d, 0x3e13, a);
  w(d, 0x3e10, hi);
  w(d, 0x3e11, dreg & 0xff);
}

/* ---- mirror and flip ---- */

/* channel 1's sensor is mounted the other way round */
static void set_flip(const struct xmv200_sns_dev *d, int mirror, int flip) {
  uint32_t v;

  r(d, 0x3221); /* the vendor's reads it twice */
  if (d->chn) {
    mirror = !mirror;
    flip = !flip;
  }
  v = r(d, 0x3221);
  v = mirror ? v | 0x06 : v & ~0x06u;
  v = flip ? v | 0xe0 : v & ~0xe0u;
  w(d, 0x3221, v);
}

/* ---- the tuning's runs: the mode's init, then the frame rate and the
 * AE's start together ---- */

static void run(const struct xmv200_sns_dev *d, int n,
                const struct xmv200_sns_ctx *c);

static const struct xmv200_sns_mode modes[] = {
    {"1080p mipi", {0, 0, 0, 0}, sc223ap_mipi, N(sc223ap_mipi)},
    {"1080p mipi max", {0, 0, 0, 0}, sc223ap_mipi_max, N(sc223ap_mipi_max)},
    {"1080p dvp", {1, 0, 0, 0}, sc223ap_dvp, N(sc223ap_dvp)},
};

const struct xmv200_sns_drv xmv200_sns_sc223ap = {
    .name = "SC223AP",
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
    .n_runs = 2,
    .run = run,
};

static void run(const struct xmv200_sns_dev *d, int n,
                const struct xmv200_sns_ctx *c) {
  if (n == 0) {
    xmv200_sns_init_step(&xmv200_sns_sc223ap, d, c);
  } else if (n == 1) {
    xmv200_sns_fps_step(&xmv200_sns_sc223ap, d, c);
    xmv200_sns_ae_step(&xmv200_sns_sc223ap, d, c);
  }
}

/* ---- XM535 ---- */

/*
 * The XM535D1's libsns (Aug 12 2025 d2db8e4), its own design: exposure in
 * half lines (0x3e02 holds (lines & 7) << 5), the analog gain in 7 steps
 * to 58x, the digital gain's doublings at bit 16 (so they reach 0x3e06),
 * no short frame, flip without turning a channel round, and any frame rate
 * by the line length alone (66000 / fps) at 1125 lines.
 */
extern const struct xmv200_sns_reg xm535_sc223ap_mipi[200],
    xm535_sc223ap_dvp[208];

static const uint32_t xm535_again_val[7] = {0x20,  0x39,  0x73, 0xe7,
                                            0x1cf, 0x39e, 0x73d};

static const struct xmv200_sns_mode xm535_modes[] = {
    {"1080p mipi", {0, 0, 0, 0}, xm535_sc223ap_mipi, N(xm535_sc223ap_mipi)},
    {"1080p dvp", {1, 0, 0, 0}, xm535_sc223ap_dvp, N(xm535_sc223ap_dvp)},
};

static void xm535_init(const struct xmv200_sns_dev *d,
                       const struct xmv200_sns_mode *m,
                       const struct xmv200_sns_attr *a) {
  if (d->chn < MAX_CHN)
    memset(&st[d->chn], 0, sizeof(st[d->chn]));
  table(d, m->regs, m->n);
  usleep(10000);
}

static void xm535_ae_default(const struct xmv200_sns_attr *a,
                             struct xmv200_sns_ae *ae) {
  ae->full_lines = 1125;
  ae->max_lines = 1120;
  ae->again_min = 0x20;
  ae->again_max = 0x73e;
  ae->dgain_min = 0x20;
  ae->dgain_max = 0x40;
}

static void xm535_set_exposure(const struct xmv200_sns_dev *d,
                               uint32_t lines) {
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

static void xm535_set_fps(const struct xmv200_sns_dev *d,
                          const struct xmv200_sns_attr *a, uint32_t fps,
                          struct xmv200_sns_ae *ae) {
  uint32_t hts = 66000 / (fps ? fps : 1);

  ae->full_lines = 1125;
  ae->max_lines = 1120;
  w(d, 0x320c, hts >> 8);
  w(d, 0x320d, hts & 0xff);
  w(d, 0x320e, 0x04);
  w(d, 0x320f, 0x65);
  if (d->chn < MAX_CHN)
    st[d->chn].vts = 1125;
}

static void xm535_set_vts(const struct xmv200_sns_dev *d, uint32_t vts,
                          struct xmv200_sns_ae *ae) {
  if (d->chn < MAX_CHN) {
    if (st[d->chn].vts == vts)
      return;
    st[d->chn].vts = vts;
  }
  ae->full_lines = vts;
  ae->max_lines = vts - 5;
  if (d->chn < MAX_CHN && st[d->chn].valid & 1 &&
      ae->max_lines < st[d->chn].exposure)
    xm535_set_exposure(d, ae->max_lines);
  w(d, 0x320e, vts >> 8);
  w(d, 0x320f, vts & 0xff);
}

static void xm535_again_calc(uint32_t gain, uint32_t *idx, uint32_t *real) {
  uint32_t i = 1;

  if (gain > 0x73e)
    gain = 0x73e;
  if (gain < 0x20)
    gain = 0x20;
  while (i < 7 && gain >= xm535_again_val[i])
    i++;
  *idx = i - 1;
  *real = xm535_again_val[i - 1];
}

static void xm535_dgain_calc(uint32_t gain, uint32_t *reg, uint32_t *real) {
  uint32_t bits = 0, k = 0;

  if (gain > 0x7f)
    gain = 0x80;
  if (gain < 0x20)
    gain = 0x20;
  while (k < 6 && gain >= 0x40) {
    k++;
    gain >>= 1;
    bits = (bits & 0x7f) << 1 | 1;
  }
  *reg = gain << 2 | bits << 16;
  *real = gain << k;
}

static void xm535_set_gain(const struct xmv200_sns_dev *d, uint32_t idx,
                           uint32_t dreg) {
  if (d->chn < MAX_CHN) {
    if (st[d->chn].valid & 2 && st[d->chn].again == idx &&
        st[d->chn].dgain == dreg)
      return;
    st[d->chn].again = idx;
    st[d->chn].dgain = dreg;
    st[d->chn].valid |= 2;
  }
  w(d, 0x3e09, again_reg[idx < 7 ? idx : 6]);
  w(d, 0x3e06, (dreg & 0xffffff) >> 16);
  w(d, 0x3e07, dreg & 0xff);
}

static void xm535_set_flip(const struct xmv200_sns_dev *d, int mirror,
                           int flip) {
  uint32_t v = r(d, 0x3221);

  v = mirror ? v | 0x06 : v & ~0x06u;
  v = flip ? v | 0xe0 : v & ~0xe0u;
  w(d, 0x3221, v);
}

static void xm535_run(const struct xmv200_sns_dev *d, int n,
                      const struct xmv200_sns_ctx *c);

const struct xmv200_sns_drv xmv200_sns_sc223ap_xm535 = {
    .name = "SC223AP",
    .family = "xm535",
    .exp_shift = 1,
    .modes = xm535_modes,
    .n_modes = N(xm535_modes),
    .init = xm535_init,
    .ae_default = xm535_ae_default,
    .set_fps = xm535_set_fps,
    .set_vts = xm535_set_vts,
    .set_exposure = xm535_set_exposure,
    .again_calc = xm535_again_calc,
    .dgain_calc = xm535_dgain_calc,
    .set_gain = xm535_set_gain,
    .set_flip = xm535_set_flip,
    .run = xm535_run,
};

/* the AE starts from half the tuning's exposure (half lines) */
static void xm535_run(const struct xmv200_sns_dev *d, int n,
                      const struct xmv200_sns_ctx *c) {
  const struct xmv200_sns_drv *drv = &xmv200_sns_sc223ap_xm535;
  uint32_t idx, reg, real;

  switch (n) {
  case 0:
    xmv200_sns_init_step(drv, d, c);
    break;
  case 1:
    xmv200_sns_fps_step(drv, d, c);
    break;
  case 2:
    xm535_set_exposure(d, c->exp_lines / 2);
    xm535_again_calc(0x20, &idx, &real);
    xm535_dgain_calc(0x20, &reg, &real);
    xm535_set_gain(d, idx, reg);
    break;
  }
}
