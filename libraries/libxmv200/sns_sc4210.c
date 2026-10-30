// SPDX-License-Identifier: MIT
/*
 * SmartSens SC4210: the XM530V200 vendor libsns's driver of it, as
 * functions, in its linear mode: MIPI, 4 lanes, 2560x1440 at 1500 lines.
 * The vendor also has a 2-frame HDR mode (sc4210_hdr in
 * sns_sc4210_regs.c), with a short exposure (0x3e04/0x3e05) and the long
 * one's maximum (0x3e23/0x3e24); sns.h has neither, so it is not a mode
 * here. The mode tables are the vendor's (tools/xmv200-sns/extract.py).
 * No tuning has this sensor: not checked against a capture.
 *
 * Exposure is 0x3e00..0x3e02 in half lines (lines << 5), the analog gain
 * 0x3e08 (doublings, and from 2.75x the high conversion gain, 0x20) and
 * 0x3e09 (fine), the digital gain 0x3e06/0x3e07; the vendor writes the
 * same gains to the short frame's 0x3e10..0x3e13 as well. Each setter
 * writes only what changed since the last call, as the vendor's do.
 */

#include <string.h>
#include <unistd.h>

#include "sns.h"
#include "xmv200.h"

extern const struct xmv200_sns_reg sc4210_linear[267];

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

/* the vendor's limits, its gains in 1/1024, here in 1/32 */
static void ae_default(const struct xmv200_sns_attr *a,
                       struct xmv200_sns_ae *ae) {
  ae->full_lines = 1500;
  ae->max_lines = 1496;
  ae->again_min = 0x400 >> 5;
  ae->again_max = 0xae9f >> 5;
  ae->dgain_min = 0x400 >> 5;
  ae->dgain_max = 0x7e00 >> 5;
}

static void set_exposure(const struct xmv200_sns_dev *d, uint32_t lines) {
  uint32_t v = lines * 2;

  if (d->chn < MAX_CHN) {
    if (st[d->chn].valid & 1 && st[d->chn].exposure == lines)
      return;
    st[d->chn].exposure = lines;
    st[d->chn].valid |= 1;
  }
  w(d, 0x3e02, v * 0x10 & 0xe0);
  w(d, 0x3e01, (v & 0xfff) >> 4);
  w(d, 0x3e00, (v & 0xffff) >> 12);
}

static void frame(const struct xmv200_sns_dev *d, uint32_t vts,
                  struct xmv200_sns_ae *ae) {
  ae->full_lines = vts;
  ae->max_lines = vts - 4;
  if (d->chn < MAX_CHN) {
    st[d->chn].vts = vts;
    st[d->chn].valid |= 4;
  }
}

/* by the frame length: 45000 / fps lines */
static void set_fps(const struct xmv200_sns_dev *d,
                    const struct xmv200_sns_attr *a, uint32_t fps,
                    struct xmv200_sns_ae *ae) {
  uint32_t vts = (uint32_t)(45000 / (int)(fps ? fps : 1));

  frame(d, vts, ae);
  w(d, 0x320e, (vts & 0xffff) >> 8);
  w(d, 0x320f, vts & 0xff);
}

static void set_vts(const struct xmv200_sns_dev *d, uint32_t vts,
                    struct xmv200_sns_ae *ae) {
  if (d->chn < MAX_CHN && st[d->chn].valid & 4 && st[d->chn].vts == vts)
    return;
  frame(d, vts, ae);
  if (d->chn < MAX_CHN && st[d->chn].valid & 1 &&
      ae->max_lines < st[d->chn].exposure)
    set_exposure(d, ae->max_lines);
  w(d, 0x320e, vts >> 8);
  w(d, 0x320f, vts & 0xff);
}

/*
 * 1x .. 43.6x (the vendor's 0x400 .. 0xae9f): below 2.75x (0xb00) the fine
 * gain (0x40 = 1x) alone; from there the high conversion gain (bit 31 here,
 * 0x20 in 0x3e08) and the rest divided by 2.75. Doublings as ones in bits
 * 8 and up.
 */
static void again_calc(uint32_t gain, uint32_t *idx, uint32_t *real) {
  uint32_t u = gain << 5, g, bits = 0, k = 0, hcg = u >= 0xb00, v;

  g = hcg ? (u << 6) / 0xb00 : u >> 4;
  if (g < 0x10)
    g = 0x10;
  while (k < 6 && g >= 0x80) {
    k++;
    g >>= 1;
    bits = bits << 1 | 1;
  }
  *idx = bits << 8 | hcg << 31 | (g & 0xff);
  v = g << (k + 4);
  if (hcg)
    v = v * 11 / 4;
  *real = v >> 5;
}

/* 1x .. 31.5x: doublings in 0x3e06, the rest in 0x3e07 (0x80 = 1x) */
static void dgain_calc(uint32_t gain, uint32_t *reg, uint32_t *real) {
  uint32_t g = gain << 2, bits = 0, k = 0; /* the vendor's gain >> 3 */

  while (k < 5 && g >= 0x100) {
    k++;
    g >>= 1;
    bits = bits << 1 | 1;
  }
  *reg = g + bits * 0x100;
  *real = (g << k) >> 2;
}

static void set_gain(const struct xmv200_sns_dev *d, uint32_t areg,
                     uint32_t dreg) {
  uint32_t coarse = ((areg & 0xffff) >> 8) << 2 | (areg >> 31 ? 0x23 : 3);

  if (d->chn < MAX_CHN) {
    if (st[d->chn].valid & 2 && st[d->chn].again == areg &&
        st[d->chn].dgain == dreg)
      return;
    st[d->chn].again = areg;
    st[d->chn].dgain = dreg;
    st[d->chn].valid |= 2;
  }
  w(d, 0x3e08, coarse);
  w(d, 0x3e09, areg & 0xff);
  w(d, 0x3e06, (dreg & 0xffff) >> 8);
  w(d, 0x3e07, dreg & 0xff);
  w(d, 0x3e12, coarse);
  w(d, 0x3e13, areg & 0xff);
  w(d, 0x3e10, (dreg & 0xffff) >> 8);
  w(d, 0x3e11, dreg & 0xff);
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
    {"4 MP mipi linear", {0, 0, 0, 0}, sc4210_linear, N(sc4210_linear)},
};

const struct xmv200_sns_drv xmv200_sns_sc4210 = {
    .name = "SC4210",
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

/* a tuning's exposure would be in the register's half lines */
static void run(const struct xmv200_sns_dev *d, int n,
                const struct xmv200_sns_ctx *c) {
  const struct xmv200_sns_drv *drv = &xmv200_sns_sc4210;
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
