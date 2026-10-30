// SPDX-License-Identifier: MIT
/*
 * SmartSens SC4336: the XM530V200 vendor libsns's driver of it, as
 * functions. One mode, 4M MIPI (the vendor's table, sns_sc4336_regs.c).
 *
 * Exposure is 0x3e00..0x3e02 (lines << 4), the analog gain 0x3e09 in
 * whole steps (1x .. 32x), the digital gain 0x3e06/0x3e07 as a doubling
 * mask and a fine part, line length 0x320c/0x320d, frame length
 * 0x320e/0x320f, mirror and flip 0x3221 (0x06, 0x60) with 0x3211 = 4 when
 * mirrored. The vendor's analog gain is a whole multiple, its digital gain
 * in 1/128.
 *
 * The vendor's ISP ends a channel's setup with the flip (the tuning's
 * fourth sensor run).
 */

#include <string.h>
#include <unistd.h>

#include "sns.h"
#include "xmv200.h"

extern const struct xmv200_sns_reg sc4336_4m[149];

#define N(a) ((int)(sizeof(a) / sizeof((a)[0])))
#define MAX_CHN 3

static const uint32_t again_val[6] = {1, 2, 4, 8, 16, 32};
static const uint8_t again_reg[6] = {0x00, 0x08, 0x09, 0x0b, 0x0f, 0x1f};

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

static void ae_default(const struct xmv200_sns_attr *a,
                       struct xmv200_sns_ae *ae) {
  ae->full_lines = 1500;
  ae->max_lines = 1490;
  ae->again_min = 0x20;
  ae->again_max = 0x20 << 5;
  ae->dgain_min = 0x20;
  ae->dgain_max = 0x7b1 >> 2;
}

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

/* 25 and 30 fps by line length, 20 otherwise; the frame stays 1500 lines */
static void set_fps(const struct xmv200_sns_dev *d,
                    const struct xmv200_sns_attr *a, uint32_t fps,
                    struct xmv200_sns_ae *ae) {
  uint32_t hts = fps == 25 ? 0xd20 : fps == 30 ? 0xaf0 : 0x1068;

  ae->full_lines = 1500;
  ae->max_lines = 1490;
  w(d, 0x320c, hts >> 8);
  w(d, 0x320d, hts & 0xf8);
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
  ae->max_lines = vts - 10;
  if (d->chn < MAX_CHN && st[d->chn].valid & 1 &&
      ae->max_lines < st[d->chn].exposure)
    set_exposure(d, ae->max_lines);
  w(d, 0x320e, vts >> 8);
  w(d, 0x320f, vts & 0xff);
}

/* gain in 1/32; the vendor's a whole multiple */
static void again_calc(uint32_t gain, uint32_t *idx, uint32_t *real) {
  uint32_t g = gain >> 5, i = 1;

  if (!g)
    g = 1;
  else if (g > 0x1f)
    g = 0x20;
  while (i < 6 && g >= again_val[i])
    i++;
  *idx = again_reg[i - 1];
  *real = again_val[i - 1] << 5;
}

/* gain in 1/32; the vendor's in 1/128 */
static void dgain_calc(uint32_t gain, uint32_t *reg, uint32_t *real) {
  *reg = gain << 2;
  *real = gain;
}

/* the digital gain's top eight bits in 0x3e07 (low two cleared), the
 * doublings above them as ones in 0x3e06 */
static void set_gain(const struct xmv200_sns_dev *d, uint32_t again,
                     uint32_t dgain) {
  uint32_t bits = 0, mask;

  if (d->chn < MAX_CHN) {
    if (st[d->chn].valid & 2 && st[d->chn].again == again &&
        st[d->chn].dgain == dgain)
      return;
    st[d->chn].again = again;
    st[d->chn].dgain = dgain;
    st[d->chn].valid |= 2;
  }
  while (dgain >> bits)
    bits++;
  if (bits < 8)
    bits = 8;
  mask = 0xffu << (bits - 8);
  w(d, 0x3e09, again & 0xff);
  w(d, 0x3e06, ~mask & 0xff);
  w(d, 0x3e07, (mask & dgain) >> (bits - 8) & 0xfc);
}

static void set_flip(const struct xmv200_sns_dev *d, int mirror, int flip) {
  uint32_t v = r(d, 0x3221);

  if (mirror) {
    v |= 0x06;
    w(d, 0x3211, 4);
  } else {
    v &= ~0x06u;
    w(d, 0x3211, 0);
  }
  v = flip ? v | 0x60 : v & ~0x60u;
  w(d, 0x3221, v);
}

static void run(const struct xmv200_sns_dev *d, int n,
                const struct xmv200_sns_ctx *c);

static const struct xmv200_sns_mode modes[] = {
    {"4m mipi", {0, 0, 0, 0}, sc4336_4m, N(sc4336_4m)},
};

const struct xmv200_sns_drv xmv200_sns_sc4336 = {
    .name = "SC4336",
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
    .n_runs = 4,
    .run = run,
};

/* init, frame rate, the AE's start, and the flip: neither mirrored nor
 * flipped, as the vendor's ISP sets it there */
static void run(const struct xmv200_sns_dev *d, int n,
                const struct xmv200_sns_ctx *c) {
  if (n == 0)
    xmv200_sns_init_step(&xmv200_sns_sc4336, d, c);
  else if (n == 1)
    xmv200_sns_fps_step(&xmv200_sns_sc4336, d, c);
  else if (n == 2)
    xmv200_sns_ae_step(&xmv200_sns_sc4336, d, c);
  else if (n == 3)
    set_flip(d, 0, 0);
}

/* ---- XM533 ---- */

/* the XM533 libsns's: the same table and functions; its ISP leaves the
 * flip alone, so a tuning has the three runs */
const struct xmv200_sns_drv xmv200_sns_sc4336_xm533 = {
    .name = "SC4336",
    .family = "xm533",
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
};
