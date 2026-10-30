// SPDX-License-Identifier: MIT
/*
 * SmartSens SC430AI: the XM530V200 vendor libsns's driver of it, as
 * functions. One mode, 4M MIPI (the vendor's table, sns_sc430ai_regs.c).
 *
 * Exposure is 0x3e00..0x3e02 in half lines, the analog gain 0x3e09 from a
 * table of steps (1x .. 76x), the digital gain 0x3e06/0x3e07 as a doubling
 * mask and a fine part, line length 0x320c/0x320d, frame length
 * 0x320e/0x320f, mirror and flip 0x3221 (0x06, 0x60). The vendor's gains
 * are in 1/1024 (analog) and 1/32 (digital).
 */

#include <string.h>
#include <unistd.h>

#include "sns.h"
#include "xmv200.h"

extern const struct xmv200_sns_reg sc430ai_4m[172];

#define N(a) ((int)(sizeof(a) / sizeof((a)[0])))
#define MAX_CHN 3

/* the analog gain's steps (1/1024) and their 0x3e09 values */
static const uint32_t again_val[8] = {0x400,  0x800,  0x98f,  0x131e,
                                      0x263d, 0x4c7a, 0x98f5, 0x131eb};
static const uint8_t again_reg[8] = {0x00, 0x01, 0x40, 0x48,
                                     0x49, 0x4b, 0x4f, 0x5f};

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
  ae->max_lines = 1495;
  ae->again_min = 0x20;
  ae->again_max = 0x131eb >> 5;
  ae->dgain_min = 0x20;
  ae->dgain_max = 0x7e;
}

/* half lines: bits 3..0 of the line count << 5 in 0x3e02 */
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

/* 25 and 30 fps by line length, 20 otherwise; the frame stays 1500 lines */
static void set_fps(const struct xmv200_sns_dev *d,
                    const struct xmv200_sns_attr *a, uint32_t fps,
                    struct xmv200_sns_ae *ae) {
  uint32_t hts = fps == 25 ? 0x1080 : fps == 30 ? 0xdc0 : 0x14a0;

  ae->full_lines = 1500;
  ae->max_lines = 1495;
  w(d, 0x320c, hts >> 9);
  w(d, 0x320d, hts >> 1 & 0xf0);
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
  ae->max_lines = vts - 5;
  if (d->chn < MAX_CHN && st[d->chn].valid & 1 &&
      ae->max_lines < st[d->chn].exposure)
    set_exposure(d, ae->max_lines);
  w(d, 0x320e, vts >> 8);
  w(d, 0x320f, vts & 0xff);
}

/* gain in 1/32; the vendor's in 1/1024 */
static void again_calc(uint32_t gain, uint32_t *idx, uint32_t *real) {
  uint32_t g = gain << 5, i = 1;

  if (g < 0x400)
    g = 0x400;
  else if (g >= 0x131eb)
    g = 0x131eb;
  while (i < 8 && g >= again_val[i])
    i++;
  *idx = again_reg[i - 1];
  *real = again_val[i - 1] >> 5;
}

static void dgain_calc(uint32_t gain, uint32_t *reg, uint32_t *real) {
  *reg = gain;
  *real = gain;
}

/* the digital gain's top six bits in 0x3e07 (<< 2), the doublings above
 * them as ones in 0x3e06 */
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
  if (bits < 6)
    bits = 6;
  mask = 0xffu << (bits - 6);
  w(d, 0x3e09, again & 0xff);
  w(d, 0x3e06, ~mask & 0xff);
  w(d, 0x3e07, ((dgain & mask) >> (bits - 6) & 0xff) << 2);
}

static void set_flip(const struct xmv200_sns_dev *d, int mirror, int flip) {
  uint32_t v = r(d, 0x3221);

  v = mirror ? v | 0x06 : v & ~0x06u;
  v = flip ? v | 0x60 : v & ~0x60u;
  w(d, 0x3221, v);
}

/* the tuning's exposure is the register's count, in half lines */
static void run(const struct xmv200_sns_dev *d, int n,
                const struct xmv200_sns_ctx *c);

static const struct xmv200_sns_mode modes[] = {
    {"4m mipi", {0, 0, 0, 0}, sc430ai_4m, N(sc430ai_4m)},
};

const struct xmv200_sns_drv xmv200_sns_sc430ai = {
    .name = "SC430AI",
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
  struct xmv200_sns_ctx half = *c;

  if (n == 2) {
    half.exp_lines = c->exp_lines / 2;
    xmv200_sns_ae_step(&xmv200_sns_sc430ai, d, &half);
  } else if (n == 1) {
    xmv200_sns_fps_step(&xmv200_sns_sc430ai, d, c);
  } else if (n == 0) {
    xmv200_sns_init_step(&xmv200_sns_sc430ai, d, c);
  }
}
