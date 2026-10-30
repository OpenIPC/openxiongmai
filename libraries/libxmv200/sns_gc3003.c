// SPDX-License-Identifier: MIT
/*
 * GalaxyCore GC3003: the XM530V200 vendor libsns's driver of it, as
 * functions. Unverified: no capture has this sensor.
 *
 * Probe (the vendor's): I2C address 0x37, 16-bit registers, 8-bit values;
 * chip ID 0x3f0 << 8 | 0x3f1 = 0x3003. (The same probe prints GC2083 for
 * 0x2083 but registers no driver for it.)
 *
 * Exposure 0xd03/0xd04, frame length 0xd41/0xd42, mirror 0x15 and 0xd15
 * bit 0, flip bit 1. The gain is a step of 25 (gc3003_again_step, 1/64)
 * whose seven analog registers come from gc3003_again_regs, and a fine
 * part in 0xb1/0xb2. The vendor's gain is in 1/64; sns.h's in 1/32.
 */

#include <string.h>
#include <unistd.h>

#include "sns.h"
#include "xmv200.h"

extern const struct xmv200_sns_reg gc3003_mipi[156];
extern const uint32_t gc3003_again_step[25];
extern const uint8_t gc3003_again_regs[175];

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

static void ae_default(const struct xmv200_sns_attr *a,
                       struct xmv200_sns_ae *ae) {
  ae->full_lines = 0x647;
  ae->max_lines = 0x63d;
  ae->again_min = 0x40 / 2;
  ae->again_max = 0x1000 / 2;
  ae->dgain_min = 0x40 / 2;
  ae->dgain_max = 0x40 / 2;
}

static void set_exposure(const struct xmv200_sns_dev *d, uint32_t lines) {
  if (d->chn < MAX_CHN) {
    if (st[d->chn].valid & 1 && st[d->chn].exposure == lines)
      return;
    st[d->chn].exposure = lines;
    st[d->chn].valid |= 1;
  }
  w(d, 0xd03, (lines & 0xffff) >> 8);
  w(d, 0xd04, lines & 0xff);
}

static void clamp_exposure(const struct xmv200_sns_dev *d,
                           const struct xmv200_sns_ae *ae) {
  if (d->chn < MAX_CHN && st[d->chn].valid & 1 &&
      ae->max_lines < st[d->chn].exposure)
    set_exposure(d, ae->max_lines);
}

/* 12 and 15 fps by frame length; 30 otherwise */
static void set_fps(const struct xmv200_sns_dev *d,
                    const struct xmv200_sns_attr *a, uint32_t fps,
                    struct xmv200_sns_ae *ae) {
  uint32_t vts = fps == 12 ? 0x960 : fps == 15 ? 2000 : 0x5dc;

  ae->full_lines = vts;
  ae->max_lines = vts - 10;
  if (d->chn < MAX_CHN)
    st[d->chn].vts = vts;
  w(d, 0xd41, vts >> 8);
  w(d, 0xd42, vts & 0xfc);
}

static void set_vts(const struct xmv200_sns_dev *d, uint32_t vts,
                    struct xmv200_sns_ae *ae) {
  if (d->chn < MAX_CHN) {
    if (st[d->chn].vts == vts)
      return;
    st[d->chn].vts = vts;
  }
  ae->full_lines = vts;
  ae->max_lines = vts - 10;
  clamp_exposure(d, ae);
  w(d, 0xd41, vts >> 8);
  w(d, 0xd42, vts & 0xff);
}

/* the step at or below the gain, and the fine part (1/64) above it */
static void again_calc(uint32_t gain, uint32_t *idx, uint32_t *real) {
  uint32_t g = gain * 2, i = 0, fine;

  if (g < gc3003_again_step[0])
    g = gc3003_again_step[0];
  while (i < 24 && g >= gc3003_again_step[i + 1])
    i++;
  fine = (g << 6) / gc3003_again_step[i];
  *idx = fine | i << 8;
  *real = (gc3003_again_step[i] * fine >> 6) / 2;
}

/* no digital gain: 1x */
static void dgain_calc(uint32_t gain, uint32_t *reg, uint32_t *real) {
  *reg = 0;
  *real = 0x20;
}

static void set_gain(const struct xmv200_sns_dev *d, uint32_t idx,
                     uint32_t dreg) {
  const uint8_t *row = &gc3003_again_regs[((idx & 0xffff) >> 8) * 7];

  if (d->chn < MAX_CHN) {
    if (st[d->chn].valid & 2 && st[d->chn].again == idx &&
        st[d->chn].dgain == dreg)
      return;
    st[d->chn].again = idx;
    st[d->chn].dgain = dreg;
    st[d->chn].valid |= 2;
  }
  w(d, 0xd1, row[0]);
  w(d, 0xd0, row[1]);
  w(d, 0x80, row[2]);
  w(d, 0x155, row[3]);
  w(d, 0xb8, row[4]);
  w(d, 0xb9, row[5]);
  w(d, 0x6e, row[6]);
  w(d, 0xb1, (idx & 0xff) >> 6);
  w(d, 0xb2, (idx & 0x3f) << 2);
}

static void set_flip(const struct xmv200_sns_dev *d, int mirror, int flip) {
  uint32_t a = r(d, 0x15), b = r(d, 0xd15);

  a = mirror ? a | 1 : a & ~1u;
  b = mirror ? b | 1 : b & ~1u;
  a = flip ? a | 2 : a & ~2u;
  b = flip ? b | 2 : b & ~2u;
  w(d, 0x15, a);
  w(d, 0xd15, b);
}

static const struct xmv200_sns_mode modes[] = {
    {"1080p mipi", {0, 0, 0, 0}, gc3003_mipi, N(gc3003_mipi)},
};

const struct xmv200_sns_drv xmv200_sns_gc3003 = {
    .name = "GC3003",
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
