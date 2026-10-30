// SPDX-License-Identifier: MIT
/*
 * Imagedesign MIS5001: the XM530V200 vendor libsns's driver of it, as
 * functions. Unverified: no capture has this sensor.
 *
 * Probe (the vendor's, shared by the MIS parts): I2C address 0x30 (0x31
 * for a second sensor on a board whose first is at 0x30), 16-bit
 * registers, 8-bit values; 0x3006 = 1, 100 ms, then chip ID 0x3000 << 8 |
 * 0x3001 = 0x1311. MIS4001 reads the same: the vendor takes MIS5001 only
 * where the product preset the channel's type (0xa0).
 *
 * One mode, 15 fps (12 by a longer frame). Exposure 0x3100/0x3101, frame
 * length 0x310c/0x310d (in a 0x3006 = 2 hold; the vendor's frame rate
 * function writes 0x310d as the length & 0x60), analog gain 0x3102
 * (doublings 0..5 and 32 fine steps; the vendor's gain in 1/1024, sns.h's
 * in 1/32), no sensor digital gain. Mirror 0x3007 bit 0 clear, flip bit 1
 * clear.
 */

#include <string.h>
#include <unistd.h>

#include "sns.h"
#include "xmv200.h"

extern const struct xmv200_sns_reg mis5001[178];

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
  ae->full_lines = 0x960;
  ae->max_lines = 0x95e;
  ae->again_min = 0x400 / 32;
  ae->again_max = 0x3e00 / 32;
  ae->dgain_min = 0x80;
  ae->dgain_max = 0x80;
}

static void set_exposure(const struct xmv200_sns_dev *d, uint32_t lines) {
  if (d->chn < MAX_CHN) {
    if (st[d->chn].valid & 1 && st[d->chn].exposure == lines)
      return;
    st[d->chn].exposure = lines;
    st[d->chn].valid |= 1;
  }
  w(d, 0x3101, lines & 0xff);
  w(d, 0x3100, (lines & 0xffff) >> 8);
}

static void clamp_exposure(const struct xmv200_sns_dev *d,
                           const struct xmv200_sns_ae *ae) {
  if (d->chn < MAX_CHN && st[d->chn].valid & 1 &&
      ae->max_lines < st[d->chn].exposure)
    set_exposure(d, ae->max_lines);
}

static void set_fps(const struct xmv200_sns_dev *d,
                    const struct xmv200_sns_attr *a, uint32_t fps,
                    struct xmv200_sns_ae *ae) {
  uint32_t vts = fps == 12 ? 0xb40 : 0x960;

  ae->full_lines = vts;
  ae->max_lines = vts - 2;
  if (d->chn < MAX_CHN)
    st[d->chn].vts = vts;
  w(d, 0x3006, 2);
  usleep(100000);
  w(d, 0x310c, vts >> 8);
  w(d, 0x310d, vts & 0x60);
  w(d, 0x3006, 0);
}

static void set_vts(const struct xmv200_sns_dev *d, uint32_t vts,
                    struct xmv200_sns_ae *ae) {
  if (d->chn < MAX_CHN) {
    if (st[d->chn].vts == vts)
      return;
    st[d->chn].vts = vts;
  }
  ae->full_lines = vts;
  ae->max_lines = vts - 2;
  clamp_exposure(d, ae);
  w(d, 0x3006, 2);
  usleep(100000);
  w(d, 0x310c, vts >> 8);
  w(d, 0x310d, vts & 0xff);
  w(d, 0x3006, 0);
}

static void again_calc(uint32_t gain, uint32_t *idx, uint32_t *real) {
  uint32_t g = gain < 0x20 ? 0x20 : gain, k = 0;

  while (k < 5 && g >= 0x40) {
    g >>= 1;
    k++;
  }
  *idx = g - 0x20 + k * 0x20;
  *real = g << k;
}

static void dgain_calc(uint32_t gain, uint32_t *reg, uint32_t *real) {
  *reg = gain;
  *real = gain;
}

static void set_gain(const struct xmv200_sns_dev *d, uint32_t idx,
                     uint32_t dreg) {
  if (d->chn < MAX_CHN) {
    if (st[d->chn].valid & 2 && st[d->chn].again == idx &&
        st[d->chn].dgain == dreg)
      return;
    st[d->chn].again = idx;
    st[d->chn].dgain = dreg;
    st[d->chn].valid |= 2;
  }
  w(d, 0x3102, idx);
}

static void set_flip(const struct xmv200_sns_dev *d, int mirror, int flip) {
  uint32_t v = r(d, 0x3007);

  v = mirror ? v & ~1u : v | 1;
  v = flip ? v & ~2u : v | 2;
  w(d, 0x3007, v);
}

static const struct xmv200_sns_mode modes[] = {
    {"15fps", {0, 0, 0, 0}, mis5001, N(mis5001)},
};

const struct xmv200_sns_drv xmv200_sns_mis5001 = {
    .name = "MIS5001",
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
