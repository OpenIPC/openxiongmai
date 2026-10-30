// SPDX-License-Identifier: MIT
/*
 * SmartSens SC2210 (chip ID 0x2210): the XM530V200 vendor libsns's driver
 * of it (in its isp daemon), as functions. One mode, 1080p MIPI; its table
 * is the vendor's (sns_sc2210_regs.c, tools/xmv200-sns/extract.py).
 *
 * The vendor's gains are in 1/1024 (0x400 = 1x); again_calc and dgain_calc
 * take sns.h's 1/32 and give it back. The analog gain's "index" is the
 * vendor's register pair, 0x3e08 << 8 | 0x3e09.
 *
 * Exposure is 0x3e00..0x3e02 (lines << 4), the analog gain 0x3e08/0x3e09,
 * the digital gain 0x3e06/0x3e07, line length 0x320c/0x320d, frame length
 * 0x320e/0x320f, mirror and flip 0x3221 (0x06, 0x60). Each setter writes
 * only what changed since the last call, as the vendor's do.
 */

#include <string.h>
#include <unistd.h>

#include "sns.h"
#include "xmv200.h"

extern const struct xmv200_sns_reg sc2210_mipi[251];

#define N(a) ((int)(sizeof(a) / sizeof((a)[0])))
#define MAX_CHN 3

/* frame and line length for 12 .. 25 fps (the vendor's table, at 0x8918a) */
static const uint16_t fps_vts[14] = {0xa8c, 0x465, 0x465, 0x8ca, 0x465,
                                     0x465, 0x465, 0x465, 0x465, 0x465,
                                     0x465, 0x465, 0x465, 0x546};
static const uint16_t fps_hts[14] = {0x44c, 0x44c, 0x44c, 0x44c, 0x44c,
                                     0x44c, 0x44c, 0x44c, 0x672, 0x44c,
                                     0x44c, 0x44c, 0x44c, 0x44c};
/* 0x3e08 per doubling of the analog gain */
static const uint8_t again_coarse[6] = {0x03, 0x07, 0x23, 0x27, 0x2f, 0x3f};

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
  ae->full_lines = 1125;
  ae->max_lines = 1118;
  ae->again_min = 0x400 >> 5;
  ae->again_max = 0xd800 >> 5;
  ae->dgain_min = 0x400 >> 5;
  ae->dgain_max = 0x7e00 >> 5;
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

static void clamp_exposure(const struct xmv200_sns_dev *d,
                           const struct xmv200_sns_ae *ae) {
  if (d->chn < MAX_CHN && st[d->chn].valid & 1 &&
      ae->max_lines < st[d->chn].exposure)
    set_exposure(d, ae->max_lines);
}

/* 12 .. 25 fps from the vendor's table; 30 otherwise */
static void set_fps(const struct xmv200_sns_dev *d,
                    const struct xmv200_sns_attr *a, uint32_t fps,
                    struct xmv200_sns_ae *ae) {
  uint32_t i = (fps - 12) & 0xff, vts = 0x465, hts = 0x44c;

  if (i < 14) {
    vts = fps_vts[i];
    hts = fps_hts[i];
  }
  ae->full_lines = vts;
  ae->max_lines = vts - 7;
  clamp_exposure(d, ae);
  w(d, 0x320c, hts >> 8);
  w(d, 0x320d, hts & 0xff);
  w(d, 0x320e, vts >> 8);
  w(d, 0x320f, vts & 0xff);
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
  ae->max_lines = vts - 7;
  clamp_exposure(d, ae);
  w(d, 0x320e, vts >> 8);
  w(d, 0x320f, vts & 0xff);
}

/*
 * The vendor's: in 1/64, halved for each doubling (0x3e08) until below 2x
 * of the step; above 3.375x 0x13 is added to it while it is below 2x.
 */
static void again_calc(uint32_t gain, uint32_t *idx, uint32_t *real) {
  uint32_t g = gain << 5, u = g >> 4, add = g < 0xd81 ? 0 : 0x13, k = 0;

  if (u < 0x40)
    u = 0x40;
  do {
    if (add && u < 0x80)
      u += add;
    if (u < 0x80)
      break;
    u >>= 1;
    k++;
  } while (k != 6);
  *idx = u | (uint32_t)again_coarse[k < 6 ? k : 5] << 8;
  *real = (u << (k + 4)) >> 5;
}

/* in 1/256 from 1x: doublings as ones in 0x3e06, the rest in 0x3e07 */
static void dgain_calc(uint32_t gain, uint32_t *reg, uint32_t *real) {
  uint32_t p = (gain << 5) >> 2, bits = 0, k = 0;

  if (p < 0x100)
    p = 0x100;
  do {
    if (k)
      bits = bits << 1 | 1;
    k++;
    p >>= 1;
  } while (k != 6 && p > 0xff);
  *reg = (p & ~3u) | bits << 8;
  *real = (((p & ~3u) << 2) << k) >> 5;
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
  w(d, 0x3e08, (idx & 0xffff) >> 8);
  w(d, 0x3e09, idx & 0xff);
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

static const struct xmv200_sns_mode modes[] = {
    {"1080p mipi", {0, 0, 0, 0}, sc2210_mipi, N(sc2210_mipi)},
};

const struct xmv200_sns_drv xmv200_sns_sc2210 = {
    .name = "SC2210",
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
