// SPDX-License-Identifier: MIT
/*
 * SmartSens SC1346 and SC1A4T (720p): the XM530V200 vendor libsns's driver
 * of them, one for both (it tells them apart by chip ID only to log it).
 * The mode tables are the vendor's (sns_sc1346_regs.c,
 * tools/xmv200-sns/extract.py).
 *
 * Exposure is 0x3e00..0x3e02 (lines << 4), the coarse analog gain 0x3e09,
 * the fine gain 0x3e06/0x3e07, line length 0x320c/0x320d, frame length
 * 0x320e/0x320f, mirror and flip 0x3221 (0x06, 0x60).
 */

#include <string.h>
#include <unistd.h>

#include "sns.h"
#include "xmv200.h"

extern const struct xmv200_sns_reg sc1346_mipi[130], sc1346_dvp[140];

#define N(a) ((int)(sizeof(a) / sizeof((a)[0])))
#define MAX_CHN 3

/* the analog gain's steps (1x .. 32x) and their 0x3e09 values */
static const uint32_t again_val[6] = {0x20, 0x40, 0x80, 0x100, 0x200, 0x400};
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

static void table(const struct xmv200_sns_dev *d,
                  const struct xmv200_sns_reg *t, int n) {
  for (int i = 0; i < n; i++)
    w(d, t[i].reg, t[i].val);
}

/*
 * The vendor's takes the table by channel, not by the mode: channel 0's
 * sensor MIPI, the others' DVP (21 fps the DVP's with 0x3641 = 2). It also
 * writes 0x320b = 0xda on channel 0 when the board's resolution attribute
 * is 1, which the captured boards' is not.
 */
static void init(const struct xmv200_sns_dev *d,
                 const struct xmv200_sns_mode *m,
                 const struct xmv200_sns_attr *a) {
  if (d->chn < MAX_CHN)
    memset(&st[d->chn], 0, sizeof(st[d->chn]));
  if (!d->chn) {
    table(d, sc1346_mipi, N(sc1346_mipi));
  } else {
    table(d, sc1346_dvp, N(sc1346_dvp));
    usleep(100);
    if (a->fps == 21)
      w(d, 0x3641, 0x02);
  }
  usleep(20000);
}

static void ae_default(const struct xmv200_sns_attr *a,
                       struct xmv200_sns_ae *ae) {
  ae->full_lines = 750;
  ae->max_lines = 744;
  ae->again_min = 0x20;
  ae->again_max = 0x400;
  ae->dgain_min = 0x20;
  ae->dgain_max = 0x7e;
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

/* line length and frame length per frame rate; others: 22500 / fps lines */
static void set_fps(const struct xmv200_sns_dev *d,
                    const struct xmv200_sns_attr *a, uint32_t fps,
                    struct xmv200_sns_ae *ae) {
  uint32_t hts = 2000, vts;

  switch (fps) {
  case 20:
    vts = 0x465;
    break;
  case 10:
    vts = 0x1194;
    break;
  case 12:
    hts = 0x960, vts = 0x5dc;
    break;
  case 25:
    hts = 0x960, vts = 0x2ee;
    break;
  case 30:
    vts = 0x2ee;
    break;
  default:
    vts = 22500 / (int)(fps ? fps : 1);
  }
  ae->full_lines = vts;
  ae->max_lines = vts - 6;
  w(d, 0x320c, hts >> 8);
  w(d, 0x320d, hts & 0xf0);
  w(d, 0x320e, vts >> 8);
  w(d, 0x320f, vts & 0xff);
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

/* the step's 0x3e09 value is what set_gain takes */
static void again_calc(uint32_t gain, uint32_t *idx, uint32_t *real) {
  uint32_t i = 1;

  if (gain < 0x20)
    gain = 0x20;
  else if (gain > 0x7ff)
    gain = 0x800;
  while (i < 6 && gain >= again_val[i])
    i++;
  *idx = again_reg[i - 1];
  *real = again_val[i - 1];
}

/* 1x .. 3.9x in 1/32: below 2x gain << 2, from 2x 0x80 and steps of 4,
 * with 0x100 set */
static void dgain_calc(uint32_t gain, uint32_t *reg, uint32_t *real) {
  if (gain < 0x20)
    gain = 0x20;
  if (gain > 0x7d)
    gain = 0x7e;
  if (gain > 0x3f) {
    *reg = (((gain - 0x40) >> 1) * 4 + 0x80) | 0x100;
    gain &= ~1u;
  } else {
    *reg = gain << 2;
  }
  *real = gain;
}

/* 0x3e06 takes bits 16..23 of the digital gain's value, where dgain_calc
 * puts nothing: the vendor's, as it is */
static void set_gain(const struct xmv200_sns_dev *d, uint32_t again,
                     uint32_t dreg) {
  if (d->chn < MAX_CHN) {
    if (st[d->chn].valid & 2 && st[d->chn].again == again &&
        st[d->chn].dgain == dreg)
      return;
    st[d->chn].again = again;
    st[d->chn].dgain = dreg;
    st[d->chn].valid |= 2;
  }
  w(d, 0x3e09, again & 0xff);
  w(d, 0x3e06, (dreg & 0xffffff) >> 16);
  w(d, 0x3e07, dreg & 0xff);
}

static void set_flip(const struct xmv200_sns_dev *d, int mirror, int flip) {
  uint32_t v = r(d, 0x3221);

  v = mirror ? v | 0x06 : v & ~0x06u;
  v = flip ? v | 0x60 : v & ~0x60u;
  w(d, 0x3221, v);
}

static const struct xmv200_sns_mode modes[] = {
    {"720p mipi", {0, 0, 1, 0}, sc1346_mipi, N(sc1346_mipi)},
    {"720p dvp", {1, 0, 1, 0}, sc1346_dvp, N(sc1346_dvp)},
};

#define DRV(nm)                                                                \
  {.name = nm,                                                                 \
   .modes = modes,                                                             \
   .n_modes = N(modes),                                                        \
   .init = init,                                                               \
   .ae_default = ae_default,                                                   \
   .set_fps = set_fps,                                                         \
   .set_vts = set_vts,                                                         \
   .set_exposure = set_exposure,                                               \
   .again_calc = again_calc,                                                   \
   .dgain_calc = dgain_calc,                                                   \
   .set_gain = set_gain,                                                       \
   .set_flip = set_flip}

const struct xmv200_sns_drv xmv200_sns_sc1346 = DRV("SC1346");
const struct xmv200_sns_drv xmv200_sns_sc1a4t = DRV("SC1A4T");

/* ---- XM533 ---- */

/*
 * The XM533 libsns's: the same tables and functions but the frame rate's,
 * 12 and 15 fps at twice the frame length and the others by line length
 * (60000 / fps) at 750 lines.
 */
static void xm533_set_fps(const struct xmv200_sns_dev *d,
                          const struct xmv200_sns_attr *a, uint32_t fps,
                          struct xmv200_sns_ae *ae) {
  uint32_t hts, vts;

  if (fps == 12)
    hts = 0x960, vts = 0x5dc;
  else if (fps == 15)
    hts = 2000, vts = 0x5dc;
  else
    hts = 60000 / (fps ? fps : 1), vts = 0x2ee;
  ae->full_lines = vts;
  ae->max_lines = vts - 6;
  w(d, 0x320c, hts >> 8);
  w(d, 0x320d, hts & 0xff);
  w(d, 0x320e, vts >> 8);
  w(d, 0x320f, vts & 0xff);
}

#define XM533_DRV(nm)                                                          \
  {.name = nm,                                                                 \
   .family = "xm533",                                                          \
   .modes = modes,                                                             \
   .n_modes = N(modes),                                                        \
   .init = init,                                                               \
   .ae_default = ae_default,                                                   \
   .set_fps = xm533_set_fps,                                                   \
   .set_vts = set_vts,                                                         \
   .set_exposure = set_exposure,                                               \
   .again_calc = again_calc,                                                   \
   .dgain_calc = dgain_calc,                                                   \
   .set_gain = set_gain,                                                       \
   .set_flip = set_flip}

const struct xmv200_sns_drv xmv200_sns_sc1346_xm533 = XM533_DRV("SC1346");
const struct xmv200_sns_drv xmv200_sns_sc1a4t_xm533 = XM533_DRV("SC1A4T");
