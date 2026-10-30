// SPDX-License-Identifier: MIT
/*
 * SmartSens SC035GS (640x480, global shutter): the XM530V200 vendor
 * libsns's driver of it. The mode tables are the vendor's
 * (sns_sc035gs_regs.c, tools/xmv200-sns/extract.py). No tuning has been
 * captured for it, so nothing checks it.
 *
 * Exposure is in 1/16 lines (0x3e00..0x3e02 as they are); the gains are
 * steps of 1/16 of a doubling (0x400 = 1x, to 15.5x): the analog gain
 * 0x3e08 (doublings) and 0x3e09 (fine), the digital gain 0x3e06/0x3e07,
 * and with them four analog registers by the total gain; frame length
 * 0x320e/0x320f, with the DVP's or MIPI's timing registers that follow it.
 */

#include <string.h>
#include <unistd.h>

#include "sns.h"
#include "xmv200.h"

extern const struct xmv200_sns_reg sc035gs_dvp[111], sc035gs_mipi[118];

#define N(a) ((int)(sizeof(a) / sizeof((a)[0])))
#define MAX_CHN 3

/* four analog registers, and their values by the total gain: below 2x,
 * below 4x, and above */
static const uint16_t logic_reg[4] = {0x3314, 0x3317, 0x3631, 0x3630};
static const uint8_t logic_val[3][4] = {
    {0x6b, 0x14, 0x58, 0x4a}, {0x4f, 0x10, 0x48, 0x4c}, {0x74, 0x15, 0x48, 0x4c}};

static struct {
  int valid;
  uint32_t exposure, again, dgain;
  uint8_t level; /* (logic level << 4); 0xff: not written */
} st[MAX_CHN];

/* step i's gain: 1x .. 1.9375x by 1/16, then the same doubled, 4 times */
static uint32_t step_gain(uint32_t i) {
  return (0x400 + (i & 15) * 0x40) << (i >> 4);
}

static void w(const struct xmv200_sns_dev *d, uint16_t reg, uint32_t val) {
  xmv200_i2c_write(d->bus, d->addr, reg, 2, val & 0xff, 1);
}

/* both: the table, then three registers */
static void init(const struct xmv200_sns_dev *d,
                 const struct xmv200_sns_mode *m,
                 const struct xmv200_sns_attr *a) {
  if (d->chn < MAX_CHN) {
    memset(&st[d->chn], 0, sizeof(st[d->chn]));
    st[d->chn].level = 0xff;
  }
  for (int i = 0; i < m->n; i++)
    w(d, m->regs[i].reg, m->regs[i].val);
  usleep(10000);
  w(d, 0x4418, 0x0a);
  w(d, 0x363d, 0x10);
  w(d, 0x4419, 0x80);
}

static void ae_default(const struct xmv200_sns_attr *a,
                       struct xmv200_sns_ae *ae) {
  ae->full_lines = a->dvp ? 0x840 : 0x83e;
  ae->max_lines = 0x8380;
  ae->again_min = 0x400;
  ae->again_max = 0x3e00;
  ae->dgain_min = 0x400;
  ae->dgain_max = 0x1f00;
}

static void set_exposure(const struct xmv200_sns_dev *d, uint32_t v) {
  if (d->chn < MAX_CHN) {
    if (st[d->chn].valid & 1 && st[d->chn].exposure == v)
      return;
    st[d->chn].exposure = v;
    st[d->chn].valid |= 1;
  }
  w(d, 0x3e02, v & 0xff);
  w(d, 0x3e01, (v & 0xffff) >> 8);
  w(d, 0x3e00, (v & 0xffffff) >> 16);
}

/* the frame length for 30 fps (0x840 DVP, 0x83e MIPI), lengthened for 25;
 * anything else is 20 fps. The exposure limit stays the vendor's for the
 * frame rate, in 1/16 lines. */
static void set_fps(const struct xmv200_sns_dev *d,
                    const struct xmv200_sns_attr *a, uint32_t fps,
                    struct xmv200_sns_ae *ae) {
  uint32_t vts = a->dvp ? 0x840 : 0x83e, lim = 0x83e;

  if (fps == 25)
    vts = vts * 30 / 25, lim = 0x9e4;
  else if (fps != 30)
    vts = vts * 30 / 20, lim = 0xc5d;
  ae->full_lines = vts;
  ae->max_lines = (lim - 6) * 16;
  w(d, 0x320e, vts >> 8);
  w(d, 0x320f, vts & 0xff);
  if (a->dvp) {
    w(d, 0x3252, (vts - 8) >> 8);
    w(d, 0x3253, (vts - 8) & 0xff);
    w(d, 0x3217, (vts - 5) >> 8);
    w(d, 0x3218, (vts - 5) & 0xff);
  } else {
    w(d, 0x3228, (vts - 6) >> 8);
    w(d, 0x3229, (vts - 6) & 0xff);
  }
}

/* the vendor's has no frame length setter */
static void set_vts(const struct xmv200_sns_dev *d, uint32_t vts,
                    struct xmv200_sns_ae *ae) {}

/* the highest step at or below gain; analog and digital alike */
static void step_calc(uint32_t gain, uint32_t *idx, uint32_t *real) {
  uint32_t i = 63;

  if (gain < step_gain(63))
    for (i = 1; gain >= step_gain(i); i++)
      ;
  if (gain < step_gain(63))
    i--;
  *idx = i;
  *real = step_gain(i);
}

static uint32_t ones(uint32_t n) {
  uint32_t v = 0;

  while (n--)
    v = v << 1 | 1;
  return v;
}

static void set_gain(const struct xmv200_sns_dev *d, uint32_t again,
                     uint32_t dgain) {
  uint32_t total, lv;

  if (d->chn < MAX_CHN) {
    if (st[d->chn].valid & 2 && st[d->chn].again == again &&
        st[d->chn].dgain == dgain)
      return;
    st[d->chn].again = again;
    st[d->chn].dgain = dgain;
    st[d->chn].valid |= 2;
  }
  total = (uint32_t)((uint64_t)step_gain(dgain & 63) *
                         step_gain(again & 63) >> 16);
  lv = total < 0x20 ? 0 : total < 0x40 ? 1 : 2;
  if (d->chn >= MAX_CHN || st[d->chn].level & 0xf ||
      st[d->chn].level >> 4 != lv) {
    if (d->chn < MAX_CHN)
      st[d->chn].level = lv << 4;
    for (int i = 0; i < 4; i++)
      w(d, logic_reg[i], logic_val[lv][i]);
  }
  w(d, 0x3e08, ones(again >> 4) << 2 | 3);
  w(d, 0x3e09, (again & 0xf) | 0x10);
  w(d, 0x3e06, ones(dgain >> 4) | 0xc);
  w(d, 0x3e07, ((dgain & 0xf) + 0x10) * 8);
}

/* the vendor's does nothing */
static void set_flip(const struct xmv200_sns_dev *d, int mirror, int flip) {}

static const struct xmv200_sns_mode modes[] = {
    {"480p mipi", {0, 0, 0, 0}, sc035gs_mipi, N(sc035gs_mipi)},
    {"480p dvp", {1, 0, 0, 0}, sc035gs_dvp, N(sc035gs_dvp)},
};

const struct xmv200_sns_drv xmv200_sns_sc035gs = {
    .name = "SC035GS",
    .modes = modes,
    .n_modes = N(modes),
    .init = init,
    .ae_default = ae_default,
    .set_fps = set_fps,
    .set_vts = set_vts,
    .set_exposure = set_exposure,
    .again_calc = step_calc,
    .dgain_calc = step_calc,
    .set_gain = set_gain,
    .set_flip = set_flip,
};
