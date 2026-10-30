// SPDX-License-Identifier: MIT
/*
 * GalaxyCore GC1084 (720p, DVP): the XM535D1 vendor libsns's driver of it,
 * as functions. Unverified: there is no capture of it.
 *
 * Probe (the vendor's): I2C address 0x37 (a product preset may give
 * another), 16-bit registers, 8-bit values; chip ID 0x3f0 << 8 | 0x3f1 =
 * 0x1084.
 *
 * As the XM533's (sns_gc1084_xm533.c) but for its DVP table, its frame
 * rate (12 fps 1500 lines on a 0x640 line, else 750 lines on 48000 / fps;
 * 0x0d05/0x0d06 the line length, 0x0d42 masked with 0xfe) and an analog
 * gain search over all 26 steps without a lower bound. Exposure
 * 0x0d03/0x0d04, frame length 0x0d41/0x0d42 (10 lines of margin), the
 * analog gain one of 25 steps (0x00d0/0x00d1, 0x0dc1 under 0x031d, 0x0155,
 * 0x00b8/0x00b9) and a fine part in 0x00b1/0x00b2; no digital gain. Mirror
 * and flip 0x0015 and 0x0d15 (bits 0 and 1). The vendor's gains are in
 * 1/64; sns.h's in 1/32. The tables are the vendor's
 * (sns_gc1084_xm535_regs.c).
 */

#include <string.h>
#include <unistd.h>

#include "sns.h"
#include "xmv200.h"

extern const struct xmv200_sns_reg xm535_gc1084_dvp[112];
extern const uint32_t xm535_gc1084_again[27];
extern const uint8_t xm535_gc1084_again_regs[156];

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
  ae->full_lines = 0x2ee;
  ae->max_lines = 0x2e4;
  ae->again_min = 0x40 >> 1;
  ae->again_max = 0x1000 >> 1;
  ae->dgain_min = 0x20;
  ae->dgain_max = 0x20;
}

static void set_exposure(const struct xmv200_sns_dev *d, uint32_t lines) {
  if (d->chn < MAX_CHN) {
    if (st[d->chn].valid & 1 && st[d->chn].exposure == lines)
      return;
    st[d->chn].exposure = lines;
    st[d->chn].valid |= 1;
  }
  w(d, 0x0d03, (lines & 0xffff) >> 8);
  w(d, 0x0d04, lines & 0xff);
}

/* the vendor's leaves the exposure as it is */
static void set_fps(const struct xmv200_sns_dev *d,
                    const struct xmv200_sns_attr *a, uint32_t fps,
                    struct xmv200_sns_ae *ae) {
  uint32_t hts, vts;

  if (fps == 12) {
    hts = 0x640, vts = 0x5dc;
    ae->max_lines = 0x5d2;
  } else {
    hts = 48000 / (fps ? fps : 1), vts = 0x2ee;
    ae->max_lines = 0x2e4;
  }
  ae->full_lines = vts;
  if (d->chn < MAX_CHN) {
    st[d->chn].vts = vts;
    st[d->chn].valid |= 4;
  }
  w(d, 0x0d41, vts >> 8);
  w(d, 0x0d42, vts & 0xfe);
  w(d, 0x0d05, (hts & 0xfff) >> 8);
  w(d, 0x0d06, hts & 0xff);
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
  w(d, 0x0d41, vts >> 8);
  w(d, 0x0d42, vts & 0xff);
}

/* the step at or below the gain (the table ends 0xffffffff), and the rest
 * in 1/64 of it: step << 8 | fine */
static void again_calc(uint32_t gain, uint32_t *idx, uint32_t *real) {
  uint32_t g = gain << 1, i = 0, fine;

  while (i != 26 &&
         (g < xm535_gc1084_again[i] || xm535_gc1084_again[i + 1] <= g))
    i++;
  fine = (g << 6) / xm535_gc1084_again[i];
  *idx = fine | i << 8;
  *real = (fine * xm535_gc1084_again[i] >> 6) >> 1;
}

static void dgain_calc(uint32_t gain, uint32_t *reg, uint32_t *real) {
  *reg = 0;
  *real = 0x20;
}

static void set_gain(const struct xmv200_sns_dev *d, uint32_t idx,
                     uint32_t dreg) {
  uint32_t step = (idx & 0xffff) >> 8;
  const uint8_t *s = &xm535_gc1084_again_regs[(step < 26 ? step : 25) * 6];

  if (d->chn < MAX_CHN) {
    if (st[d->chn].valid & 2 && st[d->chn].again == idx &&
        st[d->chn].dgain == dreg)
      return;
    st[d->chn].again = idx;
    st[d->chn].dgain = dreg;
    st[d->chn].valid |= 2;
  }
  w(d, 0x00d1, s[0]);
  w(d, 0x00d0, s[1]);
  w(d, 0x031d, 0x2e);
  w(d, 0x0dc1, s[2]);
  w(d, 0x031d, 0x28);
  w(d, 0x0155, s[3]);
  w(d, 0x00b8, s[4]);
  w(d, 0x00b9, s[5]);
  w(d, 0x00b1, (idx & 0xff) >> 6);
  w(d, 0x00b2, (idx & 0x3f) << 2);
}

static void set_flip(const struct xmv200_sns_dev *d, int mirror, int flip) {
  uint32_t a = r(d, 0x0015), b = r(d, 0x0d15);

  a = mirror ? a | 1 : a & ~1u;
  b = mirror ? b | 1 : b & ~1u;
  a = flip ? a | 2 : a & ~2u;
  b = flip ? b | 2 : b & ~2u;
  w(d, 0x0015, a);
  w(d, 0x0d15, b);
}

static const struct xmv200_sns_mode modes[] = {
    {"720p dvp", {1, 0, 1, 0}, xm535_gc1084_dvp, N(xm535_gc1084_dvp)},
};

const struct xmv200_sns_drv xmv200_sns_gc1084_xm535 = {
    .name = "GC1084",
    .family = "xm535",
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
