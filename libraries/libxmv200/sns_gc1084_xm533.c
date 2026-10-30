// SPDX-License-Identifier: MIT
/*
 * GalaxyCore GC1084 (720p, MIPI): the XM533 vendor libsns's driver of it,
 * as functions. Unverified: there is no capture of it.
 *
 * Probe (the vendor's, channel 0 only on the XM533): I2C address 0x37,
 * 16-bit registers, 8-bit values; chip ID 0x3f0 << 8 | 0x3f1 = 0x1084.
 *
 * Exposure 0x0d03/0x0d04 in lines, frame length 0x0d41/0x0d42 (10 lines
 * of margin), the analog gain one of 25 steps (0x00d0/0x00d1, 0x0dc1 under
 * 0x031d, 0x0155, 0x00b8/0x00b9 from the vendor's table) and a fine part
 * in 0x00b1/0x00b2; no digital gain. Mirror and flip 0x0015 and 0x0d15
 * (bits 0 and 1). The vendor's gains are in 1/64; sns.h's in 1/32. The
 * tables are the vendor's (sns_gc1084_xm533_regs.c).
 */

#include <string.h>
#include <unistd.h>

#include "sns.h"
#include "xmv200.h"

extern const struct xmv200_sns_reg xm533_gc1084_mipi[114];
extern const uint32_t xm533_gc1084_again[26];
extern const uint8_t xm533_gc1084_again_regs[156];

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

/* 15 and 30 fps on the shorter line (0x780), 12 and the others (18750 /
 * fps lines) on the longer */
static void set_fps(const struct xmv200_sns_dev *d,
                    const struct xmv200_sns_attr *a, uint32_t fps,
                    struct xmv200_sns_ae *ae) {
  uint32_t vts;
  int short_line = 0;

  if (fps == 15)
    vts = 0x5dc, short_line = 1;
  else if (fps == 30)
    vts = 0x2ee, short_line = 1;
  else if (fps == 12)
    vts = 0x5dc;
  else
    vts = 0x493e / (fps ? fps : 1);
  ae->full_lines = vts;
  ae->max_lines = vts - 10;
  if (d->chn < MAX_CHN) {
    st[d->chn].vts = vts;
    st[d->chn].valid |= 4;
  }
  w(d, 0x0d41, vts >> 8);
  w(d, 0x0d42, vts & 0xff);
  w(d, 0x03f8, short_line ? 0x36 : 0x3f);
  w(d, 0x0d05, short_line ? 0x09 : 0x0d);
  w(d, 0x0d06, short_line ? 0x60 : 0x20);
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

/* the step at or below the gain, and the rest in 1/64 of it: step << 8 |
 * fine */
static void again_calc(uint32_t gain, uint32_t *idx, uint32_t *real) {
  uint32_t g = gain << 1, i = 0, fine;

  if (g < 0x40)
    g = 0x40;
  while (i < 25 && (g < xm533_gc1084_again[i] || xm533_gc1084_again[i + 1] <= g))
    i++;
  fine = (g << 6) / xm533_gc1084_again[i];
  *idx = fine | i << 8;
  *real = (xm533_gc1084_again[i] * fine >> 6) >> 1;
}

static void dgain_calc(uint32_t gain, uint32_t *reg, uint32_t *real) {
  *reg = 0;
  *real = 0x20;
}

static void set_gain(const struct xmv200_sns_dev *d, uint32_t idx,
                     uint32_t dreg) {
  const uint8_t *s = &xm533_gc1084_again_regs[((idx & 0xffff) >> 8) * 6];

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
    {"720p mipi", {0, 0, 1, 0}, xm533_gc1084_mipi, N(xm533_gc1084_mipi)},
};

/* the probe: GalaxyCore: the chip ID at 0x3f0/0x3f1 */
static uint32_t probe(int bus, uint8_t addr) {
  return xmv200_sns_id_pair(bus, addr, 2, 0x3f0, 0x3f1) == 0x1084 ? 0x1084 : 0;
}

const struct xmv200_sns_drv xmv200_sns_gc1084_xm533 = {
    .probe = probe,
    .probe_addr = {0x37},
    .name = "GC1084",
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
