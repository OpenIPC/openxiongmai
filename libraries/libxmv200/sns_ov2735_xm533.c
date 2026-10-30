// SPDX-License-Identifier: MIT
/*
 * OmniVision OV2735 (1080p, MIPI): the XM533 vendor libsns's driver of it,
 * as functions. Unverified: there is no capture of it.
 *
 * Probe (the vendor's, channel 0 only on the XM533): I2C address 0x3c,
 * 8-bit registers and values; 0xfd = 0 (page 0), 0x20 = 0, 1 ms, then chip
 * ID 0x02 << 8 | 0x03 = 0x2735.
 *
 * On page 1 (0xfd = 1), each change latched with 0x01 = 1: exposure
 * 0x03/0x04 in lines, frame length 0x0e/0x0f (0x0d = 0x10), the analog
 * gain 0x24 (1/16), mirror and flip 0x3f bits 0 and 1 (cleared to mirror
 * or flip). The vendor's gains are in 1/1024; sns.h's in 1/32. The table
 * is the vendor's (sns_ov2735_xm533_regs.c).
 */

#include <string.h>
#include <unistd.h>

#include "sns.h"
#include "xmv200.h"

extern const struct xmv200_sns_reg xm533_ov2735_mipi[150];

#define N(a) ((int)(sizeof(a) / sizeof((a)[0])))
#define MAX_CHN 3

static struct {
  int valid;
  uint32_t exposure, again, dgain, vts;
} st[MAX_CHN];

static void w(const struct xmv200_sns_dev *d, uint16_t reg, uint32_t val) {
  xmv200_i2c_write(d->bus, d->addr, reg, 1, val & 0xff, 1);
}

static uint32_t r(const struct xmv200_sns_dev *d, uint16_t reg) {
  uint32_t v = 0;

  xmv200_i2c_read(d->bus, d->addr, reg, 1, &v, 1);
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
  ae->full_lines = 0x465;
  ae->max_lines = 0x461;
  ae->again_min = 0x400 >> 5;
  ae->again_max = 0x3e00 >> 5;
  ae->dgain_min = 0x400 >> 5;
  ae->dgain_max = 0x800 >> 5;
}

static void set_exposure(const struct xmv200_sns_dev *d, uint32_t lines) {
  if (d->chn < MAX_CHN) {
    if (st[d->chn].valid & 1 && st[d->chn].exposure == lines)
      return;
    st[d->chn].exposure = lines;
    st[d->chn].valid |= 1;
  }
  w(d, 0xfd, 1);
  w(d, 0x03, (lines & 0xffff) >> 8);
  w(d, 0x04, lines & 0xff);
  w(d, 0x01, 1);
}

static void frame_length(const struct xmv200_sns_dev *d, uint32_t vts,
                         struct xmv200_sns_ae *ae) {
  ae->full_lines = vts;
  ae->max_lines = vts - 4;
  if (d->chn < MAX_CHN && st[d->chn].valid & 1 &&
      ae->max_lines < st[d->chn].exposure)
    set_exposure(d, ae->max_lines);
  w(d, 0xfd, 1);
  w(d, 0x0d, 0x10);
  w(d, 0x0e, vts >> 8);
  w(d, 0x0f, vts & 0xff);
  w(d, 0x01, 1);
}

/* 12 fps at 2700 lines, the others 33750 / fps */
static void set_fps(const struct xmv200_sns_dev *d,
                    const struct xmv200_sns_attr *a, uint32_t fps,
                    struct xmv200_sns_ae *ae) {
  frame_length(d, fps == 12 ? 0xa8c : 0x83d6 / (fps ? fps : 1), ae);
}

static void set_vts(const struct xmv200_sns_dev *d, uint32_t vts,
                    struct xmv200_sns_ae *ae) {
  if (d->chn < MAX_CHN) {
    if (st[d->chn].valid & 4 && st[d->chn].vts == vts)
      return;
    st[d->chn].vts = vts;
    st[d->chn].valid |= 4;
  }
  frame_length(d, vts, ae);
}

/* 0x24 in 1/16 */
static void again_calc(uint32_t gain, uint32_t *idx, uint32_t *real) {
  *idx = gain >> 1;
  *real = *idx << 1;
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
  w(d, 0xfd, 1);
  w(d, 0x24, idx & 0xff);
  w(d, 0x01, 1);
}

static void set_flip(const struct xmv200_sns_dev *d, int mirror, int flip) {
  uint32_t v;

  w(d, 0xfd, 1);
  v = r(d, 0x3f);
  v = mirror ? v & ~1u : v | 1;
  v = flip ? v & ~2u : v | 2;
  w(d, 0x3f, v);
  w(d, 0x01, 1);
}

static const struct xmv200_sns_mode modes[] = {
    {"1080p mipi", {0, 0, 0, 0}, xm533_ov2735_mipi, N(xm533_ov2735_mipi)},
};

const struct xmv200_sns_drv xmv200_sns_ov2735_xm533 = {
    .name = "OV2735",
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
