// SPDX-License-Identifier: MIT
/*
 * SOI (JX) H63 and H63P, 1280x720: the XM530V200 vendor libsns's drivers
 * of them, as functions. Not checked against a capture: there are no
 * tunings for them.
 *
 * The vendor's probe (FUN_00046608): I2C address 0x40 (0x42 for channel 1
 * of a two-sensor board), 8-bit registers and values; 0x12 = 0x80 (soft
 * reset), 5 ms, then the chip ID 0x0a << 8 | 0x0b: 0xa63 H63, 0x848 H63P.
 *
 * Exposure is 0x01/0x02 (lines), written through the group registers
 * 0xc0..0xc3 and latched by 0x1f bit 7; the analog gain 0x00 (2^n in bits
 * 4..6, 1 + m/16 in bits 0..3); line length 0x20/0x21, frame length
 * 0x22/0x23; mirror and flip 0x12 (0x20, 0x10), both set when off (the
 * sensor's default is the other way round). There is no digital gain.
 */

#include <unistd.h>

#include "sns.h"
#include "xmv200.h"

extern const struct xmv200_sns_reg h63_dvp[96], h63_mipi[106],
    h63p_dvp[98], h63p_mipi[108];

#define N(a) ((int)(sizeof(a) / sizeof((a)[0])))
#define MAX_CHN 3

static struct {
  int valid;
  uint32_t exposure, vts;
} st[MAX_CHN];

static void w(const struct xmv200_sns_dev *d, uint8_t reg, uint32_t val) {
  xmv200_i2c_write(d->bus, d->addr, reg, 1, val & 0xff, 1);
}

static uint32_t r(const struct xmv200_sns_dev *d, uint8_t reg) {
  uint32_t v = 0;

  xmv200_i2c_read(d->bus, d->addr, reg, 1, &v, 1);
  return v & 0xff;
}

static void init(const struct xmv200_sns_dev *d,
                 const struct xmv200_sns_mode *m,
                 const struct xmv200_sns_attr *a) {
  if (d->chn < MAX_CHN)
    st[d->chn].valid = 0;
  for (int i = 0; i < m->n; i++)
    w(d, m->regs[i].reg, m->regs[i].val);
  if (!m->attr.dvp)
    usleep(20000);
}

static void ae_default(const struct xmv200_sns_attr *a,
                       struct xmv200_sns_ae *ae) {
  ae->full_lines = 750;
  ae->max_lines = 745;
  ae->again_min = 0x20;
  ae->again_max = 0x3e0; /* 31x */
  ae->dgain_min = ae->dgain_max = 0x20;
}

static void frame(const struct xmv200_sns_dev *d, uint32_t vts,
                  struct xmv200_sns_ae *ae) {
  ae->full_lines = vts;
  ae->max_lines = vts - 5;
  w(d, 0x22, vts & 0xff);
  w(d, 0x23, vts >> 8);
}

static void h63_set_fps(const struct xmv200_sns_dev *d,
                        const struct xmv200_sns_attr *a, uint32_t fps,
                        struct xmv200_sns_ae *ae) {
  uint32_t hts, vts;

  switch (fps) {
  case 20:
    w(d, 0x20, 0xe8);
    w(d, 0x21, 0x03);
    w(d, 0x22, 0x65);
    w(d, 0x23, 0x04);
    w(d, 0x27, 0x03);
    w(d, 0x2a, 0xfa);
    w(d, 0x2b, 0x12);
    hts = 2000, vts = 1125;
    break;
  case 12:
    hts = 0x960, vts = 1500;
    break;
  case 15:
    hts = 2000, vts = 1500;
    break;
  case 24:
    hts = 0x9c4, vts = 750;
    break;
  case 30:
    hts = 2000, vts = 750;
    break;
  default: /* 25 */
    hts = 0x960, vts = 750;
  }
  w(d, 0x20, (hts >> 1) & 0xfa);
  w(d, 0x21, hts >> 9);
  frame(d, vts, ae);
}

/* the line timing for 12..30 fps, 25 the other */
static void h63p_set_fps(const struct xmv200_sns_dev *d,
                         const struct xmv200_sns_attr *a, uint32_t fps,
                         struct xmv200_sns_ae *ae) {
  uint32_t vts;

  switch (fps) {
  case 15:
    vts = 1500;
    break;
  case 12:
    vts = 1800;
    break;
  case 20:
    vts = 1125;
    break;
  case 30:
    vts = 750;
    break;
  default:
    vts = 900;
  }
  if (vts == 900) {
    w(d, 0x27, 0x34);
    w(d, 0x29, 0x03);
    w(d, 0x2a, 0x2b);
    w(d, 0x2b, 0x13);
  } else {
    w(d, 0x27, 0xd4);
    w(d, 0x29, 0x02);
    w(d, 0x2a, 0xcb);
    w(d, 0x2b, 0x12);
  }
  w(d, 0x20, 0xc0);
  w(d, 0x21, 0x03);
  frame(d, vts, ae);
}

static void set_exposure(const struct xmv200_sns_dev *d, uint32_t lines) {
  uint32_t grp;

  if (d->chn < MAX_CHN) {
    if (st[d->chn].valid & 1 && st[d->chn].exposure == lines)
      return;
    st[d->chn].exposure = lines;
    st[d->chn].valid |= 1;
  }
  grp = r(d, 0x1f);
  w(d, 0xc0, 0x01);
  w(d, 0xc1, lines & 0xff);
  w(d, 0xc2, 0x02);
  w(d, 0xc3, (lines & 0xffff) >> 8);
  w(d, 0x1f, grp | 0x80);
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
  w(d, 0x23, vts >> 8);
  w(d, 0x22, vts & 0xff);
}

/* the vendor's takes 1/1024 and works in 1/16 */
static void again_calc(uint32_t gain, uint32_t *idx, uint32_t *real) {
  uint32_t g = gain >> 1, k = 0;

  if (g < 0x10)
    g = 0x10;
  while (k < 5 && g >= 0x20) {
    g >>= 1;
    k++;
  }
  *idx = (g - 0x10) + k * 0x10;
  *real = (g << k) << 1;
}

/* the vendor's passes it through; its limits keep it at 1x */
static void dgain_calc(uint32_t gain, uint32_t *reg, uint32_t *real) {
  *reg = gain;
  *real = gain;
}

/* the vendor's writes every time */
static void set_gain(const struct xmv200_sns_dev *d, uint32_t idx,
                     uint32_t dreg) {
  w(d, 0x00, idx);
}

static void set_flip(const struct xmv200_sns_dev *d, int mirror, int flip) {
  uint32_t v = r(d, 0x12) & 0xcf;

  if (!mirror)
    v |= 0x20;
  if (!flip)
    v |= 0x10;
  w(d, 0x12, v);
}

static const struct xmv200_sns_mode h63_modes[] = {
    {"720p mipi", {0, 0, 1, 0}, h63_mipi, N(h63_mipi)},
    {"720p dvp", {1, 0, 1, 0}, h63_dvp, N(h63_dvp)},
};
static const struct xmv200_sns_mode h63p_modes[] = {
    {"720p mipi", {0, 0, 1, 0}, h63p_mipi, N(h63p_mipi)},
    {"720p dvp", {1, 0, 1, 0}, h63p_dvp, N(h63p_dvp)},
};

/* the tuning's runs: init, frame rate, the AE's start, and the flip */
static const char *const steps[] = {"i", "f", "a", "p"};

#define DRV(nm, md, fps)                                                       \
  {.name = nm,                                                                 \
   .n_runs = 4,                                                                \
   .steps = steps,                                                             \
   .modes = md,                                                                \
   .n_modes = N(md),                                                           \
   .init = init,                                                               \
   .ae_default = ae_default,                                                   \
   .set_fps = fps,                                                             \
   .set_vts = set_vts,                                                         \
   .set_exposure = set_exposure,                                               \
   .again_calc = again_calc,                                                   \
   .dgain_calc = dgain_calc,                                                   \
   .set_gain = set_gain,                                                       \
   .set_flip = set_flip}

const struct xmv200_sns_drv xmv200_sns_h63 = DRV("H63", h63_modes, h63_set_fps);
const struct xmv200_sns_drv xmv200_sns_h63p =
    DRV("H63P", h63p_modes, h63p_set_fps);
