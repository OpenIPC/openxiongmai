// SPDX-License-Identifier: MIT
/*
 * SOI (JX) Q03 and Q03P, 2304x1296: the XM530V200 vendor libsns's drivers
 * of them, as functions. Not checked against a capture: there are no
 * tunings for them.
 *
 * The vendor's probe (FUN_00046608), as for the F37P (sns_f37p.c): I2C
 * address 0x40 (0x44 for channel 0 of a two-sensor board), 8-bit
 * registers and values; 0x12 = 0x80, 5 ms, then the chip ID
 * 0x0a << 8 | 0x0b: 0x507 Q03, 0x843 Q03P.
 *
 * Exposure is 0x01/0x02 (lines), the analog gain 0x00 (2^n in bits 4..6,
 * 1 + m/16 in bits 0..3), frame length 0x22/0x23, mirror and flip 0x12
 * (0x20, 0x10). The frame rate changes the line timing (0x20/0x21,
 * 0x27..0x2b, 0x4a), not the frame length. There is no digital gain. Each
 * setter writes only what changed since the last call, as the vendor's
 * do.
 */

#include <unistd.h>

#include "sns.h"
#include "xmv200.h"

extern const struct xmv200_sns_reg q03_mipi[111], q03p_mipi[111];

#define N(a) ((int)(sizeof(a) / sizeof((a)[0])))
#define MAX_CHN 3

static struct {
  int valid;
  uint32_t exposure, again, vts;
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
  usleep(20000);
}

static void limits(struct xmv200_sns_ae *ae, uint32_t full, uint32_t max) {
  ae->full_lines = full;
  ae->max_lines = max;
  ae->again_min = 0x20;
  ae->again_max = 0x1f0; /* 15.5x */
  ae->dgain_min = ae->dgain_max = 0x20;
}

static void q03_ae_default(const struct xmv200_sns_attr *a,
                           struct xmv200_sns_ae *ae) {
  limits(ae, 1375, 1371);
}

static void q03p_ae_default(const struct xmv200_sns_attr *a,
                            struct xmv200_sns_ae *ae) {
  limits(ae, 1350, 1349);
}

/* 20, 25 and (otherwise) 30 fps by the line timing */
static void line_timing(const struct xmv200_sns_dev *d, uint32_t fps) {
  static const uint8_t t[3][7] = {
      {0x46, 0x05, 0x34, 0x04, 0x2f, 0x14, 0xe1}, /* 20 */
      {0x38, 0x04, 0xad, 0x03, 0xa8, 0x13, 0x01}, /* 25 */
      {0x84, 0x03, 0x53, 0x03, 0x4e, 0x13, 0x01}, /* 30 */
  };
  static const uint8_t reg[7] = {0x20, 0x21, 0x27, 0x29, 0x2a, 0x2b, 0x4a};
  const uint8_t *v = t[fps == 20 ? 0 : fps == 25 ? 1 : 2];

  for (int i = 0; i < 7; i++)
    w(d, reg[i], v[i]);
}

static void q03_set_fps(const struct xmv200_sns_dev *d,
                        const struct xmv200_sns_attr *a, uint32_t fps,
                        struct xmv200_sns_ae *ae) {
  if (fps == 0xff)
    return;
  line_timing(d, fps);
  limits(ae, 1375, 1371);
}

static void q03p_set_fps(const struct xmv200_sns_dev *d,
                         const struct xmv200_sns_attr *a, uint32_t fps,
                         struct xmv200_sns_ae *ae) {
  if (fps == 0xff)
    return;
  line_timing(d, fps);
  limits(ae, 1350, 1349);
}

static void set_exposure(const struct xmv200_sns_dev *d, uint32_t lines) {
  if (d->chn < MAX_CHN) {
    if (st[d->chn].valid & 1 && st[d->chn].exposure == lines)
      return;
    st[d->chn].exposure = lines;
    st[d->chn].valid |= 1;
  }
  w(d, 0x01, lines & 0xff);
  w(d, 0x02, (lines & 0xffff) >> 8);
}

/* the longest exposure is the frame length (Q03) or one line less (Q03P);
 * the vendor's waits 100 ms first */
static void vts_write(const struct xmv200_sns_dev *d, uint32_t vts,
                      struct xmv200_sns_ae *ae, uint32_t margin) {
  if (d->chn < MAX_CHN) {
    if (st[d->chn].valid & 4 && st[d->chn].vts == vts)
      return;
    st[d->chn].vts = vts;
    st[d->chn].valid |= 4;
  }
  if (vts > 0xfff)
    vts = 4000;
  ae->full_lines = vts;
  ae->max_lines = vts - margin;
  usleep(100000);
  if (d->chn < MAX_CHN && st[d->chn].valid & 1 &&
      ae->max_lines < st[d->chn].exposure)
    set_exposure(d, ae->max_lines);
  w(d, 0x23, vts >> 8);
  w(d, 0x22, vts & 0xff);
}

static void q03_set_vts(const struct xmv200_sns_dev *d, uint32_t vts,
                        struct xmv200_sns_ae *ae) {
  vts_write(d, vts, ae, 0);
}

static void q03p_set_vts(const struct xmv200_sns_dev *d, uint32_t vts,
                         struct xmv200_sns_ae *ae) {
  vts_write(d, vts, ae, 1);
}

/* the vendor's takes 1/1024 and works in 1/16, up to 15.5x */
static void again_calc(uint32_t gain, uint32_t *idx, uint32_t *real) {
  uint32_t g = gain >> 1, k = 0;

  if (g < 0x10)
    g = 0x10;
  if (g > 0xf7)
    g = 0xf8;
  while (k < 5 && g >= 0x20) {
    g >>= 1;
    k++;
  }
  *idx = (g - 0x10) + k * 0x10;
  *real = (g << k) << 1;
}

static void dgain_calc(uint32_t gain, uint32_t *reg, uint32_t *real) {
  *reg = 0;
  *real = 0x20;
}

/* the vendor's writes again whatever was last given where dreg is set */
static void set_gain(const struct xmv200_sns_dev *d, uint32_t idx,
                     uint32_t dreg) {
  if (d->chn < MAX_CHN) {
    if (!dreg && st[d->chn].valid & 2 && st[d->chn].again == idx)
      return;
    st[d->chn].again = idx;
    st[d->chn].valid |= 2;
  }
  w(d, 0x00, idx);
}

static void set_flip(const struct xmv200_sns_dev *d, int mirror, int flip) {
  uint32_t v = r(d, 0x12) & 0xcf;

  if (mirror)
    v |= 0x20;
  if (flip)
    v |= 0x10;
  w(d, 0x12, v);
}

static const struct xmv200_sns_mode q03_modes[] = {
    {"2304x1296 mipi", {0, 0, 0, 0}, q03_mipi, N(q03_mipi)},
};
static const struct xmv200_sns_mode q03p_modes[] = {
    {"2304x1296 mipi", {0, 0, 0, 0}, q03p_mipi, N(q03p_mipi)},
};

#define DRV(nm, md, aed, fps, vts)                                             \
  {.name = nm,                                                                 \
   .modes = md,                                                                \
   .n_modes = N(md),                                                           \
   .init = init,                                                               \
   .ae_default = aed,                                                          \
   .set_fps = fps,                                                             \
   .set_vts = vts,                                                             \
   .set_exposure = set_exposure,                                               \
   .again_calc = again_calc,                                                   \
   .dgain_calc = dgain_calc,                                                   \
   .set_gain = set_gain,                                                       \
   .set_flip = set_flip}

const struct xmv200_sns_drv xmv200_sns_q03 =
    DRV("Q03", q03_modes, q03_ae_default, q03_set_fps, q03_set_vts);
const struct xmv200_sns_drv xmv200_sns_q03p =
    DRV("Q03P", q03p_modes, q03p_ae_default, q03p_set_fps, q03p_set_vts);
