// SPDX-License-Identifier: MIT
/*
 * SOI (JX) F37P: the XM530V200 vendor libsns's driver of it
 * (snsX20/soif37p_cmos.c in its isp daemon), as functions. Not checked
 * against a capture: there is no tuning for it.
 *
 * The vendor's probe (FUN_00046608): I2C address 0x40 (0x44 for channel 0
 * of a two-sensor board), 8-bit registers and values; 0x12 = 0x80 (soft
 * reset), 5 ms, then the chip ID 0x0a << 8 | 0x0b: 0x841 F37P (0x507 is
 * Q03, 0x843 Q03P, sns_q03.c; 0xf37, the F37, it only logs).
 *
 * Exposure is 0x01/0x02 (lines), the analog gain 0x00 (2^n in bits 4..6,
 * 1 + m/16 in bits 0..3), line length 0x20/0x21, frame length 0x22/0x23,
 * mirror and flip 0x12 (0x20, 0x10). There is no digital gain. Each setter
 * writes only what changed since the last call, as the vendor's do.
 */

#include <string.h>
#include <unistd.h>

#include "sns.h"
#include "xmv200.h"

extern const struct xmv200_sns_reg f37p_dvp[111], f37p_mipi_25fps[129],
    f37p_mipi[129];

#define N(a) ((int)(sizeof(a) / sizeof((a)[0])))
#define MAX_CHN 3

static struct {
  int valid;
  uint32_t exposure, again;
} st[MAX_CHN];

static void w(const struct xmv200_sns_dev *d, uint8_t reg, uint32_t val) {
  xmv200_i2c_write(d->bus, d->addr, reg, 1, val & 0xff, 1);
}

static uint32_t r(const struct xmv200_sns_dev *d, uint8_t reg) {
  uint32_t v = 0;

  xmv200_i2c_read(d->bus, d->addr, reg, 1, &v, 1);
  return v & 0xff;
}

/* MIPI at 25 fps has a table of its own */
static void init(const struct xmv200_sns_dev *d,
                 const struct xmv200_sns_mode *m,
                 const struct xmv200_sns_attr *a) {
  const struct xmv200_sns_reg *t = m->regs;
  int n = m->n;

  if (d->chn < MAX_CHN)
    memset(&st[d->chn], 0, sizeof(st[d->chn]));
  if (!m->attr.dvp && a->fps == 25)
    t = f37p_mipi_25fps, n = N(f37p_mipi_25fps);
  for (int i = 0; i < n; i++)
    w(d, t[i].reg, t[i].val);
  usleep(20000);
}

static void ae_default(const struct xmv200_sns_attr *a,
                       struct xmv200_sns_ae *ae) {
  ae->full_lines = 1125;
  ae->max_lines = 1123;
  ae->again_min = 0x20;
  ae->again_max = 0x1f0; /* 15.5x */
  ae->dgain_min = ae->dgain_max = 0x20;
}

/* 12, 15, 20 and 25 fps by frame length from 30 */
static void set_fps(const struct xmv200_sns_dev *d,
                    const struct xmv200_sns_attr *a, uint32_t fps,
                    struct xmv200_sns_ae *ae) {
  uint32_t vts = 1125;

  if (fps == 12 || fps == 15 || fps == 20 || fps == 25)
    vts = 33750 / fps;
  ae->full_lines = vts;
  ae->max_lines = vts - 2;
  w(d, 0x20, 0x00);
  w(d, 0x21, 0x05);
  w(d, 0x22, vts & 0xff);
  w(d, 0x23, vts >> 8);
}

/* the vendor's does nothing */
static void set_vts(const struct xmv200_sns_dev *d, uint32_t vts,
                    struct xmv200_sns_ae *ae) {}

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

/* the vendor's takes 1/1024 and works in 1/16: 2^k steps, then 16ths */
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

static void dgain_calc(uint32_t gain, uint32_t *reg, uint32_t *real) {
  *reg = 0;
  *real = 0x20;
}

static void set_gain(const struct xmv200_sns_dev *d, uint32_t idx,
                     uint32_t dreg) {
  if (d->chn < MAX_CHN) {
    if (st[d->chn].valid & 2 && st[d->chn].again == idx)
      return;
    st[d->chn].again = idx;
    st[d->chn].valid |= 2;
  }
  w(d, 0x00, idx);
}

static void set_flip(const struct xmv200_sns_dev *d, int mirror, int flip) {
  uint32_t v = r(d, 0x12);

  v = mirror ? v | 0x20 : v & ~0x20u;
  v = flip ? v | 0x10 : v & ~0x10u;
  w(d, 0x12, v);
}

static const struct xmv200_sns_mode modes[] = {
    {"1080p mipi", {0, 0, 0, 0}, f37p_mipi, N(f37p_mipi)},
    {"1080p dvp", {1, 0, 0, 0}, f37p_dvp, N(f37p_dvp)},
};

const struct xmv200_sns_drv xmv200_sns_f37p = {
    .name = "F37P",
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
