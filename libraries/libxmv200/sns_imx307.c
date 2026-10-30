// SPDX-License-Identifier: MIT
/*
 * Sony IMX307: the XM530V200 vendor libsns's driver of it, as functions.
 * Unverified: no capture has this sensor.
 *
 * Probe (the vendor's): I2C address 0x1a, 16-bit registers, 8-bit values;
 * 0x3003 = 1 (reset), 50 ms, then 0x3008 << 8 | 0x301e = 0xa0b2 and
 * 0x31dc & 6 = 4.
 *
 * Modes: 2 lanes at 25 fps, 4 lanes at 12.5 fps (the vendor's lane field
 * 2; sns.h has no lane count, so only a tuning selects it).
 *
 * The vendor hands exposure and gain to its ISP's per-frame register
 * update (0x3020, 0x3021, 0x3014, 0x3009, each written when it changed);
 * here they are written at once, in that order. Exposure is SHS1 = the
 * frame length set by the frame rate less the lines; gain is in 0.3 dB
 * steps (imx307_gain, 1/1024; sns.h's 1/32), above step 20 with the
 * conversion gain high (0x3009 = 0x12) and 21 steps less. No mirror/flip
 * (the vendor's does nothing).
 */

#include <unistd.h>

#include "sns.h"
#include "xmv200.h"

extern const struct xmv200_sns_reg imx307_mipi_4lane[56], imx307_mipi_2lane[66];
extern const uint32_t imx307_gain[141];

#define N(a) ((int)(sizeof(a) / sizeof((a)[0])))
#define MAX_CHN 3

static uint32_t full[MAX_CHN] = {0x465, 0x465, 0x465}; /* what fps set */
static int32_t last[4] = {-1, -1, -1, -1}; /* 0x3020, 0x3021, 0x3014, 0x3009 */
static uint32_t shs = 0xffff, vts_cache;

static void w(const struct xmv200_sns_dev *d, uint16_t reg, uint32_t val) {
  xmv200_i2c_write(d->bus, d->addr, reg, 2, val & 0xff, 1);
}

static void sync_w(const struct xmv200_sns_dev *d, int i, uint16_t reg,
                   uint32_t val) {
  if (last[i] == (int32_t)val)
    return;
  last[i] = val;
  w(d, reg, val);
}

static void init(const struct xmv200_sns_dev *d,
                 const struct xmv200_sns_mode *m,
                 const struct xmv200_sns_attr *a) {
  w(d, 0x3000, 1);
  for (int i = 0; i < m->n; i++)
    w(d, m->regs[i].reg, m->regs[i].val);
  usleep(20000);
  w(d, 0x3002, 0);
  w(d, 0x304b, 10);
  usleep(20000);
}

static void ae_default(const struct xmv200_sns_attr *a,
                       struct xmv200_sns_ae *ae) {
  ae->full_lines = 0x465;
  ae->max_lines = 0x461;
  ae->again_min = 0x400 / 32;
  ae->again_max = 0xf464 / 32;
  ae->dgain_min = 0x400 / 32;
  ae->dgain_max = 0x400 / 32;
}

static void set_exposure(const struct xmv200_sns_dev *d, uint32_t lines) {
  uint32_t s = (full[d->chn < MAX_CHN ? d->chn : 0] - lines) & 0xffff;

  if (s == shs)
    return;
  shs = s;
  sync_w(d, 0, 0x3020, s & 0xff);
  sync_w(d, 1, 0x3021, s >> 8);
}

/* the line length for 20, 30 or 25 fps; the frame length stays 1125 */
static void set_fps(const struct xmv200_sns_dev *d,
                    const struct xmv200_sns_attr *a, uint32_t fps,
                    struct xmv200_sns_ae *ae) {
  uint32_t hmax = fps == 20 ? 0xce4 : fps == 30 ? 0x898 : 0xa50;

  w(d, 0x301c, (hmax & 0x7c) << 1);
  w(d, 0x301d, hmax >> 7);
  ae->full_lines = 0x465;
  ae->max_lines = 0x461;
  if (d->chn < MAX_CHN)
    full[d->chn] = 0x465;
}

/* SHS1 keeps counting from the frame rate's frame length, as the vendor's */
static void set_vts(const struct xmv200_sns_dev *d, uint32_t vts,
                    struct xmv200_sns_ae *ae) {
  if (vts_cache == vts)
    return;
  vts_cache = vts;
  ae->full_lines = vts;
  ae->max_lines = vts - 4;
  w(d, 0x3018, vts & 0xff);
  w(d, 0x3019, vts >> 8);
  w(d, 0x301a, 0);
}

static void step(uint32_t gain, uint32_t *idx, uint32_t *real) {
  uint32_t g = gain * 32, i;

  if (g > 0x1e7a0) {
    i = 0x8c;
  } else {
    for (i = 1; i < 0x8d && g >= imx307_gain[i]; i++)
      ;
    i--;
  }
  *idx = i;
  *real = imx307_gain[i] / 32;
}

static void again_calc(uint32_t gain, uint32_t *idx, uint32_t *real) {
  step(gain, idx, real);
}

static void dgain_calc(uint32_t gain, uint32_t *reg, uint32_t *real) {
  step(gain, reg, real);
}

static void set_gain(const struct xmv200_sns_dev *d, uint32_t idx,
                     uint32_t dreg) {
  uint32_t a = idx > 0x14 ? idx - 0x15 : idx;

  sync_w(d, 2, 0x3014, (a + dreg) & 0xff);
  sync_w(d, 3, 0x3009, idx < 0x15 ? 2 : 0x12);
}

static void set_flip(const struct xmv200_sns_dev *d, int mirror, int flip) {}

static const struct xmv200_sns_mode modes[] = {
    {"1080p mipi 2 lanes", {0, 0, 0, 0}, imx307_mipi_2lane,
     N(imx307_mipi_2lane)},
    {"1080p mipi 4 lanes", {0, 0, 0, 0}, imx307_mipi_4lane,
     N(imx307_mipi_4lane)},
};

const struct xmv200_sns_drv xmv200_sns_imx307 = {
    .name = "IMX307",
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
