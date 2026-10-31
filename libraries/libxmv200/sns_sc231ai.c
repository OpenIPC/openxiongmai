// SPDX-License-Identifier: MIT
/*
 * SmartSens SC231AI: the XM530V200 vendor libsns's driver of it
 * (snsX20/sc231ai_cmos.c in its isp daemon). One mode, 1080p linear over
 * MIPI; its table is the vendor's (sns_sc231ai_regs.c,
 * tools/xmv200-sns/extract.py).
 *
 * Exposure is in lines, 0x3e00..0x3e02 holding half lines << 4. Gain:
 * 0x3e08/0x3e09 analog (bit 7 of 0x3e08 the high conversion gain, 3.7x),
 * 0x3e06/0x3e07 digital. Line length 0x320c/0x320d (fixed), frame length
 * 0x320e/0x320f, mirror and flip 0x3221 (0x06, 0x60).
 */

#include <string.h>
#include <unistd.h>

#include "sns.h"
#include "xmv200.h"

extern const struct xmv200_sns_reg sc231ai_mipi[147];

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
  ae->full_lines = 0x465;
  ae->max_lines = 0x45f;
  ae->again_min = 0x20; /* the vendor's 0x400 .. 0x1d200 */
  ae->again_max = 0x1d200 >> 5;
  ae->dgain_min = 0x20; /* 0x400 .. 0x4000 */
  ae->dgain_max = 0x4000 >> 5;
}

static void set_exposure(const struct xmv200_sns_dev *d, uint32_t lines) {
  if (d->chn < MAX_CHN) {
    if (st[d->chn].valid & 1 && st[d->chn].exposure == lines)
      return;
    st[d->chn].exposure = lines;
    st[d->chn].valid |= 1;
  }
  w(d, 0x3e02, (lines & 7) << 5);
  w(d, 0x3e01, (lines << 1 & 0xfff) >> 4);
  w(d, 0x3e00, (lines << 1 & 0xffff) >> 12);
}

static void clamp_exposure(const struct xmv200_sns_dev *d,
                           const struct xmv200_sns_ae *ae) {
  if (d->chn < MAX_CHN && st[d->chn].valid & 1 &&
      ae->max_lines < st[d->chn].exposure)
    set_exposure(d, ae->max_lines);
}

/* the vendor's frame lengths for 12 .. 25 fps at the fixed line length:
 * 12, 15, 20 and 25 fps, 30 for the others */
static const uint16_t vts_of[14] = {0xa8c, 0x465, 0x465, 0x8ca, 0x465,
                                    0x465, 0x465, 0x465, 0x697, 0x465,
                                    0x465, 0x465, 0x465, 0x546};

static void set_fps(const struct xmv200_sns_dev *d,
                    const struct xmv200_sns_attr *a, uint32_t fps,
                    struct xmv200_sns_ae *ae) {
  uint8_t i = fps - 12;
  uint32_t vts = i < 14 ? vts_of[i] : 0x465;

  ae->full_lines = vts;
  ae->max_lines = vts - 6;
  clamp_exposure(d, ae);
  w(d, 0x320c, 0x08);
  w(d, 0x320d, 0x98);
  w(d, 0x320e, vts >> 8);
  w(d, 0x320f, vts & 0xff);
  if (d->chn < MAX_CHN)
    st[d->chn].vts = vts;
}

static void set_vts(const struct xmv200_sns_dev *d, uint32_t vts,
                    struct xmv200_sns_ae *ae) {
  if (d->chn < MAX_CHN && st[d->chn].vts == vts)
    return;
  if (d->chn < MAX_CHN)
    st[d->chn].vts = vts;
  ae->full_lines = vts;
  ae->max_lines = vts - 6;
  clamp_exposure(d, ae);
  w(d, 0x320e, vts >> 8);
  w(d, 0x320f, vts & 0xff);
}

/*
 * The analog gain in the vendor's 1/1024 (here 1/32 in and out): above
 * 3.7x the high conversion gain (bit 31 of the value), the rest coarse
 * doublings (ones in bits 8..) and 1/32 fine steps.
 */
static void again_calc(uint32_t gain, uint32_t *idx, uint32_t *real) {
  uint32_t g = gain << 5, bits = 0, k = 0, hcg = g > 0xecc;

  if (hcg)
    g = (uint32_t)(((uint64_t)g << 10) / 0xecd);
  g >>= 5;
  if (g < 0x20)
    g = 0x20;
  while (k < 5 && g >= 0x40) {
    k++;
    g >>= 1;
    bits = bits << 1 | 1;
  }
  *idx = bits << 8 | hcg << 31 | g;
  g <<= k + 5;
  if (hcg)
    g = (uint32_t)(((uint64_t)g * 0xecd) >> 10);
  *real = g >> 5;
}

/* the digital gain: doublings (ones in bits 8..) and 1/128 fine steps */
static void dgain_calc(uint32_t gain, uint32_t *reg, uint32_t *real) {
  uint32_t g = gain << 5 >> 3, bits = 0, k = 0;

  if (g < 0x80)
    g = 0x80;
  while (k < 4 && g >= 0x100) {
    k++;
    g >>= 1;
    bits = bits << 1 | 1;
  }
  *reg = (g & ~1u) + bits * 0x100;
  *real = ((g & ~1u) << (k + 3)) >> 5;
}

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
  w(d, 0x3e08, (again & 0xffff) >> 8 | (again >> 31 ? 0x80 : 0));
  w(d, 0x3e09, again & 0xff);
  w(d, 0x3e06, (dreg & 0xffff) >> 8);
  w(d, 0x3e07, dreg & 0xff);
}

static void set_flip(const struct xmv200_sns_dev *d, int mirror, int flip) {
  uint32_t v = r(d, 0x3221);

  v = mirror ? v | 0x06 : v & ~0x06u;
  v = flip ? v | 0x60 : v & ~0x60u;
  w(d, 0x3221, v);
}

static void run(const struct xmv200_sns_dev *d, int n,
                const struct xmv200_sns_ctx *c);

static const struct xmv200_sns_mode modes[] = {
    {"1080p mipi", {0, 0, 0, 0}, sc231ai_mipi, N(sc231ai_mipi)},
};

const struct xmv200_sns_drv xmv200_sns_sc231ai = {
    .name = "SC231AI",
    .exp_shift = 1,
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
    .run = run,
};

/* The tuning's exposure is 0x3e00..0x3e02 >> 4, half lines: the vendor's
 * AE starts from half of it. */
static void run(const struct xmv200_sns_dev *d, int n,
                const struct xmv200_sns_ctx *c) {
  uint32_t idx, reg, real;

  if (n == 0)
    xmv200_sns_init_step(&xmv200_sns_sc231ai, d, c);
  if (n == 1)
    xmv200_sns_fps_step(&xmv200_sns_sc231ai, d, c);
  if (n != 2)
    return;
  set_exposure(d, c->exp_lines / 2);
  again_calc(0x20, &idx, &real);
  dgain_calc(0x20, &reg, &real);
  set_gain(d, idx, reg);
}

/* ---- XM533 ---- */

/*
 * The XM533 libsns's: two tables of its own (linear 25 fps, and a low
 * frequency one a product setting picks), the same functions but the frame
 * rate's: 12 and 15 fps at twice the frame length, 6 fps at four times,
 * the others by line length (66000 / fps) at 1125 lines.
 */
extern const struct xmv200_sns_reg xm533_sc231ai_mipi[151],
    xm533_sc231ai_mipi_lf[151];

static void xm533_set_fps(const struct xmv200_sns_dev *d,
                          const struct xmv200_sns_attr *a, uint32_t fps,
                          struct xmv200_sns_ae *ae) {
  uint32_t hts, vts;

  if (fps == 12)
    hts = 0xa50, vts = 0x8ca;
  else if (fps == 15)
    hts = 0x898, vts = 0x8ca;
  else if (fps == 6)
    hts = 0xa50, vts = 0x1194;
  else
    hts = 66000 / (fps ? fps : 1), vts = 0x465;
  ae->full_lines = vts;
  ae->max_lines = vts - 6;
  clamp_exposure(d, ae);
  w(d, 0x320c, hts >> 8);
  w(d, 0x320d, hts & 0xff);
  w(d, 0x320e, vts >> 8);
  w(d, 0x320f, vts & 0xff);
  if (d->chn < MAX_CHN)
    st[d->chn].vts = vts;
}

static const struct xmv200_sns_mode xm533_modes[] = {
    {"1080p mipi", {0, 0, 0, 0}, xm533_sc231ai_mipi, N(xm533_sc231ai_mipi)},
    {"1080p mipi low frequency", {0, 0, 0, 0}, xm533_sc231ai_mipi_lf,
     N(xm533_sc231ai_mipi_lf)},
};

static void xm533_run(const struct xmv200_sns_dev *d, int n,
                      const struct xmv200_sns_ctx *c);

const struct xmv200_sns_drv xmv200_sns_sc231ai_xm533 = {
    .name = "SC231AI",
    .family = "xm533",
    .exp_shift = 1,
    .modes = xm533_modes,
    .n_modes = N(xm533_modes),
    .init = init,
    .ae_default = ae_default,
    .set_fps = xm533_set_fps,
    .set_vts = set_vts,
    .set_exposure = set_exposure,
    .again_calc = again_calc,
    .dgain_calc = dgain_calc,
    .set_gain = set_gain,
    .set_flip = set_flip,
    .run = xm533_run,
};

static void xm533_run(const struct xmv200_sns_dev *d, int n,
                      const struct xmv200_sns_ctx *c) {
  uint32_t idx, reg, real;

  if (n == 0)
    xmv200_sns_init_step(&xmv200_sns_sc231ai_xm533, d, c);
  if (n == 1)
    xmv200_sns_fps_step(&xmv200_sns_sc231ai_xm533, d, c);
  if (n != 2)
    return;
  set_exposure(d, c->exp_lines / 2);
  again_calc(0x20, &idx, &real);
  dgain_calc(0x20, &reg, &real);
  set_gain(d, idx, reg);
}
