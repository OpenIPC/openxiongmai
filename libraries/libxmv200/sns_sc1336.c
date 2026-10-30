// SPDX-License-Identifier: MIT
/*
 * SmartSens SC1336 (720p): the XM530V200 vendor libsns's driver of it. The
 * mode tables are the vendor's (sns_sc1336_regs.c,
 * tools/xmv200-sns/extract.py).
 *
 * Exposure is in lines, the registers in half lines (0x3e00..0x3e02,
 * lines << 5); the analog gain 0x3e08 (coarse, with 0x1f) and 0x3e09
 * (fine), the digital gain 0x3e06/0x3e07; line length 0x320c/0x320d
 * (the frame is 750 lines whatever the frame rate), frame length
 * 0x320e/0x320f, mirror and flip 0x3221 (0x06, 0x60).
 */

#include <string.h>
#include <unistd.h>

#include "sns.h"
#include "xmv200.h"

extern const struct xmv200_sns_reg sc1336_dvp[206], sc1336_mipi_50fps[199],
    sc1336_mipi[200];

extern const struct xmv200_sns_drv xmv200_sns_sc1336;

#define N(a) ((int)(sizeof(a) / sizeof((a)[0])))
#define MAX_CHN 3

/* the analog gain's steps (1x .. 64x) and their 0x3e09 values */
static const uint32_t again_val[7] = {0x20, 0x40, 0x80, 0x100,
                                      0x200, 0x400, 0x800};
static const uint8_t again_reg[7] = {0x00, 0x00, 0x08, 0x09, 0x0b, 0x0f, 0x1f};

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

/* the 50 fps MIPI table is the vendor's for its frame rate attribute 100 */
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
  ae->full_lines = 750;
  ae->max_lines = 744;
  ae->again_min = 0x20;
  ae->again_max = 0x800;
  ae->dgain_min = 0x80;
  ae->dgain_max = 0x400;
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

/* the line length: 30 and 60 fps 0x640, 25 and 50 0x780, else 48000 / fps;
 * the frame length stays 750 */
static void set_fps(const struct xmv200_sns_dev *d,
                    const struct xmv200_sns_attr *a, uint32_t fps,
                    struct xmv200_sns_ae *ae) {
  uint32_t hts;

  if (fps == 30 || fps == 60)
    hts = 0x640;
  else if (fps == 25 || fps == 50)
    hts = 0x780;
  else
    hts = 48000 / (fps ? fps : 1);
  ae->full_lines = 750;
  ae->max_lines = 744;
  w(d, 0x320c, hts >> 8);
  w(d, 0x320d, hts & 0xff);
}

static void set_vts(const struct xmv200_sns_dev *d, uint32_t vts,
                    struct xmv200_sns_ae *ae) {
  if (d->chn < MAX_CHN) {
    if (st[d->chn].vts == vts)
      return;
    st[d->chn].vts = vts;
  }
  ae->full_lines = vts;
  ae->max_lines = vts - 6;
  if (d->chn < MAX_CHN && st[d->chn].valid & 1 &&
      ae->max_lines < st[d->chn].exposure)
    set_exposure(d, ae->max_lines);
  w(d, 0x320e, vts >> 8);
  w(d, 0x320f, vts & 0xff);
}

/* the step's index, with 0x20 above 1x: set_gain's 0x3e08 and 0x3e09 */
static void again_calc(uint32_t gain, uint32_t *idx, uint32_t *real) {
  uint32_t i = 1;

  if (gain < 0x20)
    gain = 0x20;
  else if (gain > 0x7ff)
    gain = 0x800;
  while (i < 7 && gain >= again_val[i])
    i++;
  *idx = (i - 1) | (i == 1 ? 0 : 0x20);
  *real = again_val[i - 1];
}

/* 1x .. 31.5x in 1/128: doublings as ones in bits 16.., the rest in 0..7 */
static void dgain_calc(uint32_t gain, uint32_t *reg, uint32_t *real) {
  uint32_t bits = 0, k = 0;

  if (gain < 0x80)
    gain = 0x80;
  else if (gain > 0xfbf)
    gain = 0xfc0;
  while (k < 6 && gain >= 0x100) {
    k++;
    gain >>= 1;
    bits = (bits & 0x7f) << 1 | 1;
  }
  *reg = (gain & ~3u) | bits << 16;
  *real = (gain & ~3u) << k;
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
  w(d, 0x3e08, (again & 0xff) | 0x1f);
  w(d, 0x3e09, (again & 0xf) < 7 ? again_reg[again & 0xf] : 0);
  w(d, 0x3e06, (dreg & 0xffffff) >> 16);
  w(d, 0x3e07, dreg & 0xff);
}

static void set_flip(const struct xmv200_sns_dev *d, int mirror, int flip) {
  uint32_t v = r(d, 0x3221);

  v = mirror ? v | 0x06 : v & ~0x06u;
  v = flip ? v | 0x60 : v & ~0x60u;
  w(d, 0x3221, v);
}

/* the AE starts from the tuning's exposure, which it recorded in half
 * lines */
static void run(const struct xmv200_sns_dev *d, int n,
                const struct xmv200_sns_ctx *c) {
  struct xmv200_sns_ctx h = *c;

  h.exp_lines = c->exp_lines >> 1;
  if (n == 0)
    xmv200_sns_init_step(&xmv200_sns_sc1336, d, c);
  else if (n == 1)
    xmv200_sns_fps_step(&xmv200_sns_sc1336, d, c);
  else if (n == 2)
    xmv200_sns_ae_step(&xmv200_sns_sc1336, d, &h);
}

static const struct xmv200_sns_mode modes[] = {
    {"720p mipi", {0, 0, 1, 0}, sc1336_mipi, N(sc1336_mipi)},
    {"720p mipi 50 fps", {0, 0, 1, 0}, sc1336_mipi_50fps,
     N(sc1336_mipi_50fps)},
    {"720p dvp", {1, 0, 1, 0}, sc1336_dvp, N(sc1336_dvp)},
};

const struct xmv200_sns_drv xmv200_sns_sc1336 = {
    .name = "SC1336",
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
