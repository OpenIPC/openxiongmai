// SPDX-License-Identifier: MIT
/*
 * SmartSens SC1345 (720p, MIPI): the XM530V200 vendor libsns's driver of
 * it. The mode table is the vendor's (sns_sc1345_regs.c,
 * tools/xmv200-sns/extract.py).
 *
 * Exposure is in lines, the registers in half lines (0x3e00..0x3e02,
 * lines << 5); the analog gain 0x3e08 (coarse doublings) and 0x3e09
 * (fine), the digital gain 0x3e06/0x3e07, and with the gain two analog
 * registers (0x363a, 0x3622) by the total gain, in a group hold (0x3812);
 * line length 0x320c/0x320d (the frame is 750 lines whatever the frame
 * rate), mirror and flip 0x3221 (0x06, 0x60).
 */

#include <string.h>
#include <unistd.h>

#include "sns.h"
#include "xmv200.h"

extern const struct xmv200_sns_reg sc1345_mipi[89];
extern const struct xmv200_sns_drv xmv200_sns_sc1345;

#define N(a) ((int)(sizeof(a) / sizeof((a)[0])))
#define MAX_CHN 3

/* 0x363a and 0x3622 by the total gain (1/16): below 2x, 4x, 8x, and above */
static const uint8_t logic[4][2] = {
    {0x0c, 0xc6}, {0x16, 0xc6}, {0x16, 0xd6}, {0x1d, 0xd6}};

static struct {
  int valid;
  uint32_t exposure, again, dgain;
} st[MAX_CHN];

/* the analog and digital gains again_calc and dgain_calc last gave; the
 * vendor's keeps them per channel, the AE calls them right before
 * set_gain */
static uint32_t again_real = 0x20, dgain_real = 0x80;

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
  ae->full_lines = 750;
  ae->max_lines = 746;
  ae->again_min = 0x20;
  ae->again_max = 0x1f8;
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

/* the line length: 30 fps 2000, else 60000 / fps */
static void set_fps(const struct xmv200_sns_dev *d,
                    const struct xmv200_sns_attr *a, uint32_t fps,
                    struct xmv200_sns_ae *ae) {
  uint32_t hts = fps == 30 ? 2000 : 60000 / (fps ? fps : 1);

  ae->full_lines = 750;
  ae->max_lines = 746;
  w(d, 0x320c, hts >> 8);
  w(d, 0x320d, hts & 0xff);
}

/* the vendor's has no frame length setter */
static void set_vts(const struct xmv200_sns_dev *d, uint32_t vts,
                    struct xmv200_sns_ae *ae) {}

/* 1x .. 15.75x in 1/32: doublings as ones in bits 8.., the rest below */
static void again_calc(uint32_t gain, uint32_t *idx, uint32_t *real) {
  uint32_t bits = 0, k = 0;

  if (gain < 0x20)
    gain = 0x20;
  else if (gain > 0x1f7)
    gain = 0x1f8;
  while (k < 6 && gain >= 0x40) {
    k++;
    gain >>= 1;
    bits = bits << 1 | 1;
  }
  *idx = gain | bits << 8;
  *real = again_real = gain << k;
}

/* 1x .. 8x in 1/128: doublings as ones in bits 16.., the rest in 0..7 */
static void dgain_calc(uint32_t gain, uint32_t *reg, uint32_t *real) {
  uint32_t bits = 0, k = 0;

  if (gain < 0x80)
    gain = 0x80;
  else if (gain > 0x3ff)
    gain = 0x400;
  while (k < 6 && gain >= 0x100) {
    k++;
    gain >>= 1;
    bits = (bits & 0x7f) << 1 | 1;
  }
  *reg = (gain & ~3u) | bits << 16;
  *real = dgain_real = (gain & ~3u) << k;
}

static void set_gain(const struct xmv200_sns_dev *d, uint32_t again,
                     uint32_t dreg) {
  uint32_t total = again_real * dgain_real >> 8;
  int lv = total < 0x20 ? 0 : total < 0x40 ? 1 : total < 0x80 ? 2 : 3;

  if (d->chn < MAX_CHN) {
    if (st[d->chn].valid & 2 && st[d->chn].again == again &&
        st[d->chn].dgain == dreg)
      return;
    st[d->chn].again = again;
    st[d->chn].dgain = dreg;
    st[d->chn].valid |= 2;
  }
  w(d, 0x3812, 0x00);
  w(d, 0x363a, logic[lv][0]);
  w(d, 0x3622, logic[lv][1]);
  w(d, 0x3812, 0x30);
  w(d, 0x3e08, again >> 6 | 3);
  w(d, 0x3e09, again & 0xff);
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
    xmv200_sns_init_step(&xmv200_sns_sc1345, d, c);
  else if (n == 1)
    xmv200_sns_fps_step(&xmv200_sns_sc1345, d, c);
  else if (n == 2)
    xmv200_sns_ae_step(&xmv200_sns_sc1345, d, &h);
}

static const struct xmv200_sns_mode modes[] = {
    {"720p mipi", {0, 0, 1, 0}, sc1345_mipi, N(sc1345_mipi)},
};

const struct xmv200_sns_drv xmv200_sns_sc1345 = {
    .name = "SC1345",
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
