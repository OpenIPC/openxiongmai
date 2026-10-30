// SPDX-License-Identifier: MIT
/*
 * SmartSens SC5239AI: the XM530V200 vendor libsns's driver of it, as
 * functions. 5M MIPI at 20 fps, or at 25/30 fps (the vendor's tables,
 * sns_sc5239ai_regs.c; it takes the second where the board's frame rate
 * setting reads 'Q'). There is no tuning of it: unverified.
 *
 * Exposure is 0x3e00..0x3e02 in half lines, the analog gain 0x3e08/0x3e09
 * (doublings and a fine part), the digital gain 0x3e06/0x3e07, line length
 * 0x320c/0x320d, frame length 0x320e/0x320f, mirror 0x3221 0x06 and flip
 * 0xe0. Each gain also sets the analog registers 0x3301, 0x3630, 0x3633,
 * 0x3622 and 0x363a by the analog gain, under group hold (0x3812). The
 * vendor's gains are in 1/1024 (analog) and 1/128 (digital).
 *
 * As the vendor's, the exposure and the frame length remember one value
 * for all channels.
 */

#include <string.h>
#include <unistd.h>

#include "sns.h"
#include "xmv200.h"

extern const struct xmv200_sns_reg sc5239ai_5m_20fps[138],
    sc5239ai_5m_25fps[141];

#define N(a) ((int)(sizeof(a) / sizeof((a)[0])))
#define MAX_CHN 3

static struct {
  int valid;
  uint32_t again, dgain;
  const struct xmv200_sns_reg *mode;
} st[MAX_CHN];
static int exp_valid, vts_valid;
static uint32_t exp_last, vts_last;

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
  if (d->chn < MAX_CHN) {
    memset(&st[d->chn], 0, sizeof(st[d->chn]));
    st[d->chn].mode = m->regs;
  }
  exp_valid = vts_valid = 0;
  for (int i = 0; i < m->n; i++)
    w(d, m->regs[i].reg, m->regs[i].val);
  usleep(20000);
}

static void ae_default(const struct xmv200_sns_attr *a,
                       struct xmv200_sns_ae *ae) {
  ae->full_lines = 0x7bc;
  ae->max_lines = 0x7b8;
  ae->again_min = 0x20;
  ae->again_max = 0x3e00 >> 5;
  ae->dgain_min = 0x20;
  ae->dgain_max = 0xfc0 >> 2;
}

static void set_exposure(const struct xmv200_sns_dev *d, uint32_t lines) {
  if (exp_valid && exp_last == lines)
    return;
  exp_last = lines;
  exp_valid = 1;
  w(d, 0x3e02, (lines & 7) << 5);
  w(d, 0x3e01, (lines << 1 & 0xfff) >> 4);
  w(d, 0x3e00, (lines << 1 & 0xffff) >> 12);
}

/* the 25/30 fps mode: 25, or 30 (with 0x3f04/0x3f05); the other: 20 */
static void set_fps(const struct xmv200_sns_dev *d,
                    const struct xmv200_sns_attr *a, uint32_t fps,
                    struct xmv200_sns_ae *ae) {
  uint32_t hts, vts;

  if (d->chn < MAX_CHN && st[d->chn].mode == sc5239ai_5m_25fps) {
    vts = 2000;
    if (fps == 25) {
      hts = 0xcf0;
    } else {
      w(d, 0x3f04, 0x02);
      w(d, 0x3f05, 0xaa);
      hts = 0xac8;
    }
  } else {
    hts = 3000, vts = 0x7bc;
  }
  ae->full_lines = vts;
  ae->max_lines = vts - 4;
  w(d, 0x320c, hts >> 9);
  w(d, 0x320d, hts >> 1 & 0xfc);
  w(d, 0x320e, 0x07);
  w(d, 0x320f, vts & 0xfc);
}

static void set_vts(const struct xmv200_sns_dev *d, uint32_t vts,
                    struct xmv200_sns_ae *ae) {
  if (vts_valid && vts_last == vts)
    return;
  vts_last = vts;
  vts_valid = 1;
  ae->full_lines = vts;
  ae->max_lines = vts - 4;
  if (exp_valid && ae->max_lines < exp_last)
    set_exposure(d, ae->max_lines);
  w(d, 0x320e, vts >> 8);
  w(d, 0x320f, vts & 0xff);
}

/* gain in 1/32, as the vendor's after its >> 5: the doublings as ones in
 * the high byte, the fine part in the low */
static void again_calc(uint32_t gain, uint32_t *idx, uint32_t *real) {
  uint32_t g = gain, bits = 0, k = 0;

  if (g < 0x20)
    g = 0x20;
  while (k < 4 && g >= 0x40) {
    k++;
    g >>= 1;
    bits = bits << 1 | 1;
  }
  *idx = g + bits * 0x100;
  *real = g << k;
}

/* gain in 1/32; the vendor's in 1/128 */
static void dgain_calc(uint32_t gain, uint32_t *reg, uint32_t *real) {
  uint32_t g = gain << 2, bits = 0, k = 0;

  if (g < 0x80)
    g = 0x80;
  while (k < 5 && g >= 0x100) {
    k++;
    g >>= 1;
    bits = bits << 1 | 1;
  }
  *reg = (g & ~1u) + bits * 0x100;
  *real = ((g & ~1u) << k) >> 2;
}

static void set_gain(const struct xmv200_sns_dev *d, uint32_t again,
                     uint32_t dgain) {
  uint32_t c;

  if (d->chn < MAX_CHN) {
    if (st[d->chn].valid && st[d->chn].again == again &&
        st[d->chn].dgain == dgain)
      return;
    st[d->chn].again = again;
    st[d->chn].dgain = dgain;
    st[d->chn].valid = 1;
  }
  c = ((again & 0xffff) >> 8) << 2 | 3;
  w(d, 0x3812, 0x00);
  w(d, 0x3e08, c);
  w(d, 0x3e09, again & 0xff);
  w(d, 0x3e06, (dgain & 0xffff) >> 8);
  w(d, 0x3e07, dgain & 0xff);
  if (c < 0x07) {
    w(d, 0x3301, 0x1c);
    w(d, 0x3630, 0x30);
    w(d, 0x3633, 0x23);
    w(d, 0x3622, 0xf6);
    w(d, 0x363a, 0x83);
  } else if (c < 0x0f) {
    w(d, 0x3301, 0x26);
    w(d, 0x3630, 0x23);
    w(d, 0x3633, 0x33);
    w(d, 0x3622, 0xf6);
    w(d, 0x363a, 0x87);
  } else if (c < 0x1f) {
    w(d, 0x3301, 0x2c);
    w(d, 0x3630, 0x24);
    w(d, 0x3633, 0x43);
    w(d, 0x3622, 0xf6);
    w(d, 0x363a, 0xf9);
  } else {
    w(d, 0x3301, 0x38);
    w(d, 0x3630, 0x28);
    w(d, 0x3633, 0x43);
    w(d, 0x3622, 0xf6);
    w(d, 0x363a, 0x9f);
  }
  w(d, 0x3812, 0x30);
}

static void set_flip(const struct xmv200_sns_dev *d, int mirror, int flip) {
  uint32_t v = r(d, 0x3221);

  v = mirror ? v | 0x06 : v & ~0x06u;
  v = flip ? v | 0xe0 : v & ~0xe0u;
  w(d, 0x3221, v);
}

static void run(const struct xmv200_sns_dev *d, int n,
                const struct xmv200_sns_ctx *c);

/* the mode flag is not in xmv200_sns_attr: the 20 fps mode first */
static const struct xmv200_sns_mode modes[] = {
    {"5m mipi 20 fps", {0, 0, 0, 0}, sc5239ai_5m_20fps,
     N(sc5239ai_5m_20fps)},
    {"5m mipi 25/30 fps", {0, 0, 0, 0}, sc5239ai_5m_25fps,
     N(sc5239ai_5m_25fps)},
};

const struct xmv200_sns_drv xmv200_sns_sc5239ai = {
    .name = "SC5239AI",
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

/* a tuning's exposure is the register's count, in half lines */
static void run(const struct xmv200_sns_dev *d, int n,
                const struct xmv200_sns_ctx *c) {
  struct xmv200_sns_ctx x = *c;

  if (n == 0) {
    xmv200_sns_init_step(&xmv200_sns_sc5239ai, d, c);
  } else if (n == 1) {
    xmv200_sns_fps_step(&xmv200_sns_sc5239ai, d, c);
  } else if (n == 2) {
    x.exp_lines = c->exp_lines / 2;
    xmv200_sns_ae_step(&xmv200_sns_sc5239ai, d, &x);
  }
}
