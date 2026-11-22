// SPDX-License-Identifier: MIT
/*
 * SmartSens SC2335: the XM530V200 vendor libsns's driver of it
 * (snsX20/sc2335_cmos.c in its isp daemon). One mode, over MIPI; its
 * table is the vendor's (sns_sc2335_regs.c, tools/xmv200-sns/extract.py).
 * Its tuning was captured with the product's FunExt 0 (the single
 * sensor DSP): the X6C-WEQ's dual one (19, DUALV200) runs the BPU offline,
 * which refuses the limit-up scaling SC2335's picture takes there.
 *
 * Exposure is in lines, 0x3e00..0x3e02 holding half lines << 4. Gain:
 * 0x3e08/0x3e09 analog, 0x3e06/0x3e07 digital, and with the total gain
 * the sensor's logic registers 0x5799 and (under group hold, 0x3812)
 * 0x363c. Line length 0x320c/0x320d in pairs of pixels and 0x3f04/0x3f05
 * with it, the frame length 750 lines; mirror and flip 0x3221 (0x06,
 * 0x60).
 */

#include <string.h>
#include <unistd.h>

#include "sns.h"
#include "xmv200.h"

extern const struct xmv200_sns_reg sc2335_mipi[135];

#define N(a) ((int)(sizeof(a) / sizeof((a)[0])))
#define MAX_CHN 3

static struct {
  int valid;
  uint32_t exposure, again, dgain;
  uint32_t logic; /* the 0x363c setting last written, + 1 */
} st[MAX_CHN];
/* the vendor keeps one frame length for all channels */
static uint32_t last_vts;

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
  ae->max_lines = 0x2e9;
  ae->again_min = 0x20; /* the vendor's 0x400 .. 0x3f80 */
  ae->again_max = 0x3f80 >> 5;
  ae->dgain_min = 0x20; /* 0x400 .. 0x7f00 */
  ae->dgain_max = 0x7f00 >> 5;
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

/* the frame rate by line length, at 750 lines a frame */
static void set_fps(const struct xmv200_sns_dev *d,
                    const struct xmv200_sns_attr *a, uint32_t fps,
                    struct xmv200_sns_ae *ae) {
  uint32_t hts, f04 = 0x240;

  if (fps == 25) {
    hts = 0x960;
  } else if (fps == 30) {
    hts = 2000, f04 = 0x1dc;
  } else {
    hts = 60000 / (fps ? fps : 1);
  }
  ae->full_lines = 0x2ee;
  ae->max_lines = 0x2e9;
  w(d, 0x320c, hts >> 9);
  w(d, 0x320d, (hts & 0x1ff) >> 1);
  w(d, 0x320e, 0x02);
  w(d, 0x320f, 0xee);
  w(d, 0x3f04, f04 >> 8);
  w(d, 0x3f05, f04 & 0xdc);
}

static void set_vts(const struct xmv200_sns_dev *d, uint32_t vts,
                    struct xmv200_sns_ae *ae) {
  if (last_vts == vts)
    return;
  last_vts = vts;
  ae->full_lines = vts;
  ae->max_lines = vts - 5;
  if (d->chn < MAX_CHN && st[d->chn].valid & 1 &&
      ae->max_lines < st[d->chn].exposure)
    set_exposure(d, ae->max_lines);
  w(d, 0x320e, vts >> 8);
  w(d, 0x320f, vts & 0xff);
}

/* the analog gain in the vendor's 1/1024 (here 1/32): doublings (ones in
 * bits 8..) from 2x and 1/64 fine steps */
static void again_calc(uint32_t gain, uint32_t *idx, uint32_t *real) {
  uint32_t g = gain << 5, bits = 0, k = 0;

  while (k < 10 && g >= 0x800) {
    k++;
    g >>= 1;
    bits = bits << 1 | 1;
  }
  *idx = g >> 4 | bits << 8;
  *real = (g >> 4 << 4 << k) >> 5;
}

/* the digital gain likewise, in 1/128 fine steps */
static void dgain_calc(uint32_t gain, uint32_t *reg, uint32_t *real) {
  uint32_t g = gain << 5, bits = 0, k = 0;

  while (k < 10 && g >= 0x800) {
    k++;
    g >>= 1;
    bits = bits << 1 | 1;
  }
  *reg = g >> 3 | bits << 8;
  *real = (g >> 3 << 3 << k) >> 5;
}

/* the gains back from their register values, in the vendor's 1/1024 */
static uint32_t again_of(uint32_t v) {
  return (v & 0xff) << 4 << __builtin_popcount(v >> 8 & 0x3ff);
}

static uint32_t dgain_of(uint32_t v) {
  return (v & 0xff) << 3 << __builtin_popcount(v >> 8 & 0x3ff);
}

/*
 * With the total gain (1/16: 0x10 = 1x) the sensor's logic: 0x5799 7 above
 * 6x and 0 to 4x; and 0x363c by a setting, 1 above 2x, 0 below, while
 * 0x3040 reads 0x40 (2 at 2x and below where it reads 0x41, 1 where
 * neither), from {0x0e, 0x07, 0x0f}.
 */
static void set_gain(const struct xmv200_sns_dev *d, uint32_t again,
                     uint32_t dreg) {
  static const uint8_t logic_363c[3] = {0x0e, 0x07, 0x0f};
  uint32_t total =
      (uint32_t)(((uint64_t)again_of(again) * dgain_of(dreg)) >> 10) >> 6;
  uint32_t set;

  if (r(d, 0x3040) == 0x40) {
    set = total > 0x1f;
  } else if (total <= 0x1f && r(d, 0x3040) == 0x41) {
    set = 2;
  } else {
    set = 1;
  }
  if (set == 2 || total <= 0x40)
    w(d, 0x5799, 0);
  else if (total > 0x5f)
    w(d, 0x5799, 7);
  if (d->chn < MAX_CHN && st[d->chn].logic != set + 1) {
    st[d->chn].logic = set + 1;
    w(d, 0x3812, 0x00);
    w(d, 0x363c, logic_363c[set]);
    w(d, 0x3812, 0x30);
  }
  if (d->chn < MAX_CHN) {
    if (st[d->chn].valid & 2 && st[d->chn].again == again &&
        st[d->chn].dgain == dreg)
      return;
    st[d->chn].again = again;
    st[d->chn].dgain = dreg;
    st[d->chn].valid |= 2;
  }
  w(d, 0x3e08, again >> 6 | 3);
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
    {"720p mipi", {0, 0, 0, 0}, sc2335_mipi, N(sc2335_mipi)},
};

const struct xmv200_sns_drv xmv200_sns_sc2335 = {
    .name = "SC2335",
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

/* A tuning's exposure would be 0x3e00..0x3e02 >> 4, half lines: the AE
 * starts from half of it. */
static void run(const struct xmv200_sns_dev *d, int n,
                const struct xmv200_sns_ctx *c) {
  uint32_t idx, reg, real;

  if (n == 0)
    xmv200_sns_init_step(&xmv200_sns_sc2335, d, c);
  if (n == 1)
    xmv200_sns_fps_step(&xmv200_sns_sc2335, d, c);
  if (n != 2)
    return;
  set_exposure(d, c->exp_lines / 2);
  again_calc(0x20, &idx, &real);
  dgain_calc(0x20, &reg, &real);
  set_gain(d, idx, reg);
}
