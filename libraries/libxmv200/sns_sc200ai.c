// SPDX-License-Identifier: MIT
/*
 * SmartSens SC200AI: the XM530V200 vendor libsns's driver of it
 * (snsX20/sc200ai_cmos.c in its isp daemon), linear modes. The mode tables
 * are the vendor's (sns_sc200ai_regs.c, tools/xmv200-sns/extract.py).
 *
 * Exposure is in lines, 0x3e00..0x3e02 holding half lines << 4; the short
 * frame's (HDR) 0x3e22/0x3e04/0x3e05, at 1 line in the linear modes.
 * Gain: 0x3e08/0x3e09 analog (bit 5 of 0x3e08 the high conversion gain,
 * 3.4x), 0x3e06/0x3e07 digital, and the short frame's copies at
 * 0x3e10..0x3e13. Line length 0x320c/0x320d in pairs of pixels, frame
 * length 0x320e/0x320f, mirror and flip 0x3221 (0x06, 0x60).
 *
 * The vendor's 720p and 1080p HDR modes (its +0x1c 2) are not here.
 */

#include <string.h>
#include <unistd.h>

#include "sns.h"
#include "xmv200.h"

extern const struct xmv200_sns_reg sc200ai_mipi[111], sc200ai_720p_mipi[132],
    sc200ai_dvp[123];

#define N(a) ((int)(sizeof(a) / sizeof((a)[0])))
#define MAX_CHN 3

/* what each channel's sensor was last given */
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
  if (m->regs == sc200ai_dvp) {
    /* the parallel output's pads: on channel 2 off */
    if (d->chn == 2) {
      w(d, 0x3000, 0x00);
      w(d, 0x3001, 0x00);
      w(d, 0x3002, 0x00);
      w(d, 0x300a, 0x20);
    } else {
      w(d, 0x3000, 0x0f);
      w(d, 0x3001, 0xff);
      w(d, 0x3002, 0xff);
      w(d, 0x300a, 0x24);
    }
  }
  usleep(20000);
}

static void ae_default(const struct xmv200_sns_attr *a,
                       struct xmv200_sns_ae *ae) {
  ae->full_lines = a->p720 ? 0x2ee : 0x465;
  ae->max_lines = ae->full_lines - 4;
  ae->again_min = 0x20; /* the vendor's 0x400 .. 0xd7e6 */
  ae->again_max = 0xd7e6 >> 5;
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

/* the short frame's exposure, at most 1 line in the linear modes */
static void set_short_exposure(const struct xmv200_sns_dev *d,
                               uint32_t lines) {
  if (lines > 1)
    lines = 1;
  w(d, 0x3e05, (lines & 7) << 5);
  w(d, 0x3e04, (lines << 1 & 0xfff) >> 4);
  w(d, 0x3e22, (lines << 1 & 0xffff) >> 12);
}

/* line length (in pixels; the sensor takes pairs) and frame length per
 * frame rate; 720p 25 and 30 fps also retime the column ADC (0x330b) */
static void set_fps(const struct xmv200_sns_dev *d,
                    const struct xmv200_sns_attr *a, uint32_t fps,
                    struct xmv200_sns_ae *ae) {
  uint32_t hts, vts;

  switch (fps) {
  case 24:
    vts = 0x494, hts = 0xa50;
    break;
  case 12:
    vts = 0x8ca, hts = 0xa50;
    break;
  case 15:
    vts = 0x8ca, hts = 0x898;
    break;
  case 30:
    if (a->p720) {
      w(d, 0x330b, 0x8b);
      vts = 0x2ee, hts = 2000;
    } else {
      vts = 0x465, hts = 0x898;
    }
    break;
  case 31: /* 30 */
    vts = 0x466, hts = 0x898;
    break;
  case 26: /* 25 */
    vts = 0x466, hts = 0xa50;
    break;
  default: /* 25 */
    if (a->p720) {
      w(d, 0x330b, 0x88);
      vts = 0x2ee, hts = 0x960;
    } else {
      vts = 0x465, hts = 0xa50;
    }
  }
  ae->full_lines = vts;
  ae->max_lines = vts - 4;
  w(d, 0x320c, hts >> 9);
  w(d, 0x320d, hts >> 1 & 0xfc);
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
  ae->max_lines = vts - 4;
  if (d->chn < MAX_CHN && st[d->chn].valid & 1 &&
      ae->max_lines < st[d->chn].exposure)
    set_exposure(d, ae->max_lines);
  w(d, 0x320e, vts >> 8);
  w(d, 0x320f, vts & 0xff);
}

/*
 * The analog gain in the vendor's 1/1024 (here 1/32 in and out): above
 * 3.4x the high conversion gain (bit 31 of the value, 0x3e08 bit 5) and
 * the rest coarse doublings (ones in bits 8..) and 1/64 fine steps.
 */
static void again_calc(uint32_t gain, uint32_t *idx, uint32_t *real) {
  uint32_t g = gain << 5, bits = 0, k = 0, hcg = g > 0xd99;

  if (hcg)
    g = (uint32_t)(((uint64_t)g << 10) / 0xd9a);
  g >>= 4;
  if (g < 0x40)
    g = 0x40;
  while (k < 5 && g >= 0x80) {
    k++;
    g >>= 1;
    bits = bits << 1 | 1;
  }
  *idx = bits << 8 | hcg << 31 | g;
  g <<= k + 4;
  if (hcg)
    g = (uint32_t)(((uint64_t)g * 0xd9a) >> 10);
  *real = g >> 5;
}

/* the digital gain: doublings (ones in bits 8..) and 1/128 fine steps */
static void dgain_calc(uint32_t gain, uint32_t *reg, uint32_t *real) {
  uint32_t g = gain << 5 >> 3, bits = 0, k = 0;

  if (g < 0x80)
    g = 0x80;
  while (k < 5 && g >= 0x100) {
    k++;
    g >>= 1;
    bits = bits << 1 | 1;
  }
  *reg = (g & ~1u) + bits * 0x100;
  *real = ((g & ~1u) << (k + 3)) >> 5;
}

static void set_gain(const struct xmv200_sns_dev *d, uint32_t again,
                     uint32_t dreg) {
  uint32_t coarse = ((again & 0xffff) >> 8) << 2 | (again >> 31 ? 0x23 : 3);

  if (d->chn < MAX_CHN) {
    if (st[d->chn].valid & 2 && st[d->chn].again == again &&
        st[d->chn].dgain == dreg)
      return;
    st[d->chn].again = again;
    st[d->chn].dgain = dreg;
    st[d->chn].valid |= 2;
  }
  w(d, 0x3e08, coarse);
  w(d, 0x3e09, again & 0xff);
  w(d, 0x3e06, (dreg & 0xffff) >> 8);
  w(d, 0x3e07, dreg & 0xff);
  w(d, 0x3e12, coarse);
  w(d, 0x3e13, again & 0xff);
  w(d, 0x3e10, (dreg & 0xffff) >> 8);
  w(d, 0x3e11, dreg & 0xff);
}

static void set_flip(const struct xmv200_sns_dev *d, int mirror, int flip) {
  uint32_t v = r(d, 0x3221);

  v = mirror ? v | 0x06 : v & ~0x06u;
  v = flip ? v | 0x60 : v & ~0x60u;
  w(d, 0x3221, v);
}

/* The tuning's exposure is 0x3e00..0x3e02 >> 4, half lines: the vendor's
 * AE starts from half of it, and from a 1-line short frame. */
static void run(const struct xmv200_sns_dev *d, int n,
                const struct xmv200_sns_ctx *c);

#define MODE(nm, dv, p7, t) {nm, {dv, 0, p7, 0}, t, N(t)}

static const struct xmv200_sns_mode modes[] = {
    MODE("1080p mipi", 0, 0, sc200ai_mipi),
    MODE("720p mipi", 0, 1, sc200ai_720p_mipi),
    MODE("1080p dvp", 1, 0, sc200ai_dvp),
};

const struct xmv200_sns_drv xmv200_sns_sc200ai = {
    .name = "SC200AI",
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

static void run(const struct xmv200_sns_dev *d, int n,
                const struct xmv200_sns_ctx *c) {
  const struct xmv200_sns_drv *drv = &xmv200_sns_sc200ai;
  uint32_t idx, reg, real;

  switch (n) {
  case 0:
    xmv200_sns_init_step(drv, d, c);
    break;
  case 1:
    xmv200_sns_fps_step(drv, d, c);
    break;
  case 2:
    set_exposure(d, c->exp_lines / 2);
    set_short_exposure(d, 1);
    again_calc(0x20, &idx, &real);
    dgain_calc(0x20, &reg, &real);
    set_gain(d, idx, reg);
    break;
  }
}
