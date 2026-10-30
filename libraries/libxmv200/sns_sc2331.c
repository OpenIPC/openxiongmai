// SPDX-License-Identifier: MIT
/*
 * SmartSens SC2331: the XM530V200 vendor libsns's driver of it
 * (snsX20/sc2331_cmos.c in its isp daemon). The mode tables are the
 * vendor's (sns_sc2331_regs.c, tools/xmv200-sns/extract.py).
 *
 * Exposure is in lines, 0x3e00..0x3e02 holding half lines << 4; the
 * coarse analog gain 0x3e08, the fine (digital) gain 0x3e06/0x3e07, line
 * length 0x320c/0x320d, frame length 0x320e/0x320f, mirror and flip
 * 0x3221 (0x06, 0x60). Each setter writes only what changed, as the
 * vendor's do.
 */

#include <string.h>
#include <unistd.h>

#include "sns.h"
#include "xmv200.h"

extern const struct xmv200_sns_reg sc2331_dvp[137], sc2331_mipi_1lane[138],
    sc2331_mipi[126];

#define N(a) ((int)(sizeof(a) / sizeof((a)[0])))
#define MAX_CHN 3

/* the analog gain's steps (1x .. 32x) and their 0x3e08 values */
static const uint32_t again_val[6] = {0x20, 0x40, 0x80, 0x100, 0x200, 0x400};
static const uint8_t again_reg[6] = {0x00, 0x08, 0x09, 0x0b, 0x0f, 0x1f};

static struct {
  int valid;
  uint32_t exposure, again, dgain, vts;
  uint32_t again_cap; /* the highest step the temperature allows */
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
  if (d->chn < MAX_CHN) {
    memset(&st[d->chn], 0, sizeof(st[d->chn]));
    st[d->chn].again_cap = 4;
  }
  for (int i = 0; i < m->n; i++)
    w(d, m->regs[i].reg, m->regs[i].val);
  usleep(20000);
}

static void ae_default(const struct xmv200_sns_attr *a,
                       struct xmv200_sns_ae *ae) {
  ae->full_lines = a->p720 ? 0x2ee : 0x465;
  ae->max_lines = ae->full_lines - 7;
  ae->again_min = 0x20;
  ae->again_max = 0x400;
  ae->dgain_min = 0x20;
  ae->dgain_max = 0x80;
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

/* 15 and 12 fps by line and frame length from 30 and 25 (on one lane
 * with 0x301f), the others by line length */
static void set_fps(const struct xmv200_sns_dev *d,
                    const struct xmv200_sns_attr *a, uint32_t fps,
                    struct xmv200_sns_ae *ae) {
  uint32_t hts, vts;

  if (fps == 12) {
    if (a->one_lane)
      w(d, 0x301f, 0x79);
    hts = 0xa50, vts = 0x8ca;
  } else if (fps == 15) {
    if (a->one_lane)
      w(d, 0x301f, 0x78);
    hts = 0x898, vts = 0x8ca;
  } else {
    hts = 66000 / (fps ? fps : 1), vts = 0x465;
  }
  ae->full_lines = vts;
  ae->max_lines = vts - 7;
  clamp_exposure(d, ae);
  w(d, 0x320c, hts >> 8);
  w(d, 0x320d, hts & 0xff);
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
  ae->max_lines = vts - 7;
  clamp_exposure(d, ae);
  w(d, 0x320e, vts >> 8);
  w(d, 0x320f, vts & 0xff);
}

static void again_calc(uint32_t gain, uint32_t *idx, uint32_t *real) {
  uint32_t i = 1;

  if (gain < 0x20)
    gain = 0x20;
  else if (gain > 0x3ff)
    gain = 0x400;
  while (i < 6 && gain >= again_val[i])
    i++;
  *idx = i - 1;
  *real = again_val[i - 1];
}

/* 1x .. 4x: doublings as ones in 0x3e06, the rest in 0x3e07 (0x80 = 1x) */
static void dgain_calc(uint32_t gain, uint32_t *reg, uint32_t *real) {
  uint32_t bits = 0, k = 0;

  if (gain < 0x20)
    gain = 0x20;
  else if (gain > 0x7f)
    gain = 0x80;
  while (k < 6 && gain >= 0x40) {
    k++;
    gain >>= 1;
    bits = bits << 1 | 1;
  }
  *reg = gain << 2 | bits << 8;
  *real = gain << k;
}

/*
 * The analog gain's cap follows the sensor's temperature reading
 * (0x3974), as on the SC2336: down a step (to 4x at least) at 0x18 and
 * above, and up again only below 2x, which it never gets to.
 */
static void set_gain(const struct xmv200_sns_dev *d, uint32_t idx,
                     uint32_t dreg) {
  uint32_t t = r(d, 0x3974), *cap;

  if (d->chn < MAX_CHN) {
    cap = &st[d->chn].again_cap;
    if (t >= 0x18) {
      if (*cap >= 3)
        (*cap)--;
    } else if (t <= 0x10 && *cap <= 1) {
      (*cap)++;
    }
    if (idx > *cap)
      idx = *cap;
    if (st[d->chn].valid & 2 && st[d->chn].again == idx &&
        st[d->chn].dgain == dreg)
      return;
    st[d->chn].again = idx;
    st[d->chn].dgain = dreg;
    st[d->chn].valid |= 2;
  }
  w(d, 0x3e08, again_reg[idx < 6 ? idx : 5]);
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

#define MODE(nm, dv, one, t) {nm, {dv, one, 0, 0}, t, N(t)}

static const struct xmv200_sns_mode modes[] = {
    MODE("1080p mipi 1 lane", 0, 1, sc2331_mipi_1lane),
    MODE("1080p mipi", 0, 0, sc2331_mipi),
    MODE("1080p dvp", 1, 0, sc2331_dvp),
};

const struct xmv200_sns_drv xmv200_sns_sc2331 = {
    .name = "SC2331",
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
    xmv200_sns_init_step(&xmv200_sns_sc2331, d, c);
  if (n == 1)
    xmv200_sns_fps_step(&xmv200_sns_sc2331, d, c);
  if (n != 2)
    return;
  set_exposure(d, c->exp_lines / 2);
  again_calc(0x20, &idx, &real);
  dgain_calc(0x20, &reg, &real);
  set_gain(d, idx, reg);
}

/* ---- XM535 ---- */

/*
 * The XM535D1's libsns (Aug 12 2025 d2db8e4): exposure in whole lines
 * (0x3e02 holds (lines & 0xf) << 4), its own frame rates (10, 12, 15, 20,
 * 30 and 66000 / fps line lengths at 1125 lines), the frame length's low
 * byte masked with 0xef, and flip without turning a channel round. The
 * gain, its temperature cap and steps are the XM530V200's. The 2-lane
 * frame-sync (+0x20 0x19) master/slave writes are not done.
 */
extern const struct xmv200_sns_reg xm535_sc2331_mipi_1lane[140],
    xm535_sc2331_mipi[129], xm535_sc2331_dvp[139];

static const struct xmv200_sns_mode xm535_modes[] = {
    MODE("1080p mipi 1 lane", 0, 1, xm535_sc2331_mipi_1lane),
    MODE("1080p mipi", 0, 0, xm535_sc2331_mipi),
    MODE("1080p dvp", 1, 0, xm535_sc2331_dvp),
};

static void xm535_set_exposure(const struct xmv200_sns_dev *d,
                               uint32_t lines) {
  if (d->chn < MAX_CHN) {
    if (st[d->chn].valid & 1 && st[d->chn].exposure == lines)
      return;
    st[d->chn].exposure = lines;
    st[d->chn].valid |= 1;
  }
  w(d, 0x3e02, (lines & 0xf) << 4);
  w(d, 0x3e01, (lines & 0xfff) >> 4);
  w(d, 0x3e00, (lines & 0xffff) >> 12);
}

static void xm535_set_fps(const struct xmv200_sns_dev *d,
                          const struct xmv200_sns_attr *a, uint32_t fps,
                          struct xmv200_sns_ae *ae) {
  uint32_t hts, vts = 1125;

  switch (fps) {
  case 15:
    if (a->one_lane)
      w(d, 0x301f, 0x78);
    hts = 2200, vts = 2250;
    break;
  case 10:
    if (a->one_lane)
      w(d, 0x301f, 0x78);
    hts = 3300, vts = 2250;
    break;
  case 12:
    if (a->one_lane)
      w(d, 0x301f, 0x79);
    hts = 2640, vts = 2250;
    break;
  case 20:
    if (a->one_lane)
      w(d, 0x301f, 0x79);
    hts = 3300;
    break;
  case 30:
    if (a->one_lane)
      w(d, 0x301f, 0x78);
    hts = 2200;
    break;
  default:
    hts = 66000 / (fps ? fps : 1);
  }
  ae->full_lines = vts;
  ae->max_lines = vts - 7;
  if (d->chn < MAX_CHN && st[d->chn].valid & 1 &&
      ae->max_lines < st[d->chn].exposure)
    xm535_set_exposure(d, ae->max_lines);
  w(d, 0x320c, hts >> 8);
  w(d, 0x320d, hts & 0xff);
  w(d, 0x320e, vts >> 8);
  w(d, 0x320f, vts & 0xef);
  if (d->chn < MAX_CHN)
    st[d->chn].vts = vts;
}

static void xm535_set_vts(const struct xmv200_sns_dev *d, uint32_t vts,
                          struct xmv200_sns_ae *ae) {
  if (d->chn < MAX_CHN && st[d->chn].vts == vts)
    return;
  if (d->chn < MAX_CHN)
    st[d->chn].vts = vts;
  ae->full_lines = vts;
  ae->max_lines = vts - 7;
  if (d->chn < MAX_CHN && st[d->chn].valid & 1 &&
      ae->max_lines < st[d->chn].exposure)
    xm535_set_exposure(d, ae->max_lines);
  w(d, 0x320e, vts >> 8);
  w(d, 0x320f, vts & 0xff);
}

static void xm535_set_flip(const struct xmv200_sns_dev *d, int mirror,
                           int flip) {
  uint32_t v = r(d, 0x3221);

  v = mirror ? v | 0x06 : v & ~0x06u;
  v = flip ? v | 0x60 : v & ~0x60u;
  w(d, 0x3221, v);
}

const struct xmv200_sns_drv xmv200_sns_sc2331_xm535 = {
    .name = "SC2331",
    .family = "xm535",
    .modes = xm535_modes,
    .n_modes = N(xm535_modes),
    .init = init,
    .ae_default = ae_default,
    .set_fps = xm535_set_fps,
    .set_vts = xm535_set_vts,
    .set_exposure = xm535_set_exposure,
    .again_calc = again_calc,
    .dgain_calc = dgain_calc,
    .set_gain = set_gain,
    .set_flip = xm535_set_flip,
};
