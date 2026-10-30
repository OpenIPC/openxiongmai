// SPDX-License-Identifier: MIT
/*
 * SmartSens SC223A: the XM530V200 vendor libsns's driver of it
 * (snsX20/sc223a_cmos.c in its isp daemon), as functions, in its linear
 * modes. It reads chip ID 0xcb3e, as the SC223AP does; the vendor's probe
 * takes it for the SC223A where 0x801b reads 0. The mode tables are the
 * vendor's (sns_sc223a_regs.c, tools/xmv200-sns/extract.py).
 *
 * The vendor's also has a 1080p HDR mode (attr +0x1c = 2: its table at
 * 0x7ca48, a short frame's exposure, 0x3e23/0x3e24); that is not here.
 *
 * Exposure is 0x3e00..0x3e02 in half lines (lines << 5), the coarse analog
 * gain 0x3e09 (0x3e13), the fine gain 0x3e06/0x3e07 (0x3e10/0x3e11), line
 * length 0x320c/0x320d, frame length 0x320e/0x320f, mirror and flip
 * 0x3221 (0x06, 0xe0). Each setter writes only what changed since the last
 * call, as the vendor's do.
 */

#include <string.h>
#include <unistd.h>

#include "sns.h"
#include "xmv200.h"

extern const struct xmv200_sns_reg sc223a_dvp[180], sc223a_720p_mipi[187],
    sc223a_mipi[174], sc223a_mipi_max[176];

#define N(a) ((int)(sizeof(a) / sizeof((a)[0])))
#define MAX_CHN 3

/* the analog gain's steps and their 0x3e09 values */
static const uint32_t again_val[7] = {0x20,  0x39,  0x73, 0xe7,
                                      0x1cf, 0x39e, 0x73d};
static const uint8_t again_reg[7] = {0x00, 0x40, 0x48, 0x49, 0x4b, 0x4f, 0x5f};

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

static void table(const struct xmv200_sns_dev *d,
                  const struct xmv200_sns_reg *t, int n) {
  for (int i = 0; i < n; i++)
    w(d, t[i].reg, t[i].val);
}

/* ---- init ---- */

static void init(const struct xmv200_sns_dev *d,
                 const struct xmv200_sns_mode *m,
                 const struct xmv200_sns_attr *a) {
  /* the vendor's has the exposure and gain written again after it */
  if (d->chn < MAX_CHN)
    memset(&st[d->chn], 0, sizeof(st[d->chn]));
  if (m->regs == sc223a_dvp) {
    w(d, 0x0103, 0x01);
    usleep(20000);
    table(d, m->regs, m->n);
    if (d->chn == 2) { /* the third sensor slaves its sync */
      w(d, 0x3000, 0x00);
      w(d, 0x3001, 0x00);
      w(d, 0x3002, 0x00);
      w(d, 0x300a, 0x20);
    } else {
      w(d, 0x3000, 0x0f);
      w(d, 0x3001, 0xff);
      w(d, 0x3002, 0xf0);
      w(d, 0x300a, 0x24);
    }
    w(d, 0x3032, 0xa0);
    w(d, 0x3225, 0x30);
    w(d, 0x3217, 0x04);
    w(d, 0x322e, 0x04);
    w(d, 0x322f, 0x4a);
    w(d, 0x0100, 0x01);
    usleep(20000);
    return;
  }
  /* MIPI; "mipi max" (the vendor's attr +0x23 bit 0) is the table and its
   * last two writes, 0x3650/0x3651, after the pause */
  if (m->regs == sc223a_mipi_max) {
    table(d, m->regs, m->n - 2);
    usleep(20000);
    table(d, m->regs + m->n - 2, 2);
  } else {
    table(d, m->regs, m->n);
    usleep(20000);
  }
}

/* ---- AE limits ---- */

/* the vendor's keeps 1080p's exposure limit in 720p */
static void ae_default(const struct xmv200_sns_attr *a,
                       struct xmv200_sns_ae *ae) {
  ae->full_lines = a->p720 ? 800 : 1125;
  ae->max_lines = 1120;
  ae->again_min = 0x20;
  ae->again_max = 0x73e;
  ae->dgain_min = 0x20;
  ae->dgain_max = 0x40;
}

/* ---- exposure ---- */

static void set_exposure(const struct xmv200_sns_dev *d, uint32_t lines) {
  uint32_t v = lines * 2;

  if (d->chn < MAX_CHN) {
    if (st[d->chn].valid & 1 && st[d->chn].exposure == lines)
      return;
    st[d->chn].exposure = lines;
    st[d->chn].valid |= 1;
  }
  w(d, 0x3e02, (v << 4) & 0xe0);
  w(d, 0x3e01, (v & 0xfff) >> 4);
  w(d, 0x3e00, (v & 0xffff) >> 12);
}

static void clamp_exposure(const struct xmv200_sns_dev *d,
                           const struct xmv200_sns_ae *ae) {
  if (d->chn < MAX_CHN && st[d->chn].valid & 1 &&
      ae->max_lines < st[d->chn].exposure)
    set_exposure(d, ae->max_lines);
}

/* ---- frame rate ---- */

/* 15 and 12 fps from 30 and 25 by the frame length; 26 and 31 the
 * vendor's alternative 25 and 30 fps timings. The vendor's leaves the
 * exposure as it is. */
static void set_fps(const struct xmv200_sns_dev *d,
                    const struct xmv200_sns_attr *a, uint32_t fps,
                    struct xmv200_sns_ae *ae) {
  uint32_t hts, vts = 0x465;

  switch (fps) {
  case 24:
    hts = 0xabe;
    break;
  case 12:
    hts = 0xa50, vts = 0x8ca;
    break;
  case 15:
    hts = 0x898, vts = 0x8ca;
    break;
  case 26:
    w(d, 0x301f, 0x68);
    w(d, 0x3690, 0x62);
    w(d, 0x3691, 0x63);
    w(d, 0x3724, 0x41);
    w(d, 0x3725, 0xc4);
    w(d, 0x37b2, 0x09);
    w(d, 0x450d, 0x11);
    hts = 0x948, vts = 0x4e2;
    break;
  case 31:
    hts = 0x898, vts = 0x464;
    break;
  default:
    if (!fps)
      fps = 1;
    if (a->p720)
      hts = 60000 / fps, vts = 800;
    else
      hts = 66000 / fps;
  }
  ae->full_lines = vts;
  ae->max_lines = vts - 5;
  w(d, 0x320c, hts >> 8);
  w(d, 0x320d, hts & 0xff);
  w(d, 0x320e, vts >> 8);
  w(d, 0x320f, vts & 0xef);
  if (d->chn < MAX_CHN)
    st[d->chn].vts = vts;
}

static void set_vts(const struct xmv200_sns_dev *d, uint32_t vts,
                    struct xmv200_sns_ae *ae) {
  if (d->chn < MAX_CHN) {
    if (st[d->chn].vts == vts)
      return;
    st[d->chn].vts = vts;
  }
  ae->full_lines = vts;
  ae->max_lines = vts - 5;
  clamp_exposure(d, ae);
  w(d, 0x320e, vts >> 8);
  w(d, 0x320f, vts & 0xff);
}

/* ---- gain ---- */

static void again_calc(uint32_t gain, uint32_t *idx, uint32_t *real) {
  uint32_t i = 1;

  if (gain < 0x20)
    gain = 0x20;
  else if (gain > 0x73d)
    gain = 0x73e;
  while (i < 7 && gain >= again_val[i])
    i++;
  *idx = i - 1;
  *real = again_val[i - 1];
}

/* 1x .. 4x: doublings as ones from bit 16 (0x3e06), the rest << 2 (0x80 =
 * 1x) */
static void dgain_calc(uint32_t gain, uint32_t *reg, uint32_t *real) {
  uint32_t bits = 0, k = 0;

  if (gain < 0x20)
    gain = 0x20;
  else if (gain > 0x7f)
    gain = 0x80;
  while (k < 6 && gain >= 0x40) {
    k++;
    gain >>= 1;
    bits = (bits & 0x7f) << 1 | 1;
  }
  *reg = gain << 2 | bits << 16;
  *real = gain << k;
}

static void set_gain(const struct xmv200_sns_dev *d, uint32_t idx,
                     uint32_t dreg) {
  uint32_t a = again_reg[idx < 7 ? idx : 6], hi = (dreg & 0xffffff) >> 16;

  if (d->chn < MAX_CHN) {
    if (st[d->chn].valid & 2 && st[d->chn].again == idx &&
        st[d->chn].dgain == dreg)
      return;
    st[d->chn].again = idx;
    st[d->chn].dgain = dreg;
    st[d->chn].valid |= 2;
  }
  w(d, 0x3e09, a);
  w(d, 0x3e06, hi);
  w(d, 0x3e07, dreg & 0xff);
  w(d, 0x3e13, a);
  w(d, 0x3e10, hi);
  w(d, 0x3e11, dreg & 0xff);
}

/* ---- mirror and flip ---- */

/* channel 1's sensor is mounted the other way round */
static void set_flip(const struct xmv200_sns_dev *d, int mirror, int flip) {
  uint32_t v;

  r(d, 0x3221); /* the vendor's reads it twice */
  if (d->chn) {
    mirror = !mirror;
    flip = !flip;
  }
  v = r(d, 0x3221);
  v = mirror ? v | 0x06 : v & ~0x06u;
  v = flip ? v | 0xe0 : v & ~0xe0u;
  w(d, 0x3221, v);
}

static const struct xmv200_sns_mode modes[] = {
    {"1080p mipi", {0, 0, 0, 0}, sc223a_mipi, N(sc223a_mipi)},
    {"1080p mipi max", {0, 0, 0, 0}, sc223a_mipi_max, N(sc223a_mipi_max)},
    {"720p mipi", {0, 0, 1, 0}, sc223a_720p_mipi, N(sc223a_720p_mipi)},
    {"1080p dvp", {1, 0, 0, 0}, sc223a_dvp, N(sc223a_dvp)},
};

const struct xmv200_sns_drv xmv200_sns_sc223a = {
    .name = "SC223A",
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
};

/* ---- XM535W1 ----
 *
 * The XM535W1 libsns's (Apr 2024), unverified (no tuning): its tables
 * (DVP, 720p and 1080p MIPI), no short frame, flip without turning channel
 * 1 round, and its frame rates. Exposure, frame length and the gain steps
 * are the XM530V200's.
 */

extern const struct xmv200_sns_reg xm535w1_sc223a_dvp[195],
    xm535w1_sc223a_720p_mipi[187], xm535w1_sc223a_mipi[173];

static void w1_init(const struct xmv200_sns_dev *d,
                    const struct xmv200_sns_mode *m,
                    const struct xmv200_sns_attr *a) {
  if (d->chn < MAX_CHN)
    memset(&st[d->chn], 0, sizeof(st[d->chn]));
  table(d, m->regs, m->n);
  if (m->regs != xm535w1_sc223a_dvp)
    usleep(20000);
}

static void w1_ae_default(const struct xmv200_sns_attr *a,
                          struct xmv200_sns_ae *ae) {
  ae->full_lines = a->p720 ? 750 : 1125;
  ae->max_lines = ae->full_lines - 5;
  ae->again_min = 0x20;
  ae->again_max = 0x73e;
  ae->dgain_min = 0x20;
  ae->dgain_max = 0x40;
}

/* 6 fps by line and frame length from 25; 26 and 31 the vendor's
 * alternative 25 and 30 fps timings */
static void w1_set_fps(const struct xmv200_sns_dev *d,
                       const struct xmv200_sns_attr *a, uint32_t fps,
                       struct xmv200_sns_ae *ae) {
  uint32_t hts, vts;

  switch (fps) {
  case 26:
    w(d, 0x301f, 0x68);
    w(d, 0x3690, 0x62);
    w(d, 0x3691, 0x63);
    w(d, 0x3724, 0x41);
    w(d, 0x3725, 0xc4);
    w(d, 0x37b2, 0x09);
    w(d, 0x450d, 0x11);
    hts = 0x948, vts = 0x4e2;
    break;
  case 31:
    hts = 0x898, vts = 0x464;
    break;
  case 6:
    hts = 0x14a0, vts = 0x8ca;
    break;
  default:
    if (!fps)
      fps = 1;
    if (a->p720)
      hts = 60000 / fps, vts = 750;
    else
      hts = 66000 / fps, vts = 0x465;
  }
  ae->full_lines = vts;
  ae->max_lines = vts - 5;
  w(d, 0x320c, hts >> 8 & 0xff);
  w(d, 0x320d, hts & 0xff);
  w(d, 0x320e, vts >> 8);
  w(d, 0x320f, vts & 0xef);
  if (d->chn < MAX_CHN)
    st[d->chn].vts = vts;
}

static void w1_set_gain(const struct xmv200_sns_dev *d, uint32_t idx,
                        uint32_t dreg) {
  if (d->chn < MAX_CHN) {
    if (st[d->chn].valid & 2 && st[d->chn].again == idx &&
        st[d->chn].dgain == dreg)
      return;
    st[d->chn].again = idx;
    st[d->chn].dgain = dreg;
    st[d->chn].valid |= 2;
  }
  w(d, 0x3e09, again_reg[idx < 7 ? idx : 6]);
  w(d, 0x3e06, (dreg & 0xffffff) >> 16);
  w(d, 0x3e07, dreg & 0xff);
}

static void w1_set_flip(const struct xmv200_sns_dev *d, int mirror, int flip) {
  uint32_t v = r(d, 0x3221);

  v = mirror ? v | 0x06 : v & ~0x06u;
  v = flip ? v | 0xe0 : v & ~0xe0u;
  w(d, 0x3221, v);
}

static const struct xmv200_sns_mode w1_modes[] = {
    {"1080p mipi", {0, 0, 0, 0}, xm535w1_sc223a_mipi, N(xm535w1_sc223a_mipi)},
    {"720p mipi", {0, 0, 1, 0}, xm535w1_sc223a_720p_mipi,
     N(xm535w1_sc223a_720p_mipi)},
    {"1080p dvp", {1, 0, 0, 0}, xm535w1_sc223a_dvp, N(xm535w1_sc223a_dvp)},
};

const struct xmv200_sns_drv xmv200_sns_sc223a_xm535w1 = {
    .name = "SC223A",
    .family = "xm535w1",
    .exp_shift = 1,
    .modes = w1_modes,
    .n_modes = N(w1_modes),
    .init = w1_init,
    .ae_default = w1_ae_default,
    .set_fps = w1_set_fps,
    .set_vts = set_vts,
    .set_exposure = set_exposure,
    .again_calc = again_calc,
    .dgain_calc = dgain_calc,
    .set_gain = w1_set_gain,
    .set_flip = w1_set_flip,
};

/* ---- XM533 ---- */

/*
 * The XM533 libsns's: its own 1080p and 720p ("1M") MIPI tables
 * (sns_sc223a_xm533_regs.c), a 5-line exposure margin, a frame rate with
 * the 26 fps analog settings, a gain without the short frame's copies and
 * a flip that does not turn channel 1's sensor round. Exposure and the gain
 * steps are the XM530V200's.
 */
extern const struct xmv200_sns_reg xm533_sc223a_mipi[173],
    xm533_sc223a_mipi_1m[187];

static void xm533_init(const struct xmv200_sns_dev *d,
                       const struct xmv200_sns_mode *m,
                       const struct xmv200_sns_attr *a) {
  if (d->chn < MAX_CHN)
    memset(&st[d->chn], 0, sizeof(st[d->chn]));
  table(d, m->regs, m->n);
  usleep(20000);
}

static void xm533_ae_default(const struct xmv200_sns_attr *a,
                             struct xmv200_sns_ae *ae) {
  ae->full_lines = a->p720 ? 0x2ee : 0x465;
  ae->max_lines = ae->full_lines - 5;
  ae->again_min = 0x20;
  ae->again_max = 0x73e;
  ae->dgain_min = 0x20;
  ae->dgain_max = 0x40;
}

static void xm533_set_fps(const struct xmv200_sns_dev *d,
                          const struct xmv200_sns_attr *a, uint32_t fps,
                          struct xmv200_sns_ae *ae) {
  uint32_t hts, vts;

  switch (fps) {
  case 15:
    hts = 0x898, vts = 0x8ca;
    break;
  case 12:
    hts = 0xa50, vts = 0x8ca;
    break;
  case 26:
    w(d, 0x301f, 0x68);
    w(d, 0x3690, 0x62);
    w(d, 0x3691, 0x63);
    w(d, 0x3724, 0x41);
    w(d, 0x3725, 0xc4);
    w(d, 0x37b2, 0x09);
    w(d, 0x450d, 0x11);
    hts = 0x948, vts = 0x4e2;
    break;
  case 31:
    hts = 0x898, vts = 0x464;
    break;
  default:
    if (!fps)
      fps = 1;
    if (a->p720)
      hts = 60000 / fps, vts = 0x2ee;
    else
      hts = 66000 / fps, vts = 0x465;
  }
  ae->full_lines = vts;
  ae->max_lines = vts - 5;
  w(d, 0x320c, hts >> 8);
  w(d, 0x320d, hts & 0xff);
  w(d, 0x320e, vts >> 8);
  w(d, 0x320f, vts & 0xef);
}

static void xm533_set_vts(const struct xmv200_sns_dev *d, uint32_t vts,
                          struct xmv200_sns_ae *ae) {
  if (d->chn < MAX_CHN) {
    if (st[d->chn].valid & 4 && st[d->chn].vts == vts)
      return;
    st[d->chn].vts = vts;
    st[d->chn].valid |= 4;
  }
  ae->full_lines = vts;
  ae->max_lines = vts - 5;
  clamp_exposure(d, ae);
  w(d, 0x320e, vts >> 8);
  w(d, 0x320f, vts & 0xff);
}

static void xm533_set_gain(const struct xmv200_sns_dev *d, uint32_t idx,
                           uint32_t dreg) {
  if (d->chn < MAX_CHN) {
    if (st[d->chn].valid & 2 && st[d->chn].again == idx &&
        st[d->chn].dgain == dreg)
      return;
    st[d->chn].again = idx;
    st[d->chn].dgain = dreg;
    st[d->chn].valid |= 2;
  }
  w(d, 0x3e09, again_reg[idx < 7 ? idx : 6]);
  w(d, 0x3e06, (dreg & 0xffffff) >> 16);
  w(d, 0x3e07, dreg & 0xff);
}

static void xm533_set_flip(const struct xmv200_sns_dev *d, int mirror,
                           int flip) {
  uint32_t v = r(d, 0x3221);

  v = mirror ? v | 0x06 : v & ~0x06u;
  v = flip ? v | 0xe0 : v & ~0xe0u;
  w(d, 0x3221, v);
}

static const struct xmv200_sns_mode xm533_modes[] = {
    {"1080p mipi", {0, 0, 0, 0}, xm533_sc223a_mipi, N(xm533_sc223a_mipi)},
    {"720p mipi", {0, 0, 1, 0}, xm533_sc223a_mipi_1m,
     N(xm533_sc223a_mipi_1m)},
};

static void xm533_run(const struct xmv200_sns_dev *d, int n,
                      const struct xmv200_sns_ctx *c);

const struct xmv200_sns_drv xmv200_sns_sc223a_xm533 = {
    .name = "SC223A",
    .family = "xm533",
    .exp_shift = 1,
    .modes = xm533_modes,
    .n_modes = N(xm533_modes),
    .init = xm533_init,
    .ae_default = xm533_ae_default,
    .set_fps = xm533_set_fps,
    .set_vts = xm533_set_vts,
    .set_exposure = set_exposure,
    .again_calc = again_calc,
    .dgain_calc = dgain_calc,
    .set_gain = xm533_set_gain,
    .set_flip = xm533_set_flip,
    .run = xm533_run,
};

/* the tuning's exposure is the register's count, in half lines */
static void xm533_run(const struct xmv200_sns_dev *d, int n,
                      const struct xmv200_sns_ctx *c) {
  struct xmv200_sns_ctx x = *c;

  if (n == 0) {
    xmv200_sns_init_step(&xmv200_sns_sc223a_xm533, d, c);
  } else if (n == 1) {
    xmv200_sns_fps_step(&xmv200_sns_sc223a_xm533, d, c);
  } else if (n == 2) {
    x.exp_lines = c->exp_lines / 2;
    xmv200_sns_ae_step(&xmv200_sns_sc223a_xm533, d, &x);
  }
}

/* ---- XM535 ----
 *
 * The XM535D1 libsns's (Aug 2025), unverified (no tuning). Its probe: I2C
 * address 0x30 (0x32 on channel 1), 16-bit registers, 8-bit values, chip
 * ID 0x3107 << 8 | 0x3108 = 0xcb3e with 0x801b reading 0 (else SC223AP).
 * Its functions are the XM535W1's (init, AE limits, frame rate, gain
 * without the short frame, flip without turning channel 1 round); the
 * tables are its own (DVP, 720p "1M" and 1080p MIPI).
 */

extern const struct xmv200_sns_reg xm535_sc223a_dvp[195],
    xm535_sc223a_720p_mipi[187], xm535_sc223a_mipi[173];

static void xm535_init(const struct xmv200_sns_dev *d,
                       const struct xmv200_sns_mode *m,
                       const struct xmv200_sns_attr *a) {
  if (d->chn < MAX_CHN)
    memset(&st[d->chn], 0, sizeof(st[d->chn]));
  table(d, m->regs, m->n);
  if (m->regs != xm535_sc223a_dvp)
    usleep(20000);
}

static const struct xmv200_sns_mode xm535_modes[] = {
    {"1080p mipi", {0, 0, 0, 0}, xm535_sc223a_mipi, N(xm535_sc223a_mipi)},
    {"720p mipi", {0, 0, 1, 0}, xm535_sc223a_720p_mipi,
     N(xm535_sc223a_720p_mipi)},
    {"1080p dvp", {1, 0, 0, 0}, xm535_sc223a_dvp, N(xm535_sc223a_dvp)},
};

const struct xmv200_sns_drv xmv200_sns_sc223a_xm535 = {
    .name = "SC223A",
    .family = "xm535",
    .exp_shift = 1,
    .modes = xm535_modes,
    .n_modes = N(xm535_modes),
    .init = xm535_init,
    .ae_default = w1_ae_default,
    .set_fps = w1_set_fps,
    .set_vts = set_vts,
    .set_exposure = set_exposure,
    .again_calc = again_calc,
    .dgain_calc = dgain_calc,
    .set_gain = w1_set_gain,
    .set_flip = w1_set_flip,
};
