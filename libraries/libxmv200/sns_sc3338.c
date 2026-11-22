// SPDX-License-Identifier: MIT
/*
 * SmartSens SC3338: the XM530V200 vendor libsns's driver of it, as
 * functions. One mode, MIPI, 2304x1296; its frame rate function writes the
 * whole mode table again before the line and frame length, with a table
 * of its own for 25 fps. The mode tables are the vendor's
 * (sns_sc3338_regs.c, tools/xmv200-sns/extract.py).
 *
 * Exposure is 0x3e00..0x3e02 (lines << 4), the coarse analog gain 0x3e09,
 * the fine (digital) gain 0x3e06/0x3e07. Before each gain the vendor sets
 * 0x57aa/0x57ad from the analog gain the sensor holds. Each setter writes
 * only what changed since the last call, as the vendor's do (the vendor
 * keeps one frame length and one frame rate for all channels; this keeps
 * one a channel).
 */

#include <string.h>
#include <unistd.h>

#include "sns.h"
#include "xmv200.h"

extern const struct xmv200_sns_reg sc3338_mipi[144], sc3338_mipi_25fps[144];

#define N(a) ((int)(sizeof(a) / sizeof((a)[0])))
#define MAX_CHN 3

/* the analog gain's steps (1x .. 24.3x) and their 0x3e09 values; the
 * vendor's copy of the values stops at the fifth, and the sixth is read
 * from past it (its table's next word, 0x4f, here) */
static const uint32_t again_val[6] = {0x20, 0x31, 0x62, 0xc3, 0x186, 0x30b};
static const uint8_t again_reg[6] = {0x00, 0x40, 0x48, 0x49, 0x4b, 0x4f};

static struct {
  int valid;
  uint32_t exposure, again, dgain, vts, fps;
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

static void init(const struct xmv200_sns_dev *d,
                 const struct xmv200_sns_mode *m,
                 const struct xmv200_sns_attr *a) {
  if (d->chn < MAX_CHN)
    memset(&st[d->chn], 0, sizeof(st[d->chn]));
  table(d, m->regs, m->n);
  usleep(20000);
}

static void ae_default(const struct xmv200_sns_attr *a,
                       struct xmv200_sns_ae *ae) {
  ae->full_lines = 0x546;
  ae->max_lines = 0x53e;
  ae->again_min = 0x20;
  ae->again_max = 0x615;
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
  w(d, 0x3e02, (lines & 0xf) << 4);
  w(d, 0x3e01, (lines & 0xfff) >> 4);
  w(d, 0x3e00, (lines & 0xffff) >> 12);
}

/* 12 and 10 fps at twice the frame length of 25 and 30, 15 by a longer
 * one; the mode's table again first (25 fps: its own, at once) */
static void set_fps(const struct xmv200_sns_dev *d,
                    const struct xmv200_sns_attr *a, uint32_t fps,
                    struct xmv200_sns_ae *ae) {
  uint32_t hts = 0xea6, vts = 0x546;

  switch (fps) {
  case 12:
    hts = 3000, vts = 0xa8c;
    break;
  case 10:
    vts = 0xa8c;
    break;
  case 15:
    vts = 0x708;
    break;
  case 25:
    hts = 3000;
    break;
  }
  if (d->chn < MAX_CHN) {
    if (st[d->chn].valid & 8 && st[d->chn].fps == fps)
      return;
    st[d->chn].fps = fps;
    st[d->chn].valid |= 8;
  }
  ae->full_lines = vts;
  ae->max_lines = vts - 8;
  if (d->chn < MAX_CHN && st[d->chn].valid & 1 &&
      ae->max_lines < st[d->chn].exposure)
    set_exposure(d, ae->max_lines);
  if (fps == 25) {
    table(d, sc3338_mipi_25fps, N(sc3338_mipi_25fps));
  } else {
    table(d, sc3338_mipi, N(sc3338_mipi));
    usleep(20000);
  }
  w(d, 0x320c, hts >> 9);
  w(d, 0x320d, hts >> 1 & 0xdf);
  w(d, 0x320e, vts >> 8);
  w(d, 0x320f, vts & 0xce);
}

static void set_vts(const struct xmv200_sns_dev *d, uint32_t vts,
                    struct xmv200_sns_ae *ae) {
  if (d->chn < MAX_CHN) {
    if (st[d->chn].valid & 4 && st[d->chn].vts == vts)
      return;
    st[d->chn].vts = vts;
    st[d->chn].valid |= 4;
  }
  ae->full_lines = vts;
  ae->max_lines = vts - 8;
  if (d->chn < MAX_CHN && st[d->chn].valid & 1 &&
      ae->max_lines < st[d->chn].exposure)
    set_exposure(d, ae->max_lines);
  w(d, 0x320e, vts >> 8);
  w(d, 0x320f, vts & 0xff);
}

static void again_calc(uint32_t gain, uint32_t *idx, uint32_t *real) {
  uint32_t i = 1;

  if (gain < 0x20)
    gain = 0x20;
  else if (gain > 0x30a)
    gain = 0x30b;
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
    bits = (bits & 0x7f) << 1 | 1;
  }
  *reg = gain << 2 | bits << 16;
  *real = gain << k;
}

static void set_gain(const struct xmv200_sns_dev *d, uint32_t idx,
                     uint32_t dreg) {
  if (r(d, 0x3e09) < 0x4b) {
    w(d, 0x57aa, 0x2a);
    w(d, 0x57ad, 0x0d);
  } else {
    w(d, 0x57aa, 0xcd);
    w(d, 0x57ad, 0x00);
  }
  if (d->chn < MAX_CHN) {
    if (st[d->chn].valid & 2 && st[d->chn].again == idx &&
        st[d->chn].dgain == dreg)
      return;
    st[d->chn].again = idx;
    st[d->chn].dgain = dreg;
    st[d->chn].valid |= 2;
  }
  w(d, 0x3e09, again_reg[idx < 6 ? idx : 5]);
  w(d, 0x3e06, (dreg & 0xffffff) >> 16);
  w(d, 0x3e07, dreg & 0xff);
}

static void set_flip(const struct xmv200_sns_dev *d, int mirror, int flip) {
  uint32_t v = r(d, 0x3221);

  v = mirror ? v | 0x06 : v & ~0x06u;
  v = flip ? v | 0x60 : v & ~0x60u;
  w(d, 0x3221, v);
}

static const struct xmv200_sns_mode modes[] = {
    {"3 MP mipi", {0, 0, 0, 0}, sc3338_mipi, N(sc3338_mipi)},
};

const struct xmv200_sns_drv xmv200_sns_sc3338 = {
    .name = "SC3338",
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
 * The XM535W1 libsns's (Apr 2024): its own table, and a frame rate that
 * only sets line and frame length (the XM530V200's writes the table again).
 */

extern const struct xmv200_sns_reg xm535w1_sc3338_mipi[144];

static void w1_set_fps(const struct xmv200_sns_dev *d,
                       const struct xmv200_sns_attr *a, uint32_t fps,
                       struct xmv200_sns_ae *ae) {
  uint32_t hts = 0xea6, vts = 0x546;

  switch (fps) {
  case 12:
    hts = 3000, vts = 0xa8c;
    break;
  case 10:
    vts = 0xa8c;
    break;
  case 15:
    vts = 0x708;
    break;
  case 25:
    hts = 3000;
    break;
  }
  if (d->chn < MAX_CHN) {
    if (st[d->chn].valid & 8 && st[d->chn].fps == fps)
      return;
    st[d->chn].fps = fps;
    st[d->chn].valid |= 8;
  }
  ae->full_lines = vts;
  ae->max_lines = vts - 8;
  if (d->chn < MAX_CHN && st[d->chn].valid & 1 &&
      ae->max_lines < st[d->chn].exposure)
    set_exposure(d, ae->max_lines);
  w(d, 0x320c, hts >> 9);
  w(d, 0x320d, hts >> 1 & 0xdf);
  w(d, 0x320e, vts >> 8);
  w(d, 0x320f, vts & 0xce);
}

static const struct xmv200_sns_mode w1_modes[] = {
    {"3 MP mipi", {0, 0, 0, 0}, xm535w1_sc3338_mipi, N(xm535w1_sc3338_mipi)},
};

const struct xmv200_sns_drv xmv200_sns_sc3338_xm535w1 = {
    .name = "SC3338",
    .family = "xm535w1",
    .modes = w1_modes,
    .n_modes = N(w1_modes),
    .init = init,
    .ae_default = ae_default,
    .set_fps = w1_set_fps,
    .set_vts = set_vts,
    .set_exposure = set_exposure,
    .again_calc = again_calc,
    .dgain_calc = dgain_calc,
    .set_gain = set_gain,
    .set_flip = set_flip,
};

/* ---- XM535 ---- */

/*
 * The XM535D1's libsns (Aug 12 2025 d2db8e4), unchecked (no tuning): its
 * table ("SC3338 SC3336P 3M"), and a frame rate that sets the XM530V200's
 * line and frame lengths without writing the mode's table again.
 * Exposure, gain and its steps, frame length and flip are the XM530V200's.
 */
extern const struct xmv200_sns_reg xm535_sc3338_mipi[152];

static const struct xmv200_sns_mode xm535_modes[] = {
    {"3 MP mipi", {0, 0, 0, 0}, xm535_sc3338_mipi, N(xm535_sc3338_mipi)},
};

static void xm535_set_fps(const struct xmv200_sns_dev *d,
                          const struct xmv200_sns_attr *a, uint32_t fps,
                          struct xmv200_sns_ae *ae) {
  uint32_t hts = 0xea6, vts = 0x546;

  switch (fps) {
  case 12:
    hts = 3000, vts = 0xa8c;
    break;
  case 10:
    vts = 0xa8c;
    break;
  case 15:
    vts = 0x708;
    break;
  case 25:
    hts = 3000;
    break;
  }
  if (d->chn < MAX_CHN) {
    if (st[d->chn].valid & 8 && st[d->chn].fps == fps)
      return;
    st[d->chn].fps = fps;
    st[d->chn].valid |= 8;
  }
  ae->full_lines = vts;
  ae->max_lines = vts - 8;
  if (d->chn < MAX_CHN && st[d->chn].valid & 1 &&
      ae->max_lines < st[d->chn].exposure)
    set_exposure(d, ae->max_lines);
  w(d, 0x320c, hts >> 9);
  w(d, 0x320d, hts >> 1 & 0xdf);
  w(d, 0x320e, vts >> 8);
  w(d, 0x320f, vts & 0xce);
}

const struct xmv200_sns_drv xmv200_sns_sc3338_xm535 = {
    .name = "SC3338",
    .family = "xm535",
    .modes = xm535_modes,
    .n_modes = N(xm535_modes),
    .init = init,
    .ae_default = ae_default,
    .set_fps = xm535_set_fps,
    .set_vts = set_vts,
    .set_exposure = set_exposure,
    .again_calc = again_calc,
    .dgain_calc = dgain_calc,
    .set_gain = set_gain,
    .set_flip = set_flip,
};

/* ---- XM533 ---- */

/*
 * The XM533 libsns's: the same table, a seventh analog gain step (48.6x,
 * 0x5f) and its frame rate by line length alone at 1350 lines (12 fps
 * 0x186a, 15 fps 5000, else 0xea6). Before that it takes the exposure the
 * sensor holds down to the frame's. The tuning's second run is the frame
 * rate and the AE's start together.
 */
static const uint32_t xm533_again_val[7] = {0x20,  0x31,  0x62, 0xc3,
                                            0x186, 0x30b, 0x615};
static const uint8_t xm533_again_reg[7] = {0x00, 0x40, 0x48, 0x49,
                                           0x4b, 0x4f, 0x5f};

static void xm533_set_fps(const struct xmv200_sns_dev *d,
                          const struct xmv200_sns_attr *a, uint32_t fps,
                          struct xmv200_sns_ae *ae) {
  uint32_t held = (r(d, 0x3e02) >> 4 | (r(d, 0x3e00) << 8 | r(d, 0x3e01)) << 4);
  uint32_t hts = fps == 12 ? 0x186a : fps == 15 ? 5000 : 0xea6;

  if (d->chn < MAX_CHN) {
    if (st[d->chn].valid & 8 && st[d->chn].fps == fps)
      return;
    st[d->chn].fps = fps;
    st[d->chn].valid |= 8;
  }
  ae->full_lines = 0x546;
  ae->max_lines = 0x53e;
  if (held > 0x53e)
    set_exposure(d, 0x53e);
  w(d, 0x320c, hts >> 9);
  w(d, 0x320d, hts >> 1 & 0xf7);
  w(d, 0x320e, 0x05);
  w(d, 0x320f, 0x46);
}

static void xm533_again_calc(uint32_t gain, uint32_t *idx, uint32_t *real) {
  uint32_t i = 1;

  if (gain < 0x20)
    gain = 0x20;
  else if (gain > 0x614)
    gain = 0x615;
  while (i < 7 && gain >= xm533_again_val[i])
    i++;
  *idx = i - 1;
  *real = xm533_again_val[i - 1];
}

static void xm533_set_gain(const struct xmv200_sns_dev *d, uint32_t idx,
                           uint32_t dreg) {
  if (r(d, 0x3e09) < 0x4b) {
    w(d, 0x57aa, 0x2a);
    w(d, 0x57ad, 0x0d);
  } else {
    w(d, 0x57aa, 0xcd);
    w(d, 0x57ad, 0x00);
  }
  if (d->chn < MAX_CHN) {
    if (st[d->chn].valid & 2 && st[d->chn].again == idx &&
        st[d->chn].dgain == dreg)
      return;
    st[d->chn].again = idx;
    st[d->chn].dgain = dreg;
    st[d->chn].valid |= 2;
  }
  w(d, 0x3e09, xm533_again_reg[idx < 7 ? idx : 6]);
  w(d, 0x3e06, (dreg & 0xffffff) >> 16);
  w(d, 0x3e07, dreg & 0xff);
}

static void xm533_run(const struct xmv200_sns_dev *d, int n,
                      const struct xmv200_sns_ctx *c);

const struct xmv200_sns_drv xmv200_sns_sc3338_xm533 = {
    .name = "SC3338",
    .family = "xm533",
    .modes = modes,
    .n_modes = N(modes),
    .init = init,
    .ae_default = ae_default,
    .set_fps = xm533_set_fps,
    .set_vts = set_vts,
    .set_exposure = set_exposure,
    .again_calc = xm533_again_calc,
    .dgain_calc = dgain_calc,
    .set_gain = xm533_set_gain,
    .set_flip = set_flip,
    .n_runs = 2,
    .run = xm533_run,
};

static void xm533_run(const struct xmv200_sns_dev *d, int n,
                      const struct xmv200_sns_ctx *c) {
  const struct xmv200_sns_drv *drv = &xmv200_sns_sc3338_xm533;

  if (n == 0) {
    xmv200_sns_init_step(drv, d, c);
  } else if (n == 1) {
    xmv200_sns_fps_step(drv, d, c);
    xmv200_sns_ae_step(drv, d, c);
  }
}
