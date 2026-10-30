// SPDX-License-Identifier: MIT
/*
 * Imagedesign MIS40C1: the XM530V200 vendor libsns's driver of it, as
 * functions. Unverified: no capture has this sensor.
 *
 * Probe (the vendor's, shared by the MIS parts): I2C address 0x30 (0x31
 * for a second sensor on a board whose first is at 0x30), 16-bit
 * registers, 8-bit values; 0x3006 = 1, 100 ms, then 0x3000 << 8 | 0x3001,
 * and where that is neither 0x2008 nor 0x1311 0x3004 << 8 | 0x3005 = 0x0004.
 *
 * One mode; register 0xffff in its table is a 50 ms wait (the vendor
 * writes it to the sensor too). Every change ends in 0x300c = 1 (update).
 * Exposure 0x3100/0x3101, frame length 0x3105/0x3106 (in a 0x3006 = 2
 * hold), analog gain 0x3103/0x3104 as 0x400 - 0x100000 / gain (the
 * vendor's gain in 1/1024; sns.h's in 1/32), the digital gain left at 1x
 * (0x3709..0x370c = 0x80). Mirror 0x3007 bit 0 clear, flip bit 1 clear.
 */

#include <string.h>
#include <unistd.h>

#include "sns.h"
#include "xmv200.h"

extern const struct xmv200_sns_reg mis40c1[267];

#define N(a) ((int)(sizeof(a) / sizeof((a)[0])))
#define MAX_CHN 3

/* the frame length for 12..25 fps (those it has none for get 30 fps) */
static const uint16_t fps_vts[14] = {3600, 1500, 1500, 3000, 1500, 1500, 1500,
                                     1500, 2250, 1500, 1500, 1500, 1500, 1800};

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
  for (int i = 0; i < m->n; i++) {
    w(d, m->regs[i].reg, m->regs[i].val);
    if (m->regs[i].reg == 0xffff)
      usleep(50000);
  }
  w(d, 0x300c, 1);
  usleep(20000);
}

static void ae_default(const struct xmv200_sns_attr *a,
                       struct xmv200_sns_ae *ae) {
  ae->full_lines = 0x5dc;
  ae->max_lines = 0x5d9;
  ae->again_min = 0x400 / 32;
  ae->again_max = 0x10000 / 32;
  ae->dgain_min = 0x20;
  ae->dgain_max = 0x20;
}

static void set_exposure(const struct xmv200_sns_dev *d, uint32_t lines) {
  if (d->chn < MAX_CHN) {
    if (st[d->chn].valid & 1 && st[d->chn].exposure == lines)
      return;
    st[d->chn].exposure = lines;
    st[d->chn].valid |= 1;
  }
  w(d, 0x3101, lines & 0xff);
  w(d, 0x3100, (lines & 0xffff) >> 8);
  w(d, 0x300c, 1);
}

static void clamp_exposure(const struct xmv200_sns_dev *d,
                           const struct xmv200_sns_ae *ae) {
  if (d->chn < MAX_CHN && st[d->chn].valid & 1 &&
      ae->max_lines < st[d->chn].exposure)
    set_exposure(d, ae->max_lines);
}

static void frame_length(const struct xmv200_sns_dev *d, uint32_t vts) {
  w(d, 0x3006, 2);
  usleep(100000);
  w(d, 0x3105, vts >> 8);
  w(d, 0x3106, vts & 0xff);
  w(d, 0x300c, 1);
  w(d, 0x3006, 0);
}

static void set_fps(const struct xmv200_sns_dev *d,
                    const struct xmv200_sns_attr *a, uint32_t fps,
                    struct xmv200_sns_ae *ae) {
  uint32_t i = (fps - 12) & 0xff, vts = i < 14 ? fps_vts[i] : 0x5dc;

  ae->full_lines = vts;
  ae->max_lines = vts - 3;
  if (d->chn < MAX_CHN)
    st[d->chn].vts = vts;
  frame_length(d, vts);
}

static void set_vts(const struct xmv200_sns_dev *d, uint32_t vts,
                    struct xmv200_sns_ae *ae) {
  if (d->chn < MAX_CHN) {
    if (st[d->chn].vts == vts)
      return;
    st[d->chn].vts = vts;
  }
  ae->full_lines = vts;
  ae->max_lines = vts - 3;
  clamp_exposure(d, ae);
  frame_length(d, vts);
}

static void again_calc(uint32_t gain, uint32_t *idx, uint32_t *real) {
  uint32_t g = gain * 32, inv;

  if (g < 0x400)
    g = 0x400;
  else if (g > 0xffff)
    g = 0x10000;
  inv = 0x100000 / g;
  *idx = 0x400 - inv;
  *real = 0x100000 / inv / 32;
}

static void dgain_calc(uint32_t gain, uint32_t *reg, uint32_t *real) {
  *reg = gain;
  *real = gain;
}

static void set_gain(const struct xmv200_sns_dev *d, uint32_t idx,
                     uint32_t dreg) {
  if (d->chn < MAX_CHN) {
    if (st[d->chn].valid & 2 && st[d->chn].again == idx &&
        st[d->chn].dgain == dreg)
      return;
    st[d->chn].again = idx;
    st[d->chn].dgain = dreg;
    st[d->chn].valid |= 2;
  }
  w(d, 0x3103, (idx & 0xfff) >> 8);
  w(d, 0x3104, idx);
  w(d, 0x3709, 0x80);
  w(d, 0x370a, 0x80);
  w(d, 0x370b, 0x80);
  w(d, 0x370c, 0x80);
  w(d, 0x300c, 1);
}

static void set_flip(const struct xmv200_sns_dev *d, int mirror, int flip) {
  uint32_t v = r(d, 0x3007);

  v = mirror ? v & ~1u : v | 1;
  v = flip ? v & ~2u : v | 2;
  w(d, 0x3007, v);
}

static const struct xmv200_sns_mode modes[] = {
    {"the vendor's", {0, 0, 0, 0}, mis40c1, N(mis40c1)},
};

const struct xmv200_sns_drv xmv200_sns_mis40c1 = {
    .name = "MIS40C1",
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

/* ---- XM533 ---- */

/* the XM533 libsns's: its own table (sns_mis40c1_xm533_regs.c), the same
 * functions. Unverified: there is no capture of it. */
extern const struct xmv200_sns_reg xm533_mis40c1[268];

static const struct xmv200_sns_mode xm533_modes[] = {
    {"4M mipi", {0, 0, 0, 0}, xm533_mis40c1, N(xm533_mis40c1)},
};

const struct xmv200_sns_drv xmv200_sns_mis40c1_xm533 = {
    .name = "MIS40C1",
    .family = "xm533",
    .modes = xm533_modes,
    .n_modes = N(xm533_modes),
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

/* ---- XM535 ----
 *
 * The XM535D1 libsns's (Aug 2025), unverified. Its probe: I2C address 0x30
 * (a product preset may give another), 16-bit registers, 8-bit values;
 * 0x3006 written, 100 ms, chip ID 0x3004 << 8 | 0x3005 = 0x0004 (after
 * 0x3000/0x3001 read neither 0x2008 nor 0x1311). Its own table; the frame
 * rate keeps the frame at 1500 lines and sets the line length (0x3107/
 * 0x3108: 99000 / fps, 7920 at 12 fps). The rest is the XM530V200's.
 */

extern const struct xmv200_sns_reg xm535_mis40c1[267];

static void xm535_set_fps(const struct xmv200_sns_dev *d,
                          const struct xmv200_sns_attr *a, uint32_t fps,
                          struct xmv200_sns_ae *ae) {
  uint32_t hts = fps == 12 ? 0x1ef0 : 0x182b8 / (fps ? fps : 1);

  ae->full_lines = 0x5dc;
  ae->max_lines = 0x5d9;
  clamp_exposure(d, ae);
  w(d, 0x3006, 2);
  usleep(100000);
  w(d, 0x3105, 0x05);
  w(d, 0x3106, 0xdc);
  w(d, 0x3107, (hts & 0xffff) >> 8);
  w(d, 0x3108, hts & 0xff);
  w(d, 0x300c, 1);
  w(d, 0x3006, 0);
}

static const struct xmv200_sns_mode xm535_modes[] = {
    {"1080p mipi", {0, 0, 0, 0}, xm535_mis40c1, N(xm535_mis40c1)},
};

const struct xmv200_sns_drv xmv200_sns_mis40c1_xm535 = {
    .name = "MIS40C1",
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
