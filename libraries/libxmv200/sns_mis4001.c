// SPDX-License-Identifier: MIT
/*
 * Imagedesign MIS4001: the XM530V200 vendor libsns's driver of it, as
 * functions. Unverified: no capture has this sensor.
 *
 * Probe (the vendor's, shared by the MIS parts): I2C address 0x30 (0x31
 * for a second sensor on a board whose first is at 0x30), 16-bit
 * registers, 8-bit values; 0x3006 = 1, 100 ms, then chip ID 0x3000 << 8 |
 * 0x3001 = 0x1311. That ID is MIS5001's too: the vendor takes MIS5001 only
 * where the product preset the channel's type (0xa0), MIS4001 otherwise.
 *
 * The vendor picks the mode from the product's resolution (3M or 4M) and a
 * product flag (its sensor attribute 0x11): with it 3M DVP or MIPI at 25
 * fps (20240123) or 4M at 15 fps (20240911, the table's first value set to
 * the interface); without it 3M MIPI at 25 fps (20240701) or 4M at 20 fps.
 * sns.h has neither, so this takes the flag for the 3M DVP and 4M 15 fps
 * tables and not for the others.
 *
 * Exposure 0x3100/0x3101, frame length 0x310c/0x310d (in a 0x3006 = 2
 * hold), analog gain 0x3102 (doublings 0..5 and 32 fine steps; the vendor's
 * gain is in 1/1024, sns.h's in 1/32); no sensor digital gain. Mirror 0x3007
 * bit 0 clear, flip bit 1 clear, channel 1's the other way round. The
 * vendor also inverts both on one product (0x2005), and on 3M pauses and
 * resumes the VI channel after a frame length change; neither is here.
 */

#include <string.h>
#include <unistd.h>

#include "sns.h"
#include "xmv200.h"

extern const struct xmv200_sns_reg mis4001_3m_dvp[176], mis4001_3m_mipi[175],
    mis4001_4m_15fps[178], mis4001_4m_20fps[199];

#define N(a) ((int)(sizeof(a) / sizeof((a)[0])))
#define MAX_CHN 3

static struct {
  int valid, m3, flag; /* the mode: 3M, and the product flag */
  uint32_t exposure, again, dgain, vts;
} st[MAX_CHN];

static int is_3m(const struct xmv200_sns_dev *d) {
  return d->chn < MAX_CHN ? st[d->chn].m3 : 1;
}

static int flagged(const struct xmv200_sns_dev *d) {
  return d->chn < MAX_CHN && st[d->chn].flag;
}

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
    st[d->chn].m3 = m->regs == mis4001_3m_dvp || m->regs == mis4001_3m_mipi;
    st[d->chn].flag = m->regs == mis4001_3m_dvp || m->regs == mis4001_4m_15fps;
  }
  for (int i = 0; i < m->n; i++)
    w(d, m->regs[i].reg,
      i == 0 && m->regs == mis4001_4m_15fps ? !a->dvp : m->regs[i].val);
  usleep(20000);
}

static void ae_default(const struct xmv200_sns_attr *a,
                       struct xmv200_sns_ae *ae) {
  /* the mode's 25 fps 3M or 15/20 fps 4M; ae_default has no channel, so
   * this is the 4M's and set_fps corrects it */
  ae->full_lines = 0x5dc;
  ae->max_lines = 0x5dc - 6;
  ae->again_min = 0x400 / 32;
  ae->again_max = 0x3e00 / 32;
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
  w(d, 0x310c, vts >> 8);
  w(d, 0x310d, vts & 0xff);
  w(d, 0x3006, 0);
}

static void set_fps(const struct xmv200_sns_dev *d,
                    const struct xmv200_sns_attr *a, uint32_t fps,
                    struct xmv200_sns_ae *ae) {
  uint32_t vts;

  if (flagged(d) && is_3m(d))
    vts = fps == 12 ? 0xaf0 : fps == 15 ? 0x91d : 0x578;
  else if (flagged(d))
    vts = fps == 12 ? 0x708 : 0x5dc;
  else
    vts = is_3m(d) ? 0x578 : 0x753; /* 25 or 20 fps */
  ae->full_lines = vts;
  ae->max_lines = vts - 6;
  if (d->chn < MAX_CHN)
    st[d->chn].vts = vts;
  frame_length(d, vts);
}

static void set_vts(const struct xmv200_sns_dev *d, uint32_t vts,
                    struct xmv200_sns_ae *ae) {
  if (flagged(d))
    return;
  if (d->chn < MAX_CHN) {
    if (st[d->chn].vts == vts)
      return;
    st[d->chn].vts = vts;
  }
  ae->full_lines = vts;
  ae->max_lines = vts - 6;
  clamp_exposure(d, ae);
  if (is_3m(d))
    w(d, 0x4240, 0x8d);
  frame_length(d, vts);
  if (is_3m(d))
    w(d, 0x4240, 0x8c);
}

/* up to five doublings and the fine part 0x20..0x3f: 0x3102 = fine - 32
 * + 32 * doublings */
static void again_calc(uint32_t gain, uint32_t *idx, uint32_t *real) {
  uint32_t g = gain < 0x20 ? 0x20 : gain, k = 0;

  while (k < 5 && g >= 0x40) {
    g >>= 1;
    k++;
  }
  *idx = g - 0x20 + k * 0x20;
  *real = g << k;
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
  w(d, 0x3102, idx);
}

static void set_flip(const struct xmv200_sns_dev *d, int mirror, int flip) {
  uint32_t v = r(d, 0x3007);

  if (d->chn) {
    mirror = !mirror;
    flip = !flip;
  }
  v = mirror ? v & ~1u : v | 1;
  v = flip ? v & ~2u : v | 2;
  w(d, 0x3007, v);
}

static const struct xmv200_sns_mode modes[] = {
    {"3M mipi 25fps", {0, 0, 0, 0}, mis4001_3m_mipi, N(mis4001_3m_mipi)},
    {"3M dvp 25fps", {1, 0, 0, 0}, mis4001_3m_dvp, N(mis4001_3m_dvp)},
    {"4M 20fps", {0, 0, 0, 0}, mis4001_4m_20fps, N(mis4001_4m_20fps)},
    {"4M 15fps", {0, 0, 0, 0}, mis4001_4m_15fps, N(mis4001_4m_15fps)},
};

const struct xmv200_sns_drv xmv200_sns_mis4001 = {
    .name = "MIS4001",
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
