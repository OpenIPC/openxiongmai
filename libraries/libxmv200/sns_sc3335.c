// SPDX-License-Identifier: MIT
/*
 * SmartSens SC3335: the XM530V200 vendor libsns's driver of it, as
 * functions. Four modes: 3 MP (2304x1296, 1500 lines) and 1 MP (800
 * lines), each on MIPI or DVP. The mode tables are the vendor's
 * (sns_sc3335_regs.c, tools/xmv200-sns/extract.py).
 *
 * Exposure is 0x3e00..0x3e02 in half lines (lines << 5), the analog gain
 * 0x3e08 (doublings) and 0x3e09 (fine), the digital gain 0x3e06/0x3e07.
 * With the total gain the vendor switches two analog settings (0x363c,
 * 0x330e) under group hold (0x3812), 0x363c by the chip's revision
 * (0x3109). Each setter writes only what changed since the last call, as
 * the vendor's do (the vendor keeps one exposure for all channels; this
 * keeps one a channel).
 */

#include <string.h>
#include <unistd.h>

#include "sns.h"
#include "xmv200.h"

extern const struct xmv200_sns_reg sc3335_1m_dvp[126], sc3335_1m_mipi[115],
    sc3335_3m_dvp[108], sc3335_3m_mipi[111];

#define N(a) ((int)(sizeof(a) / sizeof((a)[0])))
#define MAX_CHN 3

/* per mode (1 MP, 3 MP) and total gain (below 2x, 4x, 8x, above): 0x363c
 * on other revisions, 0x363c on revision 1, 0x330e */
static const uint8_t gain_logic[2][4][3] = {
    {{0x05, 0x05, 0x10}, {0x07, 0x07, 0x10}, {0x07, 0x07, 0x10},
     {0x07, 0x07, 0x10}},
    {{0x05, 0x05, 0x20}, {0x07, 0x07, 0x1c}, {0x07, 0x07, 0x1c},
     {0x07, 0x07, 0x12}}};

static struct {
  int valid;
  uint32_t exposure, again, dgain, vts;
  uint32_t level[2]; /* per mode: gain_logic's row, 0xf: none yet */
} st[MAX_CHN];

static void w(const struct xmv200_sns_dev *d, uint16_t reg, uint32_t val) {
  xmv200_i2c_write(d->bus, d->addr, reg, 2, val & 0xff, 1);
}

static uint32_t r(const struct xmv200_sns_dev *d, uint16_t reg) {
  uint32_t v = 0;

  xmv200_i2c_read(d->bus, d->addr, reg, 2, &v, 1);
  return v & 0xff;
}

/* the mode each channel's sensor runs, which the gain's settings follow */
static int p720[MAX_CHN];

static void init(const struct xmv200_sns_dev *d,
                 const struct xmv200_sns_mode *m,
                 const struct xmv200_sns_attr *a) {
  if (d->chn < MAX_CHN) {
    memset(&st[d->chn], 0, sizeof(st[d->chn]));
    st[d->chn].level[0] = st[d->chn].level[1] = 0xf;
    p720[d->chn] = a->p720;
  }
  for (int i = 0; i < m->n; i++)
    w(d, m->regs[i].reg, m->regs[i].val);
  usleep(20000);
}

/* the vendor's limits, its gains in 1/1024 and 1/128, here in 1/32 */
static void ae_default(const struct xmv200_sns_attr *a,
                       struct xmv200_sns_ae *ae) {
  uint32_t full = a->p720 ? 800 : 1500;

  ae->full_lines = full;
  ae->max_lines = full - 5;
  ae->again_min = 0x400 >> 5;
  ae->again_max = 0x3f00 >> 5;
  ae->dgain_min = 0x80 >> 2;
  ae->dgain_max = 0xfc0 >> 2;
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

/*
 * By the line length (0x320c/0x320d, the vendor's count halved). 1 MP: 30
 * fps, else 25 (with 0x301f and a line length first written as 0x4b0); its
 * frame length is left as the mode's. 3 MP: 25, 30, else 20, at 1500
 * lines.
 */
static void set_fps(const struct xmv200_sns_dev *d,
                    const struct xmv200_sns_attr *a, uint32_t fps,
                    struct xmv200_sns_ae *ae) {
  uint32_t hts;

  if (a->p720) {
    if (fps == 30) {
      hts = 2000;
    } else {
      w(d, 0x301f, 0x4c);
      w(d, 0x320c, 0x04);
      w(d, 0x320d, 0xb0);
      hts = 0x960;
    }
    ae->full_lines = 800;
    ae->max_lines = 795;
    w(d, 0x320c, hts >> 9);
    w(d, 0x320d, hts >> 1 & 0xf8);
    return;
  }
  hts = fps == 25 ? 0xd20 : fps == 30 ? 0xaf0 : 0x1068;
  ae->full_lines = 1500;
  ae->max_lines = 1496;
  w(d, 0x320c, hts >> 9);
  w(d, 0x320d, hts >> 1 & 0xfc);
  w(d, 0x320e, 0x05);
  w(d, 0x320f, 0xdc);
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
  ae->max_lines = vts - 5;
  if (d->chn < MAX_CHN && st[d->chn].valid & 1 &&
      ae->max_lines < st[d->chn].exposure)
    set_exposure(d, ae->max_lines);
  w(d, 0x320e, vts >> 8);
  w(d, 0x320f, vts & 0xff);
}

/* 1x .. 16x: doublings as ones in 0x3e08, the rest in 0x3e09 (0x40 = 1x) */
static void again_calc(uint32_t gain, uint32_t *idx, uint32_t *real) {
  uint32_t g = gain << 1, bits = 0, k = 0; /* the vendor's gain >> 4 */

  if (g < 0x10)
    g = 0x10;
  while (k < 4 && g >= 0x80) {
    k++;
    g >>= 1;
    bits = bits << 1 | 1;
  }
  *idx = g + bits * 0x100;
  *real = (g << k) >> 1;
}

/* 1x .. 32x: doublings in 0x3e06, the rest in 0x3e07 (0x80 = 1x) */
static void dgain_calc(uint32_t gain, uint32_t *reg, uint32_t *real) {
  uint32_t g = gain << 2, bits = 0, k = 0;

  while (k < 5 && g >= 0x100) {
    k++;
    g >>= 1;
    bits = bits << 1 | 1;
  }
  *reg = g + bits * 0x100;
  *real = (g << k) >> 2;
}

static uint32_t unpack(uint32_t reg) {
  return (reg & 0xff) << __builtin_popcount((reg >> 8) & 0xff);
}

static void set_gain(const struct xmv200_sns_dev *d, uint32_t areg,
                     uint32_t dreg) {
  /* total gain: 0x400 x 0x80 = 1x, >> 13 */
  uint32_t total = (uint32_t)(((uint64_t)(unpack(areg) << 4) * unpack(dreg)) >>
                              13);
  uint32_t lvl = total < 0x20 ? 0 : total < 0x40 ? 1 : total < 0x80 ? 2 : 3;
  int m = d->chn < MAX_CHN ? !p720[d->chn] : 1;
  uint32_t rev = r(d, 0x3109);

  if (d->chn >= MAX_CHN || st[d->chn].level[m] != lvl) {
    if (d->chn < MAX_CHN)
      st[d->chn].level[m] = lvl;
    w(d, 0x3812, 0x00);
    w(d, 0x363c, gain_logic[m][lvl][rev == 1]);
    w(d, 0x330e, gain_logic[m][lvl][2]);
    w(d, 0x3812, 0x30);
  }
  if (d->chn < MAX_CHN) {
    if (st[d->chn].valid & 2 && st[d->chn].again == areg &&
        st[d->chn].dgain == dreg)
      return;
    st[d->chn].again = areg;
    st[d->chn].dgain = dreg;
    st[d->chn].valid |= 2;
  }
  w(d, 0x3e08, ((areg & 0xffff) >> 8) << 2 | 3);
  w(d, 0x3e09, areg & 0xff);
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

#define MODE(nm, dv, p7, t) {nm, {dv, 0, p7, 0}, t, N(t)}

static const struct xmv200_sns_mode modes[] = {
    MODE("3 MP mipi", 0, 0, sc3335_3m_mipi),
    MODE("3 MP dvp", 1, 0, sc3335_3m_dvp),
    MODE("1 MP mipi", 0, 1, sc3335_1m_mipi),
    MODE("1 MP dvp", 1, 1, sc3335_1m_dvp),
};

const struct xmv200_sns_drv xmv200_sns_sc3335 = {
    .name = "SC3335",
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

/* the tuning's exposure is in the register's half lines */
static void run(const struct xmv200_sns_dev *d, int n,
                const struct xmv200_sns_ctx *c) {
  const struct xmv200_sns_drv *drv = &xmv200_sns_sc3335;
  uint32_t areg, dreg, real;

  switch (n) {
  case 0:
    xmv200_sns_init_step(drv, d, c);
    break;
  case 1:
    xmv200_sns_fps_step(drv, d, c);
    break;
  case 2:
    set_exposure(d, c->exp_lines / 2);
    again_calc(0x20, &areg, &real);
    dgain_calc(0x20, &dreg, &real);
    set_gain(d, areg, dreg);
    break;
  }
}

/* ---- XM535W1 ----
 *
 * The XM535W1 libsns's (Apr 2024), unverified (no tuning): one 3 MP mode
 * at 1350 lines and no frame rate setting (the vendor's function is
 * empty); exposure, frame length, the gain and its settings are the
 * XM530V200's (its 3 MP rows).
 */

extern const struct xmv200_sns_reg xm535w1_sc3335_3m[113];

static void w1_ae_default(const struct xmv200_sns_attr *a,
                          struct xmv200_sns_ae *ae) {
  ae->full_lines = 0x546;
  ae->max_lines = 0x541;
  ae->again_min = 0x400 >> 5;
  ae->again_max = 0x3f00 >> 5;
  ae->dgain_min = 0x80 >> 2;
  ae->dgain_max = 0xfc0 >> 2;
}

static void w1_set_fps(const struct xmv200_sns_dev *d,
                       const struct xmv200_sns_attr *a, uint32_t fps,
                       struct xmv200_sns_ae *ae) {
}

static void w1_run(const struct xmv200_sns_dev *d, int n,
                   const struct xmv200_sns_ctx *c);

static const struct xmv200_sns_mode w1_modes[] = {
    MODE("3 MP mipi", 0, 0, xm535w1_sc3335_3m),
};

const struct xmv200_sns_drv xmv200_sns_sc3335_xm535w1 = {
    .name = "SC3335",
    .family = "xm535w1",
    .exp_shift = 1,
    .modes = w1_modes,
    .n_modes = N(w1_modes),
    .init = init,
    .ae_default = w1_ae_default,
    .set_fps = w1_set_fps,
    .set_vts = set_vts,
    .set_exposure = set_exposure,
    .again_calc = again_calc,
    .dgain_calc = dgain_calc,
    .set_gain = set_gain,
    .set_flip = set_flip,
    .run = w1_run,
};

static void w1_run(const struct xmv200_sns_dev *d, int n,
                   const struct xmv200_sns_ctx *c) {
  const struct xmv200_sns_drv *drv = &xmv200_sns_sc3335_xm535w1;
  uint32_t areg, dreg, real;

  switch (n) {
  case 0:
    xmv200_sns_init_step(drv, d, c);
    break;
  case 1:
    xmv200_sns_fps_step(drv, d, c);
    break;
  case 2:
    set_exposure(d, c->exp_lines / 2);
    again_calc(0x20, &areg, &real);
    dgain_calc(0x20, &dreg, &real);
    set_gain(d, areg, dreg);
    break;
  }
}
