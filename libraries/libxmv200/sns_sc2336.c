// SPDX-License-Identifier: MIT
/*
 * SmartSens SC2336, SC2336P and SC2337P: the XM530V200 vendor libsns's
 * drivers of them (snsX20/sc2336_cmos.c and sc2337p_cmos.c in its isp
 * daemon), as functions. The three read the same chip ID in standby
 * (0xcb3a); board.c tells them apart. The mode tables are the vendor's
 * (sns_sc2336_regs.c, tools/xmv200-sns/extract.py).
 *
 * Exposure is 0x3e00..0x3e02 (lines << 4), the coarse analog gain 0x3e09,
 * the fine (digital) gain 0x3e06/0x3e07, line length 0x320c/0x320d, frame
 * length 0x320e/0x320f, mirror and flip 0x3221 (0x06, 0x60). Each setter
 * writes only what changed since the last call, as the vendor's do.
 */

#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "sns.h"
#include "xmv200.h"

extern const struct xmv200_sns_reg sc2336_dvp[158], sc2336_720p_mipi[168],
    sc2336_mipi_1lane[158], sc2336_mipi[149], sc2336p_mipi_1lane[151],
    sc2336p_mipi[149], sc2336p_dvp[160], sc2337p_dvp[158],
    sc2337p_mipi_1lane[158], sc2337p_mipi[150];

#define N(a) ((int)(sizeof(a) / sizeof((a)[0])))
#define MAX_CHN 3

/* the analog gain's steps (1x .. 32x) and their 0x3e09 values */
static const uint32_t again_val[6] = {0x20, 0x40, 0x80, 0x100, 0x200, 0x400};
static const uint8_t again_reg[6] = {0x00, 0x08, 0x09, 0x0b, 0x0f, 0x1f};

/* what each channel's sensor was last given */
static struct {
  int valid;
  uint32_t exposure, again, dgain, vts;
  uint32_t again_cap; /* SC2336/SC2336P: the highest step the temperature
                         allows */
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
                  const struct xmv200_sns_mode *m) {
  for (int i = 0; i < m->n; i++)
    w(d, m->regs[i].reg, m->regs[i].val);
}

static void reset_state(uint32_t chn) {
  if (chn < MAX_CHN) {
    memset(&st[chn], 0, sizeof(st[chn]));
    st[chn].again_cap = 4;
  }
}

/* ---- init ---- */

/* The vendor's 1-lane modes also set the MIPI receiver (0x30004000 =
 * 0x1f83e0) and a pad (0x10020094 = 0x42); a tuning's program does that. */
static void sc2336_init(const struct xmv200_sns_dev *d,
                        const struct xmv200_sns_mode *m,
                        const struct xmv200_sns_attr *a) {
  reset_state(d->chn);
  table(d, m);
  if (m->regs == sc2336_mipi) {
    usleep(100);
    if (a->fps == 25) { /* the 25 fps line timing */
      w(d, 0x3222, 0x02);
      w(d, 0x322e, 0x08);
      w(d, 0x322f, 0xc6);
      w(d, 0x3224, 0x82);
      w(d, 0x300a, 0x24);
      w(d, 0x3250, 0x40);
    }
  }
  usleep(20000);
}

static void sc2336p_init(const struct xmv200_sns_dev *d,
                         const struct xmv200_sns_mode *m,
                         const struct xmv200_sns_attr *a) {
  reset_state(d->chn);
  table(d, m);
  usleep(20000);
}

static void sc2337p_init(const struct xmv200_sns_dev *d,
                         const struct xmv200_sns_mode *m,
                         const struct xmv200_sns_attr *a) {
  reset_state(d->chn);
  table(d, m);
  if (m->regs == sc2337p_mipi)
    usleep(20000);
}

/* ---- AE limits ---- */

static void ae_limits(struct xmv200_sns_ae *ae, uint32_t full, uint32_t max) {
  ae->full_lines = full;
  ae->max_lines = max;
  ae->again_min = 0x20;
  ae->again_max = 0x3ff;
  ae->dgain_min = 0x20;
  ae->dgain_max = 0x80;
}

static void sc2336_ae_default(const struct xmv200_sns_attr *a,
                              struct xmv200_sns_ae *ae) {
  uint32_t full = a->p720 ? 800 : 1125;

  ae_limits(ae, full, full - 6);
}

static void sc2337p_ae_default(const struct xmv200_sns_attr *a,
                               struct xmv200_sns_ae *ae) {
  ae_limits(ae, 1125, 1119);
}

/* ---- exposure ---- */

static void set_exposure(const struct xmv200_sns_dev *d, uint32_t lines) {
  if (d->chn < MAX_CHN && st[d->chn].valid & 1 &&
      st[d->chn].exposure == lines)
    return;
  if (d->chn < MAX_CHN) {
    st[d->chn].exposure = lines;
    st[d->chn].valid |= 1;
  }
  w(d, 0x3e02, (lines & 0xf) << 4);
  w(d, 0x3e01, (lines & 0xfff) >> 4);
  w(d, 0x3e00, (lines & 0xffff) >> 12);
}

/* a shorter frame takes the exposure down with it */
static void clamp_exposure(const struct xmv200_sns_dev *d,
                           const struct xmv200_sns_ae *ae) {
  if (d->chn < MAX_CHN && st[d->chn].valid & 1 &&
      ae->max_lines < st[d->chn].exposure)
    set_exposure(d, ae->max_lines);
}

/* ---- frame rate ---- */

static void timing(const struct xmv200_sns_dev *d, uint32_t hts, uint32_t vts,
                   uint8_t hts_mask, uint8_t vts_mask,
                   struct xmv200_sns_ae *ae) {
  ae->full_lines = vts;
  ae->max_lines = vts - 6;
  clamp_exposure(d, ae);
  w(d, 0x320c, hts >> 8);
  w(d, 0x320d, hts & hts_mask);
  w(d, 0x320e, vts >> 8);
  w(d, 0x320f, vts & vts_mask);
  if (d->chn < MAX_CHN)
    st[d->chn].vts = vts;
}

/* SC2336: 15 and 12 fps by line and frame length from 30 and 25; 26 and
 * 31 the vendor's alternative 25 and 30 fps timings */
static void sc2336_set_fps(const struct xmv200_sns_dev *d,
                           const struct xmv200_sns_attr *a, uint32_t fps,
                           struct xmv200_sns_ae *ae) {
  uint32_t hts, vts;

  switch (fps) {
  case 15:
    if (a->one_lane)
      w(d, 0x301f, 0x8f);
    hts = 0x898, vts = 0x8ca;
    break;
  case 12:
    if (a->one_lane)
      w(d, 0x301f, 0x90);
    hts = 0xa50, vts = 0x8ca;
    break;
  case 26:
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
      hts = 66000 / fps, vts = 1125;
  }
  timing(d, hts, vts, 0xff, 0xff, ae);
}

static void sc2336p_set_fps(const struct xmv200_sns_dev *d,
                            const struct xmv200_sns_attr *a, uint32_t fps,
                            struct xmv200_sns_ae *ae) {
  uint32_t hts, vts = 0x465;

  switch (fps) {
  case 15:
    w(d, 0x301f, a->one_lane ? 0x14 : 0x0e);
    hts = 0x898, vts = 0x8ca;
    break;
  case 12:
    w(d, 0x301f, a->one_lane ? 0x15 : 0x0f);
    hts = 0xa50, vts = 0x8ca;
    break;
  case 25:
    w(d, 0x301f, a->one_lane ? 0x15 : 0x0f);
    hts = 0xa50;
    break;
  case 30:
    w(d, 0x301f, a->one_lane ? 0x14 : 0x0e);
    hts = 0x898;
    break;
  default:
    hts = 66000 / (fps ? fps : 1);
  }
  timing(d, hts, vts, 0xff, 0xef, ae);
}

static void sc2337p_set_fps(const struct xmv200_sns_dev *d,
                            const struct xmv200_sns_attr *a, uint32_t fps,
                            struct xmv200_sns_ae *ae) {
  uint32_t hts, vts = 0x465;

  switch (fps) {
  case 15:
    w(d, 0x301f, 3);
    hts = 0x898, vts = 0x8ca;
    break;
  case 12:
    w(d, 0x301f, 4);
    hts = 0xa50, vts = 0x8ca;
    break;
  case 20:
    w(d, 0x301f, 3);
    hts = 0xce4;
    break;
  case 25:
    w(d, 0x301f, 4);
    hts = 0xa50;
    break;
  default: /* 30 */
    w(d, 0x301f, 3);
    hts = 0x898;
  }
  if (a->dvp)
    w(d, 0x301f, 0x1f);
  timing(d, hts, vts, 0xfc, 0xef, ae);
}

static void set_vts(const struct xmv200_sns_dev *d, uint32_t vts,
                    struct xmv200_sns_ae *ae) {
  if (d->chn < MAX_CHN && st[d->chn].vts == vts)
    return;
  if (d->chn < MAX_CHN)
    st[d->chn].vts = vts;
  ae->full_lines = vts;
  ae->max_lines = vts - 6;
  clamp_exposure(d, ae);
  w(d, 0x320e, vts >> 8);
  w(d, 0x320f, vts & 0xff);
}

/* ---- gain ---- */

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

static void gain_write(const struct xmv200_sns_dev *d, uint32_t idx,
                       uint32_t dreg) {
  if (d->chn < MAX_CHN) {
    if (st[d->chn].valid & 2 && st[d->chn].again == idx &&
        st[d->chn].dgain == dreg)
      return;
    st[d->chn].again = idx;
    st[d->chn].dgain = dreg;
    st[d->chn].valid |= 2;
  }
  w(d, 0x3e09, again_reg[idx < 6 ? idx : 5]);
  w(d, 0x3e06, (dreg & 0xffff) >> 8);
  w(d, 0x3e07, dreg & 0xff);
}

/*
 * SC2336 and SC2336P: the analog gain's cap follows the sensor's
 * temperature reading (0x3974): down a step (to 4x at least) at 0x18 and
 * above. The vendor's raises it again only below 2x, which it never gets
 * to, so a cap once lowered stays; this does the same.
 */
static void sc2336_set_gain(const struct xmv200_sns_dev *d, uint32_t idx,
                            uint32_t dreg) {
  uint32_t t = r(d, 0x3974), *cap;

  if (d->chn >= MAX_CHN) {
    gain_write(d, idx, dreg);
    return;
  }
  cap = &st[d->chn].again_cap;
  if (t >= 0x18) {
    if (*cap >= 3)
      (*cap)--;
  } else if (t <= 0x10 && *cap <= 1) {
    (*cap)++;
  }
  gain_write(d, idx < *cap ? idx : *cap, dreg);
}

/* ---- mirror and flip ---- */

/* channel 1's sensor is mounted the other way round */
static void set_flip(const struct xmv200_sns_dev *d, int mirror, int flip) {
  uint32_t v = r(d, 0x3221);

  if (d->chn) {
    mirror = !mirror;
    flip = !flip;
  }
  v = mirror ? v | 0x06 : v & ~0x06u;
  v = flip ? v | 0x60 : v & ~0x60u;
  w(d, 0x3221, v);
}

/* ---- the drivers ---- */

#define MODE(nm, dv, one, p7, t) {nm, {dv, one, p7, 0}, t, N(t)}

static const struct xmv200_sns_mode sc2336_modes[] = {
    MODE("1080p mipi 1 lane", 0, 1, 0, sc2336_mipi_1lane),
    MODE("1080p mipi", 0, 0, 0, sc2336_mipi),
    MODE("720p mipi", 0, 0, 1, sc2336_720p_mipi),
    MODE("1080p dvp", 1, 0, 0, sc2336_dvp),
};
static const struct xmv200_sns_mode sc2336p_modes[] = {
    MODE("1080p mipi 1 lane", 0, 1, 0, sc2336p_mipi_1lane),
    MODE("1080p mipi", 0, 0, 0, sc2336p_mipi),
    MODE("1080p dvp", 1, 0, 0, sc2336p_dvp),
};
static const struct xmv200_sns_mode sc2337p_modes[] = {
    MODE("1080p mipi 1 lane", 0, 1, 0, sc2337p_mipi_1lane),
    MODE("1080p mipi", 0, 0, 0, sc2337p_mipi),
    MODE("1080p dvp", 1, 0, 0, sc2337p_dvp),
};

#define DRV(nm, md, in, aed, fps, gain)                                        \
  {.name = nm,                                                                 \
   .modes = md,                                                                \
   .n_modes = N(md),                                                           \
   .init = in,                                                                 \
   .ae_default = aed,                                                          \
   .set_fps = fps,                                                             \
   .set_vts = set_vts,                                                         \
   .set_exposure = set_exposure,                                               \
   .again_calc = again_calc,                                                   \
   .dgain_calc = dgain_calc,                                                   \
   .set_gain = gain,                                                           \
   .set_flip = set_flip}

const struct xmv200_sns_drv xmv200_sns_sc2336 =
    DRV("SC2336", sc2336_modes, sc2336_init, sc2336_ae_default,
        sc2336_set_fps, sc2336_set_gain);
const struct xmv200_sns_drv xmv200_sns_sc2336p =
    DRV("SC2336P", sc2336p_modes, sc2336p_init, sc2336_ae_default,
        sc2336p_set_fps, sc2336_set_gain);
const struct xmv200_sns_drv xmv200_sns_sc2337p =
    DRV("SC2337P", sc2337p_modes, sc2337p_init, sc2337p_ae_default,
        sc2337p_set_fps, gain_write);

/* ---- XM535W1 ----
 *
 * The XM535W1 libsns's (Apr 2024): SC2336 has one mode, SC2336P three, both
 * ending with stream on (0x0100 = 1, in their tables). They share their AE
 * functions; gain, exposure, frame length and the gain steps are the
 * XM530V200's, the frame rates' 0x301f values and flip differ.
 */

extern const struct xmv200_sns_reg xm535w1_sc2336_mipi[145],
    xm535w1_sc2336p_mipi_1lane[168], xm535w1_sc2336p_mipi[150],
    xm535w1_sc2336p_dvp[161];

static void w1_init(const struct xmv200_sns_dev *d,
                    const struct xmv200_sns_mode *m,
                    const struct xmv200_sns_attr *a) {
  reset_state(d->chn);
  table(d, m);
  usleep(20000);
}

static void w1_ae_default(const struct xmv200_sns_attr *a,
                          struct xmv200_sns_ae *ae) {
  uint32_t full = a->p720 ? 750 : 1125;

  ae_limits(ae, full, full - 6);
}

static void w1_set_fps(const struct xmv200_sns_dev *d,
                       const struct xmv200_sns_attr *a, uint32_t fps,
                       struct xmv200_sns_ae *ae) {
  uint32_t hts, vts;

  switch (fps) {
  case 15:
    if (a->one_lane)
      w(d, 0x301f, 0x3b);
    hts = 0x898, vts = 0x8ca;
    break;
  case 10:
    if (a->one_lane)
      w(d, 0x301f, 0x3b);
    hts = 0x898, vts = 0xd2f;
    break;
  case 12:
    if (a->one_lane)
      w(d, 0x301f, 0x48);
    hts = 0xa50, vts = 0x8ca;
    break;
  case 26:
    hts = 0x948, vts = 0x4e2;
    break;
  case 31:
    hts = 0x898, vts = 0x464;
    break;
  default:
    hts = 66000 / (fps ? fps : 1), vts = 0x465;
  }
  timing(d, hts, vts, 0xff, 0xef, ae);
}

/* flip is 0xe0, and channel 1 is not turned round */
static void w1_set_flip(const struct xmv200_sns_dev *d, int mirror, int flip) {
  uint32_t v = r(d, 0x3221);

  v = mirror ? v | 0x06 : v & ~0x06u;
  v = flip ? v | 0xe0 : v & ~0xe0u;
  w(d, 0x3221, v);
}

static const struct xmv200_sns_mode w1_sc2336_modes[] = {
    MODE("1080p mipi", 0, 0, 0, xm535w1_sc2336_mipi),
};
static const struct xmv200_sns_mode w1_sc2336p_modes[] = {
    MODE("1080p mipi 1 lane", 0, 1, 0, xm535w1_sc2336p_mipi_1lane),
    MODE("1080p mipi", 0, 0, 0, xm535w1_sc2336p_mipi),
    MODE("1080p dvp", 1, 0, 0, xm535w1_sc2336p_dvp),
};

#define W1_DRV(nm, md)                                                         \
  {.name = nm,                                                                 \
   .family = "xm535w1",                                                        \
   .modes = md,                                                                \
   .n_modes = N(md),                                                           \
   .init = w1_init,                                                            \
   .ae_default = w1_ae_default,                                                \
   .set_fps = w1_set_fps,                                                      \
   .set_vts = set_vts,                                                         \
   .set_exposure = set_exposure,                                               \
   .again_calc = again_calc,                                                   \
   .dgain_calc = dgain_calc,                                                   \
   .set_gain = sc2336_set_gain,                                                \
   .set_flip = w1_set_flip}

const struct xmv200_sns_drv xmv200_sns_sc2336_xm535w1 =
    W1_DRV("SC2336", w1_sc2336_modes);
const struct xmv200_sns_drv xmv200_sns_sc2336p_xm535w1 =
    W1_DRV("SC2336P", w1_sc2336p_modes);

/* ---- XM535 ---- */

/*
 * The XM535D1's libsns (Aug 12 2025 d2db8e4): SC2336 has one table; SC2336P
 * (and SC2337P, the same functions) DVP, MIPI on one lane, MIPI. Exposure,
 * gain and their steps are the XM530V200's. On a frame-synced board
 * (cmos_set_sns_attr's +0x20 with 0x11) channel 0 is the slave: its 1-lane
 * init and every frame length change write 0x301f = 0x47 and 0x322e/0x322f
 * = VTS - 4. The synced modes' registers are the same as the others', so a
 * tuning picks the synced one (the first). The 2-lane synced (0x19)
 * master/slave tables are not done.
 */
extern const struct xmv200_sns_reg xm535_sc2336_mipi[145],
    xm535_sc2336p_dvp[160], xm535_sc2336p_mipi_1lane_sync[172],
    xm535_sc2336p_mipi_1lane[167], xm535_sc2336p_mipi[149];

static const struct xmv200_sns_mode xm535_sc2336_modes[] = {
    MODE("1080p mipi 1 lane, frame sync", 0, 1, 0, xm535_sc2336_mipi),
    MODE("1080p mipi 1 lane", 0, 1, 0, xm535_sc2336_mipi),
};
static const struct xmv200_sns_mode xm535_sc2336p_modes[] = {
    MODE("1080p mipi 1 lane, frame sync", 0, 1, 0,
         xm535_sc2336p_mipi_1lane_sync),
    MODE("1080p mipi 1 lane", 0, 1, 0, xm535_sc2336p_mipi_1lane),
    MODE("1080p mipi", 0, 0, 0, xm535_sc2336p_mipi),
    MODE("1080p dvp", 1, 0, 0, xm535_sc2336p_dvp),
};

static int xm535_sync[MAX_CHN];

static int xm535_synced(const struct xmv200_sns_mode *m) {
  return m == &xm535_sc2336_modes[0] || m == &xm535_sc2336p_modes[0];
}

static void xm535_init(const struct xmv200_sns_dev *d,
                       const struct xmv200_sns_mode *m,
                       const struct xmv200_sns_attr *a) {
  reset_state(d->chn);
  if (d->chn < MAX_CHN)
    xm535_sync[d->chn] = xm535_synced(m);
  if (m->regs == xm535_sc2336p_mipi_1lane_sync && d->chn) {
    /* the table, then the master's writes in place of the slave's */
    for (int i = 0; i < 166; i++)
      w(d, m->regs[i].reg, m->regs[i].val);
    w(d, 0x3032, 0xa0);
    w(d, 0x3224, 0x82);
    w(d, 0x300a, 0x24);
    w(d, 0x0100, 0x01);
  } else {
    table(d, m);
  }
  usleep(20000);
}

static void xm535_ae_default(const struct xmv200_sns_attr *a,
                             struct xmv200_sns_ae *ae) {
  uint32_t full = a->p720 ? 750 : 1125;

  ae_limits(ae, full, full - 6);
}

static int xm535_slave(const struct xmv200_sns_dev *d) {
  return d->chn == 0 && xm535_sync[0];
}

static void xm535_set_fps(const struct xmv200_sns_dev *d,
                          const struct xmv200_sns_attr *a, uint32_t fps,
                          struct xmv200_sns_ae *ae) {
  uint32_t hts, vts;

  switch (fps) {
  case 15:
    if (a->one_lane)
      w(d, 0x301f, 0x3b);
    hts = 2200, vts = 2250;
    break;
  case 10:
    if (a->one_lane)
      w(d, 0x301f, 0x3b);
    hts = 2200, vts = 3375;
    break;
  case 12:
    if (a->one_lane)
      w(d, 0x301f, 0x48);
    hts = 2640, vts = 2250;
    break;
  case 26:
    hts = 2376, vts = 1250;
    break;
  case 31:
    hts = 2200, vts = 1124;
    break;
  default:
    hts = 66000 / (fps ? fps : 1), vts = 1125;
  }
  timing(d, hts, vts, 0xff, 0xef, ae);
  if (xm535_slave(d) && a->one_lane) {
    w(d, 0x301f, 0x47);
    w(d, 0x322e, (vts - 4) >> 8);
    w(d, 0x322f, (vts - 4) & 0xff);
  }
}

static void xm535_set_vts(const struct xmv200_sns_dev *d, uint32_t vts,
                          struct xmv200_sns_ae *ae) {
  if (d->chn < MAX_CHN && st[d->chn].vts == vts)
    return;
  if (d->chn < MAX_CHN)
    st[d->chn].vts = vts;
  ae->full_lines = vts;
  ae->max_lines = vts - 6;
  clamp_exposure(d, ae);
  w(d, 0x320e, vts >> 8);
  w(d, 0x320f, vts & 0xff);
  if (xm535_slave(d)) {
    w(d, 0x322e, (vts - 4) >> 8);
    w(d, 0x322f, (vts - 4) & 0xff);
  }
}

/* mirror 0x06, flip 0xe0; no channel turned round */
static void xm535_set_flip(const struct xmv200_sns_dev *d, int mirror,
                           int flip) {
  uint32_t v = r(d, 0x3221);

  v = mirror ? v | 0x06 : v & ~0x06u;
  v = flip ? v | 0xe0 : v & ~0xe0u;
  w(d, 0x3221, v);
}

#define XM535_DRV(nm, md)                                                      \
  {.name = nm,                                                                 \
   .family = "xm535",                                                          \
   .modes = md,                                                                \
   .n_modes = N(md),                                                           \
   .init = xm535_init,                                                         \
   .ae_default = xm535_ae_default,                                             \
   .set_fps = xm535_set_fps,                                                   \
   .set_vts = xm535_set_vts,                                                   \
   .set_exposure = set_exposure,                                               \
   .again_calc = again_calc,                                                   \
   .dgain_calc = dgain_calc,                                                   \
   .set_gain = sc2336_set_gain,                                                \
   .set_flip = xm535_set_flip}

const struct xmv200_sns_drv xmv200_sns_sc2336_xm535 =
    XM535_DRV("SC2336", xm535_sc2336_modes);
const struct xmv200_sns_drv xmv200_sns_sc2336p_xm535 =
    XM535_DRV("SC2336P", xm535_sc2336p_modes);
const struct xmv200_sns_drv xmv200_sns_sc2337p_xm535 =
    XM535_DRV("SC2337P", xm535_sc2336p_modes);
