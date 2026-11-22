// SPDX-License-Identifier: MIT
/*
 * Imagedesign MIS2009: the XM530V200 vendor libsns's driver of it, as
 * functions. Unverified: no capture has this sensor.
 *
 * Probe (the vendor's, shared by the MIS parts): I2C address 0x30 (0x31
 * for a second sensor on a board whose first is at 0x30), 16-bit
 * registers, 8-bit values; 0x3006 = 1, 100 ms, then chip ID 0x3000 << 8 |
 * 0x3001 = 0x2008 (0x3004/0x3005 when that reads neither 0x2008 nor
 * 0x1311).
 *
 * Modes: 1080p DVP and MIPI, 25 fps. Exposure 0x3100/0x3101, frame length
 * 0x3200/0x3201 (changed in a 0x3006 = 2 hold), line length 0x3202/0x3203,
 * analog gain 0x3102 (a doubling 0..3 and 32 fine steps), digital gain
 * 0x3702..0x3705 (1/32). Mirror 0x3007 bit 0 clear, flip bit 1 clear;
 * channel 1's sensor the other way round. The vendor's analog gain is in
 * 1/1024; sns.h's in 1/32.
 */

#include <string.h>
#include <unistd.h>

#include "sns.h"
#include "xmv200.h"

extern const struct xmv200_sns_reg mis2009_dvp[161], mis2009_mipi[161];

#define N(a) ((int)(sizeof(a) / sizeof((a)[0])))
#define MAX_CHN 3

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
  if (m->regs == mis2009_dvp)
    w(d, 0x3008, d->chn == 2 ? 0 : 0xff);
  usleep(20000);
}

static void ae_default(const struct xmv200_sns_attr *a,
                       struct xmv200_sns_ae *ae) {
  ae->full_lines = 0x465;
  ae->max_lines = 0x463;
  ae->again_min = 0x400 / 32;
  ae->again_max = 0x3f00 / 32;
  ae->dgain_min = 0x20;
  ae->dgain_max = 0xff;
}

static void set_exposure(const struct xmv200_sns_dev *d, uint32_t lines) {
  if (d->chn < MAX_CHN) {
    if (st[d->chn].valid & 1 && st[d->chn].exposure == lines)
      return;
    st[d->chn].exposure = lines;
    st[d->chn].valid |= 1;
  }
  w(d, 0x3100, (lines & 0xffff) >> 8);
  w(d, 0x3101, lines & 0xff);
}

static void clamp_exposure(const struct xmv200_sns_dev *d,
                           const struct xmv200_sns_ae *ae) {
  if (d->chn < MAX_CHN && st[d->chn].valid & 1 &&
      ae->max_lines < st[d->chn].exposure)
    set_exposure(d, ae->max_lines);
}

static void set_fps(const struct xmv200_sns_dev *d,
                    const struct xmv200_sns_attr *a, uint32_t fps,
                    struct xmv200_sns_ae *ae) {
  uint32_t hts, vts = 0x465;

  switch (fps) {
  case 20:
    hts = 0xd20;
    break;
  case 12:
    hts = 0xa80, vts = 0x8ca;
    break;
  case 15:
    w(d, 0x3303, 5);
    hts = 0x8c0, vts = 0x8ca;
    break;
  case 25:
    hts = 0xa80;
    break;
  case 30:
    w(d, 0x3303, 5);
    hts = 0x8c0;
    break;
  default:
    hts = 0x6e4000 / (fps ? fps : 1);
  }
  ae->full_lines = vts;
  ae->max_lines = vts - 2;
  if (d->chn < MAX_CHN)
    st[d->chn].vts = vts;
  w(d, 0x3203, hts & 0xff);
  w(d, 0x3202, (hts & 0xffff) >> 8);
  w(d, 0x3201, vts & 0xef);
  w(d, 0x3200, vts >> 8);
}

static void set_vts(const struct xmv200_sns_dev *d, uint32_t vts,
                    struct xmv200_sns_ae *ae) {
  if (d->chn < MAX_CHN) {
    if (st[d->chn].vts == vts)
      return;
    st[d->chn].vts = vts;
  }
  ae->full_lines = vts;
  ae->max_lines = vts - 2;
  clamp_exposure(d, ae);
  w(d, 0x3006, 2);
  usleep(100000);
  w(d, 0x3200, vts >> 8);
  w(d, 0x3201, vts & 0xff);
  w(d, 0x3006, 0);
}

/* a doubling 0..3 and the fine part 0x20..0x3f: 0x3102 = fine - 32 + 32 *
 * doublings */
static void again_calc(uint32_t gain, uint32_t *idx, uint32_t *real) {
  uint32_t g = gain, k = 0;

  if (g < 0x20)
    g = 0x20;
  if (g > 0x1f7)
    g = 0x1f8;
  while (k < 3 && g >> k >= 0x40)
    k++;
  g >>= k;
  *idx = g - 0x20 + k * 0x20;
  *real = g << k;
}

static void dgain_calc(uint32_t gain, uint32_t *reg, uint32_t *real) {
  *reg = gain;
  *real = gain;
}

static void set_gain(const struct xmv200_sns_dev *d, uint32_t idx,
                     uint32_t dreg) {
  if (d->chn >= MAX_CHN || !(st[d->chn].valid & 2) ||
      st[d->chn].again != idx)
    w(d, 0x3102, idx);
  if (d->chn >= MAX_CHN || !(st[d->chn].valid & 4) ||
      st[d->chn].dgain != dreg) {
    w(d, 0x3702, dreg);
    w(d, 0x3703, dreg);
    w(d, 0x3704, dreg);
    w(d, 0x3705, dreg);
  }
  if (d->chn < MAX_CHN) {
    st[d->chn].again = idx;
    st[d->chn].dgain = dreg;
    st[d->chn].valid |= 6;
  }
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
    {"1080p mipi", {0, 0, 0, 0}, mis2009_mipi, N(mis2009_mipi)},
    {"1080p dvp", {1, 0, 0, 0}, mis2009_dvp, N(mis2009_dvp)},
};

/* the probe: the vendor's MIS probe */
static uint32_t probe(int bus, uint8_t addr) {
  return xmv200_sns_mis_id(bus, addr) == 0x2008 ? 0x2008 : 0;
}

const struct xmv200_sns_drv xmv200_sns_mis2009 = {
    .probe = probe,
    .probe_addr = {0x30, 0x31},
    .name = "MIS2009",
    .n_runs = 4,
    .steps = xmv200_sns_steps_ifap,
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

/*
 * The XM533 libsns's: one table (1080p linear 25 fps) and a flip that sets
 * 0x3007's bits to mirror and flip, the same on every channel; the rest is
 * the XM530V200's. Unverified: there is no capture of it.
 */
extern const struct xmv200_sns_reg xm533_mis2009_mipi[160];

static void xm533_init(const struct xmv200_sns_dev *d,
                       const struct xmv200_sns_mode *m,
                       const struct xmv200_sns_attr *a) {
  if (d->chn < MAX_CHN)
    memset(&st[d->chn], 0, sizeof(st[d->chn]));
  for (int i = 0; i < m->n; i++)
    w(d, m->regs[i].reg, m->regs[i].val);
  usleep(20000);
}

static void xm533_set_flip(const struct xmv200_sns_dev *d, int mirror,
                           int flip) {
  uint32_t v = r(d, 0x3007);

  v = mirror ? v | 1 : v & ~1u;
  v = flip ? v | 2 : v & ~2u;
  w(d, 0x3007, v);
}

static const struct xmv200_sns_mode xm533_modes[] = {
    {"1080p mipi", {0, 0, 0, 0}, xm533_mis2009_mipi, N(xm533_mis2009_mipi)},
};

const struct xmv200_sns_drv xmv200_sns_mis2009_xm533 = {
    .probe = probe,
    .probe_addr = {0x30, 0x31},
    .name = "MIS2009",
    .family = "xm533",
    .modes = xm533_modes,
    .n_modes = N(xm533_modes),
    .init = xm533_init,
    .ae_default = ae_default,
    .set_fps = set_fps,
    .set_vts = set_vts,
    .set_exposure = set_exposure,
    .again_calc = again_calc,
    .dgain_calc = dgain_calc,
    .set_gain = set_gain,
    .set_flip = xm533_set_flip,
};

/* ---- XM535 ----
 *
 * The XM535D1 libsns's (Aug 2025), unverified. Probe as above (I2C
 * address 0x30 unless a product preset gives another; 0x3006 written,
 * 100 ms; chip ID 0x2008). Its MIPI and DVP tables (DVP then 0x3008 =
 * 0xff, without the pause), its frame rate (15 fps on 1875 lines with the
 * longer line on channel 2, 20 fps 1406 lines; the frame length unmasked,
 * no 0x3303) and a flip that does not turn channel 1 round. The rest is
 * the XM530V200's.
 */

extern const struct xmv200_sns_reg xm535_mis2009_mipi[161],
    xm535_mis2009_dvp[161];

static void xm535_init(const struct xmv200_sns_dev *d,
                       const struct xmv200_sns_mode *m,
                       const struct xmv200_sns_attr *a) {
  if (d->chn < MAX_CHN)
    memset(&st[d->chn], 0, sizeof(st[d->chn]));
  for (int i = 0; i < m->n; i++)
    w(d, m->regs[i].reg, m->regs[i].val);
  if (m->regs == xm535_mis2009_dvp)
    w(d, 0x3008, 0xff);
  else
    usleep(20000);
}

static void xm535_set_fps(const struct xmv200_sns_dev *d,
                          const struct xmv200_sns_attr *a, uint32_t fps,
                          struct xmv200_sns_ae *ae) {
  uint32_t hts, vts;

  switch (fps) {
  case 15:
    if (d->chn == 2)
      hts = 0xa80, vts = 0x753;
    else
      hts = 0x8c0, vts = 0x8ca;
    break;
  case 20:
    hts = 0xa80, vts = 0x57e;
    break;
  case 12:
    hts = 0xa80, vts = 0x8ca;
    break;
  default:
    hts = 0x10680 / (fps ? fps : 1), vts = 0x465;
  }
  ae->full_lines = vts;
  ae->max_lines = vts - 2;
  if (d->chn < MAX_CHN)
    st[d->chn].vts = vts;
  clamp_exposure(d, ae);
  w(d, 0x3203, hts & 0xff);
  w(d, 0x3202, (hts & 0xffff) >> 8);
  w(d, 0x3201, vts & 0xff);
  w(d, 0x3200, vts >> 8);
}

static void xm535_set_flip(const struct xmv200_sns_dev *d, int mirror,
                           int flip) {
  uint32_t v = r(d, 0x3007);

  v = mirror ? v & ~1u : v | 1;
  v = flip ? v & ~2u : v | 2;
  w(d, 0x3007, v);
}

static const struct xmv200_sns_mode xm535_modes[] = {
    {"1080p mipi", {0, 0, 0, 0}, xm535_mis2009_mipi, N(xm535_mis2009_mipi)},
    {"1080p dvp", {1, 0, 0, 0}, xm535_mis2009_dvp, N(xm535_mis2009_dvp)},
};

const struct xmv200_sns_drv xmv200_sns_mis2009_xm535 = {
    .probe = probe,
    .probe_addr = {0x30, 0x31},
    .name = "MIS2009",
    .family = "xm535",
    .n_runs = 4,
    .steps = xmv200_sns_steps_ifap,
    .modes = xm535_modes,
    .n_modes = N(xm535_modes),
    .init = xm535_init,
    .ae_default = ae_default,
    .set_fps = xm535_set_fps,
    .set_vts = set_vts,
    .set_exposure = set_exposure,
    .again_calc = again_calc,
    .dgain_calc = dgain_calc,
    .set_gain = set_gain,
    .set_flip = xm535_set_flip,
};
