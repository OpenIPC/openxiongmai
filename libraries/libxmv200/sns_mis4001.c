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

/* the probe: the vendor's MIS probe (0x1311: MIS4001 unless the product
 * says MIS5001) */
static uint32_t probe(int bus, uint8_t addr) {
  return xmv200_sns_mis_id(bus, addr) == 0x1311 ? 0x1311 : 0;
}

const struct xmv200_sns_drv xmv200_sns_mis4001 = {
    .probe = probe,
    .probe_addr = {0x30, 0x31},
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

/* ---- XM533 ---- */

/*
 * The XM533 libsns's: one 4M table, the frame length from a table for 12
 * .. 30 fps (1500 lines otherwise), no 0x4240 around it, and no channel 1
 * flip. Exposure and gain are the XM530V200's. Unverified: there is no
 * capture of it.
 */
extern const struct xmv200_sns_reg xm533_mis4001_4m[200];

static const uint16_t xm533_vts[19] = {
    0xbb8, 0x5dc, 0x5dc, 0x9c4, 0x5dc, 0x5dc, 0x5dc, 0x5dc, 0x753, 0x5dc,
    0x5dc, 0x5dc, 0x5dc, 0x5dc, 0x5dc, 0x5dc, 0x5dc, 0x5dc, 0x4e2};

static void xm533_init(const struct xmv200_sns_dev *d,
                       const struct xmv200_sns_mode *m,
                       const struct xmv200_sns_attr *a) {
  if (d->chn < MAX_CHN)
    memset(&st[d->chn], 0, sizeof(st[d->chn]));
  for (int i = 0; i < m->n; i++)
    w(d, m->regs[i].reg, m->regs[i].val);
  usleep(20000);
}

static void xm533_ae_default(const struct xmv200_sns_attr *a,
                             struct xmv200_sns_ae *ae) {
  ae->full_lines = 0x5dc;
  ae->max_lines = 0x5d6;
  ae->again_min = 0x400 >> 5;
  ae->again_max = 0x3e00 >> 5;
  ae->dgain_min = 0x20;
  ae->dgain_max = 0x20;
}

static void xm533_set_fps(const struct xmv200_sns_dev *d,
                          const struct xmv200_sns_attr *a, uint32_t fps,
                          struct xmv200_sns_ae *ae) {
  uint8_t i = fps - 12;
  uint32_t vts = i < 19 ? xm533_vts[i] : 0x5dc;

  ae->full_lines = vts;
  ae->max_lines = vts - 6;
  if (d->chn < MAX_CHN)
    st[d->chn].vts = vts;
  frame_length(d, vts);
}

static void xm533_set_vts(const struct xmv200_sns_dev *d, uint32_t vts,
                          struct xmv200_sns_ae *ae) {
  if (d->chn < MAX_CHN) {
    if (st[d->chn].vts == vts)
      return;
    st[d->chn].vts = vts;
  }
  ae->full_lines = vts;
  ae->max_lines = vts - 6;
  clamp_exposure(d, ae);
  frame_length(d, vts);
}

static void xm533_set_flip(const struct xmv200_sns_dev *d, int mirror,
                           int flip) {
  uint32_t v = r(d, 0x3007);

  v = mirror ? v & ~1u : v | 1;
  v = flip ? v & ~2u : v | 2;
  w(d, 0x3007, v);
}

static const struct xmv200_sns_mode xm533_modes[] = {
    {"4M mipi", {0, 0, 0, 0}, xm533_mis4001_4m, N(xm533_mis4001_4m)},
};

const struct xmv200_sns_drv xmv200_sns_mis4001_xm533 = {
    .probe = probe,
    .probe_addr = {0x30, 0x31},
    .name = "MIS4001",
    .family = "xm533",
    .modes = xm533_modes,
    .n_modes = N(xm533_modes),
    .init = xm533_init,
    .ae_default = xm533_ae_default,
    .set_fps = xm533_set_fps,
    .set_vts = xm533_set_vts,
    .set_exposure = set_exposure,
    .again_calc = again_calc,
    .dgain_calc = dgain_calc,
    .set_gain = set_gain,
    .set_flip = xm533_set_flip,
};

/* ---- XM535 ----
 *
 * The XM535D1 libsns's (Aug 2025), unverified. Its probe: I2C address 0x30
 * (a product preset may give another), 16-bit registers, 8-bit values;
 * 0x3006 written, 100 ms, chip ID 0x3000 << 8 | 0x3001 (0x3004/0x3005 when
 * that reads neither 0x2008 nor 0x1311) = 0x1311. One 4M table; a digital
 * gain in 0x3a00/0x3a01 (the vendor's 1/128, 1x .. 8x); frame lengths for
 * 12 (2800 lines), 15 (2333) and otherwise 20 fps (1875), or 25 fps (1400)
 * on a frame-synced board (cmos_set_sns_attr's +0x20 with 0x11, the "frame
 * sync" mode here), where the frame length setter does nothing; a 2-line
 * margin; and the flip in a 0x3006 = 2 hold without turning channel 1
 * round. Exposure and the analog gain steps are the XM530V200's.
 */

extern const struct xmv200_sns_reg xm535_mis4001_4m[175];

static const struct xmv200_sns_mode xm535_modes[] = {
    {"4M mipi", {0, 0, 0, 0}, xm535_mis4001_4m, N(xm535_mis4001_4m)},
    {"4M mipi, frame sync", {0, 0, 0, 0, 0, 0, 1}, xm535_mis4001_4m,
     N(xm535_mis4001_4m)},
};

static int xm535_sync[MAX_CHN];
static uint32_t xm535_vts_cache; /* the vendor's: one for every channel */

static int synced(const struct xmv200_sns_dev *d) {
  return d->chn < MAX_CHN && xm535_sync[d->chn];
}

static void xm535_init(const struct xmv200_sns_dev *d,
                       const struct xmv200_sns_mode *m,
                       const struct xmv200_sns_attr *a) {
  if (d->chn < MAX_CHN) {
    memset(&st[d->chn], 0, sizeof(st[d->chn]));
    xm535_sync[d->chn] = m == &xm535_modes[1];
  }
  for (int i = 0; i < m->n; i++)
    w(d, m->regs[i].reg, m->regs[i].val);
  usleep(20000);
}

static void xm535_ae_default(const struct xmv200_sns_attr *a,
                             struct xmv200_sns_ae *ae) {
  ae->full_lines = 0x753;
  ae->max_lines = 0x751;
  ae->again_min = 0x400 >> 5;
  ae->again_max = 0x3e00 >> 5;
  ae->dgain_min = 0x80 >> 2;
  ae->dgain_max = 0x400 >> 2;
}

static void xm535_frame_length(const struct xmv200_sns_dev *d, uint32_t vts) {
  w(d, 0x3006, 2);
  usleep(100000);
  w(d, 0x310c, vts >> 8);
  w(d, 0x310d, vts & 0xff);
  w(d, 0x3006, 0);
}

/* the vendor's leaves the exposure as it is */
static void xm535_set_fps(const struct xmv200_sns_dev *d,
                          const struct xmv200_sns_attr *a, uint32_t fps,
                          struct xmv200_sns_ae *ae) {
  uint32_t vts = fps == 12   ? 0xaf0
                 : fps == 15 ? 0x91d
                 : synced(d) ? 0x578
                             : 0x753;

  ae->full_lines = vts;
  ae->max_lines = vts - 2;
  xm535_frame_length(d, vts);
}

static void xm535_set_vts(const struct xmv200_sns_dev *d, uint32_t vts,
                          struct xmv200_sns_ae *ae) {
  if (synced(d) || xm535_vts_cache == vts)
    return;
  xm535_vts_cache = vts;
  ae->full_lines = vts;
  ae->max_lines = vts - 2;
  clamp_exposure(d, ae);
  xm535_frame_length(d, vts);
}

/* sns.h's 1/32 to the vendor's 1/128 */
static void xm535_dgain_calc(uint32_t gain, uint32_t *reg, uint32_t *real) {
  *reg = gain << 2;
  *real = gain;
}

static void xm535_set_gain(const struct xmv200_sns_dev *d, uint32_t idx,
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
  w(d, 0x3a00, (dreg & 0x7ff) >> 8);
  w(d, 0x3a01, dreg & 0xff);
}

static void xm535_set_flip(const struct xmv200_sns_dev *d, int mirror,
                           int flip) {
  uint32_t v = r(d, 0x3007);

  v = mirror ? v & ~1u : v | 1;
  v = flip ? v & ~2u : v | 2;
  w(d, 0x3006, 2);
  usleep(100000);
  w(d, 0x3007, v);
  w(d, 0x3006, 0);
}

const struct xmv200_sns_drv xmv200_sns_mis4001_xm535 = {
    .probe = probe,
    .probe_addr = {0x30, 0x31},
    .name = "MIS4001",
    .family = "xm535",
    .modes = xm535_modes,
    .n_modes = N(xm535_modes),
    .init = xm535_init,
    .ae_default = xm535_ae_default,
    .set_fps = xm535_set_fps,
    .set_vts = xm535_set_vts,
    .set_exposure = set_exposure,
    .again_calc = again_calc,
    .dgain_calc = xm535_dgain_calc,
    .set_gain = xm535_set_gain,
    .set_flip = xm535_set_flip,
};
