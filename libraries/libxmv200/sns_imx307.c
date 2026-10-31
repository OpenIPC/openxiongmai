// SPDX-License-Identifier: MIT
/*
 * Sony IMX307: the XM530V200 vendor libsns's driver of it, as functions.
 * Unverified: no capture has this sensor.
 *
 * Probe (the vendor's): I2C address 0x1a, 16-bit registers, 8-bit values;
 * 0x3003 = 1 (reset), 50 ms, then 0x3008 << 8 | 0x301e = 0xa0b2 and
 * 0x31dc & 6 = 4.
 *
 * Modes: 2 lanes at 25 fps, 4 lanes at 12.5 fps (the vendor's lane field
 * 2; sns.h has no lane count, so only a tuning selects it).
 *
 * The vendor hands exposure and gain to its ISP's per-frame register
 * update (0x3020, 0x3021, 0x3014, 0x3009, each written when it changed);
 * here they are written at once, in that order. Exposure is SHS1 = the
 * frame length set by the frame rate less the lines; gain is in 0.3 dB
 * steps (imx307_gain, 1/1024; sns.h's 1/32), above step 20 with the
 * conversion gain high (0x3009 = 0x12) and 21 steps less. No mirror/flip
 * (the vendor's does nothing).
 */

#include <unistd.h>

#include "sns.h"
#include "xmv200.h"

extern const struct xmv200_sns_reg imx307_mipi_4lane[56], imx307_mipi_2lane[66];
extern const uint32_t imx307_gain[141];

#define N(a) ((int)(sizeof(a) / sizeof((a)[0])))
#define MAX_CHN 3

static uint32_t full[MAX_CHN] = {0x465, 0x465, 0x465}; /* what fps set */
static int32_t last[4] = {-1, -1, -1, -1}; /* 0x3020, 0x3021, 0x3014, 0x3009 */
static uint32_t shs = 0xffff, vts_cache;

static void w(const struct xmv200_sns_dev *d, uint16_t reg, uint32_t val) {
  xmv200_i2c_write(d->bus, d->addr, reg, 2, val & 0xff, 1);
}

static void sync_w(const struct xmv200_sns_dev *d, int i, uint16_t reg,
                   uint32_t val) {
  if (last[i] == (int32_t)val)
    return;
  last[i] = val;
  w(d, reg, val);
}

static void init(const struct xmv200_sns_dev *d,
                 const struct xmv200_sns_mode *m,
                 const struct xmv200_sns_attr *a) {
  /* what the vendor's process starts with: nothing written yet */
  for (int i = 0; i < 4; i++)
    last[i] = -1;
  shs = 0xffff;
  vts_cache = 0;
  w(d, 0x3000, 1);
  for (int i = 0; i < m->n; i++)
    w(d, m->regs[i].reg, m->regs[i].val);
  usleep(20000);
  w(d, 0x3002, 0);
  w(d, 0x304b, 10);
  usleep(20000);
}

static void ae_default(const struct xmv200_sns_attr *a,
                       struct xmv200_sns_ae *ae) {
  ae->full_lines = 0x465;
  ae->max_lines = 0x461;
  ae->again_min = 0x400 / 32;
  ae->again_max = 0xf464 / 32;
  ae->dgain_min = 0x400 / 32;
  ae->dgain_max = 0x400 / 32;
}

static void set_exposure(const struct xmv200_sns_dev *d, uint32_t lines) {
  uint32_t s = (full[d->chn < MAX_CHN ? d->chn : 0] - lines) & 0xffff;

  if (s == shs)
    return;
  shs = s;
  sync_w(d, 0, 0x3020, s & 0xff);
  sync_w(d, 1, 0x3021, s >> 8);
}

/* the line length for 20, 30 or 25 fps; the frame length stays 1125 */
static void set_fps(const struct xmv200_sns_dev *d,
                    const struct xmv200_sns_attr *a, uint32_t fps,
                    struct xmv200_sns_ae *ae) {
  uint32_t hmax = fps == 20 ? 0xce4 : fps == 30 ? 0x898 : 0xa50;

  w(d, 0x301c, (hmax & 0x7c) << 1);
  w(d, 0x301d, hmax >> 7);
  ae->full_lines = 0x465;
  ae->max_lines = 0x461;
  if (d->chn < MAX_CHN)
    full[d->chn] = 0x465;
}

/* SHS1 keeps counting from the frame rate's frame length, as the vendor's */
static void set_vts(const struct xmv200_sns_dev *d, uint32_t vts,
                    struct xmv200_sns_ae *ae) {
  if (vts_cache == vts)
    return;
  vts_cache = vts;
  ae->full_lines = vts;
  ae->max_lines = vts - 4;
  w(d, 0x3018, vts & 0xff);
  w(d, 0x3019, vts >> 8);
  w(d, 0x301a, 0);
}

static void step(uint32_t gain, uint32_t *idx, uint32_t *real) {
  uint32_t g = gain * 32, i;

  if (g > 0x1e7a0) {
    i = 0x8c;
  } else {
    for (i = 1; i < 0x8d && g >= imx307_gain[i]; i++)
      ;
    i--;
  }
  *idx = i;
  *real = imx307_gain[i] / 32;
}

static void again_calc(uint32_t gain, uint32_t *idx, uint32_t *real) {
  step(gain, idx, real);
}

static void dgain_calc(uint32_t gain, uint32_t *reg, uint32_t *real) {
  step(gain, reg, real);
}

static void set_gain(const struct xmv200_sns_dev *d, uint32_t idx,
                     uint32_t dreg) {
  uint32_t a = idx > 0x14 ? idx - 0x15 : idx;

  sync_w(d, 2, 0x3014, (a + dreg) & 0xff);
  sync_w(d, 3, 0x3009, idx < 0x15 ? 2 : 0x12);
}

static void set_flip(const struct xmv200_sns_dev *d, int mirror, int flip) {}

static const struct xmv200_sns_mode modes[] = {
    {"1080p mipi 2 lanes", {0, 0, 0, 0, 0, 0, 0, 2}, imx307_mipi_2lane,
     N(imx307_mipi_2lane)},
    {"1080p mipi 4 lanes", {0, 0, 0, 0, 0, 0, 0, 4}, imx307_mipi_4lane,
     N(imx307_mipi_4lane)},
};

/* the vendor's probe: 0x3003 = 1, 50 ms, then 0x3008 << 8 | 0x301e is
 * 0xa0b2 and 0x31dc & 6 is 4 */
static uint32_t probe(int bus, uint8_t addr) {
  uint32_t hi, lo, v;

  if (xmv200_i2c_write(bus, addr, 0x3003, 2, 1, 1) < 0)
    return 0;
  usleep(50000);
  if (xmv200_i2c_read(bus, addr, 0x3008, 2, &hi, 1) ||
      xmv200_i2c_read(bus, addr, 0x301e, 2, &lo, 1) ||
      ((hi & 0xff) << 8 | (lo & 0xff)) != 0xa0b2 ||
      xmv200_i2c_read(bus, addr, 0x31dc, 2, &v, 1) || (v & 6) != 4)
    return 0;
  return 0xa0b2;
}

/* the tuning has init and frame rate: the AE's writes go to the ISP's
 * per-frame update */
const struct xmv200_sns_drv xmv200_sns_imx307 = {
    .n_runs = 2,
    .probe = probe,
    .probe_addr = {0x1a},
    .name = "IMX307",
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
 * The XM535W1 libsns's (Apr 2024), unverified: its tables (standby on
 * first), then standby off, 0x3002 and 0x304b. Its 10-bit variant (8 more
 * pairs where the vendor's lane field's high nibble is 1) is not here.
 * Frame rate, frame length, exposure and gain are the XM530V200's.
 */

extern const struct xmv200_sns_reg xm535w1_imx307_mipi_4lane[57],
    xm535w1_imx307_mipi_2lane[67];

static void w1_init(const struct xmv200_sns_dev *d,
                    const struct xmv200_sns_mode *m,
                    const struct xmv200_sns_attr *a) {
  for (int i = 0; i < m->n; i++)
    w(d, m->regs[i].reg, m->regs[i].val);
  w(d, 0x3000, 0);
  usleep(20000);
  w(d, 0x3002, 0);
  w(d, 0x304b, 10);
  usleep(20000);
}

static const struct xmv200_sns_mode w1_modes[] = {
    {"1080p mipi 2 lanes", {0, 0, 0, 0, 0, 0, 0, 2}, xm535w1_imx307_mipi_2lane,
     N(xm535w1_imx307_mipi_2lane)},
    {"1080p mipi 4 lanes", {0, 0, 0, 0, 0, 0, 0, 4}, xm535w1_imx307_mipi_4lane,
     N(xm535w1_imx307_mipi_4lane)},
};

const struct xmv200_sns_drv xmv200_sns_imx307_xm535w1 = {
    .name = "IMX307",
    .family = "xm535w1",
    .n_runs = 2,
    .probe = probe,
    .probe_addr = {0x1a},
    .modes = w1_modes,
    .n_modes = N(w1_modes),
    .init = w1_init,
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
 * The XM535D1 libsns's (Aug 2025), unverified. Probe as above (I2C
 * address 0x1a unless a product preset gives another). Its tables (standby
 * on first), the 10-bit pairs after them where the vendor's lane field's
 * high nibble is 1 (the "10-bit" modes here), then the XM535W1's standby
 * off, 0x3002 and 0x304b. Frame rate and gain steps are the XM530V200's;
 * the exposure, gain and frame length state is per channel, and a shorter
 * frame takes the exposure down with it.
 */

extern const struct xmv200_sns_reg xm535_imx307_mipi_4lane[57],
    xm535_imx307_mipi_2lane[67], xm535_imx307_10bit[8];

static struct {
  int valid;
  uint32_t lines, vts;
  int32_t last[4]; /* 0x3020, 0x3021, 0x3014, 0x3009 */
} xm535_st[MAX_CHN];

static const struct xmv200_sns_mode xm535_modes[] = {
    {"1080p mipi 2 lanes", {0, 0, 0, 0, 0, 0, 0, 2}, xm535_imx307_mipi_2lane,
     N(xm535_imx307_mipi_2lane)},
    {"1080p mipi 4 lanes", {0, 0, 0, 0, 0, 0, 0, 4}, xm535_imx307_mipi_4lane,
     N(xm535_imx307_mipi_4lane)},
    {"1080p mipi 2 lanes 10-bit", {0, 0, 0, 0, 0, 0, 0, 2},
     xm535_imx307_mipi_2lane,
     N(xm535_imx307_mipi_2lane)},
    {"1080p mipi 4 lanes 10-bit", {0, 0, 0, 0, 0, 0, 0, 4},
     xm535_imx307_mipi_4lane,
     N(xm535_imx307_mipi_4lane)},
};

static void xm535_w(const struct xmv200_sns_dev *d, int i, uint16_t reg,
                    uint32_t val) {
  if (d->chn < MAX_CHN) {
    if (xm535_st[d->chn].last[i] == (int32_t)val)
      return;
    xm535_st[d->chn].last[i] = val;
  }
  w(d, reg, val);
}

static void xm535_init(const struct xmv200_sns_dev *d,
                       const struct xmv200_sns_mode *m,
                       const struct xmv200_sns_attr *a) {
  if (d->chn < MAX_CHN) {
    xm535_st[d->chn].valid = 0;
    for (int i = 0; i < 4; i++)
      xm535_st[d->chn].last[i] = -1;
  }
  for (int i = 0; i < m->n; i++)
    w(d, m->regs[i].reg, m->regs[i].val);
  if (m >= &xm535_modes[2])
    for (int i = 0; i < N(xm535_imx307_10bit); i++)
      w(d, xm535_imx307_10bit[i].reg, xm535_imx307_10bit[i].val);
  w(d, 0x3000, 0);
  usleep(20000);
  w(d, 0x3002, 0);
  w(d, 0x304b, 10);
  usleep(20000);
}

static void xm535_set_exposure(const struct xmv200_sns_dev *d,
                               uint32_t lines) {
  uint32_t s;

  if (d->chn < MAX_CHN) {
    if (xm535_st[d->chn].valid & 1 && xm535_st[d->chn].lines == lines)
      return;
    xm535_st[d->chn].lines = lines;
    xm535_st[d->chn].valid |= 1;
  }
  s = (full[d->chn < MAX_CHN ? d->chn : 0] - lines) & 0xffff;
  xm535_w(d, 0, 0x3020, s & 0xff);
  xm535_w(d, 1, 0x3021, s >> 8);
}

static void xm535_set_vts(const struct xmv200_sns_dev *d, uint32_t vts,
                          struct xmv200_sns_ae *ae) {
  if (d->chn < MAX_CHN) {
    if (xm535_st[d->chn].valid & 2 && xm535_st[d->chn].vts == vts)
      return;
    xm535_st[d->chn].vts = vts;
    xm535_st[d->chn].valid |= 2;
  }
  ae->full_lines = vts;
  ae->max_lines = vts - 4;
  if (d->chn < MAX_CHN && xm535_st[d->chn].valid & 1 &&
      ae->max_lines < xm535_st[d->chn].lines)
    xm535_set_exposure(d, ae->max_lines);
  w(d, 0x3018, vts & 0xff);
  w(d, 0x3019, vts >> 8);
  w(d, 0x301a, 0);
}

static void xm535_set_gain(const struct xmv200_sns_dev *d, uint32_t idx,
                           uint32_t dreg) {
  uint32_t a = idx > 0x14 ? idx - 0x15 : idx;

  xm535_w(d, 2, 0x3014, (a + dreg) & 0xff);
  xm535_w(d, 3, 0x3009, idx < 0x15 ? 2 : 0x12);
}

const struct xmv200_sns_drv xmv200_sns_imx307_xm535 = {
    .name = "IMX307",
    .family = "xm535",
    .n_runs = 2,
    .probe = probe,
    .probe_addr = {0x1a},
    .modes = xm535_modes,
    .n_modes = N(xm535_modes),
    .init = xm535_init,
    .ae_default = ae_default,
    .set_fps = set_fps,
    .set_vts = xm535_set_vts,
    .set_exposure = xm535_set_exposure,
    .again_calc = again_calc,
    .dgain_calc = dgain_calc,
    .set_gain = xm535_set_gain,
    .set_flip = set_flip,
};
