// SPDX-License-Identifier: MIT
/*
 * GalaxyCore GC2083 (1080p, DVP): the XM535D1 vendor libsns's driver of
 * it, as functions. Unverified: there is no capture of it.
 *
 * Probe (the vendor's): I2C address 0x37 (a product preset may give
 * another), 16-bit registers, 8-bit values; chip ID 0x3f0 << 8 | 0x3f1 =
 * 0x2083. The D1's probe gives a GC3003 (0x3003) this driver too.
 *
 * Exposure 0x0d03/0x0d04 in lines; the frame stays 1125 lines (2 of
 * margin): the frame rate is the line length 66000 / fps in 0x0d05/0x0d06
 * (in 2-pixel units), and the vendor's frame length setter only takes the
 * exposure down with it, it writes no register. The analog gain is one of
 * 25 steps, 12 registers from the vendor's table per step (0x00d0,
 * 0x0155, 0x0410..0x0417, 0x00b8/0x00b9, 0x0dc1 under 0x031d); no fine
 * part and no digital gain. The vendor's gains are in 1/64; sns.h's in
 * 1/32. No mirror/flip (the vendor's does nothing). The tables are the
 * vendor's (sns_gc2083_xm535_regs.c).
 */

#include <string.h>
#include <unistd.h>

#include "sns.h"
#include "xmv200.h"

extern const struct xmv200_sns_reg xm535_gc2083_dvp[135];
extern const uint32_t xm535_gc2083_again[28];
extern const uint32_t xm535_gc2083_gain_regs[392];

#define N(a) ((int)(sizeof(a) / sizeof((a)[0])))
#define MAX_CHN 3

static struct {
  int valid;
  uint32_t exposure, again, dgain;
} st[MAX_CHN];
static uint32_t vts_cache; /* the vendor's: one for every channel */

static void w(const struct xmv200_sns_dev *d, uint16_t reg, uint32_t val) {
  xmv200_i2c_write(d->bus, d->addr, reg, 2, val & 0xff, 1);
}

static void init(const struct xmv200_sns_dev *d,
                 const struct xmv200_sns_mode *m,
                 const struct xmv200_sns_attr *a) {
  if (d->chn < MAX_CHN)
    memset(&st[d->chn], 0, sizeof(st[d->chn]));
  for (int i = 0; i < m->n; i++)
    w(d, m->regs[i].reg, m->regs[i].val);
  usleep(20000);
}

static void ae_default(const struct xmv200_sns_attr *a,
                       struct xmv200_sns_ae *ae) {
  ae->full_lines = 0x465;
  ae->max_lines = 0x463;
  ae->again_min = 0x40 >> 1;
  ae->again_max = 0x1000 >> 1;
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
  w(d, 0x0d03, lines >> 8);
  w(d, 0x0d04, lines & 0xff);
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
  uint32_t hts = 66000 / (fps ? fps : 1);

  ae->full_lines = 0x465;
  ae->max_lines = 0x463;
  clamp_exposure(d, ae);
  w(d, 0x0d05, hts >> 9);
  w(d, 0x0d06, (hts >> 1) & 0xff);
}

static void set_vts(const struct xmv200_sns_dev *d, uint32_t vts,
                    struct xmv200_sns_ae *ae) {
  if (vts_cache == vts)
    return;
  vts_cache = vts;
  ae->full_lines = vts;
  ae->max_lines = vts - 2;
  clamp_exposure(d, ae);
}

/* the step at or below the gain (the table ends 0xffffffff): the step in
 * the low byte, the rest in 1/64 of it above; the gain is the step's */
static void again_calc(uint32_t gain, uint32_t *idx, uint32_t *real) {
  uint32_t g = gain << 1, i = 0;

  while (i != 27 &&
         !(xm535_gc2083_again[i] <= g && g < xm535_gc2083_again[i + 1]))
    i++;
  *idx = i | (xm535_gc2083_again[i] ? (g << 6) / xm535_gc2083_again[i] : 0)
                 << 8;
  *real = xm535_gc2083_again[i] >> 1;
}

static void dgain_calc(uint32_t gain, uint32_t *reg, uint32_t *real) {
  *reg = 0;
  *real = 0x20;
}

static void set_gain(const struct xmv200_sns_dev *d, uint32_t idx,
                     uint32_t dreg) {
  static const uint16_t reg[12] = {0x00d0, 0x0155, 0x0410, 0x0411,
                                   0x0412, 0x0413, 0x0414, 0x0415,
                                   0x0416, 0x0417, 0x00b8, 0x00b9};
  const uint32_t *s = &xm535_gc2083_gain_regs[(idx & 0xff) % 28 * 14];

  if (d->chn < MAX_CHN) {
    if (st[d->chn].valid & 2 && st[d->chn].again == idx &&
        st[d->chn].dgain == dreg)
      return;
    st[d->chn].again = idx;
    st[d->chn].dgain = dreg;
    st[d->chn].valid |= 2;
  }
  for (int i = 0; i < 12; i++)
    w(d, reg[i], s[i]);
  w(d, 0x031d, 0x2e);
  w(d, 0x0dc1, s[12]);
  w(d, 0x031d, 0x28);
}

static void set_flip(const struct xmv200_sns_dev *d, int mirror, int flip) {}

static const struct xmv200_sns_mode modes[] = {
    {"1080p dvp", {1, 0, 0, 0}, xm535_gc2083_dvp, N(xm535_gc2083_dvp)},
};

/* the probe: GalaxyCore: the chip ID at 0x3f0/0x3f1 */
static uint32_t probe(int bus, uint8_t addr) {
  return xmv200_sns_id_pair(bus, addr, 2, 0x3f0, 0x3f1) == 0x2083 ? 0x2083 : 0;
}

const struct xmv200_sns_drv xmv200_sns_gc2083_xm535 = {
    .probe = probe,
    .probe_addr = {0x37},
    .name = "GC2083",
    .family = "xm535",
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
