// SPDX-License-Identifier: MIT
/*
 * Sony IMX335: the XM530V200 vendor libsns's driver of it, as functions.
 * Unverified: no capture has this sensor.
 *
 * Probe (the vendor's, for a product configured for it): I2C address 0x1a,
 * 16-bit registers, 8-bit values; 0x3302 = 0x32.
 *
 * One mode: 5M, 4 lanes, 15 fps. The vendor writes the frame length
 * (VMAX, 0x3030..0x3032) and SHR0 (0x3058..0x305a) as four times the lines;
 * the line length (HMAX) is 0x3034/0x3035. Its frame length for slow
 * shutter is a longer HMAX, not VMAX. Gain 0x30e8/0x30e9 in 0.3 dB steps
 * (imx335_gain, 1/1024; sns.h's 1/32). Mirror 0x304e; flip 0x304f with
 * the vendor's window registers.
 */

#include <unistd.h>

#include "sns.h"
#include "xmv200.h"

extern const struct xmv200_sns_reg imx335_mipi_4lane[160];
extern const uint32_t imx335_gain[141];

#define N(a) ((int)(sizeof(a) / sizeof((a)[0])))
#define MAX_CHN 3

static uint32_t full[MAX_CHN];  /* the frame length fps set */
static int32_t shr = -1, gain = -1;
static uint32_t hmax_cache;

static void w(const struct xmv200_sns_dev *d, uint16_t reg, uint32_t val) {
  xmv200_i2c_write(d->bus, d->addr, reg, 2, val & 0xff, 1);
}

static void init(const struct xmv200_sns_dev *d,
                 const struct xmv200_sns_mode *m,
                 const struct xmv200_sns_attr *a) {
  w(d, 0x3000, 1);
  w(d, 0x3004, 4);
  usleep(50000);
  for (int i = 0; i < m->n; i++)
    w(d, m->regs[i].reg, m->regs[i].val);
  usleep(10000);
  w(d, 0x3000, 0);
  usleep(50000);
  w(d, 0x3002, 0);
  usleep(50000);
}

static void ae_default(const struct xmv200_sns_attr *a,
                       struct xmv200_sns_ae *ae) {
  ae->full_lines = 0x9ab;
  ae->max_lines = 0x9a2;
  ae->again_min = 0x400 / 32;
  ae->again_max = 0x3da6 / 32;
  ae->dgain_min = 0x400 / 32;
  ae->dgain_max = 0x400 / 32;
}

/*
 * As the vendor's: what it compares with the new lines is the last SHR in
 * lines (frame length less lines), so a write is skipped when the two
 * happen to be equal.
 */
static void set_exposure(const struct xmv200_sns_dev *d, uint32_t lines) {
  uint32_t v;

  if (shr == (int32_t)lines)
    return;
  shr = full[d->chn < MAX_CHN ? d->chn : 0] - lines;
  v = shr * 4;
  w(d, 0x3058, v & 0xfc);
  w(d, 0x3059, (v & 0xffff) >> 8);
  w(d, 0x305a, (v & 0xfffff) >> 16);
}

/* 20 fps: 1856 lines; 25: 1500 (and a shorter line); else 15: 2475 */
static void set_fps(const struct xmv200_sns_dev *d,
                    const struct xmv200_sns_attr *a, uint32_t fps,
                    struct xmv200_sns_ae *ae) {
  uint32_t hmax = 4000, vts = 0x9ab;

  if (fps == 20) {
    vts = 0x740;
  } else if (fps == 25) {
    hmax = 0xf78;
    vts = 0x5dc;
  }
  ae->full_lines = vts;
  ae->max_lines = vts - 9;
  if (d->chn < MAX_CHN)
    full[d->chn] = vts;
  w(d, 0x3030, (vts << 2) & 0xfc);
  w(d, 0x3031, (vts << 2) >> 8);
  w(d, 0x3032, 0);
  w(d, 0x3034, (hmax & 0x7ff) >> 3);
  w(d, 0x3035, 1);
}

/* a frame vts lines long by the line length: 4000 * vts / the fps's */
static void set_vts(const struct xmv200_sns_dev *d, uint32_t vts,
                    struct xmv200_sns_ae *ae) {
  uint32_t f = full[d->chn < MAX_CHN ? d->chn : 0], ratio, h;

  if (!f)
    return;
  ratio = (vts << 4) / f;
  h = ratio * 4000;
  if (hmax_cache == h >> 4)
    return;
  hmax_cache = h >> 4;
  usleep(30000);
  w(d, 0x3034, (h & 0x7fff) >> 7);
  w(d, 0x3035, (h & 0x7fffff) >> 15);
}

static void again_calc(uint32_t g32, uint32_t *idx, uint32_t *real) {
  uint32_t g = g32 * 32, i;

  if (g > 0x1e7a0) {
    i = 0x8c;
  } else {
    for (i = 1; i < 0x8d && g >= imx335_gain[i]; i++)
      ;
    i--;
  }
  *idx = i;
  *real = imx335_gain[i] / 32;
}

/* none: the ISP's */
static void dgain_calc(uint32_t g, uint32_t *reg, uint32_t *real) {
  *reg = 0;
  *real = g;
}

static void set_gain(const struct xmv200_sns_dev *d, uint32_t idx,
                     uint32_t dreg) {
  uint32_t v = idx + dreg;

  if (gain == (int32_t)idx)
    return;
  gain = idx;
  w(d, 0x30e8, v & 0xff);
  w(d, 0x30e9, (v & 0x7ff) >> 8);
}

static void set_flip(const struct xmv200_sns_dev *d, int mirror, int flip) {
  w(d, 0x304e, mirror != 0);
  w(d, 0x304f, flip ? 1 : 0);
  w(d, 0x3081, flip ? 0xfe : 2);
  w(d, 0x3083, flip ? 0xfe : 2);
  w(d, 0x30b6, flip ? 0xfa : 0);
  w(d, 0x30b7, flip ? 1 : 0);
  w(d, 0x3116, flip ? 2 : 8);
}

/* the vendor's probe: 0x3302 reads 0x32; the registry's ID for it */
static uint32_t probe(int bus, uint8_t addr) {
  uint32_t v;

  if (xmv200_i2c_read(bus, addr, 0x3302, 2, &v, 1) || (v & 0xff) != 0x32)
    return 0;
  return 0x0335;
}

/* The tuning's runs: init, frame rate, the AE's start, and the flip the
 * vendor's ISP sets after them (off) */
static void run(const struct xmv200_sns_dev *d, int n,
                const struct xmv200_sns_ctx *c);

static const struct xmv200_sns_mode modes[] = {
    {"5M mipi 4 lanes", {0, 0, 0, 0}, imx335_mipi_4lane, N(imx335_mipi_4lane)},
};

const struct xmv200_sns_drv xmv200_sns_imx335 = {
    .name = "IMX335",
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
    .n_runs = 4,
    .run = run,
    .probe = probe,
    .probe_addr = {0x1a},
};

static void run(const struct xmv200_sns_dev *d, int n,
                const struct xmv200_sns_ctx *c) {
  switch (n) {
  case 0:
    xmv200_sns_init_step(&xmv200_sns_imx335, d, c);
    break;
  case 1:
    xmv200_sns_fps_step(&xmv200_sns_imx335, d, c);
    break;
  case 2:
    xmv200_sns_ae_step(&xmv200_sns_imx335, d, c);
    break;
  case 3:
    set_flip(d, 0, 0);
    break;
  }
}
