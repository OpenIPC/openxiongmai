// SPDX-License-Identifier: MIT
/*
 * SmartSens SC500AI: the XM530V200 vendor libsns's driver of it, as
 * functions. 5M MIPI on two lanes or four (the vendor's tables,
 * sns_sc500ai_regs.c; it takes four where the board's lane setting is 2).
 *
 * Exposure is 0x3e00..0x3e02 in half lines, the analog gain 0x3e08/0x3e09
 * (a fine part, doublings, and above 1.5x the high-gain bit 0x20 in
 * 0x3e08), the digital gain 0x3e06/0x3e07, line length 0x320c/0x320d,
 * frame length 0x320e/0x320f, mirror and flip 0x3221 (0x06, 0x60). With
 * each gain the vendor sets 0x5799 by the total gain and, by the sensor's
 * temperature (0x3974), 0x3637 and 0x391f under group hold (0x3812). The
 * vendor's gains are in 1/1024 (analog) and 1/128 (digital).
 *
 * As the vendor's, the exposure and the frame length remember one value
 * for all channels.
 */

#include <string.h>
#include <unistd.h>

#include "sns.h"
#include "xmv200.h"

extern const struct xmv200_sns_reg sc500ai_5m_2lane[126], sc500ai_5m_4lane[125];

#define N(a) ((int)(sizeof(a) / sizeof((a)[0])))
#define MAX_CHN 3

static struct {
  int valid;
  uint32_t again, dgain;
} st[MAX_CHN];
static int exp_valid, vts_valid;
static uint32_t exp_last, vts_last;

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
  exp_valid = vts_valid = 0;
  for (int i = 0; i < m->n; i++)
    w(d, m->regs[i].reg, m->regs[i].val);
  usleep(20000);
}

static void ae_default(const struct xmv200_sns_attr *a,
                       struct xmv200_sns_ae *ae) {
  ae->full_lines = 0x672;
  ae->max_lines = 0x66e;
  ae->again_min = 0x20;
  ae->again_max = 0x6044 >> 5;
  ae->dgain_min = 0x20;
  ae->dgain_max = 0xfe0 >> 2;
}

static void set_exposure(const struct xmv200_sns_dev *d, uint32_t lines) {
  if (exp_valid && exp_last == lines)
    return;
  exp_last = lines;
  exp_valid = 1;
  w(d, 0x3e02, (lines & 7) << 5);
  w(d, 0x3e01, (lines << 1 & 0xfff) >> 4);
  w(d, 0x3e00, (lines << 1 & 0xffff) >> 12);
}

/* 24/25 and 29/30 fps by line length and frame length, 20 otherwise */
static void set_fps(const struct xmv200_sns_dev *d,
                    const struct xmv200_sns_attr *a, uint32_t fps,
                    struct xmv200_sns_ae *ae) {
  uint32_t hts, vts;

  switch (fps) {
  case 24:
    hts = 0x12c0, vts = 0x678;
    break;
  case 25:
    hts = 0x12c0, vts = 0x672;
    break;
  case 29:
    hts = 4000, vts = 0x678;
    break;
  case 30:
    hts = 4000, vts = 0x672;
    break;
  default:
    hts = 0x12c0, vts = 0x80e;
  }
  ae->full_lines = vts;
  ae->max_lines = vts - 4;
  w(d, 0x320c, hts >> 9);
  w(d, 0x320d, hts >> 1 & 0xf0);
  w(d, 0x320e, vts >> 8);
  w(d, 0x320f, vts & 0x7e);
}

static void set_vts(const struct xmv200_sns_dev *d, uint32_t vts,
                    struct xmv200_sns_ae *ae) {
  if (vts_valid && vts_last == vts)
    return;
  vts_last = vts;
  vts_valid = 1;
  ae->full_lines = vts;
  ae->max_lines = vts - 4;
  if (exp_valid && ae->max_lines < exp_last)
    set_exposure(d, ae->max_lines);
  w(d, 0x320e, vts >> 8);
  w(d, 0x320f, vts & 0xff);
}

/* gain in 1/32; the vendor's in 1/1024. From 0x610 (1.52x) on, the sensor's
 * high gain, and the rest of it after that factor. The value: bit 31 the
 * high gain, bits 8.. the doublings as ones, bits 7..0 the fine part
 * (1/64). */
static void again_calc(uint32_t gain, uint32_t *idx, uint32_t *real) {
  uint32_t g = gain << 5, f, bits = 0, k = 0, v;
  int high = g >= 0x610;

  f = high ? (g << 6) / 0x610 : g >> 4;
  if (f < 0x40)
    f = 0x40;
  while (k < 4 && f >= 0x80) {
    k++;
    f >>= 1;
    bits = bits << 1 | 1;
  }
  *idx = bits << 8 | (uint32_t)high << 31 | f;
  v = f << (k + 4);
  if (high)
    v = (uint32_t)((uint64_t)v * 97 >> 6);
  *real = v >> 5;
}

/* gain in 1/32; the vendor's in 1/128: the doublings in the high byte */
static void dgain_calc(uint32_t gain, uint32_t *reg, uint32_t *real) {
  uint32_t g = gain << 2, bits = 0, k = 0;

  if (g < 0x80)
    g = 0x80;
  while (k < 5 && g >= 0x100) {
    k++;
    g >>= 1;
    bits = bits << 1 | 1;
  }
  *reg = (g & ~1u) + bits * 0x100;
  *real = ((g & ~1u) << k) >> 2;
}

/* what again_calc and dgain_calc made of the values, in 1/1024 and 1/128 */
static uint32_t again_of(uint32_t v) {
  uint32_t k = 0, r;

  while (v >> (8 + k) & 1)
    k++;
  r = (v & 0xff) << (k + 4);
  return v >> 31 ? (uint32_t)((uint64_t)r * 97 >> 6) : r;
}

static uint32_t dgain_of(uint32_t v) {
  uint32_t k = 0;

  while (v >> (8 + k) & 1)
    k++;
  return (v & 0xfe) << k;
}

static void set_gain(const struct xmv200_sns_dev *d, uint32_t again,
                     uint32_t dgain) {
  uint32_t total = (uint32_t)((uint64_t)again_of(again) * dgain_of(dgain) >> 7);
  uint32_t t = r(d, 0x3974), v;

  if (total >= 0x7800)
    w(d, 0x5799, 7);
  else if (total < 0x50f1)
    w(d, 0x5799, 0);
  if (t >= 0x17 || t <= 0x12) {
    w(d, 0x3812, 0x00);
    w(d, 0x3637, t >= 0x17 ? 0x50 : 0x28);
    w(d, 0x391f, t >= 0x17 ? 0x10 : 0x18);
    w(d, 0x3812, 0x30);
  }
  if (d->chn < MAX_CHN) {
    if (st[d->chn].valid && st[d->chn].again == again &&
        st[d->chn].dgain == dgain)
      return;
    st[d->chn].again = again;
    st[d->chn].dgain = dgain;
    st[d->chn].valid = 1;
  }
  v = ((again & 0xffff) >> 8) << 2 | (again >> 31 ? 0x23 : 0x03);
  w(d, 0x3e08, v);
  w(d, 0x3e09, again & 0xff);
  w(d, 0x3e06, (dgain & 0xffff) >> 8);
  w(d, 0x3e07, dgain & 0xff);
}

static void set_flip(const struct xmv200_sns_dev *d, int mirror, int flip) {
  uint32_t v = r(d, 0x3221);

  v = mirror ? v | 0x06 : v & ~0x06u;
  v = flip ? v | 0x60 : v & ~0x60u;
  w(d, 0x3221, v);
}

static void run(const struct xmv200_sns_dev *d, int n,
                const struct xmv200_sns_ctx *c);

/* the lane count is not in xmv200_sns_attr: the 2-lane mode first */
static const struct xmv200_sns_mode modes[] = {
    {"5m mipi 2 lanes", {0, 0, 0, 0}, sc500ai_5m_2lane, N(sc500ai_5m_2lane)},
    {"5m mipi 4 lanes", {0, 0, 0, 0}, sc500ai_5m_4lane, N(sc500ai_5m_4lane)},
};

const struct xmv200_sns_drv xmv200_sns_sc500ai = {
    .name = "SC500AI",
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

/*
 * The tuning's exposure is the register's count, in half lines. The
 * vendor's ISP starts this sensor at 25 fps, where it starts the SC2336 at
 * the board's 12 (its AE library chooses; not looked into).
 */
static void run(const struct xmv200_sns_dev *d, int n,
                const struct xmv200_sns_ctx *c) {
  struct xmv200_sns_ctx x = *c;

  if (n == 0) {
    xmv200_sns_init_step(&xmv200_sns_sc500ai, d, c);
  } else if (n == 1) {
    x.attr.fps = 25;
    xmv200_sns_fps_step(&xmv200_sns_sc500ai, d, &x);
  } else if (n == 2) {
    x.exp_lines = c->exp_lines / 2;
    xmv200_sns_ae_step(&xmv200_sns_sc500ai, d, &x);
  }
}

/* ---- XM535W1 ----
 *
 * The XM535W1 libsns's (Apr 2024): its own tables; the gain without the
 * temperature and 0x5799 writes, remembered per channel; the high gain's
 * part found by the vendor's multiply. Frame rate, exposure and the
 * digital gain are the XM530V200's.
 */

extern const struct xmv200_sns_reg xm535w1_sc500ai_5m_2lane[126],
    xm535w1_sc500ai_5m_4lane[125];

static void w1_again_calc(uint32_t gain, uint32_t *idx, uint32_t *real) {
  uint32_t g = gain << 5, f, bits = 0, k = 0, v;
  int high = g > 0x60f;

  f = high ? (uint32_t)((uint64_t)((g & 0x3ffffff) << 2) * 0x54741fb >> 33)
           : g >> 4;
  if (f < 0x40)
    f = 0x40;
  while (k < 4 && f >= 0x80) {
    k++;
    f >>= 1;
    bits = bits << 1 | 1;
  }
  *idx = bits << 8 | (uint32_t)high << 31 | f;
  v = f << (k + 4);
  if (high)
    v = (uint32_t)((uint64_t)v * 0x610 >> 10);
  *real = v >> 5;
}

static void w1_set_gain(const struct xmv200_sns_dev *d, uint32_t again,
                        uint32_t dgain) {
  uint32_t v;

  if (d->chn < MAX_CHN) {
    if (st[d->chn].valid && st[d->chn].again == again &&
        st[d->chn].dgain == dgain)
      return;
    st[d->chn].again = again;
    st[d->chn].dgain = dgain;
    st[d->chn].valid = 1;
  }
  v = ((again & 0xffff) >> 8) << 2 | (again >> 31 ? 0x23 : 0x03);
  w(d, 0x3e08, v);
  w(d, 0x3e09, again & 0xff);
  w(d, 0x3e06, (dgain & 0xffff) >> 8);
  w(d, 0x3e07, dgain & 0xff);
}

static void w1_run(const struct xmv200_sns_dev *d, int n,
                   const struct xmv200_sns_ctx *c);

static const struct xmv200_sns_mode w1_modes[] = {
    {"5m mipi 2 lanes", {0, 0, 0, 0}, xm535w1_sc500ai_5m_2lane,
     N(xm535w1_sc500ai_5m_2lane)},
    {"5m mipi 4 lanes", {0, 0, 0, 0}, xm535w1_sc500ai_5m_4lane,
     N(xm535w1_sc500ai_5m_4lane)},
};

const struct xmv200_sns_drv xmv200_sns_sc500ai_xm535w1 = {
    .name = "SC500AI",
    .family = "xm535w1",
    .exp_shift = 1,
    .modes = w1_modes,
    .n_modes = N(w1_modes),
    .init = init,
    .ae_default = ae_default,
    .set_fps = set_fps,
    .set_vts = set_vts,
    .set_exposure = set_exposure,
    .again_calc = w1_again_calc,
    .dgain_calc = dgain_calc,
    .set_gain = w1_set_gain,
    .set_flip = set_flip,
    .run = w1_run,
};

/* the AE starts from half the tuning's exposure (half lines) */
static void w1_run(const struct xmv200_sns_dev *d, int n,
                   const struct xmv200_sns_ctx *c) {
  struct xmv200_sns_ctx x = *c;

  if (n == 2)
    x.exp_lines = c->exp_lines / 2;
  if (n == 0)
    xmv200_sns_init_step(&xmv200_sns_sc500ai_xm535w1, d, &x);
  else if (n == 1)
    xmv200_sns_fps_step(&xmv200_sns_sc500ai_xm535w1, d, &x);
  else if (n == 2)
    xmv200_sns_ae_step(&xmv200_sns_sc500ai_xm535w1, d, &x);
}

/* ---- XM535 ---- */

/*
 * The XM535D1's libsns (Aug 12 2025 d2db8e4), unchecked (no tuning): its
 * tables (4 lanes where the vendor's lane field is 2), and its gain
 * without the XM530V200's 0x5799 and temperature settings. Exposure,
 * frame rate, frame length, the gain's steps and flip are the XM530V200's.
 */
extern const struct xmv200_sns_reg xm535_sc500ai_5m_2lane[126],
    xm535_sc500ai_5m_4lane[125];

static const struct xmv200_sns_mode xm535_modes[] = {
    {"5m mipi 2 lanes", {0, 0, 0, 0}, xm535_sc500ai_5m_2lane,
     N(xm535_sc500ai_5m_2lane)},
    {"5m mipi 4 lanes", {0, 0, 0, 0}, xm535_sc500ai_5m_4lane,
     N(xm535_sc500ai_5m_4lane)},
};

static void xm535_set_gain(const struct xmv200_sns_dev *d, uint32_t again,
                           uint32_t dgain) {
  if (d->chn < MAX_CHN) {
    if (st[d->chn].valid && st[d->chn].again == again &&
        st[d->chn].dgain == dgain)
      return;
    st[d->chn].again = again;
    st[d->chn].dgain = dgain;
    st[d->chn].valid = 1;
  }
  w(d, 0x3e08, ((again & 0xffff) >> 8) << 2 | (again >> 31 ? 0x23 : 0x03));
  w(d, 0x3e09, again & 0xff);
  w(d, 0x3e06, (dgain & 0xffff) >> 8);
  w(d, 0x3e07, dgain & 0xff);
}

const struct xmv200_sns_drv xmv200_sns_sc500ai_xm535 = {
    .name = "SC500AI",
    .family = "xm535",
    .exp_shift = 1,
    .modes = xm535_modes,
    .n_modes = N(xm535_modes),
    .init = init,
    .ae_default = ae_default,
    .set_fps = set_fps,
    .set_vts = set_vts,
    .set_exposure = set_exposure,
    .again_calc = again_calc,
    .dgain_calc = dgain_calc,
    .set_gain = xm535_set_gain,
    .set_flip = set_flip,
};

/* ---- XM533 ---- */

/* The XM533 libsns's: the same tables and functions, but a gain write
 * without the 0x5799 and temperature settings. */
static void xm533_set_gain(const struct xmv200_sns_dev *d, uint32_t again,
                           uint32_t dgain) {
  if (d->chn < MAX_CHN) {
    if (st[d->chn].valid && st[d->chn].again == again &&
        st[d->chn].dgain == dgain)
      return;
    st[d->chn].again = again;
    st[d->chn].dgain = dgain;
    st[d->chn].valid = 1;
  }
  w(d, 0x3e08, ((again & 0xffff) >> 8) << 2 | (again >> 31 ? 0x23 : 0x03));
  w(d, 0x3e09, again & 0xff);
  w(d, 0x3e06, (dgain & 0xffff) >> 8);
  w(d, 0x3e07, dgain & 0xff);
}

static void xm533_run(const struct xmv200_sns_dev *d, int n,
                      const struct xmv200_sns_ctx *c);

const struct xmv200_sns_drv xmv200_sns_sc500ai_xm533 = {
    .name = "SC500AI",
    .family = "xm533",
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
    .set_gain = xm533_set_gain,
    .set_flip = set_flip,
    .run = xm533_run,
};

/* the tuning's exposure is the register's count, in half lines */
static void xm533_run(const struct xmv200_sns_dev *d, int n,
                      const struct xmv200_sns_ctx *c) {
  struct xmv200_sns_ctx x = *c;

  if (n == 0) {
    xmv200_sns_init_step(&xmv200_sns_sc500ai_xm533, d, c);
  } else if (n == 1) {
    xmv200_sns_fps_step(&xmv200_sns_sc500ai_xm533, d, c);
  } else if (n == 2) {
    x.exp_lines = c->exp_lines / 2;
    xmv200_sns_ae_step(&xmv200_sns_sc500ai_xm533, d, &x);
  }
}
