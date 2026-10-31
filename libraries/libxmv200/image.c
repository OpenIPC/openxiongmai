// SPDX-License-Identifier: MIT
/*
 * Image controls: what the vendor's isp daemon does for Sofia's colour,
 * sharpness, mirror/flip, day/night colour and anti-flicker settings, on the
 * same isp.ko blocks.
 *
 * Each control starts from the block the sensor's tuning last sent (its own
 * setting, 50 here) and scales it the daemon's way:
 * - 0x49de {chn, Y, U, V}: the luma and chroma gains, 0x100 = 1x. A value
 *   v of 0..100 is the gain v * 512 / 100 up to 50 (1x), then on to 0x3ff
 *   at 100 (camera_set_color's luma, and its saturation through the CSC).
 * - 0x49df {chn, 65 16-bit points}: the tone curve. The daemon makes it from
 *   one contrast value: at 50 the straight line i * 4; below, flatter about
 *   the middle; above, steeper, clipped at 0 and 255.
 * - 0x49e4: the hue word at +0x160, -256..255 (9 bits) for 0..100, after a
 *   65-entry table (+0x5c) whose neutral is 0x80.
 * - 0x49f0: the sharpening strength, byte 8.
 * Mirror and flip go to the sensor (its driver's flip, or SmartSens 0x3221),
 * XORed with the layout's own flip. Anti-flicker is AE's (ae.c).
 */
#include <string.h>

#include "sns.h"
#include "xmv200.h"
#include "xmv200_priv.h"

#define ISP_LUMA 0x49de
#define ISP_CURVE 0x49df
#define ISP_HUE 0x49e4
#define ISP_SHARPEN 0x49f0
#define HUE_SIZE 0x164
#define HUE_TABLE 0x5c
#define HUE_WORD 0x160

static const struct xmv200_image defaults = {50, 50, 50, 50, 50, 0, 0, 0, 0};
static struct xmv200_image cur = {50, 50, 50, 50, 50, 0, 0, 0, 0};

void xmv200_image_defaults(struct xmv200_image *im) { *im = defaults; }

void xmv200_image_get(struct xmv200_image *im) { *im = cur; }

void xmv200_image_reset(void) {
  cur = defaults;
  xmv200_ae_set_flicker(0);
}

static uint32_t clamp100(uint32_t v) { return v > 100 ? 100 : v; }

/* 0..100 to a gain, 0x100 at 50 (the daemon's luma and saturation map) */
static uint32_t gain_of(uint32_t v) {
  uint32_t g;

  v = clamp100(v);
  g = v <= 50 ? (v << 9) / 100 : (v * 0x2ff - 0x95ce) / 50 + 0x100;
  return g > 0x3ff ? 0x3ff : g;
}

static uint32_t scale(uint32_t base, uint32_t gain, uint32_t max) {
  uint32_t v = (uint32_t)(((uint64_t)base * gain + 0x80) >> 8);

  return v > max ? max : v;
}

/* the daemon's tone curve for a contrast of c (0..100) */
static void contrast_curve(uint32_t c, uint16_t *pt) {
  c = clamp100(c);
  if (c < 50) {
    uint32_t d = 50 - c, acc = d * 0x200, step = (0x100 - 4 * d) * 4;
    int clip = 0;

    for (int i = 0; i < 65; i++, acc += step) {
      uint32_t v = clip ? 0xff : acc >> 8;

      if (v > 0xff) {
        v = 0xff;
        clip = 1;
      }
      pt[i] = v;
    }
  } else {
    uint32_t e = c - 50, div = 0x100 - 4 * e;
    uint32_t off = e * 0x20000 / div, step = 0x10000 / div * 4, x = 0;
    int clip = 0;

    for (int i = 0; i < 65; i++, x += step) {
      uint32_t v = clip ? 0xff : (x > off ? (x - off) >> 8 : 0);

      if (v > 0xff) {
        v = 0xff;
        clip = 1;
      }
      pt[i] = v;
    }
  }
}

static int send(uint32_t cmd, void *p) {
  return xmv200_isp_cmd(cmd, p) < 0 ? -1 : 0;
}

static int set_luma(uint32_t chn, const struct xmv200_image *im) {
  uint32_t size, w[4];
  const uint8_t *b = xmv200_isp_image_block(chn, ISP_LUMA, &size);

  if (!b || size < sizeof(w))
    return -1;
  memcpy(w, b, sizeof(w));
  w[0] = chn;
  w[1] = scale(w[1], gain_of(im->brightness), 0x3ff);
  w[2] = im->mono ? 0 : scale(w[2], gain_of(im->saturation), 0x3ff);
  w[3] = im->mono ? 0 : scale(w[3], gain_of(im->saturation), 0x3ff);
  return send(ISP_LUMA, w);
}

static int set_curve(uint32_t chn, const struct xmv200_image *im) {
  uint32_t size;
  const uint8_t *b = xmv200_isp_image_block(chn, ISP_CURVE, &size);
  uint8_t p[0x88];
  uint16_t pt[65];

  if (!b || size < 4 + sizeof(pt))
    return -1;
  memset(p, 0, sizeof(p));
  memcpy(p, b, size < sizeof(p) ? size : sizeof(p));
  memcpy(p, &chn, 4);
  if (im->contrast != 50) {
    contrast_curve(im->contrast, pt);
    memcpy(p + 4, pt, sizeof(pt));
  }
  return send(ISP_CURVE, p);
}

static int set_hue(uint32_t chn, const struct xmv200_image *im) {
  uint32_t size, h = clamp100(im->hue), word;
  const uint8_t *b = xmv200_isp_image_block(chn, ISP_HUE, &size);
  uint8_t p[HUE_SIZE];

  if (!b)
    return -1;
  if (size > sizeof(p))
    size = sizeof(p);
  memcpy(p, b, size);
  /* a block recorded short: the rest of the table neutral */
  for (uint32_t o = size < HUE_TABLE ? HUE_TABLE : (size + 3) & ~3u;
       o < HUE_WORD; o += 4) {
    uint32_t n = 0x80;

    memcpy(p + o, &n, 4);
  }
  memcpy(p, &chn, 4);
  word = (h <= 50 ? (h << 8) / 50 - 0x100 : (h - 50) * 0xff / 50) & 0x1ff;
  memcpy(p + HUE_WORD, &word, 4);
  return send(ISP_HUE, p);
}

static int set_sharpen(uint32_t chn, const struct xmv200_image *im) {
  uint32_t size;
  const uint8_t *b = xmv200_isp_image_block(chn, ISP_SHARPEN, &size);
  uint8_t p[XMV200_IMG_MAX];
  uint32_t v;

  if (!b || size < 12)
    return -1;
  memcpy(p, b, size);
  memcpy(p, &chn, 4);
  v = b[8] * clamp100(im->sharpness) / 50;
  p[8] = v > 0xff ? 0xff : v;
  return send(ISP_SHARPEN, p);
}

static int set_flip(const struct xmv200_isp_cfg *cfg, uint32_t c,
                    const struct xmv200_image *im) {
  const struct xmv200_isp_sensor *s = &cfg->sensor[c];
  int mirror = !!(s->flip & 0x06) ^ !!im->mirror;
  int flip = !!(s->flip & 0x60) ^ !!im->flip;

  if (cfg->drv && cfg->drv->set_flip) {
    struct xmv200_sns_dev d = {s->bus, s->addr, c};

    cfg->drv->set_flip(&d, mirror, flip);
    return 0;
  }
  /* SmartSens without a driver */
  return xmv200_i2c_write(s->bus, s->addr, 0x3221, 2,
                          (mirror ? 0x06 : 0) | (flip ? 0x60 : 0), 1) < 0
             ? -1
             : 0;
}

int xmv200_image_set(const struct xmv200_image *im) {
  const struct xmv200_isp_cfg *cfg = xmv200_isp_running();
  int r = 0;

  if (!cfg)
    return -1;
  for (int c = 0; c < cfg->n_chn; c++) {
    if (im->brightness != cur.brightness || im->saturation != cur.saturation ||
        im->mono != cur.mono)
      r |= set_luma(c, im);
    if (im->contrast != cur.contrast)
      r |= set_curve(c, im);
    if (im->hue != cur.hue)
      r |= set_hue(c, im);
    if (im->sharpness != cur.sharpness)
      r |= set_sharpen(c, im);
    if (im->mirror != cur.mirror || im->flip != cur.flip)
      r |= set_flip(cfg, c, im);
  }
  if (im->anti_flicker != cur.anti_flicker)
    xmv200_ae_set_flicker(im->anti_flicker);
  cur = *im;
  return r;
}
