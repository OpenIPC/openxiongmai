// SPDX-License-Identifier: MIT
/*
 * Auto white balance, the way the vendor's isp daemon does it (its AWB
 * library: init, run, ctrl and exit callbacks, a core that turns the
 * statistics into gains and a colour temperature), on the same inputs:
 *
 * Statistics: write the channel to /dev/isp, read 0xc00 bytes: 512 zones
 * (32 x 16) of mean R, then G (halved: two G pixels a quad), then B, as
 * 16-bit planes (isp.ko's awb_stat_get, CM_AWB_STAT).
 *
 * Calibration: the daemon keeps a channel's AWB context (0x100c bytes) in
 * isp.ko's shared page, and the sensor's tuning carries a snapshot of that
 * page. The context starts at 0x1a7c (XM530V200), 0x1d78 (XM535D1/D2),
 * 0x1d64 (XM535W1) or 0x1e00 (XM533), the same layout on all of them; it
 * is found by its colour matrix table:
 *   +0x00  bypass (1: gains 1x)        +0x04  0 auto, 1 manual, 3 auto
 *   +0x08  manual R, G, B gains (u16)  +0x0e  speed (u16); +0x10 its slope
 *   +0x1e..+0x21  bias bytes (0x80 neutral)  +0x22  the white bin quantile
 *   +0x2a  7x7 zone weights            +0x5c  CCM bypass, +0x60 CCM mode
 *   +0x64  manual CCM (12 s16)         +0x7c  3 x {CT, 12 s16} CCMs
 *   +0xf0  the sensor's calibration, 65 s16:
 *          [0..7] R, [8..15] B, [16..23] G coefficients of 8 lines, [24]
 *          the side of each a white zone is on (a bit each); [25], [26]
 *          the B and R ratios of the curve's start, [27], [28] its
 *          distance limits, [30..44] the curve's distances and [45..60] the
 *          colour temperatures at them, [61..63] the reference white's R,
 *          G, B gains (1/1024)
 *   +0xfb8 up to 5 zones in RGB that are (bit clear) or are not white
 *
 * The daemon's frame thread then sends the gains as isp.ko 0x4965, and the
 * matrix as 0x4966 after the saturation for the sensor's gain (the ISP
 * context's 16-step table at +0x3dd, XM530V200: page 0x8c): from 0x80 up
 * the matrix is kept and the chroma gains raised (s * 6 - 0x200), between
 * 0x40 and 0x80 the matrix is blended towards identity, at 0x40 and below
 * it is identity and the chroma gains go down (s * 4).
 *
 * Each frame: every zone that is white (inside the calibration's lines,
 * or a zone saying so), neither too dark nor too bright, is put in a bin by
 * how saturated it is under the current gains; the bins near grey count
 * the most. Their mean gives the raw gains G/R and G/B, taken every fifth
 * frame; the gains move there at a speed that is slower the further from
 * the reference white they are; the colour temperature comes from where
 * the gains sit on the calibration's curve, and the colour matrix is
 * interpolated between the three calibrated ones by it.
 */
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "awb.h"
#include "sns_tuning.h"
#include "xmv200.h"
#include "xmv200_priv.h"

#define ISP_CCM 0x4966
/* {chn, R, G, B}, 0x100 = 1x (the daemon sends them as FUN_0001be48: each
 * scaled by AE's factor, normalised to 0x7ff at most); 0x4967 is the black
 * level, not these */
#define ISP_WB 0x4965
#define N_ZONES 0x200
#define STAT_BYTES 0xc00
#define CTX_SIZE 0x100c
#define CAL 0xf0
#define BINS 0x81
#define MAX_CHN 3

/* the daemon's tables: by saturation, the bins added past the mean, and
 * the speed (/64) */
static const uint8_t bin_add[132] = {
    0,  0,  0,  0,  0,  0,  0,  1,  1,  1,  1,  1,  1,  2,  2,  2,  2,  2,
    3,  3,  3,  3,  3,  4,  4,  4,  4,  4,  5,  5,  5,  5,  6,  6,  6,  6,
    7,  7,  7,  7,  8,  8,  8,  8,  9,  9,  9,  9,  10, 10, 10, 11, 11, 11,
    12, 12, 12, 13, 13, 13, 14, 14, 15, 15, 16, 16, 17, 17, 18, 18, 19, 19,
    20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20,
    20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20,
    20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20,
    20, 20, 20, 20, 20, 20};
static const uint8_t speed_of[132] = {
    99, 97, 95, 92, 88, 84, 80, 76, 72, 68, 64, 60, 56, 52, 48, 44, 40, 36,
    32, 28, 24, 20, 17, 14, 12, 10, 9,  8,  8,  7,  7,  7,  6,  6,  6,  6,
    5,  5,  5,  5,  4,  4,  4,  4,  3,  3,  3,  3,  2,  2,  2,  2,  2,  2,
    2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,
    2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,
    2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,
    2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,  2,
    2,  2,  2,  2,  2,  2};

struct awb {
  int on;
  uint8_t ctx[CTX_SIZE]; /* the daemon's, from the page */
  /* the core's parameters (its init) */
  uint16_t speed, slope, hi, lo, bin_extra, bin_min, min_zones, pull_lo,
      pull_hi, pull;
  /* state */
  uint32_t sm[3];  /* the gains moving, << 10 */
  uint32_t raw[3]; /* the last raw gains taken (0xfffe: none yet) */
  uint16_t eval_r, eval_b; /* the gains the zones are judged under */
  uint16_t out[3];         /* gains, 1/1024 */
  uint16_t ct;
  uint32_t sat_idx;
  int first, cycle;
  uint16_t sent[3];
  int16_t ccm_sent[12];
  int ccm_valid;
  uint8_t weight[64];
  const uint8_t *isp; /* the ISP context in the page (XM530V200), or NULL */
  uint32_t gain;      /* the sensor's total gain, 100 = 1x */
  uint32_t chroma;    /* the chroma gain for it, 0x100 = 1x */
  uint32_t bl_sent;   /* the black level last sent, +1 (0: none) */
};

static struct awb awb[MAX_CHN];
static int stat_fd = -1;

static int16_t S16(const uint8_t *p, int o) {
  int16_t v;

  memcpy(&v, p + o, 2);
  return v;
}

static uint16_t U16(const uint8_t *p, int o) { return (uint16_t)S16(p, o); }

static int32_t S32(const uint8_t *p, int o) {
  int32_t v;

  memcpy(&v, p + o, 4);
  return v;
}

/* the calibration's s16 n */
static int cal(const struct awb *a, int n) { return S16(a->ctx, CAL + 2 * n); }

static uint32_t recip(uint32_t x) {
  return x ? (0x100000 + x / 2) / x : 0x100000;
}

/* the AWB contexts in a page: each at its colour matrix table (three
 * colour temperatures falling, the first row's diagonal plausible) */
static int find_ctx(const uint8_t *p, uint32_t size, uint32_t *at, int max) {
  int n = 0;

  for (uint32_t o = 0x7c; o + 0x80 <= size && n < max; o += 2) {
    uint16_t a = U16(p, o), b = U16(p, o + 26), c = U16(p, o + 52);
    int16_t d = S16(p, o + 4);

    if (a >= 4500 && a <= 7000 && b >= 3000 && b < a && c >= 2000 &&
        c < b && c <= 3200 && d > 200 && d < 700 &&
        o - 0x7c + CTX_SIZE <= size) {
      at[n++] = o - 0x7c;
      o += CTX_SIZE - 2;
    }
  }
  return n;
}

/* the vendor's zone list (+0xfb8): 1 when a zone says white, 2 when it
 * says not, 0 when none has it */
static int zones(const uint8_t *z, int32_t r, int32_t g, int32_t b) {
  uint32_t in = 0, out = 0;
  int n = z[0];

  if (n >= 6)
    return 0;
  for (int k = 0; k < n; k++) {
    int32_t x = S32(z, 4 + 4 * k), y = S32(z, 0x18 + 4 * k);
    int32_t d = S32(z, 0x2c + 4 * k), w = S32(z, 0x40 + 4 * k);
    int32_t c, e, f, t, den, m;
    int32_t c0, c1, c2, c3, c4, c5, c6, c7, c8, c9, c10, c11;
    int bits;

    if (!d)
      d = 1;
    c = (int32_t)((uint32_t)(x * w) / (uint32_t)d);
    e = (int32_t)((uint32_t)(w << 12) / (uint32_t)d);
    t = y + w;
    f = c + t;
    den = e < 0x1000 ? (f < 0x1000 ? 0x1000 : f) : (f < e ? e : f);
    c0 = 0x1000000 / den;
    c1 = (e << 12) / den;
    c2 = -((f * 0x1000) / den);
    m = c + (y - w);
    den = abs(m);
    den = e < 0x1000 ? (den < 0x1000 ? 0x1000 : den) : (den < e ? e : den);
    c3 = (int16_t)(0x1000000 / den);
    c4 = (int16_t)((e << 12) / den);
    c5 = m < 1 ? (int16_t)((-m * 0x1000) / den) : -(int16_t)((m * 0x1000) / den);
    m = t - c;
    den = abs(m);
    den = e < 0x1000 ? (den < 0x1000 ? 0x1000 : den) : (den < e ? e : den);
    c6 = (int16_t)(0x1000000 / den);
    c7 = -(int16_t)((e << 12) / den);
    c8 = m < 1 ? (int16_t)((-m * 0x1000) / den) : -(int16_t)((m * 0x1000) / den);
    m = (y - w) - c;
    den = abs(m);
    den = e < 0x1000 ? (den < 0x1000 ? 0x1000 : den) : (den < e ? e : den);
    c9 = (int16_t)(0x1000000 / den);
    c10 = (int16_t)((e << 12) / den);
    c11 = m < 1 ? (int16_t)((-m * 0x1000) / den) : -(int16_t)((m * 0x1000) / den);
    bits = (g * c5 + r * c3 + b * c4 > 0) ? 2 : 0;
    bits += g * (int16_t)c2 + r * c0 + b * (int16_t)c1 > 0;
    bits += g * c8 + r * c6 + b * c7 > 0 ? 4 : 0;
    bits += g * c11 + r * c9 + b * -c10 > 0 ? 8 : 0;
    if (bits == 10) {
      if (z[1] >> k & 1)
        out |= 1u << k;
      else
        in |= 1u << k;
    }
  }
  return out ? 2 : in ? 1 : 0;
}

static uint32_t clamp_u(uint32_t v, uint32_t max) { return v > max ? max : v; }

/* the core (the daemon's per-frame run): 0 with new gains, -1 too few
 * white zones */
static int core(int c, struct awb *a, const uint16_t *st) {
  const uint16_t *R = st, *G = st + N_ZONES, *B = st + 2 * N_ZONES;
  uint32_t sum_r[BINS], sum_g[BINS], sum_b[BINS], cnt[BINS];
  uint32_t er, eg, eb, total = 0, wsum = 0, binsel, lim, k;
  uint32_t wr = 0, wg = 0, wb = 0, ur = 0, ug = 0, ub = 0;
  uint32_t graw, braw, r10, g10, b10, nr, ng, nb, mx, mn, sat;
  int cyc = a->cycle;

  if (cyc < 1 || !a->eval_r || !a->eval_b) {
    er = (uint16_t)cal(a, 0x3d);
    eg = (uint16_t)cal(a, 0x3e);
    eb = (uint16_t)cal(a, 0x3f);
  } else {
    er = a->eval_r;
    eg = 0x400;
    eb = a->eval_b;
  }
  memset(sum_r, 0, sizeof(sum_r));
  memset(sum_g, 0, sizeof(sum_g));
  memset(sum_b, 0, sizeof(sum_b));
  memset(cnt, 0, sizeof(cnt));
  for (int i = 0; i < N_ZONES; i++) {
    int32_t r = R[i], g = G[i], b = B[i];
    uint32_t mask = 0, rr, gg, bb, hi, lo;
    int white, z;

    for (int l = 0; l < 8; l++)
      if (g * cal(a, 16 + l) + r * cal(a, l) + b * cal(a, 8 + l) > 0)
        mask |= 1u << l;
    z = zones(a->ctx + 0xfb8, r, g, b);
    white = z == 0 ? mask == (uint16_t)cal(a, 0x18) : z == 1;
    rr = (((uint32_t)r * er + 0x2000) & 0x3fffffff) >> 14;
    gg = (((uint32_t)g * eg + 0x2000) & 0x3fffffff) >> 14;
    bb = (((uint32_t)b * eb + 0x2000) & 0x3fffffff) >> 14;
    hi = rr > gg ? rr : gg;
    hi = hi > bb ? hi : bb;
    hi = clamp_u(hi, 0xff);
    lo = rr < gg ? rr : gg;
    lo = clamp_u(lo, 0xff);
    lo = lo < bb ? lo : bb;
    if (white && hi < a->hi && a->lo < lo) {
      uint32_t bin = (recip(hi) * (hi - lo) & 0x1fffffff) >> 13;
      uint32_t w = a->weight[((i & 0x1f) >> 2) + (i >> 6) * 8];

      if (bin >= BINS)
        bin = BINS - 1;
      sum_r[bin] += w * r;
      sum_g[bin] += w * g;
      sum_b[bin] += w * b;
      cnt[bin] += w;
    }
  }
  for (int i = 0; i < BINS; i++) {
    wsum += i * cnt[i];
    total += cnt[i];
  }
  if (!a->ctx[0x22]) {
    binsel = (wsum + total / 2) / (total ? total : 1);
  } else {
    /* the bin where the count passes the quantile */
    uint32_t q = a->ctx[0x22] * N_ZONES >> 8, acc = 0;

    for (binsel = 0; binsel < 0x80; binsel++) {
      acc += cnt[binsel];
      if (q <= acc)
        break;
    }
  }
  if (a->min_zones * N_ZONES > (total & 0xffff) * 0x400)
    return -1;
  lim = a->bin_extra + (uint16_t)(binsel + bin_add[a->sat_idx < 132 ? a->sat_idx : 131]);
  if (lim < a->bin_min)
    lim = a->bin_min;
  if (lim > 0x7f)
    lim = 0x80;
  for (uint32_t i = 0; i <= lim; i++) {
    uint32_t w = ((0x80 - i) * (0x80 - i) & 0x3ffffff) >> 10;

    w = w >= 16 ? 16 : w ? w : 1;
    wr += sum_r[i] * w;
    wg += sum_g[i] * w;
    wb += sum_b[i] * w;
    ur += sum_r[i];
    ug += sum_g[i];
    ub += sum_b[i];
  }
  if (wg > 0x1fffff) {
    wr = ur;
    wg = ug;
    wb = ub;
  }
  graw = clamp_u((uint32_t)(((uint64_t)wg * 0x400 + wr / 2) / (wr ? wr : 1)),
                 0xffff);
  braw = clamp_u((uint32_t)(((uint64_t)wg * 0x400 + wb / 2) / (wb ? wb : 1)),
                 0xffff);
  a->eval_r = graw;
  a->eval_b = braw;
  if (a->raw[0] == 0xfffe || cyc > 3) {
    a->raw[0] = graw;
    a->raw[1] = 0x400;
    a->raw[2] = braw;
  }
  a->cycle = cyc + 1 > 4 ? 0 : cyc + 1;
  r10 = a->raw[0] * 0x400;
  g10 = a->raw[1] * 0x400;
  b10 = a->raw[2] * 0x400;

  /* how far from the reference white: the speed */
  nr = clamp_u(r10 / ((uint16_t)cal(a, 0x3d) ?: 1), 0xfff);
  ng = clamp_u(g10 / ((uint16_t)cal(a, 0x3e) ?: 1), 0xfff);
  nb = clamp_u(b10 / ((uint16_t)cal(a, 0x3f) ?: 1), 0xfff);
  mx = nr > ng ? nr : ng;
  mx = mx > nb ? mx : nb;
  if (!mx)
    mx = 1;
  mn = nr < ng ? nr : ng;
  mn = mn < nb ? mn : nb;
  sat = (mx - mn) * 0x80 / mx;
  if (!a->first) {
    uint32_t si = (sat & 0xff) < 132 ? sat & 0xff : 131;

    k = a->speed + (((uint32_t)a->slope * speed_of[si] + 0x20) >> 6);
    k = (k < 0x400 ? k : 0x400) >> 2;
    r10 = (uint32_t)(((uint64_t)a->sm[0] * (0x100 - k) + (uint64_t)r10 * k) >> 8);
    g10 = (uint32_t)(((uint64_t)a->sm[1] * (0x100 - k) + (uint64_t)g10 * k) >> 8);
    b10 = (uint32_t)(((uint64_t)a->sm[2] * (0x100 - k) + (uint64_t)b10 * k) >> 8);
  } else {
    a->first = 0;
  }
  a->sm[0] = r10;
  a->sm[1] = g10;
  a->sm[2] = b10;
  a->sat_idx = sat & 0xff;

  /* the colour temperature: the distance along the calibration's curve */
  uint32_t r = (r10 + 0x200) >> 10, b = (b10 + 0x200) >> 10;
  uint32_t gq = (g10 + 0x200) & ~0x3ffu;
  int32_t q = (int32_t)((gq + (r & 0xffff) / 2) / (r ? r : 1));
  uint32_t d = (uint16_t)cal(a, 0x1b);

  if ((uint32_t)(q & 0xffff) <= (uint16_t)cal(a, 0x1a)) {
    int32_t dr = (q & 0xffff) - (uint16_t)cal(a, 0x1a);
    int32_t qb = (int32_t)((gq + (b & 0xffff) / 2) / (b ? b : 1));
    int32_t db = (qb & 0xffff) - (uint16_t)cal(a, 0x19);

    d = (abs(dr) + abs(db)) & 0xffff;
  }
  if (d < (uint16_t)cal(a, 0x1b))
    d = (uint16_t)cal(a, 0x1b);
  if ((uint16_t)cal(a, 0x1c) <= d)
    d = (uint16_t)cal(a, 0x1c);
  int i;

  for (i = 0; i < 15; i++)
    if (d <= (uint16_t)cal(a, 0x1d + i + 1))
      break;
  if (i == 15)
    i = 0;
  int32_t span = (uint16_t)cal(a, 0x1d + i + 1) - (uint16_t)cal(a, 0x1d + i);

  if (span < 1)
    span = 1;
  a->ct = (uint16_t)(((uint16_t)cal(a, 0x2d + i) *
                          (int32_t)((uint16_t)cal(a, 0x1d + i + 1) - d) +
                      (uint16_t)cal(a, 0x2d + i + 1) *
                          (int32_t)(d - (uint16_t)cal(a, 0x1d + i))) /
                     span);

  /* the gains, pulled towards the reference white where far from it */
  uint32_t rf = (uint16_t)cal(a, 0x3d) ?: 1, gf = (uint16_t)cal(a, 0x3e) ?: 1,
           bf = (uint16_t)cal(a, 0x3f) ?: 1;
  int32_t xr = (int32_t)clamp_u((r << 10) / rf, 0xfff);
  int32_t xg = (int32_t)clamp_u(gq / gf, 0xfff);
  int32_t xb = (int32_t)clamp_u((b << 10) / bf, 0xfff);
  int32_t hi2 = xr > xg ? xr : xg, lo2 = xr < xg ? xr : xg, s2, kk, dd, top;

  if (hi2 < xb)
    hi2 = xb;
  if (!hi2)
    hi2 = 1;
  if (xb < lo2)
    lo2 = xb;
  s2 = (hi2 - lo2) * 0x80 / hi2;
  s2 = s2 > a->pull_lo ? s2 : a->pull_lo;
  dd = a->pull_hi < s2 ? a->pull_hi - a->pull_lo : s2 - a->pull_lo;
  top = a->pull_lo + 1 < a->pull_hi ? a->pull_hi : a->pull_lo + 1;
  kk = ((0x80 - a->pull) * dd) / (a->pull_lo - (top & 0xffff));
  kk = (kk + 0x80) & 0xffff;
  {
    int32_t o[3] = {(int32_t)rf * (kk * (xr - 0x400) + 0x20000) >> 17,
                    (int32_t)gf * (kk * (xg - 0x400) + 0x20000) >> 17,
                    (int32_t)bf * (kk * (xb - 0x400) + 0x20000) >> 17};

    for (int j = 0; j < 3; j++)
      a->out[j] = (o[j] & 0xffff) < 0x100 ? 0x100 : (uint16_t)o[j];
  }
  (void)c;
  return 0;
}

/* the daemon's colour matrix for a colour temperature: between the three
 * calibrated ones, each row summing to 1 in auto */
static void ccm_for(const struct awb *a, uint32_t ct, int16_t out[12]) {
  const uint8_t *t = a->ctx + 0x7c;
  int mode = S32(a->ctx, 0x5c) == 1 ? 0xff : a->ctx[0x60];
  uint32_t c0 = U16(t, 0), c1 = U16(t, 26), c2 = U16(t, 52);

  if (mode == 1) {
    for (int i = 0; i < 12; i++)
      out[i] = S16(a->ctx, 0x64 + 2 * i);
    return;
  }
  if (mode == 0xff) {
    memset(out, 0, 24);
    out[1] = out[6] = out[11] = 0x100;
    return;
  }
  if (ct <= c2) {
    for (int i = 0; i < 12; i++)
      out[i] = S16(t, 54 + 2 * i);
  } else if (ct <= c1) {
    int32_t w = (int32_t)(c1 - ct) + (int32_t)(ct - c2);

    if (w < 1)
      w = 1;
    for (int i = 0; i < 12; i++)
      out[i] = (S16(t, 54 + 2 * i) * (int32_t)(c1 - ct) +
                S16(t, 28 + 2 * i) * (int32_t)(ct - c2)) / w;
  } else if (ct <= c0) {
    int32_t w = (int32_t)(c0 - ct) + (int32_t)(ct - c1);

    if (w < 1)
      w = 1;
    for (int i = 0; i < 12; i++)
      out[i] = (S16(t, 28 + 2 * i) * (int32_t)(c0 - ct) +
                S16(t, 2 + 2 * i) * (int32_t)(ct - c1)) / w;
  } else {
    for (int i = 0; i < 12; i++)
      out[i] = S16(t, 2 + 2 * i);
  }
  if (mode == 0) {
    out[3] = 0x100 - (out[1] + out[2]);
    out[7] = 0x100 - (out[5] + out[6]);
    out[11] = 0x100 - (out[9] + out[10]);
  }
}

/* the bias bytes (+0x1e..+0x21) on auto gains, as the daemon's run */
static void bias(const struct awb *a, uint32_t *r, uint32_t *b) {
  uint32_t x = a->ctx[0x1e], y = a->ctx[0x21], fr, fb;

  if (x < 0x81) {
    if (!(x & 0x80)) {
      fb = (0x100 - x) * 2 & 0xffff;
      fr = 0x100;
    } else {
      fr = fb = 0x100;
    }
  } else {
    fr = x << 1;
    fb = 0x100;
  }
  if (y < 0x81) {
    if (!(y & 0x80)) {
      fb = ((0x100 - y) * fb & 0x7fffff) >> 7;
      fr = ((0x100 - y) * fr & 0x7fffff) >> 7;
    }
  } else {
    fb = ((fb << 7) / y) & 0xffff;
    fr = ((fr << 7) / y) & 0xffff;
  }
  *r = (*r * (a->ctx[0x1f] * fb >> 7) & 0xffffff) >> 8;
  *b = (*b * (a->ctx[0x20] * fr >> 7) & 0xffffff) >> 8;
}

static void send_gains(uint32_t chn, struct awb *a, uint32_t r, uint32_t g,
                       uint32_t b) {
  uint32_t w[4] = {chn, r, g, b};

  if (a->sent[0] == r && a->sent[1] == g && a->sent[2] == b)
    return;
  if (xmv200_isp_cmd(ISP_WB, w) < 0)
    return;
  a->sent[0] = r;
  a->sent[1] = g;
  a->sent[2] = b;
}

/* the saturation for the gain (the daemon's frame thread): the chroma
 * gain, and the matrix blended towards identity where low */
static void saturate(struct awb *a, int16_t m[12]) {
  uint32_t s, idx = 0, frac, g = a->gain < 100 ? 100 : a->gain, q;

  a->chroma = 0x100;
  if (!a->isp)
    return;
  if (S32(a->isp, 0x3d8) == 1) {
    s = a->isp[0x3dc];
  } else {
    for (q = g / 100; q >>= 1;)
      idx++;
    if (idx > 15)
      idx = 15;
    frac = clamp_u((g - (100u << idx)) * 0xff / (100u << idx), 0xff);
    uint32_t s0 = a->isp[0x3dd + idx], s1 = a->isp[0x3dd + (idx < 15 ? idx + 1 : 15)];

    s = s0 < s1 ? s0 + frac * (s1 - s0) / 0xff : s0 - frac * (s0 - s1) / 0xff;
  }
  s = clamp_u(s, 0xff);
  if (S32(a->isp, 0x3d4) == 1) {
    a->chroma = s >= 0x80 ? s * 6 - 0x200 : s << 1;
    return;
  }
  if (s >= 0x80) {
    a->chroma = s * 6 - 0x200;
  } else if (s <= 0x40) {
    memset(m, 0, 24);
    m[1] = m[6] = m[11] = 0x100;
    a->chroma = s << 2;
  } else {
    /* the off-diagonal terms by (s - 0x40) / 0x40, the diagonal what makes
     * each row 1 */
    int32_t f = (int32_t)s - 0x40;
    int16_t x, y;

    x = (int16_t)((uint32_t)(m[2] * f) >> 6);
    y = (int16_t)((uint32_t)(m[3] * f) >> 6);
    m[1] = 0x100 - (x + y);
    m[2] = x;
    m[3] = y;
    x = (int16_t)((uint32_t)(m[5] * f) >> 6);
    y = (int16_t)((uint32_t)(m[7] * f) >> 6);
    m[5] = x;
    m[6] = 0x100 - (x + y);
    m[7] = y;
    x = (int16_t)((uint32_t)(m[9] * f) >> 6);
    y = (int16_t)((uint32_t)(m[10] * f) >> 6);
    m[9] = x;
    m[10] = y;
    m[11] = 0x100 - (x + y);
    a->chroma = 0x100;
  }
}

static void send_ccm(uint32_t chn, struct awb *a) {
  int16_t m[12];
  uint32_t w[13] = {chn};

  uint32_t chroma = a->chroma;

  ccm_for(a, a->ct, m);
  saturate(a, m);
  if (a->isp && a->chroma != chroma)
    xmv200_image_chroma(chn); /* the U/V gains follow */
  if (a->ccm_valid && !memcmp(m, a->ccm_sent, sizeof(m)))
    return;
  for (int i = 0; i < 12; i++)
    w[1 + i] = (uint32_t)(int32_t)m[i]; /* as the daemon: sign-extended */
  if (xmv200_isp_cmd(ISP_CCM, w) < 0)
    return;
  memcpy(a->ccm_sent, m, sizeof(m));
  a->ccm_valid = 1;
}

int xmv200_awb_init(const struct xmv200_isp_cfg *c) {
  const struct xmv200_sns_tuning *t = c->tuning;
  uint32_t at[MAX_CHN];
  int n;

  memset(awb, 0, sizeof(awb));
  n = t->page ? find_ctx(t->page, t->page_size, at, MAX_CHN) : 0;
  if (!n)
    return -1;
  for (int ch = 0; ch < c->n_chn && ch < MAX_CHN; ch++) {
    struct awb *a = &awb[ch];

    uint32_t off = at[ch < n ? ch : 0];

    memcpy(a->ctx, t->page + off, CTX_SIZE);
    /* the XM530V200's page: per channel 0x2a88, the ISP context at 0x8c,
     * the AWB's 0x1188 + 0x868 after it */
    if ((off - 0x1a7c) % 0x2a88 == 0 && off >= 0x1a7c)
      a->isp = t->page + off - 0x1188 - 0x868;
    a->gain = 100;
    a->chroma = 0x100;
    if (!cal(a, 0x3d) || !cal(a, 0x3e) || !cal(a, 0x3f))
      continue;
    /* the core's init (and the run's speed from the context) */
    a->speed = U16(a->ctx, 0xe);
    a->slope = (uint16_t)S32(a->ctx, 0x10);
    a->hi = 0xa0;
    a->lo = 1;
    a->bin_extra = 1;
    a->bin_min = 3;
    a->min_zones = 0x10;
    a->pull_lo = 0x2d;
    a->pull_hi = 0x5a;
    a->pull = 0x80;
    a->raw[0] = a->raw[1] = a->raw[2] = 0xfffe;
    a->first = 1;
    /* it starts at the reference white, 5600 K */
    for (int j = 0; j < 3; j++) {
      a->out[j] = (uint16_t)cal(a, 0x3d + j);
      a->sm[j] = (uint32_t)a->out[j] << 10;
    }
    a->ct = 0x15e0;
    /* the 7x7 weights to 8x8, the middle row and column twice */
    for (int y = 0; y < 8; y++)
      for (int x = 0; x < 8; x++)
        a->weight[y * 8 + x] =
            a->ctx[0x2a + 7 * (y < 4 ? y : y - 1) + (x < 4 ? x : x - 1)];
    a->on = 1;
  }
  if (stat_fd < 0)
    stat_fd = open("/dev/isp", O_RDWR);
  return 0;
}

void xmv200_awb_stop(void) {
  memset(awb, 0, sizeof(awb));
  if (stat_fd >= 0)
    close(stat_fd);
  stat_fd = -1;
}

int xmv200_awb_get(uint32_t chn, uint16_t gain[3], uint16_t *ct) {
  if (chn >= MAX_CHN || !awb[chn].on)
    return -1;
  memcpy(gain, awb[chn].sent, 3 * sizeof(uint16_t));
  *ct = awb[chn].ct;
  return 0;
}

/* the black level for the gain (the daemon's frame thread, XM530V200's
 * ISP context): +0x3ac 1 manual (+0x3b0), else its 16-step table (+0x3b2,
 * by log2 of the gain, interpolated); isp.ko 0x4967, the same for R, G, B */
static void black_level(uint32_t chn, struct awb *a) {
  uint32_t g = a->gain < 100 ? 100 : a->gain, idx = 0, frac, bl, q;

  if (!a->isp)
    return;
  if (S32(a->isp, 0x3ac) == 1) {
    bl = U16(a->isp, 0x3b0);
  } else {
    for (q = g / 100; q >>= 1;)
      idx++;
    if (idx > 15)
      idx = 15;
    frac = clamp_u((g - (100u << idx)) * 0xff / (100u << idx), 0xff);
    uint32_t b0 = U16(a->isp, 0x3b2 + idx * 2),
             b1 = U16(a->isp, 0x3b2 + (idx < 15 ? idx + 1 : 15) * 2);

    bl = b0 < b1 ? b0 + frac * (b1 - b0) / 0xff : b0 - frac * (b0 - b1) / 0xff;
  }
  if (bl + 1 != a->bl_sent) {
    uint32_t w[3] = {chn, bl | bl << 16, bl};

    if (xmv200_isp_cmd(0x4967, w) >= 0)
      a->bl_sent = bl + 1;
  }
}

void xmv200_awb_set_gain(uint32_t chn, uint32_t gain) {
  if (chn < MAX_CHN && awb[chn].on) {
    awb[chn].gain = gain;
    black_level(chn, &awb[chn]);
  }
}

uint32_t xmv200_awb_chroma(uint32_t chn) {
  return chn < MAX_CHN && awb[chn].on && awb[chn].isp ? awb[chn].chroma : 0;
}

int xmv200_awb_gains(uint32_t chn, uint32_t g[3]) {
  if (chn >= MAX_CHN || !awb[chn].on || !awb[chn].sent[1])
    return -1;
  g[0] = awb[chn].sent[0];
  g[1] = awb[chn].sent[1];
  g[2] = awb[chn].sent[2];
  return 0;
}

int xmv200_awb_frame(uint32_t chn) {
  struct awb *a = chn < MAX_CHN ? &awb[chn] : NULL;
  uint16_t st[STAT_BYTES / 2];
  uint32_t r, g, b, op;
  int ok = -1;

  if (!a || !a->on || stat_fd < 0)
    return -1;
  op = S32(a->ctx, 4);
  if (S32(a->ctx, 0) == 1) { /* bypass */
    send_gains(chn, a, 0x100, 0x100, 0x100);
    return 0;
  }
  if (op != 1) {
    if (!xmv200_isp_stats(stat_fd, chn, st, sizeof(st)))
      ok = core(chn, a, st);
  }
  if (op == 1) { /* manual */
    r = U16(a->ctx, 8);
    g = U16(a->ctx, 10);
    b = U16(a->ctx, 12);
  } else {
    r = (a->out[0] + 1) >> 2;
    g = (a->out[1] + 1) >> 2;
    b = (a->out[2] + 1) >> 2;
    bias(a, &r, &b);
  }
  send_gains(chn, a, r, g, b);
  send_ccm(chn, a);
  return ok;
}

int xmv200_awb_state(uint32_t sensor, uint16_t gain[3], uint16_t *ct) {
  return xmv200_awb_get(sensor, gain, ct);
}
