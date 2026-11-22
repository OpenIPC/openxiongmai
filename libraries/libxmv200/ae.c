// SPDX-License-Identifier: MIT
/*
 * Auto exposure, as the vendor's isp daemon runs it (its AE library:
 * registration at 0x1f09c, the per-frame run at 0x20c34, the metering at
 * 0x20410, the split at 0x1fb50), and auto white balance by gray world.
 *
 * Statistics: isp.ko's AWB zones, 32 x 16 of each sensor's picture with
 * their R, G and B means (write() the channel, read() 0xc00 bytes: three
 * arrays of 512 u16, 12-bit linear; G is the two greens' over two), which
 * the driver takes each frame. As the daemon (0x1bb44), a zone's luma is
 * (77 R gr + 150 G gg + 29 B gb) >> 12 with the white balance and the
 * ISP's digital gain in the gains (0x100 = 1x); the zones are averaged
 * into an 8x8 grid and that into 7x7 (neighbours' means).
 *
 * Metering (0x20410): the 7x7 zones sorted by luma and their weighted mean
 * taken over a part of them -- all (mode 0), the darkest (2), the
 * brightest (1) or the middle (3), the part being (255 - bias)/255 -- as an
 * 8-bit value.
 *
 * The step (0x20c34): when the luma is more than the tolerance off the
 * target, the exposure (lines x gain, 0x100 = a line at 1x) of the frame
 * that luma is from is scaled towards target / luma (1x..4x either way)
 * by (ratio - 1) / speed, at least 6. The step starts from the last one's
 * exposure (the daemon keeps a ring of them, and takes the previous); the
 * picture lagging it by a frame or two is what the speed damps.
 *
 * The split (0x1fb50): lines first, up to the frame length less the
 * sensor's margin (anti-flicker: whole half periods of the mains light),
 * then the sensor's analog gain (its driver's again_calc), its digital gain
 * (dgain_calc), and the rest the ISP's (0x4964), at most 8x.
 *
 * The target, tolerance, speed, metering mode and the 7x7 weights are the
 * AE context's in the tuning's shared page (the daemon's, at +0x1214 on
 * the XM530V200: +0x78 speed, +0x79 tolerance, +0x7a target, +0x80 mode,
 * +0x84 bias, +0xa0 weights); other families' pages keep them elsewhere,
 * and get the XM530V200 tunings' values (57, 3, 8, the centre weighted 2).
 *
 * Without statistics (a read that fails or reads nothing) AE samples the
 * frames VENC gets instead (every 16th pixel of every 16th row) towards a
 * mean luma of 110. White balance is gray world on the same samples: the R
 * and B gains (0x4967, 0x100 = 1x) move until the mean chroma is neutral.
 *
 * Each sensor's picture is a band of the first stream with its group
 * (struct xmv200_ae_sensor). Where the frame thread sends the digital gain
 * with every frame (XM535, XM533), AE hands it the value instead of
 * sending it.
 */
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include "sns_tuning.h"
#include "xmv200.h"
#include "awb.h"
#include "xmv200_priv.h"

#define ISP_DGAIN 0x4964 /* {chn, gain} -> the ISP's digital gain */
#define ISP_WB 0x4965    /* {chn, R, G, B}: white balance gains (0x100 = 1x) */

#define FRAME_TARGET 110 /* mean luma of the frames, without statistics */
#define STEP 16
#define MAX_SNS 3
#define ISP_DGAIN_MAX 0x800
#define ZONES 512
#define AE_CTX 0x1214 /* the AE context in the XM530V200 page */

static const uint8_t again_reg[] = {0x00, 0x08, 0x09, 0x0b, 0x0f};

/* the daemon's metering and step parameters */
struct ae_param {
  uint8_t target, tolerance, speed, mode;
  uint16_t bias;
  uint8_t weight[49];
};

static const struct ae_param ae_default = {
    57, 3, 8, 0, 80,
    {1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 1, 1, 1, 2, 2, 2, 2, 2, 1, 1, 2, 2, 2,
     2, 2, 1, 1, 2, 2, 2, 2, 2, 1, 1, 1, 2, 2, 2, 1, 1, 1, 1, 1, 1, 1, 1, 1}};

struct ae_state {
  struct xmv200_ae_sensor where;
  uint32_t max_lines; /* the frame length less the sensor's margin */
  uint32_t lines;     /* exposure */
  int again;          /* SmartSens without a driver: index into again_reg */
  uint32_t again_idx, again_real; /* with a driver: its step, 1/32 */
  uint32_t dgain_reg, dgain_real;
  uint32_t dgain;     /* ISP, 0x100 = 1x */
  uint32_t r, b;      /* white balance */
  uint64_t total;     /* the exposure set, lines x gain (0x100 = 1x) */
  unsigned n;         /* statistics steps */
  uint8_t luma;       /* the last metered, 8 bits */
};

static struct ae_state st[MAX_SNS];
static int stats_seen; /* the statistics have been non-zero */
static struct ae_param par;
static const struct xmv200_sns_drv *drv; /* NULL: SmartSens registers */
static int hdr;                          /* the short frame's exposure too */
static int n_sns, handoff, stats_ok = 1;
static uint32_t line_rate;          /* the sensors' lines a second, or 0 */
static volatile uint32_t flicker; /* the half period in lines, or 0 */
static volatile int flicker_hz;
static int isp_fd = -1, mem_fd = -1;
static unsigned count[XMV200_MAX_STREAMS];

static void open_devs(void) {
  if (isp_fd < 0)
    isp_fd = open("/dev/isp", O_RDWR);
  if (mem_fd < 0)
    mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
}

/* the AE context's parameters in the tuning's page, where it is the
 * XM530V200's and they make sense */
static void params(const struct xmv200_sns_tuning *t) {
  const uint8_t *a = t->page + AE_CTX;

  par = ae_default;
  if (!t->chip || strcmp(t->chip, "xm530v200") ||
      t->page_size < AE_CTX + 0xa0 + 49)
    return;
  if (a[0x7a] < 16 || a[0x7a] > 200 || !a[0x78] || a[0x78] > 64 ||
      a[0x79] > 32 || a[0x80] > 3)
    return;
  par.speed = a[0x78];
  par.tolerance = a[0x79];
  par.target = a[0x7a];
  par.mode = a[0x80];
  par.bias = a[0x84] | a[0x85] << 8;
  memcpy(par.weight, a + 0xa0, sizeof(par.weight));
}

void xmv200_ae_init(const struct xmv200_isp_cfg *c,
                    const struct xmv200_ae_sensor *where, int give) {
  const struct xmv200_sns_tuning *t = c->tuning;

  n_sns = c->n_chn < MAX_SNS ? c->n_chn : MAX_SNS;
  handoff = give;
  drv = c->drv;
  hdr = drv && c->attr.hdr && drv->set_exposure_short;
  stats_ok = 1;
  stats_seen = 0;
  params(t);
  for (int i = 0; i < n_sns; i++)
    st[i] = (struct ae_state){.where = where[i],
                              /* the frame length less the sensor's margin */
                              .max_lines = t->chn_attr[2] - 4,
                              .lines = t->exp_lines,
                              .again_real = 0x20,
                              .dgain_real = 0x20,
                              .dgain = 0x100,
                              .r = 0x100,
                              .b = 0x100};
  /* with a driver: its exposure limit for the frame rate it set, and the
   * tuning's start in lines */
  for (int i = 0; drv && i < n_sns; i++) {
    uint32_t max = xmv200_isp_max_lines(where[i].isp_chn);

    if (max)
      st[i].max_lines = max;
    st[i].lines >>= drv->exp_shift;
  }
  for (int i = 0; i < n_sns; i++)
    st[i].total = (uint64_t)(st[i].lines ? st[i].lines : 1) << 8;
  /* the frame length and rate the driver runs the sensors at */
  line_rate = 0;
  if (drv && xmv200_isp_full_lines(where[0].isp_chn))
    line_rate = xmv200_isp_full_lines(where[0].isp_chn) * c->attr.fps;
  xmv200_ae_set_flicker(flicker_hz);
  open_devs();
}

void xmv200_ae_set_flicker(int hz) {
  flicker_hz = hz == 50 || hz == 60 ? hz : 0;
  flicker = flicker_hz && line_rate ? line_rate / (2 * flicker_hz) : 0;
}

static void apply(struct ae_state *a) {
  const struct xmv200_ae_sensor *s = &a->where;
  uint32_t dg[2] = {s->isp_chn, a->dgain};
  uint32_t wb[4] = {s->isp_chn, a->r, 0x100, a->b};
  uint32_t g[3];
  uint64_t gain = a->total / (a->lines ? a->lines : 1); /* 0x100 = 1x */

  if (drv) {
    struct xmv200_sns_dev d = {s->bus, s->addr, s->isp_chn};

    drv->set_exposure(&d, a->lines);
    if (hdr) /* the long one over the ratio, rounded up */
      drv->set_exposure_short(&d, (a->lines + drv->hdr_ratio - 1) /
                                      (drv->hdr_ratio ? drv->hdr_ratio : 16));
    drv->set_gain(&d, a->again_idx, a->dgain_reg);
  } else {
    xmv200_i2c_write(s->bus, s->addr, 0x3e02, 2, (a->lines << 4) & 0xff, 1);
    xmv200_i2c_write(s->bus, s->addr, 0x3e01, 2, (a->lines >> 4) & 0xff, 1);
    xmv200_i2c_write(s->bus, s->addr, 0x3e00, 2, (a->lines >> 12) & 0x0f, 1);
    xmv200_i2c_write(s->bus, s->addr, 0x3e09, 2, again_reg[a->again], 1);
  }
  if (handoff)
    xmv200_isp_set_dgain(s->isp_chn, a->dgain);
  else
    ioctl(isp_fd, ISP_DGAIN, dg);
  /* the gain AWB's saturation follows (100 = 1x); its own gains, or
   * gray world's where it has no calibration */
  xmv200_awb_set_gain(s->isp_chn, gain * 100 >> 8);
  if (xmv200_awb_gains(s->isp_chn, g)) {
    if (handoff)
      xmv200_isp_set_wb(s->isp_chn, wb + 1);
    else
      ioctl(isp_fd, ISP_WB, wb);
  }
}

/* the longest exposure in lines, and lines for want of them: anti-flicker's
 * whole half periods where one fits */
static uint32_t lines_for(uint64_t want, uint32_t max_lines) {
  uint32_t f = flicker;

  if (f && f <= max_lines)
    max_lines -= max_lines % f;
  if (want > max_lines)
    return max_lines;
  if (f && want >= f)
    return want - want % f;
  return want ? want : 1;
}

/*
 * The split: lines, then the sensor's analog gain, its digital gain and
 * the ISP's for the rest of total (lines x gain, 0x100 = 1x).
 */
static void split(struct ae_state *a, uint64_t total) {
  const struct xmv200_sns_ae *lim = xmv200_isp_sns_ae(a->where.isp_chn);
  uint32_t lines = lines_for(total >> 8, a->max_lines);
  uint64_t gain = total / lines; /* 0x100 = 1x */
  uint32_t idx = 0, real = 0x20, reg = 0, dreal = 0x20;

  if (gain < 0x100)
    gain = 0x100;
  a->lines = lines;
  if (drv) {
    uint32_t amax = lim && lim->again_max ? lim->again_max : 0x200;
    uint32_t dmax = lim && lim->dgain_max ? lim->dgain_max : 0x20;
    uint64_t g32 = gain >> 3; /* 1/32 */

    drv->again_calc(g32 < amax ? (uint32_t)g32 : amax, &idx, &real);
    if (!real)
      real = 0x20;
    g32 = gain * 0x20 / real >> 3; /* the rest, 1/32 */
    drv->dgain_calc(g32 < dmax ? (uint32_t)g32 : dmax, &reg, &dreal);
    if (!dreal)
      dreal = 0x20;
    a->again_idx = idx;
    a->again_real = real;
    a->dgain_reg = reg;
    a->dgain_real = dreal;
    gain = gain * 0x20 / real * 0x20 / dreal;
  } else {
    a->again = 0;
    while (a->again < 4 && gain >= (0x200u << a->again))
      a->again++;
    gain >>= a->again;
  }
  a->dgain = gain > ISP_DGAIN_MAX ? ISP_DGAIN_MAX : gain < 0x100 ? 0x100 : gain;
}

/* the most total can be */
static uint64_t total_max(const struct ae_state *a) {
  const struct xmv200_sns_ae *lim = xmv200_isp_sns_ae(a->where.isp_chn);
  uint64_t m = (uint64_t)a->max_lines << 8;

  if (drv)
    m = m * (lim && lim->again_max ? lim->again_max : 0x200) / 0x20 *
        (lim && lim->dgain_max ? lim->dgain_max : 0x20) / 0x20;
  else
    m <<= 4;
  return m * ISP_DGAIN_MAX / 0x100;
}

/* ---- with statistics ---- */

/* the metered luma (8 bits) of a channel's zones, or -1 */
static int meter(struct ae_state *a) {
  static uint16_t z[3][ZONES];
  uint32_t grid8[64], grid7[49], idx[49], sel, n = 49;
  uint64_t gr, gg, gb, sum = 0, wsum = 0;
  uint32_t ch = a->where.isp_chn;
  int lo, hi;

  if (xmv200_isp_stats(isp_fd, ch, z, sizeof(z)))
    return -1;
  /* the white balance (AWB's, else gray world's) and the ISP's digital
   * gain, 0x100 = 1x */
  uint32_t wbg[3] = {a->r, 0x100, a->b};

  xmv200_awb_gains(ch, wbg);
  gr = (uint64_t)wbg[0] * a->dgain >> 8;
  gg = (uint64_t)wbg[1] * a->dgain >> 8;
  gb = (uint64_t)wbg[2] * a->dgain >> 8;
  for (int y = 0; y < 8; y++)
    for (int x = 0; x < 8; x++) {
      uint64_t s = 0;

      for (int j = 0; j < 2; j++)
        for (int i = 0; i < 4; i++) {
          int k = (y * 2 + j) * 32 + x * 4 + i;
          uint64_t l = (77 * z[0][k] * gr + 150 * z[1][k] * gg +
                        29 * z[2][k] * gb) >> 12;

          s += l > 0xfe00 ? 0xfe00 : l;
        }
      grid8[y * 8 + x] = s >> 3;
    }
  xmv200_bd_sample((int)(a - st), grid8);
  for (int y = 0; y < 7; y++)
    for (int x = 0; x < 7; x++)
      grid7[y * 7 + x] = (grid8[y * 8 + x] + grid8[y * 8 + x + 1] +
                          grid8[(y + 1) * 8 + x] + grid8[(y + 1) * 8 + x + 1]) >>
                         2;
  for (unsigned i = 0; i < n; i++) {
    idx[i] = i;
    sum |= grid7[i];
  }
  /* all zero: no statistics on this chip, until some came; after that a
   * black picture (covered) */
  if (!sum)
    return stats_seen ? 0 : -1;
  stats_seen = 1;
  /* sorted darkest first, with their weights */
  for (unsigned i = 0; i < n; i++)
    for (unsigned j = i + 1; j < n; j++)
      if (grid7[idx[j]] < grid7[idx[i]]) {
        uint32_t t = idx[i];

        idx[i] = idx[j];
        idx[j] = t;
      }
  sel = n * (par.bias < 0x100 ? 0xff - par.bias : 0) / 0xff;
  if (!sel)
    sel = 1;
  if (sel > n)
    sel = n;
  switch (par.mode) {
  case 1: /* the brightest */
    lo = n - sel;
    hi = n;
    break;
  case 2: /* the darkest */
    lo = 0;
    hi = sel;
    break;
  case 3: /* the middle */
    lo = (n - sel) / 2;
    hi = lo + sel;
    break;
  default:
    lo = 0;
    hi = n;
  }
  sum = 0;
  for (int i = lo; i < hi; i++) {
    sum += (uint64_t)grid7[idx[i]] * par.weight[idx[i]];
    wsum += par.weight[idx[i]];
  }
  if (!wsum)
    return -1;
  sum /= wsum;
  if (sum > 0xff00)
    sum = 0xff00;
  return (int)((sum + 0x80) >> 8);
}

/* the daemon's step: the exposure the luma is from towards the target */
static void ae_stats_step(struct ae_state *a, int luma) {
  /* the last step's, as the daemon (its ring's previous entry); the lag
   * of the picture behind it is taken up by the speed */
  uint64_t base = a->total, total = base, d;
  uint32_t t = par.target, l = luma ? luma : 1, ratio;
  uint32_t speed = par.speed ? par.speed : 1;

  a->luma = luma;
  if ((uint32_t)abs((int)l - (int)t) > par.tolerance) {
    ratio = l < t ? (t << 8) / l : (l << 8) / t;
    if (ratio > 0x400)
      ratio = 0x400;
    d = base * (ratio - 0x100) / speed >> 8;
    if (d < 6)
      d = 6;
    if (l < t)
      total = base + d;
    else
      total = base > d + 0x100 ? base - d : 0x100;
  }
  if (total > total_max(a))
    total = total_max(a);
  if (total < 0x100)
    total = 0x100;
  a->total = total;
  a->n++;
  split(a, total);
}

/* ---- without: the frames ---- */

/* one step of AE: scale the total exposure by target / measured, taking
 * exposure lines first and gain last (and back in the other order) */
static void ae_frame_step(struct ae_state *a, uint32_t y) {
  uint64_t want;

  if (y < 1)
    y = 1;
  if (abs((int)y - FRAME_TARGET) < 6)
    return;
  /* half the way, so the loop does not oscillate */
  want = a->total * (FRAME_TARGET + y) / (2 * y);
  if (want > total_max(a))
    want = total_max(a);
  if (want < 0x100)
    want = 0x100;
  a->total = want;
  split(a, want);
}

static void awb_step(struct ae_state *a, int u, int v) {
  /* chroma above neutral (128): too red (V) or too blue (U) */
  a->r = a->r * (256 - (v - 128)) / 256;
  a->b = a->b * (256 - (u - 128)) / 256;
  if (a->r < 0x80)
    a->r = 0x80;
  if (a->r > 0x400)
    a->r = 0x400;
  if (a->b < 0x80)
    a->b = 0x80;
  if (a->b > 0x400)
    a->b = 0x400;
}

/* the last metered luma (8 bits) and exposure of sensor c, for the tools */
int xmv200_ae_state(int c, uint32_t *luma, uint32_t *lines, uint32_t *gain) {
  if (c < 0 || c >= n_sns)
    return -1;
  *luma = st[c].luma;
  *lines = st[c].lines;
  *gain = (uint32_t)(st[c].total / (st[c].lines ? st[c].lines : 1));
  return stats_ok;
}

void xmv200_ae_frame(const struct xmv200_frame *f) {
  uint32_t ysz, len;
  uint8_t *m = NULL;
  unsigned k;
  int changed = 0;

  if (!n_sns || mem_fd < 0 || f->chn >= XMV200_MAX_STREAMS)
    return;
  k = count[f->chn]++;
  /* statistics: an AE step every other frame of the sensor's stream */
  for (int c = 0; stats_ok && isp_fd >= 0 && !(k % 2) && c < n_sns; c++) {
    int l;

    if (st[c].where.stream != f->chn)
      continue;
    if ((l = meter(&st[c])) < 0) {
      stats_ok = 0; /* none: from the frames */
      break;
    }
    ae_stats_step(&st[c], l);
    changed |= 1 << c;
  }
  /* the frames, every third: gray world, motion, and AE without them */
  if (!(k % 3)) {
    ysz = f->stride * f->height;
    len = ysz + ysz / 2;
    for (int c = 0; c < n_sns; c++) {
      struct ae_state *a = &st[c];
      uint32_t band = f->height / a->where.n_bands, sy = 0, su = 0, sv = 0,
               n = 0;

      if (a->where.stream != f->chn)
        continue;
      if (!m) {
        m = mmap(NULL, len, PROT_READ, MAP_SHARED, mem_fd, f->y);
        if (m == MAP_FAILED)
          return;
      }
      for (uint32_t y = a->where.band * band; y < (a->where.band + 1) * band;
           y += STEP)
        for (uint32_t x = 0; x < f->width; x += STEP) {
          const uint8_t *uv = m + ysz + (y / 2) * f->stride + (x & ~1u);

          sy += m[y * f->stride + x];
          su += uv[0];
          sv += uv[1];
          n++;
        }
      if (!n)
        continue;
      xmv200_md_sample(c, m, f->stride, f->width, a->where.band * band, band);
      if (!stats_ok)
        ae_frame_step(a, sy / n);
      if (xmv200_awb_gains(a->where.isp_chn, (uint32_t[3]){0}))
        awb_step(a, su / n, sv / n); /* no calibration: gray world */
      changed |= 1 << c;
    }
    if (m)
      munmap(m, len);
  }
  for (int c = 0; c < n_sns; c++)
    if (changed & 1 << c)
      apply(&st[c]);
}
