// SPDX-License-Identifier: MIT
/*
 * Auto exposure and auto white balance from the pictures themselves.
 *
 * The statistics are a coarse sample of a frame VENC got (every 16th pixel
 * of every 16th row; a spliced picture is one band per sensor, top to
 * bottom). That keeps the loop independent of the BPU's statistics blocks.
 *
 * AE steers mean luma to a target through, in order: the sensor's exposure
 * (lines, up to the frame length), its coarse analog gain (SmartSens
 * 0x3e09: 1x, 2x, 4x, 8x, 16x) and isp.ko's digital gain (0x4964, 0x100 =
 * 1x). AWB is gray world: the R and B gains (0x4967, 0x100 = 1x) move
 * until the mean chroma is neutral.
 *
 * Anti-flicker (xmv200_ae_set_flicker): an exposure at least one of the
 * mains light's half periods long is a whole number of them, so every frame
 * sees the same light.
 *
 * Each sensor's picture is a band of the first stream with its group
 * (struct xmv200_ae_sensor). Where the frame thread sends the digital gain
 * with every frame (XM535, XM533), AE hands it the value instead of
 * sending it.
 */
#include <fcntl.h>
#include <stdlib.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include "sns_tuning.h"
#include "xmv200.h"
#include "xmv200_priv.h"

#define ISP_DGAIN 0x4964 /* {chn, gain} -> the ISP's digital gain */
#define ISP_WB 0x4967    /* {chn, R | G << 16, B}: white balance gains */

#define AE_TARGET 110 /* mean luma, 0..255 */
#define STEP 16
#define MAX_SNS 3

static const uint8_t again_reg[] = {0x00, 0x08, 0x09, 0x0b, 0x0f};

struct ae_state {
  struct xmv200_ae_sensor where;
  uint32_t max_lines; /* the frame length less the sensor's margin */
  uint32_t lines;     /* exposure */
  int again;          /* index into again_reg */
  uint32_t dgain;     /* ISP, 0x100 = 1x */
  uint32_t r, b;      /* white balance */
};

static struct ae_state st[MAX_SNS];
static const struct xmv200_sns_drv *drv; /* NULL: SmartSens registers */
static int hdr;                          /* the short frame's exposure too */
static int n_sns, handoff;
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

void xmv200_ae_init(const struct xmv200_isp_cfg *c,
                    const struct xmv200_ae_sensor *where, int give) {
  const struct xmv200_sns_tuning *t = c->tuning;

  n_sns = c->n_chn < MAX_SNS ? c->n_chn : MAX_SNS;
  handoff = give;
  drv = c->drv;
  hdr = drv && c->attr.hdr && drv->set_exposure_short;
  for (int i = 0; i < n_sns; i++)
    st[i] = (struct ae_state){.where = where[i],
                              /* the frame length less the sensor's margin */
                              .max_lines = t->chn_attr[2] - 4,
                              .lines = t->exp_lines,
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
  uint32_t wb[3] = {s->isp_chn, a->r | 0x100 << 16, a->b};

  if (drv) {
    struct xmv200_sns_dev d = {s->bus, s->addr, s->isp_chn};
    uint32_t idx, reg, real;

    drv->set_exposure(&d, a->lines);
    if (hdr) /* the long one over the ratio, rounded up */
      drv->set_exposure_short(&d, (a->lines + drv->hdr_ratio - 1) /
                                      (drv->hdr_ratio ? drv->hdr_ratio : 16));
    drv->again_calc(0x20u << a->again, &idx, &real);
    drv->dgain_calc(0x20, &reg, &real); /* the sensor's at 1x: */
    drv->set_gain(&d, idx, reg);        /* the ISP's takes the rest */
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
  ioctl(isp_fd, ISP_WB, wb);
}

/* one step of AE: scale the total exposure by target / measured, taking
 * exposure lines first and gain last (and back in the other order) */
static void ae_step(struct ae_state *a, uint32_t y) {
  uint32_t max_lines = a->max_lines;
  /* total = lines * again * dgain / 0x100 */
  uint64_t total = (uint64_t)a->lines * (1u << a->again) * a->dgain / 0x100;
  uint64_t want;

  if (y < 1)
    y = 1;
  if (abs((int)y - AE_TARGET) < 6)
    return;
  /* half the way, so the loop does not oscillate */
  want = total * (AE_TARGET + y) / (2 * y);
  if (want < 1)
    want = 1;

  a->again = 0;
  a->dgain = 0x100;
  /* anti-flicker: the longest exposure of whole half periods, and any
   * exposure over one period rounded down to them, gain making up the rest */
  uint32_t f = flicker, lines;

  if (f && f <= max_lines)
    max_lines -= max_lines % f;
  if (want <= max_lines) {
    if (!f || want < f) {
      a->lines = want;
      return;
    }
    lines = want - want % f;
  } else {
    lines = max_lines;
  }
  a->lines = lines;
  while (a->again < 4 && want > (uint64_t)lines << (a->again + 1))
    a->again++;
  a->dgain = want * 0x100 / ((uint64_t)lines << a->again);
  if (a->dgain > 0x800)
    a->dgain = 0x800; /* 8x: noise beyond that */
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

void xmv200_ae_frame(const struct xmv200_frame *f) {
  uint32_t ysz, len;
  uint8_t *m = NULL;

  if (!n_sns || mem_fd < 0 || f->chn >= XMV200_MAX_STREAMS ||
      count[f->chn]++ % 3) /* the sensor takes a frame or two */
    return;
  ysz = f->stride * f->height;
  len = ysz + ysz / 2;
  for (int c = 0; c < n_sns; c++) {
    struct ae_state *a = &st[c];
    uint32_t band = f->height / a->where.n_bands, sy = 0, su = 0, sv = 0,
             k = 0;

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
        k++;
      }
    if (!k)
      continue;
    ae_step(a, sy / k);
    awb_step(a, su / k, sv / k);
    apply(a);
  }
  if (m)
    munmap(m, len);
}
