// SPDX-License-Identifier: MIT
/*
 * Auto exposure and auto white balance from the pictures themselves.
 *
 * The statistics are a coarse sample of a frame VPSS made (every 16th pixel
 * of every 16th row; a spliced picture is one band per sensor, top to
 * bottom). That keeps the loop independent of the BPU's statistics blocks.
 *
 * AE steers mean luma to a target through, in order: the sensor's exposure
 * (lines, up to the frame length), its coarse analog gain (SmartSens
 * 0x3e09: 1x, 2x, 4x, 8x, 16x) and isp.ko's digital gain (0x4964, 0x100 =
 * 1x). AWB is gray world: the R and B gains (0x4967, 0x100 = 1x) move
 * until the mean chroma is neutral.
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

static const uint8_t again_reg[] = {0x00, 0x08, 0x09, 0x0b, 0x0f};

struct ae_state {
  uint32_t lines; /* exposure */
  int again;      /* index into again_reg */
  uint32_t dgain; /* ISP, 0x100 = 1x */
  uint32_t r, b;  /* white balance */
};

static struct ae_state st[2];
static const struct xmv200_isp_cfg *cfg;
static int isp_fd = -1, mem_fd = -1;
static unsigned count;

void xmv200_ae_init(const struct xmv200_isp_cfg *c) {
  const struct xmv200_sns_tuning *t = c->tuning;
  uint32_t lines = 0;

  /* the mode's starting exposure: 0x3e00..0x3e02 hold lines << 4 */
  for (int i = 0; i < t->n_exposure_regs; i++)
    if (t->exposure_regs[i].reg >= 0x3e00 && t->exposure_regs[i].reg <= 0x3e02)
      lines |= (uint32_t)t->exposure_regs[i].val
               << (8 * (0x3e02 - t->exposure_regs[i].reg));
  cfg = c;
  for (int i = 0; i < c->n_chn; i++)
    st[i] = (struct ae_state){lines >> 4, 0, 0x100, 0x100, 0x100};
  isp_fd = open("/dev/isp", O_RDWR);
  mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
}

static void apply(int c) {
  const struct xmv200_isp_sensor *s = &cfg->sensor[c];
  struct ae_state *a = &st[c];
  uint32_t dg[2] = {cfg->chn[c].chn, a->dgain};
  uint32_t wb[3] = {cfg->chn[c].chn, a->r | 0x100 << 16, a->b};

  xmv200_i2c_write(s->bus, s->addr, 0x3e02, 2, (a->lines << 4) & 0xff, 1);
  xmv200_i2c_write(s->bus, s->addr, 0x3e01, 2, (a->lines >> 4) & 0xff, 1);
  xmv200_i2c_write(s->bus, s->addr, 0x3e00, 2, (a->lines >> 12) & 0x0f, 1);
  xmv200_i2c_write(s->bus, s->addr, 0x3e09, 2, again_reg[a->again], 1);
  ioctl(isp_fd, ISP_DGAIN, dg);
  ioctl(isp_fd, ISP_WB, wb);
}

/* one step of AE: scale the total exposure by target / measured, taking
 * exposure lines first and gain last (and back in the other order) */
static void ae_step(int c, uint32_t y) {
  struct ae_state *a = &st[c];
  uint32_t max_lines = cfg->tuning->chn_attr[2] - 4; /* VTS */
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
  if (want <= max_lines) {
    a->lines = want;
    return;
  }
  a->lines = max_lines;
  while (a->again < 4 && want > (uint64_t)max_lines << (a->again + 1))
    a->again++;
  a->dgain = want * 0x100 / ((uint64_t)max_lines << a->again);
  if (a->dgain > 0x800)
    a->dgain = 0x800; /* 8x: noise beyond that */
}

static void awb_step(int c, int u, int v) {
  struct ae_state *a = &st[c];

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
  uint8_t *m;
  int n;

  if (!cfg || mem_fd < 0 || count++ % 3) /* the sensor takes a frame or two */
    return;
  n = cfg->n_chn;
  ysz = f->stride * f->height;
  len = ysz + ysz / 2;
  m = mmap(NULL, len, PROT_READ, MAP_SHARED, mem_fd, f->y);
  if (m == MAP_FAILED)
    return;
  for (int c = 0; c < n; c++) {
    uint32_t band = f->height / n, sy = 0, su = 0, sv = 0, k = 0;

    for (uint32_t y = c * band; y < (c + 1) * band; y += STEP)
      for (uint32_t x = 0; x < f->width; x += STEP) {
        const uint8_t *uv = m + ysz + (y / 2) * f->stride + (x & ~1u);

        sy += m[y * f->stride + x];
        su += uv[0];
        sv += uv[1];
        k++;
      }
    if (!k)
      continue;
    ae_step(c, sy / k);
    awb_step(c, su / k, sv / k);
    apply(c);
  }
  munmap(m, len);
}
