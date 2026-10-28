// SPDX-License-Identifier: MIT
/*
 * The ISP: sensor, MIPI receiver, VI window and isp.ko (/dev/isp), the way
 * the vendor's isp daemon brings a channel up.
 *
 * isp.ko does the register work. It takes:
 * - plain ioctls on /dev/isp ('I', no size in the number: each command
 *   copies a fixed-size struct);
 * - a shared parameter page (8 pages through mmap after 0x400c490c), which
 *   the per-frame BPU work reads;
 * - per-channel buffers in MMZ (3D-NR, defog, lens correction).
 * The MIPI receiver, the VI windows and the ISP's clocks are written from
 * user space through /dev/mem.
 *
 * Each channel runs its sensor mode's program (sns_tuning.h): the parameter
 * blocks, with the structural steps here between them.
 */
#include <fcntl.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include "sns_tuning.h"
#include "xmv200.h"
#include "xmv200_priv.h"

#define ISP_PAGES 0x400c490c   /* {-, count, -}: the shared pages */
#define ISP_RESET_PAGES 0x4963 /* {0} */
#define ISP_COMM_INIT 0x4962   /* 56 bytes, then the same to 0x49d2 */
#define ISP_COMM_INIT_VPSS 0x49d2
#define ISP_CHN_ENABLE 0x4961 /* {chn, 8, chn, 0} */
#define ISP_CHN_ATTR 0x4908   /* {chn, 0, frame length, ...} 28 bytes */
#define ISP_EXPOSURE 0x4964   /* {chn, lines} */
#define ISP_GAIN 0x4965       /* {chn, again, dgain, ispgain} */
#define ISP_CHN_INIT 0x490e   /* 52 bytes: vicu_chn_init + bpu_chn_init */
#define ISP_CHN_SIZE 0x4980   /* {chn, x0 | y0 << 16, w | h << 16} */
#define ISP_LDC 0x49e9        /* 48 bytes */
#define ISP_COMM 0x4912       /* 28 bytes */
#define ISP_VICU_RUN 0x495e   /* {chn} */
#define ISP_WAIT_FRAME 0x490a /* 12 bytes; the channels with a new frame */

#define VI 0x30000000
#define VICU(c) (VI + 0x40000 + (c) * 0x10000)

static int isp_fd = -1;
static uint8_t *isp_page;
static pthread_t isp_thread;
static volatile int isp_running;

static int isp_cmd(uint32_t cmd, void *arg) {
  int r = ioctl(isp_fd, cmd, arg);

  if (r < 0)
    fprintf(stderr, "libxmv200: isp %#x failed\n", cmd);
  return r;
}

static void sns_write(const struct xmv200_isp_sensor *s, uint16_t reg,
                      uint8_t val) {
  xmv200_i2c_write(s->bus, s->addr, reg, 2, val, 1);
}

static void sns_table(const struct xmv200_isp_sensor *s,
                      const struct xmv200_sns_reg *r, int n) {
  for (int i = 0; i < n; i++)
    sns_write(s, r[i].reg, r[i].val);
}

/* The VI window of a channel (the vendor's VI_SetChnAttr through /dev/mem):
 * x0/x1 and y0/y1 at +0x98..+0xa4, the MIPI source of a second channel,
 * and its window in the shared receiver block at 0x30080098. */
static void vi_window(const struct xmv200_isp_chn *c,
                      const struct xmv200_sns_tuning *t) {
  uint32_t b = VICU(c->chn);

  xmv200_reg_write(b + 0x98, c->x0);
  xmv200_reg_write(b + 0x9c, c->x0 + t->width);
  xmv200_reg_write(b + 0xa0, c->y0);
  xmv200_reg_write(b + 0xa4, c->y0 + t->height);
  if (c->chn == 1) {
    xmv200_reg_write(VI + 0x80098, 0);
    xmv200_reg_write(VI + 0x8009c, t->width);
    xmv200_reg_write(VI + 0x800a0, 0);
    xmv200_reg_write(VI + 0x800a4, t->height);
  }
  if (c->mipi) {
    xmv200_reg_write(b + 0xc0, 0x110050);
    xmv200_reg_write(b + 0xc4, c->mipi);
  } else {
    xmv200_reg_write(b + 0xc4, 0);
  }
  xmv200_reg_write(b + 0x08, 0xffff0000);
}

static void chn_size(const struct xmv200_isp_chn *c,
                     const struct xmv200_sns_tuning *t) {
  uint32_t a[3] = {c->chn, c->x0 | c->y0 << 16, t->width | t->height << 16};

  isp_cmd(ISP_CHN_SIZE, a);
}

static int block(const struct xmv200_isp_block *b, uint32_t chn,
                 const struct xmv200_buf *bufs) {
  uint8_t p[0x400];

  if (b->size > sizeof(p))
    return -1;
  memcpy(p, b->data, b->size);
  memcpy(p, &chn, 4);
  for (int i = 0; i < 9 && b->patch[i].buf; i++) {
    uint32_t w = (bufs[b->patch[i].buf].phys + b->patch[i].rel - 0x80000000u) >> 4;

    memcpy(p + b->patch[i].off, &w, 4);
  }
  return isp_cmd(b->cmd, p);
}

/* the 3D-NR buffers, the sizes the mode needs */
static int nr_alloc(uint32_t chn, const struct xmv200_sns_tuning *t,
                    struct xmv200_buf *bufs) {
  static const char *const names[] = {NULL, "isp_nr_alpha_ch%u",
                                      "isp_nr_idx_ch%u", "isp_nr_idx_max_ch%u",
                                      "isp_nr_fetch_ch%u"};
  char name[32];

  for (int i = XMV200_BUF_NR_ALPHA; i <= XMV200_BUF_NR_FETCH; i++) {
    snprintf(name, sizeof(name), names[i], chn);
    if (xmv200_mmz_alloc(&bufs[i], name, t->buf_size[i], 0))
      return -1;
  }
  return 0;
}

static int ldc(const struct xmv200_isp_chn *c,
               const struct xmv200_sns_tuning *t) {
  uint32_t a[12] = {c->chn};
  struct xmv200_buf b;
  char name[32];

  if (c->ldc && t->ldc[1]) {
    snprintf(name, sizeof(name), "isp_ldc_ch%u", c->chn);
    if (xmv200_mmz_alloc(&b, name, 0x1000, 0))
      return -1;
    memcpy(b.virt, t->ldc_table, t->ldc_size < 0x1000 ? t->ldc_size : 0x1000);
    /* enable, centre, strength, the picture size and the table */
    memcpy(a, t->ldc, sizeof(a));
    a[0] = c->chn;
    a[9] = b.phys;
  }
  return isp_cmd(ISP_LDC, a);
}

static int run_program(const struct xmv200_isp_chn *c,
                       const struct xmv200_isp_sensor *s,
                       const struct xmv200_sns_tuning *t) {
  struct xmv200_buf bufs[XMV200_BUF_COUNT];
  char name[32];

  memset(bufs, 0, sizeof(bufs));
  for (int i = 0; i < t->n_program; i++) {
    uint16_t op = t->program[i];
    int r = 0;

    if (op & 0x8000) {
      if (block(&t->blocks[op & 0x7fff], c->chn, bufs))
        return -1;
      continue;
    }
    switch (op) {
    case XMV200_OP_SNS_INIT:
      sns_write(s, 0x0100, 0); /* standby while it is set up */
      sns_table(s, t->init_regs, t->n_init_regs);
      break;
    case XMV200_OP_SNS_MODE:
      sns_table(s, t->mode_regs, t->n_mode_regs);
      break;
    case XMV200_OP_SNS_EXPOSURE:
      sns_table(s, t->exposure_regs, t->n_exposure_regs);
      break;
    case XMV200_OP_SNS_FLIP:
      sns_write(s, 0x3221, s->flip);
      break;
    case XMV200_OP_MIPI_CTRL:
      if (c->chn == 0)
        xmv200_reg_write(VI + 0x4000, t->mipi_ctrl);
      break;
    case XMV200_OP_VI_SYNC:
      if (c->chn == 0) {
        xmv200_reg_write(VI + 0x430c, t->vi_sync);
        xmv200_reg_write(VI + 0x4310, t->vi_sync);
      }
      break;
    case XMV200_OP_VI_WINDOW:
      vi_window(c, t);
      break;
    case XMV200_OP_NR_ALLOC:
      r = nr_alloc(c->chn, t, bufs);
      break;
    case XMV200_OP_DEFOG_ALLOC:
      snprintf(name, sizeof(name), "isp_defog_ch%u", c->chn);
      r = xmv200_mmz_alloc(&bufs[XMV200_BUF_DEFOG], name,
                           t->buf_size[XMV200_BUF_DEFOG], 0);
      break;
    case XMV200_OP_LDC:
      r = ldc(c, t);
      break;
    case XMV200_OP_CHN_ATTR: {
      uint32_t a[7];

      memcpy(a, t->chn_attr, sizeof(a));
      a[0] = c->chn;
      r = isp_cmd(ISP_CHN_ATTR, a);
      break;
    }
    case XMV200_OP_AE_INIT: {
      uint32_t e[2] = {c->chn, t->dgain}, g[4];

      memcpy(g, t->gain, sizeof(g));
      g[0] = c->chn;
      r = isp_cmd(ISP_EXPOSURE, e) | isp_cmd(ISP_GAIN, g);
      break;
    }
    case XMV200_OP_CHN_INIT: {
      /* the VICU/BPU channel: its input (second byte), the capture and BPU
       * windows, the sensor's line timing, the active picture, and the size
       * VPSS makes of it (the board's) */
      uint32_t a[13];

      memcpy(a, t->chn_init, sizeof(a));
      a[0] = c->chn | (c->chn + 1) << 8;
      a[10] = a[11] = c->out_width | c->out_height << 16;
      r = isp_cmd(ISP_CHN_INIT, a);
      break;
    }
    case XMV200_OP_CHN_SIZE:
      chn_size(c, t);
      break;
    case XMV200_OP_VICU_IRQ0:
    case XMV200_OP_VICU_IRQ1:
      xmv200_reg_write(VICU(c->chn) + 0x7050, op == XMV200_OP_VICU_IRQ1);
      break;
    case XMV200_OP_VICU_RUN: {
      uint32_t a = c->chn;

      r = isp_cmd(ISP_VICU_RUN, &a);
      break;
    }
    }
    if (r)
      return -1;
  }
  return 0;
}

static int frames_ae; /* answer each frame with its exposure (XM535) */
/* the digital gain (0x100 = 1x) it answers with, per channel: the vendor's
 * at capture time until AE sets its own */
static volatile uint32_t dgain[3] = {0x109, 0x109, 0x109};

void xmv200_isp_set_dgain(uint32_t chn, uint32_t gain) {
  if (chn < 3)
    dgain[chn] = gain;
}

/* isp.ko's frame wait: the return value has a bit per channel with a new
 * frame. The XM535's offline BPU takes a channel's frame only after the
 * exposure for it (0x4964/0x4965) came, as the vendor AE sends it. */
static void *isp_frames(void *p) {
  uint32_t arg[3];

  while (isp_running) {
    int r = ioctl(isp_fd, ISP_WAIT_FRAME, arg);

    if (r < 0) {
      usleep(20000);
      continue;
    }
    for (uint32_t c = 0; frames_ae && c < 3; c++)
      if (r & 1 << c) {
        uint32_t e[2] = {c, dgain[c]}, g[4] = {c, 0x1db, 0x100, 0x247};

        ioctl(isp_fd, ISP_EXPOSURE, e);
        ioctl(isp_fd, ISP_GAIN, g);
      }
  }
  return NULL;
}

int xmv200_isp_start(const struct xmv200_isp_cfg *cfg) {
  const struct xmv200_sns_tuning *t = cfg->tuning;
  int n = cfg->n_chn;

  isp_fd = open("/dev/isp", O_RDWR);
  if (isp_fd < 0)
    return -1;

  /* windows first: the channels' VI and isp.ko's view of them */
  for (int c = 0; c < n; c++) {
    struct xmv200_isp_chn w = cfg->chn[c];

    w.x0 = 1; /* the crop each lens gets comes with CHN_SIZE later */
    w.y0 = 0;
    vi_window(&w, t);
    chn_size(&w, t);
  }

  /* the shared parameter page */
  uint32_t zero = 0, pages[3] = {0, 8, 0};
  if (isp_cmd(ISP_RESET_PAGES, &zero) || isp_cmd(ISP_PAGES, pages))
    return -1;
  isp_page = mmap(NULL, pages[1] << 12, PROT_READ | PROT_WRITE, MAP_SHARED,
                  isp_fd, 0);
  if (isp_page == MAP_FAILED)
    return -1;
  memset(isp_page, 0, pages[1] << 12);
  memcpy(isp_page, t->page,
         t->page_size < pages[1] << 12 ? t->page_size : pages[1] << 12);

  /* the common setup: picture size, the offline BPU (MemorySaverV2) with
   * its raw buffers, the channels */
  uint32_t mask = (1u << n) - 1;
  uint8_t comm[64];

  memset(comm, 0, sizeof(comm));
  memcpy(comm, t->comm, sizeof(t->comm));
  comm[9] = n;     /* ISPs in use */
  comm[10] = mask; /* their channels */
  comm[12] = n;
  comm[13] = mask;
  if (isp_cmd(ISP_COMM_INIT, comm) || isp_cmd(ISP_COMM_INIT_VPSS, comm))
    return -1;
  for (int c = 0; c < n; c++) {
    uint32_t a[4] = {c, 8, c, 0};

    if (isp_cmd(ISP_CHN_ENABLE, a))
      return -1;
  }

  for (int c = 0; c < n; c++) {
    if (c == 1) {
      uint32_t a[7] = {0, 1, 1};

      isp_cmd(ISP_COMM, a);
    }
    if (run_program(&cfg->chn[c], &cfg->sensor[c], t))
      return -1;
  }

  isp_running = 1;
  pthread_create(&isp_thread, NULL, isp_frames, NULL);
  return 0;
}

/* the frame waits alone, for boards whose ISP a board program set up */
int xmv200_isp_frames_start(void) {
  frames_ae = 1;
  if (isp_fd < 0)
    isp_fd = open("/dev/isp", O_RDWR);
  if (isp_fd < 0)
    return -1;
  isp_running = 1;
  return pthread_create(&isp_thread, NULL, isp_frames, NULL);
}

void xmv200_isp_stop(void) {
  if (isp_running) {
    isp_running = 0;
    pthread_join(isp_thread, NULL);
  }
}
