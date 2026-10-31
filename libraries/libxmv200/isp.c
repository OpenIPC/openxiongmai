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
#define ISP_CHN_SIZE 0x4980   /* the channel's window */
#define ISP_CHN_ENABLE2 0x4914 /* {chn, on}: vicu_chn_enable (XM535) */
#define ISP_LDC 0x49e9        /* 48 bytes */
#define ISP_COMM 0x4912       /* 28 bytes */
#define ISP_VICU_RUN 0x495e   /* {chn} */
#define ISP_WAIT_FRAME 0x490a /* 12 bytes; the channels with a new frame */

#define VI 0x30000000

static const struct xmv200_chip *cur_chip;
#define VICU(c) (VI + cur_chip->vicu + (c) * 0x10000)

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

/* the channel's window: {chn, x0 | y0 << 16, w | h << 16}, or on the
 * XM535 the longer form with the VI device, the window at words 4..7 and
 * the channel's input */
static void chn_size(const struct xmv200_isp_chn *c,
                     const struct xmv200_sns_tuning *t) {
  uint32_t a[12];

  if (t->n_chn_size < 12) {
    a[0] = c->chn;
    a[1] = c->x0 | c->y0 << 16;
    a[2] = t->width | t->height << 16;
  } else {
    memcpy(a, t->chn_size, sizeof(a));
    a[0] = c->chn;
    a[1] = c->vi_dev;
    a[2] = (a[2] & ~0x10000u) | (c->vi_dev ? 0x10000u : 0);
    a[4] = c->x0;
    a[5] = c->y0;
    a[6] = t->width;
    a[7] = t->height;
    a[10] = !c->vi_dev;
    a[11] = c->chn + 1;
  }
  isp_cmd(ISP_CHN_SIZE, a);
}

#define MAX_BUFS 16

static int block(const struct xmv200_isp_block *b, uint32_t chn,
                 const struct xmv200_buf *bufs) {
  uint8_t p[0x400];

  if (b->size > sizeof(p))
    return -1;
  memcpy(p, b->data, b->size);
  if (!b->raw)
    memcpy(p, &chn, 4);
  for (int i = 0; i < 16 && b->patch[i].buf; i++) {
    uint32_t a = bufs[b->patch[i].buf - 1].phys + b->patch[i].rel;
    uint32_t w = b->patch[i].shift ? (a - 0x80000000u) >> b->patch[i].shift : a;

    memcpy(p + b->patch[i].off, &w, 4);
  }
  return isp_cmd(b->cmd, p);
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

/* where a channel is: its own steps differ for the first channel on a
 * receiver, and with how many share it */
struct chn_place {
  int first_on_phy, n_on_phy;
};

/* what each channel's sensor driver set: the frame length isp.ko and AE
 * take */
static struct xmv200_sns_ae sns_ae[3];

/*
 * A tuning's sensor runs are what the vendor's libsns does in turn: the
 * mode's registers, the frame rate, and the exposure and gain the ISP
 * starts from. With a driver, the driver does them (xmv200_sns_run).
 */
static void sns_run(const struct xmv200_isp_cfg *cfg,
                    const struct xmv200_isp_chn *c,
                    const struct xmv200_isp_sensor *s, int n) {
  struct xmv200_sns_dev d = {s->bus, s->addr, c->chn};
  struct xmv200_sns_ctx ctx = {cfg->mode, cfg->attr, cfg->tuning->exp_lines,
                               &sns_ae[c->chn < 3 ? c->chn : 0]};

  xmv200_sns_run(cfg->drv, &d, n, &ctx);
}

static int run_program(const struct xmv200_isp_cfg *cfg,
                       const struct xmv200_isp_chn *c,
                       const struct xmv200_isp_sensor *s,
                       const struct xmv200_sns_tuning *t,
                       const struct xmv200_chip *chip, struct chn_place pl) {
  struct xmv200_buf bufs[MAX_BUFS];
  uint32_t rx = VI + chip->rx + c->phy * 0x1000; /* the channel's receiver */
  char name[40];

  memset(bufs, 0, sizeof(bufs));
  for (int i = 0; i < t->n_program; i++) {
    uint16_t op = t->program[i];
    int r = 0;

    if (op & 0x8000) {
      if (block(&t->blocks[op & 0x7fff], c->chn, bufs))
        return -1;
      continue;
    }
    if (op & 0x4000) {
      const struct xmv200_isp_buf *b = &t->bufs[op & 0xff];

      snprintf(name, sizeof(name), b->name, c->chn);
      if ((op & 0xff) >= MAX_BUFS ||
          xmv200_mmz_alloc(&bufs[op & 0xff], name, b->size, 0))
        return -1;
      continue;
    }
    if ((op & 0xf000) == 0x1000) {
      uint32_t w = c->chn << 24 | (op & 0xff);

      if (isp_cmd(0x4959, &w) < 0)
        return -1;
      continue;
    }
    if (op & 0x2000) {
      if ((op & 0xff) >= t->n_runs)
        return -1;
      if (cfg->drv && t->n_runs == xmv200_sns_n_runs(cfg->drv, cfg->mode))
        sns_run(cfg, c, s, op & 0xff);
      else
        sns_table(s, t->runs[op & 0xff].regs, t->runs[op & 0xff].n);
      continue;
    }
    switch (op) {
    case XMV200_OP_SNS_FLIP:
      sns_write(s, 0x3221, s->flip);
      break;
    case XMV200_OP_MIPI_CTRL:
      if (pl.first_on_phy)
        xmv200_reg_write(rx, t->mipi_ctrl);
      break;
    case XMV200_OP_VI_SYNC:
      if (pl.first_on_phy) {
        xmv200_reg_write(rx + 0x30c, t->vi_sync);
        if (chip->vi_sync2)
          xmv200_reg_write(rx + 0x310, t->vi_sync);
      }
      break;
    case XMV200_OP_VI_WINDOW:
      vi_window(c, t);
      break;
    case XMV200_OP_LDC:
      r = ldc(c, t);
      break;
    case XMV200_OP_CHN_ATTR: {
      uint32_t a[7];

      memcpy(a, t->chn_attr, sizeof(a));
      a[0] = c->chn | (chip->chn_attr_idx ? c->vi_idx << 4 : 0);
      if (cfg->drv && sns_ae[c->chn < 3 ? c->chn : 0].full_lines)
        a[2] = sns_ae[c->chn < 3 ? c->chn : 0].full_lines;
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
      /* the VICU/BPU channel: the channel and its input mask (and on the
       * XM535 how many channels share its receiver, and the receiver), the
       * capture and BPU windows, the sensor's line timing, the active
       * picture, and the size VPSS makes of it (the board's) */
      uint32_t a[20];
      int out = t->n_chn_init > 13 ? 11 : 10;

      memcpy(a, t->chn_init, sizeof(a));
      a[0] = c->chn | (1u << c->chn) << 8;
      if (t->n_chn_init > 13) {
        if (pl.n_on_phy > 1)
          a[0] |= (uint32_t)pl.n_on_phy << 16;
        a[2] = (a[2] & ~0xffu) | c->phy;
      }
      a[out] = a[out + 1] = c->out_width | c->out_height << 16;
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
    case XMV200_OP_CHN_ENABLE0:
    case XMV200_OP_CHN_ENABLE1: {
      uint32_t a[2] = {c->chn, op == XMV200_OP_CHN_ENABLE1};

      r = isp_cmd(ISP_CHN_ENABLE2, a);
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
  const struct xmv200_chip *chip = xmv200_chip_of(t);
  int n = cfg->n_chn;

  cur_chip = chip;

  isp_fd = open("/dev/isp", O_RDWR);
  if (isp_fd < 0)
    return -1;

  /* XM530V200: windows first, the channels' VI and isp.ko's view of them */
  for (int c = 0; chip->vi_window && c < n; c++) {
    struct xmv200_isp_chn w = cfg->chn[c];

    w.x0 = 1; /* the crop each lens gets comes with CHN_SIZE later */
    w.y0 = 0;
    vi_window(&w, t);
    chn_size(&w, t);
  }

  /* the shared parameter pages, as many as the tuning's image */
  uint32_t zero = 0, pages[3] = {0, (t->page_size + 0xfff) >> 12, 0};
  if (isp_cmd(ISP_RESET_PAGES, &zero) || isp_cmd(ISP_PAGES, pages))
    return -1;
  isp_page = mmap(NULL, pages[1] << 12, PROT_READ | PROT_WRITE, MAP_SHARED,
                  isp_fd, 0);
  if (isp_page == MAP_FAILED)
    return -1;
  memset(isp_page, 0, pages[1] << 12);
  memcpy(isp_page, t->page, t->page_size);

  /* the common setup: picture size, the offline BPU (MemorySaverV2) with
   * its raw buffers, the channels */
  uint32_t mask = (1u << n) - 1;
  uint8_t comm[64];

  memset(comm, 0, sizeof(comm));
  memcpy(comm, t->comm, sizeof(t->comm));
  if (chip->comm_n) {
    comm[chip->comm_n] = n;        /* ISPs in use */
    comm[chip->comm_n + 1] = mask; /* their channels */
    comm[chip->comm_n + 3] = n;
    comm[chip->comm_n + 4] = mask;
  }
  if (chip->comm_ib && t->comm[9] < (uint32_t)n)
    memcpy(comm + 36, &n, 4); /* the BPU's raw buffers, one per channel */
  if (isp_cmd(ISP_COMM_INIT, comm) || isp_cmd(ISP_COMM_INIT_VPSS, comm))
    return -1;
  for (int c = 0; c < n; c++) {
    uint32_t a[4] = {c, 8, c, 0};

    if (isp_cmd(ISP_CHN_ENABLE, a))
      return -1;
  }

  for (int c = 0; c < n; c++) {
    const struct xmv200_isp_chn *ch = &cfg->chn[c];
    struct chn_place pl = {1, 0};

    for (int k = 0; k < n; k++)
      if (cfg->chn[k].phy == ch->phy) {
        pl.n_on_phy++;
        if (k < c)
          pl.first_on_phy = 0;
      }
    if (c > 0) {
      /* a further channel: its VI device and its index there */
      uint32_t a[7] = {ch->vi_dev, ch->vi_idx, 1};

      isp_cmd(ISP_COMM, a);
    }
    if (run_program(cfg, ch, &cfg->sensor[c], t, chip, pl))
      return -1;
  }

  frames_ae = chip->frame_answer;
  isp_running = 1;
  pthread_create(&isp_thread, NULL, isp_frames, NULL);
  return 0;
}

void xmv200_isp_stop(void) {
  if (isp_running) {
    isp_running = 0;
    pthread_join(isp_thread, NULL);
  }
}

uint32_t xmv200_isp_max_lines(uint32_t chn) {
  return chn < 3 ? sns_ae[chn].max_lines : 0;
}
