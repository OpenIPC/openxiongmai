// SPDX-License-Identifier: MIT
/*
 * The XM510 side of libxmisp: device access, and the interpreter for sensor
 * scripts. The interfaces are the ones the vendor daemon uses, as traced:
 *
 * - /dev/isp: raw commands 0x101 (write) and 0x103 (read) on {addr, val},
 *   for the ISP/VI block at 0x40000000 and GPIO;
 * - the system controller (0x20000000), written through /dev/mem;
 * - /dev/xm_i2c: commands 1 (write) and 3 (read) on {dev, reg, reg_bytes,
 *   data, data_bytes}; the driver takes the address from the low byte of dev;
 * - /dev/mmz: _IO('M', 0) resets, _IOWR('M', 1, 40) allocates by name
 *   {char name[32]; u32 size; u32 phys} (the XM510's 40-byte record);
 * - /dev/vi: _IO('V', 6) initialises, the rest as the script gives them;
 * - /dev/rgn: _IO('R', 1) resets the regions.
 *
 * The AE statistics are 8-bit luma, linear (before gamma), of the frame
 * before: a 32-bin histogram at 0x40033080 (bins of 8 levels; its total is
 * the pixels counted) and 7x7 zone sums from 0x40033100, rows in pairs
 * 0x40 apart, the second of a pair 0x1c after the first. Zone total over
 * histogram total is the mean.
 */
#define _DEFAULT_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

#include "ae.h"
#include "xmisp.h"
#include "xmisp_priv.h"

#define ISP_WRITE 0x101
#define ISP_READ 0x103
#define I2C_WRITE 1
#define I2C_READ 3
#define MMZ_RESET _IO('M', 0)
#define MMZ_ALLOC 0xC0284D01 /* _IOWR('M', 1, 40 bytes) */
#define VI_INIT _IO('V', 6)
#define RGN_RESET _IO('R', 1)

#define SYSCTL_BASE 0x20000000u
#define SYSCTL_SIZE 0x1000u
#define DDR_BASE 0x80000000u
#define NR_3D_CTRL 0x40040000u
#define NR_3D_STOP_MS 80 /* two frames at 25 fps */

#define AE_HIST 0x40033080u
#define AE_HIST_BINS 32
#define AE_ZONES 0x40033100u
#define AE_ZONE_ROWS 7
#define AE_ZONE_COLS 7

/* The mean the vendor daemon's AE holds on these statistics, in daylight
 * and indoors alike (JX-H62 traces), and the steps it takes: a change
 * every second frame. */
static const struct xmisp_ae_cfg ae_xm510 = {
    .target = 37.5f,
    .tolerance = 0.08f,
    .interval = 2,
};

const struct xmisp_script *const xmisp_scripts[] = {&xmisp_h62_720p, NULL};

struct xmisp {
  int isp, i2c, vi, mmz, rgn, mem;
  volatile uint32_t *sysctl;
  uint32_t buf_phys, buf_size;
  const struct xmisp_script *script;
  struct xmisp_ae ae;
  int ae_on;
  float mean;
  uint32_t lines;
  float gain;
  char err[160];
};

struct isp_rec {
  uint32_t addr;
  uint32_t val;
};

struct i2c_rec {
  uint32_t dev;
  uint32_t reg;
  uint32_t reg_bytes;
  uint32_t data;
  uint32_t data_bytes;
};

struct mmz_rec {
  char name[32];
  uint32_t size;
  uint32_t phys;
};

static int fail(struct xmisp *s, const char *fmt, ...) {
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(s->err, sizeof(s->err), fmt, ap);
  va_end(ap);
  return -1;
}

static int isp_w(struct xmisp *s, uint32_t addr, uint32_t val) {
  struct isp_rec r = {addr, val};
  return ioctl(s->isp, ISP_WRITE, &r);
}

static int isp_r(struct xmisp *s, uint32_t addr, uint32_t *val) {
  struct isp_rec r = {addr, 0};
  if (ioctl(s->isp, ISP_READ, &r) < 0)
    return -1;
  *val = r.val;
  return 0;
}

static int i2c(struct xmisp *s, int cmd, uint32_t reg, uint32_t *data) {
  struct i2c_rec r = {s->script->i2c_dev, reg, 1, *data, 1};
  if (ioctl(s->i2c, cmd, &r) < 0)
    return -1;
  *data = r.data & 0xff;
  return 0;
}

int xmisp_sensor_write(struct xmisp *s, uint32_t reg, uint32_t val) {
  return i2c(s, I2C_WRITE, reg, &val);
}

int xmisp_sensor_read(struct xmisp *s, uint32_t reg, uint32_t *val) {
  *val = 0;
  return i2c(s, I2C_READ, reg, val);
}

static void delay_ms(unsigned ms) {
  struct timespec ts = {ms / 1000, (long)(ms % 1000) * 1000000};
  while (nanosleep(&ts, &ts) < 0 && errno == EINTR)
    ;
}

struct xmisp *xmisp_open(void) {
  struct xmisp *s = calloc(1, sizeof(*s));
  if (!s)
    return NULL;
  s->isp = open("/dev/isp", O_RDWR | O_CLOEXEC);
  s->i2c = open("/dev/xm_i2c", O_RDONLY | O_CLOEXEC);
  s->vi = open("/dev/vi", O_RDONLY | O_CLOEXEC);
  s->mmz = open("/dev/mmz", O_RDONLY | O_CLOEXEC);
  s->rgn = open("/dev/rgn", O_RDONLY | O_CLOEXEC);
  s->mem = open("/dev/mem", O_RDWR | O_SYNC | O_CLOEXEC);
  if (s->isp < 0 || s->i2c < 0 || s->vi < 0 || s->mmz < 0 || s->rgn < 0 ||
      s->mem < 0) {
    const int err = errno;
    xmisp_close(s);
    errno = err;
    return NULL;
  }
  void *p = mmap(NULL, SYSCTL_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, s->mem,
                 SYSCTL_BASE);
  if (p == MAP_FAILED) {
    const int err = errno;
    xmisp_close(s);
    errno = err;
    return NULL;
  }
  s->sysctl = p;
  return s;
}

static int has_expect(const struct xmisp_op *op) {
  for (; op->type != XMISP_END; op++)
    if (op->type == XMISP_I2C_EXPECT)
      return 1;
  return 0;
}

/* Runs a list of steps. 1 if this isn't the script's sensor: a chip ID
 * didn't match, or the sensor didn't answer before its ID was read. Another
 * script may fit. */
static int run(struct xmisp *s, const struct xmisp_op *op) {
  const uint32_t *args = s->script->args;
  int probing = has_expect(op);
  for (; op->type != XMISP_END; op++) {
    uint32_t v;
    struct mmz_rec m;
    switch (op->type) {
    case XMISP_DELAY_MS:
      delay_ms(op->a);
      break;
    case XMISP_MMZ_RESET:
      if (ioctl(s->mmz, MMZ_RESET) < 0)
        return fail(s, "MMZ reset: %s", strerror(errno));
      s->buf_phys = 0;
      break;
    case XMISP_VI_INIT:
      if (ioctl(s->vi, VI_INIT) < 0)
        return fail(s, "VI init: %s", strerror(errno));
      break;
    case XMISP_RGN_RESET:
      if (ioctl(s->rgn, RGN_RESET) < 0)
        return fail(s, "region reset: %s", strerror(errno));
      break;
    case XMISP_SYS_W:
      if (op->a - SYSCTL_BASE >= SYSCTL_SIZE)
        return fail(s, "system controller register %#x outside the page",
                    op->a);
      s->sysctl[(op->a - SYSCTL_BASE) / 4] = op->b;
      break;
    case XMISP_I2C_W:
      v = op->b;
      if (i2c(s, I2C_WRITE, op->a, &v) < 0) {
        fail(s, "sensor write %#x: %s", op->a, strerror(errno));
        return probing ? 1 : -1;
      }
      break;
    case XMISP_I2C_EXPECT:
      v = 0;
      if (i2c(s, I2C_READ, op->a, &v) < 0) {
        fail(s, "sensor read %#x: %s", op->a, strerror(errno));
        return 1;
      }
      if (v != op->b) {
        fail(s, "sensor register %#x is %#x, not %#x", op->a, v, op->b);
        return 1;
      }
      /* Identified once the last ID register matches. */
      if (op[1].type != XMISP_I2C_EXPECT)
        probing = 0;
      break;
    case XMISP_VI_IOW:
      if (ioctl(s->vi, op->a, (void *)&args[op->b + 1]) < 0)
        return fail(s, "VI ioctl %#x: %s", op->a, strerror(errno));
      break;
    case XMISP_ISP_BUF:
      memset(&m, 0, sizeof(m));
      strcpy(m.name, "Isp");
      m.size = op->a;
      /* The module returns 0 even when it refuses: no address, no block. */
      if (ioctl(s->mmz, MMZ_ALLOC, &m) < 0 || !m.phys)
        return fail(s, "MMZ block of %#x bytes: %s", op->a,
                    m.phys ? strerror(errno) : "not allocated");
      s->buf_phys = m.phys;
      s->buf_size = m.size;
      break;
    case XMISP_ISP_W:
      if (isp_w(s, op->a, op->b) < 0)
        return fail(s, "ISP write %#x: %s", op->a, strerror(errno));
      break;
    case XMISP_ISP_W_BUF:
      if (!s->buf_phys || op->b >= s->buf_size)
        return fail(s, "ISP buffer offset %#x without the block", op->b);
      if (isp_w(s, op->a, (s->buf_phys + op->b - DDR_BASE) / 8) < 0)
        return fail(s, "ISP write %#x: %s", op->a, strerror(errno));
      break;
    default:
      return fail(s, "unknown script step %u", op->type);
    }
  }
  return 0;
}

/* AE from the exposure the fixed state left, if the script's sensor has a
 * driver; a driver that can't read the sensor fails the start. */
static int ae_start(struct xmisp *s) {
  const struct xmisp_sensor *sn = s->script->sensor;
  s->ae_on = 0;
  if (!sn)
    return 0;
  if (sn->get_exposure(s, &s->lines, &s->gain) < 0)
    return fail(s, "sensor exposure read: %s", strerror(errno));
  struct xmisp_ae_cfg cfg = ae_xm510;
  cfg.lines_min = sn->lines_min;
  cfg.lines_max = sn->lines_max;
  cfg.gain_max = sn->gain_max;
  xmisp_ae_init(&s->ae, &cfg, s->lines, s->gain);
  s->ae_on = 1;
  return 0;
}

/* The mean luma of the last frame, from the AE statistics. */
static int ae_mean(struct xmisp *s, float *mean) {
  uint32_t v, pixels = 0, sum = 0;
  for (unsigned i = 0; i < AE_HIST_BINS; i++) {
    if (isp_r(s, AE_HIST + i * 4, &v) < 0)
      return fail(s, "AE statistics: %s", strerror(errno));
    pixels += v;
  }
  for (unsigned row = 0; row < AE_ZONE_ROWS; row++)
    for (unsigned col = 0; col < AE_ZONE_COLS; col++) {
      const uint32_t a =
          AE_ZONES + (row / 2) * 0x40 + (row % 2) * 0x1c + col * 4;
      if (isp_r(s, a, &v) < 0)
        return fail(s, "AE statistics: %s", strerror(errno));
      sum += v;
    }
  if (!pixels)
    return fail(s, "AE statistics: no pixels counted");
  *mean = (float)sum / (float)pixels;
  return 0;
}

int xmisp_frame(struct xmisp *s) {
  if (!s->ae_on)
    return 0;
  if (ae_mean(s, &s->mean) < 0)
    return -1;
  uint32_t lines;
  float gain;
  if (!xmisp_ae_update(&s->ae, s->mean, &lines, &gain))
    return 0;
  const struct xmisp_sensor *sn = s->script->sensor;
  if (sn->set_exposure(s, lines, &gain) < 0) {
    fail(s, "sensor exposure write: %s", strerror(errno));
    /* Part of it may have landed: AE goes on from what the sensor has. */
    if (sn->get_exposure(s, &s->lines, &s->gain) == 0)
      xmisp_ae_applied(&s->ae, s->lines, s->gain);
    return -1;
  }
  s->lines = lines;
  s->gain = gain;
  xmisp_ae_applied(&s->ae, lines, gain);
  return 1;
}

int xmisp_ae_enabled(const struct xmisp *s) { return s->ae_on; }

void xmisp_exposure(const struct xmisp *s, float *mean, uint32_t *lines,
                    float *gain) {
  *mean = s->mean;
  *lines = s->lines;
  *gain = s->gain;
}

/* 3D noise reduction off, then two frames' time for its DMA to stop. */
static void nr3d_off(struct xmisp *s) {
  isp_w(s, NR_3D_CTRL, 0);
  delay_ms(NR_3D_STOP_MS);
}

int xmisp_start(struct xmisp *s, const struct xmisp_script *const *scripts) {
  char why[sizeof(s->err)] = "";
  /* A daemon that died without its way out left 3D-NR writing to the "Isp"
   * block: stop it before the MMZ and VI resets hand that memory on. */
  nr3d_off(s);
  for (; *scripts; scripts++) {
    s->script = *scripts;
    int r = run(s, s->script->start);
    if (r > 0) {
      snprintf(why, sizeof(why), "%s", s->err);
      continue;
    }
    if (r == 0)
      r = run(s, s->script->fixed);
    if (r == 0)
      r = ae_start(s);
    if (r == 0)
      return 0;
    /* A start that failed part way may have 3D-NR on already. */
    nr3d_off(s);
    s->script = NULL;
    return -1;
  }
  s->script = NULL;
  return fail(s, "no sensor script fits (last: %s)", why[0] ? why : "none");
}

const struct xmisp_script *xmisp_script(const struct xmisp *s) {
  return s->script;
}

int xmisp_fd(const struct xmisp *s) { return s->isp; }

void xmisp_stop(struct xmisp *s) { isp_w(s, NR_3D_CTRL, 0); }

void xmisp_close(struct xmisp *s) {
  if (!s)
    return;
  if (s->sysctl)
    munmap((void *)s->sysctl, SYSCTL_SIZE);
  const int fds[] = {s->isp, s->i2c, s->vi, s->mmz, s->rgn, s->mem};
  for (unsigned i = 0; i < sizeof(fds) / sizeof(fds[0]); i++)
    if (fds[i] >= 0)
      close(fds[i]);
  free(s);
}

const char *xmisp_error(const struct xmisp *s) { return s->err; }
