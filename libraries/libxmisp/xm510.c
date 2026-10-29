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

#include "xmisp.h"

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

const struct xmisp_script *const xmisp_scripts[] = {&xmisp_h62_720p, NULL};

struct xmisp {
  int isp, i2c, vi, mmz, rgn, mem;
  volatile uint32_t *sysctl;
  uint32_t buf_phys, buf_size;
  const struct xmisp_script *script;
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

static int i2c(struct xmisp *s, int cmd, uint32_t reg, uint32_t *data) {
  struct i2c_rec r = {s->script->i2c_dev, reg, 1, *data, 1};
  if (ioctl(s->i2c, cmd, &r) < 0)
    return -1;
  *data = r.data & 0xff;
  return 0;
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

/* Runs a list of steps. 1 if a chip ID didn't match: another script may
 * fit. */
static int run(struct xmisp *s, const struct xmisp_op *op) {
  const uint32_t *args = s->script->args;
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
      if (i2c(s, I2C_WRITE, op->a, &v) < 0)
        return fail(s, "sensor write %#x: %s", op->a, strerror(errno));
      break;
    case XMISP_I2C_EXPECT:
      v = 0;
      if (i2c(s, I2C_READ, op->a, &v) < 0)
        return fail(s, "sensor read %#x: %s", op->a, strerror(errno));
      if (v != op->b) {
        fail(s, "sensor register %#x is %#x, not %#x", op->a, v, op->b);
        return 1;
      }
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

int xmisp_start(struct xmisp *s, const struct xmisp_script *const *scripts) {
  char why[sizeof(s->err)] = "";
  for (; *scripts; scripts++) {
    s->script = *scripts;
    const int r = run(s, s->script->start);
    if (r < 0) {
      s->script = NULL;
      return -1;
    }
    if (r > 0) {
      snprintf(why, sizeof(why), "%s", s->err);
      continue;
    }
    if (run(s, s->script->fixed) != 0) {
      s->script = NULL;
      return -1;
    }
    return 0;
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
