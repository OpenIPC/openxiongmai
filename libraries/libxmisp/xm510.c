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
 *
 * The AWB statistics are of the raw picture times candidate gains: four
 * channels, each with its own R and B gains (0x40010000.. and
 * 0x40010010.., 0x100 = 1, G 1), none of which reach the picture. For the
 * pixels that look grey under a channel's gains (inside a window around
 * Cb = Cr = 0 and a luma range), each channel gives four regions of
 * {count, Cb sum, Cr sum} from 0x400140c0, 0x40 apart; summed, they are the
 * grey pixels' count and mean Cb and Cr (BT.601, 9-bit signed coefficients
 * at 0x4001300c..). The picture's gains are AWB_FRONTGAIN R, G, B at
 * 0x40015000.
 *
 * Colour: the correction matrix (CSM) is double-buffered, banks at
 * 0x40061040 and 0x40061080 (per row an offset, then three coefficients,
 * 0x100 = 1, signed 32-bit), switched by a write to 0x40061000. After the
 * RGB gamma and RGB to YCbCr, Cb and Cr are scaled by 0x40076004/08,
 * 0x100 = 1: a blue gain change through an identity matrix moved only B at
 * 0x100, and leaked into R and G at 0x200. The vendor daemon moves both with
 * the light (Cb/Cr 0x20e-0x250 in the lab traces, 80-150 matrices a
 * trace): a 2-2.3x saturation boost on top of its matrix.
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
#include "awb.h"
#include "tune.h"
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

#define AWB_BACK_R 0x40010000u /* 4 channels */
#define AWB_BACK_B 0x40010010u
#define AWB_STAT 0x400140c0u  /* channel 1: 4 regions x {n, Cb, Cr} */
#define AWB_L45 0x40014048u   /* Cb + Cr limits: min << 8 | max, signed */
#define AWB_L135 0x4001404cu  /* Cb - Cr limits */
#define AWB_YTHD 0x40014054u  /* luma limits: max << 8 | min */
#define AWB_FRONT 0x40015000u /* R, G, B */
#define AWB_GAIN_ONE 0x100
#define CSM_BANK0 0x40061040u
#define CSM_BANK1 0x40061080u
#define CSM_SWITCH 0x40061000u
#define CB_GAIN 0x40076004u
#define CR_GAIN 0x40076008u
#define CBCR_ONE 0x100
#define AWB_GAIN_MAX 0xfff

/* The grey window, Cb ± Cr within ±38 (the vendor's) while far off, ±8
 * once near. Luma at least 0x80: dimmer pixels in a room of wood and beige
 * pass as grey under far too blue gains (lab colour checker, daylight:
 * 0x80 lands within 5% of the chart's grey, the vendor's 0x40 nowhere
 * near). Too few bright grey pixels, as in the first frames before AE has
 * the picture bright, or in a dark scene, and AWB holds its gains: falling
 * back to 0x40 there settles in exactly that trap. */
#define AWB_WINDOW_WIDE 0xda26
#define AWB_WINDOW_NARROW 0xf808
#define AWB_LUMA 0xdc80

/* Noise reduction and sharpening by gain. The vendor daemon moves these
 * with gain (traced under the lab's dimmable bulbs); the values here are
 * chosen by measurement on the JX-H62 at 16x gain (temporal and spatial
 * noise in the chart's grey patches against the chair mesh's detail):
 * - 2D NR: BASE and THDH set its strength; COEF and DELTA alone change
 *   nothing;
 * - 3D NR (YCTRL_ERR70): bits 3:0 are a strength, 0 off; 6 halves the noise
 *   at 16x, 10 and up cost detail for little more;
 * - sharpening amplifies the noise: halving it at 16x halves the noise too.
 * At 16x the high set takes temporal noise from 12.6 to 1.4 and keeps
 * detail at 2.5 (vendor's set: 0.8, 2.3); at 3x it would over-smooth
 * (detail 26 to 15), so the strength stays 0 up to 4x and reaches 1 at
 * 16x: at 9x it halves the noise (5.0 to 2.5) and keeps detail at 20 of
 * 26. The low set is the script's, read at start. */
#define NR_GAIN_LO 4.0f
#define NR_GAIN_HI 16.0f
#define NR_STEPS 8
#define NR_3D_STRENGTH 0x0fu /* of NR_3D_YCTRL_ERR70 */
static const uint32_t nr_regs[] = {
    0x40020000, /* NR_2D_BASE */
    0x40020004, /* NR_2D_THDH */
    0x40020008, /* NR_2D_COEF */
    0x40020014, /* NR_2D_DELTA_THD */
    0x400400d8, /* NR_3D_YCTRL_ERR70 */
    0x40075018, /* SHARPEN_KD */
    0x4007514c, /* SHARPEN_KM0 */
    0x40075174, /* SHARPEN_KH0 */
};
#define NR_REGS (sizeof(nr_regs) / sizeof(nr_regs[0]))
#define NR_3D 4 /* index of NR_3D_YCTRL_ERR70 */
static const uint32_t nr_high[NR_REGS] = {0x10, 0x2f, 0x82, 0x70,
                                          6,    0x26, 0x20, 0x17};

static const struct xmisp_awb_cfg awb_xm510 = {
    .points_min = 300,
    .near = 3.0f,
    .tolerance = 0.5f,
    .step = 40.0f,
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
  struct xmisp_awb awb;
  int awb_on;
  uint32_t front_g; /* the script's G gain, which AWB keeps */
  int awb_dirty;    /* a gain write failed part way */
  uint32_t window, ythd;
  uint32_t nr_low[NR_REGS]; /* the script's, read at start */
  float nr_strength;        /* as set; -1 before the first */
  int nr_on;
  uint32_t grey; /* the last measurement */
  float cb, cr;
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

static int ae_frame(struct xmisp *s) {
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

static uint32_t gain_reg(float gain, uint32_t one) {
  const float v = gain * (float)one + 0.5f;
  return v > AWB_GAIN_MAX ? AWB_GAIN_MAX : (uint32_t)v;
}

/* The statistics' candidate gains, all four channels, and the picture's. */
static int awb_set(struct xmisp *s, float r, float b) {
  for (unsigned i = 0; i < 4; i++)
    if (isp_w(s, AWB_BACK_R + i * 4, gain_reg(r, AWB_GAIN_ONE)) < 0 ||
        isp_w(s, AWB_BACK_B + i * 4, gain_reg(b, AWB_GAIN_ONE)) < 0)
      return fail(s, "AWB gains: %s", strerror(errno));
  if (isp_w(s, AWB_FRONT, gain_reg(r, s->front_g)) < 0 ||
      isp_w(s, AWB_FRONT + 8, gain_reg(b, s->front_g)) < 0)
    return fail(s, "AWB gains: %s", strerror(errno));
  return 0;
}

static int awb_gate(struct xmisp *s, uint32_t window, uint32_t ythd) {
  if (window != s->window &&
      (isp_w(s, AWB_L45, window) < 0 || isp_w(s, AWB_L135, window) < 0))
    return fail(s, "AWB window: %s", strerror(errno));
  s->window = window;
  if (ythd != s->ythd && isp_w(s, AWB_YTHD, ythd) < 0)
    return fail(s, "AWB luma range: %s", strerror(errno));
  s->ythd = ythd;
  return 0;
}

/* AWB from the fixed state's gains, if the script's sensor has a range. */
static int awb_start(struct xmisp *s) {
  const struct xmisp_sensor *sn = s->script->sensor;
  uint32_t r = 0, g = 0, b = 0;
  s->awb_on = 0;
  s->awb_dirty = 0;
  if (!sn || !sn->awb_r_max)
    return 0;
  if (isp_r(s, AWB_FRONT, &r) < 0 || isp_r(s, AWB_FRONT + 4, &g) < 0 ||
      isp_r(s, AWB_FRONT + 8, &b) < 0)
    return fail(s, "AWB gains read: %s", strerror(errno));
  if (!g)
    return fail(s, "AWB gains read: G is 0");
  /* The range must fit the registers: candidate gains in 0x100 units, the
   * picture's in units of its G. */
  const float top =
      (float)AWB_GAIN_MAX / (float)(g > AWB_GAIN_ONE ? g : AWB_GAIN_ONE);
  if (!(sn->awb_r_min > 0 && sn->awb_r_min <= sn->awb_r_max &&
        sn->awb_r_max <= top && sn->awb_b_min > 0 &&
        sn->awb_b_min <= sn->awb_b_max && sn->awb_b_max <= top))
    return fail(s, "sensor AWB range R %g-%g, B %g-%g: not within 0-%g",
                sn->awb_r_min, sn->awb_r_max, sn->awb_b_min, sn->awb_b_max,
                top);
  struct xmisp_awb_cfg cfg = awb_xm510;
  cfg.r_min = sn->awb_r_min;
  cfg.r_max = sn->awb_r_max;
  cfg.b_min = sn->awb_b_min;
  cfg.b_max = sn->awb_b_max;
  s->front_g = g;
  xmisp_awb_init(&s->awb, &cfg, (float)r / (float)g, (float)b / (float)g);
  s->window = s->ythd = 0;
  if (awb_set(s, s->awb.r, s->awb.b) < 0 ||
      awb_gate(s, AWB_WINDOW_WIDE, AWB_LUMA) < 0)
    return -1;
  s->awb_on = 1;
  return 0;
}

static int awb_measure(struct xmisp *s) {
  int32_t sum[3] = {0, 0, 0};
  for (unsigned i = 0; i < 12; i++) {
    uint32_t v;
    if (isp_r(s, AWB_STAT + i * 4, &v) < 0)
      return fail(s, "AWB statistics: %s", strerror(errno));
    sum[i % 3] += (int32_t)v;
  }
  s->grey = sum[0] > 0 ? (uint32_t)sum[0] : 0;
  s->cb = s->grey ? (float)sum[1] / (float)s->grey : 0;
  s->cr = s->grey ? (float)sum[2] / (float)s->grey : 0;
  return 0;
}

static int awb_frame(struct xmisp *s) {
  if (!s->awb_on)
    return 0;
  /* A write that failed part way left the candidate and picture gains
   * apart: set them all to the gains AWB holds before measuring again. */
  if (s->awb_dirty) {
    if (awb_set(s, s->awb.r, s->awb.b) < 0)
      return -1;
    s->awb_dirty = 0;
  }
  if (awb_measure(s) < 0)
    return -1;
  float r, b;
  const int change = xmisp_awb_update(&s->awb, s->grey, s->cb, s->cr, &r, &b);
  if (awb_gate(s,
               s->awb.window == XMISP_AWB_NARROW ? AWB_WINDOW_NARROW
                                                 : AWB_WINDOW_WIDE,
               s->ythd) < 0)
    return -1;
  if (!change)
    return 0;
  if (awb_set(s, r, b) < 0) {
    s->awb_dirty = 1;
    return -1;
  }
  xmisp_awb_applied(&s->awb, r, b);
  return 1;
}

static int nr_frame(struct xmisp *s);

/* Two failures' reasons as one, the first's kept from the copy taken. */
static void both(struct xmisp *s, const char *first) {
  char second[sizeof(s->err)];
  memcpy(second, s->err, sizeof(second));
  snprintf(s->err, sizeof(s->err), "%.78s; %.78s", first, second);
}

int xmisp_frame(struct xmisp *s) {
  int ae = ae_frame(s);
  /* Noise reduction follows the gain the sensor has even when AE failed:
   * a failed write is applied as read back. */
  char err[sizeof(s->err)];
  memcpy(err, s->err, sizeof(err));
  const int nr = nr_frame(s);
  if (nr < 0) {
    if (ae < 0)
      both(s, err);
    ae = -1;
  } else if (ae < 0) {
    memcpy(s->err, err, sizeof(err));
  } else {
    ae = ae > 0 || nr > 0;
  }
  memcpy(err, s->err, sizeof(err));
  memcpy(err, s->err, sizeof(err));
  const int awb = awb_frame(s);
  if (ae < 0 && awb < 0) {
    both(s, err);
  } else if (ae < 0) {
    memcpy(s->err, err, sizeof(err)); /* AE's reason, not overwritten */
  }
  if (ae < 0 || awb < 0)
    return -1;
  return ae > 0 || awb > 0;
}

int xmisp_awb_enabled(const struct xmisp *s) { return s->awb_on; }

void xmisp_white_balance(const struct xmisp *s, uint32_t *grey, float *cb,
                         float *cr, float *r, float *b) {
  *grey = s->grey;
  *cb = s->cb;
  *cr = s->cr;
  *r = s->awb.r;
  *b = s->awb.b;
}

/* The script's noise reduction and sharpening, as the low-gain end. */
static int nr_start(struct xmisp *s) {
  s->nr_on = 0;
  s->nr_strength = -1.0f;
  if (!s->ae_on)
    return 0;
  for (unsigned i = 0; i < NR_REGS; i++)
    if (isp_r(s, nr_regs[i], &s->nr_low[i]) < 0)
      return fail(s, "noise reduction read: %s", strerror(errno));
  s->nr_on = 1;
  /* For the gain the script left, so the strength is known from here. */
  return nr_frame(s) < 0 ? -1 : 0;
}

static int nr_frame(struct xmisp *s) {
  if (!s->nr_on)
    return 0;
  const float n = xmisp_strength(s->gain, NR_GAIN_LO, NR_GAIN_HI, NR_STEPS);
  if (n == s->nr_strength)
    return 0;
  for (unsigned i = 0; i < NR_REGS; i++) {
    uint32_t v;
    if (i == NR_3D) /* the strength bits only */
      v = (s->nr_low[i] & ~NR_3D_STRENGTH) |
          xmisp_lerp(s->nr_low[i] & NR_3D_STRENGTH, nr_high[i], n);
    else
      v = xmisp_lerp(s->nr_low[i], nr_high[i], n);
    if (isp_w(s, nr_regs[i], v) < 0)
      return fail(s, "noise reduction: %s", strerror(errno));
  }
  s->nr_strength = n;
  return 1;
}

float xmisp_nr_strength(const struct xmisp *s) {
  return s->nr_on && s->nr_strength > 0 ? s->nr_strength : 0.0f;
}

int xmisp_ae_enabled(const struct xmisp *s) { return s->ae_on; }

int xmisp_set_antiflicker(struct xmisp *s, unsigned hz) {
  if (hz && hz != 50 && hz != 60)
    return fail(s, "anti-flicker at %u Hz: 50 or 60, or 0 for off", hz);
  if (!s->ae_on || !s->script)
    return hz ? fail(s, "anti-flicker without auto-exposure") : 0;
  const uint32_t line_ns = s->script->sensor->line_ns;
  if (hz && !line_ns)
    return fail(s, "anti-flicker: the sensor's line time is unknown");
  /* Lights flicker at twice the mains frequency. */
  xmisp_ae_flicker(&s->ae,
                   hz ? 1e9f / (2.0f * (float)hz) / (float)line_ns : 0.0f);
  return 0;
}

void xmisp_exposure(const struct xmisp *s, float *mean, uint32_t *lines,
                    float *gain) {
  *mean = s->mean;
  *lines = s->lines;
  *gain = s->gain;
}

/* The sensor's colour matrix in both banks, and Cb/Cr at 1. */
static int color_start(struct xmisp *s) {
  const struct xmisp_sensor *sn = s->script->sensor;
  if (!sn || !sn->ccm)
    return 0;
  for (unsigned row = 0; row < 3; row++)
    if (sn->ccm[row * 3] + sn->ccm[row * 3 + 1] + sn->ccm[row * 3 + 2] != 256)
      return fail(s, "colour matrix row %u does not sum to 256", row);
  const uint32_t banks[] = {CSM_BANK0, CSM_BANK1};
  for (unsigned b = 0; b < 2; b++)
    for (unsigned row = 0; row < 3; row++) {
      const uint32_t a = banks[b] + row * 0x10;
      if (isp_w(s, a, 0) < 0)
        return fail(s, "colour matrix: %s", strerror(errno));
      for (unsigned col = 0; col < 3; col++)
        if (isp_w(s, a + 4 + col * 4,
                  (uint32_t)(int32_t)sn->ccm[row * 3 + col]) < 0)
          return fail(s, "colour matrix: %s", strerror(errno));
    }
  if (isp_w(s, CSM_SWITCH, 0xffffffff) < 0 || isp_w(s, CB_GAIN, CBCR_ONE) < 0 ||
      isp_w(s, CR_GAIN, CBCR_ONE) < 0)
    return fail(s, "colour matrix: %s", strerror(errno));
  return 0;
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
      r = awb_start(s);
    if (r == 0)
      r = color_start(s);
    if (r == 0)
      r = nr_start(s);
    if (r == 0)
      return 0;
    /* A start that failed part way may have 3D-NR on already, and the 3A
     * of the script it was running must not run on. */
    nr3d_off(s);
    s->ae_on = s->awb_on = s->nr_on = 0;
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
