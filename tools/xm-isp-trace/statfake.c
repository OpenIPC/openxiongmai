// SPDX-License-Identifier: MIT
/*
 * statfake.so: an LD_PRELOAD probe that holds the XM530/XM550 AE at a gain
 * of one's choosing, for tracing the ISP library's tuning by gain where
 * the picture is synthetic (qemu-bench, where the statistics read 0).
 *
 * Each AWB zone's R, G and B sums, as /dev/isp reads give them, become
 * STATFAKE_L x the ISO of the AE library's last result / 100: the picture
 * gets brighter with the gain and nothing else, so AE settles where
 * L x ISO meets its target, the exposure at its longest. The 64-bin luma
 * histogram (0x30073000) reads as every pixel in STATFAKE_BIN (default 24,
 * about the AE target). The ISO comes from wrapping the AE library's run
 * function, as 3aprobe.so does it (gAlgAeFun, filled by
 * XM_MPI_ISP_MemInit); it stacks with isptrace.so and 3aprobe.so.
 *   STATFAKE_L      the zone sum at ISO 100 (default 4000)
 *   STATFAKE_BIN    the histogram bin (default 24)
 *   STATFAKE_PIXELS the histogram's total (default 5080400)
 */
#define _GNU_SOURCE
#include <dlfcn.h>
#include <stdarg.h>
#include <stdint.h>
#include <stdlib.h>

typedef uint32_t (*fn5)(uint32_t, uint32_t, uint32_t, uint32_t, uint32_t);

static fn5 ae_run;
static volatile uint32_t iso = 100;

static long env(const char *k, long def) {
  const char *v = getenv(k);
  return v ? strtol(v, NULL, 0) : def;
}

/* ae_run(dev, handle, info, result, flag): the result's third word is the
 * ISO */
static uint32_t ae_run_wrap(uint32_t a, uint32_t b, uint32_t c, uint32_t d,
                            uint32_t e) {
  const uint32_t r = ae_run(a, b, c, d, e);
  if (d) {
    const uint32_t i = ((const uint32_t *)(uintptr_t)d)[2];
    if (i >= 100 && i < 1000000)
      iso = i;
  }
  return r;
}

int XM_MPI_ISP_MemInit(int dev) {
  static int (*real)(int);
  if (!real)
    real = (int (*)(int))dlsym(RTLD_NEXT, "XM_MPI_ISP_MemInit");
  if (!real)
    return -1;
  const int r = real(dev);
  uint32_t *ae = dlsym(RTLD_DEFAULT, "gAlgAeFun");
  if (ae && ae[1] && !ae_run) {
    ae_run = (fn5)(uintptr_t)ae[1];
    ae[1] = (uint32_t)(uintptr_t)ae_run_wrap;
  }
  return r;
}

/* /dev/isp reads: _IOW('I', 2 or 3, 12) on {addr, val, arg} */
#define ISP_READ2 0x400c4902u
#define ISP_READ3 0x400c4903u

int ioctl(int fd, unsigned long req, ...) {
  static int (*real)(int, unsigned long, void *);
  va_list ap;
  va_start(ap, req);
  void *arg = va_arg(ap, void *);
  va_end(ap);
  if (!real)
    real = (int (*)(int, unsigned long, void *))dlsym(RTLD_NEXT, "ioctl");
  const int r = real(fd, req, arg);
  if (r < 0 || !arg || (req != ISP_READ2 && req != ISP_READ3))
    return r;
  uint32_t *p = arg;
  const uint32_t a = p[0];
  if (a >= 0x30011148 && a <= 0x30011150) {
    /* AWB SRAM R, G, B of the zone the port is at */
    p[1] = (uint32_t)((uint64_t)env("STATFAKE_L", 4000) * iso / 100);
  } else if (a >= 0x30073000 && a < 0x30073100) {
    const long bin = env("STATFAKE_BIN", 24);
    p[1] = (a - 0x30073000) / 4 == (uint32_t)bin
               ? (uint32_t)env("STATFAKE_PIXELS", 5080400)
               : 0;
  }
  return r;
}
