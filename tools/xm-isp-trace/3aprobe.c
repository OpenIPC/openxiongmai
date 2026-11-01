// SPDX-License-Identifier: MIT
/*
 * 3aprobe.so: an LD_PRELOAD probe for the XM530/XM550 ISP library,
 * libisp_XM530.so, inside majestic. The library takes its AE and AWB from
 * the 3A side during XM_MPI_ISP_MemInit, as two tables of four functions
 * (init, run, ctrl, exit) it exports as gAlgAeFun and gAlgAwbFun. Once
 * MemInit returns, the probe puts a logging wrapper in each slot, so every
 * call the library makes into the 3A is logged with its four arguments and
 * the words at each that points at user memory, before and after, then
 * passed on.
 *
 *   PROBE_LOG    the output (default /tmp/3aprobe.txt)
 *   PROBE_WORDS  words dumped at each pointer argument (default 48)
 *   PROBE_EVERY  after the first three calls of a function, log one in
 *                this many (default 100)
 *
 * The library's own calls aren't wrapped here: the tracer (isptrace.so)
 * does that. Use one or the other: both stand in for XM_MPI_ISP_MemInit.
 */
#define _GNU_SOURCE
#include <dlfcn.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>

typedef uint32_t (*fn4)(uint32_t, uint32_t, uint32_t, uint32_t);

static fn4 orig[8];
static unsigned count[8];
static pthread_mutex_t lock = PTHREAD_MUTEX_INITIALIZER;
static const char *const name[8] = {"ae_init",  "ae_run",  "ae_ctrl",
                                    "ae_exit",  "awb_init", "awb_run",
                                    "awb_ctrl", "awb_exit"};

static long env(const char *k, long def) {
  const char *v = getenv(k);
  return v ? strtol(v, NULL, 0) : def;
}

static void dump(FILE *f, const char *tag, int i, uint32_t a, long words) {
  if (a < 0x10000 || a >= 0xc0000000u)
    return;
  const uint32_t *p = (const uint32_t *)(uintptr_t)a;
  fprintf(f, "    %s%d %08x:", tag, i, a);
  for (long w = 0; w < words; w++)
    fprintf(f, "%s%08x", w % 8 ? " " : "\n      ", p[w]);
  fprintf(f, "\n");
}

static uint32_t call(int k, uint32_t a0, uint32_t a1, uint32_t a2,
                     uint32_t a3) {
  const uint32_t a[4] = {a0, a1, a2, a3};
  const long words = env("PROBE_WORDS", 48), every = env("PROBE_EVERY", 100);
  pthread_mutex_lock(&lock);
  const unsigned n = ++count[k];
  FILE *f = n <= 3 || (every > 0 && n % every == 0)
                ? fopen(getenv("PROBE_LOG") ? getenv("PROBE_LOG")
                                            : "/tmp/3aprobe.txt",
                        "a")
                : NULL;
  if (f) {
    fprintf(f, "%s #%u (%08x, %08x, %08x, %08x)\n", name[k], n, a0, a1, a2,
            a3);
    for (int i = 0; i < 4; i++)
      dump(f, "in", i, a[i], words);
  }
  pthread_mutex_unlock(&lock);
  const uint32_t r = orig[k](a0, a1, a2, a3);
  if (f) {
    for (int i = 0; i < 4; i++)
      dump(f, "out", i, a[i], words);
    fprintf(f, "  -> %08x\n", r);
    fclose(f);
  }
  return r;
}

#define WRAP(k)                                                                \
  static uint32_t wrap##k(uint32_t a, uint32_t b, uint32_t c, uint32_t d) {    \
    return call(k, a, b, c, d);                                                \
  }
WRAP(0) WRAP(1) WRAP(2) WRAP(3) WRAP(4) WRAP(5) WRAP(6) WRAP(7)
static const fn4 wrapper[8] = {wrap0, wrap1, wrap2, wrap3,
                               wrap4, wrap5, wrap6, wrap7};

int XM_MPI_ISP_MemInit(int dev) {
  static int (*real)(int);
  if (!real)
    real = (int (*)(int))dlsym(RTLD_NEXT, "XM_MPI_ISP_MemInit");
  if (!real)
    return -1;
  const int r = real(dev);
  uint32_t *ae = dlsym(RTLD_DEFAULT, "gAlgAeFun");
  uint32_t *awb = dlsym(RTLD_DEFAULT, "gAlgAwbFun");
  pthread_mutex_lock(&lock);
  if (ae && awb && ae[1] && !orig[1])
    for (int i = 0; i < 4; i++) {
      orig[i] = (fn4)(uintptr_t)ae[i];
      ae[i] = (uint32_t)(uintptr_t)wrapper[i];
      orig[4 + i] = (fn4)(uintptr_t)awb[i];
      awb[i] = (uint32_t)(uintptr_t)wrapper[4 + i];
    }
  pthread_mutex_unlock(&lock);
  return r;
}
