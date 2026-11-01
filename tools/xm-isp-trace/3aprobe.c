// SPDX-License-Identifier: MIT
/*
 * 3aprobe.so: an LD_PRELOAD probe for the XM530/XM550 ISP library,
 * libisp_XM530.so, inside majestic. The library takes its AE and AWB from
 * the 3A side during XM_MPI_ISP_MemInit, as two tables of four functions
 * (init, run, ctrl, exit) it exports as gAlgAeFun and gAlgAwbFun. Once
 * MemInit returns, the probe puts a logging wrapper in each slot, so every
 * call the library makes into the 3A is logged with its four arguments and
 * the words at each that points at readable user memory (and, one level
 * down, at each pointer among the first 16 of them), before and after,
 * then passed on. Memory is read through a pipe, so an argument that isn't
 * a pointer is skipped, not a fault in the camera process.
 *
 *   PROBE_LOG    the output (default /tmp/3aprobe.txt): opened with
 *                O_NOFOLLOW and used only if it is a regular file of the
 *                process's user
 *   PROBE_WORDS  words dumped at each pointer argument (default 48)
 *   PROBE_EVERY  after the first three calls of a function, log one in
 *                this many (default 100)
 *
 * The library's own calls aren't wrapped here: the tracer (isptrace.so)
 * does that. Use one or the other: both stand in for XM_MPI_ISP_MemInit.
 */
#define _GNU_SOURCE
#include <dlfcn.h>
#include <errno.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <unistd.h>

/* Five arguments: ae_run takes five (the fifth, s32Rsv, on the stack), and
 * the others ignore what they don't take. */
typedef uint32_t (*fn5)(uint32_t, uint32_t, uint32_t, uint32_t, uint32_t);

#define NFN 8
#define FOLLOW 16 /* words dumped at a pointer found in a dumped object */

static fn5 orig[NFN];
static unsigned count[NFN];
static pthread_mutex_t lock = PTHREAD_MUTEX_INITIALIZER;
static const char *const name[NFN] = {"ae_init",  "ae_run",  "ae_ctrl",
                                      "ae_exit",  "awb_init", "awb_run",
                                      "awb_ctrl", "awb_exit"};
static int probe[2] = {-1, -1}; /* a pipe: a write from a bad address fails */

static long env(const char *k, long def) {
  const char *v = getenv(k);
  return v ? strtol(v, NULL, 0) : def;
}

/* Copies n bytes from address a if all of it is readable, without
 * touching it directly: the kernel checks the range of a pipe write. */
static int safe_copy(void *dst, uint32_t a, size_t n) {
  if (a < 0x10000 || a >= 0xc0000000u || probe[0] < 0)
    return 0;
  const char *p = (const char *)(uintptr_t)a;
  char *d = dst;
  while (n) {
    const size_t k = n > 512 ? 512 : n;
    if (write(probe[1], p, k) != (ssize_t)k)
      return 0;
    if (read(probe[0], d, k) != (ssize_t)k)
      return 0;
    p += k, d += k, n -= k;
  }
  return 1;
}

static void dump_words(FILE *f, const uint32_t *w, long n) {
  for (long i = 0; i < n; i++)
    fprintf(f, "%s%08x", i % 8 ? " " : "\n      ", w[i]);
  fprintf(f, "\n");
}

/* The words at a, then (one level) at each word of them that points at
 * readable memory: AWB's result holds its gains and matrix that way. */
static void dump(FILE *f, const char *tag, int i, uint32_t a, long words) {
  static uint32_t w[1024], v[FOLLOW];
  if (words > 1024)
    words = 1024;
  if (!safe_copy(w, a, (size_t)words * 4))
    return;
  fprintf(f, "    %s%d %08x:", tag, i, a);
  dump_words(f, w, words);
  for (long k = 0; k < words && k < 16; k++)
    if (w[k] != a && safe_copy(v, w[k], sizeof(v))) {
      fprintf(f, "      [%ld] -> %08x:", k, w[k]);
      dump_words(f, v, FOLLOW);
    }
}

/* The log: a regular file of ours, never through a symlink. */
static FILE *log_open(void) {
  const char *path = getenv("PROBE_LOG") ? getenv("PROBE_LOG") : "/tmp/3aprobe.txt";
  const int fd = open(path, O_WRONLY | O_APPEND | O_CREAT | O_NOFOLLOW | O_CLOEXEC,
                      0600);
  if (fd < 0)
    return NULL;
  struct stat st;
  if (fstat(fd, &st) < 0 || !S_ISREG(st.st_mode) || st.st_uid != geteuid()) {
    close(fd);
    return NULL;
  }
  FILE *f = fdopen(fd, "a");
  if (!f)
    close(fd);
  return f;
}

static uint32_t call(int k, uint32_t a0, uint32_t a1, uint32_t a2, uint32_t a3,
                     uint32_t a4) {
  const uint32_t a[4] = {a0, a1, a2, a3};
  const long words = env("PROBE_WORDS", 48), every = env("PROBE_EVERY", 100);
  pthread_mutex_lock(&lock);
  const unsigned n = ++count[k];
  FILE *f = n <= 3 || (every > 0 && n % every == 0) ? log_open() : NULL;
  if (f) {
    fprintf(f, "%s #%u (%08x, %08x, %08x, %08x, %08x)\n", name[k], n, a0, a1,
            a2, a3, a4);
    for (int i = 0; i < 4; i++)
      dump(f, "in", i, a[i], words);
  }
  pthread_mutex_unlock(&lock);
  const uint32_t r = orig[k](a0, a1, a2, a3, a4);
  if (f) {
    pthread_mutex_lock(&lock);
    for (int i = 0; i < 4; i++)
      dump(f, "out", i, a[i], words);
    fprintf(f, "  -> %08x\n", r);
    pthread_mutex_unlock(&lock);
    fclose(f);
  }
  return r;
}

#define WRAP(k)                                                                \
  static uint32_t wrap##k(uint32_t a, uint32_t b, uint32_t c, uint32_t d,      \
                          uint32_t e) {                                        \
    return call(k, a, b, c, d, e);                                             \
  }
WRAP(0) WRAP(1) WRAP(2) WRAP(3) WRAP(4) WRAP(5) WRAP(6) WRAP(7)
static const fn5 wrapper[NFN] = {wrap0, wrap1, wrap2, wrap3,
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
  if (probe[0] < 0 && pipe(probe) == 0) {
    fcntl(probe[0], F_SETFD, FD_CLOEXEC);
    fcntl(probe[1], F_SETFD, FD_CLOEXEC);
  }
  if (ae && awb && ae[1] && !orig[1])
    for (int i = 0; i < 4; i++) {
      orig[i] = (fn5)(uintptr_t)ae[i];
      ae[i] = (uint32_t)(uintptr_t)wrapper[i];
      orig[4 + i] = (fn5)(uintptr_t)awb[i];
      awb[i] = (uint32_t)(uintptr_t)wrapper[4 + i];
    }
  pthread_mutex_unlock(&lock);
  return r;
}
