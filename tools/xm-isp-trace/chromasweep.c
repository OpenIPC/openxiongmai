// SPDX-License-Identifier: MIT
/*
 * chromasweep.so: an LD_PRELOAD probe for the XM530/XM550 ISP library,
 * libisp_XM530.so, inside majestic. 22 s after start (the ISP is up by
 * then) it sets each of the six colour strengths of XM_MPI_ISP_SetChromaAttr
 * in turn to every value, the others at 0x80 (neutral), and logs the hue
 * LUT the library wrote (65 entries at 0x300710e4) after each, read back
 * with XM_MPI_ISP_GetRegister: one line "colour strength e0 ... e64".
 * chroma530.py turns it into the library's table.
 * The log (SWEEP_LOG, default /tmp/chromasweep.txt) is a regular file of
 * the process's user, opened without following symlinks.
 */
#define _GNU_SOURCE
#include <dlfcn.h>
#include <fcntl.h>
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <unistd.h>

/* ISP_CHROMA_ATTR_S: bEnable, offsets Mg R Ye G Cy B, strengths likewise */
struct chroma {
  uint32_t enable;
  uint16_t offset[6];
  uint8_t strength[6];
  uint8_t pad[2];
};

static FILE *log_open(void) {
  const char *path =
      getenv("SWEEP_LOG") ? getenv("SWEEP_LOG") : "/tmp/chromasweep.txt";
  int fd =
      open(path, O_WRONLY | O_CREAT | O_TRUNC | O_NOFOLLOW | O_CLOEXEC, 0600);
  if (fd < 0)
    return NULL;
  struct stat st;
  if (fstat(fd, &st) < 0 || !S_ISREG(st.st_mode) || st.st_uid != geteuid()) {
    close(fd);
    return NULL;
  }
  FILE *f = fdopen(fd, "w");
  if (!f)
    close(fd);
  return f;
}

static void *sweep(void *arg) {
  (void)arg;
  sleep(22);
  int (*set)(struct chroma *) = dlsym(RTLD_DEFAULT, "XM_MPI_ISP_SetChromaAttr");
  int (*get_attr)(struct chroma *) =
      dlsym(RTLD_DEFAULT, "XM_MPI_ISP_GetChromaAttr");
  int (*get)(uint32_t, uint32_t *) =
      dlsym(RTLD_DEFAULT, "XM_MPI_ISP_GetRegister");
  FILE *f = log_open();
  struct chroma saved;
  if (!set || !get || !get_attr || !f || get_attr(&saved)) {
    if (f)
      fclose(f);
    return NULL;
  }
  const struct chroma neutral = {
      1, {0}, {0x80, 0x80, 0x80, 0x80, 0x80, 0x80}, {0}};
  for (int c = 0; c < 6; c++)
    for (int s = 0; s < 256; s++) {
      struct chroma a = neutral;
      a.strength[c] = (uint8_t)s;
      /* a failed call ends the sweep, so the log holds no made-up rows */
      if (set(&a))
        goto out;
      uint32_t v[65];
      for (int e = 0; e < 65; e++)
        if (get(0x300710e4 + 4 * e, &v[e]))
          goto out;
      fprintf(f, "%d %d", c, s);
      for (int e = 0; e < 65; e++)
        fprintf(f, " %u", v[e]);
      fprintf(f, "\n");
    }
out:
  set(&saved);
  fclose(f);
  return NULL;
}

__attribute__((constructor)) static void init(void) {
  pthread_t t;
  if (!pthread_create(&t, NULL, sweep, NULL))
    pthread_detach(t);
}
