// SPDX-License-Identifier: MIT
/*
 * Bring-up scaffolding: replays a recorded init sequence (registers, sensor
 * I2C, device ioctls, MMZ blocks) through libxmv200's primitives, so the
 * pipeline after it can be developed before every step has its own code.
 * Built only with REPLAY_DATA=<file> (the recording is not in this tree).
 */
#ifdef REPLAY_DATA
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include "xmv200.h"

enum { S_REG, S_I2C, S_IOC, S_MMZ };
struct step {
  int type, dev;
  uint32_t a, b, c, d;
};

#include REPLAY_DATA

static const char *const devs[] = {"/dev/isp", "/dev/vpss", "/dev/vb",
                                   "/dev/sys", "/dev/mmz"};

/* recorded MMZ block -> ours */
static struct {
  uint32_t old, new, size;
} reloc[64];
static int nreloc;

/* Words that hold an address of a recorded block: plain, or as
 * (phys - 0x80000000) >> 4 the way the ISP and VPSS take them. */
static void relocate(uint8_t *p, uint32_t len) {
  for (uint32_t i = 0; i + 4 <= len; i += 4) {
    uint32_t w;

    memcpy(&w, p + i, 4);
    for (int r = 0; r < nreloc; r++) {
      uint32_t o = reloc[r].old, n = reloc[r].new, s = reloc[r].size;

      if (w >= 0x80000000u && w >= o && w < o + s)
        w = w - o + n;
      else if (w >= 0x100000 && (w << 4) + 0x80000000u >= o &&
               (w << 4) + 0x80000000u < o + s)
        w = w - ((o - 0x80000000u) >> 4) + ((n - 0x80000000u) >> 4);
      else
        continue;
      memcpy(p + i, &w, 4);
      break;
    }
  }
}

int replay(int verbose) {
  static int fd[5] = {-1, -1, -1, -1, -1};
  static uint8_t buf[0x1000];
  int n = sizeof(steps) / sizeof(steps[0]);

  for (int i = 0; i < n; i++) {
    const struct step *s = &steps[i];
    int r = 0;

    switch (s->type) {
    case S_REG:
      r = xmv200_reg_write(s->a, s->b);
      break;
    case S_I2C:
      r = xmv200_i2c_write(0, s->dev, s->a, 2, s->b, 1);
      break;
    case S_MMZ: {
      struct xmv200_buf b;
      char name[33];
      uint32_t size;

      memcpy(name, blob + s->b, 32);
      name[32] = 0;
      memcpy(&size, blob + s->b + 32, 4);
      r = xmv200_mmz_alloc(&b, name, size, s->a == 0xc02c4d04);
      if (!r && s->d && nreloc < 64) {
        reloc[nreloc].old = s->d;
        reloc[nreloc].new = b.phys;
        reloc[nreloc].size = size;
        nreloc++;
      }
      if (verbose)
        printf("mmz %s %#x: %08x (was %08x)\n", name, size, b.phys, s->d);
      break;
    }
    case S_IOC: {
      void *arg = NULL;

      if (fd[s->dev] < 0)
        fd[s->dev] = open(devs[s->dev], O_RDWR);
      if (s->c) {
        memcpy(buf, blob + s->b, s->c);
        relocate(buf, s->c);
        arg = buf;
      }
      if (verbose) {
        printf("%d ioc %d %08x", i, s->dev, s->a);
        for (uint32_t k = 0; k < s->c && k < 16; k += 4) {
          uint32_t w;
          memcpy(&w, buf + k, 4);
          printf(" %08x", w);
        }
        printf("\n");
        fflush(stdout);
      }
      r = ioctl(fd[s->dev], s->a, arg);
      /* the ISP's shared parameter pages, mapped once their count is set */
      if (s->dev == 0 && s->a == 0x400c490c && !r) {
        uint32_t pages;

        memcpy(&pages, blob + s->b + 4, 4);
        void *m = mmap(NULL, pages << 12, PROT_READ | PROT_WRITE, MAP_SHARED,
                       fd[0], 0);
        if (m == MAP_FAILED)
          r = -1;
        else
          memset(m, 0, pages << 12);
      }
      break;
    }
    }
    if (r && (verbose || s->type != S_I2C))
      fprintf(stderr, "step %d (type %d %#x) failed: %d\n", i, s->type, s->a,
              r);
  }
  return 0;
}
#endif
