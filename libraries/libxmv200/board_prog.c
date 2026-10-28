// SPDX-License-Identifier: MIT
/*
 * Runs a board program (board_prog.h). The MMZ blocks are allocated again
 * by name and size; words of later payloads that hold an address in a
 * block the vendor got (plainly, or as (phys - 0x80000000) >> 4 the way the
 * ISP and VPSS take addresses) are moved to ours.
 */
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include "board_prog.h"
#include "xmv200.h"

static const char *const devs[] = {"/dev/isp",  "/dev/vpss", "/dev/vb",
                                   "/dev/sys",  "/dev/mmz",  "/dev/venc_info",
                                   "/dev/h265", "/dev/vi",   "/dev/h264"};

struct reloc {
  uint32_t old, new, size;
};

static void relocate(uint8_t *p, uint32_t len, const struct reloc *r, int n) {
  for (uint32_t i = 0; i + 4 <= len; i += 4) {
    uint32_t w;

    memcpy(&w, p + i, 4);
    for (int k = 0; k < n; k++) {
      uint32_t o = r[k].old, nw = r[k].new, s = r[k].size;

      if (w >= 0x80000000u && w >= o && w < o + s)
        w = w - o + nw;
      else if (w >= 0x100000 && w < 0x8000000 && (w << 4) + 0x80000000u >= o &&
               (w << 4) + 0x80000000u < o + s)
        w = w - ((o - 0x80000000u) >> 4) + ((nw - 0x80000000u) >> 4);
      else
        continue;
      memcpy(p + i, &w, 4);
      break;
    }
  }
}

int xmv200_prog_run(const struct xmv200_prog *p) {
  static struct reloc reloc[64];
  static uint8_t buf[0x800];
  int fd[9] = {-1, -1, -1, -1, -1, -1, -1, -1, -1}, nreloc = 0, last_isp = -1;
  uint8_t *page = NULL;

  for (int i = 0; i < p->n_steps; i++) {
    const struct xmv200_prog_step *s = &p->steps[i];
    int r = 0;

    switch (s->type) {
    case XMV200_PROG_REG:
      r = xmv200_reg_write(s->a, s->b);
      break;
    case XMV200_PROG_I2C:
      xmv200_i2c_write(s->c, s->dev, s->a, 2, s->b, 1);
      break;
    case XMV200_PROG_MMZ: {
      struct xmv200_buf b;
      char name[33];
      uint32_t size;

      memcpy(name, p->blob + s->b, 32);
      name[32] = 0;
      memcpy(&size, p->blob + s->b + 32, 4);
      r = xmv200_mmz_alloc(&b, name, size, s->a == 0xc02c4d04);
      if (!r && s->d && nreloc < 64)
        reloc[nreloc++] = (struct reloc){s->d, b.phys, size};
      break;
    }
    case XMV200_PROG_IOC:
      if (fd[s->dev] < 0)
        fd[s->dev] = open(devs[s->dev], O_RDWR);
      if (s->c > sizeof(buf))
        return -1;
      memcpy(buf, p->blob + s->b, s->c);
      relocate(buf, s->c, reloc, nreloc);
      r = ioctl(fd[s->dev], s->a, s->c ? buf : NULL);
      if (s->dev == 0 && s->a == 0x400c490c && !r) {
        uint32_t pages;

        memcpy(&pages, p->blob + s->b + 4, 4);
        page = mmap(NULL, pages << 12, PROT_READ | PROT_WRITE, MAP_SHARED,
                    fd[0], 0);
        if (page == MAP_FAILED)
          return -1;
        memset(page, 0, pages << 12);
        last_isp = pages << 12;
      }
      break;
    case XMV200_PROG_PAGE:
      if (page && last_isp > 0)
        memcpy(page, p->page,
               p->page_size < (uint32_t)last_isp ? p->page_size : last_isp);
      break;
    }
    if (r)
      fprintf(stderr, "libxmv200: board step %d (%d %#x) failed\n", i,
              s->type, s->a);
  }
  return 0;
}

int xmv200_prog_i2c_last(const struct xmv200_prog *p, int bus, uint8_t dev,
                         uint32_t reg) {
  int v = -1;

  for (int i = 0; i < p->n_steps; i++) {
    const struct xmv200_prog_step *s = &p->steps[i];

    if (s->type == XMV200_PROG_I2C && (int)s->c == bus && s->dev == dev &&
        s->a == reg)
      v = (int)s->b;
  }
  return v;
}
