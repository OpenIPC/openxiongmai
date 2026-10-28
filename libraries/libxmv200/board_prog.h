/* SPDX-License-Identifier: MIT */
/*
 * A board's start-up as a program of the vendor stack's own steps, for the
 * chips whose ISP set-up libxmv200 doesn't build itself (XM535D1/D2, ...):
 * tools/xmv200-capture/gen_board.py writes one from a capture.
 */
#ifndef XMV200_BOARD_PROG_H
#define XMV200_BOARD_PROG_H

#include <stdint.h>

enum {
  XMV200_PROG_REG,  /* a: address, b: value (/dev/mem) */
  XMV200_PROG_I2C,  /* dev: 7-bit address, a: register, b: value (bus 0) */
  XMV200_PROG_IOC,  /* dev: device, a: command, b/c: payload in the blob */
  XMV200_PROG_MMZ,  /* a: command, b/c: the 44-byte request, d: where the
                     * vendor got it (for relocating later payloads) */
  XMV200_PROG_PAGE, /* fill isp.ko's shared pages just mapped */
};

struct xmv200_prog_step {
  uint8_t type, dev;
  uint32_t a, b, c, d;
};

struct xmv200_prog {
  const struct xmv200_prog_step *steps;
  int n_steps;
  const uint8_t *blob;
  const uint8_t *page;
  uint32_t page_size;
};

int xmv200_prog_run(const struct xmv200_prog *p);

#endif
