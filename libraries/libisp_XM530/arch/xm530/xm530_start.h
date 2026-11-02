/* Start-up tables of the XM530/XM550 ISP: the writes the vendor library made
 * for one sensor and mode, generated from a trace by
 * tools/xm-isp-trace/extract530.py. */
#ifndef XM530_START_H
#define XM530_START_H

#include <stddef.h>

#include "xm_type.h"

/* the library's own, not exported */
#pragma GCC visibility push(hidden)

/* val is an offset into the "Isp" MMZ block, in 8-byte units */
#define XM530_MMZ 1
/* the end of a table */
#define XM530_END 2

struct xm530_reg {
  XM_U32 addr;
  XM_U32 val;
  XM_U32 flags;
};

/* an attribute as the vendor library's XM_MPI_ISP_Init set it, by the
 * setter's name (XM_MPI_ISP_Set<name>Attr) */
struct xm530_attr {
  const char *name;
  XM_U32 w[128];
};

/* 3D noise reduction where the gain has settled at iso: NR_3D_BASE + 0x14,
 * 0x50, 0x54, 0x60, 0x64, 0x68, 0x6c, 0x14c, 0x150, 0x154, 0x158 */
#define XM530_NR3D_REGS 11
struct xm530_nr3d {
  XM_U32 iso;
  XM_U32 r[XM530_NR3D_REGS];
};

/* The hue LUT's response to a colour strength s at an entry: with
 * a = |s - 0x80|, a - k1 below cut, a - k2 from it, not below 0, with the
 * sign of s - 0x80. Ends with colour 0xff. */
struct xm530_chroma {
  XM_U8 colour, entry, k1, cut, k2;
};
extern const struct xm530_chroma xm530_chroma[];

struct xm530_mode {
  const char *name;
  XM_U32 width, height;
  /* DIGITAL_GAIN for an ISP gain of 1x */
  XM_U32 dgain_1x;
  const struct xm530_reg *init; /* in XM_MPI_ISP_Init, before the 3A inits */
  const struct xm530_reg *post; /* in XM_MPI_ISP_Init, after them */
  const struct xm530_reg *run;  /* the first pass of XM_MPI_ISP_Run */
  const struct xm530_attr *attr;
  const struct xm530_nr3d *nr3d; /* by ISO, ending with iso 0 */
};

extern const struct xm530_reg xm530_imx335_init[], xm530_imx335_post[],
    xm530_imx335_run[];
extern const struct xm530_attr xm530_imx335_attr[];
extern const struct xm530_nr3d xm530_imx335_nr3d[];
extern const XM_U32 xm530_imx335_mmz_size;

#pragma GCC visibility pop

#endif
