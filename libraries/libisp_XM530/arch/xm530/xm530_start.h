/* Start-up of the XM530/XM550 ISP for any sensor: the template the vendor
 * library's writes follow, generated from traces by
 * tools/xm-isp-trace/start530.py (xm530_start.c), the geometry it
 * computes from the picture size (xm530_geometry.c), and what the vendor
 * library has built in for some sensors. */
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
/* val is an enum xm530_geo: computed from the picture size */
#define XM530_GEO 4

struct xm530_reg {
  XM_U32 addr;
  XM_U32 val;
  XM_U32 flags;
};

/* What the vendor library writes for the picture size (xm530_geometry()):
 * - the 3D noise reduction's planes in the "Isp" block, as addresses in
 *   8-byte units from DDR: the frame's 1.6 bytes a pixel (whole 8-byte
 *   units a line), then the motion plane's 0.4, then the error plane's byte
 *   a pixel at half size, and their lines in 8-byte units;
 * - the local tone mapping grid: n = round(size / 32) - 1 nodes each way,
 *   2^20 / (n + 1) a step, and at each corner 2^20 over the product of the
 *   two edge cells' counts, the last row and column counting
 *   size - 31 (n + 1). */
enum xm530_geo {
  XM530_GEO_NR3D_MOTION,
  XM530_GEO_NR3D_ERR,
  XM530_GEO_NR3D_FRAME_LINE,
  XM530_GEO_NR3D_MOTION_LINE,
  XM530_GEO_NR3D_ERR_LINE,
  XM530_GEO_LTM_NX,
  XM530_GEO_LTM_NY,
  XM530_GEO_LTM_STEP_X,
  XM530_GEO_LTM_STEP_Y,
  XM530_GEO_LTM_TL,
  XM530_GEO_LTM_TR,
  XM530_GEO_LTM_BL,
  XM530_GEO_LTM_BR,
  XM530_GEO_NUM
};

/* The "Isp" block's planes for a w x h picture, in bytes */
struct xm530_planes {
  XM_U32 frame, motion, err, err_w, err_h;
};
void xm530_planes(XM_U32 w, XM_U32 h, struct xm530_planes *p);
/* the XM530_GEO values, the "Isp" block at mmz8 */
void xm530_geometry(XM_U32 w, XM_U32 h, XM_U32 mmz8, XM_U32 geo[XM530_GEO_NUM]);

/* an attribute as the vendor library's XM_MPI_ISP_Init set it, by the
 * setter's name (XM_MPI_ISP_Set<name>Attr) */
struct xm530_attr {
  const char *name;
  XM_U32 w[128];
};

/* bytes of an attribute the vendor library sets for one sensor */
struct xm530_patch {
  const char *name;
  XM_U16 offset, len;
  XM_U8 data[128];
};

/* what the vendor library has built in for a sensor, by the SENSOR_CHIP id
 * its library registers: registers written after each part, and
 * attribute bytes */
struct xm530_sensor {
  XM_U32 id;
  const struct xm530_reg *init, *post, *run;
  const struct xm530_patch *attr;
};

/* the template: in XM_MPI_ISP_Init, before the 3A inits (init) and after
 * them (post), and the first pass of XM_MPI_ISP_Run */
extern const struct xm530_reg xm530_init[], xm530_post[], xm530_run[];
extern const struct xm530_attr xm530_attr[];
/* ends with id 0 */
extern const struct xm530_sensor xm530_sensors[];

/* DIGITAL_GAIN for an ISP gain of 1x: the vendor library scales the AE
 * library's gain by 0x109 >> 8, for every sensor traced */
#define XM530_DGAIN_1X 0x109

/* 3D noise reduction where the gain has settled at iso: NR_3D_BASE + 0x14,
 * 0x50, 0x54, 0x60, 0x64, 0x68, 0x6c, 0x14c, 0x150, 0x154, 0x158 */
#define XM530_NR3D_REGS 11
struct xm530_nr3d {
  XM_U32 iso;
  XM_U32 r[XM530_NR3D_REGS];
};
/* measured on the IMX335; by ISO, ending with iso 0 */
extern const struct xm530_nr3d xm530_imx335_nr3d[];

/* The hue LUT's response to a colour strength s at an entry: with
 * a = |s - 0x80|, a - k1 below cut, a - k2 from it, not below 0, with the
 * sign of s - 0x80. Ends with colour 0xff. */
struct xm530_chroma {
  XM_U8 colour, entry, k1, cut, k2;
};
extern const struct xm530_chroma xm530_chroma[];

#pragma GCC visibility pop

#endif
