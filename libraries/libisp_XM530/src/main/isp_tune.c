/* The picture tuning by gain, every few frames, from the attributes' auto
 * tables (16 entries, one an ISO octave from 100), as the vendor library's
 * Run loop does it. Fitted against the vendor library on the lab XM550
 * (IMX335) at eleven light levels, ISO 100 to 12800:
 *
 *  - each entry is interpolated linearly in ISO between its octaves;
 *  - 2D NR (NR_2D_BASE): t, rounded; then 3t, 128/t and 0x20 + 3t;
 *  - sharpening: Ud, D and Kd, rounded up, to 0x3007500c/508c/511c;
 *  - black level: rounded up, to R, G and B;
 *  - Cb/Cr gain: 0x100 + 6 (s - 0x80), s the saturation rounded up, at
 *    least 0x100 (with the CSC saturation 50);
 *  - dynamic defect pixels (DDPC_BASE + 8, + 0xc): 511 - 2s and 255 - s, s
 *    the threshold rounded down;
 *  - DRC (0x3000b020): the defog strength, rounded down; 0x3000b024 the
 *    largest of the 64 local tone mapping statistics at 0x3000b300;
 *  - the luma curve (YGAMMA, 65 nodes, once): (x - 12) 2140 / 2048, about
 *    255 (x - 12) / 244, which takes the black level's remains (0xc9 in 12
 *    bits is 12) out; all 65 nodes as the vendor library's.
 */
#include <math.h>

#include "isp_main.h"

#define REG_NR2D 0x30036000
#define REG_SHARPEN_UD 0x3007500c
#define REG_SHARPEN_D 0x3007508c
#define REG_SHARPEN_KD 0x3007511c
#define REG_BLACK 0x30004000
#define REG_DDPC 0x30035008
#define REG_DRC 0x3000b020
#define REG_LTM_BRIGHT 0x3000b300
#define REG_YGAMMA 0x30072000

/* tab[i] is for ISO 100 << i */
static double interp(const XM_U8 *tab8, const XM_U16 *tab16, XM_U32 iso) {
#define T(i) (tab8 ? tab8[i] : tab16[i])
  if (iso <= 100)
    return T(0);
  for (int i = 0; i < ISP_AUTO_STENGTH_NUM - 1; i++) {
    const XM_U32 lo = 100u << i, hi = 100u << (i + 1);
    if (iso <= hi)
      return T(i) +
             (double)((int)T(i + 1) - (int)T(i)) * (iso - lo) / (hi - lo);
  }
  return T(ISP_AUTO_STENGTH_NUM - 1);
#undef T
}

/* the auto value, scaled by the table's global strength (0x80 is 1) */
static XM_U32 strength(double v, XM_U8 global) {
  return (XM_U32)v * global / 0x80;
}

static void nr2d(XM_U32 t) {
  isp_write(REG_NR2D, t);
  isp_write(REG_NR2D + 0x04, 3 * t);
  isp_write(REG_NR2D + 0x08, t ? 128 / t : 0x80);
  isp_write(REG_NR2D + 0x10, 0x20 + 3 * t);
}

static void luma_curve(void) {
  for (int k = 0; k <= 64; k++) {
    const int x = 4 * k - 12;
    const int y = x < 0 ? 0 : (x * 2140 + 1024) >> 11;
    isp_write(REG_YGAMMA + 4 * k, y > 255 ? 255 : y);
  }
}

void isp_tune(XM_U32 iso, const struct isp_tune_attrs *a) {
  static int curve;
  if (!curve) {
    luma_curve();
    curve = 1;
  }

  if (a->nr.bEnable)
    nr2d(a->nr.enOpType == OP_TYPE_MANUAL
             ? a->nr.stManual.u8Thresh
             : strength(floor(interp(a->nr.stAuto.au8Thresh, NULL, iso) + 0.5),
                        a->nr.stAuto.u8GlobalSth));

  if (a->sharpen.bEnable) {
    const ISP_SHARPEN_AUTO_ATTR_S *s = &a->sharpen.stAuto;
    XM_U32 ud, d, kd;
    if (a->sharpen.enOpType == OP_TYPE_MANUAL) {
      ud = a->sharpen.stManual.u8SharpenUd;
      d = a->sharpen.stManual.u8SharpenD;
      kd = a->sharpen.stManual.u8SharpenKd;
    } else {
      ud = strength(ceil(interp(s->au8SharpenUd, NULL, iso)), s->u8GlobalSth);
      d = strength(ceil(interp(s->au8SharpenD, NULL, iso)), s->u8GlobalSth);
      kd = strength(ceil(interp(s->au8SharpenKd, NULL, iso)), s->u8GlobalSth);
    }
    isp_write(REG_SHARPEN_UD, ud);
    isp_write(REG_SHARPEN_D, d);
    isp_write(REG_SHARPEN_KD, kd);
  }

  const XM_U32 blc =
      a->black.enOpType == OP_TYPE_MANUAL
          ? a->black.stManual.u16Blc
          : (XM_U32)ceil(interp(NULL, a->black.stAuto.au16Blc, iso));
  for (int i = 0; i < 3; i++)
    isp_write(REG_BLACK + 4 * i, blc);

  /* black and white (no CSC saturation) is the CSC setter's */
  if (!(a->csc.bEnable && !a->csc.stManual.u8SatuVal)) {
    const XM_U32 s =
        a->saturation.enOpType == OP_TYPE_MANUAL
            ? a->saturation.stManual.u8Saturation
            : (XM_U32)ceil(interp(a->saturation.stAuto.au8Sat, NULL, iso));
    const XM_U32 cbcr = s > 0x80 ? 0x100 + 6 * (s - 0x80) : 0x100;
    isp_write(REG_CBCR, cbcr);
    isp_write(REG_CBCR + 4, cbcr);
  }

  const XM_U32 dpc =
      a->dpc.enOpType == OP_TYPE_MANUAL
          ? a->dpc.stManual.u8Sth
          : (XM_U32)floor(interp(a->dpc.stAuto.au8Sth, NULL, iso));
  if (dpc <= 0xff) {
    isp_write(REG_DDPC, 511 - 2 * dpc);
    isp_write(REG_DDPC + 4, 255 - dpc);
  }

  if (a->defog.bEnable) {
    isp_write(REG_DRC,
              a->defog.enOpType == OP_TYPE_MANUAL
                  ? a->defog.stManual.u8Strength
                  : (XM_U32)floor(interp(a->defog.stAuto.au8Sth, NULL, iso)));
    XM_U32 top = 0;
    for (int i = 0; i < 64; i++) {
      const XM_U32 v = isp_read(REG_LTM_BRIGHT + 4 * i);
      if (v > top)
        top = v;
    }
    isp_write(REG_DRC + 4, top);
  }
}
