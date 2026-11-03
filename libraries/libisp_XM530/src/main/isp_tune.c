/* The picture tuning by gain, every few frames, from the attributes' auto
 * tables (16 entries, one an ISO octave from 100), as the vendor library's
 * Run loop does it. Fitted against the vendor library on the lab XM550
 * (IMX335) at eleven light levels, ISO 100 to 12800, and checked in
 * qemu-bench with the gain held for every sensor the sensor libraries
 * start:
 *
 *  - each entry is interpolated linearly in ISO between its octaves;
 *  - 2D NR (NR_2D_BASE): t, rounded half down; then 3t, 128/t (0x100 at
 *    t 0) and 0x20 + 3t;
 *  - sharpening: Ud, D and Kd, rounded up, to 0x3007500c/508c/511c; in the
 *    SharpenV2 algorithm (u8AlgChoice 1) Ud and D 16, Kd, the detail
 *    control and threshold, and two triples ((x - 8) / 2, x - 8, x) each
 *    of V2's Ud and D, rounded half down;
 *  - the sharpening curve (0x30075144, 7 nodes): 16, 32, 64, 96, 128, 160,
 *    160, each less min(48, 2 (t - 16)), t the 2D NR threshold, at least 0;
 *  - black level: rounded up, to R, G and B;
 *  - Cb/Cr gain: 0x100 + 6 (s - 0x80), s the saturation rounded up, at
 *    least 0x100 (with the CSC saturation 50);
 *  - dynamic defect pixels (DDPC_BASE + 8, + 0xc): 2b and b = 256 - s
 *    below s 128, 2b + 1 and b = 255 - s from it, s the threshold rounded
 *    down;
 *  - DRC (0x3000b020): the defog strength, rounded down; 0x3000b024 the
 *    largest of the 64 local tone mapping statistics at 0x3000b300;
 *  - 3D noise reduction: every register the vendor's loop sets, as it
 *    leaves them where the gain has settled, at the nearest ISO measured,
 *    from the sensor's schedule (arch/xm530: the IMX335's measured on the
 *    lab camera, every other sensor's in qemu-bench with the gain held).
 *    Its rule also follows the picture (a 64-bin luma histogram) and how
 *    the gain got there, so it is a measured schedule;
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
#define REG_SHARPEN_UD_V2 0x3007501c /* and 0x3007505c */
#define REG_SHARPEN_D_V2 0x3007509c  /* and 0x300750dc */
#define REG_SHARPEN_DETAIL 0x30075130
#define REG_SHARPEN_CURVE 0x30075144
#define REG_BLACK 0x30004000
#define REG_DDPC 0x30035008
#define REG_DRC 0x3000b020
#define REG_LTM_BRIGHT 0x3000b300
#define REG_YGAMMA 0x30072000
#define REG_NR3D 0x30040000

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

/* rounded half down, as the vendor rounds most of them */
static XM_U32 half_down(double v) { return (XM_U32)ceil(v - 0.5); }

static void nr2d(XM_U32 t) {
  isp_write(REG_NR2D, t);
  isp_write(REG_NR2D + 0x04, 3 * t);
  isp_write(REG_NR2D + 0x08, t ? 128 / t : 0x100);
  isp_write(REG_NR2D + 0x10, 0x20 + 3 * t);
  /* the sharpening curve eases off as the noise reduction rises */
  static const XM_U32 base[7] = {16, 32, 64, 96, 128, 160, 160};
  const XM_U32 d = t <= 16 ? 0 : t >= 40 ? 48 : 2 * (t - 16);
  for (int k = 0; k < 7; k++)
    isp_write(REG_SHARPEN_CURVE + 4 * k, base[k] > d ? base[k] - d : 0);
}

/* at two banks 0x40 apart */
static void sharpen_banks(XM_U32 reg, const XM_U32 v[3]) {
  for (int b = 0; b < 2; b++)
    for (int i = 0; i < 3; i++)
      isp_write(reg + 0x40 * b + 4 * i, v[i]);
}

/* (x - 8) / 2, x - 8 and x */
static void sharpen_triple(XM_U32 reg, XM_U32 x) {
  const XM_U32 v[3] = {x >= 8 ? (x - 8) / 2 : 0, x >= 8 ? x - 8 : 0, x};
  sharpen_banks(reg, v);
}

static void sharpen(XM_U32 iso, const struct isp_tune_attrs *a) {
  const ISP_SHARPENV2_ATTR_S *v2 = &a->sharpen2;
  if (v2->u8AlgChoice == 1 && v2->bEnable) {
    XM_U32 ud, d, kd, thr, ctrl;
    if (v2->enOpType == OP_TYPE_MANUAL) {
      ud = v2->stManual.u8SharpenUd;
      d = v2->stManual.u8SharpenD;
      kd = v2->stManual.u8SharpenKd;
      thr = v2->stManual.u8DetailThr;
      ctrl = v2->stManual.u8DetailCtrl;
    } else {
      const ISP_SHARPENV2_AUTO_ATTR_S *s = &v2->stAuto;
#define V2(tab) strength(half_down(interp(s->tab, NULL, iso)), s->u8GlobalSth)
      ud = V2(au8SharpenUd);
      d = V2(au8SharpenD);
      kd = V2(au8SharpenKd);
      thr = V2(au8DetailThr);
      ctrl = V2(au8DetailCtrl);
#undef V2
    }
    isp_write(REG_SHARPEN_UD, 16);
    isp_write(REG_SHARPEN_D, 16);
    isp_write(REG_SHARPEN_KD, kd);
    isp_write(REG_SHARPEN_DETAIL, ctrl);
    isp_write(REG_SHARPEN_DETAIL + 4, thr);
    sharpen_triple(REG_SHARPEN_UD_V2, ud);
    sharpen_triple(REG_SHARPEN_D_V2, d);
    return;
  }
  /* V2's own registers as every V1 sensor's start-up leaves them, for a
   * switch from V2 */
  static const XM_U32 v1[3] = {3, 8, 16};
  sharpen_banks(REG_SHARPEN_UD_V2, v1);
  sharpen_banks(REG_SHARPEN_D_V2, v1);
  isp_write(REG_SHARPEN_DETAIL, 0);
  isp_write(REG_SHARPEN_DETAIL + 4, 2);
  const ISP_SHARPEN_AUTO_ATTR_S *s = &a->sharpen.stAuto;
  XM_U32 ud, d, kd;
  if (!a->sharpen.bEnable) {
    ud = d = kd = 0;
  } else if (a->sharpen.enOpType == OP_TYPE_MANUAL) {
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

static void luma_curve(void) {
  for (int k = 0; k <= 64; k++) {
    const int x = 4 * k - 12;
    const int y = x < 0 ? 0 : (x * 2140 + 1024) >> 11;
    isp_write(REG_YGAMMA + 4 * k, y > 255 ? 255 : y);
  }
}

static const XM_U32 nr3d_regs[XM530_NR3D_REGS] = {
    0x14, 0x40, 0x44, 0x48,  0x4c,  0x50,  0x54,  0x58,  0x5c,  0x60,
    0x64, 0x68, 0x6c, 0x130, 0x134, 0x138, 0x14c, 0x150, 0x154, 0x158};

/* The sensor's schedule (the IMX335's for one not traced), the nearest ISO
 * in log; written every pass, so a failed write is put right on the next.
 * But 0x134 and 0x138, which step at each ISO octave: min(0x40, 0x10 +
 * 3 << k), k the octave above 100, and 0x40 less half that, rounded up, at
 * every point of every schedule, computed so the step is at its ISO. */
static void nr3d(XM_U32 iso) {
  const struct xm530_nr3d *t = xm530_imx335_nr3d, *best = NULL;
  double gap = 0;
  const SENSOR_ID id = isp_sensor_id(); /* one snapshot a lookup */
  for (const struct xm530_nr3d_sensor *s = xm530_nr3d_sensors; s->id; s++)
    if ((SENSOR_ID)s->id == id)
      t = s->t;
  for (const struct xm530_nr3d *n = t; n->iso; n++) {
    const double g = fabs(log((double)iso / n->iso));
    if (!best || g < gap)
      best = n, gap = g;
  }
  if (best)
    for (int i = 0; i < XM530_NR3D_REGS; i++)
      if (nr3d_regs[i] != 0x134 && nr3d_regs[i] != 0x138)
        isp_write(REG_NR3D + nr3d_regs[i], best->r[i]);
  XM_U32 k = 0;
  while (k < 15 && iso >= 200u << k)
    k++;
  const XM_U32 s = 0x10 + (3u << (k > 4 ? 4 : k)); /* 0x40 from k 4 */
  isp_write(REG_NR3D + 0x134, s);
  isp_write(REG_NR3D + 0x138, 0x40 - (s + 1) / 2);
}

void isp_tune(XM_U32 iso, const struct isp_tune_attrs *a) {
  static int curve;
  if (!curve) {
    luma_curve();
    curve = 1;
  }
  nr3d(iso);

  /* a disabled block gets strength 0 */
  if (!a->nr.bEnable)
    nr2d(0);
  else
    nr2d(a->nr.enOpType == OP_TYPE_MANUAL
             ? a->nr.stManual.u8Thresh
             : strength(half_down(interp(a->nr.stAuto.au8Thresh, NULL, iso)),
                        a->nr.stAuto.u8GlobalSth));
  sharpen(iso, a);

  const XM_U32 blc =
      a->black.enOpType == OP_TYPE_MANUAL
          ? a->black.stManual.u16Blc
          : (XM_U32)ceil(interp(NULL, a->black.stAuto.au16Blc, iso));
  for (int i = 0; i < 3; i++)
    isp_write(REG_BLACK + 4 * i, blc);

  /* No CSC saturation is black and white. The CSC setter writes 0 at once;
   * writing it here as well puts right a pass that raced with it. */
  {
    const XM_U32 s =
        a->saturation.enOpType == OP_TYPE_MANUAL
            ? a->saturation.stManual.u8Saturation
            : (XM_U32)ceil(interp(a->saturation.stAuto.au8Sat, NULL, iso));
    XM_U32 cbcr = s > 0x80 ? 0x100 + 6 * (s - 0x80) : 0x100;
    if (a->csc.bEnable && !a->csc.stManual.u8SatuVal)
      cbcr = 0;
    isp_write(REG_CBCR, cbcr);
    isp_write(REG_CBCR + 4, cbcr);
  }

  const XM_U32 dpc =
      a->dpc.enOpType == OP_TYPE_MANUAL
          ? a->dpc.stManual.u8Sth
          : (XM_U32)floor(interp(a->dpc.stAuto.au8Sth, NULL, iso));
  if (dpc <= 0xff) {
    const XM_U32 b = dpc < 128 ? 256 - dpc : 255 - dpc;
    isp_write(REG_DDPC, 2 * b + (dpc >= 128));
    isp_write(REG_DDPC + 4, b);
  }

  if (!a->defog.bEnable) {
    isp_write(REG_DRC, 0);
  } else {
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
