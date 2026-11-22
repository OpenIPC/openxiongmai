/* The chroma and anti-false-colour setters, which take effect at once, as
 * the vendor library's do. Both measured against it on the lab XM550, every
 * input value swept (tools/xm-isp-trace/chromasweep.c).
 *
 * Chroma (0x30071000):
 *  - the offsets of R, G and B go to 0x30071210, 0x22c and 0x250 (those of
 *    Mg, Ye and Cy take no effect), with the three hue sectors' ranges;
 *  - the 65-entry hue LUT at 0x300710e4 is 0x80 but where a colour's
 *    strength reaches it (arch/xm530/xm530_chroma.c); where two colours'
 *    do, the one further from 0x80 takes the entry, the first on a tie.
 *    That is exact for one colour at a time (every strength of each
 *    tried: the LUT the same as the vendor library's), and with two
 *    colours away from 0x80 97% of the entries are;
 *  - disabled: the LUT at 0x80 and the offsets at 0.
 *
 * Anti-false-colour (0x30063000): +0x18 is 0 enabled, 1 disabled (and only
 * that is written); with strength s, +4 is 0x100 + 256 s / (255 - s) up to
 * 0x3ff, +8 0xff but 0xff - 4 (s - 191) (at least 3) from s 192 on, and
 * +0x14 1024 / (+8).
 */
#include "isp_main.h"

#define REG_CHROMA 0x30071000
#define REG_CHROMA_LUT 0x300710e4
#define CHROMA_LUT 65
#define REG_AFC 0x30063000

/* the hue sectors of the R, G and B offsets, at these addresses: start,
 * end and two more, then the offset, then two more */
static const XM_U32 sector_reg[3] = {0x30071200, 0x3007121c, 0x30071240};
static const XM_U32 sectors[3][7] = {{0x1aa, 0x2aa, 0x1ff, 0x255, 0, 3, 3},
                                     {0x400, 0x500, 0x455, 0x4ab, 0, 3, 3},
                                     {0x6e3, 0x7e3, 0x738, 0x78e, 0, 3, 3}};

void isp_chroma(const ISP_CHROMA_ATTR_S *a) {
  const XM_U16 ofst[3] = {a->u16OfstR, a->u16OfstG, a->u16OfstB};
  const XM_U8 sth[6] = {a->u8SthMg, a->u8SthR,  a->u8SthYe,
                        a->u8SthG,  a->u8SthCy, a->u8SthB};
  int lut[CHROMA_LUT];
  XM_U32 owner[CHROMA_LUT]; /* the strength's distance from 0x80, + 1 */
  for (int e = 0; e < CHROMA_LUT; e++)
    lut[e] = 0, owner[e] = 0;

  isp_write(REG_CHROMA, 1);
  if (a->bEnable)
    for (const struct xm530_chroma *t = xm530_chroma; t->colour != 0xff; t++) {
      const int s = sth[t->colour] - 0x80, dist = s < 0 ? -s : s;
      const int k = dist < t->cut ? t->k1 : t->k2;
      if (!s || dist < k || t->entry >= CHROMA_LUT)
        continue;
      if ((XM_U32)dist + 1 > owner[t->entry]) {
        owner[t->entry] = dist + 1;
        lut[t->entry] = s < 0 ? -(dist - k) : dist - k;
      }
    }
  for (int e = 0; e < CHROMA_LUT; e++)
    isp_write(REG_CHROMA_LUT + 4 * e, 0x80 + lut[e]);
  for (int i = 0; i < 3; i++) {
    const XM_U32 base = sector_reg[i];
    if (a->bEnable)
      for (int j = 0; j < 7; j++)
        if (j != 4)
          isp_write(base + 4 * j, sectors[i][j]);
    isp_write(base + 0x10, a->bEnable ? ofst[i] : 0);
  }
}

void isp_antifalsecolor(const ISP_ANTI_FALSECOLOR_S *a) {
  if (!a->bEnable) {
    isp_write(REG_AFC + 0x18, 1);
    return;
  }
  const XM_U32 s = a->u8Strength;
  const XM_U32 f = s < 255 ? s * 256 / (255 - s) : 0xffff;
  const XM_U32 r4 = 0x100 + f > 0x3ff ? 0x3ff : 0x100 + f;
  const int v = s < 192 ? 0xff : 0xff - 4 * ((int)s - 191);
  const XM_U32 r8 = v < 3 ? 3 : v;
  isp_write(REG_AFC + 0x04, r4);
  isp_write(REG_AFC + 0x08, r8);
  isp_write(REG_AFC + 0x14, 1024 / r8);
  isp_write(REG_AFC + 0x18, 0);
}
