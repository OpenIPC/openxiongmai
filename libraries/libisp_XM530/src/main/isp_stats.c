/* The AE library's luma statistics.
 *
 * The AWB statistics are 32 x 16 zones of R, G (both greens) and B sums. The
 * AWB library reads them itself, through Read_IspReg: AWB_SRAM_ADDR_RC = 1
 * selects zone 0, then each zone's R, G and B are read and AWB_SRAM_RD_EN = 1
 * moves to the next. What it reads is kept here.
 *
 * The AE library takes two grids made from them (ISP_AE_INFO_S, measured
 * against the vendor library):
 *  - stAeStat3, 16 x 32: each zone's BT.601 luma with the AWB gains applied,
 *    16 times the mean per pixel of a colour;
 *  - stAeStat2, 7 x 7: means of 4 x 2 of those, but the last column and row,
 *    which take the rest (8 columns, 4 rows).
 */
#include <string.h>

#include "isp_main.h"

static XM_U32 zone_rgb[AWB_ZONES][3];
static int zone;

void stats_port_write(XM_U32 addr, XM_U32 val) {
  if (addr == REG_AWB_PORT && val)
    zone = 0;
  else if (addr == REG_AWB_NEXT && val && zone < AWB_ZONES)
    zone++;
}

void stats_port_read(XM_U32 addr, XM_U32 val) {
  if (addr >= REG_AWB_R && addr <= REG_AWB_R + 8 && zone < AWB_ZONES)
    zone_rgb[zone][(addr - REG_AWB_R) / 4] = val;
}

/* gains: AWB R, G, B in 8.8; sites: the pixels of one colour in a zone */
void stats_grid(const XM_U32 gains[3], XM_U32 sites, ISP_AE_STAT_2_S *grid7,
                ISP_AE_STAT_3_S *grid16) {
  if (!sites)
    sites = 1;
  for (int i = 0; i < AWB_ZONES; i++) {
    const XM_U32 *c = zone_rgb[i];
    /* 0.299, 0.587 and 0.114 in 16.16; G is two pixels' worth */
    XM_U64 y = (19595ULL * c[0] * gains[0] >> 8) +
               (38470ULL * c[1] * gains[1] >> 9) +
               (7471ULL * c[2] * gains[2] >> 8);
    y = (y * 16 >> 16) / sites;
    grid16->au16MeteringWinV2[i / AWB_COLS][i % AWB_COLS] =
        y > 0xffff ? 0xffff : (XM_U16)y;
  }
  for (int r = 0; r < AE_ZONE_ROW; r++) {
    const int r0 = r * 2, rn = r < AE_ZONE_ROW - 1 ? 2 : AWB_ROWS - r0;
    for (int c = 0; c < AE_ZONE_COLUMN; c++) {
      const int c0 = c * 4, cn = c < AE_ZONE_COLUMN - 1 ? 4 : AWB_COLS - c0;
      XM_U32 sum = 0;
      for (int i = 0; i < rn; i++)
        for (int j = 0; j < cn; j++)
          sum += grid16->au16MeteringWinV2[r0 + i][c0 + j];
      grid7->au32MeteringWin[r][c] = sum / (XM_U32)(rn * cn);
    }
  }
}
