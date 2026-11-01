/* The library's own state and the calls between its parts. */
#ifndef ISP_MAIN_H
#define ISP_MAIN_H

#include "mpi_isp.h"
#include "xm530_start.h"

/* registers (xm530_isp.h names in comments) */
#define REG_MIPI 0x300c0000      /* MIPI receiver set-up */
#define REG_MIPI_CTRL 0x300c002c /* MIPI receiver: 0 stops, 4 runs */
#define REG_INPUT 0x300f0010     /* 1 with a MIPI sensor (no name) */
#define REG_VSYNC_INT 0x300f0040 /* frame interrupt enable */
#define REG_DGAIN 0x30005000     /* DIGITAL_GAIN */
#define REG_AWB_GAIN 0x30010000  /* AWB_BASE: R, G, B gains */
#define REG_AWB_CTRL 0x30010010  /* AWB statistics on */
#define REG_AWB_LIMIT 0x3001000c /* AWB statistics upper limit */
#define REG_AWB_HGRID 0x30011000 /* 33 column starts */
#define REG_AWB_VGRID 0x300110c0 /* 17 row starts */
#define REG_AWB_PORT 0x30011140  /* AWB_SRAM_ADDR_RC */
#define REG_AWB_NEXT 0x30011144  /* AWB_SRAM_RD_EN */
#define REG_AWB_R 0x30011148     /* AWB_R_INTG, then G, B */
#define REG_CSM 0x30060040       /* colour matrix: 3 rows of offset, R, G, B */
#define REG_GAMMA 0x30061000     /* GAMMA96_BASE: 97 nodes */
#define REG_CBCR 0x30076004      /* Cb gain, then Cr gain */
#define REG_NR3D_BUF 0x300400a0  /* 3D NR frame buffers (8) */

#define AWB_COLS 32
#define AWB_ROWS 16
#define AWB_ZONES (AWB_COLS * AWB_ROWS)

/* isp_reg.c */
int isp_open(void);
XM_U32 isp_read(XM_U32 addr);
void isp_write(XM_U32 addr, XM_U32 val);
int isp_wait_frame(void);
void isp_write_table(const struct xm530_reg *t, XM_U32 mmz8);

/* isp_stats.c: the AWB statistics, as the AWB library reads them through
 * Read_IspReg, and the AE library's luma grids from them */
void stats_port_write(XM_U32 addr, XM_U32 val);
void stats_port_read(XM_U32 addr, XM_U32 val);
void stats_grid(const XM_U32 gains[3], XM_U32 sites, ISP_AE_STAT_2_S *grid7,
                ISP_AE_STAT_3_S *grid16);

/* isp_3a.c */
int isp_3a_register(ISP_DEV dev);
int isp_3a_init(ISP_DEV dev, const ISP_PUB_ATTR_S *pub);
int isp_3a_start(void);

/* mpi_isp.c */
const ISP_PUB_ATTR_S *isp_pub(void);
const struct xm530_mode *xm530_mode_find(XM_U32 width, XM_U32 height);
extern const struct xm530_mode *isp_mode;

#endif
