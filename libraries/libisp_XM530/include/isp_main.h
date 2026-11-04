/* The library's own state and the calls between its parts. */
#ifndef ISP_MAIN_H
#define ISP_MAIN_H

#include "mpi_ae.h"
#include "mpi_awb.h"
#include "mpi_isp.h"
#include "xm530_start.h"

/* What follows is the library's own, not exported. */
#pragma GCC visibility push(hidden)

/* registers (xm530_isp.h names in comments) */
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
void *isp_shm(void); /* NULL without the shared block */
/* a start-up table: XM530_MMZ values relative to the "Isp" block at mmz8
 * (8-byte units from DDR), XM530_GEO ones from geo */
void isp_write_table(const struct xm530_reg *t, XM_U32 mmz8, const XM_U32 *geo);

/* isp_stats.c: the AWB statistics, as the AWB library reads them through
 * Read_IspReg, and the AE library's luma grids from them */
void stats_port_write(XM_U32 addr, XM_U32 val);
void stats_port_read(XM_U32 addr, XM_U32 val);
void stats_grid(const XM_U32 gains[3], XM_U32 sites, ISP_AE_STAT_2_S *grid7,
                ISP_AE_STAT_3_S *grid16);
XM_U32 stats_mean(const ISP_AE_STAT_2_S *grid7);
void stats_dump(const char *path);
/* the zones read by the ISP library itself, for the open AWB */
void stats_read_zones(void);
const XM_U32 (*stats_zones(void))[3];

/* isp_3a.c: whether an open 3A part is asked for, by env or system.key */
int open_3a_wanted(const char *env, const char *key);

/* isp_ae.c: the open AE, in place of majestic's when asked for and the
 * sensor library gave its callbacks; state is the AE state buffer */
int open_ae_init(XM_U8 fps, void *state);
int open_ae_active(void);
/* isp_awb.c: the open AWB, in place of majestic's when LIBISP_OPEN_AWB=1 on
 * a sensor whose calibration it reads; on zones read with
 * stats_read_zones(); 1 with gains and matrix to apply */
int open_awb_init(void);
int open_awb_active(void);
int open_awb_frame(XM_U32 sites, ISP_AWB_RESULT_GAIN_S *gain,
                   ISP_AWB_RESULT_CC_S *cc);
/* 0 when majestic's AE is to run the frame (manual or bypassed exposure) */
int open_ae_frame(const ISP_AE_INFO_S *info, void *state, ISP_AE_RESULT_S *res);

/* isp_tune.c: the attributes the tuning by gain follows */
struct isp_tune_attrs {
  ISP_2DNR_ATTR_S nr;
  ISP_SHARPEN_ATTR_S sharpen;
  ISP_SHARPENV2_ATTR_S sharpen2;
  ISP_BLACKLVL_ATTR_S black;
  ISP_SATURATION_ATTR_S saturation;
  ISP_CSC_ATTR_S csc;
  ISP_DYDPC_ATTR_S dpc;
  ISP_DEFOG_ATTR_S defog;
};
void isp_tune(XM_U32 iso, const struct isp_tune_attrs *a);

/* isp_color.c: the chroma and anti-false-colour setters' registers */
void isp_chroma(const ISP_CHROMA_ATTR_S *a);
void isp_antifalsecolor(const ISP_ANTI_FALSECOLOR_S *a);

/* mpi_vda.c: motion detection on each new AE grid */
void vda_frame(const ISP_AE_STAT_2_S *grid);

/* isp_3a.c */
int isp_3a_register(ISP_DEV dev);
int isp_3a_init(ISP_DEV dev, const ISP_PUB_ATTR_S *pub);
int isp_3a_start(void);
XM_U32 isp_3a_iso(void); /* the last AE result's ISO, 0 before the first */

/* mpi_isp.c */
const ISP_PUB_ATTR_S *isp_pub(void);
SENSOR_ID isp_sensor_id(void); /* the registered sensor's, -1 for none */

#pragma GCC visibility pop

#endif
