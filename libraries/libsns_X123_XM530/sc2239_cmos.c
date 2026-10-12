#include "XAx_cmos.h"
#include "mpi_ae.h"
#include "mpi_awb.h"
#include "mpi_isp.h"
#include "xm_awb_comm.h"
#include "xm_comm_isp.h"
#include "xm_comm_sns.h"
#include "xm_print.h"
#include "xm_sns_ctrl.h"
#include "xm_type.h"

#ifndef SOC_NONE
#include "xm510_tmp.h"
#endif
#include "mpi_phyvi.h"

#define SC2239_VTS_30FPS (1125)
#define SC2239_VTS_25FPS (1350)
#define SC2239_EXP_OFFSET (4)

extern XM_U8 gu8Fps;
extern const ISP_CMOS_GAMMA_S gstIspGamma;
extern XM_VOID XM_MPI_ISP_Memset(XM_U8 *pu8Addr, XM_U8 u8Ch, XM_U32 u32Num);
extern XM_U32 gau32AllGain;
extern XM_U32 gu32DGainNow;
extern XM_U32 gu32AGainNow;

/* Based on the SC2335 tables, with stronger NR at high gain; not calibrated. */
static const ISP_CMOS_AGC_TABLE_S g_stIspAgcTable = {
    /* bvalid */
    1,
    /* 100, 200, 400, 800, 1600, 3200, 6400, 12800, 25600, 51200, 102400,
       204800, 409600, 819200, 1638400, 3276800 */
    /* sharpen_D	H	*/
    {0x18, 0x18, 0x18, 0x18, 0x18, 0x14, 0x06, 0x04, 0x04, 0x04, 0x04, 0x04,
     0x04, 0x04, 0x04, 0x04},
    /* sharpen_Ud M	*/
    {0x1C, 0x1C, 0x1C, 0x1C, 0x1C, 0x18, 0x10, 0x10, 0x04, 0x04, 0x04, 0x04,
     0x04, 0x04, 0x04, 0x04},
    /* sharpen_Kd  */
    {0x2C, 0x2C, 0x2C, 0x2C, 0x28, 0x18, 0x10, 0x10, 0x08, 0x08, 0x08, 0x08,
     0x08, 0x08, 0x08, 0x08},
    /* snr_thresh 2DNr		*/
    {0x01, 0x02, 0x04, 0x0C, 0x18, 0x28, 0x38, 0x48, 0x60, 0x60, 0x60, 0x60,
     0x60, 0x60, 0x60, 0x60},
    /* snr_thresh 3DNr	 Tf	*/
    {0x04, 0x05, 0x08, 0x0C, 0x12, 0x18, 0x1C, 0x20, 0x20, 0x20, 0x20, 0x20,
     0x20, 0x20, 0x20, 0x20},
    /* snr_thresh 3DNr	 Sf	*/
    {0x00, 0x00, 0x02, 0x06, 0x10, 0x20, 0x30, 0x40, 0x40, 0x40, 0x40, 0x40,
     0x40, 0x40, 0x40, 0x40},
    /* DyDpc_thresh 		*/
    {0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0xD0, 0xD8, 0xD8, 0xD8, 0xD8, 0xD8,
     0xD8, 0xD8, 0xD8, 0xD8},
    /* saturation_strength */
    {0xE8, 0xE6, 0xE0, 0xD0, 0xB0, 0x80, 0x50, 0x30, 0x20, 0x20, 0x20, 0x20,
     0x20, 0x20, 0x20, 0x20},
    /* Blc	*/
    {0x104, 0x104, 0x104, 0x104, 0x104, 0x104, 0x104, 0x104, 0x104, 0x104,
     0x104, 0x104, 0x104, 0x104, 0x104, 0x104},
    /*Y_Tran gamma*/
    {0x32, 0x32, 0x32, 0x32, 0x32, 0x32, 0x38, 0x38, 0x38, 0x38, 0x28, 0x28,
     0x28, 0x28, 0x28, 0x28}};

static XM_S32 cmos_get_isp_default(ISP_CMOS_DEFAULT_S *pstDef) {
  if (XM_NULL == pstDef) {
    ERR("null pointer when get isp default value!\n");
    return -1;
  }
  pstDef->stGamma.bValid = XM_TRUE;
  XM_MPI_ISP_Memncpy((XM_U8 *)&pstDef->stGamma, (XM_U8 *)&gstIspGamma,
                     sizeof(ISP_CMOS_GAMMA_S));
  XM_MPI_ISP_Memncpy((XM_U8 *)&pstDef->stAgcTbl, (XM_U8 *)&g_stIspAgcTable,
                     sizeof(ISP_CMOS_AGC_TABLE_S));
  return 0;
}

static XM_VOID cmos_set_pixel_detect(XM_BOOL bEnable) { return; }

static XM_S32 cmos_set_mirror_flip(XM_U8 u8Mirror, XM_U8 u8Flip) {
  XM_U32 u32Val;

  u32Val = sensor_read_register(0x3221);
  if (u8Mirror) {
    u32Val |= 0x06;
  } else {
    u32Val &= ~0x06;
  }
  if (u8Flip) {
    u32Val |= 0x60;
  } else {
    u32Val &= ~0x60;
  }
  sensor_write_register(0x3221, u32Val);
  VI_WinSet(1, u8Mirror, u8Flip, 0, 0);
  XM_MPI_MIPI_RefreshFV(100, 0);
  return XM_SUCCESS;
}

/* 0x3e08 coarse 0x03/0x07/0x0f/0x1f, 0x3e09 fine 0x20..0x3f */
static XM_VOID cmos_again_calc_table(XM_U32 u32InTimes,
                                     AE_SENSOR_GAININFO_S *pstAeSnsGainInfo) {
  XM_U8 u8i;
  if (XM_NULL == pstAeSnsGainInfo) {
    return;
  }
  pstAeSnsGainInfo->u32GainDb = 0;

  for (u8i = 0; u8i < 3; u8i++) {
    if (u32InTimes < 2048) // 2x
    {
      break;
    }
    u32InTimes >>= 1;
    pstAeSnsGainInfo->u32GainDb = (pstAeSnsGainInfo->u32GainDb << 1) | 1;
  }
  if (u32InTimes > 2047) {
    u32InTimes = 2047;
  }

  u32InTimes = (u32InTimes >> 5);
  pstAeSnsGainInfo->u32GainDb = (pstAeSnsGainInfo->u32GainDb << 8) | u32InTimes;
  pstAeSnsGainInfo->u32SnsTimes = (u32InTimes << 5) << u8i;
  gu32AGainNow = pstAeSnsGainInfo->u32SnsTimes;
  return;
}

/* 0x3e06 coarse 0x00/0x01/0x03/0x07/0x0f, 0x3e07 fine 0x80..0xff */
static XM_VOID cmos_dgain_calc_table(XM_U32 u32InTimes,
                                     AE_SENSOR_GAININFO_S *pstAeSnsGainInfo) {
  XM_U8 u8i;
  if (XM_NULL == pstAeSnsGainInfo) {
    return;
  }
  pstAeSnsGainInfo->u32GainDb = 0;

  for (u8i = 0; u8i < 4; u8i++) {
    if (u32InTimes < 2048) // 2x
    {
      break;
    }
    u32InTimes >>= 1;
    pstAeSnsGainInfo->u32GainDb = (pstAeSnsGainInfo->u32GainDb << 1) | 1;
  }
  if (u32InTimes > 2047) {
    u32InTimes = 2047;
  }

  u32InTimes = (u32InTimes >> 3);
  pstAeSnsGainInfo->u32GainDb = (pstAeSnsGainInfo->u32GainDb << 8) | u32InTimes;
  pstAeSnsGainInfo->u32SnsTimes = (u32InTimes << 3) << u8i;
  gu32DGainNow = pstAeSnsGainInfo->u32SnsTimes;
  return;
}

static XM_S32 cmos_get_ae_default(AE_SENSOR_DEFAULT_S *pstAeSnsDft) {
  if (XM_NULL == pstAeSnsDft) {
    ERR("null pointer when get ae default value!\n");
    return -1;
  }
  pstAeSnsDft->u32FullLinesStd = SC2239_VTS_25FPS;
  pstAeSnsDft->u8AeCompensation = 0x39;
  pstAeSnsDft->u32LinesPer500ms = pstAeSnsDft->u32FullLinesStd * 25 / 2;
  pstAeSnsDft->u32FlickerFreq = 0;

  pstAeSnsDft->u32MaxIntTime = pstAeSnsDft->u32FullLinesStd - SC2239_EXP_OFFSET;
  pstAeSnsDft->u32MinIntTime = 1;

  pstAeSnsDft->u32MaxAgain = 16128;
  pstAeSnsDft->u32MinAgain = 1024;

  pstAeSnsDft->u32MaxDgain = 2048;
  pstAeSnsDft->u32MinDgain = 1024;

  pstAeSnsDft->u32ISPDgainShift = 8;
  pstAeSnsDft->u32MaxISPDgain = 4 << pstAeSnsDft->u32ISPDgainShift;
  pstAeSnsDft->u32MinISPDgain = 1 << pstAeSnsDft->u32ISPDgainShift;

  pstAeSnsDft->bDelayCfg = XM_TRUE;

  gu32DGainNow = pstAeSnsDft->u32MinDgain;
  gu16FullLines = SC2239_VTS_25FPS;
  gu8Fps = 25;
  return 0;
}

static XM_S32 cmos_get_sensor_max_resolution(
    ISP_CMOS_SENSOR_MAX_RESOLUTION *pstSensorMaxResolution) {
  if (XM_NULL == pstSensorMaxResolution) {
    ERR("null pointer when get sensor max resolution \n");
    return -1;
  }

  pstSensorMaxResolution->u32MaxWidth = 1920;
  pstSensorMaxResolution->u32MaxHeight = 1080;
  return 0;
}

const static XM_U8 gau8Logic_sc2239[5][3] = {
    // 0x3634, 0x3306, 0x3301
    {0x64, 0x44, 0x04}, {0x64, 0x68, 0x05}, {0x64, 0x68, 0x05},
    {0x44, 0x68, 0x05}, {0x24, 0x68, 0x05},
};
const static XM_U16 gau16LogicAddr_sc2239[3] = {0x3634, 0x3306, 0x3301};

static void gainLogic_sc2239(XM_U8 u8CoarseAgain, XM_U8 u8FineAgain) {
  static XM_U8 su8Idx = 0xFF;
  XM_U8 u8Idx, u8i;

  switch (u8CoarseAgain) {
  case 0x03:
    u8Idx = (u8FineAgain <= 0x28) ? 0 : 1;
    break;
  case 0x07:
    u8Idx = 2;
    break;
  case 0x0f:
    u8Idx = 3;
    break;
  default:
    u8Idx = 4;
    break;
  }
  if (su8Idx == u8Idx) {
    return;
  }
  su8Idx = u8Idx;
  for (u8i = 0; u8i < 3; u8i++) {
    sensor_write_register((XM_U32)gau16LogicAddr_sc2239[u8i],
                          (XM_U32)gau8Logic_sc2239[u8Idx][u8i]);
  }
}

static XM_VOID cmos_inttime_update(XM_U32 u32IntTime) {
  if (gu32ShutNow == u32IntTime)
    return;
  gu32ShutNow = u32IntTime;
  u32IntTime = u32IntTime * 2;
  sensor_write_register(0x3e00, ((u32IntTime & 0xFF000) >> 12));
  sensor_write_register(0x3e01, ((u32IntTime & 0x0ff0) >> 4));
  sensor_write_register(0x3e02, ((u32IntTime & 0xf) << 4));
  return;
}

static XM_VOID cmos_gains_update(XM_U32 u32Again, XM_U32 u32Dgain) {
  static XM_U32 su32AGain = 0xFFFFFFF;
  static XM_U32 su32DGain = 0xFFFFFFF;
  XM_U8 u8CoarseAgain, u8FineAgain;

  gau32AllGain = (XM_U64)gu32AGainNow * gu32DGainNow >> 10;
  if ((su32AGain != u32Again) || (su32DGain != u32Dgain)) {
    su32AGain = u32Again;
    su32DGain = u32Dgain;
    u8FineAgain = u32Again & 0xFF;
    u8CoarseAgain = (((u32Again >> 8) & 0x07) << 2) | 0x03;

    sensor_write_register(0x3812, 0x00);
    sensor_write_register(0x3e08, u8CoarseAgain);
    sensor_write_register(0x3e09, u8FineAgain);
    sensor_write_register(0x3e06, (u32Dgain >> 8) & 0x0F);
    sensor_write_register(0x3e07, u32Dgain & 0xFF);
    gainLogic_sc2239(u8CoarseAgain, u8FineAgain);
    sensor_write_register(0x3812, 0x30);
  }
  return;
}

static XM_VOID cmos_slow_framerate_set(XM_U16 u16FullLines,
                                       AE_SENSOR_DEFAULT_S *pstAeSnsDft) {
  u16FullLines = (u16FullLines >= 4096) ? 4000 : u16FullLines;
  if (gu16FullLines == u16FullLines)
    return;

  pstAeSnsDft->u32MaxIntTime = u16FullLines - SC2239_EXP_OFFSET;
  SysDelay_ms(100);
  if (gu32ShutNow > pstAeSnsDft->u32MaxIntTime) {
    cmos_inttime_update(pstAeSnsDft->u32MaxIntTime);
  }
  sensor_write_register(0x320e, (u16FullLines & 0xff00) >> 8);
  sensor_write_register(0x320f, u16FullLines & 0xff);
  gu16FullLines = u16FullLines;
  XM_MPI_MIPI_RefreshFV(0, (XM_U32)u16FullLines);
}

static XM_VOID cmos_fps_set(XM_U8 u8Fps, AE_SENSOR_DEFAULT_S *pstAeSnsDft) {
  static XM_U8 su8Mode = 0xFF;
  XM_U32 u32TotalSizeV;

  switch (u8Fps) {
  case 30:
    u32TotalSizeV = SC2239_VTS_30FPS;
    break;
  case 25:
    u32TotalSizeV = SC2239_VTS_25FPS;
    break;
  default:
    return;
  }

  if (su8Mode == u8Fps) {
    return;
  }
  su8Mode = u8Fps;

  if (pstAeSnsDft != NULL) {
    pstAeSnsDft->u32FullLinesStd = u32TotalSizeV;
    pstAeSnsDft->u32MaxIntTime = u32TotalSizeV - SC2239_EXP_OFFSET;
    pstAeSnsDft->u32LinesPer500ms = u32TotalSizeV * u8Fps / 2;
  }
  if (gu32ShutNow > u32TotalSizeV - SC2239_EXP_OFFSET) {
    cmos_inttime_update(u32TotalSizeV - SC2239_EXP_OFFSET);
  }
  sensor_write_register(0x320e, (u32TotalSizeV >> 8) & 0xFF);
  sensor_write_register(0x320f, u32TotalSizeV & 0xFF);
  gu16FullLines = u32TotalSizeV;
  gu8Fps = u8Fps;
  XM_MPI_MIPI_RefreshFV(0, u32TotalSizeV);

  SysDelay_ms(20);
  return;
}

/****************************************************************************
 * AWB
 ****************************************************************************/
const static ISP_COLORMATRIX_AUTO_S g_stAwbCcm = {
    5000, {0x000, 180, 30, 46, 0x000, -22, 230, 48, 0x000, -2, -78, 336},
    4000, {0x000, 215, -61, 102, 0x000, -7, 142, 121, 0x000, 4, -136, 388},
    2800, {0x000, 189, -85, 152, 0x000, -40, 105, 192, 0x000, -58, -284, 598}};

const static ISP_AWB_CALIBRATION_V2_S gstAwbCal = {
    {0, 0, 4096, 2499, 3127, 2234, 3619, 4096},
    {4096, 3779, 0, 0, 4096, 2469, 4096, 1874},
    {-782, -4096, -1385, -4096, -3530, -4096, -3819, -2926},
    213,
    0,
    1533,
    0,
    2786,
    {0, 738, 1082, 1317, 1388, 1445, 1704, 1757, 2786, 0, 0, 0, 0, 0, 0, 0},
    {2000, 2150, 2800, 4000, 4150, 5000, 6500, 7500, 12000, 0, 0, 0, 0, 0, 0,
     0},
    {1396, 1024, 1582, 0}};

static XM_S32 cmos_get_awb_default(AWB_SENSOR_DEFAULT_S *pstAwbSnsDft) {
  if (XM_NULL == pstAwbSnsDft) {
    ERR("null pointer when get awb default value!\n");
    return XM_FAILURE;
  }
  pstAwbSnsDft->pstRbTable = (const AWB_COEF_TABLE_S *)(&gstAwbCal);
  // CCM
  XM_MPI_ISP_Memncpy((XM_U8 *)&(pstAwbSnsDft->stCcm), (XM_U8 *)&g_stAwbCcm,
                     sizeof(ISP_COLORMATRIX_AUTO_S));
  return XM_SUCCESS;
}

/****************************************************************************
 * callback structure                                                       *
 ****************************************************************************/
XM_S32
cmos_init_sensor_exp_function_sc2239(ISP_SENSOR_EXP_FUNC_S *pstSensorExpFunc) {
  pstSensorExpFunc->pfn_cmos_sensor_init = NULL;
  pstSensorExpFunc->pfn_cmos_get_isp_default = cmos_get_isp_default;
  pstSensorExpFunc->pfn_cmos_set_pixel_detect = cmos_set_pixel_detect;
  pstSensorExpFunc->pfn_cmos_get_sensor_max_resolution =
      cmos_get_sensor_max_resolution;
  pstSensorExpFunc->pfn_cmos_set_mirror_flip = cmos_set_mirror_flip;
  return 0;
}

XM_S32 cmos_init_ae_exp_function_sc2239(AE_SENSOR_EXP_FUNC_S *pstExpFuncs) {
  pstExpFuncs->pfn_cmos_fps_get = NULL;
  pstExpFuncs->pfn_cmos_get_ae_default = cmos_get_ae_default;
  pstExpFuncs->pfn_cmos_fps_set = cmos_fps_set;
  pstExpFuncs->pfn_cmos_slow_framerate_set = cmos_slow_framerate_set;
  pstExpFuncs->pfn_cmos_inttime_update = cmos_inttime_update;
  pstExpFuncs->pfn_cmos_gains_update = cmos_gains_update;
  pstExpFuncs->pfn_cmos_again_calc_table = cmos_again_calc_table;
  pstExpFuncs->pfn_cmos_dgain_calc_table = cmos_dgain_calc_table;
  pstExpFuncs->pfn_cmos_shut_calc_table = NULL;
  return 0;
}

XM_S32 cmos_init_awb_exp_function_sc2239(AWB_SENSOR_EXP_FUNC_S *pstExpFuncs) {
  pstExpFuncs->pfn_cmos_get_awb_default = cmos_get_awb_default;
  return 0;
}

/* MIPI 1 lane, 27 MHz input, from OpenIPC/sensors sensor_sc2239_mipi.c, then
 * full 1928x1088 output and 25 fps. */
static const XM_U16 gau16SnsInit_sc2239_mipi[][2] = {
    {0x0103, 0x01}, {0x0100, 0x00}, {0x36e9, 0x80}, {0x36f9, 0x80},
    {0x301f, 0x21}, {0x3038, 0x44}, {0x3253, 0x12}, {0x3301, 0x04},
    {0x3304, 0xa8}, {0x3306, 0x44}, {0x3308, 0x10}, {0x3309, 0x48},
    {0x330a, 0x01}, {0x330b, 0x40}, {0x331e, 0xa1}, {0x331f, 0x41},
    {0x3333, 0x10}, {0x3364, 0x17}, {0x3390, 0x08}, {0x3391, 0x18},
    {0x3392, 0x38}, {0x3393, 0x08}, {0x3394, 0x0d}, {0x3395, 0x70},
    {0x33af, 0x20}, {0x360f, 0x01}, {0x3630, 0x00}, {0x3634, 0x64},
    {0x3637, 0x10}, {0x363c, 0x05}, {0x3670, 0x0c}, {0x3671, 0xc2},
    {0x3672, 0x02}, {0x3673, 0x02}, {0x3677, 0x84}, {0x3678, 0x84},
    {0x3679, 0x8e}, {0x367a, 0x18}, {0x367b, 0x38}, {0x367e, 0x08},
    {0x367f, 0x38}, {0x3690, 0x74}, {0x3691, 0x64}, {0x3692, 0x64},
    {0x369c, 0x08}, {0x369d, 0x18}, {0x3904, 0x08}, {0x3908, 0x82},
    {0x3933, 0x82}, {0x3934, 0x1b}, {0x3940, 0x77}, {0x3941, 0x18},
    {0x3942, 0x02}, {0x3943, 0x1c}, {0x3944, 0x0b}, {0x3945, 0x80},
    {0x3e01, 0x8c}, {0x3e02, 0x20}, {0x4509, 0x20}, {0x4800, 0x64},
    {0x4819, 0x09}, {0x481b, 0x05}, {0x481d, 0x14}, {0x4821, 0x0a},
    {0x4823, 0x05}, {0x5000, 0x06}, {0x5780, 0x7f}, {0x5781, 0x04},
    {0x5782, 0x03}, {0x5783, 0x02}, {0x5784, 0x01}, {0x5785, 0x18},
    {0x5786, 0x10}, {0x5787, 0x08}, {0x5788, 0x02}, {0x5789, 0x20},
    {0x578a, 0x7f}, {0x36e9, 0x20}, {0x36f9, 0x27}, {0x3208, 0x07},
    {0x3209, 0x88}, {0x320a, 0x04}, {0x320b, 0x40}, {0x3211, 0x00},
    {0x3213, 0x00}, {0x320e, 0x05}, {0x320f, 0x46}, {0x0100, 0x01},

    {0xF5, 0xA5}, // SleepMask1
    {0x5A, 0x5F}, // SleepMask2
    {0x00, 20},   // 20ms
};

XM_U32 sensor_getlist_sc2239(XM_U16 *pu16Num) {
  DEBUG("------------- SC2239 1080P 25fps MIPI ----------------\n");
  *pu16Num =
      sizeof(gau16SnsInit_sc2239_mipi) / sizeof(gau16SnsInit_sc2239_mipi[0]);
  return (XM_U32)gau16SnsInit_sc2239_mipi;
}
