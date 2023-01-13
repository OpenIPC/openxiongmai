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

#ifdef SOC_NONE
#include "xm540_tmp.h"
#else
#include "xm510_tmp.h"
#endif
#include "mpi_phyvi.h"

#define HD1080_25_LINES (1125)

extern XM_U8 gu8Fps;
extern ISP_CMOS_SNS_ATTR_S gstSnsAttr;
extern const ISP_CMOS_GAMMA_S gstIspGamma;
extern XM_VOID XM_MPI_ISP_Memset(XM_U8 *pu8Addr, XM_U8 u8Ch, XM_U32 u32Num);
extern XM_U32 gau32AllGain;
extern XM_U32 gu32DGainNow;
static const ISP_CMOS_AGC_TABLE_S g_stIspAgcTable = {
    /* bvalid */
    1,
    /* 100, 200, 400, 800, 1600, 3200, 6400, 12800, 25600, 51200, 102400,
       204800, 409600, 819200, 1638400, 3276800 */
    /* sharpen_D	H	*/
    {0x30, 0x30, 0x2C, 0x2C, 0x24, 0x20, 0x1C, 0x10, 0x04, 0x04, 0x04, 0x04,
     0x04, 0x04, 0x04, 0x04},
    /* sharpen_Ud M	*/
    {0x30, 0x30, 0x30, 0x2C, 0x28, 0x24, 0x18, 0x10, 0x04, 0x04, 0x04, 0x04,
     0x04, 0x04, 0x04, 0x04},
    /* sharpen_Kd  */
    {0x38, 0x38, 0x2E, 0x2C, 0x24, 0x20, 0x1C, 0x10, 0x08, 0x08, 0x08, 0x08,
     0x08, 0x08, 0x08, 0x08},
    /* snr_thresh 2DNr		*/
    {0x01, 0x02, 0x03, 0x06, 0x09, 0x0D, 0x0F, 0x1C, 0x30, 0x40, 0x40, 0x40,
     0x40, 0x40, 0x40, 0x40},
    /* snr_thresh 3DNr	 Tf	*/
    {0x04, 0x05, 0x08, 0x0A, 0x11, 0x14, 0x18, 0x20, 0x20, 0x19, 0x18, 0x18,
     0x18, 0x18, 0x18, 0x18},
    /* snr_thresh 3DNr	 Sf	*/
    {0x00, 0x00, 0x01, 0x04, 0x0C, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10, 0x10,
     0x10, 0x10, 0x10, 0x10},
    /* DyDpc_thresh 		*/
    {0x00, 0x00, 0x00, 0x00, 0x00, 0xC0, 0xD0, 0xD8, 0xD8, 0xD8, 0xD8, 0xD8,
     0xD8, 0xD8, 0xD8, 0xD8},
    /* saturation_strength */
    {0xAC, 0xAC, 0xA6, 0xA3, 0xA0, 0x9E, 0x8C, 0x7C, 0x20, 0x20, 0x20, 0x20,
     0x20, 0x20, 0x20, 0x20},
    /* Blc	*/
    {0x44, 0x44, 0x44, 0x44, 0x3E, 0x3C, 0x2A, 0x28, 0x28, 0x28, 0x28, 0x28,
     0x28, 0x28, 0x28, 0x28},
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
  XM_U16 u16OfstV;

  u32Val = sensor_read_register(0x3221);

  if (u8Mirror) {
    u32Val |= 0x06;
  } else {
    u32Val &= ~0x06;
  }

  if (u8Flip) {
    u32Val |= 0xe0;
    if (gstSnsAttr.u8InputMode == SENSCONT_DVP) {
      u16OfstV = 1;
    } else {
      u16OfstV = 0;
    }
  } else {
    u32Val &= ~0xe0;
    u16OfstV = 0;
  }
  sensor_write_register(0x3221, u32Val);
  VI_WinSet(1, u8Mirror, u8Flip, 0, u16OfstV);
  XM_MPI_MIPI_RefreshFV(100, 0);
  return XM_SUCCESS;
}

static XM_VOID cmos_again_calc_table(XM_U32 u32InTimes,
                                     AE_SENSOR_GAININFO_S *pstAeSnsGainInfo) {
  if (XM_NULL == pstAeSnsGainInfo) {
    return;
  }
  XM_U8 u8i;
  if (u32InTimes >= gau16GainTbl_SmatSns[63]) {
    pstAeSnsGainInfo->u32GainDb = 63;
  } else {
    pstAeSnsGainInfo->u32GainDb = 0;
    for (u8i = 0x1; u8i < 64; u8i++) {
      if (u32InTimes < gau16GainTbl_SmatSns[u8i]) {
        pstAeSnsGainInfo->u32GainDb = u8i - 1;
        break;
      }
    }
  }
  pstAeSnsGainInfo->u32SnsTimes =
      (XM_U32)gau16GainTbl_SmatSns[pstAeSnsGainInfo->u32GainDb];
  return;
}

static XM_VOID cmos_dgain_calc_table(XM_U32 u32InTimes,
                                     AE_SENSOR_GAININFO_S *pstAeSnsGainInfo) {
  int i;
  if (XM_NULL == pstAeSnsGainInfo) {
    ERR("cmos_dgain_calc_table failed!\n");
    return;
  }
  pstAeSnsGainInfo->u32GainDb = 0;
  for (i = 0; i < 7; i++) {
    if (u32InTimes < 256) {
      break;
    }
    u32InTimes >>= 1;
    pstAeSnsGainInfo->u32GainDb = (pstAeSnsGainInfo->u32GainDb << 1) | 1;
  }
  pstAeSnsGainInfo->u32SnsTimes = u32InTimes << i;
  pstAeSnsGainInfo->u32GainDb = (pstAeSnsGainInfo->u32GainDb << 8) + u32InTimes;
  gu32DGainNow = pstAeSnsGainInfo->u32SnsTimes;
  return;
}

static XM_S32 cmos_get_ae_default(AE_SENSOR_DEFAULT_S *pstAeSnsDft) {
  if (XM_NULL == pstAeSnsDft) {
    ERR("null pointer when get ae default value!\n");
    return -1;
  }
  pstAeSnsDft->u32FullLinesStd = HD1080_25_LINES;
  pstAeSnsDft->u8AeCompensation = 0x39;
  pstAeSnsDft->u32LinesPer500ms = pstAeSnsDft->u32FullLinesStd * 25 / 2;
  pstAeSnsDft->u32FlickerFreq = 0;

  pstAeSnsDft->u32MaxIntTime = pstAeSnsDft->u32FullLinesStd - 4;
  pstAeSnsDft->u32MinIntTime = 2;

  pstAeSnsDft->u32MaxAgain = 15872;
  pstAeSnsDft->u32MinAgain = 1024;

  pstAeSnsDft->u32MaxDgain = 3968;
  pstAeSnsDft->u32MinDgain = 128;

  pstAeSnsDft->u32ISPDgainShift = 8;
  pstAeSnsDft->u32MaxISPDgain = 8 << pstAeSnsDft->u32ISPDgainShift;
  pstAeSnsDft->u32MinISPDgain = 1 << pstAeSnsDft->u32ISPDgainShift;

  pstAeSnsDft->bDelayCfg = XM_TRUE;

  gu32DGainNow = pstAeSnsDft->u32MinDgain;
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

static void shutLogic_sc307e(XM_U32 u32IntTime) {
  static XM_U8 su8Val = 0xFF;
  XM_U8 u8Val;
  u8Val = su8Val;
  if (u32IntTime < 0x250)
    u8Val = 0x14;
  else if (u32IntTime > 0x450)
    u8Val = 0x04;
  if (su8Val != u8Val) {
    su8Val = u8Val;
    sensor_write_register(0x3314, (XM_U32)u8Val);
  }
}

const static XM_U8 gau8Logic_sc307e[5][2] = {
    //  0x3301 0x3632
    {0x0f, 0x08}, {0x20, 0x08}, {0x28, 0x08}, {0x80, 0x08}, {0x80, 0x48},
};
const static XM_U16 gau16LogicAddr_Mipi[2] = {0x3301, 0x3632};
static void gainLogic_sc307e(XM_U32 u32AllGain) {
  static XM_U8 su8Idx = 0xFF; //[bit0~bit3]:Vstd   [bit4~bit7]:Agc
  XM_U8 u8Idx2, u8i;
  u32AllGain = u32AllGain / 64; // *1024->*16
  if (u32AllGain < 32)
    u8Idx2 = 0;
  else if (u32AllGain < 64)
    u8Idx2 = 1;
  else if (u32AllGain < 128)
    u8Idx2 = 2;
  else if (u32AllGain < 248)
    u8Idx2 = 3;
  else
    u8Idx2 = 4;
  if (((su8Idx >> 4) & 0x0F) != u8Idx2) {
    su8Idx = ((u8Idx2 & 0x0F) << 4);
    sensor_write_register(0x3812, 0x00);
    for (u8i = 0; u8i < 2; u8i++) {
      sensor_write_register((XM_U32)gau16LogicAddr_Mipi[u8i],
                            (XM_U32)gau8Logic_sc307e[u8Idx2][u8i]);
    }
    sensor_write_register(0x3812, 0x30);
  }
}

static XM_VOID cmos_inttime_update(XM_U32 u32IntTime) {
  if (gu32ShutNow == u32IntTime)
    return;
  gu32ShutNow = u32IntTime;
  shutLogic_sc307e(u32IntTime);
  u32IntTime = u32IntTime * 2;
  sensor_write_register(0x3e00, ((u32IntTime & 0xFF000) >> 12));
  sensor_write_register(0x3e01, ((u32IntTime & 0x0ff0) >> 4));
  sensor_write_register(0x3e02, ((u32IntTime & 0xf) << 4));
  return;
}

static XM_VOID cmos_gains_update(XM_U32 u32Again, XM_U32 u32Dgain) {
  static XM_U32 su32AGain = 0xFFFFFFF;
  static XM_U32 su32DGain = 0xFFFFFFF;
  unsigned int u32Tmp, tmp[4];
  u32Again = (u32Again > 63) ? 63 : u32Again;
  gau32AllGain = (XM_U64)gau16GainTbl_SmatSns[u32Again] * gu32DGainNow / 128;
  gainLogic_sc307e(gau32AllGain);
  if ((su32AGain != u32Again) || (su32DGain != u32Dgain)) {
    su32AGain = u32Again;
    su32DGain = u32Dgain;
    // 2.GainUpdate
    // a.Again
    tmp[0] = (u32Again & 0x0F) | 0x10;
    u32Tmp = 0x03;
    u32Again = u32Again / 16;
    for (; u32Again > 0; u32Again--) {
      u32Tmp = (u32Tmp << 1) | 0x01;
    }
    tmp[1] = u32Tmp;
    // b.DGain
    tmp[2] = (u32Dgain >> 8) & 0xFF;
    tmp[3] = u32Dgain & 0xFF;

    sensor_write_register(0x3e08, tmp[1]);
    sensor_write_register(0x3e09, tmp[0]);
    sensor_write_register(0x3e06, tmp[2]);
    sensor_write_register(0x3e07, tmp[3]);
  }
  return;
}

static XM_VOID cmos_slow_framerate_set(XM_U16 u16FullLines,
                                       AE_SENSOR_DEFAULT_S *pstAeSnsDft) {
  static XM_U16 preU16FullLine = 0xffff;
  if (preU16FullLine == u16FullLines)
    return;

  preU16FullLine = u16FullLines;
  u16FullLines = (u16FullLines >= 4096) ? 4000 : u16FullLines;
  pstAeSnsDft->u32MaxIntTime = u16FullLines - 4;
  SysDelay_ms(100);
  if (gu32ShutNow > pstAeSnsDft->u32MaxIntTime) {
    cmos_inttime_update(pstAeSnsDft->u32MaxIntTime);
  }
  sensor_write_register(0x320e, (u16FullLines & 0xff00) >> 8);
  sensor_write_register(0x320f, u16FullLines & 0xff);

  XM_MPI_MIPI_RefreshFV(0, (XM_U32)u16FullLines);
}

/* the function of sensor set fps */
static XM_VOID cmos_fps_set(XM_U8 u8Fps, AE_SENSOR_DEFAULT_S *pstAeSnsDft) {
  static XM_U8 su8Mode = 0xFF;
  XM_U32 u32Pixs = 2640;
  XM_U32 u32TotalSizeV;
  XM_U32 u32ExpNow;
  XM_U8 u8Mode;
  u32ExpNow = sensor_read_register(0x3e00);
  u32ExpNow = (u32ExpNow << 8) | sensor_read_register(0x3e01);
  u32Pixs = sensor_read_register(0x3e02);
  u32ExpNow = (u32ExpNow << 4) | ((u32Pixs >> 4) & 0x0F);
  switch (u8Fps) {
  case 30:
    u32Pixs = 2200;
    u32TotalSizeV = HD1080_25_LINES;
    break;
  case 25:
    u32Pixs = 2640;
    u32TotalSizeV = HD1080_25_LINES;
    break;
  default:
    return;
    break;
  }
  u8Mode = u8Fps;
  if (su8Mode == u8Mode) {
    return;
  }
  su8Mode = u8Mode;

  if (pstAeSnsDft != NULL) {
    pstAeSnsDft->u32FullLinesStd = u32TotalSizeV;
    pstAeSnsDft->u32MaxIntTime = HD1080_25_LINES - 4;
    pstAeSnsDft->u32LinesPer500ms = HD1080_25_LINES * u8Fps / 2;
  }
  if (u32ExpNow > (u32TotalSizeV - 4)) {
    u32ExpNow = u32TotalSizeV - 4;
    cmos_inttime_update(u32ExpNow);
  }
  sensor_write_register(0x320c, (u32Pixs >> 8) & 0xFF);
  sensor_write_register(0x320d, u32Pixs & 0xFF);
  sensor_write_register(0x320e, (u32TotalSizeV >> 8) & 0xFF);
  sensor_write_register(0x320f, u32TotalSizeV & 0xFF);
  gu8Fps = u8Fps;
  return;
}

/****************************************************************************
 * AWB
 ****************************************************************************/
const static ISP_COLORMATRIX_AUTO_S g_stAwbCcm = {
    5000, // 0.16
    {0x0000, 348, -127, 35, 0x0000, -91, 372, -25, 0x0000, 22, -148, 382},
    4000, // 0.15
    {0x0000, 342, -143, 57, 0x0000, -95, 369, -18, 0x0000, 8, -200, 448},
    2800, // 0.13
    {0x0000, 245, -38, 49, 0x0000, -144, 397, 3, 0x0000, -110, -338, 704}};

const static ISP_AWB_CALIBRATION_V2_S gstAwbCal = {
    {0, 0, 4096, 3114, 3894, 2386, 1159, 4096},
    {4096, 4096, 0, 0, 4096, 2865, 4096, 1061},
    {-443, -3814, -1254, -4096, -2990, -4096, -1562, -1985},
    213,
    0,
    1389,
    0,
    2421,
    {0, 643, 857, 1096, 1179, 1279, 1469, 1486, 2421, 0, 0, 0, 0, 0, 0, 0},
    {2000, 2150, 2800, 4000, 4150, 5000, 6500, 7500, 12000, 0, 0, 0, 0, 0, 0,
     0},
    {1682, 1024, 2045, 0}};

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
cmos_init_sensor_exp_function_sc307e(ISP_SENSOR_EXP_FUNC_S *pstSensorExpFunc) {
  pstSensorExpFunc->pfn_cmos_sensor_init = NULL;
  pstSensorExpFunc->pfn_cmos_get_isp_default = cmos_get_isp_default;
  pstSensorExpFunc->pfn_cmos_set_pixel_detect = cmos_set_pixel_detect;
  pstSensorExpFunc->pfn_cmos_get_sensor_max_resolution =
      cmos_get_sensor_max_resolution;
  pstSensorExpFunc->pfn_cmos_set_mirror_flip = cmos_set_mirror_flip;
  return 0;
}

XM_S32 cmos_init_ae_exp_function_sc307e(AE_SENSOR_EXP_FUNC_S *pstExpFuncs) {
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

XM_S32 cmos_init_awb_exp_function_sc307e(AWB_SENSOR_EXP_FUNC_S *pstExpFuncs) {
  pstExpFuncs->pfn_cmos_get_awb_default = cmos_get_awb_default;
  return 0;
}

static const XM_U16 gau16SnsInit_sc307e_mipi[][2] = {
    {0x0100, 0x00}, {0x3034, 0x81}, // pll2 bypass
    {0x3039, 0xa2},                 // pll1 bypass
    {0x3001, 0xfe}, {0x3018, 0x33}, {0x301c, 0x78}, {0x3031, 0x0a},
    {0x3037, 0x20}, {0x3038, 0xff}, {0x303c, 0x0e}, {0x3200, 0x00},
    {0x3201, 0x00}, {0x3202, 0x00}, {0x3203, 0x00}, {0x3204, 0x07},
    {0x3205, 0x8f}, {0x3206, 0x04}, {0x3207, 0x47}, {0x3208, 0x07},
    {0x3209, 0x88}, {0x320a, 0x04}, {0x320b, 0x40}, {0x320c, 0x0a},
    {0x320d, 0x50}, {0x320e, 0x04}, {0x320f, 0x65}, {0x3211, 0x04},
    {0x3213, 0x04}, {0x3222, 0x29}, {0x3235, 0x08}, {0x3236, 0xc8},
    {0x3301, 0x0f}, {0x3302, 0x1f}, {0x3303, 0x20}, {0x3306, 0x48},
    {0x3308, 0x10}, {0x3309, 0x60}, {0x330b, 0xcd}, {0x330e, 0x30},
    {0x3314, 0x04}, {0x331b, 0x83}, {0x331e, 0x19}, {0x331f, 0x59},
    {0x3320, 0x01}, {0x3326, 0x00}, {0x3333, 0x30}, {0x335e, 0x01},
    {0x335f, 0x03}, {0x3366, 0x7c}, {0x3367, 0x08}, {0x3368, 0x04},
    {0x3369, 0x00}, {0x336a, 0x00}, {0x336b, 0x00}, {0x337c, 0x04},
    {0x337d, 0x06}, {0x337f, 0x03}, {0x33a0, 0x05}, {0x33aa, 0x10},
    {0x360f, 0x01}, {0x3614, 0x80}, {0x3620, 0x28}, {0x3621, 0x28},
    {0x3622, 0x06}, {0x3624, 0x08}, {0x3625, 0x02}, {0x3630, 0x9c},
    {0x3631, 0x84}, {0x3632, 0x08}, {0x3633, 0x4f}, {0x3635, 0xa0},
    {0x3636, 0x25}, {0x3637, 0x55}, {0x3638, 0x1f}, {0x3639, 0x09},
    {0x363a, 0x9f}, {0x363b, 0x06}, {0x363c, 0x05}, {0x3641, 0x01},
    {0x366e, 0x08}, {0x366f, 0x2c}, {0x3670, 0x0c}, {0x3671, 0xc6},
    {0x3672, 0x06}, {0x3673, 0x16}, {0x3677, 0x86}, {0x3678, 0x88},
    {0x3679, 0x88}, {0x367a, 0x28}, {0x367b, 0x3f}, {0x367e, 0x08},
    {0x367f, 0x28}, {0x3690, 0x33}, {0x3691, 0x11}, {0x3692, 0x43},
    {0x369c, 0x08}, {0x369d, 0x28}, {0x3802, 0x01}, {0x3900, 0x19},
    {0x3901, 0x02}, {0x3902, 0x45}, {0x3903, 0x08}, {0x3905, 0x98},
    {0x3907, 0x00}, {0x3908, 0x11}, {0x391d, 0x04}, {0x391e, 0x00},
    {0x3933, 0x0a}, {0x3934, 0x18}, // ����ƫ��
    {0x3940, 0x60}, {0x3942, 0x02}, {0x3943, 0x1f}, {0x395e, 0xc0},
    {0x3960, 0xba}, {0x3961, 0xae}, {0x3962, 0x89}, {0x3966, 0xba},
    {0x3980, 0xa0}, {0x3981, 0x40}, {0x3982, 0x18}, {0x3984, 0x08},
    {0x3985, 0x18}, // ����ƫ��
    {0x3986, 0x28}, // ����ƫ��
    {0x3987, 0x70}, // ����ƫ��
    {0x3988, 0x08}, {0x3989, 0x10}, {0x398a, 0x20}, {0x398b, 0x30},
    {0x398c, 0x60}, {0x398d, 0x20}, {0x398e, 0x10}, {0x398f, 0x08},
    {0x3990, 0x40}, // ����ƫ��
    {0x3991, 0x24}, {0x3992, 0x15}, {0x3993, 0x08}, {0x3994, 0x0a},
    {0x3995, 0x20}, {0x3996, 0x38}, {0x3997, 0x70}, // ����ƫ��
    {0x3998, 0x08}, {0x3999, 0x10}, {0x399a, 0x18}, {0x399b, 0x30},
    {0x399c, 0x30}, {0x399d, 0x18}, {0x399e, 0x10}, {0x399f, 0x08},
    {0x3e00, 0x00}, {0x3e01, 0x8c}, {0x3e02, 0x60}, {0x3e03, 0x0b},
    {0x3e06, 0x00}, {0x3e07, 0x80}, {0x3e08, 0x03}, {0x3e09, 0x10},
    {0x3e1e, 0x34}, {0x3f00, 0x07}, {0x3f04, 0x04}, {0x3f05, 0x28},
    {0x4603, 0x00}, {0x4809, 0x01}, {0x4827, 0x48}, {0x4837, 0x35},
    {0x5000, 0x06}, {0x5002, 0x06}, {0x5780, 0x7f}, {0x5781, 0x04},
    {0x5782, 0x03}, {0x5783, 0x02}, {0x5784, 0x01}, {0x5785, 0x18},
    {0x5786, 0x10}, {0x5787, 0x08}, {0x5788, 0x02}, {0x57a0, 0x00},
    {0x57a1, 0x71}, {0x57a2, 0x01}, {0x57a3, 0xf1}, {0x3034, 0x01},
    {0x3039, 0x22}, {0x0100, 0x01},

    {0xF5, 0xA5}, // SleepMask1
    {0x5A, 0x5F}, // SleepMask2
    {0x00, 50},   // 50ms
};

// ExtClk: 27M  20190409
static const unsigned short gau16SnsInit_sc307e_dvp[][2] = {
    {0x0100, 0x00}, {0x3034, 0x81}, // pll2 bypass
    {0x3039, 0xa6},                 // pll1 bypass
    {0x3018, 0x1f}, {0x3019, 0xff}, {0x301c, 0xb4}, {0x301f, 0x02},
    {0x3038, 0xff}, {0x303b, 0x16}, {0x303f, 0x81}, {0x320c, 0x0A},
    {0x320d, 0x50}, {0x320e, 0x04}, {0x320f, 0x65},

    {0x3200, 0x00}, {0x3201, 0x00}, {0x3202, 0x00}, {0x3203, 0x00},
    {0x3204, 0x07}, {0x3205, 0x8F}, // 0x01
    {0x3206, 0x04}, {0x3207, 0x47}, {0x3208, 0x07}, {0x3209, 0x88},
    {0x320a, 0x04}, {0x320b, 0x40}, {0x3210, 0x00}, {0x3211, 0x04},
    {0x3212, 0x00}, {0x3213, 0x04},

    {0x3222, 0x29}, {0x3235, 0x08}, {0x3236, 0xc8}, {0x3301, 0x0f},
    {0x3302, 0x1f}, {0x3303, 0x20}, {0x3306, 0x48}, {0x3308, 0x10},
    {0x3309, 0x60}, {0x330b, 0xcd}, {0x330e, 0x30}, {0x3314, 0x04},
    {0x331b, 0x83}, {0x331e, 0x19}, {0x331f, 0x59}, {0x3320, 0x01},
    {0x3326, 0x00}, {0x3333, 0x30}, {0x335e, 0x01}, {0x335f, 0x03},
    {0x3366, 0x7c}, {0x3367, 0x08}, {0x3368, 0x04}, {0x3369, 0x00},
    {0x336a, 0x00}, {0x336b, 0x00}, {0x337c, 0x04}, {0x337d, 0x06},
    {0x337f, 0x03}, {0x33a0, 0x05}, {0x33aa, 0x10}, {0x360f, 0x01},
    {0x3614, 0x80}, {0x3620, 0x28}, {0x3621, 0x28}, {0x3622, 0x06},
    {0x3624, 0x08}, {0x3625, 0x02}, {0x3630, 0x9c}, {0x3631, 0x84},
    {0x3632, 0x08}, {0x3633, 0x4f}, {0x3635, 0xa0}, {0x3636, 0x25},
    {0x3637, 0x55}, {0x3638, 0x1f}, {0x3639, 0x09}, {0x363a, 0x9f},
    {0x363b, 0x06}, {0x363c, 0x05}, {0x3641, 0x01}, {0x366e, 0x08},
    {0x366f, 0x2c}, {0x3670, 0x0c}, {0x3671, 0xc6}, {0x3672, 0x06},
    {0x3673, 0x16}, {0x3677, 0x86}, {0x3678, 0x88}, {0x3679, 0x88},
    {0x367a, 0x28}, {0x367b, 0x3f}, {0x367e, 0x08}, {0x367f, 0x28},
    {0x3690, 0x33}, {0x3691, 0x11}, {0x3692, 0x43}, {0x369c, 0x08},
    {0x369d, 0x28}, {0x3802, 0x00}, {0x3900, 0x19}, {0x3901, 0x02},
    {0x3902, 0xc5}, {0x3903, 0x08}, {0x3905, 0x98}, {0x3907, 0x00},
    {0x3908, 0x11}, {0x391d, 0x04}, {0x391e, 0x00}, {0x3933, 0x0a},
    {0x3934, 0x18}, // ����ƫ��
    {0x3940, 0x60}, {0x3942, 0x02}, {0x3943, 0x1f}, {0x395e, 0xc0},
    {0x3960, 0xba}, {0x3961, 0xae}, {0x3962, 0x89}, {0x3963, 0x80},
    {0x3966, 0xba}, {0x3980, 0xa0}, {0x3981, 0x40}, {0x3982, 0x18},
    {0x3984, 0x08}, {0x3985, 0x18}, // ����ƫ��
    {0x3986, 0x28},                 // ����ƫ��
    {0x3987, 0x70},                 // ����ƫ��
    {0x3988, 0x08}, {0x3989, 0x10}, {0x398a, 0x20}, {0x398b, 0x30},
    {0x398c, 0x60}, {0x398d, 0x20}, {0x398e, 0x10}, {0x398f, 0x08},
    {0x3990, 0x40}, // ����ƫ��
    {0x3991, 0x24}, {0x3992, 0x15}, {0x3993, 0x08}, {0x3994, 0x0a},
    {0x3995, 0x20}, {0x3996, 0x38}, {0x3997, 0x70}, // ����ƫ��
    {0x3998, 0x08}, {0x3999, 0x10}, {0x399a, 0x18}, {0x399b, 0x30},
    {0x399c, 0x30}, {0x399d, 0x18}, {0x399e, 0x10}, {0x399f, 0x08},
    {0x3e00, 0x00}, {0x3e01, 0x8c}, {0x3e02, 0x60}, {0x3e03, 0x0b},
    {0x3e06, 0x00}, {0x3e07, 0x80}, {0x3e08, 0x03}, {0x3e09, 0x10},
    {0x3e1e, 0x34}, {0x3f00, 0x07}, {0x3f04, 0x04}, {0x3f05, 0x28},
    {0x5000, 0x06}, {0x5780, 0x7f}, {0x5781, 0x04}, {0x5782, 0x03},
    {0x5783, 0x02}, {0x5784, 0x01}, {0x5785, 0x18}, {0x5786, 0x10},
    {0x5787, 0x08}, {0x5788, 0x02}, {0x57a0, 0x00}, {0x57a1, 0x71},
    {0x57a2, 0x01}, {0x57a3, 0xf1}, {0x3034, 0x01}, // pll2 enable
    {0x3039, 0x26},                                 // pll1 enable
    {0x3d08, 0x03},                                 // Lycai ��ͬ���ź�ƥ��
    {0x0100, 0x01},

    {0xF5, 0xA5}, // SleepMask1
    {0x5A, 0x5F}, // SleepMask2
    {0x00, 50},   // 50ms
};

XM_U32 sensor_getlist_sc307e(XM_U16 *pu16Num) {
  if (gstSnsAttr.u8InputMode == SENSCONT_DVP) {
    DEBUG("------------- SC307E (@20190718_dvp) ----------------\n");
    *pu16Num =
        sizeof(gau16SnsInit_sc307e_dvp) / sizeof(gau16SnsInit_sc307e_dvp[0]);
    return (XM_U32)gau16SnsInit_sc307e_dvp;
  } else {
    DEBUG("------------- SC307E (@20190110_mipi) ----------------\n");
    *pu16Num =
        sizeof(gau16SnsInit_sc307e_mipi) / sizeof(gau16SnsInit_sc307e_mipi[0]);
    return (XM_U32)gau16SnsInit_sc307e_mipi;
  }
}
