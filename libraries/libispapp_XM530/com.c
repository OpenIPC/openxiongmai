/*********************************************************************
Copyright (C), 2015-8-26, JuFeng.Tech. Co., Ltd.
File name: 	com.c
Author:
Versoin: 	       1.00
Data: 		2015-08-26
Desc:		串口处理源文件，实现串口相关通信处理
Ohters:		// 其他说明
Function List:

**********************************************************************/

/**************************** 调用函数库声明 *******************************/
#include "com.h"
#include "mpi_ae.h"
#include "mpi_awb.h"
#include "mpi_isp.h"
#include "xm_math.h"
#include "xm_type.h"

#include "Camera.h"
#include "IspTools.h"
#include "IspToolsFlash.h"
#include "xm_i2c.h"

#include "mpi_config.h"
#include "xm_sns_ctrl.h"

#include "mpi_venctx.h"
#include "xm_print.h"
/************************************************************************/

#define OPEN_ISPDEBUG 1

volatile uint8 m_pbCom_Data[13] = {0};

extern I2C_DATA_S gstI2CData;
extern volatile unsigned char gu8FpsNow;
extern volatile XM_U32 gU32T10msCnt;

volatile uint32 g_uLComTimeoutF = COM_TimeOut0;
volatile uint32 g_uLComTimeoutS = COM_TimeOut1;

eCOMERR eComErr;

XM_U8 gu8DebugCom_Loop = 1;
extern XM_S32 Init_UART0(XM_VOID);
XM_U8 gu8UartDebugEn = 0;
extern XM_U8 xmprop_get_value_v2(XM_U8 *KeyName, XM_VOID *pu32Addr);
/************************************************************************/
#if OPEN_ISPDEBUG // Lycai @20160114
extern XM_PRODUCT_INFO gstProductInfo;
extern XM_S32 Awb_Test(XM_U8 u8Mode, XM_U8 u8Choice);
extern void awb_manualCurveKg_set(XM_U8 u8Mode, XM_U8 u8Curve, XM_U16 u16Kg);
extern XM_S32 awb_manualCurveKg_get(XM_U8 *pu8Mode, XM_U8 *pu8Curve,
                                    XM_U16 *pu16Kg);
extern XM_S32 SysVstdSet(XM_U8 u8StdType, XM_U8 u8Rslt, XM_U8 u8Encoder);
extern XM_S32 XM_MPI_MENU_SetDefault(XM_U8 u8Mode);
extern XM_S32 XM_MPI_MENU_SaveCfg(XM_U32 u32Cmd, XM_S32 s32Data);

static XM_S32 IspDebugAe(unsigned char u8Mode, unsigned int u32Addr,
                         XM_U32 *pu32Data) {
  XM_S32 s32Ret;
  XM_U32 u32Data, u32Value;
  ISP_INNER_STATE_INFO_S stInnerStateInfo;
  ISP_EXPOSURE_ATTR_S stExpAttr;
  ISP_EXP_STA_INFO_S stExpStatistic;
  u32Data = *pu32Data;
  u32Value = 0xFFFFFFFF;
  s32Ret = XM_MPI_ISP_QueryInnerStateInfo(ISP_USED, &stInnerStateInfo);
  if (s32Ret != XM_SUCCESS) {
    putstr(0, "XM_MPI_ISP_QueryInnerStateInfo failed!\r\n");
    return XM_FAILURE;
  }
  s32Ret = XM_MPI_ISP_GetExposureAttr(ISP_USED, &stExpAttr);
  if (s32Ret != XM_SUCCESS) {
    putstr(0, "XM_MPI_ISP_GetExposureAttr failed!\r\n");
    return XM_FAILURE;
  }

  s32Ret = XM_MPI_ISP_GetExpStaInfo(ISP_USED, &stExpStatistic);
  if (s32Ret != XM_SUCCESS) {
    putstr(0, "XM_MPI_ISP_GetExpStaInfo failed!\r\n");
    return XM_FAILURE;
  }
  switch (u32Addr) {
  case AE_BYPASS:
    u32Value = stExpAttr.bByPass;
    stExpAttr.bByPass = CLIP3(u32Data, XM_FALSE, XM_TRUE);
    break;
  case AE_OPTYPE:
    u32Value = stExpAttr.enOpType;
    stExpAttr.enOpType = CLIP3(u32Data, OP_TYPE_AUTO, OP_TYPE_MANUAL);
    break;
  case AE_MANUAL_AGAIN:
    u32Value = stExpAttr.stManual.s32AGain;
    stExpAttr.stManual.s32AGain = CLIP3(u32Data, 0x400, 0xFFFFFFFF);
    break;
  case AE_MANUAL_DGAIN:
    u32Value = stExpAttr.stManual.s32DGain;
    stExpAttr.stManual.s32DGain = CLIP3(u32Data, 0x400, 0xFFFFFFFF);
    break;
  case AE_MANUAL_ISPGAIN:
    u32Value = stExpAttr.stManual.s32IspGain;
    stExpAttr.stManual.s32IspGain = CLIP3(u32Data, 0x400, 0xFFFF);
    break;
  case AE_MANUAL_EXPTIMER:
    u32Value = stExpAttr.stManual.u32ExpLine;
    stExpAttr.stManual.u32ExpLine = CLIP3(u32Data, 0, 0xFFFF);
    break;
  case AE_MANUAL_EXPENABLE:
    u32Value = stExpAttr.stManual.bManualExpLineEnable;
    stExpAttr.stManual.bManualExpLineEnable = CLIP3(u32Data, XM_FALSE, XM_TRUE);
    break;
  case AE_MANUAL_AGENABLE:
    u32Value = stExpAttr.stManual.bManualAGainEnable;
    stExpAttr.stManual.bManualAGainEnable = CLIP3(u32Data, XM_FALSE, XM_TRUE);
    break;
  case AE_MANUAL_DGENABLE:
    u32Value = stExpAttr.stManual.bManualDGainEnable;
    stExpAttr.stManual.bManualDGainEnable = CLIP3(u32Data, XM_FALSE, XM_TRUE);
    break;
  case AE_MANUAL_ISPDGENABLE:
    u32Value = stExpAttr.stManual.bManualIspGainEnable;
    stExpAttr.stManual.bManualIspGainEnable = CLIP3(u32Data, XM_FALSE, XM_TRUE);
    break;
  case AE_RANGEEXP_MAX:
    u32Value = stExpAttr.stAuto.stExpTimeRange.u32Max;
    u32Data =
        CLIP3(u32Data, stExpAttr.stAuto.stExpTimeRange.u32Min, 0xFFFFFFFF);
    stExpAttr.stAuto.stExpTimeRange.u32Max = u32Data;
    break;
  case AE_RANGEEXP_MIN:
    u32Value = stExpAttr.stAuto.stExpTimeRange.u32Min;
    u32Data = CLIP3(u32Data, 0, stExpAttr.stAuto.stExpTimeRange.u32Max);
    stExpAttr.stAuto.stExpTimeRange.u32Min = u32Data;
    break;
  case AE_RANGEAGAIN_MAX:
    u32Value = stExpAttr.stAuto.stAGainRange.u32Max;
    u32Data = CLIP3(u32Data, stExpAttr.stAuto.stAGainRange.u32Min, 0xFFFFFFFF);
    stExpAttr.stAuto.stAGainRange.u32Max = u32Data;
    break;
  case AE_RANGEAGAIN_MIN:
    u32Value = stExpAttr.stAuto.stAGainRange.u32Min;
    u32Data = CLIP3(u32Data, 0x400, stExpAttr.stAuto.stAGainRange.u32Max);
    stExpAttr.stAuto.stAGainRange.u32Min = u32Data;
    break;
  case AE_RANGEDGAIN_MIN:
    u32Value = stExpAttr.stAuto.stDGainRange.u32Min;
    u32Data = CLIP3(u32Data, 0x400, stExpAttr.stAuto.stDGainRange.u32Max);
    stExpAttr.stAuto.stDGainRange.u32Min = u32Data;
    break;
  case AE_RANGEDGAIN_MAX:
    u32Value = stExpAttr.stAuto.stDGainRange.u32Max;
    u32Data = CLIP3(u32Data, stExpAttr.stAuto.stDGainRange.u32Min, 0xFFFFFFFF);
    stExpAttr.stAuto.stDGainRange.u32Max = u32Data;
    break;
  case AE_RANGEISPDGAIN_MIN:
    u32Value = stExpAttr.stAuto.stISPDGainRange.u32Min;
    u32Data = CLIP3(u32Data, 0x400, stExpAttr.stAuto.stISPDGainRange.u32Max);
    stExpAttr.stAuto.stISPDGainRange.u32Min = u32Data;
    break;
  case AE_RANGEISPDGAIN_MAX:
    u32Value = stExpAttr.stAuto.stISPDGainRange.u32Max;
    u32Data =
        CLIP3(u32Data, stExpAttr.stAuto.stISPDGainRange.u32Min, 0xFFFFFFFF);
    stExpAttr.stAuto.stISPDGainRange.u32Max = u32Data;
    break;
  case AE_RANGESYSDGAIN_MIN:
    u32Value = stExpAttr.stAuto.stSysGainRange.u32Min;
    u32Data = CLIP3(u32Data, 0x400, stExpAttr.stAuto.stSysGainRange.u32Max);
    stExpAttr.stAuto.stSysGainRange.u32Min = u32Data;
    break;
  case AE_RANGESYSDGAIN_MAX:
    u32Value = stExpAttr.stAuto.stSysGainRange.u32Max;
    u32Data =
        CLIP3(u32Data, stExpAttr.stAuto.stSysGainRange.u32Min, 0xFFFFFFFF);
    stExpAttr.stAuto.stSysGainRange.u32Max = u32Data;
    break;
  case AE_GAINTHRESHOLD:
    u32Value = stExpAttr.stAuto.u32GainThreshold;
    u32Data = CLIP3(u32Data, 0x400, 0xFFFFFFFF);
    stExpAttr.stAuto.u32GainThreshold = u32Data;
    break;
  case AE_SPEED:
    u32Value = stExpAttr.stAuto.u8Speed;
    u32Data = CLIP3(u32Data, 0, 255);
    stExpAttr.stAuto.u8Speed = u32Data;
    break;
  case AE_TOLERANCE:
    u32Value = stExpAttr.stAuto.u8Tolerance;
    u32Data = CLIP3(u32Data, 0, 255);
    stExpAttr.stAuto.u8Tolerance = u32Data;
    break;
  case AE_COMPENSATION:
    u32Value = stExpAttr.stAuto.u8Compensation;
    u32Data = CLIP3(u32Data, 0, 255);
    stExpAttr.stAuto.u8Compensation = u32Data;
    break;
  case AE_AESTRATEGYMODE:
    u32Value = stExpAttr.stAuto.enAEStrategyMode;
    u32Data = CLIP3(u32Data, AE_EXP_NORMAL, AE_STRATEGY_MODE_BUTT - 1);
    stExpAttr.stAuto.enAEStrategyMode = u32Data;
    break;
  case AE_HISTRATIOSLOPE:
    u32Value = stExpAttr.stAuto.u16HistRatioSlope;
    u32Data = CLIP3(u32Data, 0, 0xFF);
    stExpAttr.stAuto.u16HistRatioSlope = u32Data;
    break;
  case AE_MAXHISTOFST:
    u32Value = stExpAttr.stAuto.u8MaxHistOffset;
    u32Data = CLIP3(u32Data, 0, 0xFF);
    stExpAttr.stAuto.u8MaxHistOffset = u32Data;
    break;
  case AE_AEMODE:
    u32Value = stExpAttr.stAuto.enAEMode;
    u32Data = CLIP3(u32Data, AE_MODE_LOW_NOISE, AE_MODE_BUTT - 1);
    stExpAttr.stAuto.enAEMode = u32Data;
    break;
  case AE_DEFLICKERENABLE:
    u32Value = stExpAttr.stAuto.stAntiflicker.bEnable;
    u32Data = CLIP3(u32Data, XM_FALSE, XM_TRUE);
    stExpAttr.stAuto.stAntiflicker.bEnable = u32Data;
    break;
  case AE_DEFLICKERFRE:
    u32Value = stExpAttr.stAuto.stAntiflicker.u8Frequency;
    u32Data = CLIP3(u32Data, 50, 60);
    stExpAttr.stAuto.stAntiflicker.u8Frequency = u32Data;
    break;
  case AE_DEFLICKERMODE:
    u32Value = stExpAttr.stAuto.stAntiflicker.enMode;
    u32Data =
        CLIP3(u32Data, ISP_ANTIFLICKER_MODE_0, ISP_ANTIFLICKER_MODE_BUTT - 1);
    stExpAttr.stAuto.stAntiflicker.enMode = u32Data;
    break;
  case AE_ISPGAINEX:
    u32Value = stExpAttr.stAuto.s32IspGainEx;
    u32Data = CLIP3(u32Data, 0x400, 65535);
    stExpAttr.stAuto.s32IspGainEx = u32Data;
    break;
  case AE_SHUTTER:
    u32Value = stInnerStateInfo.u32ExposureTime;
    break;
  case AE_AGAIN:
    u32Value = stInnerStateInfo.u32AnalogGain;
    break;
  case AE_DGAIN:
    u32Value = stInnerStateInfo.u32DigitalGain;
    break;
  case AE_ISPGAIN:
    u32Value = stInnerStateInfo.u32IspDGain;
    break;
  case AE_ALLGAIN:
    u32Value = stInnerStateInfo.u32AnalogGain;
    u32Value = u32Value * stInnerStateInfo.u32DigitalGain / 1024;
    u32Value = u32Value * stInnerStateInfo.u32IspDGain / 1024;
    break;
  case AE_EXPOSURE:
    u32Value = stInnerStateInfo.u32Exposure;
    break;
  case AE_AVE_LUM:
    u32Value = stInnerStateInfo.u8AveLum;
    break;
  case AE_EXP_HIST16_0:
  case AE_EXP_HIST16_1:
  case AE_EXP_HIST16_2:
  case AE_EXP_HIST16_3:
  case AE_EXP_HIST16_4:
  case AE_EXP_HIST16_5:
  case AE_EXP_HIST16_6:
  case AE_EXP_HIST16_7:
  case AE_EXP_HIST16_8:
  case AE_EXP_HIST16_9:
  case AE_EXP_HIST16_10:
  case AE_EXP_HIST16_11:
  case AE_EXP_HIST16_12:
  case AE_EXP_HIST16_13:
  case AE_EXP_HIST16_14:
  case AE_EXP_HIST16_15:
    u32Value = (u32Addr - AE_EXP_HIST16_0) / 4;
    u32Value = stInnerStateInfo.u16AE_Hist16Value[u32Value];
    break;
  case AE_HIST_TGT:
    u32Value = stExpStatistic.u8HisTarget;
    break;
  case AE_HIST_NOW:
    u32Value = stExpStatistic.u8AveLum;
    break;
  case AE_HIST_ERR:
    u32Value = stExpStatistic.s16HistError;
    break;
  case AE_TGT_HIST16_0:
  case AE_TGT_HIST16_1:
  case AE_TGT_HIST16_2:
  case AE_TGT_HIST16_3:
  case AE_TGT_HIST16_4:
  case AE_TGT_HIST16_5:
  case AE_TGT_HIST16_6:
  case AE_TGT_HIST16_7:
  case AE_TGT_HIST16_8:
  case AE_TGT_HIST16_9:
  case AE_TGT_HIST16_10:
  case AE_TGT_HIST16_11:
  case AE_TGT_HIST16_12:
  case AE_TGT_HIST16_13:
  case AE_TGT_HIST16_14:
  case AE_TGT_HIST16_15:
    u32Value = stExpStatistic.u8ExpHistTarget[(u32Addr - AE_TGT_HIST16_0) / 4];
    u32Data = CLIP3(u32Data, 0, 0xFF);
    stExpStatistic.u8ExpHistTarget[(u32Addr - AE_TGT_HIST16_0) / 4] = u32Data;
    break;
  case AE_DELAY_B2D:
    u32Value = stExpAttr.stAuto.stAEDelayAttr.u16WhiteDelayFrame;
    u32Data = CLIP3(u32Data, 0, 0xFFFF);
    stExpAttr.stAuto.stAEDelayAttr.u16WhiteDelayFrame = u32Data;
    break;
  case AE_DELAY_D2B:
    u32Value = stExpAttr.stAuto.stAEDelayAttr.u16BlackDelayFrame;
    u32Data = CLIP3(u32Data, 0, 0xFFFF);
    stExpAttr.stAuto.stAEDelayAttr.u16BlackDelayFrame = u32Data;
    break;
  case AE_IS_MAX:
    u32Value = (XM_U32)(stInnerStateInfo.bExposureIsMAX);
    break;
  default:
    if ((u32Addr >= AE_LUM_WIN0) &&
        (u32Addr < AE_LUM_WIN0 + AE_ZONE_ROW * AE_ZONE_COLUMN)) {
      ISP_AE_STAT_S stAeStat;
      u32Addr -= AE_LUM_WIN0;
      if (XM_MPI_ISP_GetStatisticsAE(0, &stAeStat) != XM_SUCCESS) {
        putstr(0, "XM_MPI_ISP_GetStatisticsAE failed!\r\n");
        return XM_FAILURE;
      }
      XM_U32 i, j;
      for (u32Data = 0, i = 0; i < AE_ZONE_ROW; i++) {
        for (j = 0; j < AE_ZONE_COLUMN; j++) {
          u32Data += stAeStat.au32MeteringWin[i][j];
        }
      }
      u32Data = u32Data / (AE_ZONE_ROW * AE_ZONE_COLUMN);
      if (u32Addr == 0) {
        u32Value = u32Data;
      } else {
        u32Value = stAeStat.au32MeteringWin[u32Addr / AE_ZONE_COLUMN]
                                           [u32Addr % AE_ZONE_COLUMN];
      }

    } else {
      u32Value = 0xFFFFFFFF;
    }
    break;
  }

  if (u8Mode == 1) // Write
  {
    s32Ret = XM_MPI_ISP_SetExposureAttr(ISP_USED, &stExpAttr);
    if (s32Ret != XM_SUCCESS) {
      putstr(0, "XM_MPI_ISP_SetExposureAttr failed!\r\n");
      return XM_FAILURE;
    }

    s32Ret = XM_MPI_ISP_SetExpStaInfo(ISP_USED, &stExpStatistic);
    if (s32Ret != XM_SUCCESS) {
      putstr(0, "XM_MPI_ISP_SetExpStaInfo failed!\r\n");
      return XM_FAILURE;
    }
  }
  *pu32Data = u32Value;
  return XM_SUCCESS;
}

static XM_S32 IspDebugNr(unsigned char u8Mode, unsigned int u32Addr,
                         XM_U32 *pu32Data) {
  ISP_2DNR_ATTR_S st2DNRAttr;
  XM_S32 s32Ret;
  XM_U32 u32Data, u32Value;

  u32Data = *pu32Data;
  u32Value = 0xFFFFFFFF;

  // 2DNr
  {
    s32Ret = XM_MPI_ISP_GetNRAttr(&st2DNRAttr);
    if (s32Ret != XM_SUCCESS) {
      putstr(0, "XM_MPI_ISP_GetNRAttr failed!\r\n");
      return XM_FAILURE;
    }
    switch (u32Addr) {
    case NR2D_ENABLE:
      u32Value = st2DNRAttr.bEnable;
      st2DNRAttr.bEnable = CLIP3(u32Data, XM_FALSE, XM_TRUE);
      break;
    case NR2D_OPTYPE:
      u32Value = st2DNRAttr.enOpType;
      st2DNRAttr.enOpType = CLIP3(u32Data, OP_TYPE_AUTO, OP_TYPE_BUTT - 1);
      break;
    case NR2D_GLOBAL:
      u32Value = st2DNRAttr.stAuto.u8GlobalSth;
      st2DNRAttr.stAuto.u8GlobalSth = CLIP3(u32Data, 0, 0xFF);
      break;
    case NR2D_MANUAL_THR:
      u32Value = st2DNRAttr.stManual.u8Thresh;
      st2DNRAttr.stManual.u8Thresh = CLIP3(u32Data, 0, 0xFF);
      break;
    case NR2D_AUTO_THR0:
      u32Value = st2DNRAttr.stAuto.au8Thresh[0];
      st2DNRAttr.stAuto.au8Thresh[0] = CLIP3(u32Data, 0, 0xFF);
      break;
    case NR2D_AUTO_THR1:
      u32Value = st2DNRAttr.stAuto.au8Thresh[1];
      st2DNRAttr.stAuto.au8Thresh[1] = CLIP3(u32Data, 0, 0xFF);
      break;
    case NR2D_AUTO_THR2:
      u32Value = st2DNRAttr.stAuto.au8Thresh[2];
      st2DNRAttr.stAuto.au8Thresh[2] = CLIP3(u32Data, 0, 0xFF);
      break;
    case NR2D_AUTO_THR3:
      u32Value = st2DNRAttr.stAuto.au8Thresh[3];
      st2DNRAttr.stAuto.au8Thresh[3] = CLIP3(u32Data, 0, 0xFF);
      break;
    case NR2D_AUTO_THR4:
      u32Value = st2DNRAttr.stAuto.au8Thresh[4];
      st2DNRAttr.stAuto.au8Thresh[4] = CLIP3(u32Data, 0, 0xFF);
      break;
    case NR2D_AUTO_THR5:
      u32Value = st2DNRAttr.stAuto.au8Thresh[5];
      st2DNRAttr.stAuto.au8Thresh[5] = CLIP3(u32Data, 0, 0xFF);
      break;
    case NR2D_AUTO_THR6:
      u32Value = st2DNRAttr.stAuto.au8Thresh[6];
      st2DNRAttr.stAuto.au8Thresh[6] = CLIP3(u32Data, 0, 0xFF);
      break;
    case NR2D_AUTO_THR7:
      u32Value = st2DNRAttr.stAuto.au8Thresh[7];
      st2DNRAttr.stAuto.au8Thresh[7] = CLIP3(u32Data, 0, 0xFF);
      break;
    case NR2D_AUTO_THR8:
      u32Value = st2DNRAttr.stAuto.au8Thresh[8];
      st2DNRAttr.stAuto.au8Thresh[8] = CLIP3(u32Data, 0, 0xFF);
      break;
    case NR2D_AUTO_THR9:
      u32Value = st2DNRAttr.stAuto.au8Thresh[9];
      st2DNRAttr.stAuto.au8Thresh[9] = CLIP3(u32Data, 0, 0xFF);
      break;
    case NR2D_AUTO_THR10:
      u32Value = st2DNRAttr.stAuto.au8Thresh[10];
      st2DNRAttr.stAuto.au8Thresh[10] = CLIP3(u32Data, 0, 0xFF);
      break;
    case NR2D_AUTO_THR11:
      u32Value = st2DNRAttr.stAuto.au8Thresh[11];
      st2DNRAttr.stAuto.au8Thresh[11] = CLIP3(u32Data, 0, 0xFF);
      break;
    case NR2D_AUTO_THR12:
      u32Value = st2DNRAttr.stAuto.au8Thresh[12];
      st2DNRAttr.stAuto.au8Thresh[12] = CLIP3(u32Data, 0, 0xFF);
      break;
    case NR2D_AUTO_THR13:
      u32Value = st2DNRAttr.stAuto.au8Thresh[13];
      st2DNRAttr.stAuto.au8Thresh[13] = CLIP3(u32Data, 0, 0xFF);
      break;
    case NR2D_AUTO_THR14:
      u32Value = st2DNRAttr.stAuto.au8Thresh[14];
      st2DNRAttr.stAuto.au8Thresh[14] = CLIP3(u32Data, 0, 0xFF);
      break;
    case NR2D_AUTO_THR15:
      u32Value = st2DNRAttr.stAuto.au8Thresh[15];
      st2DNRAttr.stAuto.au8Thresh[15] = CLIP3(u32Data, 0, 0xFF);
      break;
    default:
      break;
    }

    if (u8Mode == 1) // Write
    {
      s32Ret = XM_MPI_ISP_SetNRAttr(&st2DNRAttr);
      if (s32Ret != XM_SUCCESS) {
        putstr(0, "XM_MPI_ISP_SetNRAttr failed!\r\n");
        return XM_FAILURE;
      }
    }
  }

  *pu32Data = u32Value;
  return XM_SUCCESS;
}

static XM_S32 IspDebugNr3D(unsigned char u8Mode, unsigned int u32Addr,
                           XM_U32 *pu32Data) {
  ISP_3DNR_ATTR_S st3DNRAttr;
  ISP_NR_INFO_S stNRInfo;
  XM_S32 s32Ret;
  XM_U32 u32Data, u32Value;

  u32Data = *pu32Data;
  u32Value = 0xFFFFFFFF;
  // 3DNr
  s32Ret = XM_MPI_ISP_Get3DNrAttr(&st3DNRAttr);
  if (s32Ret != XM_SUCCESS) {
    putstr(0, "XM_MPI_ISP_Get3DNrAttr failed!\r\n");
    return XM_FAILURE;
  }
  s32Ret = XM_MPI_ISP_GetNrInfo(&stNRInfo);
  if (s32Ret != XM_SUCCESS) {
    putstr(0, "XM_MPI_ISP_GetNrInfo failed!\r\n");
    return XM_FAILURE;
  }
  switch (u32Addr) {
  case NR3D_ENABLE:
    u32Value = st3DNRAttr.bEnable;
    st3DNRAttr.bEnable = CLIP3(u32Data, XM_FALSE, XM_TRUE);
    break;
  case NR3D_OPTYPE:
    u32Value = st3DNRAttr.enOpType;
    st3DNRAttr.enOpType = CLIP3(u32Data, OP_TYPE_AUTO, OP_TYPE_BUTT - 1);
    break;
  case NR3D_MANUAL_TF:
    u32Value = st3DNRAttr.stManual.u8TfStrength;
    st3DNRAttr.stManual.u8TfStrength = CLIP3(u32Data, 0, 0xFF);
    break;
  case NR3D_MANUAL_SF:
    u32Value = st3DNRAttr.stManual.u8SfStrength;
    st3DNRAttr.stManual.u8SfStrength = CLIP3(u32Data, 0, 0xFF);
    break;
  case NR3D_GLOBAL:
    u32Value = st3DNRAttr.stAuto.u8GlobalSth;
    st3DNRAttr.stAuto.u8GlobalSth = CLIP3(u32Data, 0, 0xFF);
    break;
  case NR3D_TF_NOW:
    u32Value = stNRInfo.au8Nr[0];
    break;
  case NR3D_SF_NOW:
    u32Value = stNRInfo.au8Nr[1];
    break;
  case NR3D_AUTO_TF0:
    u32Value = st3DNRAttr.stAuto.au8TfStrength[0];
    st3DNRAttr.stAuto.au8TfStrength[0] = CLIP3(u32Data, 0, 0xFF);
    break;
  case NR3D_AUTO_TF1:
    u32Value = st3DNRAttr.stAuto.au8TfStrength[1];
    st3DNRAttr.stAuto.au8TfStrength[1] = CLIP3(u32Data, 0, 0xFF);
    break;
  case NR3D_AUTO_TF2:
    u32Value = st3DNRAttr.stAuto.au8TfStrength[2];
    st3DNRAttr.stAuto.au8TfStrength[2] = CLIP3(u32Data, 0, 0xFF);
    break;
  case NR3D_AUTO_TF3:
    u32Value = st3DNRAttr.stAuto.au8TfStrength[3];
    st3DNRAttr.stAuto.au8TfStrength[3] = CLIP3(u32Data, 0, 0xFF);
    break;
  case NR3D_AUTO_TF4:
    u32Value = st3DNRAttr.stAuto.au8TfStrength[4];
    st3DNRAttr.stAuto.au8TfStrength[4] = CLIP3(u32Data, 0, 0xFF);
    break;
  case NR3D_AUTO_TF5:
    u32Value = st3DNRAttr.stAuto.au8TfStrength[5];
    st3DNRAttr.stAuto.au8TfStrength[5] = CLIP3(u32Data, 0, 0xFF);
    break;
  case NR3D_AUTO_TF6:
    u32Value = st3DNRAttr.stAuto.au8TfStrength[6];
    st3DNRAttr.stAuto.au8TfStrength[6] = CLIP3(u32Data, 0, 0xFF);
    break;
  case NR3D_AUTO_TF7:
    u32Value = st3DNRAttr.stAuto.au8TfStrength[7];
    st3DNRAttr.stAuto.au8TfStrength[7] = CLIP3(u32Data, 0, 0xFF);
    break;
  case NR3D_AUTO_TF8:
    u32Value = st3DNRAttr.stAuto.au8TfStrength[8];
    st3DNRAttr.stAuto.au8TfStrength[8] = CLIP3(u32Data, 0, 0xFF);
    break;
  case NR3D_AUTO_TF9:
    u32Value = st3DNRAttr.stAuto.au8TfStrength[9];
    st3DNRAttr.stAuto.au8TfStrength[9] = CLIP3(u32Data, 0, 0xFF);
    break;
  case NR3D_AUTO_TF10:
    u32Value = st3DNRAttr.stAuto.au8TfStrength[10];
    st3DNRAttr.stAuto.au8TfStrength[10] = CLIP3(u32Data, 0, 0xFF);
    break;
  case NR3D_AUTO_TF11:
    u32Value = st3DNRAttr.stAuto.au8TfStrength[11];
    st3DNRAttr.stAuto.au8TfStrength[11] = CLIP3(u32Data, 0, 0xFF);
    break;
  case NR3D_AUTO_TF12:
    u32Value = st3DNRAttr.stAuto.au8TfStrength[12];
    st3DNRAttr.stAuto.au8TfStrength[12] = CLIP3(u32Data, 0, 0xFF);
    break;
  case NR3D_AUTO_TF13:
    u32Value = st3DNRAttr.stAuto.au8TfStrength[13];
    st3DNRAttr.stAuto.au8TfStrength[13] = CLIP3(u32Data, 0, 0xFF);
    break;
  case NR3D_AUTO_TF14:
    u32Value = st3DNRAttr.stAuto.au8TfStrength[14];
    st3DNRAttr.stAuto.au8TfStrength[14] = CLIP3(u32Data, 0, 0xFF);
    break;
  case NR3D_AUTO_TF15:
    u32Value = st3DNRAttr.stAuto.au8TfStrength[15];
    st3DNRAttr.stAuto.au8TfStrength[15] = CLIP3(u32Data, 0, 0xFF);
    break;
  case NR3D_AUTO_SF0:
    u32Value = st3DNRAttr.stAuto.au8SfStrength[0];
    st3DNRAttr.stAuto.au8SfStrength[0] = CLIP3(u32Data, 0, 0xFF);
    break;
  case NR3D_AUTO_SF1:
    u32Value = st3DNRAttr.stAuto.au8SfStrength[1];
    st3DNRAttr.stAuto.au8SfStrength[1] = CLIP3(u32Data, 0, 0xFF);
    break;
  case NR3D_AUTO_SF2:
    u32Value = st3DNRAttr.stAuto.au8SfStrength[2];
    st3DNRAttr.stAuto.au8SfStrength[2] = CLIP3(u32Data, 0, 0xFF);
    break;
  case NR3D_AUTO_SF3:
    u32Value = st3DNRAttr.stAuto.au8SfStrength[3];
    st3DNRAttr.stAuto.au8SfStrength[3] = CLIP3(u32Data, 0, 0xFF);
    break;
  case NR3D_AUTO_SF4:
    u32Value = st3DNRAttr.stAuto.au8SfStrength[4];
    st3DNRAttr.stAuto.au8SfStrength[4] = CLIP3(u32Data, 0, 0xFF);
    break;
  case NR3D_AUTO_SF5:
    u32Value = st3DNRAttr.stAuto.au8SfStrength[5];
    st3DNRAttr.stAuto.au8SfStrength[5] = CLIP3(u32Data, 0, 0xFF);
    break;
  case NR3D_AUTO_SF6:
    u32Value = st3DNRAttr.stAuto.au8SfStrength[6];
    st3DNRAttr.stAuto.au8SfStrength[6] = CLIP3(u32Data, 0, 0xFF);
    break;
  case NR3D_AUTO_SF7:
    u32Value = st3DNRAttr.stAuto.au8SfStrength[7];
    st3DNRAttr.stAuto.au8SfStrength[7] = CLIP3(u32Data, 0, 0xFF);
    break;
  case NR3D_AUTO_SF8:
    u32Value = st3DNRAttr.stAuto.au8SfStrength[8];
    st3DNRAttr.stAuto.au8SfStrength[8] = CLIP3(u32Data, 0, 0xFF);
    break;
  case NR3D_AUTO_SF9:
    u32Value = st3DNRAttr.stAuto.au8SfStrength[9];
    st3DNRAttr.stAuto.au8SfStrength[9] = CLIP3(u32Data, 0, 0xFF);
    break;
  case NR3D_AUTO_SF10:
    u32Value = st3DNRAttr.stAuto.au8SfStrength[10];
    st3DNRAttr.stAuto.au8SfStrength[10] = CLIP3(u32Data, 0, 0xFF);
    break;
  case NR3D_AUTO_SF11:
    u32Value = st3DNRAttr.stAuto.au8SfStrength[11];
    st3DNRAttr.stAuto.au8SfStrength[11] = CLIP3(u32Data, 0, 0xFF);
    break;
  case NR3D_AUTO_SF12:
    u32Value = st3DNRAttr.stAuto.au8SfStrength[12];
    st3DNRAttr.stAuto.au8SfStrength[12] = CLIP3(u32Data, 0, 0xFF);
    break;
  case NR3D_AUTO_SF13:
    u32Value = st3DNRAttr.stAuto.au8SfStrength[13];
    st3DNRAttr.stAuto.au8SfStrength[13] = CLIP3(u32Data, 0, 0xFF);
    break;
  case NR3D_AUTO_SF14:
    u32Value = st3DNRAttr.stAuto.au8SfStrength[14];
    st3DNRAttr.stAuto.au8SfStrength[14] = CLIP3(u32Data, 0, 0xFF);
    break;
  case NR3D_AUTO_SF15:
    u32Value = st3DNRAttr.stAuto.au8SfStrength[15];
    st3DNRAttr.stAuto.au8SfStrength[15] = CLIP3(u32Data, 0, 0xFF);
    break;
  default:
    break;
  }

  if (u8Mode == 1) // Write
  {
    s32Ret = XM_MPI_ISP_Set3DNrAttr(&st3DNRAttr);
    if (s32Ret != XM_SUCCESS) {
      putstr(0, "XM_MPI_ISP_Set3DNrAttr failed!\r\n");
      return XM_FAILURE;
    }
  }
  *pu32Data = u32Value;
  return XM_SUCCESS;
}

static XM_S32 IspDebugYcNr(unsigned char u8Mode, unsigned int u32Addr,
                           XM_U32 *pu32Data) {
  return XM_SUCCESS;
}

#if 0
static XM_S32 IspDebugStDpc(unsigned char u8Mode, unsigned int u32Addr,
                            XM_U32 *pu32Data) {
  ISP_STDPC_ATTR_S stStDPAttr;
  XM_S32 s32Ret;
  XM_U32 u32Data, u32Value;

  u32Data = *pu32Data;
  u32Value = 0xFFFFFFFF;
  s32Ret = XM_MPI_ISP_GetStDefectPixelAttr(&stStDPAttr);
  if (s32Ret != XM_SUCCESS) {
    putstr(0, "XM_MPI_ISP_GetStDefectPixelAttr failed!\r\n");
    return XM_FAILURE;
  }
  switch (u32Addr) {

  case SDPC_BP_THR_MIN:
    u32Value = stStDPAttr.u16BadPixelThreshMin;
    stStDPAttr.u16BadPixelThreshMin = CLIP3(u32Data, 0, 0xFFF);
    break;
  case SDPC_BP_THR_MAX:
    u32Value = stStDPAttr.u16BadPixelThreshMax;
    stStDPAttr.u16BadPixelThreshMax = CLIP3(u32Data, 0, 0xFFF);
    break;
  case SDPC_BP_THR:
    u32Value = stStDPAttr.u16BadPixelThresh;
    stStDPAttr.u16BadPixelThresh = CLIP3(u32Data, 0, 0xFFF);
    break;
  default:
    break;
  }

  if (u8Mode == 1) // Write
  {
    s32Ret = XM_MPI_ISP_SetStDefectPixelAttr(&stStDPAttr);
    if (s32Ret != XM_SUCCESS) {
      putstr(0, "XM_MPI_ISP_SetStDefectPixelAttr failed!\r\n");
      return XM_FAILURE;
    }
  }

  *pu32Data = u32Value;
  return XM_SUCCESS;
}
#endif

static XM_S32 IspDebugDpc(unsigned char u8Mode, unsigned int u32Addr,
                          XM_U32 *pu32Data) {
  ISP_DYDPC_ATTR_S stDyDPAttr;
  XM_S32 s32Ret;
  XM_U32 u32Data, u32Value;

  u32Data = *pu32Data;
  u32Value = 0xFFFFFFFF;
  s32Ret = XM_MPI_ISP_GetDyDefectPixelAttr(&stDyDPAttr);
  if (s32Ret != XM_SUCCESS) {
    putstr(0, "XM_MPI_ISP_GetDyDefectPixelAttr failed!\r\n");
    return XM_FAILURE;
  }
  switch (u32Addr) {
  case DDPC_OP_TYPE:
    u32Value = stDyDPAttr.enOpType;
    stDyDPAttr.enOpType = CLIP3(u32Data, OP_TYPE_AUTO, OP_TYPE_BUTT - 1);
    break;
  case DDPC_MANUAL_STH:
    u32Value = stDyDPAttr.stManual.u8Sth;
    stDyDPAttr.stManual.u8Sth = CLIP3(u32Data, 0, 0xFF);
    break;

  case DDPC_AUTO_STH0:
    u32Value = stDyDPAttr.stAuto.au8Sth[0];
    stDyDPAttr.stAuto.au8Sth[0] = CLIP3(u32Data, 0, 0xFF);
    break;
  case DDPC_AUTO_STH1:
    u32Value = stDyDPAttr.stAuto.au8Sth[1];
    stDyDPAttr.stAuto.au8Sth[1] = CLIP3(u32Data, 0, 0xFF);
    break;
  case DDPC_AUTO_STH2:
    u32Value = stDyDPAttr.stAuto.au8Sth[2];
    stDyDPAttr.stAuto.au8Sth[2] = CLIP3(u32Data, 0, 0xFF);
    break;
  case DDPC_AUTO_STH3:
    u32Value = stDyDPAttr.stAuto.au8Sth[3];
    stDyDPAttr.stAuto.au8Sth[3] = CLIP3(u32Data, 0, 0xFF);
    break;
  case DDPC_AUTO_STH4:
    u32Value = stDyDPAttr.stAuto.au8Sth[4];
    stDyDPAttr.stAuto.au8Sth[4] = CLIP3(u32Data, 0, 0xFF);
    break;
  case DDPC_AUTO_STH5:
    u32Value = stDyDPAttr.stAuto.au8Sth[5];
    stDyDPAttr.stAuto.au8Sth[5] = CLIP3(u32Data, 0, 0xFF);
    break;
  case DDPC_AUTO_STH6:
    u32Value = stDyDPAttr.stAuto.au8Sth[6];
    stDyDPAttr.stAuto.au8Sth[6] = CLIP3(u32Data, 0, 0xFF);
    break;
  case DDPC_AUTO_STH7:
    u32Value = stDyDPAttr.stAuto.au8Sth[7];
    stDyDPAttr.stAuto.au8Sth[7] = CLIP3(u32Data, 0, 0xFF);
    break;
  case DDPC_AUTO_STH8:
    u32Value = stDyDPAttr.stAuto.au8Sth[8];
    stDyDPAttr.stAuto.au8Sth[8] = CLIP3(u32Data, 0, 0xFF);
    break;
  case DDPC_AUTO_STH9:
    u32Value = stDyDPAttr.stAuto.au8Sth[9];
    stDyDPAttr.stAuto.au8Sth[9] = CLIP3(u32Data, 0, 0xFF);
    break;
  case DDPC_AUTO_STH10:
    u32Value = stDyDPAttr.stAuto.au8Sth[10];
    stDyDPAttr.stAuto.au8Sth[10] = CLIP3(u32Data, 0, 0xFF);
    break;
  case DDPC_AUTO_STH11:
    u32Value = stDyDPAttr.stAuto.au8Sth[11];
    stDyDPAttr.stAuto.au8Sth[11] = CLIP3(u32Data, 0, 0xFF);
    break;
  case DDPC_AUTO_STH12:
    u32Value = stDyDPAttr.stAuto.au8Sth[12];
    stDyDPAttr.stAuto.au8Sth[12] = CLIP3(u32Data, 0, 0xFF);
    break;
  case DDPC_AUTO_STH13:
    u32Value = stDyDPAttr.stAuto.au8Sth[13];
    stDyDPAttr.stAuto.au8Sth[13] = CLIP3(u32Data, 0, 0xFF);
    break;
  case DDPC_AUTO_STH14:
    u32Value = stDyDPAttr.stAuto.au8Sth[14];
    stDyDPAttr.stAuto.au8Sth[14] = CLIP3(u32Data, 0, 0xFF);
    break;
  case DDPC_AUTO_STH15:
    u32Value = stDyDPAttr.stAuto.au8Sth[15];
    stDyDPAttr.stAuto.au8Sth[15] = CLIP3(u32Data, 0, 0xFF);
    break;
  default:
    break;
  }

  if (u8Mode == 1) // Write
  {
    s32Ret = XM_MPI_ISP_SetDyDefectPixelAttr(&stDyDPAttr);
    if (s32Ret != XM_SUCCESS) {
      putstr(0, "XM_MPI_ISP_SetDyDefectPixelAttr failed!\r\n");
      return XM_FAILURE;
    }
  }

  *pu32Data = u32Value;
  return XM_SUCCESS;
}

static XM_S32 IspDebugDRC(unsigned char u8Mode, unsigned int u32Addr,
                          XM_U32 *pu32Data) {
  ISP_DRC_ATTR_S stDRC;
  XM_S32 s32Ret;
  XM_U32 u32Data, u32Value;

  u32Data = *pu32Data;
  u32Value = 0xFFFFFFFF;
  s32Ret = XM_MPI_ISP_GetDRCAttr(0, &stDRC);
  if (s32Ret != XM_SUCCESS) {
    putstr(0, "XM_MPI_ISP_GetDRCAttr failed!\r\n");
    return XM_FAILURE;
  }
  switch (u32Addr) {
  case DRC_OP_TYPE:
    u32Value = stDRC.enOpType;
    stDRC.enOpType = CLIP3(u32Data, OP_TYPE_AUTO, OP_TYPE_BUTT - 1);
    break;
  case DRC_MANUAL_STH:
    u32Value = stDRC.stManual.u8Strength;
    stDRC.stManual.u8Strength = CLIP3(u32Data, 0, 0xFF);
    break;
  case DRC_GLOBAL_STH:
    u32Value = stDRC.u8GlobalSth;
    stDRC.u8GlobalSth = CLIP3(u32Data, 0, 0xFF);
    break;
  case DRC_AUTO_STH0:
    u32Value = stDRC.stAuto.au8Sth[0];
    stDRC.stAuto.au8Sth[0] = CLIP3(u32Data, 0, 0xFF);
    break;
  case DRC_AUTO_STH1:
    u32Value = stDRC.stAuto.au8Sth[1];
    stDRC.stAuto.au8Sth[1] = CLIP3(u32Data, 0, 0xFF);
    break;
  case DRC_AUTO_STH2:
    u32Value = stDRC.stAuto.au8Sth[2];
    stDRC.stAuto.au8Sth[2] = CLIP3(u32Data, 0, 0xFF);
    break;
  case DRC_AUTO_STH3:
    u32Value = stDRC.stAuto.au8Sth[3];
    stDRC.stAuto.au8Sth[3] = CLIP3(u32Data, 0, 0xFF);
    break;
  case DRC_AUTO_STH4:
    u32Value = stDRC.stAuto.au8Sth[4];
    stDRC.stAuto.au8Sth[4] = CLIP3(u32Data, 0, 0xFF);
    break;
  case DRC_AUTO_STH5:
    u32Value = stDRC.stAuto.au8Sth[5];
    stDRC.stAuto.au8Sth[5] = CLIP3(u32Data, 0, 0xFF);
    break;
  case DRC_AUTO_STH6:
    u32Value = stDRC.stAuto.au8Sth[6];
    stDRC.stAuto.au8Sth[6] = CLIP3(u32Data, 0, 0xFF);
    break;
  case DRC_AUTO_STH7:
    u32Value = stDRC.stAuto.au8Sth[7];
    stDRC.stAuto.au8Sth[7] = CLIP3(u32Data, 0, 0xFF);
    break;
  case DRC_AUTO_STH8:
    u32Value = stDRC.stAuto.au8Sth[8];
    stDRC.stAuto.au8Sth[8] = CLIP3(u32Data, 0, 0xFF);
    break;
  case DRC_AUTO_STH9:
    u32Value = stDRC.stAuto.au8Sth[9];
    stDRC.stAuto.au8Sth[9] = CLIP3(u32Data, 0, 0xFF);
    break;
  case DRC_AUTO_STH10:
    u32Value = stDRC.stAuto.au8Sth[10];
    stDRC.stAuto.au8Sth[10] = CLIP3(u32Data, 0, 0xFF);
    break;
  case DRC_AUTO_STH11:
    u32Value = stDRC.stAuto.au8Sth[11];
    stDRC.stAuto.au8Sth[11] = CLIP3(u32Data, 0, 0xFF);
    break;
  case DRC_AUTO_STH12:
    u32Value = stDRC.stAuto.au8Sth[12];
    stDRC.stAuto.au8Sth[12] = CLIP3(u32Data, 0, 0xFF);
    break;
  case DRC_AUTO_STH13:
    u32Value = stDRC.stAuto.au8Sth[13];
    stDRC.stAuto.au8Sth[13] = CLIP3(u32Data, 0, 0xFF);
    break;
  case DRC_AUTO_STH14:
    u32Value = stDRC.stAuto.au8Sth[14];
    stDRC.stAuto.au8Sth[14] = CLIP3(u32Data, 0, 0xFF);
    break;
  case DRC_AUTO_STH15:
    u32Value = stDRC.stAuto.au8Sth[15];
    stDRC.stAuto.au8Sth[15] = CLIP3(u32Data, 0, 0xFF);
    break;

  default:
    break;
  }

  if (u8Mode == 1) // Write
  {
    s32Ret = XM_MPI_ISP_SetDRCAttr(0, &stDRC);
    if (s32Ret != XM_SUCCESS) {
      putstr(0, "XM_MPI_ISP_SetDRCAttr failed!\r\n");
      return XM_FAILURE;
    }
  }

  *pu32Data = u32Value;
  return XM_SUCCESS;
}

static XM_S32 IspDebgAwbCal(unsigned char u8Mode, unsigned int u32Addr,
                            XM_U32 *pu32Data) {
  XM_S32 s32Ret;
  XM_U32 u32Data, u32Value;
  ISP_AWB_CALIBRATION_V2_S stWBCalAttr;
  if (u32Addr < AWB_DEG_CAL_A0 || u32Addr > AWB_DEG_INIT_GAIN3)
    return XM_FAILURE;
  u32Data = *pu32Data;
  u32Value = 0xFFFFFFFF;

  s32Ret = XM_MPI_ISP_GetWBCalAttr(&stWBCalAttr);
  if (s32Ret != XM_SUCCESS) {
    putstr(0, "XM_MPI_ISP_GetWBCalAttr failed!\r\n");
    return XM_FAILURE;
  }
  if (u32Addr < AWB_DEG_CAL_B0) {
    u32Value = stWBCalAttr.A[(u32Addr - AWB_DEG_CAL_A0) >> 2];
    stWBCalAttr.A[(u32Addr - AWB_DEG_CAL_A0) >> 2] = CLIP3(u32Data, 0, 0xFFFF);
  } else if (u32Addr < AWB_DEG_CAL_C0) {
    u32Value = stWBCalAttr.B[(u32Addr - AWB_DEG_CAL_B0) >> 2];
    stWBCalAttr.B[(u32Addr - AWB_DEG_CAL_B0) >> 2] = CLIP3(u32Data, 0, 0xFFFF);
  } else if (u32Addr < AWB_DEG_CAL_KEY) {
    u32Value = stWBCalAttr.C[(u32Addr - AWB_DEG_CAL_C0) >> 2];
    stWBCalAttr.C[(u32Addr - AWB_DEG_CAL_C0) >> 2] = CLIP3(u32Data, 0, 0xFFFF);
  } else if (u32Addr == AWB_DEG_CAL_KEY) {
    u32Value = stWBCalAttr.key;
    stWBCalAttr.key = CLIP3(u32Data, 0, 0xFFFF);
  } else if (u32Addr == AWB_DEG_CAL_X) {
    u32Value = stWBCalAttr.ini_x;
    stWBCalAttr.ini_x = CLIP3(u32Data, 0, 0xFFFF);
  } else if (u32Addr == AWB_DEG_CAL_Y) {
    u32Value = stWBCalAttr.ini_y;
    stWBCalAttr.ini_y = CLIP3(u32Data, 0, 0xFFFF);
  } else if (u32Addr == AWB_DEG_DIS_MIN) {
    u32Value = stWBCalAttr.dis_min;
    stWBCalAttr.dis_min = CLIP3(u32Data, 0, 0xFFFF);
  } else if (u32Addr == AWB_DEG_DIS_MAX) {
    u32Value = stWBCalAttr.dis_max;
    stWBCalAttr.dis_max = CLIP3(u32Data, 0, 0xFFFF);
  } else if (u32Addr < AWB_DEG_VAL0) {
    u32Value = stWBCalAttr.dis[(u32Addr - AWB_DEG_DIS0) >> 2];
    stWBCalAttr.dis[(u32Addr - AWB_DEG_DIS0) >> 2] = CLIP3(u32Data, 0, 0xFFFF);
  } else if (u32Addr < AWB_DEG_INIT_GAIN0) {
    u32Value = stWBCalAttr.val[(u32Addr - AWB_DEG_VAL0) >> 2];
    stWBCalAttr.val[(u32Addr - AWB_DEG_VAL0) >> 2] = CLIP3(u32Data, 0, 0xFFFF);
  } else {
    u32Value = stWBCalAttr.init_gain[(u32Addr - AWB_DEG_INIT_GAIN0) >> 2];
    stWBCalAttr.init_gain[(u32Addr - AWB_DEG_INIT_GAIN0) >> 2] =
        CLIP3(u32Data, 0, 0xFFFF);
  }
  if (u8Mode == 1) // Write
  {
    s32Ret = XM_MPI_ISP_SetWBCalAttr(&stWBCalAttr);
    if (s32Ret != XM_SUCCESS) {
      putstr(0, "XM_MPI_ISP_SetWBCalAttr failed!\r\n");
      return XM_FAILURE;
    }
  }
  *pu32Data = u32Value;
  return XM_SUCCESS;
}

static XM_S32 IspDebugAwb(unsigned char u8Mode, unsigned int u32Addr,
                          XM_U32 *pu32Data) {
  ISP_WB_ATTR_S stWBAttr;
  ISP_COLORMATRIX_ATTR_S stCCMAttr;
  ISP_WB_INFO_S stWBInfo;
  XM_S32 s32Ret;
  XM_U32 u32Data, u32Value;
  u32Data = *pu32Data;
  u32Value = 0xFFFFFFFF;
  s32Ret = XM_MPI_ISP_GetWBAttr(&stWBAttr);
  if (s32Ret != XM_SUCCESS) {
    putstr(0, "XM_MPI_ISP_GetWBAttr failed!\r\n");
    return XM_FAILURE;
  }

  s32Ret = XM_MPI_ISP_GetCCMAttr(&stCCMAttr);
  if (s32Ret != XM_SUCCESS) {
    putstr(0, "XM_MPI_ISP_GetWBAttr failed!\r\n");
    return XM_FAILURE;
  }
  s32Ret = XM_MPI_ISP_QueryWBInfo(&stWBInfo);
  if (s32Ret != XM_SUCCESS) {
    putstr(0, "XM_MPI_ISP_QueryWBInfo failed!\r\n");
    return XM_FAILURE;
  }

  switch (u32Addr) {
  case AWB_BYPASS:
    u32Value = stWBAttr.bByPass;
    stWBAttr.bByPass = CLIP3(u32Data, XM_FALSE, XM_TRUE);
    break;
  case AWB_OPTYPE:
    u32Value = stWBAttr.enOpType;
    stWBAttr.enOpType = CLIP3(u32Data, OP_TYPE_AUTO, OP_TYPE_BUTT - 1);
    break;
  case AWB_MANUAL_R:
    u32Value = stWBAttr.stManual.u16Rgain;
    stWBAttr.stManual.u16Rgain = CLIP3(u32Data, 0, 0xFFF);
    break;
  case AWB_MANUAL_G:
    u32Value = stWBAttr.stManual.u16Ggain;
    stWBAttr.stManual.u16Ggain = CLIP3(u32Data, 0, 0xFFF);
    break;
  case AWB_MANUAL_B:
    u32Value = stWBAttr.stManual.u16Bgain;
    stWBAttr.stManual.u16Bgain = CLIP3(u32Data, 0, 0xFFF);
    break;
  case AWB_AUTO_GM_OFST:
    u32Value = stWBAttr.stAuto.u8GmOfst;
    stWBAttr.stAuto.u8GmOfst = CLIP3(u32Data, 0, 0xFF);
    break;
  case AWB_AUTO_RB_OFST:
    u32Value = stWBAttr.stAuto.u8RbOfst;
    stWBAttr.stAuto.u8RbOfst = CLIP3(u32Data, 0, 0xFF);
    break;
#if 0
  case AWB_AUTO_R_OFST:
    u32Value = stWBAttr.stAuto.u8ROfst;
    stWBAttr.stAuto.u8ROfst = CLIP3(u32Data, 0, 0xFF);
    break;
  case AWB_AUTO_B_OFST:
    u32Value = stWBAttr.stAuto.u8BOfst;
    stWBAttr.stAuto.u8BOfst = CLIP3(u32Data, 0, 0xFF);
    break;
#endif
  case AWB_AUTO_SPEED2H:
    u32Value = stWBAttr.stAuto.u16SpeedToH;
    stWBAttr.stAuto.u16SpeedToH = CLIP3(u32Data, 0, 0xFFFF);
    break;
  case AWB_AUTO_SPEED2L:
    u32Value = stWBAttr.stAuto.u16SpeedToL;
    stWBAttr.stAuto.u16SpeedToL = CLIP3(u32Data, 0, 0xFFFF);
    break;
  case AWB_AUTO_CTMAX:
    u32Value = stWBAttr.stAuto.u8HighColorTemp;
    stWBAttr.stAuto.u8HighColorTemp = CLIP3(u32Data, 0, 0xFF);
    break;
  case AWB_AUTO_CTMIN:
    u32Value = stWBAttr.stAuto.u8LowColorTemp;
    stWBAttr.stAuto.u8LowColorTemp = CLIP3(u32Data, 0, 0xFF);
    break;
  case AWB_INFO_RGAIN:
    u32Value = stWBInfo.u16Rgain;
    break;
  case AWB_INFO_GGAIN:
    u32Value = stWBInfo.u16Ggain;
    break;
  case AWB_INFO_BGAIN:
    u32Value = stWBInfo.u16Bgain;
    break;
  case AWB_INFO_CT:
    u32Value = stWBInfo.u16ColorTemp;
    break;

  case AWB_INFO_GM_OFST:
    u32Value = stWBInfo.u8GmOfst;
    break;
  case AWB_INFO_RB_OFST:
    u32Value = stWBInfo.u8RbOfst;
    break;

  default:
    u32Value = 0;
    break;
  }
  if (u8Mode == 1) // Write
  {

    s32Ret = XM_MPI_ISP_SetWBAttr(&stWBAttr);
    if (s32Ret != XM_SUCCESS) {
      putstr(0, "XM_MPI_ISP_SetWBAttr failed!\r\n");
      return XM_FAILURE;
    }
  }
  *pu32Data = u32Value;
  return XM_SUCCESS;
}

// 针对PQTool
static XM_S32 IspDebugAwbPQTool(unsigned char u8Mode, unsigned int u32Addr,
                                XM_U32 *pu32Data) {
  ISP_WB_ATTR_S stWBAttr;
  ISP_AWB_CALIBRATION_V2_S stWBCalAttr;
  XM_S32 s32Ret;
  XM_U32 u32Data, u32Value;
  u32Data = *pu32Data;
  u32Value = 0xFFFFFFFF;
  s32Ret = XM_MPI_ISP_GetWBAttr(&stWBAttr);
  if (s32Ret != XM_SUCCESS) {
    putstr(0, "XM_MPI_ISP_GetWBAttr failed!\r\n");
    return XM_FAILURE;
  }
  s32Ret = XM_MPI_ISP_GetWBCalAttr(&stWBCalAttr);
  if (s32Ret != XM_SUCCESS) {
    putstr(0, "XM_MPI_ISP_GetWBCalAttr failed!\r\n");
    return XM_FAILURE;
  }
  switch (u32Addr) {
  case PQ_AWB_BYPASS:
    u32Value = stWBAttr.bByPass;
    stWBAttr.bByPass = CLIP3(u32Data, XM_FALSE, XM_TRUE);
    break;
  case PQ_AWB_OPTYPE:
    u32Value = stWBAttr.enOpType;
    stWBAttr.enOpType = CLIP3(u32Data, OP_TYPE_AUTO, OP_TYPE_BUTT - 1);
    break;
  case PQ_AWB_MANUAL_R:
    u32Value = stWBAttr.stManual.u16Rgain;
    stWBAttr.stManual.u16Rgain = CLIP3(u32Data, 0, 0xFFF);
    break;
  case PQ_AWB_MANUAL_G:
    u32Value = stWBAttr.stManual.u16Ggain;
    stWBAttr.stManual.u16Ggain = CLIP3(u32Data, 0, 0xFFF);
    break;
  case PQ_AWB_MANUAL_B:
    u32Value = stWBAttr.stManual.u16Bgain;
    stWBAttr.stManual.u16Bgain = CLIP3(u32Data, 0, 0xFFF);
    break;
  case PQ_AWB_AUTO_GM_OFST:
    u32Value = stWBAttr.stAuto.u8GmOfst;
    stWBAttr.stAuto.u8GmOfst = CLIP3(u32Data, 0, 0xFF);
    break;
  case PQ_AWB_AUTO_RB_OFST:
    u32Value = stWBAttr.stAuto.u8RbOfst;
    stWBAttr.stAuto.u8RbOfst = CLIP3(u32Data, 0, 0xFF);
    break;
  case PQ_AWB_AUTO_SPEED2H:
    u32Value = stWBAttr.stAuto.u16SpeedToH;
    stWBAttr.stAuto.u16SpeedToH = CLIP3(u32Data, 0, 0xFFFF);
    break;
  case PQ_AWB_AUTO_SPEED2L:
    u32Value = stWBAttr.stAuto.u16SpeedToL;
    stWBAttr.stAuto.u16SpeedToL = CLIP3(u32Data, 0, 0xFFFF);
    break;
  case PQ_AWB_AUTO_CTMAX:
    u32Value = stWBAttr.stAuto.u8HighColorTemp;
    stWBAttr.stAuto.u8HighColorTemp = CLIP3(u32Data, 0, 0xFF);
    break;
  case PQ_AWB_AUTO_CTMIN:
    u32Value = stWBAttr.stAuto.u8LowColorTemp;
    stWBAttr.stAuto.u8LowColorTemp = CLIP3(u32Data, 0, 0xFF);
    break;
  case PQ_AWB_DEG_CAL_KEY:
    u32Value = stWBCalAttr.key;
    stWBCalAttr.key = CLIP3(u32Data, 0, 0xFFFF);
    break;
  case PQ_AWB_DEG_CAL_X:
    u32Value = stWBCalAttr.ini_x;
    stWBCalAttr.ini_x = CLIP3(u32Data, 0, 0xFFFF);
    break;
  case PQ_AWB_DEG_CAL_Y:
    u32Value = stWBCalAttr.ini_y;
    stWBCalAttr.ini_y = CLIP3(u32Data, 0, 0xFFFF);
    break;
  case PQ_AWB_DEG_DIS_MIN:
    u32Value = stWBCalAttr.dis_min;
    stWBCalAttr.dis_min = CLIP3(u32Data, 0, 0xFFFF);
    break;
  case PQ_AWB_DEG_DIS_MAX:
    u32Value = stWBCalAttr.dis_max;
    stWBCalAttr.dis_max = CLIP3(u32Data, 0, 0xFFFF);
    break;
  default:
    if (u32Addr < PQ_AWB_DEG_CAL_B0) {
      u32Value = stWBCalAttr.A[(u32Addr - PQ_AWB_DEG_CAL_A0) >> 1];
      stWBCalAttr.A[(u32Addr - PQ_AWB_DEG_CAL_A0) >> 1] =
          CLIP3(u32Data & 0xFFFF, 0, 0xFFFF);
    } else if (u32Addr < PQ_AWB_DEG_CAL_C0) {
      u32Value = stWBCalAttr.B[(u32Addr - PQ_AWB_DEG_CAL_B0) >> 1];
      stWBCalAttr.B[(u32Addr - PQ_AWB_DEG_CAL_B0) >> 1] =
          CLIP3(u32Data & 0xFFFF, 0, 0xFFFF);
    } else if (u32Addr < PQ_AWB_DEG_CAL_KEY) {
      u32Value = stWBCalAttr.C[(u32Addr - PQ_AWB_DEG_CAL_C0) >> 1];
      stWBCalAttr.C[(u32Addr - PQ_AWB_DEG_CAL_C0) >> 1] =
          CLIP3(u32Data & 0xFFFF, 0, 0xFFFF);
    } else if (u32Addr < PQ_AWB_DEG_VAL0) {
      u32Value = stWBCalAttr.dis[(u32Addr - PQ_AWB_DEG_DIS0) >> 1];
      stWBCalAttr.dis[(u32Addr - PQ_AWB_DEG_DIS0) >> 1] =
          CLIP3(u32Data & 0xFFFF, 0, 0xFFFF);
    } else if (u32Addr < PQ_AWB_DEG_INIT_GAIN0) {
      u32Value = stWBCalAttr.val[(u32Addr - PQ_AWB_DEG_VAL0) >> 1];
      stWBCalAttr.val[(u32Addr - PQ_AWB_DEG_VAL0) >> 1] =
          CLIP3(u32Data & 0xFFFF, 0, 0xFFFF);
    } else if (u32Addr <= PQ_AWB_DEG_INIT_GAIN3) {
      u32Value = stWBCalAttr.init_gain[(u32Addr - PQ_AWB_DEG_INIT_GAIN0) >> 1];
      stWBCalAttr.init_gain[(u32Addr - PQ_AWB_DEG_INIT_GAIN0) >> 1] =
          CLIP3(u32Data & 0xFFFF, 0, 0xFFFF);
    } else {
      putstr(0, "Not Support!\r\n");
    }
    break;
  }
  if (u8Mode == 1) // Write
  {
    s32Ret = XM_MPI_ISP_SetWBAttr(&stWBAttr);
    if (s32Ret != XM_SUCCESS) {
      putstr(0, "XM_MPI_ISP_SetWBAttr failed!\r\n");
      return XM_FAILURE;
    }
    s32Ret = XM_MPI_ISP_SetWBCalAttr(&stWBCalAttr);
    if (s32Ret != XM_SUCCESS) {
      putstr(0, "XM_MPI_ISP_SetWBCalAttr failed!\r\n");
      return XM_FAILURE;
    }
  }
  *pu32Data = u32Value & 0xFFFF;
  return XM_SUCCESS;
}

// Mode:   0:Read   1:Write
static XM_S32 IspDebugGamma(unsigned char u8Mode, unsigned int u32Addr,
                            XM_U32 *pu32Data) {
  ISP_GAMMA_ATTR_S stGammaAttr;
  XM_S32 s32Ret;
  XM_U32 u32Data, u32Value;

  u32Data = *pu32Data;
  u32Value = 0xFFFFFFFF;
  s32Ret = XM_MPI_ISP_GetGammaAttr(&stGammaAttr);
  if (s32Ret != XM_SUCCESS) {
    putstr(0, "XM_MPI_ISP_GetGammaAttr failed!\r\n");
    return XM_FAILURE;
  }

  switch (u32Addr) {
  case GAM_ENABLE:
    u32Value = stGammaAttr.bEnable;
    stGammaAttr.bEnable = CLIP3(u32Data, 0, 1);
    break;
  case GAM_TYPE:
    u32Value = stGammaAttr.enCurveType;
    stGammaAttr.enCurveType = CLIP3(u32Data, 0, ISP_GAMMA_CURVE_BUTT - 1);
    break;
  default:
    if ((u32Addr >= GAM_TAB0) && (u32Addr <= GAM_TAB96)) {
      u32Value = stGammaAttr.u16Table[(u32Addr - GAM_TAB0) / 2];
      stGammaAttr.u16Table[(u32Addr - GAM_TAB0) / 2] =
          CLIP3(u32Data, 0, 0xFFFF);
    } else {
      u32Value = 0xFFFFFFFF;
    }
    break;
  }

  if (u8Mode == 1) // Write
  {
    s32Ret = XM_MPI_ISP_SetGammaAttr(&stGammaAttr);
    if (s32Ret != XM_SUCCESS) {
      putstr(0, "XM_MPI_ISP_SetGammaAttr failed!\r\n");
      return XM_FAILURE;
    }
  }
  *pu32Data = u32Value;
  return XM_SUCCESS;
}

// Mode:   0:Read   1:Write
static XM_S32 IspDebugCsc_V2(unsigned char u8Mode, unsigned int u32Addr,
                             XM_U32 *pu32Data) {
  ISP_CSC_ATTR_S stCSCAttr;
  XM_S32 s32Ret;
  XM_U32 u32Data, u32Value;
  XM_U8 u8Flg = 0; // 1:Other		2:CSC	0:Rsv
  u32Data = *pu32Data;
  u32Value = 0xFFFFFFFF;
  s32Ret = XM_MPI_ISP_GetCSCAttr(&stCSCAttr);
  if (s32Ret != XM_SUCCESS) {
    putstr(0, "XM_MPI_ISP_GetCSCAttr failed!\r\n");
    return XM_FAILURE;
  }

  CAM_INIT_DATA stCamData;
  s32Ret = camera_para_get(&stCamData);
  if (s32Ret != XM_SUCCESS) {
    putstr(0, "camera_para_get failed!\r\n");
    return XM_FAILURE;
  }
  u8Flg = 1;
  switch (u32Addr) {
  case CSC_CFG_LUM:
    // u32Value = stCSCAttr.stManual.u8LumaVal;
    // stCSCAttr.stManual.u8LumaVal = u32Data&0xFF;
    // u8Flg = 2;
    u32Value = (stCamData.u32CscLumCon >> 8) & 0xFF;
    stCamData.u32CscLumCon =
        (stCamData.u32CscLumCon & 0x00FF) | ((u32Data << 8) & 0xFF00);
    break;
  case CSC_CFG_CON:
    // u32Value = stCSCAttr.stManual.u8ContrVal;
    // stCSCAttr.stManual.u8ContrVal = u32Data&0xFF;
    // u8Flg = 2;
    u32Value = stCamData.u32CscLumCon & 0xFF;
    stCamData.u32CscLumCon =
        (stCamData.u32CscLumCon & 0xFF00) | (u32Data & 0xFF);
    break;
  case CSC_CFG_AUTANCE:
    u32Value = stCamData.u32CscAcutance & 0xFF;
    stCamData.u32CscAcutance = u32Data & 0xFF;
    break;
  case CSC_CFG_HUE:
    // u32Value = stCSCAttr.stManual.u8HueVal;
    // stCSCAttr.stManual.u8HueVal = u32Data&0xFF;
    // u8Flg = 2;
    u32Value = (stCamData.u32CscHueSat >> 8) & 0xFF;
    stCamData.u32CscHueSat =
        (stCamData.u32CscHueSat & 0x00FF) | ((u32Data << 8) & 0xFF00);
    break;
  case CSC_CFG_SAT:
    // u32Value = stCSCAttr.stManual.u8SatuVal;
    // stCSCAttr.stManual.u8SatuVal = u32Data&0xFF;
    // u8Flg = 2;
    u32Value = stCamData.u32CscHueSat & 0xFF;
    stCamData.u32CscHueSat =
        (stCamData.u32CscHueSat & 0xFF00) | (u32Data & 0xFF);
    break;
  default:
    if ((u32Addr >= CSC_CFG_AUTO_CON0) && (u32Addr <= CSC_CFG_AUTO_CON15)) {
      u32Addr = (u32Addr - CSC_CFG_AUTO_CON0) >> 1;
      u32Value = stCSCAttr.stAuto.au8ContrVal[u32Addr];
      stCSCAttr.stAuto.au8ContrVal[u32Addr] = CLIP3(u32Data, 0, 100);
      u8Flg = 2;
    } else {
      u32Value = 0;
      u8Flg = 0;
    }

    break;
  }
  stCSCAttr.enOpType = OP_TYPE_MANUAL;
  if (u8Mode == 1) // Write
  {
    if (u8Flg == 2) {
      s32Ret = XM_MPI_ISP_SetCSCAttr(&stCSCAttr);
      if (s32Ret != XM_SUCCESS) {
        putstr(0, "XM_MPI_ISP_SetCSCAttr failed!\r\n");
        return XM_FAILURE;
      }
    } else if (u8Flg == 1) {
      s32Ret = camera_para_set(&stCamData);
      if (s32Ret != XM_SUCCESS) {
        putstr(0, "camera_para_set failed!\r\n");
        return XM_FAILURE;
      }
    }
  }
  *pu32Data = u32Value;
  return XM_SUCCESS;
}

// Mode:   0:Read   1:Write
static XM_S32 IspDebugCsc(unsigned char u8Mode, unsigned int u32Addr,
                          XM_U32 *pu32Data) {
  ISP_CSC_ATTR_S stCSCAttr;
  XM_S32 s32Ret;
  XM_U32 u32Data, u32Value;
  u32Data = *pu32Data;
  u32Value = 0xFFFFFFFF;
  s32Ret = XM_MPI_ISP_GetCSCAttr(&stCSCAttr);
  if (s32Ret != XM_SUCCESS) {
    putstr(0, "XM_MPI_ISP_GetCSCAttr failed!\r\n");
    return XM_FAILURE;
  }
  switch (u32Addr) {
  case CSC_TYPE:
    break;
  case CSC_OP:
    u32Value = stCSCAttr.enOpType;
    stCSCAttr.enOpType = CLIP3(u32Data, 0, 3);
    break;
  case CSC_LUM:
    u32Value = (stCSCAttr.enOpType == OP_TYPE_MANUAL)
                   ? stCSCAttr.stManual.u8LumaVal
                   : stCSCAttr.stState.u8LumaVal;

    stCSCAttr.enOpType = OP_TYPE_MANUAL;
    stCSCAttr.stManual.u8LumaVal = CLIP3(u32Data, 0, 100);
    break;
  case CSC_CONTRAST:
    u32Value = (stCSCAttr.enOpType == OP_TYPE_MANUAL)
                   ? stCSCAttr.stManual.u8ContrVal
                   : stCSCAttr.stState.u8ContrVal;
    stCSCAttr.enOpType = OP_TYPE_MANUAL;
    stCSCAttr.stManual.u8ContrVal = CLIP3(u32Data, 0, 100);
    break;
  case CSC_HUE:
    u32Value = (stCSCAttr.enOpType == OP_TYPE_MANUAL)
                   ? stCSCAttr.stManual.u8HueVal
                   : stCSCAttr.stState.u8HueVal;
    stCSCAttr.enOpType = OP_TYPE_MANUAL;
    stCSCAttr.stManual.u8HueVal = CLIP3(u32Data, 0, 100);
    break;
  case CSC_SATURATION:
    u32Value = (stCSCAttr.enOpType == OP_TYPE_MANUAL)
                   ? stCSCAttr.stManual.u8SatuVal
                   : stCSCAttr.stState.u8SatuVal;
    stCSCAttr.enOpType = OP_TYPE_MANUAL;
    stCSCAttr.stManual.u8SatuVal = CLIP3(u32Data, 0, 100);
    break;
  case Csc_Ctr_Map0:
    u32Value = stCSCAttr.stAuto.au8ContrVal[0];
    stCSCAttr.stAuto.au8ContrVal[0] = CLIP3(u32Data, 0, 100);
    break;
  case Csc_Ctr_Map1:
    u32Value = stCSCAttr.stAuto.au8ContrVal[1];
    stCSCAttr.stAuto.au8ContrVal[1] = CLIP3(u32Data, 0, 100);
    break;
  case Csc_Ctr_Map2:
    u32Value = stCSCAttr.stAuto.au8ContrVal[2];
    stCSCAttr.stAuto.au8ContrVal[2] = CLIP3(u32Data, 0, 100);
    break;
  case Csc_Ctr_Map3:
    u32Value = stCSCAttr.stAuto.au8ContrVal[3];
    stCSCAttr.stAuto.au8ContrVal[3] = CLIP3(u32Data, 0, 100);
    break;
  case Csc_Ctr_Map4:
    u32Value = stCSCAttr.stAuto.au8ContrVal[4];
    stCSCAttr.stAuto.au8ContrVal[4] = CLIP3(u32Data, 0, 100);
    break;
  case Csc_Ctr_Map5:
    u32Value = stCSCAttr.stAuto.au8ContrVal[5];
    stCSCAttr.stAuto.au8ContrVal[5] = CLIP3(u32Data, 0, 100);
    break;
  case Csc_Ctr_Map6:
    u32Value = stCSCAttr.stAuto.au8ContrVal[6];
    stCSCAttr.stAuto.au8ContrVal[6] = CLIP3(u32Data, 0, 100);
    break;
  case Csc_Ctr_Map7:
    u32Value = stCSCAttr.stAuto.au8ContrVal[7];
    stCSCAttr.stAuto.au8ContrVal[7] = CLIP3(u32Data, 0, 100);
    break;
  case Csc_Ctr_Map8:
    u32Value = stCSCAttr.stAuto.au8ContrVal[8];
    stCSCAttr.stAuto.au8ContrVal[8] = CLIP3(u32Data, 0, 100);
    break;
  case Csc_Ctr_Map9:
    u32Value = stCSCAttr.stAuto.au8ContrVal[9];
    stCSCAttr.stAuto.au8ContrVal[9] = CLIP3(u32Data, 0, 100);
    break;
  case Csc_Ctr_Map10:
    u32Value = stCSCAttr.stAuto.au8ContrVal[10];
    stCSCAttr.stAuto.au8ContrVal[10] = CLIP3(u32Data, 0, 100);
    break;
  case Csc_Ctr_Map11:
    u32Value = stCSCAttr.stAuto.au8ContrVal[11];
    stCSCAttr.stAuto.au8ContrVal[11] = CLIP3(u32Data, 0, 100);
    break;
  case Csc_Ctr_Map12:
    u32Value = stCSCAttr.stAuto.au8ContrVal[12];
    stCSCAttr.stAuto.au8ContrVal[12] = CLIP3(u32Data, 0, 100);
    break;
  case Csc_Ctr_Map13:
    u32Value = stCSCAttr.stAuto.au8ContrVal[13];
    stCSCAttr.stAuto.au8ContrVal[13] = CLIP3(u32Data, 0, 100);
    break;
  case Csc_Ctr_Map14:
    u32Value = stCSCAttr.stAuto.au8ContrVal[14];
    stCSCAttr.stAuto.au8ContrVal[14] = CLIP3(u32Data, 0, 100);
    break;
  case Csc_Ctr_Map15:
    u32Value = stCSCAttr.stAuto.au8ContrVal[15];
    stCSCAttr.stAuto.au8ContrVal[15] = CLIP3(u32Data, 0, 100);
    break;
  default:
    u32Value = 0;
    break;
  }
  // stCSCAttr.enOpType = OP_TYPE_MANUAL;
  if (u8Mode == 1) // Write
  {
    s32Ret = XM_MPI_ISP_SetCSCAttr(&stCSCAttr);
    if (s32Ret != XM_SUCCESS) {
      putstr(0, "XM_MPI_ISP_SetCSCAttr failed!\r\n");
      return XM_FAILURE;
    }
  }
  *pu32Data = u32Value;
  return XM_SUCCESS;
}

// Mode:   0:Read   1:Write
static XM_S32 IspDebugChroma(unsigned char u8Mode, unsigned int u32Addr,
                             XM_U32 *pu32Data) {
  ISP_CHROMA_ATTR_S stChromaAttr;
  XM_S32 s32Ret;
  XM_U32 u32Data, u32Value;
  u32Data = *pu32Data;
  u32Value = 0xFFFFFFFF;
  s32Ret = XM_MPI_ISP_GetChromaAttr(&stChromaAttr);
  if (s32Ret != XM_SUCCESS) {
    putstr(0, "XM_MPI_ISP_GetChromaAttr failed!\r\n");
    return XM_FAILURE;
  }
  switch (u32Addr) {

  case CHROMA_ENABLE:
    u32Value = stChromaAttr.bEnable;
    stChromaAttr.bEnable = (u32Data > 0) ? XM_TRUE : XM_FALSE;
    break;
  case CHROMA_OFST_PP:
    u32Value = stChromaAttr.u16OfstMg;
    u32Value = (u32Value & 0x400) ? u32Value | 0xFFFFFC00 : u32Value;
    stChromaAttr.u16OfstMg = u32Data & 0x7FF;
    break;
  case CHROMA_OFST_R:
    u32Value = stChromaAttr.u16OfstR;
    u32Value = (u32Value & 0x400) ? u32Value | 0xFFFFFC00 : u32Value;
    stChromaAttr.u16OfstR = u32Data & 0x7FF;
    break;
  case CHROMA_OFST_YE:
    u32Value = stChromaAttr.u16OfstYe;
    u32Value = (u32Value & 0x400) ? u32Value | 0xFFFFFC00 : u32Value;
    stChromaAttr.u16OfstYe = u32Data & 0x7FF;
    break;
  case CHROMA_OFST_G:
    u32Value = stChromaAttr.u16OfstG;
    u32Value = (u32Value & 0x400) ? u32Value | 0xFFFFFC00 : u32Value;
    stChromaAttr.u16OfstG = u32Data & 0x7FF;
    break;
  case CHROMA_OFST_CY:
    u32Value = stChromaAttr.u16OfstCy;
    u32Value = (u32Value & 0x400) ? u32Value | 0xFFFFFC00 : u32Value;
    stChromaAttr.u16OfstCy = u32Data & 0x7FF;
    break;
  case CHROMA_OFST_B:
    u32Value = stChromaAttr.u16OfstB;
    u32Value = (u32Value & 0x400) ? u32Value | 0xFFFFFC00 : u32Value;
    stChromaAttr.u16OfstB = u32Data & 0x7FF;
    break;

  case CHROMA_STH_PP:
    u32Value = stChromaAttr.u8SthMg;
    stChromaAttr.u8SthMg = u32Data & 0xFF;
    break;
  case CHROMA_STH_R:
    u32Value = stChromaAttr.u8SthR;
    stChromaAttr.u8SthR = u32Data & 0xFF;
    break;
  case CHROMA_STH_YE:
    u32Value = stChromaAttr.u8SthYe;
    stChromaAttr.u8SthYe = u32Data & 0xFF;
    break;
  case CHROMA_STH_G:
    u32Value = stChromaAttr.u8SthG;
    stChromaAttr.u8SthG = u32Data & 0xFF;
    break;
  case CHROMA_STH_CY:
    u32Value = stChromaAttr.u8SthCy;
    stChromaAttr.u8SthCy = u32Data & 0xFF;
    break;
  case CHROMA_STH_B:
    u32Value = stChromaAttr.u8SthB;
    stChromaAttr.u8SthB = u32Data & 0xFF;
    break;
  default:
    u32Value = 0;
    break;
  }

  if (u8Mode == 1) // Write
  {
    s32Ret = XM_MPI_ISP_SetChromaAttr(&stChromaAttr);
    if (s32Ret != XM_SUCCESS) {
      putstr(0, "XM_MPI_ISP_SetChromaAttr failed!\r\n");
      return XM_FAILURE;
    }
  }
  *pu32Data = u32Value;
  return XM_SUCCESS;
}

static XM_S32 IspDebugSharpeness(unsigned char u8Mode, unsigned int u32Addr,
                                 XM_U32 *pu32Data) {
  ISP_SHARPEN_ATTR_S stSharpenAttr;
  XM_S32 s32Ret;
  XM_U32 u32Data, u32Value;

  u32Data = *pu32Data;
  u32Value = 0xFFFFFFFF;
  s32Ret = XM_MPI_ISP_GetSharpenAttr(&stSharpenAttr);
  if (s32Ret != XM_SUCCESS) {
    putstr(0, "XM_MPI_ISP_GetSharpenAttr failed!\r\n");
    return XM_FAILURE;
  }

  switch (u32Addr) {

  case SHARP_ENABLE:
    u32Value = stSharpenAttr.bEnable;
    stSharpenAttr.bEnable = CLIP3(u32Data, XM_FALSE, XM_TRUE);
    break;
  case SHARP_OPTYPE:
    u32Value = stSharpenAttr.enOpType;
    stSharpenAttr.enOpType = CLIP3(u32Data, OP_TYPE_AUTO, OP_TYPE_BUTT - 1);
    break;
  case SHARP_AUTO_GLOBAL:
    u32Value = stSharpenAttr.stAuto.u8GlobalSth;
    stSharpenAttr.stAuto.u8GlobalSth = CLIP3(u32Data, 0, 0xFF);
    break;
  case SHARP_MANUAL_D:
    u32Value = stSharpenAttr.stManual.u8SharpenD;
    stSharpenAttr.stManual.u8SharpenD = CLIP3(u32Data, 0, 0xFF);
    break;
  case SHARP_MANUAL_UD:
    u32Value = stSharpenAttr.stManual.u8SharpenUd;
    stSharpenAttr.stManual.u8SharpenUd = CLIP3(u32Data, 0, 0xFF);
    break;
  case SHARP_MANUAL_KD:
    u32Value = stSharpenAttr.stManual.u8SharpenKd;
    stSharpenAttr.stManual.u8SharpenKd = CLIP3(u32Data, 0, 0xFF);
    break;
  // 高频
  case SHARP_AUTO_D0:
    u32Value = stSharpenAttr.stAuto.au8SharpenD[0];
    stSharpenAttr.stAuto.au8SharpenD[0] = CLIP3(u32Data, 0, 0xFF);
    break;
  case SHARP_AUTO_D1:
    u32Value = stSharpenAttr.stAuto.au8SharpenD[1];
    stSharpenAttr.stAuto.au8SharpenD[1] = CLIP3(u32Data, 0, 0xFF);
    break;
  case SHARP_AUTO_D2:
    u32Value = stSharpenAttr.stAuto.au8SharpenD[2];
    stSharpenAttr.stAuto.au8SharpenD[2] = CLIP3(u32Data, 0, 0xFF);
    break;
  case SHARP_AUTO_D3:
    u32Value = stSharpenAttr.stAuto.au8SharpenD[3];
    stSharpenAttr.stAuto.au8SharpenD[3] = CLIP3(u32Data, 0, 0xFF);
    break;
  case SHARP_AUTO_D4:
    u32Value = stSharpenAttr.stAuto.au8SharpenD[4];
    stSharpenAttr.stAuto.au8SharpenD[4] = CLIP3(u32Data, 0, 0xFF);
    break;
  case SHARP_AUTO_D5:
    u32Value = stSharpenAttr.stAuto.au8SharpenD[5];
    stSharpenAttr.stAuto.au8SharpenD[5] = CLIP3(u32Data, 0, 0xFF);
    break;
  case SHARP_AUTO_D6:
    u32Value = stSharpenAttr.stAuto.au8SharpenD[6];
    stSharpenAttr.stAuto.au8SharpenD[6] = CLIP3(u32Data, 0, 0xFF);
    break;
  case SHARP_AUTO_D7:
    u32Value = stSharpenAttr.stAuto.au8SharpenD[7];
    stSharpenAttr.stAuto.au8SharpenD[7] = CLIP3(u32Data, 0, 0xFF);
    break;
  case SHARP_AUTO_D8:
    u32Value = stSharpenAttr.stAuto.au8SharpenD[8];
    stSharpenAttr.stAuto.au8SharpenD[8] = CLIP3(u32Data, 0, 0xFF);
    break;
  case SHARP_AUTO_D9:
    u32Value = stSharpenAttr.stAuto.au8SharpenD[9];
    stSharpenAttr.stAuto.au8SharpenD[9] = CLIP3(u32Data, 0, 0xFF);
    break;
  case SHARP_AUTO_D10:
    u32Value = stSharpenAttr.stAuto.au8SharpenD[10];
    stSharpenAttr.stAuto.au8SharpenD[10] = CLIP3(u32Data, 0, 0xFF);
    break;
  case SHARP_AUTO_D11:
    u32Value = stSharpenAttr.stAuto.au8SharpenD[11];
    stSharpenAttr.stAuto.au8SharpenD[11] = CLIP3(u32Data, 0, 0xFF);
    break;
  case SHARP_AUTO_D12:
    u32Value = stSharpenAttr.stAuto.au8SharpenD[12];
    stSharpenAttr.stAuto.au8SharpenD[12] = CLIP3(u32Data, 0, 0xFF);
    break;
  case SHARP_AUTO_D13:
    u32Value = stSharpenAttr.stAuto.au8SharpenD[13];
    stSharpenAttr.stAuto.au8SharpenD[13] = CLIP3(u32Data, 0, 0xFF);
    break;
  case SHARP_AUTO_D14:
    u32Value = stSharpenAttr.stAuto.au8SharpenD[14];
    stSharpenAttr.stAuto.au8SharpenD[14] = CLIP3(u32Data, 0, 0xFF);
    break;
  case SHARP_AUTO_D15:
    u32Value = stSharpenAttr.stAuto.au8SharpenD[15];
    stSharpenAttr.stAuto.au8SharpenD[15] = CLIP3(u32Data, 0, 0xFF);
    break;

  // 中频
  case SHARP_AUTO_UD0:
    u32Value = stSharpenAttr.stAuto.au8SharpenUd[0];
    stSharpenAttr.stAuto.au8SharpenUd[0] = CLIP3(u32Data, 0, 0xFF);
    break;
  case SHARP_AUTO_UD1:
    u32Value = stSharpenAttr.stAuto.au8SharpenUd[1];
    stSharpenAttr.stAuto.au8SharpenUd[1] = CLIP3(u32Data, 0, 0xFF);
    break;
  case SHARP_AUTO_UD2:
    u32Value = stSharpenAttr.stAuto.au8SharpenUd[2];
    stSharpenAttr.stAuto.au8SharpenUd[2] = CLIP3(u32Data, 0, 0xFF);
    break;
  case SHARP_AUTO_UD3:
    u32Value = stSharpenAttr.stAuto.au8SharpenUd[3];
    stSharpenAttr.stAuto.au8SharpenUd[3] = CLIP3(u32Data, 0, 0xFF);
    break;
  case SHARP_AUTO_UD4:
    u32Value = stSharpenAttr.stAuto.au8SharpenUd[4];
    stSharpenAttr.stAuto.au8SharpenUd[4] = CLIP3(u32Data, 0, 0xFF);
    break;
  case SHARP_AUTO_UD5:
    u32Value = stSharpenAttr.stAuto.au8SharpenUd[5];
    stSharpenAttr.stAuto.au8SharpenUd[5] = CLIP3(u32Data, 0, 0xFF);
    break;
  case SHARP_AUTO_UD6:
    u32Value = stSharpenAttr.stAuto.au8SharpenUd[6];
    stSharpenAttr.stAuto.au8SharpenUd[6] = CLIP3(u32Data, 0, 0xFF);
    break;
  case SHARP_AUTO_UD7:
    u32Value = stSharpenAttr.stAuto.au8SharpenUd[7];
    stSharpenAttr.stAuto.au8SharpenUd[7] = CLIP3(u32Data, 0, 0xFF);
    break;
  case SHARP_AUTO_UD8:
    u32Value = stSharpenAttr.stAuto.au8SharpenUd[8];
    stSharpenAttr.stAuto.au8SharpenUd[8] = CLIP3(u32Data, 0, 0xFF);
    break;
  case SHARP_AUTO_UD9:
    u32Value = stSharpenAttr.stAuto.au8SharpenUd[9];
    stSharpenAttr.stAuto.au8SharpenUd[9] = CLIP3(u32Data, 0, 0xFF);
    break;
  case SHARP_AUTO_UD10:
    u32Value = stSharpenAttr.stAuto.au8SharpenUd[10];
    stSharpenAttr.stAuto.au8SharpenUd[10] = CLIP3(u32Data, 0, 0xFF);
    break;
  case SHARP_AUTO_UD11:
    u32Value = stSharpenAttr.stAuto.au8SharpenUd[11];
    stSharpenAttr.stAuto.au8SharpenUd[11] = CLIP3(u32Data, 0, 0xFF);
    break;
  case SHARP_AUTO_UD12:
    u32Value = stSharpenAttr.stAuto.au8SharpenUd[12];
    stSharpenAttr.stAuto.au8SharpenUd[12] = CLIP3(u32Data, 0, 0xFF);
    break;
  case SHARP_AUTO_UD13:
    u32Value = stSharpenAttr.stAuto.au8SharpenUd[13];
    stSharpenAttr.stAuto.au8SharpenUd[13] = CLIP3(u32Data, 0, 0xFF);
    break;
  case SHARP_AUTO_UD14:
    u32Value = stSharpenAttr.stAuto.au8SharpenUd[14];
    stSharpenAttr.stAuto.au8SharpenUd[14] = CLIP3(u32Data, 0, 0xFF);
    break;
  case SHARP_AUTO_UD15:
    u32Value = stSharpenAttr.stAuto.au8SharpenUd[15];
    stSharpenAttr.stAuto.au8SharpenUd[15] = CLIP3(u32Data, 0, 0xFF);
    break;
  // 差分
  case SHARP_AUTO_KD0:
    u32Value = stSharpenAttr.stAuto.au8SharpenKd[0];
    stSharpenAttr.stAuto.au8SharpenKd[0] = CLIP3(u32Data, 0, 0xFF);
    break;
  case SHARP_AUTO_KD1:
    u32Value = stSharpenAttr.stAuto.au8SharpenKd[1];
    stSharpenAttr.stAuto.au8SharpenKd[1] = CLIP3(u32Data, 0, 0xFF);
    break;
  case SHARP_AUTO_KD2:
    u32Value = stSharpenAttr.stAuto.au8SharpenKd[2];
    stSharpenAttr.stAuto.au8SharpenKd[2] = CLIP3(u32Data, 0, 0xFF);
    break;
  case SHARP_AUTO_KD3:
    u32Value = stSharpenAttr.stAuto.au8SharpenKd[3];
    stSharpenAttr.stAuto.au8SharpenKd[3] = CLIP3(u32Data, 0, 0xFF);
    break;
  case SHARP_AUTO_KD4:
    u32Value = stSharpenAttr.stAuto.au8SharpenKd[4];
    stSharpenAttr.stAuto.au8SharpenKd[4] = CLIP3(u32Data, 0, 0xFF);
    break;
  case SHARP_AUTO_KD5:
    u32Value = stSharpenAttr.stAuto.au8SharpenKd[5];
    stSharpenAttr.stAuto.au8SharpenKd[5] = CLIP3(u32Data, 0, 0xFF);
    break;
  case SHARP_AUTO_KD6:
    u32Value = stSharpenAttr.stAuto.au8SharpenKd[6];
    stSharpenAttr.stAuto.au8SharpenKd[6] = CLIP3(u32Data, 0, 0xFF);
    break;
  case SHARP_AUTO_KD7:
    u32Value = stSharpenAttr.stAuto.au8SharpenKd[7];
    stSharpenAttr.stAuto.au8SharpenKd[7] = CLIP3(u32Data, 0, 0xFF);
    break;
  case SHARP_AUTO_KD8:
    u32Value = stSharpenAttr.stAuto.au8SharpenKd[8];
    stSharpenAttr.stAuto.au8SharpenKd[8] = CLIP3(u32Data, 0, 0xFF);
    break;
  case SHARP_AUTO_KD9:
    u32Value = stSharpenAttr.stAuto.au8SharpenKd[9];
    stSharpenAttr.stAuto.au8SharpenKd[9] = CLIP3(u32Data, 0, 0xFF);
    break;
  case SHARP_AUTO_KD10:
    u32Value = stSharpenAttr.stAuto.au8SharpenKd[10];
    stSharpenAttr.stAuto.au8SharpenKd[10] = CLIP3(u32Data, 0, 0xFF);
    break;
  case SHARP_AUTO_KD11:
    u32Value = stSharpenAttr.stAuto.au8SharpenKd[11];
    stSharpenAttr.stAuto.au8SharpenKd[11] = CLIP3(u32Data, 0, 0xFF);
    break;
  case SHARP_AUTO_KD12:
    u32Value = stSharpenAttr.stAuto.au8SharpenKd[12];
    stSharpenAttr.stAuto.au8SharpenKd[12] = CLIP3(u32Data, 0, 0xFF);
    break;
  case SHARP_AUTO_KD13:
    u32Value = stSharpenAttr.stAuto.au8SharpenKd[13];
    stSharpenAttr.stAuto.au8SharpenKd[13] = CLIP3(u32Data, 0, 0xFF);
    break;
  case SHARP_AUTO_KD14:
    u32Value = stSharpenAttr.stAuto.au8SharpenKd[14];
    stSharpenAttr.stAuto.au8SharpenKd[14] = CLIP3(u32Data, 0, 0xFF);
    break;
  case SHARP_AUTO_KD15:
    u32Value = stSharpenAttr.stAuto.au8SharpenKd[15];
    stSharpenAttr.stAuto.au8SharpenKd[15] = CLIP3(u32Data, 0, 0xFF);
    break;
  default:
    break;
  }

  if (u8Mode == 1) // Write
  {
    s32Ret = XM_MPI_ISP_SetSharpenAttr(&stSharpenAttr);
    if (s32Ret != XM_SUCCESS) {
      putstr(0, "XM_MPI_ISP_SetSharpenAttr failed!\r\n");
      return XM_FAILURE;
    }
  }

  *pu32Data = u32Value;
  return XM_SUCCESS;
}

static XM_S32 IspDebugBlc(unsigned char u8Mode, unsigned int u32Addr,
                          XM_U32 *pu32Data) {
  ISP_BLACKLVL_ATTR_S stBlcAttr;
  XM_S32 s32Ret;
  XM_U32 u32Data, u32Value;

  u32Data = *pu32Data;
  u32Value = 0xFFFFFFFF;
  s32Ret = XM_MPI_ISP_GetBlackLevelAttr(&stBlcAttr);
  if (s32Ret != XM_SUCCESS) {
    putstr(0, "XM_MPI_ISP_GetBlackLevelAttr failed!\r\n");
    return XM_FAILURE;
  }
  switch (u32Addr) {
  case BLC_OPTYPE:
    u32Value = stBlcAttr.enOpType;
    stBlcAttr.enOpType = CLIP3(u32Data, OP_TYPE_AUTO, OP_TYPE_BUTT - 1);
    break;
  case BLC_MANUAL_BLC:
    u32Value = stBlcAttr.stManual.u16Blc;
    stBlcAttr.stManual.u16Blc = CLIP3(u32Data, 0, 0xFFFF);
    break;
  case BLC_AUTO_BLC0:
    u32Value = stBlcAttr.stAuto.au16Blc[0];
    stBlcAttr.stAuto.au16Blc[0] = CLIP3(u32Data, 0, 0xFFFF);
    break;
  case BLC_AUTO_BLC1:
    u32Value = stBlcAttr.stAuto.au16Blc[1];
    stBlcAttr.stAuto.au16Blc[1] = CLIP3(u32Data, 0, 0xFFFF);
    break;
  case BLC_AUTO_BLC2:
    u32Value = stBlcAttr.stAuto.au16Blc[2];
    stBlcAttr.stAuto.au16Blc[2] = CLIP3(u32Data, 0, 0xFFFF);
    break;
  case BLC_AUTO_BLC3:
    u32Value = stBlcAttr.stAuto.au16Blc[3];
    stBlcAttr.stAuto.au16Blc[3] = CLIP3(u32Data, 0, 0xFFFF);
    break;
  case BLC_AUTO_BLC4:
    u32Value = stBlcAttr.stAuto.au16Blc[4];
    stBlcAttr.stAuto.au16Blc[4] = CLIP3(u32Data, 0, 0xFFFF);
    break;
  case BLC_AUTO_BLC5:
    u32Value = stBlcAttr.stAuto.au16Blc[5];
    stBlcAttr.stAuto.au16Blc[5] = CLIP3(u32Data, 0, 0xFFFF);
    break;
  case BLC_AUTO_BLC6:
    u32Value = stBlcAttr.stAuto.au16Blc[6];
    stBlcAttr.stAuto.au16Blc[6] = CLIP3(u32Data, 0, 0xFFFF);
    break;
  case BLC_AUTO_BLC7:
    u32Value = stBlcAttr.stAuto.au16Blc[7];
    stBlcAttr.stAuto.au16Blc[7] = CLIP3(u32Data, 0, 0xFFFF);
    break;
  case BLC_AUTO_BLC8:
    u32Value = stBlcAttr.stAuto.au16Blc[8];
    stBlcAttr.stAuto.au16Blc[8] = CLIP3(u32Data, 0, 0xFFFF);
    break;
  case BLC_AUTO_BLC9:
    u32Value = stBlcAttr.stAuto.au16Blc[9];
    stBlcAttr.stAuto.au16Blc[9] = CLIP3(u32Data, 0, 0xFFFF);
    break;
  case BLC_AUTO_BLC10:
    u32Value = stBlcAttr.stAuto.au16Blc[10];
    stBlcAttr.stAuto.au16Blc[10] = CLIP3(u32Data, 0, 0xFFFF);
    break;
  case BLC_AUTO_BLC11:
    u32Value = stBlcAttr.stAuto.au16Blc[11];
    stBlcAttr.stAuto.au16Blc[11] = CLIP3(u32Data, 0, 0xFFFF);
    break;
  case BLC_AUTO_BLC12:
    u32Value = stBlcAttr.stAuto.au16Blc[12];
    stBlcAttr.stAuto.au16Blc[12] = CLIP3(u32Data, 0, 0xFFFF);
    break;
  case BLC_AUTO_BLC13:
    u32Value = stBlcAttr.stAuto.au16Blc[13];
    stBlcAttr.stAuto.au16Blc[13] = CLIP3(u32Data, 0, 0xFFFF);
    break;
  case BLC_AUTO_BLC14:
    u32Value = stBlcAttr.stAuto.au16Blc[14];
    stBlcAttr.stAuto.au16Blc[14] = CLIP3(u32Data, 0, 0xFFFF);
    break;
  case BLC_AUTO_BLC15:
    u32Value = stBlcAttr.stAuto.au16Blc[15];
    stBlcAttr.stAuto.au16Blc[15] = CLIP3(u32Data, 0, 0xFFFF);
    break;
  default:
    break;
  }

  if (u8Mode == 1) // Write
  {
    s32Ret = XM_MPI_ISP_SetBlackLevelAttr(&stBlcAttr);
    if (s32Ret != XM_SUCCESS) {
      putstr(0, "XM_MPI_ISP_SetBlackLevelAttr failed!\r\n");
      return XM_FAILURE;
    }
  }
  *pu32Data = u32Value;
  return XM_SUCCESS;
}

static XM_S32 IspDebugSatration(unsigned char u8Mode, unsigned int u32Addr,
                                XM_U32 *pu32Data) {
  ISP_SATURATION_ATTR_S stSatAttr;
  XM_S32 s32Ret;
  XM_U32 u32Data, u32Value;

  u32Data = *pu32Data;
  u32Value = 0xFFFFFFFF;
  s32Ret = XM_MPI_ISP_GetSaturationAttr(&stSatAttr);
  if (s32Ret != XM_SUCCESS) {
    putstr(0, "XM_MPI_ISP_GetSaturationAttr failed!\r\n");
    return XM_FAILURE;
  }
  switch (u32Addr) {
  case SAT_OPTYPE:
    u32Value = stSatAttr.enOpType;
    stSatAttr.enOpType = CLIP3(u32Data, OP_TYPE_AUTO, OP_TYPE_BUTT - 1);
    break;
  case SAT_MODE:
    u32Value = (XM_U32)(stSatAttr.enSatMode);
    stSatAttr.enSatMode = (XM_U8)CLIP3(u32Data, 0, 1);
    break;
  case SAT_MANUAL_SAT:
    u32Value = stSatAttr.stManual.u8Saturation;
    stSatAttr.stManual.u8Saturation = CLIP3(u32Data, 0, 0xFF);
    break;
  case SAT_AUTO_SAT0:
    u32Value = stSatAttr.stAuto.au8Sat[0];
    stSatAttr.stAuto.au8Sat[0] = CLIP3(u32Data, 0, 0xFF);
    break;
  case SAT_AUTO_SAT1:
    u32Value = stSatAttr.stAuto.au8Sat[1];
    stSatAttr.stAuto.au8Sat[1] = CLIP3(u32Data, 0, 0xFF);
    break;
  case SAT_AUTO_SAT2:
    u32Value = stSatAttr.stAuto.au8Sat[2];
    stSatAttr.stAuto.au8Sat[2] = CLIP3(u32Data, 0, 0xFF);
    break;
  case SAT_AUTO_SAT3:
    u32Value = stSatAttr.stAuto.au8Sat[3];
    stSatAttr.stAuto.au8Sat[3] = CLIP3(u32Data, 0, 0xFF);
    break;
  case SAT_AUTO_SAT4:
    u32Value = stSatAttr.stAuto.au8Sat[4];
    stSatAttr.stAuto.au8Sat[4] = CLIP3(u32Data, 0, 0xFF);
    break;
  case SAT_AUTO_SAT5:
    u32Value = stSatAttr.stAuto.au8Sat[5];
    stSatAttr.stAuto.au8Sat[5] = CLIP3(u32Data, 0, 0xFF);
    break;
  case SAT_AUTO_SAT6:
    u32Value = stSatAttr.stAuto.au8Sat[6];
    stSatAttr.stAuto.au8Sat[6] = CLIP3(u32Data, 0, 0xFF);
    break;
  case SAT_AUTO_SAT7:
    u32Value = stSatAttr.stAuto.au8Sat[7];
    stSatAttr.stAuto.au8Sat[7] = CLIP3(u32Data, 0, 0xFF);
    break;
  case SAT_AUTO_SAT8:
    u32Value = stSatAttr.stAuto.au8Sat[8];
    stSatAttr.stAuto.au8Sat[8] = CLIP3(u32Data, 0, 0xFF);
    break;
  case SAT_AUTO_SAT9:
    u32Value = stSatAttr.stAuto.au8Sat[9];
    stSatAttr.stAuto.au8Sat[9] = CLIP3(u32Data, 0, 0xFF);
    break;
  case SAT_AUTO_SAT10:
    u32Value = stSatAttr.stAuto.au8Sat[10];
    stSatAttr.stAuto.au8Sat[10] = CLIP3(u32Data, 0, 0xFF);
    break;
  case SAT_AUTO_SAT11:
    u32Value = stSatAttr.stAuto.au8Sat[11];
    stSatAttr.stAuto.au8Sat[11] = CLIP3(u32Data, 0, 0xFF);
    break;
  case SAT_AUTO_SAT12:
    u32Value = stSatAttr.stAuto.au8Sat[12];
    stSatAttr.stAuto.au8Sat[12] = CLIP3(u32Data, 0, 0xFF);
    break;
  case SAT_AUTO_SAT13:
    u32Value = stSatAttr.stAuto.au8Sat[13];
    stSatAttr.stAuto.au8Sat[13] = CLIP3(u32Data, 0, 0xFF);
    break;
  case SAT_AUTO_SAT14:
    u32Value = stSatAttr.stAuto.au8Sat[14];
    stSatAttr.stAuto.au8Sat[14] = CLIP3(u32Data, 0, 0xFF);
    break;
  case SAT_AUTO_SAT15:
    u32Value = stSatAttr.stAuto.au8Sat[15];
    stSatAttr.stAuto.au8Sat[15] = CLIP3(u32Data, 0, 0xFF);
    break;
  default:
    break;
  }

  if (u8Mode == 1) // Write
  {
    s32Ret = XM_MPI_ISP_SetSaturationAttr(&stSatAttr);
    if (s32Ret != XM_SUCCESS) {
      putstr(0, "XM_MPI_ISP_GetSaturationAttr failed!\r\n");
      return XM_FAILURE;
    }
  }
  *pu32Data = u32Value;
  return XM_SUCCESS;
}

// Mode:   0:Read   1:Write
static XM_S32 IspDebugCcm(unsigned char u8Mode, unsigned int u32Addr,
                          XM_U32 *pu32Data) {
  ISP_COLORMATRIX_ATTR_S stCCMAttr;
  XM_S32 s32Ret;
  XM_U32 u32Data, u32Value, u32Tmp;

  u32Data = *pu32Data;
  u32Value = 0xFFFFFFFF;
  s32Ret = XM_MPI_ISP_GetCCMAttr(&stCCMAttr);
  if (s32Ret != XM_SUCCESS) {
    putstr(0, "XM_MPI_ISP_GetCCMAttr failed!\r\n");
    return XM_FAILURE;
  }
  switch (u32Addr) {
  case CCM_BYPASS:
    u32Value = stCCMAttr.bByPass;
    stCCMAttr.bByPass = CLIP3(u32Data, XM_FALSE, XM_TRUE);
    break;
  case CCM_OPTYPE:
    u32Value = stCCMAttr.enOpType;
    stCCMAttr.enOpType = CLIP3(u32Data, OP_TYPE_AUTO, OP_TYPE_BUTT - 1);
    break;
  case CCM_MANUAL_R_OFST:
    u32Value = stCCMAttr.stManual.au16CCM[0] & 0xFFFF;
    if (u32Value & 0x1000)
      u32Value |= 0xFFFFF000;
    u32Tmp = u32Data & 0x1FFF;
    u32Tmp = (u32Tmp & 0x1000) ? (u32Tmp | 0xFFFFF000) : u32Tmp;
    stCCMAttr.stManual.au16CCM[0] = u32Tmp;
    break;
  case CCM_MANUAL_R_R:
    u32Value = stCCMAttr.stManual.au16CCM[1] & 0xFFFF;
    if (u32Value & 0x1000)
      u32Value |= 0xFFFFF000;
    u32Tmp = u32Data & 0x1FFF;
    u32Tmp = (u32Tmp & 0x1000) ? (u32Tmp | 0xFFFFF000) : u32Tmp;
    stCCMAttr.stManual.au16CCM[1] = u32Tmp;
    break;
  case CCM_MANUAL_R_G:
    u32Value = stCCMAttr.stManual.au16CCM[2] & 0xFFFF;
    if (u32Value & 0x1000)
      u32Value |= 0xFFFFF000;
    u32Tmp = u32Data & 0x1FFF;
    u32Tmp = (u32Tmp & 0x1000) ? (u32Tmp | 0xFFFFF000) : u32Tmp;
    stCCMAttr.stManual.au16CCM[2] = u32Tmp;
    break;
  case CCM_MANUAL_R_B:
    u32Value = stCCMAttr.stManual.au16CCM[3] & 0xFFFF;
    if (u32Value & 0x1000)
      u32Value |= 0xFFFFF000;
    u32Tmp = u32Data & 0x1FFF;
    u32Tmp = (u32Tmp & 0x1000) ? (u32Tmp | 0xFFFFF000) : u32Tmp;
    stCCMAttr.stManual.au16CCM[3] = u32Tmp;
    break;
  case CCM_MANUAL_G_OFST:
    u32Value = stCCMAttr.stManual.au16CCM[4] & 0xFFFF;
    if (u32Value & 0x1000)
      u32Value |= 0xFFFFF000;
    u32Tmp = u32Data & 0x1FFF;
    u32Tmp = (u32Tmp & 0x1000) ? (u32Tmp | 0xFFFFF000) : u32Tmp;
    stCCMAttr.stManual.au16CCM[4] = u32Tmp;
    break;
  case CCM_MANUAL_G_R:
    u32Value = stCCMAttr.stManual.au16CCM[5] & 0xFFFF;
    if (u32Value & 0x1000)
      u32Value |= 0xFFFFF000;
    u32Tmp = u32Data & 0x1FFF;
    u32Tmp = (u32Tmp & 0x1000) ? (u32Tmp | 0xFFFFF000) : u32Tmp;
    stCCMAttr.stManual.au16CCM[5] = u32Tmp;
    break;
  case CCM_MANUAL_G_G:
    u32Value = stCCMAttr.stManual.au16CCM[6] & 0xFFFF;
    if (u32Value & 0x1000)
      u32Value |= 0xFFFFF000;
    u32Tmp = u32Data & 0x1FFF;
    u32Tmp = (u32Tmp & 0x1000) ? (u32Tmp | 0xFFFFF000) : u32Tmp;
    stCCMAttr.stManual.au16CCM[6] = u32Tmp;
    break;
  case CCM_MANUAL_G_B:
    u32Value = stCCMAttr.stManual.au16CCM[7] & 0xFFFF;
    if (u32Value & 0x1000)
      u32Value |= 0xFFFFF000;
    u32Tmp = u32Data & 0x1FFF;
    u32Tmp = (u32Tmp & 0x1000) ? (u32Tmp | 0xFFFFF000) : u32Tmp;
    stCCMAttr.stManual.au16CCM[7] = u32Tmp;
    break;
  case CCM_MANUAL_B_OFST:
    u32Value = stCCMAttr.stManual.au16CCM[8] & 0xFFFF;
    if (u32Value & 0x1000)
      u32Value |= 0xFFFFF000;
    u32Tmp = u32Data & 0x1FFF;
    u32Tmp = (u32Tmp & 0x1000) ? (u32Tmp | 0xFFFFF000) : u32Tmp;
    stCCMAttr.stManual.au16CCM[8] = u32Tmp;
    break;
  case CCM_MANUAL_B_R:
    u32Value = stCCMAttr.stManual.au16CCM[9] & 0xFFFF;
    if (u32Value & 0x1000)
      u32Value |= 0xFFFFF000;
    u32Tmp = u32Data & 0x1FFF;
    u32Tmp = (u32Tmp & 0x1000) ? (u32Tmp | 0xFFFFF000) : u32Tmp;
    stCCMAttr.stManual.au16CCM[9] = u32Tmp;
    break;
  case CCM_MANUAL_B_G:
    u32Value = stCCMAttr.stManual.au16CCM[10] & 0xFFFF;
    if (u32Value & 0x1000)
      u32Value |= 0xFFFFF000;
    u32Tmp = u32Data & 0x1FFF;
    u32Tmp = (u32Tmp & 0x1000) ? (u32Tmp | 0xFFFFF000) : u32Tmp;
    stCCMAttr.stManual.au16CCM[10] = u32Tmp;
    break;
  case CCM_MANUAL_B_B:
    u32Value = stCCMAttr.stManual.au16CCM[11] & 0xFFFF;
    if (u32Value & 0x1000)
      u32Value |= 0xFFFFF000;
    u32Tmp = u32Data & 0x1FFF;
    u32Tmp = (u32Tmp & 0x1000) ? (u32Tmp | 0xFFFFF000) : u32Tmp;
    stCCMAttr.stManual.au16CCM[11] = u32Tmp;
    break;

  case CCM_HIGH_CT:
    u32Value = stCCMAttr.stAuto.u16HighColorTemp;
    stCCMAttr.stAuto.u16HighColorTemp = u32Data;
    break;
  case CCM_AUTO1_R_OFST:
    u32Value = stCCMAttr.stAuto.au16HighCCM[0] & 0xFFFF;
    if (u32Value & 0x1000)
      u32Value |= 0xFFFFF000;
    u32Tmp = u32Data & 0x1FFF;
    u32Tmp = (u32Tmp & 0x1000) ? (u32Tmp | 0xFFFFF000) : u32Tmp;
    stCCMAttr.stAuto.au16HighCCM[0] = u32Tmp;
    break;
  case CCM_AUTO1_R_R:
    u32Value = stCCMAttr.stAuto.au16HighCCM[1] & 0xFFFF;
    if (u32Value & 0x1000)
      u32Value |= 0xFFFFF000;
    u32Tmp = u32Data & 0x1FFF;
    u32Tmp = (u32Tmp & 0x1000) ? (u32Tmp | 0xFFFFF000) : u32Tmp;
    stCCMAttr.stAuto.au16HighCCM[1] = u32Tmp;
    break;
  case CCM_AUTO1_R_G:
    u32Value = stCCMAttr.stAuto.au16HighCCM[2] & 0xFFFF;
    if (u32Value & 0x1000)
      u32Value |= 0xFFFFF000;
    u32Tmp = u32Data & 0x1FFF;
    u32Tmp = (u32Tmp & 0x1000) ? (u32Tmp | 0xFFFFF000) : u32Tmp;
    stCCMAttr.stAuto.au16HighCCM[2] = u32Tmp;
    break;
  case CCM_AUTO1_R_B:
    u32Value = stCCMAttr.stAuto.au16HighCCM[3] & 0xFFFF;
    if (u32Value & 0x1000)
      u32Value |= 0xFFFFF000;
    u32Tmp = u32Data & 0x1FFF;
    u32Tmp = (u32Tmp & 0x1000) ? (u32Tmp | 0xFFFFF000) : u32Tmp;
    stCCMAttr.stAuto.au16HighCCM[3] = u32Tmp;
    break;
  case CCM_AUTO1_G_OFST:
    u32Value = stCCMAttr.stAuto.au16HighCCM[4] & 0xFFFF;
    if (u32Value & 0x1000)
      u32Value |= 0xFFFFF000;
    u32Tmp = u32Data & 0x1FFF;
    u32Tmp = (u32Tmp & 0x1000) ? (u32Tmp | 0xFFFFF000) : u32Tmp;
    stCCMAttr.stAuto.au16HighCCM[4] = u32Tmp;
    break;
  case CCM_AUTO1_G_R:
    u32Value = stCCMAttr.stAuto.au16HighCCM[5] & 0xFFFF;
    if (u32Value & 0x1000)
      u32Value |= 0xFFFFF000;
    u32Tmp = u32Data & 0x1FFF;
    u32Tmp = (u32Tmp & 0x1000) ? (u32Tmp | 0xFFFFF000) : u32Tmp;
    stCCMAttr.stAuto.au16HighCCM[5] = u32Tmp;
    break;
  case CCM_AUTO1_G_G:
    u32Value = stCCMAttr.stAuto.au16HighCCM[6] & 0xFFFF;
    if (u32Value & 0x1000)
      u32Value |= 0xFFFFF000;
    u32Tmp = u32Data & 0x1FFF;
    u32Tmp = (u32Tmp & 0x1000) ? (u32Tmp | 0xFFFFF000) : u32Tmp;
    stCCMAttr.stAuto.au16HighCCM[6] = u32Tmp;
    break;
  case CCM_AUTO1_G_B:
    u32Value = stCCMAttr.stAuto.au16HighCCM[7] & 0xFFFF;
    if (u32Value & 0x1000)
      u32Value |= 0xFFFFF000;
    u32Tmp = u32Data & 0x1FFF;
    u32Tmp = (u32Tmp & 0x1000) ? (u32Tmp | 0xFFFFF000) : u32Tmp;
    stCCMAttr.stAuto.au16HighCCM[7] = u32Tmp;
    break;
  case CCM_AUTO1_B_OFST:
    u32Value = stCCMAttr.stAuto.au16HighCCM[8] & 0xFFFF;
    if (u32Value & 0x1000)
      u32Value |= 0xFFFFF000;
    u32Tmp = u32Data & 0x1FFF;
    u32Tmp = (u32Tmp & 0x1000) ? (u32Tmp | 0xFFFFF000) : u32Tmp;
    stCCMAttr.stAuto.au16HighCCM[8] = u32Tmp;
    break;
  case CCM_AUTO1_B_R:
    u32Value = stCCMAttr.stAuto.au16HighCCM[9] & 0xFFFF;
    if (u32Value & 0x1000)
      u32Value |= 0xFFFFF000;
    u32Tmp = u32Data & 0x1FFF;
    u32Tmp = (u32Tmp & 0x1000) ? (u32Tmp | 0xFFFFF000) : u32Tmp;
    stCCMAttr.stAuto.au16HighCCM[9] = u32Tmp;
    break;
  case CCM_AUTO1_B_G:
    u32Value = stCCMAttr.stAuto.au16HighCCM[10] & 0xFFFF;
    if (u32Value & 0x1000)
      u32Value |= 0xFFFFF000;
    u32Tmp = u32Data & 0x1FFF;
    u32Tmp = (u32Tmp & 0x1000) ? (u32Tmp | 0xFFFFF000) : u32Tmp;
    stCCMAttr.stAuto.au16HighCCM[10] = u32Tmp;
    break;
  case CCM_AUTO1_B_B:
    u32Value = stCCMAttr.stAuto.au16HighCCM[11] & 0xFFFF;
    if (u32Value & 0x1000)
      u32Value |= 0xFFFFF000;
    u32Tmp = u32Data & 0x1FFF;
    u32Tmp = (u32Tmp & 0x1000) ? (u32Tmp | 0xFFFFF000) : u32Tmp;
    stCCMAttr.stAuto.au16HighCCM[11] = u32Tmp;
    break;

  case CCM_MID_CT:
    u32Value = stCCMAttr.stAuto.u16MidColorTemp;
    stCCMAttr.stAuto.u16MidColorTemp = CLIP3(u32Data, 0, 0x7FFF);
    break;
  case CCM_AUTO2_R_OFST:
    u32Value = stCCMAttr.stAuto.au16MidCCM[0] & 0xFFFF;
    if (u32Value & 0x1000)
      u32Value |= 0xFFFFF000;
    u32Tmp = u32Data & 0x1FFF;
    u32Tmp = (u32Tmp & 0x1000) ? (u32Tmp | 0xFFFFF000) : u32Tmp;
    stCCMAttr.stAuto.au16MidCCM[0] = u32Tmp;
    break;
  case CCM_AUTO2_R_R:
    u32Value = stCCMAttr.stAuto.au16MidCCM[1] & 0xFFFF;
    if (u32Value & 0x1000)
      u32Value |= 0xFFFFF000;
    u32Tmp = u32Data & 0x1FFF;
    u32Tmp = (u32Tmp & 0x1000) ? (u32Tmp | 0xFFFFF000) : u32Tmp;
    stCCMAttr.stAuto.au16MidCCM[1] = u32Tmp;
    break;
  case CCM_AUTO2_R_G:
    u32Value = stCCMAttr.stAuto.au16MidCCM[2] & 0xFFFF;
    if (u32Value & 0x1000)
      u32Value |= 0xFFFFF000;
    u32Tmp = u32Data & 0x1FFF;
    u32Tmp = (u32Tmp & 0x1000) ? (u32Tmp | 0xFFFFF000) : u32Tmp;
    stCCMAttr.stAuto.au16MidCCM[2] = u32Tmp;
    break;
  case CCM_AUTO2_R_B:
    u32Value = stCCMAttr.stAuto.au16MidCCM[3] & 0xFFFF;
    if (u32Value & 0x1000)
      u32Value |= 0xFFFFF000;
    u32Tmp = u32Data & 0x1FFF;
    u32Tmp = (u32Tmp & 0x1000) ? (u32Tmp | 0xFFFFF000) : u32Tmp;
    stCCMAttr.stAuto.au16MidCCM[3] = u32Tmp;
    break;
  case CCM_AUTO2_G_OFST:
    u32Value = stCCMAttr.stAuto.au16MidCCM[4] & 0xFFFF;
    if (u32Value & 0x1000)
      u32Value |= 0xFFFFF000;
    u32Tmp = u32Data & 0x1FFF;
    u32Tmp = (u32Tmp & 0x1000) ? (u32Tmp | 0xFFFFF000) : u32Tmp;
    stCCMAttr.stAuto.au16MidCCM[4] = u32Tmp;
    break;
  case CCM_AUTO2_G_R:
    u32Value = stCCMAttr.stAuto.au16MidCCM[5] & 0xFFFF;
    if (u32Value & 0x1000)
      u32Value |= 0xFFFFF000;
    u32Tmp = u32Data & 0x1FFF;
    u32Tmp = (u32Tmp & 0x1000) ? (u32Tmp | 0xFFFFF000) : u32Tmp;
    stCCMAttr.stAuto.au16MidCCM[5] = u32Tmp;
    break;
  case CCM_AUTO2_G_G:
    u32Value = stCCMAttr.stAuto.au16MidCCM[6] & 0xFFFF;
    if (u32Value & 0x1000)
      u32Value |= 0xFFFFF000;
    u32Tmp = u32Data & 0x1FFF;
    u32Tmp = (u32Tmp & 0x1000) ? (u32Tmp | 0xFFFFF000) : u32Tmp;
    stCCMAttr.stAuto.au16MidCCM[6] = u32Tmp;
    break;
  case CCM_AUTO2_G_B:
    u32Value = stCCMAttr.stAuto.au16MidCCM[7] & 0xFFFF;
    if (u32Value & 0x1000)
      u32Value |= 0xFFFFF000;
    u32Tmp = u32Data & 0x1FFF;
    u32Tmp = (u32Tmp & 0x1000) ? (u32Tmp | 0xFFFFF000) : u32Tmp;
    stCCMAttr.stAuto.au16MidCCM[7] = u32Tmp;
    break;
  case CCM_AUTO2_B_OFST:
    u32Value = stCCMAttr.stAuto.au16MidCCM[8] & 0xFFFF;
    if (u32Value & 0x1000)
      u32Value |= 0xFFFFF000;
    u32Tmp = u32Data & 0x1FFF;
    u32Tmp = (u32Tmp & 0x1000) ? (u32Tmp | 0xFFFFF000) : u32Tmp;
    stCCMAttr.stAuto.au16MidCCM[8] = u32Tmp;
    break;
  case CCM_AUTO2_B_R:
    u32Value = stCCMAttr.stAuto.au16MidCCM[9] & 0xFFFF;
    if (u32Value & 0x1000)
      u32Value |= 0xFFFFF000;
    u32Tmp = u32Data & 0x1FFF;
    u32Tmp = (u32Tmp & 0x1000) ? (u32Tmp | 0xFFFFF000) : u32Tmp;
    stCCMAttr.stAuto.au16MidCCM[9] = u32Tmp;
    break;
  case CCM_AUTO2_B_G:
    u32Value = stCCMAttr.stAuto.au16MidCCM[10] & 0xFFFF;
    if (u32Value & 0x1000)
      u32Value |= 0xFFFFF000;
    u32Tmp = u32Data & 0x1FFF;
    u32Tmp = (u32Tmp & 0x1000) ? (u32Tmp | 0xFFFFF000) : u32Tmp;
    stCCMAttr.stAuto.au16MidCCM[10] = u32Tmp;
    break;
  case CCM_AUTO2_B_B:
    u32Value = stCCMAttr.stAuto.au16MidCCM[11] & 0xFFFF;
    if (u32Value & 0x1000)
      u32Value |= 0xFFFFF000;
    u32Tmp = u32Data & 0x1FFF;
    u32Tmp = (u32Tmp & 0x1000) ? (u32Tmp | 0xFFFFF000) : u32Tmp;
    stCCMAttr.stAuto.au16MidCCM[11] = u32Tmp;
    break;

  case CCM_LOW_CT:
    u32Value = stCCMAttr.stAuto.u16LowColorTemp;
    stCCMAttr.stAuto.u16LowColorTemp = CLIP3(u32Data, 0, 0x7FFF);
    break;
  case CCM_AUTO3_R_OFST:
    u32Value = stCCMAttr.stAuto.au16LowCCM[0] & 0xFFFF;
    if (u32Value & 0x1000)
      u32Value |= 0xFFFFF000;
    u32Tmp = u32Data & 0x1FFF;
    u32Tmp = (u32Tmp & 0x1000) ? (u32Tmp | 0xFFFFF000) : u32Tmp;
    stCCMAttr.stAuto.au16LowCCM[0] = u32Tmp;
    break;
  case CCM_AUTO3_R_R:
    u32Value = stCCMAttr.stAuto.au16LowCCM[1] & 0xFFFF;
    if (u32Value & 0x1000)
      u32Value |= 0xFFFFF000;
    u32Tmp = u32Data & 0x1FFF;
    u32Tmp = (u32Tmp & 0x1000) ? (u32Tmp | 0xFFFFF000) : u32Tmp;
    stCCMAttr.stAuto.au16LowCCM[1] = u32Tmp;
    break;
  case CCM_AUTO3_R_G:
    u32Value = stCCMAttr.stAuto.au16LowCCM[2] & 0xFFFF;
    if (u32Value & 0x1000)
      u32Value |= 0xFFFFF000;
    u32Tmp = u32Data & 0x1FFF;
    u32Tmp = (u32Tmp & 0x1000) ? (u32Tmp | 0xFFFFF000) : u32Tmp;
    stCCMAttr.stAuto.au16LowCCM[2] = u32Tmp;
    break;
  case CCM_AUTO3_R_B:
    u32Value = stCCMAttr.stAuto.au16LowCCM[3] & 0xFFFF;
    if (u32Value & 0x1000)
      u32Value |= 0xFFFFF000;
    u32Tmp = u32Data & 0x1FFF;
    u32Tmp = (u32Tmp & 0x1000) ? (u32Tmp | 0xFFFFF000) : u32Tmp;
    stCCMAttr.stAuto.au16LowCCM[3] = u32Tmp;
    break;
  case CCM_AUTO3_G_OFST:
    u32Value = stCCMAttr.stAuto.au16LowCCM[4] & 0xFFFF;
    if (u32Value & 0x1000)
      u32Value |= 0xFFFFF000;
    u32Tmp = u32Data & 0x1FFF;
    u32Tmp = (u32Tmp & 0x1000) ? (u32Tmp | 0xFFFFF000) : u32Tmp;
    stCCMAttr.stAuto.au16LowCCM[4] = u32Tmp;
    break;
  case CCM_AUTO3_G_R:
    u32Value = stCCMAttr.stAuto.au16LowCCM[5] & 0xFFFF;
    if (u32Value & 0x1000)
      u32Value |= 0xFFFFF000;
    u32Tmp = u32Data & 0x1FFF;
    u32Tmp = (u32Tmp & 0x1000) ? (u32Tmp | 0xFFFFF000) : u32Tmp;
    stCCMAttr.stAuto.au16LowCCM[5] = u32Tmp;
    break;
  case CCM_AUTO3_G_G:
    u32Value = stCCMAttr.stAuto.au16LowCCM[6] & 0xFFFF;
    if (u32Value & 0x1000)
      u32Value |= 0xFFFFF000;
    u32Tmp = u32Data & 0x1FFF;
    u32Tmp = (u32Tmp & 0x1000) ? (u32Tmp | 0xFFFFF000) : u32Tmp;
    stCCMAttr.stAuto.au16LowCCM[6] = u32Tmp;
    break;
  case CCM_AUTO3_G_B:
    u32Value = stCCMAttr.stAuto.au16LowCCM[7] & 0xFFFF;
    if (u32Value & 0x1000)
      u32Value |= 0xFFFFF000;
    u32Tmp = u32Data & 0x1FFF;
    u32Tmp = (u32Tmp & 0x1000) ? (u32Tmp | 0xFFFFF000) : u32Tmp;
    stCCMAttr.stAuto.au16LowCCM[7] = u32Tmp;
    break;
  case CCM_AUTO3_B_OFST:
    u32Value = stCCMAttr.stAuto.au16LowCCM[8] & 0xFFFF;
    if (u32Value & 0x1000)
      u32Value |= 0xFFFFF000;
    u32Tmp = u32Data & 0x1FFF;
    u32Tmp = (u32Tmp & 0x1000) ? (u32Tmp | 0xFFFFF000) : u32Tmp;
    stCCMAttr.stAuto.au16LowCCM[8] = u32Tmp;
    break;
  case CCM_AUTO3_B_R:
    u32Value = stCCMAttr.stAuto.au16LowCCM[9] & 0xFFFF;
    if (u32Value & 0x1000)
      u32Value |= 0xFFFFF000;
    u32Tmp = u32Data & 0x1FFF;
    u32Tmp = (u32Tmp & 0x1000) ? (u32Tmp | 0xFFFFF000) : u32Tmp;
    stCCMAttr.stAuto.au16LowCCM[9] = u32Tmp;
    break;
  case CCM_AUTO3_B_G:
    u32Value = stCCMAttr.stAuto.au16LowCCM[10] & 0xFFFF;
    if (u32Value & 0x1000)
      u32Value |= 0xFFFFF000;
    u32Tmp = u32Data & 0x1FFF;
    u32Tmp = (u32Tmp & 0x1000) ? (u32Tmp | 0xFFFFF000) : u32Tmp;
    stCCMAttr.stAuto.au16LowCCM[10] = u32Tmp;
    break;
  case CCM_AUTO3_B_B:
    u32Value = stCCMAttr.stAuto.au16LowCCM[11] & 0xFFFF;
    if (u32Value & 0x1000)
      u32Value |= 0xFFFFF000;
    u32Tmp = u32Data & 0x1FFF;
    u32Tmp = (u32Tmp & 0x1000) ? (u32Tmp | 0xFFFFF000) : u32Tmp;
    stCCMAttr.stAuto.au16LowCCM[11] = u32Tmp;
    break;
  default:
    break;
  }

  if (u8Mode == 1) // Write
  {
    s32Ret = XM_MPI_ISP_SetCCMAttr(&stCCMAttr);
    if (s32Ret != XM_SUCCESS) {
      putstr(0, "XM_MPI_ISP_SetCCMAttr failed!\r\n");
      return XM_FAILURE;
    }
  }
  *pu32Data = u32Value;
  return XM_SUCCESS;
}

// Mode:   0:Read   1:Write
static XM_S32 IspDebugAntiFalseColor(unsigned char u8Mode, unsigned int u32Addr,
                                     XM_U32 *pu32Data) {
  ISP_ANTI_FALSECOLOR_S stAntiFC;
  XM_S32 s32Ret;
  XM_U32 u32Data, u32Value;
  u32Data = *pu32Data;
  u32Value = 0xFFFFFFFF;
  s32Ret = XM_MPI_ISP_GetAntiFalseColorAttr(&stAntiFC);
  if (s32Ret != XM_SUCCESS) {
    putstr(0, "XM_MPI_ISP_GetAntiFalseColorAttr failed!\r\n");
    return XM_FAILURE;
  }

  switch (u32Addr) {
  case AFC_ENABLE:
    u32Value = stAntiFC.bEnable;
    stAntiFC.bEnable = CLIP3(u32Data, XM_FALSE, XM_TRUE);
    break;
  case AFC_STH:
    u32Value = stAntiFC.u8Strength;
    stAntiFC.u8Strength = CLIP3(u32Data, 0, 0xFF);
    break;
  default:
    u32Value = 0;
    break;
  }

  if (u8Mode == 1) // Write
  {
    s32Ret = XM_MPI_ISP_SetAntiFalseColorAttr(&stAntiFC);
    if (s32Ret != XM_SUCCESS) {
      putstr(0, "XM_MPI_ISP_SetAntiFalseColorAttr failed!\r\n");
      return XM_FAILURE;
    }
  }
  *pu32Data = u32Value;
  return XM_SUCCESS;
}

static XM_S32 IspDebugAe2(unsigned char u8Mode, unsigned int u32Addr,
                          XM_U32 *pu32Data) {
  XM_S32 s32Ret;
  XM_U32 u32Data, u32Value;
  ISP_INNER_STATE_INFO_S stInnerStateInfo;
  ISP_EXPOSURE_ATTR_S stExpAttr;
  ISP_EXP_STA_INFO_S stExpStatistic;
  u32Data = *pu32Data;
  u32Value = 0xFFFFFFFF;
  s32Ret = XM_MPI_ISP_QueryInnerStateInfo(ISP_USED, &stInnerStateInfo);
  if (s32Ret != XM_SUCCESS) {
    putstr(0, "XM_MPI_ISP_QueryInnerStateInfo failed!\r\n");
    return XM_FAILURE;
  }
  s32Ret = XM_MPI_ISP_GetExposureAttr(ISP_USED, &stExpAttr);
  if (s32Ret != XM_SUCCESS) {
    putstr(0, "XM_MPI_ISP_GetExposureAttr failed!\r\n");
    return XM_FAILURE;
  }

  s32Ret = XM_MPI_ISP_GetExpStaInfo(ISP_USED, &stExpStatistic);
  if (s32Ret != XM_SUCCESS) {
    putstr(0, "XM_MPI_ISP_GetExpStaInfo failed!\r\n");
    return XM_FAILURE;
  }
  XM_U8 u8Flg = 0; // 0:Ae  1:Camera
  CAM_INIT_DATA stCamData;
  s32Ret = camera_para_get(&stCamData);
  if (s32Ret != XM_SUCCESS) {
    putstr(0, "camera_para_get failed!\r\n");
    return XM_FAILURE;
  }

  switch (u32Addr) {
  case AE_T_MIN:
    u32Value = stExpAttr.stAuto.stExpTimeRange.u32Min;
    stExpAttr.stAuto.stExpTimeRange.u32Min = CLIP3(u32Data, 0, 0xFFFFFFFF);
    break;
  case AE_T_MAX:
    u32Value = stExpAttr.stAuto.stExpTimeRange.u32Max;
    stExpAttr.stAuto.stExpTimeRange.u32Max = CLIP3(u32Data, 0, 0xFFFFFFFF);
    break;
  case AE_AGAIN_MIN:
    u32Value = stExpAttr.stAuto.stAGainRange.u32Min / 4;
    u32Data = u32Data * 4;
    stExpAttr.stAuto.stAGainRange.u32Min = CLIP3(u32Data, 0x400, 0xFFFFFFFF);
    break;
  case AE_AGAIN_MAX:
    u32Value = stExpAttr.stAuto.stAGainRange.u32Max / 4;
    u32Data = u32Data * 4;
    stExpAttr.stAuto.stAGainRange.u32Max = CLIP3(u32Data, 0x400, 0xFFFFFFFF);
    break;
  case AE_DGAIN_MIN:
    u32Value = stExpAttr.stAuto.stDGainRange.u32Min / 4;
    u32Data = u32Data * 4;
    stExpAttr.stAuto.stDGainRange.u32Min = CLIP3(u32Data, 0x400, 0xFFFFFFFF);
    break;
  case AE_DGAIN_MAX:
    u32Value = stExpAttr.stAuto.stDGainRange.u32Max / 4;
    u32Data = u32Data * 4;
    stExpAttr.stAuto.stDGainRange.u32Max = CLIP3(u32Data, 0x400, 0xFFFFFFFF);
    break;
  case AE_ISPDGAIN_MIN:
    u32Value = stExpAttr.stAuto.stISPDGainRange.u32Min / 4;
    u32Data = u32Data * 4;
    stExpAttr.stAuto.stISPDGainRange.u32Min = CLIP3(u32Data, 0x400, 0xFFFFFFFF);
    break;
  case AE_ISPDGAIN_MAX:
    u32Value = stExpAttr.stAuto.stISPDGainRange.u32Max / 4;
    u32Data = u32Data * 4;
    stExpAttr.stAuto.stISPDGainRange.u32Max = CLIP3(u32Data, 0x400, 0xFFFFFFFF);
    break;
  case AE_ALLGAIN_MAX1:
    if (gstProductInfo.u8Encoder == 0x03) {
      u32Value = (XM_U32)(stCamData.u16GainDefSD) << 8;
      stCamData.u16GainDefSD = u32Data >> 8;
    } else {
      u32Value = (XM_U32)(stCamData.u16GainDef) << 8;
      stCamData.u16GainDef = u32Data >> 8;
    }
    u8Flg = 1;
    break;
  case AE_ALLGAIN_MAX2:
    u32Value = (XM_U32)(stCamData.u16GainMax) << 8;
    stCamData.u16GainMax = u32Data >> 8;
    u8Flg = 1;
    break;
  case AE_TARGET_NUM:
    u32Value = stCamData.s32TgtNum;
    stCamData.s32TgtNum = u32Data;
    u8Flg = 1;
    break;
  case AE_EXP0_H:
    u32Value = stCamData.au32TgtExp[0] >> 16;
    stCamData.au32TgtExp[0] =
        (stCamData.au32TgtExp[0] & 0xFFFF) | ((u32Data & 0xFFFF) << 16);
    u8Flg = 1;
    break;
  case AE_EXP0_L:
    u32Value = stCamData.au32TgtExp[0] & 0xFFFF;
    stCamData.au32TgtExp[0] =
        (stCamData.au32TgtExp[0] & 0xFFFF0000) | (u32Data & 0xFFFF);
    u8Flg = 1;
    break;
  case AE_EXP1_H:
    u32Value = stCamData.au32TgtExp[1] >> 16;
    stCamData.au32TgtExp[1] =
        (stCamData.au32TgtExp[1] & 0xFFFF) | ((u32Data & 0xFFFF) << 16);
    u8Flg = 1;
    break;
  case AE_EXP1_L:
    u32Value = stCamData.au32TgtExp[1] & 0xFFFF;
    stCamData.au32TgtExp[1] =
        (stCamData.au32TgtExp[1] & 0xFFFF0000) | (u32Data & 0xFFFF);
    u8Flg = 1;
    break;
  case AE_EXP2_H:
    u32Value = stCamData.au32TgtExp[2] >> 16;
    stCamData.au32TgtExp[2] =
        (stCamData.au32TgtExp[2] & 0xFFFF) | ((u32Data & 0xFFFF) << 16);
    u8Flg = 1;
    break;
  case AE_EXP2_L:
    u32Value = stCamData.au32TgtExp[2] & 0xFFFF;
    stCamData.au32TgtExp[2] =
        (stCamData.au32TgtExp[2] & 0xFFFF0000) | (u32Data & 0xFFFF);
    u8Flg = 1;
    break;
  case AE_EXP3_H:
    u32Value = stCamData.au32TgtExp[3] >> 16;
    stCamData.au32TgtExp[3] =
        (stCamData.au32TgtExp[3] & 0xFFFF) | ((u32Data & 0xFFFF) << 16);
    u8Flg = 1;
    break;
  case AE_EXP3_L:
    u32Value = stCamData.au32TgtExp[3] & 0xFFFF;
    stCamData.au32TgtExp[3] =
        (stCamData.au32TgtExp[3] & 0xFFFF0000) | (u32Data & 0xFFFF);
    u8Flg = 1;
    break;
  case AE_LUMTGT0:
    u32Value = stCamData.au32TgtLum[0] & 0xFF;
    stCamData.au32TgtLum[0] = u32Data & 0xFF;
    u8Flg = 1;
    break;
  case AE_LUMTGT1:
    u32Value = stCamData.au32TgtLum[1] & 0xFF;
    stCamData.au32TgtLum[1] = u32Data & 0xFF;
    u8Flg = 1;
    break;
  case AE_LUMTGT2:
    u32Value = stCamData.au32TgtLum[2] & 0xFF;
    stCamData.au32TgtLum[2] = u32Data & 0xFF;
    u8Flg = 1;
    break;
  case AE_LUMTGT3:
    u32Value = stCamData.au32TgtLum[3] & 0xFF;
    stCamData.au32TgtLum[3] = u32Data & 0xFF;
    u8Flg = 1;
    break;
  case GAMMA_BY_AGC:
    u32Value = stCamData.u32GammaAgc & 0xFF;
    stCamData.u32GammaAgc = u32Data & 0xFF;
    u8Flg = 1;
    break;
  case GAMMA_THRESHOLD_1:
    u32Value = stCamData.u32GamAgcStartLvl >> 2;
    stCamData.u32GamAgcStartLvl = u32Data << 2;
    u8Flg = 1;
    break;
  case GAMMA_THRESHOLD_2:
    u32Value = stCamData.u32GamAgcEndLvl >> 2;
    stCamData.u32GamAgcEndLvl = u32Data << 2;
    u8Flg = 1;
    break;
  default:
    break;
  }
  u32Value = CLIP3(u32Value, 0x0000, 0xFFFF);
  if (u8Mode == 1) // Write
  {
    if (u8Flg == 1) {
      s32Ret = camera_para_set(&stCamData);
      if (s32Ret != XM_SUCCESS) {
        putstr(0, "camera_para_set failed!\r\n");
        return XM_FAILURE;
      }
    } else {
      s32Ret = XM_MPI_ISP_SetExposureAttr(ISP_USED, &stExpAttr);
      if (s32Ret != XM_SUCCESS) {
        putstr(0, "XM_MPI_ISP_SetExposureAttr failed!\r\n");
        return XM_FAILURE;
      }
    }
  }
  *pu32Data = u32Value;
  return XM_SUCCESS;
}

static XM_S32 IspDebugOther(unsigned char u8Mode, unsigned int u32Addr,
                            XM_U32 *pu32Data) {
#if 0
  static XM_U16 su16En = 0xF0F0;
  static XM_U8 su8Year = 0xF0;
  static XM_U8 su8Mon = 0xF0;
  static XM_U8 su8Day = 0xF0;
  static XM_U16 su16Cus = 0xFFFE;
  static XM_U8 su8VerEn = 0xF0;
  XM_U32 u32Data, u32Value, u32Tmp1;
  CAM_INIT_DATA stCamData;
  XM_U16 u16Tmp;
  if (camera_para_get(&stCamData) != XM_SUCCESS) {
    putstr(0, "camera_para_get failed!\r\n");
    return XM_FAILURE;
  }
  if (su16En == 0xF0F0) {
    u16Tmp = 0xFFFF;
    u32Addr = ISPFLASH_EN - ISP_TOOLS_BASE + FLASH_ADDR_ISP;
    SysReadFromFlash((XM_U8 *)&u16Tmp, u32Addr, sizeof(XM_U16));
    su16En = u16Tmp;
  }
  if (su8Year == 0xF0) {
    u16Tmp = 0xFFFF;
    u32Addr = OTHER_YEAR - ISP_TOOLS_BASE + FLASH_ADDR_ISP;
    ;
    SysReadFromFlash((XM_U8 *)&u16Tmp, u32Addr, sizeof(XM_U16));
    su8Year = (XM_U8)(u16Tmp & 0xFF);
  }
  if (su8Mon == 0xF0) {
    u16Tmp = 0xFFFF;
    u32Addr = OTHER_MON - ISP_TOOLS_BASE + FLASH_ADDR_ISP;
    ;
    SysReadFromFlash((XM_U8 *)&u16Tmp, u32Addr, sizeof(XM_U16));
    su8Mon = (XM_U8)(u16Tmp & 0xFF);
  }
  if (su8Day == 0xF0) {
    u16Tmp = 0xFFFF;
    u32Addr = OTHER_DAY - ISP_TOOLS_BASE + FLASH_ADDR_ISP;
    ;
    SysReadFromFlash((XM_U8 *)&u16Tmp, u32Addr, sizeof(XM_U16));
    su8Day = (XM_U8)(u16Tmp & 0xFF);
  }
  if (su16Cus == 0xFFFE) {
    u16Tmp = 0xFFFF;
    u32Addr = OTHER_CUSTOMER - ISP_TOOLS_BASE + FLASH_ADDR_ISP;
    ;
    SysReadFromFlash((XM_U8 *)&u16Tmp, u32Addr, sizeof(XM_U16));
    su16Cus = u16Tmp;
  }
  if (su8VerEn == 0xF0) {
    u16Tmp = 0xFFFF;
    u32Addr = OTHER_VERSION_EN - ISP_TOOLS_BASE + FLASH_ADDR_ISP;
    ;
    SysReadFromFlash((XM_U8 *)&u16Tmp, u32Addr, sizeof(XM_U16));
    su8VerEn = u16Tmp ? 1 : 0;
  }
  u32Data = *pu32Data;
  switch (u32Addr) {
  case ISPFLASH_EN:
    if (su16En != 0xA55A) {
      u32Value = 0;
    } else {
      u32Value = 0xA55A;
    }
    if (u8Mode == 1) // Write
    {
      su16En = u32Data ? 0xA55A : 0;
    }
    break;
#if 0
  case OTHER_EE_OPN:
    u32Value = stCamData.u32EshutterLvEn;
    stCamData.u32EshutterLvEn = CLIP3(u32Data, 0, 0xFFFFFFFF);
    break;
  case OTHER_EE_CLS:
    u32Value = stCamData.u32EshutterLvDis;
    stCamData.u32EshutterLvDis = CLIP3(u32Data, 0, 0xFFFFFFFF);
    break;
  case OTHER_TDN_DAY1:
    u32Value = stCamData.u32DnThrDay[0];
    stCamData.u32DnThrDay[0] = CLIP3(u32Data, 0, 0xFFFFFFFF);
    break;
  case OTHER_TDN_NIT1:
    u32Value = stCamData.u32DnThrNight[0];
    stCamData.u32DnThrNight[0] = CLIP3(u32Data, 0, 0xFFFFFFFF);
    break;

  case OTHER_TDN_DAY2:
    u32Value = stCamData.u32DnThrDay[1];
    stCamData.u32DnThrDay[1] = CLIP3(u32Data, 0, 0xFFFFFFFF);
    break;
  case OTHER_TDN_NIT2:
    u32Value = stCamData.u32DnThrNight[1];
    stCamData.u32DnThrNight[1] = CLIP3(u32Data, 0, 0xFFFFFFFF);
    break;
  case OTHER_TDN_DAY3:
    u32Value = stCamData.u32DnThrDay[2];
    stCamData.u32DnThrDay[2] = CLIP3(u32Data, 0, 0xFFFFFFFF);
    break;
  case OTHER_TDN_NIT3:
    u32Value = stCamData.u32DnThrNight[2];
    stCamData.u32DnThrNight[2] = CLIP3(u32Data, 0, 0xFFFFFFFF);
    break;
  case OTHER_TDN_DAY4:
    u32Value = stCamData.u32DnThrDay[3];
    stCamData.u32DnThrDay[3] = CLIP3(u32Data, 0, 0xFFFFFFFF);
    break;
  case OTHER_TDN_NIT4:
    u32Value = stCamData.u32DnThrNight[3];
    stCamData.u32DnThrNight[3] = CLIP3(u32Data, 0, 0xFFFFFFFF);
    break;
  case OTHER_TDN_DAY5:
    u32Value = stCamData.u32DnThrDay[4];
    stCamData.u32DnThrDay[4] = CLIP3(u32Data, 0, 0xFFFFFFFF);
    break;
  case OTHER_TDN_NIT5:
    u32Value = stCamData.u32DnThrNight[4];
    stCamData.u32DnThrNight[4] = CLIP3(u32Data, 0, 0xFFFFFFFF);
    break;
#endif
  case OTHER_VERSION_EN:
    u32Value = (XM_U32)su8VerEn;
    if (u8Mode == 1) // Write
    {
      su8VerEn = u32Data ? 1 : 0;
    }
    break;
  case OTHER_OSDMENU_X:
    osd_get_menu_left_top((uint32 *)&u32Value, (uint32 *)&u32Tmp1);
    if (u8Mode == 1) // Write
    {
      u32Tmp1 = CLIP3(u32Tmp1, 0, 1000);
      u32Data = CLIP3(u32Data, 0, 1000);
      osd_set_menu_left_top((uint32)u32Data, (uint32)u32Tmp1);
    }
    break;
    break;
  case OTHER_OSDMENU_Y:
    osd_get_menu_left_top((uint32 *)&u32Tmp1, (uint32 *)&u32Value);
    if (u8Mode == 1) // Write
    {
      u32Tmp1 = CLIP3(u32Tmp1, 0, 1000);
      u32Data = CLIP3(u32Data, 0, 1000);
      osd_set_menu_left_top((uint32)u32Tmp1, (uint32)u32Data);
    }
    break;
  case OTHER_INFRARED_SWAP:
    u32Value = stCamData.s32Rvs & 0x01;
    if (u8Mode == 1) // Write
    {
      if (u32Data) {
        stCamData.s32Rvs |= 0x01;
      } else {
        stCamData.s32Rvs &= ~0x01;
      }
      if (camera_para_set(&stCamData) != XM_SUCCESS) {
        putstr(0, "camera_para_set failed!\r\n");
        return XM_FAILURE;
      }
    }
    break;
  case OTHER_YEAR:
    u32Value = (XM_U32)su8Year;
    if (u8Mode == 1) // Write
    {
      su8Year = u32Data & 0xFF;
    }
    break;
  case OTHER_MON:
    u32Value = (XM_U32)su8Mon;
    if (u8Mode == 1) // Write
    {
      su8Mon = u32Data & 0xFF;
    }
    break;
  case OTHER_DAY:
    u32Value = (XM_U32)su8Day;
    if (u8Mode == 1) // Write
    {
      su8Day = u32Data & 0xFF;
    }
    break;
  case OTHER_CUSTOMER:
    u32Value = (XM_U32)su16Cus;
    if (u8Mode == 1) // Write
    {
      su16Cus = u32Data & 0xFFFF;
    }
    break;
  default:
    u32Value = 0xFFFFFFFF;
    break;
  }
  *pu32Data = u32Value;
#endif
  return XM_SUCCESS;
}

static XM_S32 IspDebugVencTx(unsigned char u8Mode, unsigned int u32Addr,
                             XM_U32 *pu32Data) {
#if (defined CHIPID_XM330) || (defined CHIPID_XM350)
  XM_U32 u32Data, u32Value;
  VENC_TX_ATTR stTxAttr;
  XM_U8 u8Enable;
  if (camera_get_txAttr(&u8Enable, &stTxAttr) != XM_SUCCESS) {
    ERR("camera_get_txAttr failed!\n");
    return XM_FAILURE;
  }
  u32Data = *pu32Data;
  switch (u32Addr) {
  case PQ_TX_EN:
    u32Value = (XM_U32)u8Enable;
    u8Enable = (XM_U8)CLIP3(u32Data, 0, 1);
    break;
  case PQ_TX_TEST:
    u32Value = (XM_U32)stTxAttr.u32TestMode;
    stTxAttr.u32TestMode = u32Data;
    break;
  case PQ_TX_BLANKVAL_PAL:
    u32Value = (XM_U32)stTxAttr.u32BlankVal_P;
    stTxAttr.u32BlankVal_P = u32Data;
    break;
  case PQ_TX_SYNCVAL_PAL:
    u32Value = (XM_U32)stTxAttr.u32SyncVal_P;
    stTxAttr.u32SyncVal_P = u32Data;
    break;
  case PQ_TX_CSYNCVAL_PAL:
    u32Value = (XM_U32)stTxAttr.u32CSyncVal_P;
    stTxAttr.u32CSyncVal_P = u32Data;
    break;
  case PQ_TX_SUBCARRIER_H_PAL:
    u32Value = (stTxAttr.u32SubCarrierPar_P >> 16) & 0xFFFF;
    stTxAttr.u32SubCarrierPar_P =
        (stTxAttr.u32SubCarrierPar_P & 0xFFFF) | ((u32Data & 0xFFFF) << 16);
    break;
  case PQ_TX_SUBCARRIER_L_PAL:
    u32Value = stTxAttr.u32SubCarrierPar_P & 0xFFFF;
    stTxAttr.u32SubCarrierPar_P =
        (stTxAttr.u32SubCarrierPar_P & 0xFFFF0000) | (u32Data & 0xFFFF);
    break;
  case PQ_TX_YCGAIN_H_PAL:
    u32Value = (stTxAttr.u32YColorGain_P >> 16) & 0xFFFF;
    stTxAttr.u32YColorGain_P =
        (stTxAttr.u32YColorGain_P & 0xFFFF) | ((u32Data & 0xFFFF) << 16);
    break;
  case PQ_TX_YCGAIN_L_PAL:
    u32Value = stTxAttr.u32YColorGain_P & 0xFFFF;
    stTxAttr.u32YColorGain_P =
        (stTxAttr.u32YColorGain_P & 0xFFFF0000) | (u32Data & 0xFFFF);
    break;

  case PQ_TX_BLANKVAL_NTSC:
    u32Value = (XM_U32)stTxAttr.u32BlankVal_N;
    stTxAttr.u32BlankVal_N = u32Data;
    break;
  case PQ_TX_SYNCVAL_NTSC:
    u32Value = (XM_U32)stTxAttr.u32SyncVal_N;
    stTxAttr.u32SyncVal_N = u32Data;
    break;
  case PQ_TX_CSYNCVAL_NTSC:
    u32Value = (XM_U32)stTxAttr.u32CSyncVal_N;
    stTxAttr.u32CSyncVal_N = u32Data;
    break;
  case PQ_TX_SUBCARRIER_H_NTSC:
    u32Value = (stTxAttr.u32SubCarrierPar_N >> 16) & 0xFFFF;
    stTxAttr.u32SubCarrierPar_N =
        (stTxAttr.u32SubCarrierPar_N & 0xFFFF) | ((u32Data & 0xFFFF) << 16);
    break;
  case PQ_TX_SUBCARRIER_L_NTSC:
    u32Value = stTxAttr.u32SubCarrierPar_N & 0xFFFF;
    stTxAttr.u32SubCarrierPar_N =
        (stTxAttr.u32SubCarrierPar_N & 0xFFFF0000) | (u32Data & 0xFFFF);
    break;
  case PQ_TX_YCGAIN_H_NTSC:
    u32Value = (stTxAttr.u32YColorGain_N >> 16) & 0xFFFF;
    stTxAttr.u32YColorGain_N =
        (stTxAttr.u32YColorGain_N & 0xFFFF) | ((u32Data & 0xFFFF) << 16);
    break;
  case PQ_TX_YCGAIN_L_NTSC:
    u32Value = stTxAttr.u32YColorGain_N & 0xFFFF;
    stTxAttr.u32YColorGain_N =
        (stTxAttr.u32YColorGain_N & 0xFFFF0000) | (u32Data & 0xFFFF);
    break;

  default:
    u32Value = 0xFFFFFFFF;
    break;
  }

  if (u8Mode == 1) // Write
  {
    if (camera_set_txAttr(u8Enable, stTxAttr) != XM_SUCCESS) {
      ERR("camera_set_txAttr failed!\n");
      return XM_FAILURE;
    }
  }
  *pu32Data = u32Value;
#endif
  return XM_SUCCESS;
}

static XM_S32 IspDebugMenu(unsigned char u8Mode, unsigned int u32Addr,
                           XM_U32 *pu32Data) {
#if 0
  XM_S32 s32Ret;
  XM_U32 u32Cmd;
  XM_U16 u16Tmp;
  if (u32Addr < MENU_ENABLE || u32Addr > MENU_END)
    return XM_FAILURE;
  u32Cmd = u32Addr - ISP_TOOLS_BASE + F_DEBUG_BASE;
  // to Now MenuConfig(MENU_CFGUSE_OFST)
  u32Addr = FLASH_ADDR_ISP + u32Addr + MENU_CFGUSE_OFST - ISP_TOOLS_BASE;
  s32Ret = ReadFlashInterface((XM_U8 *)&u16Tmp, u32Addr, sizeof(XM_U16));
  if (s32Ret != XM_TRUE) {
    putstr(0, "ReadFlashInterface failed!\n");
    return XM_FAILURE;
  }
  if (u8Mode == 1) // Write
  {
    XM_MPI_MENU_SetCfg(0, u32Cmd, pu32Data);
    return XM_SUCCESS;
  }
  *pu32Data = (XM_U32)u16Tmp;
  return XM_SUCCESS;
}

#if 0
static XM_U8 gau8AeTest0[7][7] = {
    {1, 1, 1, 1, 1, 1, 1}, {1, 1, 1, 1, 1, 1, 1}, {1, 1, 1, 1, 1, 1, 1},
    {1, 1, 1, 1, 1, 1, 1}, {1, 1, 1, 1, 1, 1, 1}, {1, 1, 1, 1, 1, 1, 1},
    {1, 1, 1, 1, 1, 1, 1},
};
static XM_U8 gau8AeTest1[7][7] = {
    {01, 01, 01, 01, 01, 01, 01}, {01, 01, 02, 02, 02, 01, 01},
    {01, 02, 02, 02, 02, 02, 01}, {01, 02, 02, 02, 02, 02, 01},
    {01, 02, 02, 02, 02, 02, 01}, {01, 01, 02, 02, 02, 01, 01},
    {01, 01, 01, 01, 01, 01, 01},
};
static XM_U8 gau8AeTest2[7][7] = {
    {01, 01, 01, 01, 01, 01, 01}, {01, 01, 02, 02, 02, 01, 01},
    {01, 02, 04, 04, 04, 02, 01}, {01, 02, 04, 04, 04, 02, 01},
    {01, 02, 04, 04, 04, 02, 01}, {01, 01, 02, 02, 02, 01, 01},
    {01, 01, 01, 01, 01, 01, 01},
};

static XM_S32 AeTest(XM_U8 u8Mode) {
  XM_S32 s32Ret;
  ISP_EXPOSURE_ATTR_S stExpAttr;
  XM_U8 u8i, u8j;
  s32Ret = XM_MPI_ISP_GetExposureAttr(&stExpAttr);
  if (s32Ret != XM_SUCCESS) {
    putstr(0, "XM_MPI_ISP_GetExposureAttr failed!\r\n");
    return XM_FAILURE;
  }
  if (u8Mode == 0) {
    for (u8i = 0; u8i < 7; u8i++)
      for (u8j = 0; u8j < 7; u8j++) {
        stExpAttr.stAuto.au8Weight[u8i][u8j] = gau8AeTest0[u8i][u8j];
      }
    putstr(0, "gau8AeTest0 \n\r");
  } else if (u8Mode == 1) {
    for (u8i = 0; u8i < 7; u8i++)
      for (u8j = 0; u8j < 7; u8j++) {
        stExpAttr.stAuto.au8Weight[u8i][u8j] = gau8AeTest1[u8i][u8j];
      }
    putstr(0, "gau8AeTest1 \n\r");
  } else {
    for (u8i = 0; u8i < 7; u8i++)
      for (u8j = 0; u8j < 7; u8j++) {
        stExpAttr.stAuto.au8Weight[u8i][u8j] = gau8AeTest2[u8i][u8j];
      }
    putstr(0, "gau8AeTest2 \n\r");
  }
  s32Ret = XM_MPI_ISP_SetExposureAttr(ISP_USED, &stExpAttr);
  if (s32Ret != XM_SUCCESS) {
    putstr(0, "XM_MPI_ISP_GetExposureAttr failed!\r\n");
    return XM_FAILURE;
  }
#endif
#endif
  return XM_SUCCESS;
}
#endif

XM_S32 IspDebugCameraSetting(unsigned char u8Mode, unsigned int u32Addr,
                             XM_U32 *pu32Data) {
  CAMERA_COLOR stCameraColor;
  XM_S32 s32Ret;
  XM_U32 u32Data, u32Value, u32Tmp1, u32Tmp2, u32Tmp3;
  XM_U8 u8Tmp;
  u32Data = *pu32Data;
  u32Value = 0xFFFFFFFF;
  switch (u32Addr) {
  case CAMERA_EXP_MODE:
    s32Ret = camera_get_exposure_level((XM_S32 *)&u32Tmp1, &u32Tmp2, &u32Tmp3);
    if (s32Ret != XM_SUCCESS)
      return XM_FAILURE;
    u32Value = u32Tmp1;
    u32Tmp1 = CLIP3(u32Data, 0, 0xFF);
    if (u8Mode == 1) // Write
    {
      s32Ret = camera_set_exposure_level((XM_S32)u32Tmp1, u32Tmp2, u32Tmp3);
      if (s32Ret != XM_SUCCESS)
        return XM_FAILURE;
    }

    break;
  case CAMERA_EXP_AUTO_LMT:
    s32Ret = camera_get_exposure_level((XM_S32 *)&u32Tmp1, &u32Tmp2, &u32Tmp3);
    if (s32Ret != XM_SUCCESS)
      return XM_FAILURE;
    u32Value = (u32Tmp2 << 16) | u32Tmp3;
    u32Tmp2 = (u32Data >> 16) & 0xFFFF;
    u32Tmp3 = u32Data & 0xFFFF;
    u32Tmp2 = CLIP3(u32Tmp2, 0, 0xFFFF);
    u32Tmp3 = CLIP3(u32Tmp3, 0, 0xFFFF);
    if (u8Mode == 1) // Write
    {
      s32Ret = camera_set_exposure_level(u32Tmp1, u32Tmp2, u32Tmp3);
      if (s32Ret != XM_SUCCESS)
        return XM_FAILURE;
    }
    break;
  case CAMERA_DN_MODE:
    s32Ret = camera_get_dnc_mode(&u32Tmp1);
    if (s32Ret != XM_SUCCESS)
      return XM_FAILURE;
    u32Value = u32Tmp1;
    u32Tmp1 = CLIP3(u32Data, DNC_AUTO, DNC_BLACKWHITE);
    if (u8Mode == 1) // Write
    {
      s32Ret = camera_set_dnc_mode(u32Tmp1);
      if (s32Ret != XM_SUCCESS)
        return XM_FAILURE;
    }
    break;
  case CAMERA_BLC:
    s32Ret = camera_get_blc(&u32Tmp1);
    if (s32Ret != XM_SUCCESS)
      return XM_FAILURE;
    u32Value = u32Tmp1;
    u32Tmp1 = CLIP3(u32Data, 0, 1);
    if (u8Mode == 1) // Write
    {
      s32Ret = camera_set_blc(u32Tmp1);
      if (s32Ret != XM_SUCCESS)
        return XM_FAILURE;
    }
    break;
  case CAMERA_SCENE_MODE:
    s32Ret = camera_get_scene((CAMERA_SCENE *)&u32Tmp1);
    if (s32Ret != XM_SUCCESS)
      return XM_FAILURE;
    u32Value = u32Tmp1;
    u32Tmp1 = CLIP3(u32Data, SCENE_AUTO, SCENE_OUTDOOR);
    if (u8Mode == 1) // Write
    {
      s32Ret = camera_set_scene((CAMERA_SCENE)u32Tmp1);
      if (s32Ret != XM_SUCCESS)
        return XM_FAILURE;
    }
    break;
  case CAMERA_AE_TGT:
    u32Value = camera_get_refrence_level();
    u32Tmp1 = CLIP3(u32Data, 0, 100);
    if (u8Mode == 1) // Write
    {
      s32Ret = camera_set_refrence_level(u32Tmp1);
      if (s32Ret != XM_SUCCESS)
        return XM_FAILURE;
    }
    break;
  case CAMERA_DN_THR:
    s32Ret = camera_get_dnc_thr((XM_S32 *)&u32Tmp1);
    if (s32Ret != XM_SUCCESS)
      return XM_FAILURE;
    u32Value = u32Tmp1;
    u32Tmp1 = CLIP3(u32Data, 10, 50);
    if (u8Mode == 1) // Write
    {
      s32Ret = camera_set_dnc_thr((XM_S32)u32Tmp1);
      if (s32Ret != XM_SUCCESS)
        return XM_FAILURE;
    }
    break;
  case CAMERA_DWDR_EN:
    s32Ret = camera_get_wdr((XM_S32 *)&u32Tmp1, (XM_S32 *)&u32Tmp2);
    if (s32Ret != XM_SUCCESS)
      return XM_FAILURE;
    u32Value = u32Tmp2;
    u32Tmp2 = CLIP3(u32Data, 0, 1);
    if (u8Mode == 1) // Write
    {
      s32Ret = camera_set_wdr((XM_S32)u32Tmp1, (XM_S32)u32Tmp2);
      if (s32Ret != XM_SUCCESS)
        return XM_FAILURE;
    }
    break;
  case CAMERA_DWDR_STR:
    s32Ret = camera_get_wdr((XM_S32 *)&u32Tmp1, (XM_S32 *)&u32Tmp2);
    if (s32Ret != XM_SUCCESS)
      return XM_FAILURE;
    u32Value = u32Tmp1;
    u32Tmp1 = CLIP3(u32Data, 0, 100);
    if (u8Mode == 1) // Write
    {
      s32Ret = camera_set_wdr((XM_S32)u32Tmp1, (XM_S32)u32Tmp2);
      if (s32Ret != XM_SUCCESS)
        return XM_FAILURE;
    }
    break;
  case CAMERA_AE_SENSITIVITY:
    s32Ret = camera_get_ae_sensitivity((XM_S32 *)&u32Tmp1);
    if (s32Ret != XM_SUCCESS)
      return XM_FAILURE;
    u32Value = u32Tmp1;
    u32Tmp1 = CLIP3(u32Data, 1, 10);
    if (u8Mode == 1) // Write
    {
      s32Ret = camera_set_ae_sensitivity((XM_S32)u32Tmp1);
      if (s32Ret != XM_SUCCESS)
        return XM_FAILURE;
    }
    break;
  case CAMERA_AE_SENSITIVITY2:
    s32Ret = camera_get_ae_sensitivity2((XM_S32 *)&u32Tmp1);
    if (s32Ret != XM_SUCCESS)
      return XM_FAILURE;
    u32Value = u32Tmp1;
    u32Tmp1 = CLIP3(u32Data, 1, 10);
    if (u8Mode == 1) // Write
    {
      s32Ret = camera_set_ae_sensitivity2((XM_S32)u32Tmp1);
      if (s32Ret != XM_SUCCESS)
        return XM_FAILURE;
    }
    break;
  case CAMERA_STYLE:
    s32Ret = Camera_Get_StyleMode((XM_S32 *)&u32Tmp1);
    if (s32Ret != XM_SUCCESS)
      return XM_FAILURE;
    u32Value = u32Tmp1;
    u32Tmp1 = CLIP3(u32Data, 0, 2);
    if (u8Mode == 1) // Write
    {
      s32Ret = Camera_Set_StyleMode((XM_S32)u32Tmp1);
      if (s32Ret != XM_SUCCESS)
        return XM_FAILURE;
    }
    break;
  case CAMERA_AUTO_GAIN:
    s32Ret = camera_get_gain((XM_S32 *)&u32Tmp1, (XM_S32 *)&u32Tmp2);
    if (s32Ret != XM_SUCCESS)
      return XM_FAILURE;
    u32Value = u32Tmp2;
    u32Tmp2 = CLIP3(u32Data, 0, 2);
    if (u8Mode == 1) // Write
    {
      s32Ret = camera_set_gain((XM_S32)u32Tmp1, (XM_S32)u32Tmp2);
      if (s32Ret != XM_SUCCESS)
        return XM_FAILURE;
    }
    break;
  case CAMERA_AUTO_GAIN_STR:
    s32Ret = camera_get_gain((XM_S32 *)&u32Tmp1, (XM_S32 *)&u32Tmp2);
    if (s32Ret != XM_SUCCESS)
      return XM_FAILURE;
    u32Value = u32Tmp1;
    u32Tmp1 = CLIP3(u32Data, 0, 100);
    if (u8Mode == 1) // Write
    {
      s32Ret = camera_set_gain((XM_S32)u32Tmp1, (XM_S32)u32Tmp2);
      if (s32Ret != XM_SUCCESS)
        return XM_FAILURE;
    }
    break;
  case CAMERA_ESHUTTER:
    s32Ret = camera_get_es_shutter((XM_S32 *)&u32Tmp1);
    if (s32Ret != XM_SUCCESS)
      return XM_FAILURE;
    u32Value = u32Tmp1;
    if (u8Mode == 1) // Write
    {
      u32Tmp1 = CLIP3(u32Data, 0, 6);
      s32Ret = camera_set_es_shutter((XM_S32)u32Tmp1);
      if (s32Ret != XM_SUCCESS)
        return XM_FAILURE;
    }
    break;
  case CAMERA_IRCUT_MODE:
    s32Ret = camera_get_ircut_mode((XM_S32 *)&u32Tmp1);
    if (s32Ret != XM_SUCCESS)
      return XM_FAILURE;
    u32Value = u32Tmp1;
    u32Tmp1 = CLIP3(u32Data, IRCUT_SYN_INFRARED, IRCUT_SWITCH_AUTO);
    if (u8Mode == 1) // Write
    {
      s32Ret = camera_set_ircut_mode((XM_S32)u32Tmp1);
      if (s32Ret != XM_SUCCESS)
        return XM_FAILURE;
    }
    break;
  case CAMERA_NR_DAY:
    s32Ret = CameraGetNFLevel(0, (XM_S32 *)&u32Tmp1);
    if (s32Ret != XM_SUCCESS)
      return XM_FAILURE;
    u32Value = u32Tmp1;
    u32Tmp1 = CLIP3(u32Data, 0, 5);
    if (u8Mode == 1) // Write
    {
      s32Ret = CameraSetNFLevel(0, (XM_S32)u32Tmp1);
      if (s32Ret != XM_SUCCESS)
        return XM_FAILURE;
    }
    break;
  case CAMERA_NR_NIGHT:
    s32Ret = CameraGetNFLevel(1, (XM_S32 *)&u32Tmp1);
    if (s32Ret != XM_SUCCESS)
      return XM_FAILURE;
    u32Value = u32Tmp1;
    u32Tmp1 = CLIP3(u32Data, 1, 5);
    if (u8Mode == 1) // Write
    {
      s32Ret = CameraSetNFLevel(1, (XM_S32)u32Tmp1);
      if (s32Ret != XM_SUCCESS)
        return XM_FAILURE;
    }
    break;
  case CAMERA_MIRROR:
    s32Ret = camera_get_mirror((XM_S32 *)&u32Tmp1);
    if (s32Ret != XM_SUCCESS)
      return XM_FAILURE;
    u32Value = u32Tmp1;
    u32Tmp1 = CLIP3(u32Data, 0, 1);
    if (u8Mode == 1) // Write
    {
      s32Ret = camera_set_mirror((XM_S32)u32Tmp1);
      if (s32Ret != XM_SUCCESS)
        return XM_FAILURE;
    }
    break;
  case CAMERA_FLIP:
    s32Ret = camera_get_flip((XM_S32 *)&u32Tmp1);
    if (s32Ret != XM_SUCCESS)
      return XM_FAILURE;
    u32Value = u32Tmp1;
    u32Tmp1 = CLIP3(u32Data, 0, 1);
    if (u8Mode == 1) // Write
    {
      s32Ret = camera_set_flip((XM_S32)u32Tmp1);
      if (s32Ret != XM_SUCCESS)
        return XM_FAILURE;
    }
    break;
  case CAMERA_DEFLICKER:
    s32Ret = camera_get_reject_flicker((XM_S32 *)&u32Tmp1);
    if (s32Ret != XM_SUCCESS)
      return XM_FAILURE;
    u32Value = u32Tmp1;
    u32Tmp1 = CLIP3(u32Data, 0, 1);
    if (u8Mode == 1) // Write
    {
      s32Ret = camera_set_reject_flicker((XM_S32)u32Tmp1);
      if (s32Ret != XM_SUCCESS)
        return XM_FAILURE;
    }
    break;
  case CAMERA_IRCUT_SWAP:
    s32Ret = CameraGetSwapICR((XM_S32 *)&u32Tmp1);
    if (s32Ret != XM_SUCCESS)
      return XM_FAILURE;
    u32Value = u32Tmp1;
    u32Tmp1 = CLIP3(u32Data, 0, 1);
    if (u8Mode == 1) // Write
    {
      s32Ret = CameraSwapICR((XM_S32)u32Tmp1);
      if (s32Ret != XM_SUCCESS)
        return XM_FAILURE;
    }
    break;
  case CAMERA_LUM:
    s32Ret = camera_get_color(0, &stCameraColor);
    if (s32Ret != XM_SUCCESS)
      return XM_FAILURE;
    u32Value = stCameraColor.Brightness;
    stCameraColor.Brightness = CLIP3(u32Data, 0, 100);
    if (u8Mode == 1) // Write
    {
      s32Ret = camera_set_color(0, &stCameraColor);
      if (s32Ret != XM_SUCCESS)
        return XM_FAILURE;
    }
    break;
  case CAMERA_CONTRAST:
    s32Ret = camera_get_color(0, &stCameraColor);
    if (s32Ret != XM_SUCCESS)
      return XM_FAILURE;
    u32Value = stCameraColor.Contrast;
    stCameraColor.Contrast = CLIP3(u32Data, 0, 100);
    if (u8Mode == 1) // Write
    {
      s32Ret = camera_set_color(0, &stCameraColor);
      if (s32Ret != XM_SUCCESS)
        return XM_FAILURE;
    }
    break;
  case CAMERA_SATURATION:
    s32Ret = camera_get_color(0, &stCameraColor);
    if (s32Ret != XM_SUCCESS)
      return XM_FAILURE;
    u32Value = stCameraColor.Saturation;
    stCameraColor.Saturation = CLIP3(u32Data, 0, 100);
    if (u8Mode == 1) // Write
    {
      s32Ret = camera_set_color(0, &stCameraColor);
      if (s32Ret != XM_SUCCESS)
        return XM_FAILURE;
    }
    break;
  case CAMERA_HUE:
    s32Ret = camera_get_color(0, &stCameraColor);
    if (s32Ret != XM_SUCCESS)
      return XM_FAILURE;
    u32Value = stCameraColor.Hue;
    stCameraColor.Hue = CLIP3(u32Data, 0, 100);
    if (u8Mode == 1) // Write
    {
      s32Ret = camera_set_color(0, &stCameraColor);
      if (s32Ret != XM_SUCCESS)
        return XM_FAILURE;
    }
    break;
  case CAMERA_SHARPEN:
    s32Ret = camera_get_color(0, &stCameraColor);
    if (s32Ret != XM_SUCCESS)
      return XM_FAILURE;
    u32Value = stCameraColor.Acutance;
    stCameraColor.Acutance = CLIP3(u32Data, 0, 100);
    if (u8Mode == 1) // Write
    {
      s32Ret = camera_set_color(0, &stCameraColor);
      if (s32Ret != XM_SUCCESS)
        return XM_FAILURE;
    }
    break;

  case CAMERA_ATFALSECOL:
    s32Ret = camera_get_antiFalseColor(&u32Value);
    if (s32Ret != XM_SUCCESS)
      return XM_FAILURE;
    u32Data = CLIP3(u32Data, 0, 100);
    if (u8Mode == 1) // Write
    {
      s32Ret = camera_set_antiFalseColor(u32Data);
      if (s32Ret != XM_SUCCESS)
        return XM_FAILURE;
    }
    break;
  case CAMERA_SAWTOOTH:
    s32Ret = camera_get_sawtooth(&u32Value);
    if (s32Ret != XM_SUCCESS)
      return XM_FAILURE;
    u32Data = CLIP3(u32Data, 0, 100);
    if (u8Mode == 1) // Write
    {
      s32Ret = camera_set_sawtooth(u32Data);
      if (s32Ret != XM_SUCCESS)
        return XM_FAILURE;
    }
    break;
  case CAMERA_GAMMA:
    s32Ret = camera_get_gamma(&u32Value);
    if (s32Ret != XM_SUCCESS)
      return XM_FAILURE;
    u32Data = CLIP3(u32Data, 0, 0xFFFFFF);
    if (u8Mode == 1) // Write
    {
      s32Ret = camera_set_gamma(u32Data);
      if (s32Ret != XM_SUCCESS)
        return XM_FAILURE;
    }
    break;
  case CAMERA_FPS_NOW:
    break;
  case CAMERA_ESHUTTER_EX:
    camera_es_shutter_ex(1, &u8Tmp);
    u32Value = (XM_U32)u8Tmp;
    if (u8Mode == 1) // Write
    {
      u8Tmp = CLIP3(u32Data, 0, 2);
      camera_es_shutter_ex(2, &u8Tmp);
    }
    break;
  case CAMERA_BURST_BW:
    camera_bwBurst_get(&u8Tmp);
    u32Value = (XM_U32)u8Tmp;
    if (u8Mode == 1) // Write
    {
      u8Tmp = CLIP3(u32Data, 0, 1);
      camera_bwBurst_set(u8Tmp);
    }
    break;
  case CAMERA_WB_RB:
    camera_get_wbRB(&u8Tmp);
    u32Value = (XM_U32)u8Tmp;
    if (u8Mode == 1) // Write
    {
      u8Tmp = CLIP3(u32Data, 0, 100);
      camera_set_wbRB(u8Tmp);
    }
    break;
  case CAMERA_WB_GM:
    camera_get_wbGM(&u8Tmp);
    u32Value = (XM_U32)u8Tmp;
    if (u8Mode == 1) // Write
    {
      u8Tmp = CLIP3(u32Data, 0, 100);
      camera_set_wbGM(u8Tmp);
    }
    break;

  case CAMERA_VSTD:
    break;
  case CAMERA_VENC:
    break;
  case CAMERA_RESOLUTION:
    break;

  case CAMERA_CONFIG_SAVE:
    break;
  case CAMERA_CONFIG_DEFAULT:
    break;
  case CAMERA_XVI_EN:
    break;
  case CAMERA_DEBUG:
    u32Value = 0;
    if (u8Mode == 1) // Write
    {
      if (u32Data)
        camera_debug("debug 1");
      else
        camera_debug("debug 0");
    }
    break;
  case CAMERA_TT:
    break;
  case CAMERA_DN_STATUS:
    camera_get_dn_state(&u8Tmp);
    u32Value = (XM_U32)u8Tmp;
    break;
  case CAMERA_KEY:
    break;
  case CAMERA_PRINT_EN:
    break;
  case CAMERA_GET_PRODUCT:
#if 0
    switch (gstProductInfo.u32DSPType) {
    case DSP_XM320:
      if (gstProductInfo.u8RsltType == P720_) {
        u32Value = DSP_XM310;
      } else {
        u32Value = DSP_XM320;
      }
      break;
    case DSP_XM322:
      u32Value = DSP_XM322;
      break;
    case DSP_XM350:
      if (gstProductInfo.u8RsltType == P720_) {
        u32Value = DSP_XM310V300;
      } else if (gstProductInfo.u8RsltType == P1080_) {
        u32Value = DSP_XM330V200;
      } else {
        u32Value = DSP_XM350;
      }
      break;
    default: // UnKnow
      u32Value = DSP_XM510 + 0xFF;
      break;
    }
#else
    u32Value = DSP_XM350;
#endif
    u32Value = (u32Value - DSP_XM510) & 0xFF;
    u32Value |= (gstProductInfo.u8RsltType << 8);
    break;
  default:
    u32Value = 0;
    break;
  }
  *pu32Data = u32Value;
  return XM_SUCCESS;
}

// Mode:   0:Read   1:Write
static int IspDebugTool(unsigned char u8Mode, unsigned int u32Addr,
                        unsigned int u32Data) {
#if OPEN_ISPDEBUG // Lycai @20160114
#ifdef CHIPID_XM350
  XM_S8 buff;
  XM_U32 buff1;
  XM_U32 u32i, u32j, u32AllAver = 0, u32Apixel, u32Aline;
  XM_S32 s32LineAver;
  XM_U32 FlashAddr = 0x1E000; // SBPC_DATA_BASE;
  ISP_FPN_ATTR_S pstFpnAttr;
#endif
  XM_S32 s32Ret = XM_FAILURE;
  // AE
  if ((TOOLS_AE_MASK & u32Addr) == TOOLS_AE_BASE) {
    s32Ret = IspDebugAe(u8Mode, u32Addr, &u32Data);
  }
  // AWB
  else if ((0xFFF00000 & u32Addr) == TOOLS_AWB_BASE) {
    if (u32Addr >= AWB_DEG_CAL_A0)
      s32Ret = IspDebgAwbCal(u8Mode, u32Addr, &u32Data);
    else
      s32Ret = IspDebugAwb(u8Mode, u32Addr, &u32Data);
  }
  // YUV_ Lum Contrast Hue Saturation
  else if ((0xFFF00000 & u32Addr) == TOOLS_CSC_BASE) {
    s32Ret = IspDebugCsc(u8Mode, u32Addr, &u32Data);
  }
  // Camera Setting
  else if ((0xFFF00000 & u32Addr) == CAMERA_BASE) {
    s32Ret = IspDebugCameraSetting(u8Mode, u32Addr, &u32Data);
  }
  // MENU
  else if ((MENU_MASK & u32Addr) == MENU_BASE) {
    s32Ret = IspDebugMenu(u8Mode, u32Addr, &u32Data);
  }
  // Nr
  else if ((NR2D_MASK & u32Addr) == TOOLS_NR_BASE) {
    if (u32Addr >= TOOLS_NR3D_BASE)
      s32Ret = IspDebugNr3D(u8Mode, u32Addr, &u32Data);
    else
      s32Ret = IspDebugNr(u8Mode, u32Addr, &u32Data);
  }
  // Sharpness
  else if ((SHARP_MASK & u32Addr) == TOOLS_SHARP_BASE) {
    s32Ret = IspDebugSharpeness(u8Mode, u32Addr, &u32Data);
  }
  // DPC
  else if ((DDPC_MASK & u32Addr) == DPC_BASE) {
    if ((u32Addr < DRC_BASE) ||
        ((u32Addr >= DPC_BASE + 0x80) && (u32Addr < DRC_BASE + 0x80)))
      s32Ret = IspDebugDpc(u8Mode, u32Addr, &u32Data);
    else
      s32Ret = IspDebugDRC(u8Mode, u32Addr, &u32Data);
  }
  // Black Level
  else if ((BLC_MASK & u32Addr) == BLC_BASE) {
    s32Ret = IspDebugBlc(u8Mode, u32Addr, &u32Data);
  }
  // Gamma
  else if ((GAM_MASK & u32Addr) == GAM_BASE) {
    s32Ret = IspDebugGamma(u8Mode, u32Addr, &u32Data);
  }
  // CCM
  else if ((CCM_MASK & u32Addr) == TOOLS_CCM_BASE) {
    s32Ret = IspDebugCcm(u8Mode, u32Addr, &u32Data);
  }
  // Chroma Setting
  else if ((CHROMA_MASK & u32Addr) == TOOLS_CHROMA_BASE) {
    s32Ret = IspDebugChroma(u8Mode, u32Addr, &u32Data);
  }
  // AntiFalseColor
  else if ((AFC_MASK & u32Addr) == TOOLS_ANTIFC_BASE) {
    s32Ret = IspDebugAntiFalseColor(u8Mode, u32Addr, &u32Data);
  }
  // Satutation
  else if ((SAT_MASK & u32Addr) == TOOLS_SAT_BASE) {
    s32Ret = IspDebugSatration(u8Mode, u32Addr, &u32Data);
  }
  // Other
  else if ((OTHER_MASK & u32Addr) == OTHER_BASE) {
    s32Ret = IspDebugOther(u8Mode, u32Addr, &u32Data);
  }
  // Ae 2
  else if ((AE_MASK & u32Addr) == AE_BASE) {
    s32Ret = IspDebugAe2(u8Mode, u32Addr, &u32Data);
  }
  // CSC(CFG)
  else if ((CSC_CFG_MASK & u32Addr) == CSC_CFG_BASE) {
    s32Ret = IspDebugCsc_V2(u8Mode, u32Addr, &u32Data);
  }
  // YCNr
  else if (((YCNR_MASK & u32Addr) == TOOLS_YCNR_BASE) ||
           ((YCNR_MASK & u32Addr) == TOOLS_YCNR_BASE2)) {
    s32Ret = IspDebugYcNr(u8Mode, u32Addr, &u32Data);
  }
  // VencTx
  else if ((u32Addr >= PQTOOLS_TX_BASE) && (u32Addr <= PQTOOLS_TX_BASE2)) {
    s32Ret = IspDebugVencTx(u8Mode, u32Addr, &u32Data);
  }
  // Awb(PQ_Tool)
  else if ((PQTOOLS_AWB_MASK & u32Addr) == PQTOOLS_AWB_BASE) {
    s32Ret = IspDebugAwbPQTool(u8Mode, u32Addr, &u32Data);
  } else {
    u32Data = 0;
    s32Ret = XM_SUCCESS;
  }
  if (s32Ret == XM_FAILURE) {
    u32Data = XM_FAILURE;
    putstr(0, "Err IspDebugTool failed!\r\n");
  }
  return u32Data;
#else
  return 0;
#endif
}

/************************************************************************
** Func Name	: UART_CMDCheck
** Discription	: 串口接收命令校验
**
** Parameter	: Null
** Return		: Null
************************************************************************/
uint8 UART_CMDCheck(uint8 uart_NO) {
  uint8 check_code = 0;
  uint8 num = 0;
  uint8 random_number = 0;
  uint8 temp = 0;

  num = m_pbCom_Data[COM_CMDLEN - 3];
  random_number = m_pbCom_Data[COM_CMDLEN - 2];
  check_code = m_pbCom_Data[COM_CMDLEN - 1];

  if (num > COM_CMDLEN)
    return FALSE;

  temp = (~m_pbCom_Data[num]) ^ random_number;

  if (temp != check_code) {
    return FALSE;
  }
  return TRUE;
}

/************************************************************************
** Func Name	: Com_DebugProcess
** Discription	: 串口处理函数
**
** Parameter	: Null
** Return		: Null
************************************************************************/
static void Com_DebugProcess(uint8 uart_NO) {
  uint8 i = 0;
  uint8 flag = 0;

  uint32 address = 0;
  uint32 data = 0;
  for (i = 1; i < COM_CMDLEN; i++) {
    m_pbCom_Data[i] = getbyte(g_uLComTimeoutS);
    if (eComErr == E_RECV_TIMEOUT) {
      return;
    }
  }

  // CMD CHECK
  flag = UART_CMDCheck(uart_NO);
  if (flag == FALSE) {
    putstr(uart_NO, "\r\n UART_CMDCheck Error!!");
    return;
  }

  address = m_pbCom_Data[2];
  address = address << 8;
  address |= m_pbCom_Data[3];
  address = address << 8;
  address |= m_pbCom_Data[4];
  address = address << 8;
  address |= m_pbCom_Data[5];

  data = m_pbCom_Data[6];
  data = data << 8;
  data |= m_pbCom_Data[7];
  data = data << 8;
  data |= m_pbCom_Data[8];
  data = data << 8;
  data |= m_pbCom_Data[9];

  switch (m_pbCom_Data[1]) {
  case CMD_DEBUG_REGWRTIE:
    if ((address & ISP_TOOLS_MASK) == 0xF8000000) // IspTool
    {
      // debug_lock(1);
      data = IspDebugTool(1, address, data);
    } else // Reg
    {
      XM_MPI_ISP_SetRegister(address, data);
    }
    break;

  case CMD_DEBUG_REGREAD:
    if ((address & ISP_TOOLS_MASK) == 0xF8000000) // IspTool
    {
      // debug_lock(1);
      data = IspDebugTool(0, address, data);
      putdword(uart_NO, data);
    } else // Reg
    {
      XM_MPI_ISP_GetRegister(address, &data);
      putdword(uart_NO, data);
    }
    break;

  case CMD_SENSOR_WRITE:
    break;

  case CMD_SENSOR_READ:
    break;
  default:
    break;
  }
}

/************************************************************************
** Func Name	: Com_process
** Discription	: 串口处理函数
**
** Parameter	: NULL
** Return		: 1: Uart debuging(Stop other server)  0:Normal(Do other
*server)
************************************************************************/
void *Com_process(void *arg) {
  uint8 ch = 0;
  DEBUG("Com_process start!\n");
#ifdef SOC_ALIOS
  gu8DebugCom_Loop = 0;
  DEBUG("ALIOS not support debug by uart now!\n");
#else
  xmprop_get_value_v2((XM_U8 *)"xmuart", (XM_VOID *)&gu8UartDebugEn);
  if (gu8UartDebugEn) {
    gu8DebugCom_Loop = 1;
    g_uLComTimeoutF = COM_TimeOut0;
    Init_UART0();
  } else {
    gu8DebugCom_Loop = 0;
  }
#endif
  while (gu8DebugCom_Loop) {
    ISR_UART0();
    // uart 0
    ch = getbyte(g_uLComTimeoutF);
    if (eComErr == E_RECV_OK) {
      m_pbCom_Data[0] = ch;
      if (m_pbCom_Data[0] == CMD_DEBUGMODE) {
        Com_DebugProcess(UART_0);
      } else if (ch >= CMD_VISCAMIN_MODE && ch <= CMD_VISCAMAX_MODE) {
        ;
      } else if (m_pbCom_Data[0] == 0x20) {
        ;
      }
    }
    SysDelay_ms(40);
  }
  return 0;
}
