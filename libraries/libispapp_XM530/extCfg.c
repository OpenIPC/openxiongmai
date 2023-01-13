#include "extCfg.h"
#include "mpi_ae.h"
#include "mpi_awb.h"
#include "mpi_isp.h"
#include "xm_awb_comm.h"
#include "xm_math.h"
#include "xm_print.h"
#include "xm_type.h"
XM_U8 *pgu8Buffer;

extern XM_S32 SysRegReadTmp(XM_U32 phyaddr, XM_U32 *val);
extern XM_S32 SysRegWriteTmp(XM_U32 phyaddr, XM_U32 val);

typedef struct _extcfg_base_s {
  XM_U32 u32Addr;
} EXTCFG_BASE_S;

typedef struct _extcfg_data_s {
  XM_U8 au8MarkID[4];
  XM_U8 u8FunMask_H;
  XM_U32 u32FunMask_L;
  EXTCFG_BASE_S astExtCfgBase[FUN_NUM];
} EXTCFG_DATA_S;

EXTCFG_DATA_S gstExtCfgData;
XM_U16 gu16Tolerance;

/***********************************************
gu8ExtCfgFlg:
        0: 存在配置文件(配置过---SOC)
        1: 存在配置文件(未配置过---ROM)
        2: 不存在配置文件
***********************************************/
static XM_U8 gu8ExtCfgFlg = 2;

XM_S32 ExtCfg_BaseAddrGet(XM_U8 u8SnsIdx, XM_U32 u32Module, XM_U32 *pu32Addr) {
  if ((gu8ExtCfgFlg == 2) || (u32Module > ROM_FUN_SHUTLOGIC)) {
    return XM_FAILURE;
  }
  *pu32Addr = gstExtCfgData.astExtCfgBase[u32Module].u32Addr;
  return XM_SUCCESS;
}

/*********************************************************
函数功能:	功能校验
输入参数:	u8Mode:    	0: Common Funciton
                                                        1: Sensor Function
                                u32Fun:		功能码
返回参数:	-1:		不支持
                                0:		支持
*********************************************************/
XM_S32 ExtCfg_CheckFun(XM_U8 u8Mode, XM_U32 u32Fun) {
  if (gu8ExtCfgFlg == 2) {
    return XM_FAILURE;
  }

  if ((u32Fun < 32) && (gstExtCfgData.u32FunMask_L & (1 << u32Fun))) {
    return XM_SUCCESS;
  } else if ((u32Fun >= 32) &&
             (gstExtCfgData.u8FunMask_H & (1 << (u32Fun - 32)))) {
    return XM_SUCCESS;
  }
  return XM_FAILURE;
}

// littleEndian
#if 0
XM_S32 ExtCfg_Read_V2(XM_U8 u8Bytes, XM_U32 u32Add, XM_U8 *pu8Data) {
  XM_U8 u8i;

  for (u8i = 0; u8i < u8Bytes; u8i++) {
    *pu8Data = *(pgu8Buffer + u32Add);
    u32Add++;
    pu8Data++;
  }
  return XM_SUCCESS;
}
#else
XM_S32 ExtCfg_Write_V2(XM_U16 u16Bytes, XM_U32 u32Add, XM_U8 *pu8Data) {
  XM_U16 u16i;
  XM_U8 *pu8Val;
  XM_U32 u32Val;
  u32Add += BUFFER_START;
  pu8Val = (XM_U8 *)&u32Val;
  for (u16i = 0; u16i < u16Bytes; u16i++) {
    SysRegReadTmp(((u32Add >> 2) << 2), &u32Val);
    *(pu8Val + u32Add % 4) = *pu8Data;
    SysRegWriteTmp(((u32Add >> 2) << 2), u32Val);
    u32Add++;
    pu8Data++;
  }
  return XM_SUCCESS;
}

XM_S32 ExtCfg_Read_V2(XM_U8 u8Bytes, XM_U32 u32Add, XM_U8 *pu8Data) {
  XM_U8 u8i;
  XM_U8 *pu8Val;
  XM_U32 u32Val;
  u32Add += BUFFER_START;
  pu8Val = (XM_U8 *)&u32Val;
  for (u8i = 0; u8i < u8Bytes; u8i++) {
    SysRegReadTmp(((u32Add >> 2) << 2), &u32Val);
    *pu8Data = *(pu8Val + u32Add % 4);
    u32Add++;
    pu8Data++;
  }
  return XM_SUCCESS;
}
#endif

//函数功能: 从配置文件读取数据 写入Isp寄存器
// u32Addr:  数据起始地址(配置文件)
// u8Num: 寄存器个数
// u8Mode: 0:Read From E2		1:Read From RAM
XM_S32 ExtCft_WriteIsp(XM_U8 u8Mode, XM_U32 u32Addr, XM_U16 u16Num) {
  XM_U32 au32RegAddr[2];
  XM_U32 au32RegData[2];
  XM_U8 u8AddrWidth, u8DataWidth;
  XM_U16 u16i, u16Len;
  XM_S32 (*pFun_cfgRead)(XM_U8 u8Bytes, XM_U32 u32Add, XM_U8 * pu8Data) = NULL;
  u8AddrWidth = 4;
  u8DataWidth = 4;
  if (u16Num > 0) {
    if (u8Mode)
      pFun_cfgRead = ExtCfg_Read_V2;
    else
      pFun_cfgRead = ExtCfg_Read_V2;
    u16Len = u8AddrWidth + u8DataWidth;
    for (u16i = 0; u16i < u16Num; u16i++) {
      au32RegAddr[0] = au32RegData[0] = 0;
      au32RegAddr[1] = au32RegData[1] = 0;
      (*pFun_cfgRead)(u8AddrWidth, u32Addr, (XM_U8 *)&au32RegAddr[0]); // Addr
      (*pFun_cfgRead)(u8DataWidth, u32Addr + u8AddrWidth,
                      (XM_U8 *)&au32RegData[0]); // Data
      (*pFun_cfgRead)(u8AddrWidth, u32Addr + u16Len,
                      (XM_U8 *)&au32RegAddr[1]); // Addr
      (*pFun_cfgRead)(u8DataWidth, u32Addr + u8AddrWidth + u16Len,
                      (XM_U8 *)&au32RegData[1]); // Data
      if ((u16Num > 2) && (u16i < u16Num - 2) && (au32RegAddr[0] == 0xF5) &&
          (au32RegData[0] == 0xA5) && (au32RegAddr[1] == 0x5A) &&
          (au32RegData[1] == 0x5F)) {
        u16i += 2;
        au32RegAddr[0] = au32RegData[0] = 0;
        u32Addr += (XM_U32)u16Len * 2;
        (*pFun_cfgRead)(u8AddrWidth, u32Addr, (XM_U8 *)&au32RegAddr[0]);
        (*pFun_cfgRead)(u8DataWidth, u32Addr + u8AddrWidth,
                        (XM_U8 *)&au32RegData[0]);

        au32RegAddr[0] = au32RegAddr[0] & 0xFF;
        au32RegAddr[0] = (au32RegAddr[0] << 8) | (au32RegData[0] & 0xFF);
        SysDelay_ms((XM_U32)au32RegAddr[0]);
#ifdef DEBUG_ON
        DEBUG("Sleepms:\n");
        PrintInt(8, au32RegAddr[0]);
        DBG("\r\n");
#endif
      } else {
        XM_MPI_ISP_SetRegister((XM_U32)au32RegAddr[0], (XM_U32)au32RegData[0]);

#ifdef DEBUG_ON
        PrintHex(8, au32RegAddr[0]);
        DBG("  ");
        PrintHex(8, au32RegData[0]);
        DBG("\r\n");
#endif
      }
      u32Addr += u16Len;
    }
  }
  return XM_SUCCESS;
}

/***********************************************************************
函数名称:	ExtCfg_GetExtCfgFlg
函数功能:	判断是否需要配置
输入参数:	无
输出参数:	无
返回参数:	0:	不用配置(前面(ROM)已配置过)
                                1:    需要配置(前面未配置过)
                                2: 	需要配置(无配置表)
                                -1: 	出错
Note:Lycai

        ROM 和ISP不能重复配置部分配置前判断
***********************************************************************/
XM_S32 ExtCfg_GetExtCfgFlg() { return (XM_S32)gu8ExtCfgFlg; }

XM_S32 ExtCft_GetIspWndRect(XM_U8 u8Fps, RECT_S *pstRect) {
  XM_U32 u32Addr, u32Data;
  XM_U8 u8Size;
  if (ExtCfg_BaseAddrGet(0, ROM_FUN_ISPCUT, &u32Addr) != XM_SUCCESS) {
    return XM_FAILURE;
  }

  ExtCfg_Read_V2(1, u32Addr, &u8Size);
  if (u8Size < 17)
    u8Size = 17;
  u32Addr++;
  if (u8Fps == 30)
    u32Addr += 8;
  u32Data = 0;
  ExtCfg_Read_V2(2, u32Addr + ISPCUT_OFST_X_PAL, (XM_U8 *)&u32Data);
  pstRect->s32X = u32Data;
  u32Data = 0;
  ExtCfg_Read_V2(2, u32Addr + ISPCUT_OFST_Y_PAL, (XM_U8 *)&u32Data);
  pstRect->s32Y = u32Data;

  u32Data = 0;
  ExtCfg_Read_V2(2, u32Addr + ISPCUT_OFST_WIDHT, (XM_U8 *)&u32Data);
  pstRect->u32Width = u32Data;

  u32Data = 0;
  ExtCfg_Read_V2(2, u32Addr + ISPCUT_OFST_HEIGHT, (XM_U8 *)&u32Data);
  pstRect->u32Height = u32Data;

#ifdef DEBUG_ON
  DEBUG("ExtCft_GetIspWndRect start!\n");
  PrintInt(8, pstRect->s32X);
  PrintInt(8, pstRect->s32Y);
  PrintInt(8, pstRect->u32Width);
  PrintInt(8, pstRect->u32Height);
  ENTER();
  DEBUG("ExtCft_GetIspWndRect over ~~~~\n");
#endif
  return XM_SUCCESS;
}

// PAL/NTSC  ISP Set
// u8Encode: 0:AHD 1:CVI 2:TVI
// u8VstdMode: 0 PAL			1:NTSC
XM_S32 ExtCfg_PN_IspSet(XM_U8 u8Encode, XM_U8 u8VstdMode) {
  XM_U8 u8Mode, u8i, u8Tmp;
  XM_U32 u32Addr, u32Tmp;
  u8VstdMode = CLIP3(u8VstdMode, 0, 1);
  if (ExtCfg_CheckFun(1, ROM_FUN_PN_ISPSET) == XM_SUCCESS) {
#ifdef DEBUG_ON
    DEBUG("ExtCfg_PN_IspSet start\n");
#endif
    ExtCfg_BaseAddrGet(0, ROM_FUN_PN_ISPSET, &u32Addr);
    ExtCfg_Read_V2(1, u32Addr, &u8Mode);
    u32Tmp = 0;
    ExtCfg_Read_V2(1, u32Addr + 1, (XM_U8 *)&u32Tmp);

    u32Addr += 2;
    u8Tmp = u8Encode * 2 + u8VstdMode;
    for (u8i = 0; u8i < 6; u8i++) {
      if (u8Tmp == u8i) {
        break;
      }
      if ((1 << u8i) & u8Mode)
        u32Addr += u32Tmp * 8;
    }
    ExtCft_WriteIsp(0, u32Addr, (XM_U16)(u32Tmp & 0xFF));
#ifdef DEBUG_ON
    DEBUG("ExtCfg_PN_IspSet over ~~~~~~\n");
#endif
    return XM_SUCCESS;
  }
  return XM_FAILURE;
}

// 白平衡矫正参数设置
XM_S32 ExtCfg_AwbCal_Set(XM_VOID) {
  XM_U32 u32Addr;
  ISP_AWB_CALIBRATION_V2_S stAwbCal;
  XM_U8 u8Num;
  if (ExtCfg_BaseAddrGet(0, ROM_FUN_AWB, &u32Addr) != XM_SUCCESS) {
    return XM_FAILURE;
  }

  ExtCfg_Read_V2(1, u32Addr, &u8Num);
  if (u8Num != sizeof(ISP_AWB_CALIBRATION_V2_S)) {
    ERR("ExtCfg_AwbCal_Set num err!\n");
    return XM_FAILURE;
  }
  u32Addr += 1;
  ExtCfg_Read_V2(u8Num, u32Addr, (XM_U8 *)&stAwbCal);

#if (defined SOC_SYSTEM) || (defined SOC_ALIOS)
  XM_U16 u16Tmp;
  ExtCfg_BaseAddrGet(0, ROM_FUN_AEAWBINFO, &u32Addr);
  ExtCfg_Read_V2(2, u32Addr + 37, (XM_U8 *)&u16Tmp);
  stAwbCal.init_gain[0] = u16Tmp << 2;
  ExtCfg_Read_V2(2, u32Addr + 39, (XM_U8 *)&u16Tmp);
  stAwbCal.init_gain[1] = u16Tmp << 2;
  ExtCfg_Read_V2(2, u32Addr + 41, (XM_U8 *)&u16Tmp);
  stAwbCal.init_gain[2] = u16Tmp << 2;
  ExtCfg_Read_V2(2, u32Addr + 43, (XM_U8 *)&u16Tmp);
  stAwbCal.init_gain[3] = u16Tmp;
#endif

  if (XM_MPI_ISP_SetWBCalAttr(&stAwbCal) != XM_SUCCESS) {
    ERR("XM_MPI_ISP_SetWBCalAttr failed!\n");
    return XM_FAILURE;
  }
#ifdef DEBUG_ON
  DEBUG("ExtCfg_AwbCal_Set:\n");
  XM_U8 u8i;
  DBG("A: ");
  for (u8i = 0; u8i < 8; u8i++) {
    PrintInt(8, (XM_S32)stAwbCal.A[u8i]);
  }
  DBG("\r\nB: ");
  for (u8i = 0; u8i < 8; u8i++) {
    PrintInt(8, (XM_S32)stAwbCal.B[u8i]);
  }
  DBG("\r\nC: ");
  for (u8i = 0; u8i < 8; u8i++) {
    PrintInt(8, (XM_S32)stAwbCal.C[u8i]);
  }
  DBG("\r\nKey: ");
  PrintHex(8, (XM_S32)stAwbCal.key);
  DBG("\r\nInit: ");
  PrintInt(8, (XM_S32)stAwbCal.ini_x);
  DBG("    ");
  PrintInt(8, (XM_S32)stAwbCal.ini_y);
  DBG("\r\nDis1: ");
  PrintInt(8, (XM_S32)stAwbCal.dis_min);
  DBG("    ");
  PrintInt(8, (XM_S32)stAwbCal.dis_max);
  DBG("\r\nDis2: ");
  for (u8i = 0; u8i < 16; u8i++) {
    PrintInt(8, (XM_S32)stAwbCal.dis[u8i]);
  }
  DBG("\r\nVal: ");
  for (u8i = 0; u8i < 16; u8i++) {
    PrintInt(8, (XM_S32)stAwbCal.val[u8i]);
  }
  DBG("\r\nInitGain: ");
  for (u8i = 0; u8i < 4; u8i++) {
    PrintInt(8, (XM_S32)stAwbCal.init_gain[u8i]);
  }
  ENTER();
#endif
  return XM_SUCCESS;
}

// CCM参数设置
XM_S32 ExtCfg_CCM_Set(ISP_COLORMATRIX_AUTO_S *pstCCM) {
  XM_U32 u32Addr;
  XM_U8 u8Num;
  if (pstCCM == NULL)
    return XM_FAILURE;
  if (ExtCfg_BaseAddrGet(0, ROM_FUN_CCM, &u32Addr) != XM_SUCCESS) {
    return XM_FAILURE;
  }

  ExtCfg_Read_V2(1, u32Addr, &u8Num);
  if (u8Num != sizeof(ISP_COLORMATRIX_AUTO_S)) {
    ERR("ExtCfg_CCM_Set num err!\n");
    return XM_FAILURE;
  }
  u32Addr += 1;
  ExtCfg_Read_V2(u8Num, u32Addr, (XM_U8 *)pstCCM);

#ifdef DEBUG_ON
  DEBUG("ExtCfg_CCM_Set start:\n");
  XM_U8 u8i;
  PrintInt(8, (XM_S32)pstCCM->u16HighColorTemp);
  for (u8i = 0; u8i < 12; u8i++) {
    if (u8i % 4 == 0)
      ENTER();
    PrintInt(8, (XM_S32)pstCCM->au16HighCCM[u8i]);
  }
  ENTER();
  PrintInt(8, (XM_S32)pstCCM->u16MidColorTemp);
  for (u8i = 0; u8i < 12; u8i++) {
    if (u8i % 4 == 0)
      ENTER();
    PrintInt(8, (XM_S32)pstCCM->au16MidCCM[u8i]);
  }
  ENTER();
  PrintInt(8, (XM_S32)pstCCM->u16LowColorTemp);
  for (u8i = 0; u8i < 12; u8i++) {
    if (u8i % 4 == 0)
      ENTER();
    PrintInt(8, (XM_S32)pstCCM->au16LowCCM[u8i]);
  }
  ENTER();
#endif

  return XM_SUCCESS;
}

/************************************************************************
函数功能:	获取GammaTable
输入参数:	u8GmId:  0
输出参数:	*pu16Tab:
                                        Gamma表
返回参数:	0: 正常
                                -1: 出错
************************************************************************/
XM_S32 ExtCfg_GammaTab_Get(XM_U8 u8GmId, XM_U16 *pu16Tab) {
  static XM_U8 su8Num = 0xFF;
  static XM_U8 su8Size;
  static XM_U32 u32Addr;
  if (ExtCfg_BaseAddrGet(0, ROM_FUN_GAMMATAB, &u32Addr) != XM_SUCCESS) {
    return XM_FAILURE;
  }

  ExtCfg_Read_V2(1, u32Addr, &su8Num);
  ExtCfg_Read_V2(1, u32Addr + 1, &su8Size);
  u32Addr += 2;

  ExtCfg_Read_V2(2 * su8Size, u32Addr + (XM_U32)u8GmId * 2 * su8Size,
                 (XM_U8 *)pu16Tab);
  return XM_SUCCESS;
}

/*********************************************************
函数功能:	验证当前模式是否有外部配置
输入参数:	*pu8Now:	当前模式
                                u8Support:	配置文件支持的模式
返回参数:	<0	: 	无需配置参数
                                >=0	: 	需要配置参数
                                        值为偏移参数结构体个数
a. 优先对应各自模式
b. 没用对应模式时，使用AHD

bit0:AHD
bit1:CVI
bit2:TVI

*********************************************************/
static XM_S32 getChoice(XM_U8 *pu8Now, XM_U8 u8Support) {
  XM_U8 u8Num;
  u8Num = 0;
  switch (*pu8Now) {
  // CVI
  case 1:
    if (u8Support & (1 << 1))
      u8Num = 1;
    break;
  // TVI
  case 2:
    if (u8Support & (1 << 2)) {
      if (u8Support & (1 << 1))
        u8Num = 2;
      else
        u8Num = 1;
    }
    break;
  // AHD
  case 0:
  default:
    u8Num = 0;
    break;
  }
  return (XM_S32)u8Num;
}

/*********************************************************
函数功能:	配置锐化
输入参数:	u8Mode:
                                        bit0: AHD
                                        bit1: CVI
                                        bit2: TVI
*********************************************************/
XM_S32 ExtCfg_Sharp_Set(XM_U8 u8Mode) {
  XM_U32 u32Addr;
  XM_S32 s32Ret;
  XM_U8 u8i, u8Size, u8CfgMode;
  ISP_SHARPEN_ATTR_S stSharpenAttr;
  if (ExtCfg_BaseAddrGet(0, ROM_FUN_SHARP, &u32Addr) == XM_SUCCESS) {
    ExtCfg_Read_V2(1, u32Addr, &u8Size);
    ExtCfg_Read_V2(1, u32Addr + 1, &u8CfgMode);
    u32Addr = u32Addr + 2;
    s32Ret = getChoice(&u8Mode, u8CfgMode);
    if (s32Ret >= 0) // 有外部配置
    {
      u32Addr += (XM_U32)u8Size * s32Ret;
      if (XM_MPI_ISP_GetSharpenAttr(&stSharpenAttr) != XM_SUCCESS) {
        ERR("XM_MPI_ISP_GetSharpenAttr failed!\n");
      }
      ExtCfg_Read_V2(8, u32Addr, (XM_U8 *)(stSharpenAttr.stAuto.au8SharpenD));
      ExtCfg_Read_V2(8, u32Addr + 8,
                     (XM_U8 *)(stSharpenAttr.stAuto.au8SharpenUd));
      ExtCfg_Read_V2(8, u32Addr + 16,
                     (XM_U8 *)(stSharpenAttr.stAuto.au8SharpenKd));
      for (u8i = 8; u8i < 16; u8i++) {
        stSharpenAttr.stAuto.au8SharpenD[u8i] =
            stSharpenAttr.stAuto.au8SharpenD[7];
        stSharpenAttr.stAuto.au8SharpenUd[u8i] =
            stSharpenAttr.stAuto.au8SharpenUd[7];
        stSharpenAttr.stAuto.au8SharpenKd[u8i] =
            stSharpenAttr.stAuto.au8SharpenKd[7];
      }
      if (XM_MPI_ISP_SetSharpenAttr(&stSharpenAttr) != XM_SUCCESS) {
        ERR("XM_MPI_ISP_SetSharpenAttr failed!\n");
        return XM_FAILURE;
      }
    }
    return XM_SUCCESS;
  }
  return XM_FAILURE;
}

/*********************************************************
函数功能:	2DNr
输入参数:	u8Mode:
                                        bit0: AHD
                                        bit1: CVI
                                        bit2: TVI
*********************************************************/
XM_S32 ExtCfg_2DNr_Set(XM_U8 u8Mode) {
  XM_U32 u32Addr;
  XM_S32 s32Ret;
  ISP_2DNR_ATTR_S stNRAttr;
  XM_U8 u8i, u8Size, u8CfgMode;
  if (ExtCfg_BaseAddrGet(0, ROM_FUN_2DNR, &u32Addr) == XM_SUCCESS) {
    ExtCfg_Read_V2(1, u32Addr, &u8Size);
    ExtCfg_Read_V2(1, u32Addr + 1, &u8CfgMode);
    u32Addr = u32Addr + 2;
    s32Ret = getChoice(&u8Mode, u8CfgMode);
    if (s32Ret >= 0) // 有外部配置
    {
      u32Addr += (XM_U32)u8Size * s32Ret;
      if (XM_MPI_ISP_GetNRAttr(&stNRAttr) != XM_SUCCESS) {
        ERR("XM_MPI_ISP_GetNRAttr failed!\n");
      }
      ExtCfg_Read_V2(8, u32Addr, (XM_U8 *)(stNRAttr.stAuto.au8Thresh));
      for (u8i = 8; u8i < 16; u8i++) {
        stNRAttr.stAuto.au8Thresh[u8i] = stNRAttr.stAuto.au8Thresh[7];
      }
      if (XM_MPI_ISP_SetNRAttr(&stNRAttr) != XM_SUCCESS) {
        ERR("XM_MPI_ISP_SetNRAttr failed!\n");
        return XM_FAILURE;
      }
    }
    return XM_SUCCESS;
  }
  return XM_FAILURE;
}

/*********************************************************
函数功能:	3DNr
输入参数:	u8Mode:
                                        bit0: AHD
                                        bit1: CVI
                                        bit2: TVI
*********************************************************/
XM_S32 ExtCfg_3DNr_Set(XM_U8 u8Mode) {
  XM_U32 u32Addr;
  XM_S32 s32Ret;
  ISP_3DNR_ATTR_S stNRAttr;
  XM_U8 u8i, u8Size, u8CfgMode;
  if (ExtCfg_BaseAddrGet(0, ROM_FUN_3DNR, &u32Addr) == XM_SUCCESS) {
    ExtCfg_Read_V2(1, u32Addr, &u8Size);
    ExtCfg_Read_V2(1, u32Addr + 1, &u8CfgMode);
    u32Addr = u32Addr + 2;
    s32Ret = getChoice(&u8Mode, u8CfgMode);
    if (s32Ret >= 0) // 有外部配置
    {
      u32Addr += (XM_U32)u8Size * s32Ret;
      if (XM_MPI_ISP_Get3DNrAttr(&stNRAttr) != XM_SUCCESS) {
        ERR("XM_MPI_ISP_Get3DNrAttr failed!\n");
      }
      ExtCfg_Read_V2(8, u32Addr, (XM_U8 *)(stNRAttr.stAuto.au8TfStrength));
      ExtCfg_Read_V2(8, u32Addr + 8, (XM_U8 *)(stNRAttr.stAuto.au8SfStrength));
      for (u8i = 8; u8i < 16; u8i++) {
        stNRAttr.stAuto.au8TfStrength[u8i] = stNRAttr.stAuto.au8TfStrength[7];
        stNRAttr.stAuto.au8SfStrength[u8i] = stNRAttr.stAuto.au8SfStrength[7];
      }
      if (XM_MPI_ISP_Set3DNrAttr(&stNRAttr) != XM_SUCCESS) {
        ERR("XM_MPI_ISP_Set3DNrAttr failed!\n");
        return XM_FAILURE;
      }
    }
    return XM_SUCCESS;
  }
  return XM_FAILURE;
}

/*********************************************************
函数功能:	DyDpc
输入参数:	u8Mode:
                                        bit0: AHD
                                        bit1: CVI
                                        bit2: TVI
*********************************************************/
XM_S32 ExtCfg_DyDPC_Set(XM_U8 u8Mode) {
  XM_U32 u32Addr;
  XM_S32 s32Ret;
  ISP_DYDPC_ATTR_S stDyDPAttr;
  XM_U8 u8i, u8Size, u8CfgMode;
  if (ExtCfg_BaseAddrGet(0, ROM_FUN_DYDPC, &u32Addr) == XM_SUCCESS) {
    ExtCfg_Read_V2(1, u32Addr, &u8Size);
    ExtCfg_Read_V2(1, u32Addr + 1, &u8CfgMode);
    u32Addr = u32Addr + 2;
    s32Ret = getChoice(&u8Mode, u8CfgMode);
    if (s32Ret >= 0) // 有外部配置
    {
      u32Addr += (XM_U32)u8Size * s32Ret;

      if (XM_MPI_ISP_GetDyDefectPixelAttr(&stDyDPAttr) != XM_SUCCESS) {
        ERR("XM_MPI_ISP_GetDyDefectPixelAttr failed!\n");
      }
      ExtCfg_Read_V2(8, u32Addr, (XM_U8 *)(stDyDPAttr.stAuto.au8Sth));
      for (u8i = 8; u8i < 16; u8i++) {
        stDyDPAttr.stAuto.au8Sth[u8i] = stDyDPAttr.stAuto.au8Sth[7];
      }
      if (XM_MPI_ISP_SetDyDefectPixelAttr(&stDyDPAttr) != XM_SUCCESS) {
        ERR("XM_MPI_ISP_SetDyDefectPixelAttr failed!\n");
        return XM_FAILURE;
      }
    }
    return XM_SUCCESS;
  }
  return XM_FAILURE;
}

/*********************************************************
函数功能:	BLC
输入参数:	u8Mode:
                                        bit0: AHD
                                        bit1: CVI
                                        bit2: TVI
*********************************************************/
XM_S32 ExtCfg_BLC_Set(XM_U8 u8Mode) {
  XM_U32 u32Addr;
  XM_S32 s32Ret;
  ISP_BLACKLVL_ATTR_S stBlackLevel;
  XM_U8 u8i, u8Size, u8CfgMode;
  if (ExtCfg_BaseAddrGet(0, ROM_FUN_BLC, &u32Addr) == XM_SUCCESS) {
    ExtCfg_Read_V2(1, u32Addr, &u8Size);
    ExtCfg_Read_V2(1, u32Addr + 1, &u8CfgMode);
    u32Addr = u32Addr + 2;
    s32Ret = getChoice(&u8Mode, u8CfgMode);
    if (s32Ret >= 0) // 有外部配置
    {
      u32Addr += (XM_U32)u8Size * s32Ret;

      if (XM_MPI_ISP_GetBlackLevelAttr(&stBlackLevel) != XM_SUCCESS) {
        ERR("XM_MPI_ISP_GetBlackLevelAttr failed!\n");
      }
      ExtCfg_Read_V2(16, u32Addr, (XM_U8 *)(stBlackLevel.stAuto.au16Blc));
      for (u8i = 8; u8i < 16; u8i++) {
        stBlackLevel.stAuto.au16Blc[u8i] = stBlackLevel.stAuto.au16Blc[7];
      }
      if (XM_MPI_ISP_SetBlackLevelAttr(&stBlackLevel) != XM_SUCCESS) {
        ERR("XM_MPI_ISP_SetBlackLevelAttr failed!\n");
        return XM_FAILURE;
      }
    }
    return XM_SUCCESS;
  }
  return XM_FAILURE;
}

/*********************************************************
函数功能:	contrast
输入参数:	u8Mode:
                                        bit0: AHD
                                        bit1: CVI
                                        bit2: TVI
*********************************************************/
XM_S32 ExtCfg_Con_Set(XM_U8 u8Mode) {
  XM_U32 u32Addr;
  XM_S32 s32Ret;
  ISP_CSC_ATTR_S stCSCAttr;
  XM_U8 u8i, u8Size, u8CfgMode;
  if (ExtCfg_BaseAddrGet(0, ROM_FUN_CON, &u32Addr) == XM_SUCCESS) {
    ExtCfg_Read_V2(1, u32Addr, &u8Size);
    ExtCfg_Read_V2(1, u32Addr + 1, &u8CfgMode);
    u32Addr = u32Addr + 2;
    s32Ret = getChoice(&u8Mode, u8CfgMode);
    if (s32Ret >= 0) // 有外部配置
    {
      u32Addr += (XM_U32)u8Size * s32Ret;

      if (XM_MPI_ISP_GetCSCAttr(&stCSCAttr) != XM_SUCCESS) {
        ERR("XM_MPI_ISP_GetCSCAttr failed!\n");
      }
      ExtCfg_Read_V2(8, u32Addr, (XM_U8 *)(stCSCAttr.stAuto.au8ContrVal));
      for (u8i = 8; u8i < 16; u8i++) {
        stCSCAttr.stAuto.au8ContrVal[u8i] = stCSCAttr.stAuto.au8ContrVal[7];
      }
      if (XM_MPI_ISP_SetCSCAttr(&stCSCAttr) != XM_SUCCESS) {
        ERR("XM_MPI_ISP_SetCSCAttr failed!\n");
        return XM_FAILURE;
      }
    }
    return XM_SUCCESS;
  }
  return XM_FAILURE;
}

/*********************************************************
函数功能:	saturation
输入参数:	u8Mode:
                                        bit0: AHD
                                        bit1: CVI
                                        bit2: TVI
*********************************************************/
XM_S32 ExtCfg_Sat_Set(XM_U8 u8Mode) {
  XM_U32 u32Addr;
  XM_S32 s32Ret;
  ISP_SATURATION_ATTR_S stSatAttr;
  XM_U8 u8i, u8Size, u8CfgMode, u8SatMode;
  if (ExtCfg_BaseAddrGet(0, ROM_FUN_SAT, &u32Addr) == XM_SUCCESS) {
    ExtCfg_Read_V2(1, u32Addr, &u8Size);
    ExtCfg_Read_V2(1, u32Addr + 1, &u8CfgMode);
    ExtCfg_Read_V2(1, u32Addr + 2, &u8SatMode);
    u32Addr = u32Addr + 3;
    s32Ret = getChoice(&u8Mode, u8CfgMode);
    if (s32Ret >= 0) // 有外部配置
    {
      u32Addr += (XM_U32)u8Size * s32Ret;

      if (XM_MPI_ISP_GetSaturationAttr(&stSatAttr) != XM_SUCCESS) {
        ERR("XM_MPI_ISP_GetSaturationAttr failed!\n");
      }
      ExtCfg_Read_V2(8, u32Addr, (XM_U8 *)(stSatAttr.stAuto.au8Sat));
      for (u8i = 8; u8i < 16; u8i++) {
        stSatAttr.stAuto.au8Sat[u8i] = stSatAttr.stAuto.au8Sat[7];
      }
      stSatAttr.enSatMode = (ISP_SAT_MODE_E)u8SatMode;
      if (XM_MPI_ISP_SetSaturationAttr(&stSatAttr) != XM_SUCCESS) {
        ERR("XM_MPI_ISP_SetSaturationAttr failed!\n");
        return XM_FAILURE;
      }
    }
    return XM_SUCCESS;
  }
  return XM_FAILURE;
}

/*********************************************************
函数功能:	Chroma
输入参数:	u8Mode:
                                        bit0: AHD
                                        bit1: CVI
                                        bit2: TVI
*********************************************************/
XM_S32 ExtCfg_Chroma_Set(XM_U8 u8Mode) {
  XM_U32 u32Addr;
  XM_S32 s32Ret;
  XM_U8 u8Tmp, u8Size, u8CfgMode;
  ISP_CHROMA_ATTR_S stChromaAttr;
  if (ExtCfg_BaseAddrGet(0, ROM_FUN_CHROMA, &u32Addr) == XM_SUCCESS) {
    ExtCfg_Read_V2(1, u32Addr, &u8Size);
    ExtCfg_Read_V2(1, u32Addr + 1, &u8CfgMode);
    u32Addr = u32Addr + 2;
    s32Ret = getChoice(&u8Mode, u8CfgMode);
    if (s32Ret >= 0) // 有外部配置
    {
      u32Addr += (XM_U32)u8Size * s32Ret;

      if (XM_MPI_ISP_GetChromaAttr(&stChromaAttr) != XM_SUCCESS) {
        ERR("XM_MPI_ISP_GetChromaAttr failed!\n");
      }
      ExtCfg_Read_V2(1, u32Addr, &u8Tmp);
      stChromaAttr.bEnable = (XM_BOOL)u8Tmp;
      u32Addr += 1;
      ExtCfg_Read_V2(12, u32Addr, (XM_U8 *)&(stChromaAttr.u16OfstMg));
      ExtCfg_Read_V2(6, u32Addr + 12, (XM_U8 *)&(stChromaAttr.u8SthMg));
      if (XM_MPI_ISP_SetChromaAttr(&stChromaAttr) != XM_SUCCESS) {
        ERR("XM_MPI_ISP_SetChromaAttr failed!\n");
        return XM_FAILURE;
      }
    }
    return XM_SUCCESS;
  }
  return XM_FAILURE;
}

/*********************************************************
函数功能:	Afc
输入参数:	u8Mode:
                                        bit0: AHD
                                        bit1: CVI
                                        bit2: TVI
*********************************************************/
XM_S32 ExtCfg_Afc_Set(XM_U8 u8Mode) {
  XM_U32 u32Addr;
  XM_S32 s32Ret;
  ISP_ANTI_FALSECOLOR_S stAntiFC;
  XM_U8 u8Size, u8CfgMode, u8Tmp;
  if (ExtCfg_BaseAddrGet(0, ROM_FUN_AFC, &u32Addr) == XM_SUCCESS) {
    ExtCfg_Read_V2(1, u32Addr, &u8Size);
    ExtCfg_Read_V2(1, u32Addr + 1, &u8CfgMode);
    u32Addr = u32Addr + 2;
    s32Ret = getChoice(&u8Mode, u8CfgMode);
    if (s32Ret >= 0) // 有外部配置
    {
      u32Addr += (XM_U32)u8Size * s32Ret;
      if (XM_MPI_ISP_GetAntiFalseColorAttr(&stAntiFC) != XM_SUCCESS) {
        ERR("XM_MPI_ISP_GetAntiFalseColorAttr failed!\n");
      }
      ExtCfg_Read_V2(1, u32Addr, &u8Tmp);
      stAntiFC.bEnable = (XM_BOOL)u8Tmp;
      u32Addr += 1;
      ExtCfg_Read_V2(1, u32Addr, &u8Tmp);
      stAntiFC.u8Strength = u8Tmp;
      if (XM_MPI_ISP_SetAntiFalseColorAttr(&stAntiFC) != XM_SUCCESS) {
        ERR("XM_MPI_ISP_SetAntiFalseColorAttr failed!\n");
        return XM_FAILURE;
      }
    }
    return XM_SUCCESS;
  }
  return XM_FAILURE;
}

/************************************************************************
函数功能:	获取lum参数
输入参数:	无
输出参数:	无
返回参数: 	-1: 不支持
                                其他: 亮度值
************************************************************************/
XM_S32 ExtCfg_Lum_Get() {
  XM_U32 u32Addr;
  XM_U8 u8Mode, u8Tmp;
  if (ExtCfg_CheckFun(1, ROM_FUN_LUM) == XM_SUCCESS) {
    ExtCfg_BaseAddrGet(0, ROM_FUN_LUM, &u32Addr);
    ExtCfg_Read_V2(1, u32Addr, &u8Mode);
    u32Addr += 1;
    if (u8Mode & 0x01) {
      ExtCfg_Read_V2(1, u32Addr, &u8Tmp);
      ExtCfg_Read_V2(1, u32Addr + 1, &u8Tmp);
      return (XM_S32)u8Tmp;
    }
  }
  return XM_FAILURE;
}

// 获取Other参数
XM_S32 ExtCfg_Other_Get(OHTER_DATA_S *pstData) {
  XM_U32 u32Addr;
  XM_U8 u8Tmp;
  if (ExtCfg_CheckFun(1, ROM_FUN_OTHER) == XM_SUCCESS) {
    ExtCfg_BaseAddrGet(0, ROM_FUN_OTHER, &u32Addr);
    ExtCfg_Read_V2(1, u32Addr, &(pstData->u8Size));
    u32Addr += 1;
    if (pstData->u8Size > 0) {
      ExtCfg_Read_V2(1, u32Addr, &u8Tmp);
      pstData->u8Gain50 = u8Tmp;
      u32Addr++;
      ExtCfg_Read_V2(1, u32Addr, &u8Tmp);
      pstData->u8Gain100 = u8Tmp;
      u32Addr++;

      ExtCfg_Read_V2(1, u32Addr, &u8Tmp);
      pstData->u8CscLum = u8Tmp;
      u32Addr++;

      ExtCfg_Read_V2(1, u32Addr, &u8Tmp);
      pstData->u8CscSat = u8Tmp;
      u32Addr++;

      ExtCfg_Read_V2(1, u32Addr, &u8Tmp);
      pstData->u8CscContrast = u8Tmp;
      u32Addr++;

      ExtCfg_Read_V2(1, u32Addr, &u8Tmp);
      pstData->u8CscHue = u8Tmp;
      u32Addr++;

      ExtCfg_Read_V2(1, u32Addr, &u8Tmp);
      pstData->u8FlipMirror = u8Tmp;
      u32Addr++;

      ExtCfg_Read_V2(1, u32Addr, &u8Tmp);
      pstData->u8VencVstd = u8Tmp;
      u32Addr++;

      ExtCfg_Read_V2(1, u32Addr, &u8Tmp);
      pstData->u8Rslt = u8Tmp;
      u32Addr++;
      return XM_SUCCESS;
    }
  }
  return XM_FAILURE;
}

// u8Mode: 0: First   1:After Stab
// u8Vstd: 0:PAL  1:NTSC
XM_S32 ExtCfg_AeAwbInit(XM_U8 u8Mode, XM_U8 u8Vstd) {
  static XM_U8 sau8SpeedStab;
  static XM_U8 sau8TolStab;
  static XM_U16 sau16RatioStab;
  static XM_U16 sau16CoefStab;
  static XM_U32 su32UpdateCfgStab;
  ISP_EXPOSURE_ATTR_S stExpAttr;
  ISP_WB_ATTR_S stWBAttr;
  XM_U32 u32Addr;
  AE_INIT_S stAe;
  XM_U8 u8Tmp;
  if (ExtCfg_CheckFun(1, ROM_FUN_AEAWBINIT) != XM_SUCCESS)
    return XM_FAILURE;
  if (XM_MPI_ISP_GetExposureAttr(0, &stExpAttr) != XM_SUCCESS) {
    ERR("XM_MPI_ISP_GetExposureAttr failed!\n");
    return XM_FAILURE;
  }
  if (XM_MPI_ISP_GetWBAttr(&stWBAttr) != XM_SUCCESS) {
    ERR("XM_MPI_ISP_GetWBAttr failed !\n");
    return XM_FAILURE;
  }

  if (u8Mode < 1) {
#ifdef DEBUG_ON
    DEBUG("ExtCfg_AeAwbInit start:\n");
#endif
    ExtCfg_BaseAddrGet(0, ROM_FUN_AEAWBINIT, &u32Addr);
    u32Addr++;
    // AE
    ExtCfg_Read_V2(1, u32Addr, (XM_U8 *)&stAe.u8Speed);
    u32Addr += 1;
    ExtCfg_Read_V2(1, u32Addr, (XM_U8 *)&stAe.u8Tolerance);
    u32Addr += 1;
    ExtCfg_Read_V2(1, u32Addr, (XM_U8 *)&stAe.u8Speed_Stab);
    u32Addr += 1;
    ExtCfg_Read_V2(1, u32Addr, (XM_U8 *)&stAe.u8Tolerance_Stab);
    u32Addr += 1;
    ExtCfg_Read_V2(4, u32Addr, (XM_U8 *)&stAe.u32UpdateCfg_Stab);
    u32Addr += 4;
    ExtCfg_Read_V2(1, u32Addr, (XM_U8 *)&stAe.u8AntiFlicker);
    u32Addr += 1;
    ExtCfg_Read_V2(1, u32Addr, (XM_U8 *)&stAe.u8AntiFlicker_Freq);
    u32Addr += 1;
    ExtCfg_Read_V2(1, u32Addr, (XM_U8 *)&stAe.u8BlackDelayFrame);
    u32Addr += 1;
    ExtCfg_Read_V2(1, u32Addr, (XM_U8 *)&stAe.u8WhiteDelayFrame);
    u32Addr += 1;
    ExtCfg_Read_V2(1, u32Addr, (XM_U8 *)&stAe.u8ExpMode);
    u32Addr += 1;
    ExtCfg_Read_V2(4, u32Addr, (XM_U8 *)&stAe.u32ExpManual);
    u32Addr += 4;
    ExtCfg_Read_V2(4, u32Addr, (XM_U8 *)&stAe.u32MinAGain);
    u32Addr += 4;
    ExtCfg_Read_V2(4, u32Addr, (XM_U8 *)&stAe.u32MaxAGain);
    u32Addr += 4;
    ExtCfg_Read_V2(4, u32Addr, (XM_U8 *)&stAe.u32MinDGain);
    u32Addr += 4;
    ExtCfg_Read_V2(4, u32Addr, (XM_U8 *)&stAe.u32MaxDGain);
    u32Addr += 4;
    ExtCfg_Read_V2(4, u32Addr, (XM_U8 *)&stAe.u32MinIspDGain);
    u32Addr += 4;
    ExtCfg_Read_V2(4, u32Addr, (XM_U8 *)&stAe.u32MaxIspDGain);
    u32Addr += 4;
    stExpAttr.stAuto.u8Speed = stAe.u8Speed;
    stExpAttr.stAuto.u8Tolerance = stAe.u8Tolerance;
    stExpAttr.stAuto.stAntiflicker.u8Frequency = stAe.u8AntiFlicker_Freq;
    stExpAttr.stAuto.stAntiflicker.enMode = stAe.u8AntiFlicker & 0xF;
    stExpAttr.stAuto.stAntiflicker.bEnable = (stAe.u8AntiFlicker >> 7) & 0x01;
    stExpAttr.stAuto.stAEDelayAttr.u16BlackDelayFrame =
        (XM_U16)(stAe.u8BlackDelayFrame);
    stExpAttr.stAuto.stAEDelayAttr.u16WhiteDelayFrame =
        (XM_U16)(stAe.u8WhiteDelayFrame);
    stExpAttr.stManual.bManualExpEnable = stAe.u8ExpMode;
    stExpAttr.stManual.u32Exp = stAe.u32ExpManual;
    stExpAttr.enOpType = (stAe.u8ExpMode) ? OP_TYPE_MANUAL : OP_TYPE_AUTO;
    stExpAttr.stAuto.stAGainRange.u32Min = stAe.u32MinAGain;
    stExpAttr.stAuto.stAGainRange.u32Max = stAe.u32MaxAGain;
    stExpAttr.stAuto.stDGainRange.u32Min = stAe.u32MinDGain;
    stExpAttr.stAuto.stDGainRange.u32Max = stAe.u32MaxDGain;
    stExpAttr.stAuto.stISPDGainRange.u32Min = stAe.u32MinIspDGain;
    stExpAttr.stAuto.stISPDGainRange.u32Max = stAe.u32MaxIspDGain;
    sau8SpeedStab = stAe.u8Speed_Stab;
    sau8TolStab = stAe.u8Tolerance_Stab;
    su32UpdateCfgStab = stAe.u32UpdateCfg_Stab;
    // AWB
    ExtCfg_Read_V2(1, u32Addr, &stWBAttr.stAuto.u8HighColorTemp);
    u32Addr++;
    ExtCfg_Read_V2(1, u32Addr, &stWBAttr.stAuto.u8LowColorTemp);
    u32Addr++;
    ExtCfg_Read_V2(2, u32Addr, (XM_U8 *)&stWBAttr.stAuto.u16SpeedToH);
    u32Addr += 2;
    ExtCfg_Read_V2(2, u32Addr, (XM_U8 *)&stWBAttr.stAuto.u16SpeedToL);
    u32Addr += 2;
    ExtCfg_Read_V2(2, u32Addr, (XM_U8 *)&sau16RatioStab);
    u32Addr += 2;
    ExtCfg_Read_V2(2, u32Addr, (XM_U8 *)&sau16CoefStab);
    u32Addr += 2;
    ExtCfg_Read_V2(1, u32Addr, (XM_U8 *)&u8Tmp);
    u32Addr += 1;
    stWBAttr.enOpType = u8Tmp;
    ExtCfg_Read_V2(2, u32Addr, (XM_U8 *)&(stWBAttr.stManual.u16Rgain));
    u32Addr += 2;
    ExtCfg_Read_V2(2, u32Addr, (XM_U8 *)&(stWBAttr.stManual.u16Ggain));
    u32Addr += 2;
    ExtCfg_Read_V2(2, u32Addr, (XM_U8 *)&(stWBAttr.stManual.u16Bgain));
    u32Addr += 2;
  } else {
    stExpAttr.stAuto.u8Speed = sau8SpeedStab;
    stExpAttr.stAuto.u8Tolerance = sau8TolStab;
    if (su32UpdateCfgStab != 0xFFFFFFFF)
      stExpAttr.stAuto.u32UpdateCfg = su32UpdateCfgStab;
    stWBAttr.stAuto.u16SpeedToH = sau16RatioStab;
    stWBAttr.stAuto.u16SpeedToL = sau16CoefStab;
  }
#if (defined SOC_SYSTEM) || (defined SOC_ALIOS)
  stExpAttr.stAuto.u8Speed = sau8SpeedStab;
  stExpAttr.stAuto.u8Tolerance = sau8TolStab;
  stWBAttr.stAuto.u16SpeedToH = sau16RatioStab;
  stWBAttr.stAuto.u16SpeedToL = sau16CoefStab;
  if (su32UpdateCfgStab != 0xFFFFFFFF)
    stExpAttr.stAuto.u32UpdateCfg = su32UpdateCfgStab;
#endif
  if (XM_MPI_ISP_SetExposureAttr(0, &stExpAttr) != XM_SUCCESS) {
    ERR("XM_MPI_ISP_SetExposureAttr failed!\n");
  }
  if (XM_MPI_ISP_SetWBAttr(&stWBAttr) != XM_SUCCESS) {
    ERR("XM_MPI_ISP_SetWBAttr failed !\n");
  }
  gu16Tolerance = (XM_U16)stExpAttr.stAuto.u8Tolerance;
#ifdef DEBUG_ON
  PrintInt(8, (XM_U32)stExpAttr.stAuto.u8Speed);
  PrintInt(8, (XM_U32)stExpAttr.stAuto.u8Tolerance);
  ENTER();
  PrintInt(8, (XM_U32)sau8SpeedStab);
  PrintInt(8, (XM_U32)sau8TolStab);
  PrintHex(8, (XM_U32)su32UpdateCfgStab);
  ENTER();
  PrintInt(8, (XM_U32)stExpAttr.stAuto.stAntiflicker.bEnable);
  ENTER();
  PrintInt(8, (XM_U32)stExpAttr.stAuto.stAntiflicker.u8Frequency);
  ENTER();
  PrintInt(8, (XM_U32)stExpAttr.stAuto.stAntiflicker.enMode);
  ENTER();
  PrintInt(8, (XM_U32)stExpAttr.stAuto.stAEDelayAttr.u16BlackDelayFrame);
  ENTER();
  PrintInt(8, (XM_U32)stExpAttr.stAuto.stAEDelayAttr.u16WhiteDelayFrame);
  ENTER();
  PrintHex(8, (XM_U32)stExpAttr.stAuto.stAGainRange.u32Min);
  ENTER();
  PrintHex(8, (XM_U32)stExpAttr.stAuto.stAGainRange.u32Max);
  ENTER();
  PrintHex(8, (XM_U32)stExpAttr.stAuto.stDGainRange.u32Min);
  ENTER();
  PrintHex(8, (XM_U32)stExpAttr.stAuto.stDGainRange.u32Max);
  ENTER();
  PrintHex(8, (XM_U32)stExpAttr.stAuto.stISPDGainRange.u32Min);
  ENTER();
  PrintHex(8, (XM_U32)stExpAttr.stAuto.stISPDGainRange.u32Max);
  ENTER();
  ENTER();

  PrintInt(8, (XM_U32)stWBAttr.stAuto.u8HighColorTemp);
  ENTER();
  PrintInt(8, (XM_U32)stWBAttr.stAuto.u8LowColorTemp);
  ENTER();
  PrintInt(8, (XM_U32)stWBAttr.stAuto.u16SpeedToH);
  ENTER();
  PrintInt(8, (XM_U32)stWBAttr.stAuto.u16SpeedToL);
  ENTER();
  PrintInt(8, (XM_U32)sau16RatioStab);
  ENTER();
  PrintInt(8, (XM_U32)sau16CoefStab);
  ENTER();
  DEBUG("ExtCfg_AeAwbInit over ~~~~TTT\n");
#endif
  return XM_SUCCESS;
}

XM_S32 ExtCfg_Nr3DInit(NR3D_INIT_S *pstNr3D) {
  XM_U32 u32Addr;
  XM_U8 u8i;

  if ((pstNr3D == NULL) || ExtCfg_CheckFun(1, ROM_FUN_NR3D) != XM_SUCCESS)
    return XM_FAILURE;

#ifdef DEBUG_ON
  DEBUG("ExtCfg_Nr3DInit start: ~~~~~~~\n");
#endif
  ExtCfg_BaseAddrGet(0, ROM_FUN_NR3D, &u32Addr);

  ExtCfg_Read_V2(1, u32Addr, &(pstNr3D->u8Size));
  ExtCfg_Read_V2(1, u32Addr + 1, &(pstNr3D->u8FrameCnt));
  ExtCfg_Read_V2(1, u32Addr + 2, &(pstNr3D->u8Ctrl));
  ExtCfg_Read_V2(2, u32Addr + 3, (XM_U8 *)&(pstNr3D->u16Width));
  ExtCfg_Read_V2(2, u32Addr + 5, (XM_U8 *)&(pstNr3D->u16Height));
  ExtCfg_Read_V2(2, u32Addr + 7, (XM_U8 *)&(pstNr3D->u16ChangePoint));
  ExtCfg_Read_V2(4, u32Addr + 9, (XM_U8 *)&(pstNr3D->u32PhyAddr));
  u32Addr += 13;
  for (u8i = 0; u8i < 8; u8i++) {
    ExtCfg_Read_V2(4, u32Addr, (XM_U8 *)&(pstNr3D->u32BaseAddr[u8i]));
    u32Addr += 4;
  }
  ExtCfg_Read_V2(4, u32Addr, (XM_U8 *)&(pstNr3D->u32YAddr));
  u32Addr += 4;
  ExtCfg_Read_V2(4, u32Addr, (XM_U8 *)&(pstNr3D->u32ErrAddr));

#ifdef DEBUG_ON
  PrintInt(8, (XM_U32)pstNr3D->u8FrameCnt);
  ENTER();
  PrintHex(8, (XM_U32)pstNr3D->u8Ctrl);
  ENTER();
  PrintInt(8, (XM_U32)pstNr3D->u16Width);
  ENTER();
  PrintInt(8, (XM_U32)pstNr3D->u16Height);
  ENTER();
  PrintInt(8, (XM_U32)pstNr3D->u16ChangePoint);
  ENTER();
  PrintHex(8, (XM_U32)pstNr3D->u32PhyAddr);
  ENTER();
  for (u8i = 0; u8i < 8; u8i++)
    PrintHex(8, (XM_U32)pstNr3D->u32BaseAddr[u8i]);
  ENTER();
  PrintHex(8, (XM_U32)pstNr3D->u32YAddr);
  ENTER();
  PrintHex(8, (XM_U32)pstNr3D->u32ErrAddr);
  ENTER();
  DEBUG("ExtCfg_Nr3DInit over ~~~~TTT\n");
#endif
  return XM_SUCCESS;
}

/***********************************************************************
函数名称:	ExtCfg_VencSet
函数功能:	不同编码、视频支持进行外部配置参数
输入参数:	u8VencMode: 当前制式
                                        bit0: AHD_PAL
                                        bit1: AHD_NTSC
                                        bit2: CVI_PAL
                                        bit3: CVI_NTSC
                                        bit4: TVI_PAL
                                        bit5: TVI_NTSC

                                        0: 仅仅获取是否使用内部配置
输出参数:	无
返回参数:	0:	成功
                                1:    不使用内部配置
                                -1: 	出错
Note:Lycai
***********************************************************************/
XM_S32 ExtCfg_VencSet(XM_U8 u8VencMode) {
  XM_U32 u32Addr;
  XM_U8 u8SuptMode, u8Num, u8i;
  if (ExtCfg_CheckFun(1, ROM_FUN_VENC) != XM_SUCCESS)
    return XM_FAILURE;
#ifdef DEBUG_ON
  DEBUG("ExtCfg_VencSet start:\n");
  PrintInt(3, (XM_U32)u8VencMode);
  DBG("\r\n");
#endif

  ExtCfg_BaseAddrGet(0, ROM_FUN_VENC, &u32Addr);
  ExtCfg_Read_V2(1, u32Addr, &u8SuptMode);
  if (u8VencMode == 0) {
    if (u8SuptMode & (1 << 6)) {
      return 1;
    }
    return 0;
  }

  ExtCfg_Read_V2(1, u32Addr + 1, &u8Num);
  u32Addr += 2;

  for (u8i = 0; u8i < 8; u8i++) {
    if ((1 << u8i) == u8VencMode) {
      break;
    }
    if ((1 << u8i) & u8SuptMode) {
      u32Addr += 8 * u8Num;
    }
  }
  if (u8VencMode & u8SuptMode) {

    ExtCft_WriteIsp(0, u32Addr, (XM_U16)u8Num);

#ifdef DEBUG_ON
    DEBUG("ExtCfg_VencSet over ~~~~\n");
#endif
  }
  return XM_SUCCESS;
}

XM_S32 ExtCfg_StabDeal(STABDEAL_S *pstStabDeal) {
  XM_U32 u32Addr;
  if (pstStabDeal == NULL)
    return XM_FAILURE;
  if (ExtCfg_BaseAddrGet(0, ROM_FUN_STABDEAL, &u32Addr) == XM_SUCCESS) {
#ifdef DEBUG_ON
    DEBUG("ExtCfg_StabDeal start: ~~~~~~~\n");
#endif

    ExtCfg_Read_V2(1, u32Addr, &(pstStabDeal->u8Size));
    u32Addr += 1;
    ExtCfg_Read_V2(1, u32Addr, &(pstStabDeal->u8DealAfterStab));
    u32Addr += 1;
    ExtCfg_Read_V2(2, u32Addr, (XM_U8 *)&(pstStabDeal->u16FmRunNum));
    u32Addr += 2;
    ExtCfg_Read_V2(2, u32Addr, (XM_U8 *)&(pstStabDeal->u16StabFmID));
    u32Addr += 2;
    ExtCfg_Read_V2(1, u32Addr, (XM_U8 *)&(pstStabDeal->u8IspRegNum));
    u32Addr += 1;
    pstStabDeal->u32IspAddrData = u32Addr;

#ifdef DEBUG_ON
    PrintInt(8, (XM_U32)(pstStabDeal->u8Size));
    ENTER();
    PrintHex(8, (XM_U32)(pstStabDeal->u8DealAfterStab));
    ENTER();
    PrintInt(8, (XM_U32)(pstStabDeal->u16FmRunNum));
    ENTER();
    PrintInt(8, (XM_U32)(pstStabDeal->u16StabFmID));
    ENTER();
    PrintInt(8, (XM_U32)(pstStabDeal->u8IspRegNum));
    ENTER();
    PrintHex(8, (XM_U32)(pstStabDeal->u32IspAddrData));
    ENTER();
    DEBUG("ExtCfg_StabDeal over ~~~~TTT\n");
#endif
  }
  return XM_SUCCESS;
}

// 对Isp寄存器进行初始化
// u8Mode:  0: Front		1:Back
XM_S32 ExtCfg_IspRegInit(XM_U8 u8Mode) {
  XM_U32 u32Addr, u32Tmp, u32Tmp2;
  if (ExtCfg_GetExtCfgFlg() == 0) {
    return XM_SUCCESS;
  }

  if (ExtCfg_CheckFun(0, ROM_FUN_ISPINIT_FRONT) != XM_SUCCESS)
    return XM_FAILURE;
  ExtCfg_BaseAddrGet(0, ROM_FUN_ISPINIT_FRONT, &u32Addr);
  // 0. Front
  if (u8Mode == 0) {
#ifdef DEBUG_ON
    DEBUG("ExtCfg_IspRegInit(Front) start\n");
#endif
    u32Tmp = 0;
    ExtCfg_Read_V2(2, u32Addr, (XM_U8 *)&u32Tmp);
    ExtCft_WriteIsp(0, u32Addr + 4, (XM_U16)(u32Tmp & 0xFFFF));
#ifdef DEBUG_ON
    DEBUG("ExtCfg_IspRegInit(Front) over ~~~~~\n");
#endif
    return XM_SUCCESS;
  }

  // 1. back
  if (u8Mode == 1) {
#ifdef DEBUG_ON
    DEBUG("ExtCfg_IspRegInit(Back) start\n");
#endif
    u32Tmp = 0;
    ExtCfg_Read_V2(2, u32Addr, (XM_U8 *)&u32Tmp);
    ExtCfg_Read_V2(2, u32Addr + 2, (XM_U8 *)&u32Tmp2);
    u32Addr = u32Addr + 4 + u32Tmp * 8;
    ExtCft_WriteIsp(0, u32Addr, (XM_U16)(u32Tmp2 & 0xFFFF));
#ifdef DEBUG_ON
    DEBUG("ExtCfg_IspRegInit(Back) over ~~~~~\n");
#endif
    return XM_SUCCESS;
  }
  return XM_FAILURE;
}

XM_S32 ExtCfg_AgcMode_Set(XM_U8 u8Venc) {
  ExtCfg_Sharp_Set(u8Venc);
  ExtCfg_2DNr_Set(u8Venc);
  ExtCfg_3DNr_Set(u8Venc);
  ExtCfg_DyDPC_Set(u8Venc);
  ExtCfg_Sat_Set(u8Venc);
  ExtCfg_BLC_Set(u8Venc);
  ExtCfg_Con_Set(u8Venc);
  ExtCfg_Chroma_Set(u8Venc);
  ExtCfg_Afc_Set(u8Venc);
  return XM_SUCCESS;
}

static void cscMirrorFlip_set() {
  OHTER_DATA_S stOtherData;
  ISP_CSC_ATTR_S stCSCAttr;
  ISP_CHN_ATTR_S stChnAttr;
  if (ExtCfg_Other_Get(&stOtherData) == XM_SUCCESS) {
    if (XM_MPI_ISP_GetCSCAttr(&stCSCAttr) == XM_SUCCESS) {
      stCSCAttr.stManual.u8LumaVal = stOtherData.u8CscLum;
      stCSCAttr.stManual.u8ContrVal = stOtherData.u8CscContrast;
      stCSCAttr.stManual.u8SatuVal = stOtherData.u8CscSat;
      stCSCAttr.stManual.u8HueVal = stOtherData.u8CscHue;
      XM_MPI_ISP_SetCSCAttr(&stCSCAttr);
    }
    if (XM_MPI_ISP_GetChnAttr(&stChnAttr) == XM_SUCCESS) {
      stChnAttr.bMirror = stOtherData.u8FlipMirror & 0x0F;
      stChnAttr.bFlip = (stOtherData.u8FlipMirror >> 4) & 0x0F;
      XM_MPI_ISP_SetChnAttr(&stChnAttr);
    }

    // Max SysGain
    ISP_EXPOSURE_ATTR_S stExpAttr;
    if (XM_MPI_ISP_GetExposureAttr(0, &stExpAttr) == XM_SUCCESS) {
      stExpAttr.stAuto.stSysGainRange.u32Max = 1024 * stOtherData.u8Gain50;
      XM_MPI_ISP_SetExposureAttr(0, &stExpAttr);
    }
  }
}

// u8Venc:  	0:AHD  1:CVI  2:TVI
// u8Std:  	0:PAL   1:NTSC
XM_S32 ExtCfg_IspDataInit(XM_U8 u8Venc, XM_U8 u8Std) {
  ExtCfg_AwbCal_Set();
  ExtCfg_AeAwbInit(0, u8Std);
  ExtCfg_AgcMode_Set(u8Venc);
  ExtCfg_IspRegInit(1);
  cscMirrorFlip_set();
  return XM_SUCCESS;
}

XM_S32 ExtCfg_Init(XM_U8 *pu8Buffer) {
  XM_U8 u8i;
  XM_U32 u32Tmp;
  gu8ExtCfgFlg = 2;
  pgu8Buffer = pu8Buffer;

#if (defined SOC_ALIOS)
  return XM_SUCCESS;
#endif

  // MARK
  ExtCfg_Read_V2(4, 0, gstExtCfgData.au8MarkID);
  u32Tmp = (gstExtCfgData.au8MarkID[0] << 24) |
           (gstExtCfgData.au8MarkID[1] << 16) |
           (gstExtCfgData.au8MarkID[2] << 8) | gstExtCfgData.au8MarkID[3];
  if ((u32Tmp & 0xFFFFFF00) != (MARK_ID & 0xFFFFFF00)) {
#if (defined SOC_SYSTEM) || (defined SOC_ALIOS)
    return XM_SUCCESS;
#else
    ERR("MarkID err!\n");
    return XM_FAILURE;
#endif
  }
  DEBUG("MarkID Right!\n");
#ifdef DEBUG_ON
  DEBUG("Mark:\n");
  PrintHex(8, u32Tmp);
  ENTER();
#endif
  gu8ExtCfgFlg = 1;
#ifdef SOC_NONE
  u32Tmp = MARK_ID_TO_SYS;
  gstExtCfgData.au8MarkID[0] = (u32Tmp >> 24) & 0xFF;
  gstExtCfgData.au8MarkID[1] = (u32Tmp >> 16) & 0xFF;
  gstExtCfgData.au8MarkID[2] = (u32Tmp >> 8) & 0xFF;
  gstExtCfgData.au8MarkID[3] = (u32Tmp >> 0) & 0xFF;
  ExtCfg_Write_V2(4, 0, gstExtCfgData.au8MarkID);
#else
  if (u32Tmp == MARK_ID_TO_SYS) {
    gu8ExtCfgFlg = 0;
  }
#endif
  // FunMask
  ExtCfg_Read_V2(1, 4, &(gstExtCfgData.u8FunMask_H));
  ExtCfg_Read_V2(4, 5, (XM_U8 *)&(gstExtCfgData.u32FunMask_L));

  u32Tmp = 9;
  for (u8i = 0; u8i < FUN_NUM; u8i++) {
    ExtCfg_Read_V2(4, u32Tmp,
                   (XM_U8 *)&(gstExtCfgData.astExtCfgBase[u8i].u32Addr));
    u32Tmp += 4;
  }
#ifdef DEBUG_ON
  DEBUG("Mask&BaseAddr:\n");
  PrintHex(2, gstExtCfgData.u8FunMask_H);
  PrintHex(8, gstExtCfgData.u32FunMask_L);
  ENTER();
  for (u8i = 0; u8i < FUN_NUM; u8i++) {
    PrintInt(8, (XM_S32)u8i);
    PrintInt(8, gstExtCfgData.astExtCfgBase[u8i].u32Addr);
    ENTER();
  }
#endif
  ExtCfg_IspRegInit(0);
  return XM_SUCCESS;
}
