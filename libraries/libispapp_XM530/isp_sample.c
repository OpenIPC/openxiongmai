#ifdef SOC_SYSTEM
#include <fcntl.h>
#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/types.h>

#include <string.h>
#include <sys/mman.h>
#include <sys/time.h>
#include <time.h>

#include "mpi_sys.h"
#include "mpi_vi.h"
#elif (defined SOC_ALIOS)
#include <fcntl.h>
#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>

#include "mpi_sys.h"
#include "mpi_vi.h"
#endif
#include "mpi_ae.h"
#include "mpi_awb.h"
#include "mpi_isp.h"
#include "mpi_venctx.h"
#include "xm_math.h"
#include "xm_print.h"
#include "xm_sns_ctrl.h"

#include "xm530_isp.h"
#include "xm530_tmp.h"

#include "Camera.h"
#include "extCfg.h"
#include "i2c.h"
#include "ispMsg.h"
#include "mpi_phyvi.h"
#include "mpi_vo.h"
XM_U8 gu8PclkEdge = 1; // 0:	Rising edge	1:Falling edge

ISP_DEV gIspDevApp = 0;
extern XM_U8 gu8LoopIsp;
XM_U8 gu8IspOk = 0;
XM_U8 Mipi_Check = 0;

int Create_Isplock(void);

#if (defined DEVTYPE_IPC) && ((!defined SOC_XMSDK) && (!defined SOC_ALIOS))
pthread_t gIspDebug = NULL;
pthread_t gIspTool = NULL;
#endif

#if (defined SOC_SYSTEM) || (defined SOC_ALIOS)
static pthread_t gIspDebugCom = NULL;
static pthread_t gIspPid = NULL;
#endif
extern int Mipi_check(void);
#ifdef SOC_SYSTEM
extern XM_S32 SysRegExit();
extern XM_S32 Write_IspReg(XM_U32 u32Addr, XM_U32 u32Value);
extern XM_S32 Read_IspReg(XM_U32 u32Addr);
extern void *ispMsgReceive(void *arg);
extern void *toolMsgReceive(void *arg);
#else
#define Write_IspReg(u32Addr, u32Value)                                        \
  (*((volatile unsigned long *)(u32Addr)) = (u32Value))
#define Read_IspReg(u32Addr) (*((volatile unsigned long *)(u32Addr)))
#endif
XM_PRODUCT_INFO gstProductInfo;
extern XM_U16 gu16Tolerance;
extern XM_U8 gu8DebugCom_Loop;
extern void *Com_process(void *arg);
extern XM_U8 xmprop_get_value_v2(XM_U8 *KeyName, XM_VOID *pu32Addr);
extern XM_S32 xmprop_set_value_v2(XM_U8 *KeyName, XM_VOID *pu32Addr);

//#define TWOCHANNEL	(1)
extern XM_S32 vda_sample();
/*************************************************************************
函数功能:	使用说明
输出参数:
                -m	 0/1	注:(0:VI Sofia中配置   	1:VI ISP中配置)
                -r   1920 1080
                -v   p/n  	注:(p: PAL  n:NTSC)

note:
        u16ValH/u16ValV = 0xFFFF 时标准不写入
*************************************************************************/
typedef struct xm_PARAM_IN {
  XM_U8 u8Mode;         // 0: VI/SYS Sofia中配置    1:VI/SYS ISP中配置
  XM_U8 u8Vstd;         // 0:UN  1: PAL  2:NTSC
  PROFILE enRslt;       // 0:720P 1:1080P ...
  XM_U8 u8IpcVenc;      // 1:H.265 	other:H.264
  XM_U8 u8SnsInterface; // 0:by src  1:DVP  2:MIPI
  XM_U8 au8Rsv[8];
} PARAM_IN_S;
static PARAM_IN_S gstParamIn;

extern XM_S32 SysRegExit();
extern XM_U8 *Get_CfgBufferAddr();
#if 0
int raise(int signum)
{
	return 0;
}
#endif

#if (defined SOC_SYSTEM) && ((!defined SOC_XMSDK) && (!defined SOC_ALIOS))
extern int readFile(const char *_fileName, void *_buf, int _bufLen);
static void sig_kill_handler(int sign) {
  static XM_S32 ss32sign = 0xFFFFFFF;
  if (ss32sign == sign)
    return;
  ss32sign = sign;
  if (sign == SIGINT) // Ctrl+C
  {
    sensor_restart(0);
    DEBUG("IspSample Catch a signal [Ctlr + C] !\n");
    gu8DebugCom_Loop = 0;
    XM_MPI_ISP_Exit(gIspDevApp);
    if (gIspPid)
      pthread_join(gIspPid, NULL);
    if (gIspDebugCom)
      pthread_join(gIspDebugCom, NULL);
    gu8LoopIsp = 0;
  } else if (sign == SIGTERM) // Kill
  {
    sensor_restart(0);
    DEBUG("IspSample Catch a signal [kill]!!!\n");
    gu8DebugCom_Loop = 0;
    XM_MPI_ISP_Exit(gIspDevApp);
    if (gIspPid)
      pthread_join(gIspPid, NULL);
    if (gIspDebugCom)
      pthread_join(gIspDebugCom, NULL);
    gu8LoopIsp = 0;
  } else {
    ERR("Catch a signal UNSUPPORT!!!\n");
  }
  return;
}

static void ispSignalInit() {

  // 对SIGIO 信号屏蔽
  sigset_t set;
  sigemptyset(&set);
  sigaddset(&set, SIGIO);
  pthread_sigmask(SIG_BLOCK, &set, NULL);

  signal(SIGINT, sig_kill_handler);
  signal(SIGTERM, sig_kill_handler);
}
#endif

XM_S32 SysMemnCpy(XM_U8 *pDst, XM_U8 *pSrc, XM_U32 u32Num) {
  if ((pSrc == NULL) || (pDst == NULL))
    return XM_FAILURE;
  for (; u32Num > 0; u32Num--)
    *(pDst++) = *(pSrc++);
  return XM_SUCCESS;
}

XM_S32 SysGetProductInfo(XM_PRODUCT_INFO *pstProductInfo) {
  XM_S32 s32Ret;
  if (pstProductInfo == NULL) {
    ERR("SysGetProductInfo input err!\n");
    return XM_FAILURE;
  }
  s32Ret = XM_MPI_ISP_Memncpy((XM_U8 *)pstProductInfo, (XM_U8 *)&gstProductInfo,
                              sizeof(XM_PRODUCT_INFO));
  if (s32Ret != XM_SUCCESS) {
    ERR("SysMemnCpy failed!\n");
    return XM_FAILURE;
  }
  return XM_SUCCESS;
}

XM_S32 SysSetProductInfo(XM_PRODUCT_INFO *pstProductInfo) {
  XM_S32 s32Ret;
  if (pstProductInfo == NULL) {
    ERR("SysGetProductInfo input err!\n");
    return XM_FAILURE;
  }
  s32Ret = XM_MPI_ISP_Memncpy((XM_U8 *)&gstProductInfo, (XM_U8 *)pstProductInfo,
                              sizeof(XM_PRODUCT_INFO));
  if (s32Ret != XM_SUCCESS) {
    ERR("XM_MPI_ISP_Memncpy failed!\n");
    return XM_FAILURE;
  }
  return XM_SUCCESS;
}

void GPIO_Init() {
  XM_MPI_ISP_SetRegister(GPIO_POC_18_SEL, 0x02); // 	1.8	IO

  XM_MPI_ISP_SetRegister(GPIO_I2C0_SDA, 0x2);
  XM_MPI_ISP_SetRegister(GPIO_I2C0_SCL, 0x2);

  XM_MPI_ISP_SetRegister(GPIO_SEN0_MCLK, 0x82); // Sns0_Mclk 8MA
  XM_MPI_ISP_SetRegister(GPIO_SEN0_VSYNC, 0x2);
  XM_MPI_ISP_SetRegister(GPIO_SEN0_PCLK, 0x2);
  XM_MPI_ISP_SetRegister(GPIO_SEN0_HSYNC, 0x2);
#if 0
  XM_MPI_ISP_SetRegister(GPIO_SEN0_DATE0, 0x2);
  XM_MPI_ISP_SetRegister(GPIO_SEN0_DATE1, 0x2);
  XM_MPI_ISP_SetRegister(GPIO_SEN0_DATE2, 0x2);
  XM_MPI_ISP_SetRegister(GPIO_SEN0_DATE3, 0x2);
  XM_MPI_ISP_SetRegister(GPIO_SEN0_DATE4, 0x2);
  XM_MPI_ISP_SetRegister(GPIO_SEN0_DATE5, 0x2);
  XM_MPI_ISP_SetRegister(GPIO_SEN0_DATE6, 0x2);
  XM_MPI_ISP_SetRegister(GPIO_SEN0_DATE7, 0x2);
  XM_MPI_ISP_SetRegister(GPIO_SEN0_DATE8, 0x2);
  XM_MPI_ISP_SetRegister(GPIO_SEN0_DATE9, 0x2);
  XM_MPI_ISP_SetRegister(GPIO_SEN0_DATE10, 0x2);
  XM_MPI_ISP_SetRegister(GPIO_SEN0_DATE11, 0x2);
#endif
#ifdef TWOCHANNEL
  // TP_AD
  XM_MPI_ISP_SetRegister(GPIO_SEN1_MCLK, 0x42); // Sns1_MCLK(AD MClk)
  XM_MPI_ISP_SetRegister(GPIO_SEN1_PCLK, 0x4);  // Sns1_PCLK(AD PClk)

  XM_MPI_ISP_SetRegister(GPIO_SEN1_DATE0, 0x4);
  XM_MPI_ISP_SetRegister(GPIO_SEN1_DATE1, 0x4);
  XM_MPI_ISP_SetRegister(GPIO_SEN1_DATE2, 0x4);
  XM_MPI_ISP_SetRegister(GPIO_SEN1_DATE3, 0x4);
  XM_MPI_ISP_SetRegister(GPIO_SEN1_DATE4, 0x4);
  XM_MPI_ISP_SetRegister(GPIO_SEN1_DATE5, 0x4);
  XM_MPI_ISP_SetRegister(GPIO_SEN1_DATE6, 0x4);
  XM_MPI_ISP_SetRegister(GPIO_SEN1_DATE7, 0x4);

  XM_MPI_ISP_SetRegister(GPIO_I2C1_SCL, 0x4); // i2c
  XM_MPI_ISP_SetRegister(GPIO_I2C1_SDA, 0x4);

  XM_MPI_ISP_SetRegister(GPIO_PIN5_4,
                         0xc00); // tp2823 release reset (Output:High)
#endif

#if (defined SNS1_XM330Y)
  Write_IspReg(0x10020134, 0xC00); // sensor Reset stop
  Write_IspReg(0x10020000, 0xC00); // XM330Y Reset stop
#endif
}

typedef struct xm_pll_attr {
  XM_U32 B_preDiv;
  XM_U32 B_Ni;
  XM_U32 B_Vco;
  XM_U32 B_postDiv_en;
  XM_U32 B_postDiv_mipi;
  XM_U32 B_postDiv_sen0;
  XM_U32 B_postDiv_sen1;
  XM_U32 B_postDiv_ahd;
  XM_U32 B_postDiv_isp_bus;
  XM_U32 B_postDiv_h264;
  XM_U32 B_postDiv_h265;
  XM_U32 B_postDiv_lcd1;
  XM_U32 B_postDiv_lcd2;
} XM_PLL_ATTR;

/**********************************************************
pau8Data:
        0: u32Pll_preD
        1: u32Pll_M
        2: u32Pll_Ctrl
        3: u32AHB_Div
        4: u32SensorClk_Div
        5: u32AHD_Div
        6: u32MIPI_Div
        7: u32CVBS_Div
u8Venc: 		0:AHD 1:CVI 2:TVI 3;CVBS 4:AHD_15fps
u8VstdType: 	0: unknow 1:PAL 2:NTSC
**********************************************************/
static void pllSet_1M(XM_PLL_ATTR *pstPllAttr, XM_U8 u8Venc, XM_U8 u8VstdType,
                      XM_U8 u8SnsClk) {
  switch (u8SnsClk) {
  case SENSORCLK_37x125M:
#if 0
    pstPllAttr->B_preDiv = 6; // 2 M
    pstPllAttr->B_Ni = 594;   // 1188 M
    pstPllAttr->B_Vco = 0;
    pstPllAttr->B_postDiv_sen0 = 32;   // 37.125M
    pstPllAttr->B_postDiv_sen1 = 32;   // 37.125M
    pstPllAttr->B_postDiv_isp_bus = 4; // 297M
    pstPllAttr->B_postDiv_mipi = 32;   // 37.125M
    pstPllAttr->B_postDiv_ahd = 16;    // 74.25M
    pstPllAttr->B_postDiv_h264 = 2;    // 594M
    pstPllAttr->B_postDiv_h265 = 2;    // 594M
#else
    pstPllAttr->B_preDiv = 6; // 2 M
    pstPllAttr->B_Ni = 297;   // 1188 M
    pstPllAttr->B_Vco = 0;
    pstPllAttr->B_postDiv_sen0 = 16;   // 37.125M
    pstPllAttr->B_postDiv_sen1 = 16;   // 37.125M
    pstPllAttr->B_postDiv_isp_bus = 2; // 297M
    pstPllAttr->B_postDiv_mipi = 16;   // 37.125M
    pstPllAttr->B_postDiv_ahd = 8;     // 74.25M
    pstPllAttr->B_postDiv_h264 = 2;    // 594M
    pstPllAttr->B_postDiv_h265 = 2;    // 594M
#endif
    break;
  case SENSORCLK_27M:
    pstPllAttr->B_preDiv = 6; // 2 M
    pstPllAttr->B_Ni = 432;   // 864M
    pstPllAttr->B_Vco = 0;
    pstPllAttr->B_postDiv_sen0 = 32;   // 27M
    pstPllAttr->B_postDiv_sen1 = 32;   // 27M
    pstPllAttr->B_postDiv_isp_bus = 4; // 216M
    pstPllAttr->B_postDiv_mipi = 24;   // 36M
    pstPllAttr->B_postDiv_ahd = 12;    // 72M
    pstPllAttr->B_postDiv_h264 = 2;    // 432M
    pstPllAttr->B_postDiv_h265 = 2;    // 432M
    break;
  case SENSORCLK_24M:
  default:
    if (gstProductInfo.u32SensorType == SENSOR_CHIP_SP140A) {
      if (gstProductInfo.u8StdType == PALS) {
        pstPllAttr->B_preDiv = 6; // 2 M
        pstPllAttr->B_Ni = 504;   // 1008M
        pstPllAttr->B_Vco = 0;
        pstPllAttr->B_postDiv_mipi = 24;   // 42M
        pstPllAttr->B_postDiv_sen0 = 42;   // 24M
        pstPllAttr->B_postDiv_sen1 = 42;   // 24M
        pstPllAttr->B_postDiv_ahd = 12;    // 84M
        pstPllAttr->B_postDiv_isp_bus = 2; // 504M
        pstPllAttr->B_postDiv_h264 = 2;    // 504M
        pstPllAttr->B_postDiv_h265 = 2;    // 504M
      } else {
        pstPllAttr->B_preDiv = 6; // 2 M
        pstPllAttr->B_Ni = 360;   // 720M
        pstPllAttr->B_Vco = 0;
        pstPllAttr->B_postDiv_mipi = 16;   // 45M
        pstPllAttr->B_postDiv_sen0 = 30;   // 24M
        pstPllAttr->B_postDiv_sen1 = 30;   // 24M
        pstPllAttr->B_postDiv_ahd = 12;    // 60M
        pstPllAttr->B_postDiv_isp_bus = 2; // 360M
        pstPllAttr->B_postDiv_h264 = 2;    // 360M
        pstPllAttr->B_postDiv_h265 = 2;    // 360M
      }
    } else if (gstProductInfo.u32SensorType == SENSOR_CHIP_OV9732) {
      pstPllAttr->B_preDiv = 6; // 2 M
      pstPllAttr->B_Ni = 648;   // 1296M
      pstPllAttr->B_Vco = 0;
      pstPllAttr->B_postDiv_mipi = 36;   // 36M
      pstPllAttr->B_postDiv_sen0 = 54;   // 24M
      pstPllAttr->B_postDiv_sen1 = 48;   // 27M
      pstPllAttr->B_postDiv_ahd = 18;    // 72M
      pstPllAttr->B_postDiv_isp_bus = 4; // 324M
      pstPllAttr->B_postDiv_h264 = 4;    // 324M
      pstPllAttr->B_postDiv_h265 = 4;    // 324M
      pstPllAttr->B_postDiv_lcd1 = 48;   // 27M
      pstPllAttr->B_postDiv_lcd2 = 144;  // 9M
    } else {
      pstPllAttr->B_preDiv = 6; // 2 M
      pstPllAttr->B_Ni = 648;   // 1296M
      pstPllAttr->B_Vco = 0;
      pstPllAttr->B_postDiv_mipi = 32;   // 36M
      pstPllAttr->B_postDiv_sen0 = 54;   // 24M
      pstPllAttr->B_postDiv_sen1 = 48;   // 27M
      pstPllAttr->B_postDiv_ahd = 18;    // 72M
      pstPllAttr->B_postDiv_isp_bus = 4; // 324M
      pstPllAttr->B_postDiv_h264 = 4;    // 324M
      pstPllAttr->B_postDiv_h265 = 4;    // 324M
      pstPllAttr->B_postDiv_lcd1 = 48;   // 27M
      pstPllAttr->B_postDiv_lcd2 = 144;  // 9M
    }
    break;
  }

#ifdef OSC_24M
  pstPllAttr->B_preDiv = pstPllAttr->B_preDiv << 1;
#endif
  return;
}

static void pllSet_2M(XM_PLL_ATTR *pstPllAttr, XM_U8 u8Venc, XM_U8 u8VstdType,
                      XM_U8 u8SnsClk) {
  switch (u8SnsClk) {
  case SENSORCLK_27M:
    if (gstProductInfo.u32SensorType == SENSOR_CHIP_F37) {
      pstPllAttr->B_preDiv = 4; // 3 M
      pstPllAttr->B_Ni = 495;   // 1485M
      pstPllAttr->B_Vco = 0;
      pstPllAttr->B_postDiv_sen0 = 55;   // 27M
      pstPllAttr->B_postDiv_sen1 = 55;   // 27M
      pstPllAttr->B_postDiv_isp_bus = 3; // 495M
      pstPllAttr->B_postDiv_mipi = 10;   // 148.5M
      pstPllAttr->B_postDiv_ahd = 10;    // 148.5M
      pstPllAttr->B_postDiv_h264 = 3;    // 495M
      pstPllAttr->B_postDiv_h265 = 3;    // 495M
      DEBUG("Venc:495\n");
    } else {
      if (gstParamIn.u8IpcVenc == IPC_Hx265) {
        pstPllAttr->B_preDiv = 4; // 3 M
        pstPllAttr->B_Ni = 495;   // 1485M
        pstPllAttr->B_Vco = 0;
        pstPllAttr->B_postDiv_sen0 = 55;   // 27M
        pstPllAttr->B_postDiv_sen1 = 55;   // 27M
        pstPllAttr->B_postDiv_isp_bus = 3; // 495M
        pstPllAttr->B_postDiv_mipi = 20;   // 74.25M
        pstPllAttr->B_postDiv_ahd = 10;    // 148.5M
        pstPllAttr->B_postDiv_h264 = 3;    // 495M
        pstPllAttr->B_postDiv_h265 = 3;    // 495M
        DEBUG("Venc(265):495\n");
      } else {
        pstPllAttr->B_preDiv = 4; // 3 M
        pstPllAttr->B_Ni = 495;   // 1485M
        pstPllAttr->B_Vco = 0;
        pstPllAttr->B_postDiv_sen0 = 55;   // 27M
        pstPllAttr->B_postDiv_sen1 = 55;   // 27M
        pstPllAttr->B_postDiv_isp_bus = 3; // 495M
        pstPllAttr->B_postDiv_mipi = 20;   // 74.25M
        pstPllAttr->B_postDiv_ahd = 10;    // 148.5M
        pstPllAttr->B_postDiv_h264 = 3;    // 495M
        pstPllAttr->B_postDiv_h265 = 3;    // 495M
        DEBUG("Venc(264):495\n");
      }
    }
    break;
  case SENSORCLK_24M:
    pstPllAttr->B_preDiv = 4; // 3 M
    pstPllAttr->B_Ni = 648;   // 1944M
    pstPllAttr->B_Vco = 0;
    pstPllAttr->B_postDiv_sen0 = 81;   // 24M
    pstPllAttr->B_postDiv_sen1 = 81;   // 24M
    pstPllAttr->B_postDiv_isp_bus = 4; // 486M
    pstPllAttr->B_postDiv_mipi = 24;   // 81M
    pstPllAttr->B_postDiv_ahd = 24;    // 162M
    pstPllAttr->B_postDiv_h264 = 4;    // 486M
    pstPllAttr->B_postDiv_h265 = 4;    // 486M
    ERR("h264/265: 486M\n");
    break;
  case SENSORCLK_18x5625M:
    if (gstParamIn.u8IpcVenc == IPC_Hx265) {
      pstPllAttr->B_preDiv = 4; // 3 M
      pstPllAttr->B_Ni = 495;   // 1485M
      pstPllAttr->B_Vco = 0;
      pstPllAttr->B_postDiv_sen0 = 80;   // 18.5625M
      pstPllAttr->B_postDiv_sen1 = 80;   // 18.5625M
      pstPllAttr->B_postDiv_isp_bus = 3; // 495M
      pstPllAttr->B_postDiv_mipi = 20;   // 74.25M
      pstPllAttr->B_postDiv_ahd = 10;    // 148.5M
      pstPllAttr->B_postDiv_h264 = 3;    // 495M
      pstPllAttr->B_postDiv_h265 = 3;    // 495M
      DEBUG("Venc(265):495\n");
    } else {
      pstPllAttr->B_preDiv = 4; // 3 M
      pstPllAttr->B_Ni = 495;   // 1485M
      pstPllAttr->B_Vco = 0;
      pstPllAttr->B_postDiv_sen0 = 80;   // 18.5625M
      pstPllAttr->B_postDiv_sen1 = 80;   // 18.5625M
      pstPllAttr->B_postDiv_isp_bus = 3; // 495M
      pstPllAttr->B_postDiv_mipi = 20;   // 74.25M
      pstPllAttr->B_postDiv_ahd = 10;    // 148.5M
      pstPllAttr->B_postDiv_h264 = 3;    // 495M
      pstPllAttr->B_postDiv_h265 = 3;    // 495M
      DEBUG("Venc(264):495\n");
    }
    break;
  case SENSORCLK_37x125M:
  default:
    if (gstParamIn.u8IpcVenc == IPC_Hx265) {
      pstPllAttr->B_preDiv = 4; // 3 M
      pstPllAttr->B_Ni = 495;   // 1485M
      pstPllAttr->B_Vco = 0;
      pstPllAttr->B_postDiv_sen0 = 40;   // 37.125M
      pstPllAttr->B_postDiv_sen1 = 40;   // 37.125M
      pstPllAttr->B_postDiv_isp_bus = 3; // 495M
      pstPllAttr->B_postDiv_mipi = 20;   // 74.25M
      pstPllAttr->B_postDiv_ahd = 10;    // 148.5M
      pstPllAttr->B_postDiv_h264 = 3;    // 495M
      pstPllAttr->B_postDiv_h265 = 3;    // 495M
      DEBUG("Venc(265):495\n");
    } else {
      pstPllAttr->B_preDiv = 4; // 3 M
      pstPllAttr->B_Ni = 495;   // 1485M
      pstPllAttr->B_Vco = 0;
      pstPllAttr->B_postDiv_sen0 = 40;   // 37.125M
      pstPllAttr->B_postDiv_sen1 = 40;   // 37.125M
      pstPllAttr->B_postDiv_isp_bus = 3; // 495M
      pstPllAttr->B_postDiv_mipi = 20;   // 74.25M
      pstPllAttr->B_postDiv_ahd = 10;    // 148.5M
      pstPllAttr->B_postDiv_h264 = 3;    // 495M
      pstPllAttr->B_postDiv_h265 = 3;    // 495M
      DEBUG("Venc(264):495\n");
    }
    break;
  }

#ifdef OSC_24M
  pstPllAttr->B_preDiv = pstPllAttr->B_preDiv << 1;
#endif
  return;
}

static void pllSet_4M(XM_PLL_ATTR *pstPllAttr, XM_U8 u8Venc, XM_U8 u8VstdType,
                      XM_U8 u8SnsClk) {
  switch (u8SnsClk) {
  case SENSORCLK_18M:
    pstPllAttr->B_preDiv = 2; // 6 M
    pstPllAttr->B_Ni = 273;   // 1638M
    pstPllAttr->B_Vco = 0;
    pstPllAttr->B_postDiv_mipi = 14; // 117M 13*9
    pstPllAttr->B_postDiv_sen0 = 91; // 18M
    pstPllAttr->B_postDiv_sen1 = 91; // 18M
    pstPllAttr->B_postDiv_ahd = 26;
    pstPllAttr->B_postDiv_isp_bus = 4; // 409.5M
    pstPllAttr->B_postDiv_h264 = 3;    // 546M
    pstPllAttr->B_postDiv_h265 = 3;    // 546M
    break;
  case SENSORCLK_24M:
  default:
    pstPllAttr->B_preDiv = 6; // 2 M
    pstPllAttr->B_Ni = 576;   // 2*576 = 1152M
    pstPllAttr->B_Vco = 0;
    pstPllAttr->B_postDiv_mipi = 8;    // 1152/8 = 144M
    pstPllAttr->B_postDiv_sen0 = 48;   // 1152/48 = 24M
    pstPllAttr->B_postDiv_sen1 = 48;   // 1296/48 = 27M
    pstPllAttr->B_postDiv_ahd = 18;    // 72M
    pstPllAttr->B_postDiv_isp_bus = 2; // 576M
    pstPllAttr->B_postDiv_h264 = 2;    // 576M
    pstPllAttr->B_postDiv_h265 = 2;    // 576M
    break;
  }
#ifdef OSC_24M
  pstPllAttr->B_preDiv = pstPllAttr->B_preDiv << 1;
#endif
  return;
}

static void pllSet_3M(XM_PLL_ATTR *pstPllAttr, XM_U8 u8Venc, XM_U8 u8VstdType,
                      XM_U8 u8SnsClk) {
  switch (u8SnsClk) {
  case SENSORCLK_18M:
    pstPllAttr->B_preDiv = 2; // 6 M
    pstPllAttr->B_Ni = 273;   // 1638M
    pstPllAttr->B_Vco = 0;
    pstPllAttr->B_postDiv_mipi = 14; // 117M 13*9
    pstPllAttr->B_postDiv_sen0 = 91; // 18M
    pstPllAttr->B_postDiv_sen1 = 91; // 18M
    pstPllAttr->B_postDiv_ahd = 26;
    pstPllAttr->B_postDiv_isp_bus = 4; // 409.5M
    pstPllAttr->B_postDiv_h264 = 3;    // 546M
    pstPllAttr->B_postDiv_h265 = 3;    // 546M
    break;
  case SENSORCLK_27M:
  default:
    pstPllAttr->B_preDiv = 3; // 4 M
    pstPllAttr->B_Ni = 486;   // 1944M
    pstPllAttr->B_Vco = 0;
    pstPllAttr->B_postDiv_mipi = 20;   // 97.2M
    pstPllAttr->B_postDiv_sen0 = 72;   // 27M
    pstPllAttr->B_postDiv_sen1 = 72;   // 27M
    pstPllAttr->B_postDiv_ahd = 10;    // 72M
    pstPllAttr->B_postDiv_isp_bus = 4; // 486M
    pstPllAttr->B_postDiv_h264 = 4;    // 486M
    pstPllAttr->B_postDiv_h265 = 4;    // 486M
    break;
  }
#ifdef OSC_24M
  pstPllAttr->B_preDiv = pstPllAttr->B_preDiv << 1;
#endif
  return;
}

static void pllSet_5M(XM_PLL_ATTR *pstPllAttr, XM_U8 u8Venc, XM_U8 u8VstdType,
                      XM_U8 u8SnsClk) {
  switch (u8SnsClk) {
  case SENSORCLK_27M:
    if (gstProductInfo.u32SensorType != SENSOR_CHIP_SC5332 &&
        gstProductInfo.u32SensorType != SENSOR_CHIP_SC5235) {
      pstPllAttr->B_preDiv = 3;
      pstPllAttr->B_Ni = 486;
      pstPllAttr->B_Vco = 0;
      pstPllAttr->B_postDiv_mipi = 20;
      pstPllAttr->B_postDiv_sen0 = 72;
      pstPllAttr->B_postDiv_sen1 = 72;
      pstPllAttr->B_postDiv_ahd = 10;
      pstPllAttr->B_postDiv_isp_bus = 4;
      pstPllAttr->B_postDiv_h264 = 4;
      pstPllAttr->B_postDiv_h265 = 4;
    } else {
      pstPllAttr->B_preDiv = 4;
      pstPllAttr->B_Ni = 594;
      pstPllAttr->B_Vco = 0;
      pstPllAttr->B_postDiv_mipi = 12;
      pstPllAttr->B_postDiv_sen0 = 66;
      pstPllAttr->B_postDiv_sen1 = 66;
      pstPllAttr->B_postDiv_ahd = 66;
      pstPllAttr->B_postDiv_isp_bus = 5;
      pstPllAttr->B_postDiv_h264 = 4;
      pstPllAttr->B_postDiv_h265 = 4;
    }
    break;
  default:
    if (gstProductInfo.u32SensorType == SENSOR_CHIP_IMX335) {
      pstPllAttr->B_preDiv = 4; // 2 M
      pstPllAttr->B_Ni = 792;   // 2376M
      pstPllAttr->B_Vco = 0;
      pstPllAttr->B_postDiv_mipi = 8;    // 297M
      pstPllAttr->B_postDiv_sen0 = 99;   // 24M
      pstPllAttr->B_postDiv_sen1 = 99;   // 24M
      pstPllAttr->B_postDiv_ahd = 4;     // 594M
      pstPllAttr->B_postDiv_isp_bus = 6; // 396M
      pstPllAttr->B_postDiv_h264 = 5;    // 594M
      pstPllAttr->B_postDiv_h265 = 6;    // 396M
    } else {
      pstPllAttr->B_preDiv = 6; // 2 M
      pstPllAttr->B_Ni = 792;   // 1584M
      pstPllAttr->B_Vco = 0;
      pstPllAttr->B_postDiv_mipi = 10;   // 158.4M
      pstPllAttr->B_postDiv_sen0 = 66;   // 24M
      pstPllAttr->B_postDiv_sen1 = 66;   // 24M
      pstPllAttr->B_postDiv_ahd = 5;     // 316.8
      pstPllAttr->B_postDiv_isp_bus = 4; // 396M
      pstPllAttr->B_postDiv_h264 = 3;    // 528M
      pstPllAttr->B_postDiv_h265 = 4;    // 396M
    }
    break;
  }
#ifdef OSC_24M
  pstPllAttr->B_preDiv = pstPllAttr->B_preDiv << 1;
#endif
  return;
}

static void pllSet_8M(XM_PLL_ATTR *pstPllAttr, XM_U8 u8Venc, XM_U8 u8VstdType,
                      XM_U8 u8SnsClk) {
  switch (u8SnsClk) {
  case SENSORCLK_37x125M:
  default:
    pstPllAttr->B_preDiv = 6; // 2 M
    pstPllAttr->B_Ni = 594;   // 1188M
    pstPllAttr->B_Vco = 0;
    pstPllAttr->B_postDiv_sen0 = 32;   // 37.125M
    pstPllAttr->B_postDiv_sen1 = 32;   // 37.125M
    pstPllAttr->B_postDiv_isp_bus = 2; // 594M
    pstPllAttr->B_postDiv_mipi = 16;   // 74.25M
    pstPllAttr->B_postDiv_ahd = 8;     // 148.5M
    pstPllAttr->B_postDiv_h264 = 2;    // 594M
    pstPllAttr->B_postDiv_h265 = 2;    // 594M
    break;
  }
#ifdef OSC_24M
  pstPllAttr->B_preDiv = pstPllAttr->B_preDiv << 1;
#endif
  return;
}

// u8Mode: 0:Auto		1:(fixed 27M mode)
static void SysPLL_Init(XM_U8 u8Mode, const XM_PRODUCT_INFO *pstProductInfo) {
  if (ExtCfg_GetExtCfgFlg() == 0) {
    return;
  }

#ifdef CHIP_FPGA
  ;
#else
  XM_U8 u8RsltMode;                    // 0:720P		1:1080P
  XM_U32 u32tmp;
  XM_PLL_ATTR stPllAttr;
  void (*pfun_pllSet)(XM_PLL_ATTR * pstPllAttr, XM_U8 u8Venc, XM_U8 u8VstdType,
                      XM_U8 u8SnsClk);
  XM_U8 u8SnsClk = pstProductInfo->u8SensorClk;
  stPllAttr.B_postDiv_en = 1;
  if (u8Mode == 1) {
    u8RsltMode = P720_;
    u8SnsClk = SENSORCLK_27M;

    stPllAttr.B_preDiv = 6; // 2 M
    stPllAttr.B_Ni = 567;   // 2*567 = 1134M
    stPllAttr.B_Vco = 0;
    stPllAttr.B_postDiv_sen0 = 42;   // 1134/42 = 27M
    stPllAttr.B_postDiv_sen1 = 42;   // 1134/42 = 27M
    stPllAttr.B_postDiv_isp_bus = 6; // 297M
    stPllAttr.B_postDiv_h264 = 2;    // 567M
    stPllAttr.B_postDiv_h265 = 2;    // 567M

    stPllAttr.B_postDiv_lcd1 = 48;
    stPllAttr.B_postDiv_lcd2 = 144;
    stPllAttr.B_postDiv_mipi = 8;
    stPllAttr.B_postDiv_ahd = 18;

#ifdef OSC_24M
    stPllAttr.B_preDiv = stPllAttr.B_preDiv << 1;
#endif
  } else {
    u8RsltMode = pstProductInfo->u8RsltType;
    switch (u8RsltMode) {
    case P1080_:
      pfun_pllSet = pllSet_2M;
      break;
    case P1536_:
      pfun_pllSet = pllSet_3M;
      break;
    case P4M_:
      pfun_pllSet = pllSet_4M;
      break;
    case P5M_:
      pfun_pllSet = pllSet_5M;
      break;
    case P4K_:
      pfun_pllSet = pllSet_8M;
      break;
    case P720_:
    default:
      pfun_pllSet = pllSet_1M;
      break;
    }
    (*pfun_pllSet)(&stPllAttr, pstProductInfo->u8Encoder,
                   pstProductInfo->u8StdType, u8SnsClk);
  }
#if 0
  switch (u8VstMode) {
  case P1536_:
    if (pstProductInfo->u8SensorClk == SENSORCLK_18M) {
      B_preDiv = 12; // 2 M
      B_Ni = 234;    // 2*234 = 468M
      B_Vco = 0;
      B_postDiv_sen0 = 26; // 594/16 = 37.125M
      // B_postDiv_sen1 = 26;	//594/16 = 37.125M
      B_postDiv_ahd = 2;     // 234
      B_postDiv_isp_bus = 2; //
      B_postDiv_h264 = 1;    //
    } else if (pstProductInfo->u8SensorClk == SENSORCLK_24M) {
      B_preDiv = 3; // 2 M
      B_Ni = 234;   // 2*234 = 468M
      B_Vco = 0;
      B_postDiv_sen0 = 78; // 24M
      // B_postDiv_sen1 = 78;	//24M
      B_postDiv_ahd = 8;     // 234
      B_postDiv_isp_bus = 8; //
      B_postDiv_h264 = 4;    // 468M
    } else {
      B_preDiv = 12; // 2 M
      B_Ni = 297;    // 2*297 = 594M
      B_Vco = 0;
      B_postDiv_sen0 = 16; // 594/16 = 37.125M
      // B_postDiv_sen1 = 16;	//594/16 = 37.125M
      B_postDiv_ahd = 4;     // 148.5M
      B_postDiv_isp_bus = 2; // 297M
      B_postDiv_h264 = 2;    // 297M
    }
    break;
  case P1080_:
    B_preDiv = 12; // 2 M
    B_Ni = 297;    // 2*297 = 594M
    B_Vco = 0;
    B_postDiv_sen0 = 16; // 594/16 = 37.125M
    // B_postDiv_sen1 = 16;	//594/16 = 37.125M
    B_postDiv_ahd = 4;     // 148.5M
    B_postDiv_isp_bus = 2; // 297M
    B_postDiv_h264 = 2;    // 297M
    break;
  case P720_:
  default:
    if (pstProductInfo->u8SensorClk == SENSORCLK_27M) {
      B_preDiv = 6; // 2 M
      B_Ni = 324;   // 1296M
      B_Vco = 0;
      B_postDiv_sen0 = 48; // 27M
      // B_postDiv_sen1 = 48;	//27M
      B_postDiv_ahd = 18;    // 72M
      B_postDiv_isp_bus = 4; // 288M
      B_postDiv_h264 = 4;    // 288M
    } else {
      if (pstProductInfo->u32SensorType == SENSOR_CHIP_SP140A) {
        B_preDiv = 4; // 2 M
        B_Ni = 224;   // 2*288 = 576M
        B_Vco = 0;
        B_postDiv_sen0 = 56; // 576/24 = 24M
        // B_postDiv_sen1 = 24;	//576/24 = 24M
        B_postDiv_ahd = 16;    // 84M
        B_postDiv_isp_bus = 4; // 288M
        B_postDiv_h264 = 4;    // 288M
      } else {
        B_preDiv = 12; // 2 M
        B_Ni = 288;    // 2*288 = 576M
        B_Vco = 0;
        B_postDiv_sen0 = 24; // 576/24 = 24M
        // B_postDiv_sen1 = 24;	//576/24 = 24M
        B_postDiv_ahd = 8;     // 72M
        B_postDiv_isp_bus = 2; // 288M
        B_postDiv_h264 = 2;    // 288M
      }
    }

    break;
  }
  XM_MPI_ISP_SetRegister(PLL_SYSCTRLREG_LOCK, 0x01);
  XM_MPI_ISP_GetRegister(PLLB1_CTRL, &u32tmp);
  u32tmp &= ~0x1FFFFF;
  B_Ni = MIN2(MAX2(B_Ni, 8), 4095);
  u32tmp |= B_Vco << 19 | B_postDiv_en << 18 | B_Ni << 6 | B_preDiv;
  XM_MPI_ISP_SetRegister(PLLB1_CTRL, u32tmp);

  XM_MPI_ISP_SetRegister(0x2000003c, B_postDiv_sen0 - 1);       // sensor0 clk
  XM_MPI_ISP_SetRegister(0x20000040, (B_postDiv_ahd - 1) << 8); // ahd clk
  XM_MPI_ISP_SetRegister(0x20000048, B_postDiv_isp_bus - 1);    // isp clk
  XM_MPI_ISP_SetRegister(0x2000004C, 0x100 | (B_postDiv_h264 - 1)); // 264 clk
  XM_MPI_ISP_SetRegister(PLL_SYSCTRLREG_LOCK, 0x00);
#else
#if 1
  XM_MPI_ISP_SetRegister(PLL_SYSCTRLREG_LOCK, 0x01);
  XM_MPI_ISP_GetRegister(PLLB1_CTRL, &u32tmp);
  u32tmp &= ~0x1FFFFF;
  stPllAttr.B_Ni = MIN2(MAX2(stPllAttr.B_Ni, 8), 4095);
  u32tmp |= stPllAttr.B_Vco << 19 | stPllAttr.B_postDiv_en << 18 |
            stPllAttr.B_Ni << 6 | stPllAttr.B_preDiv;
  u32tmp |= 0x120 << 20;
  XM_MPI_ISP_SetRegister(PLLB1_CTRL, u32tmp);

  XM_MPI_ISP_SetRegister(PLL_SENCLK_CTRL,
                         ((stPllAttr.B_postDiv_mipi - 1) << 24) |
                             ((stPllAttr.B_postDiv_sen1 - 1) << 8) |
                             (stPllAttr.B_postDiv_sen0 - 1)); // sensor0 clk
  XM_MPI_ISP_SetRegister(PLL_ISPCLK_CTRL, (stPllAttr.B_postDiv_ahd - 1)
                                              << 8); // ahd clk
  XM_MPI_ISP_SetRegister(PLL_LCDCLK_CTRL,
                         (stPllAttr.B_postDiv_lcd1 - 1) << 8 |
                             (stPllAttr.B_postDiv_lcd2 - 1)); // lcd clk
  XM_MPI_ISP_SetRegister(PLL_ISPBUSCLK_CTRL,
                         stPllAttr.B_postDiv_isp_bus - 1); // isp clk
  XM_MPI_ISP_SetRegister(PLL_VENCCLK_CTRL,
                         0x1000000 | ((stPllAttr.B_postDiv_h265 - 1) << 16) |
                             0x100 |
                             (stPllAttr.B_postDiv_h264 - 1)); // 265/ 264 clk
  XM_MPI_ISP_SetRegister(PLL_SYSCTRLREG_LOCK, 0x00);
#else
  XM_MPI_ISP_SetRegister(0x20000000, 0x00000001);
  XM_MPI_ISP_SetRegister(0x20000010, 0x92049484);
  XM_MPI_ISP_SetRegister(0x2000003C, 0x0B004141);
  XM_MPI_ISP_SetRegister(0x20000040, 0x00004100);
  XM_MPI_ISP_SetRegister(0x20000044, 0x00009B33);
  XM_MPI_ISP_SetRegister(0x20000048, 0x00000004);
  XM_MPI_ISP_SetRegister(0x2000004C, 0x01030103);
  XM_MPI_ISP_SetRegister(0x20000000, 0x00000000);
  (void)u32tmp;
#endif
#endif
#endif
}

/*************************************************************************
函数功能:	配置VI裁剪参数(H、V)
输出参数:	u8Mode:
                                        0: Real Data(实际值)
                                        1: 基于标准值进行偏移
                                        2: Refresh
                                u16ValH: 水平方向值(bit15:为符号)    [0, 0xFFFE]
                                        0~0x7FFF			:
                                        0x8000 ~ 0xFFFE	:  <0

                                u16ValV: 垂直方向值(bit15:为符号)	  [0,
0xFFFE] 0~0x7FFF			: 0x8000 ~ 0xFFFE	:  <0 u8Mirror:

note:
        u16ValH/u16ValV = 0xFFFF 时标准不写入
*************************************************************************/
XM_S32 VI_WinSet(XM_U8 u8Mode, XM_U8 u8Mirror, XM_U8 u8Flip, XM_U16 u16ValH,
                 XM_U16 u16ValV) {
  static XM_U8 su8FlipMirrorMode = 0xFF; // bit0: Mode   bit1: Mirror bit2:Flip
  static XM_U16 su16ValH, su16ValV;

  XM_S32 s32StartH, s32StartV;
  XM_S32 s32Ret;
  ISP_PUB_ATTR_S stPubAttr;
#if 0
	DEBUG("VI_WinSet:\n", u8Mode, u8Mirror, u8Flip, u16ValH, u16ValV);
	PrintInt(8,u8Mode);
	PrintInt(8,u8Mirror);
	PrintInt(8,u8Flip);
	PrintInt(8,u16ValH);
	PrintInt(8,u16ValV);	
	ENTER();
#endif
  // 1. PCLK
  if (su8FlipMirrorMode == 0xFF) // first
  {
    su8FlipMirrorMode = 0x01;
    Write_IspReg(VIDOEMODE_DEC_DATA_SEL,
                 (XM_U32)gu8PclkEdge | 0x0fff0000); // Pclk Polarity
    if (gstProductInfo.bHsyncRecEn == XM_TRUE) {
      Write_IspReg(VI_SYNC_HVALID_RECV, 0x00110050);
      Write_IspReg(VI_SYNC_HVALID_RECV_EN, 0x1);
    }
  }

  if (u8Mode == 2) {
    u8Mode = su8FlipMirrorMode & 0x01;
    u8Mirror = (su8FlipMirrorMode >> 1) & 0x01;
    u8Flip = (su8FlipMirrorMode >> 2) & 0x01;
    u16ValH = su16ValH;
    u16ValV = su16ValV;
  }
  // 2. Cut
  s32Ret = XM_MPI_ISP_GetPubAttr(gIspDevApp, &stPubAttr);
  if (s32Ret != XM_SUCCESS) {
    ERR("XM_MPI_ISP_GetPubAttr failed!\r\n");
    return XM_FAILURE;
  }
  s32StartH = stPubAttr.stWndRect.s32X;
  s32StartV = stPubAttr.stWndRect.s32Y;
#if 0
	DEBUG("Get Normal: \n");
	PrintInt(8, s32StartH);
	PrintInt(8, s32StartV);
	PrintInt(8, stPubAttr.stWndRect.u32Width);
	PrintInt(8, stPubAttr.stWndRect.u32Height);
	ENTER();
#endif

  if (u8Mode == 0) {
    if (u16ValH != 0xFFFF)
      s32StartH = (XM_S32)u16ValH;

    if (u16ValV != 0xFFFF)
      s32StartV = (XM_S32)u16ValV;
#if 0
		DEBUG("Mode0: \n");
		PrintInt(8, s32StartH);
		PrintInt(8, s32StartV);
		ENTER();
#endif
  } else {
    if (u16ValH & 0x8000)
      s32StartH = s32StartH - (u16ValH & 0x7FFF);
    else
      s32StartH = s32StartH + (u16ValH & 0x7FFF);

    if (u16ValV & 0x8000)
      s32StartV = s32StartV - (u16ValV & 0x7FFF);
    else
      s32StartV = s32StartV + (u16ValV & 0x7FFF);

    if (s32StartH < 0)
      s32StartH = 0;
    if (s32StartV < 0)
      s32StartV = 0;

#if 0
		DEBUG("Mode1: \n");
		PrintInt(8, s32StartH);
		PrintInt(8, s32StartV);
		ENTER();
#endif
  }

  Write_IspReg(VI_CUT_BEGIN_H, s32StartH);
  Write_IspReg(VI_CUT_END_H, s32StartH + stPubAttr.stWndRect.u32Width);
  Write_IspReg(VI_CUT_BEGIN_V, s32StartV);
  Write_IspReg(VI_CUT_END_V, s32StartV + stPubAttr.stWndRect.u32Height);

  su8FlipMirrorMode =
      (u8Mode & 0x01) | ((u8Mirror & 0x01) << 1) | ((u8Flip & 0x01) << 2);
  su16ValH = u16ValH;
  su16ValV = u16ValV;
  return XM_SUCCESS;
}

#if (defined SNS1_XM330Y)
static XM_VOID bt1120_GpioSet(XM_VOID) {
  XM_MPI_ISP_SetRegister(GPIO_SEN1_PCLK, 0x4); // Sns1_PCLK(AD PClk)
  XM_MPI_ISP_SetRegister(GPIO_SEN1_HSYNC, 0x4);
  XM_MPI_ISP_SetRegister(GPIO_SEN1_VSYNC, 0x4);
  XM_MPI_ISP_SetRegister(GPIO_SEN1_DATE0, 0x4);
  XM_MPI_ISP_SetRegister(GPIO_SEN1_DATE1, 0x4);
  XM_MPI_ISP_SetRegister(GPIO_SEN1_DATE2, 0x4);
  XM_MPI_ISP_SetRegister(GPIO_SEN1_DATE3, 0x4);
  XM_MPI_ISP_SetRegister(GPIO_SEN1_DATE4, 0x4);
  XM_MPI_ISP_SetRegister(GPIO_SEN1_DATE5, 0x4);
  XM_MPI_ISP_SetRegister(GPIO_SEN1_DATE6, 0x4);
  XM_MPI_ISP_SetRegister(GPIO_SEN1_DATE7, 0x4);

  Write_IspReg(0x300e00c0, 0x00110050);
  Write_IspReg(0x300e00c4, 0x1);
}
#endif

/*************************************************************************
函数功能:	配置VI裁剪参数(H、V)(BT1120_LUM)
输出参数:	u8Mode:
                                        0: Real Data(实际值)
                                        1: 基于标准值进行偏移
                                        2: Refresh
                u16ValH: 水平方向值(bit15:为符号)	 [0, 0xFFFE]
                        0~0x7FFF			:
                        0x8000 ~ 0xFFFE :  <0

                u16ValV: 垂直方向值(bit15:为符号)	  [0, 0xFFFE]
                        0~0x7FFF			:
                        0x8000 ~ 0xFFFE :  <0

note:
        u16ValH/u16ValV = 0xFFFF 时标准不写入
        TP2833
        BT1120
*************************************************************************/
static XM_S32 lumaVideo_Set(XM_U8 u8Mode, XM_U16 u16ValH, XM_U16 u16ValV) {
#if (defined SNS1_AD) // TP2833
  // Write_IspReg(LUMA_V_VIE_EN,0x1);
  Write_IspReg(LUM_DEC_SYNC_CTRL, 0x00000003); // 嵌入式行场
  Write_IspReg(LUM_DEC_HEAD_DAT1, 0x0000ff00);
  Write_IspReg(LUM_DEC_HEAD_MSK1, 0x0000ff00);
  Write_IspReg(LUM_DEC_HEAD_MSK2, 0x0000ff00);
  Write_IspReg(LUM_DEC_HEAD_MSK3, 0x0000ff00);

  Write_IspReg(LUM_DEC_SOL_DAT, 0x0000c0c0);
  Write_IspReg(LUM_DEC_EOL_DAT, 0x0000d0d0);
  Write_IspReg(LUM_DEC_SOF_DAT, 0x0000e0e0);
  Write_IspReg(LUM_DEC_EOF_DAT, 0x0000f0f0);
  Write_IspReg(LUM_DEC_SOL_MSK, 0x0000ff00);
  Write_IspReg(LUM_DEC_EOL_MSK, 0x0000ff00);
  Write_IspReg(LUM_DEC_SOF_MSK, 0x0000ff00);
  Write_IspReg(LUM_DEC_EOF_MSK, 0x0000ff00);

  Write_IspReg(LUM_CUT_BEGIN_H, 0x0000001);
  Write_IspReg(LUM_CUT_END_H, 0x0000a01);
  Write_IspReg(LUM_CUT_BEGIN_V, 0x0000000);
  Write_IspReg(LUM_CUT_END_V, 0x00002d0);

  DEBUG("LumaVideo_Set AD(TP2833)!\n");
#elif (defined SNS1_XM330Y)
  XM_U32 u32TotalSizeH, u32TotalSizeV;
  XM_U32 u32ActiveSizeH, u32ActiveSizeV;
  if (u16ValV > 900) {
    u32TotalSizeH = 2640;
    u32TotalSizeV = 1125;
    u32ActiveSizeH = 1920;
    u32ActiveSizeV = 1080;
  } else {
    u32TotalSizeH = 1920;
    u32TotalSizeV = 750;
    u32ActiveSizeH = 1280;
    u32ActiveSizeV = 720;
  }
  bt1120_GpioSet();
  Write_IspReg(LUM_DEC_SYNC_CTRL, 0x00000000); // Bt1120
  Write_IspReg(LUM_DEC_HEAD_DAT1, 0x0000ff00);
  Write_IspReg(LUM_DEC_HEAD_MSK1, 0x0000ff00);
  Write_IspReg(LUM_DEC_HEAD_MSK2, 0x0000ff00);
  Write_IspReg(LUM_DEC_HEAD_MSK3, 0x0000ff00);

  Write_IspReg(LUM_DEC_SOL_DAT, 0x0000c0c0);
  Write_IspReg(LUM_DEC_EOL_DAT, 0x0000d0d0);
  Write_IspReg(LUM_DEC_SOF_DAT, 0x0000e0e0);
  Write_IspReg(LUM_DEC_EOF_DAT, 0x0000f0f0);
  Write_IspReg(LUM_DEC_SOL_MSK, 0x0000ff00);
  Write_IspReg(LUM_DEC_EOL_MSK, 0x0000ff00);
  Write_IspReg(LUM_DEC_SOF_MSK, 0x0000ff00);
  Write_IspReg(LUM_DEC_EOF_MSK, 0x0000ff00);

  Write_IspReg(LUM_TST_TOTALSIZE_H, u32TotalSizeH);
  Write_IspReg(LUM_TST_TOTALSIZE_V, u32TotalSizeV);
  Write_IspReg(LUM_TST_ACTIVESIZE_H, u32ActiveSizeH);
  Write_IspReg(LUM_TST_ACTIVESIZE_V, u32ActiveSizeV);

  Write_IspReg(LUM_CUT_BEGIN_H, 0x0000000);
  Write_IspReg(LUM_CUT_END_H, u16ValH);
  Write_IspReg(LUM_CUT_BEGIN_V, 0x0000000);
  Write_IspReg(LUM_CUT_END_V, u16ValV);

  DEBUG("LumaVideo_Set 330Y!\n");
#endif
  return XM_SUCCESS;
}

static XM_S32 sensorType_get(XM_VOID) {
  XM_U32 u32Tmp;
  I2C_DATA_S stI2CData;

  u32Tmp = SENSOR_CHIP_UNKNOW;
  xmprop_get_value_v2((XM_U8 *)"snsType", &u32Tmp);
  gstProductInfo.u32SensorType = u32Tmp;
  if (u32Tmp != SENSOR_CHIP_UNKNOW) {
    DEBUG("SnsCfgFile: %d\n", gstProductInfo.u32SensorType);
  }
  sensor_set_chip(gstProductInfo.u32SensorType);
  gstProductInfo.u32SensorType = sensor_get_chip(&stI2CData);
  if (u32Tmp != gstProductInfo.u32SensorType) {
    xmprop_set_value_v2((XM_U8 *)"snsType", &(gstProductInfo.u32SensorType));
  }
  return XM_SUCCESS;
}

static void GetProductInfo() {
  XM_U8 au8Bit[4] = {8, 10, 12, 16};
#if 0
  VENC_SCALER_ATTR stVenAttr;
#endif
  ISP_CMOS_SNS_ATTR_S stSnsAttr;

  SysPLL_Init(1, &gstProductInfo);
  gstProductInfo.u32ProductType = 0x00;
  sensorType_get();
  gstProductInfo.u32DSPType = DSP_XM550;
  gstProductInfo.bHsyncRecEn = XM_FALSE;
  gstProductInfo.u8SnsCommMode = 0; // I2C
  gstProductInfo.stSnsIO.SsDovdd = SNSIO_18V;
  gstProductInfo.u8RsltType = P720_;
  gstProductInfo.u8StdType = PALS;

  COMBO_DEV_ATTR_S stComboDevAttr;
  if (XM_MPI_MIPI_GetDevAttr(0, &stComboDevAttr) != XM_SUCCESS) {
    ERR("XM_MPI_MIPI_GetDevAttr failed!\n");
  }
  stComboDevAttr.input_mode = SENSCONT_DVP;
  switch (gstProductInfo.u32SensorType) {
  case SENSOR_CHIP_SP140A:
    // stVenAttr.enPclk = PCLK_42M;
    // XM_MPI_VENC_SetScaler(&stVenAttr);
    gstProductInfo.u8StdType =
        (gstParamIn.u8Vstd == VSTDNULL) ? PALS : gstParamIn.u8Vstd;
    gstProductInfo.u8RsltType = P720_;
    gstProductInfo.SnsBwide = SENSBWIDE_10BIT;
    gstProductInfo.bHsyncRecEn = XM_TRUE;
    gu8PclkEdge = 0;

    gstProductInfo.u8SensorClk = SENSORCLK_24M;
    stComboDevAttr.input_mode = SENSCONT_MIPI;
    stComboDevAttr.mipi_attr.MipiCtrl = 0x001400b8;
    stComboDevAttr.mipi_attr.lane = MIPI_1LANE;  // mipi的lane数
    stComboDevAttr.mipi_attr.depth = MIPI_10BIT; // sensor点深度

    if (gstProductInfo.u8StdType == PALS) {
      stComboDevAttr.mipi_attr.snsAllPixs = 2240; // sensor输出总点数
      stComboDevAttr.mipi_attr.bMpDvpclk = 42000000; //芯片内部并行取点时钟
    } else {
      stComboDevAttr.mipi_attr.snsAllPixs = 2000; // sensor输出总点数
      stComboDevAttr.mipi_attr.bMpDvpclk = 45000000; //芯片内部并行取点时钟
    }
    stComboDevAttr.mipi_attr.snsAllLine = 750; // sensor输出总行数

    stComboDevAttr.mipi_attr.snsActiveLine = 728;
    stComboDevAttr.mipi_attr.snsActivePixs = 1288;

    stComboDevAttr.mipi_attr.bAllPixs =
        stComboDevAttr.mipi_attr.snsAllPixs; //芯片内部并行取点总点数
    stComboDevAttr.mipi_attr.snsMpOutclk =
        stComboDevAttr.mipi_attr.bMpDvpclk * au8Bit[gstProductInfo.SnsBwide] /
        (1 << stComboDevAttr.mipi_attr.lane) / (2); // sensor输出的mipi
    stComboDevAttr.mipi_attr.delay = 0x01010101;    // mipi内部delay
  case SENSOR_CHIP_SC2145H:
  case SENSOR_CHIP_H62:
    gu8PclkEdge = 0;
  case SENSOR_CHIP_H65:
    gstProductInfo.u8RsltType = P720_;
    gstProductInfo.u8SensorClk = SENSORCLK_24M;
    gstProductInfo.bHsyncRecEn = XM_TRUE;
    break;
  case SENSOR_CHIP_OV9732:
    gstProductInfo.u8StdType =
        (gstParamIn.u8Vstd == VSTDNULL) ? PALS : gstParamIn.u8Vstd;
    gstProductInfo.u8RsltType = P720_;
    gstProductInfo.SnsBwide = SENSBWIDE_10BIT;
    gstProductInfo.bHsyncRecEn = XM_TRUE;
    gu8PclkEdge = 0;
    gstProductInfo.u8SensorClk = SENSORCLK_24M;
    stComboDevAttr.input_mode = SENSCONT_MIPI;
    stComboDevAttr.mipi_attr.MipiCtrl = 0x000800b8;
    stComboDevAttr.mipi_attr.lane = MIPI_1LANE;  // mipi的lane数
    stComboDevAttr.mipi_attr.depth = MIPI_10BIT; // sensor点深度
    if (gstProductInfo.u8StdType == PALS) {
      stComboDevAttr.mipi_attr.snsAllPixs = 1920; // sensor输出总点数
    } else {
      stComboDevAttr.mipi_attr.snsAllPixs = 1600; // sensor输出总点数
    }
    stComboDevAttr.mipi_attr.snsAllLine = 750; // sensor输出总行数
    stComboDevAttr.mipi_attr.snsActiveLine = 728;
    stComboDevAttr.mipi_attr.snsActivePixs = 1288;
    stComboDevAttr.mipi_attr.bMpDvpclk = 36000000; //芯片内部并行取点时钟
    stComboDevAttr.mipi_attr.bAllPixs =
        stComboDevAttr.mipi_attr.snsAllPixs; //芯片内部并行取点总点数
    stComboDevAttr.mipi_attr.snsMpOutclk = stComboDevAttr.mipi_attr.bMpDvpclk *
                                           au8Bit[gstProductInfo.SnsBwide] /
                                           (2);  // sensor输出的mipi
    stComboDevAttr.mipi_attr.delay = 0x01010101; // mipi内部delay

    cmos_get_sns_attr(&stSnsAttr);
    stSnsAttr.u8InputMode = SENSCONT_MIPI;
    cmos_set_sns_attr(&stSnsAttr);
    break;
  case SENSOR_CHIP_SC1235:
    gu8PclkEdge = 0;
#ifdef RES_960
    gstProductInfo.u8RsltType = P960_;
#else
    gstProductInfo.u8RsltType = P720_;
#endif
    gstProductInfo.u8SensorClk = SENSORCLK_27M;
    gstProductInfo.SnsBwide = SENSBWIDE_12BIT;
    gstProductInfo.bHsyncRecEn = XM_TRUE;
    break;
  case SENSOR_CHIP_IMX323:
  case SENSOR_CHIP_SC2235:
    gstProductInfo.u8RsltType = P1080_;
    gstProductInfo.u8SensorClk = SENSORCLK_37x125M;
    gstProductInfo.SnsBwide = SENSBWIDE_12BIT;
    gstProductInfo.bHsyncRecEn = XM_TRUE;
    gu8PclkEdge = 0;
    break;
#if 0
  case SENSOR_CHIP_SC2235E:
  case SENSOR_CHIP_SC2235P:
    gstProductInfo.u8StdType =
        (gstParamIn.u8Vstd == VSTDNULL) ? PALS : gstParamIn.u8Vstd;
    gstProductInfo.u8RsltType =
        P1080_; //(gstParamIn.enRslt==PNULL)?P1080_:gstParamIn.enRslt;
    gstProductInfo.SnsBwide = SENSBWIDE_10BIT;
    gstProductInfo.bHsyncRecEn = XM_TRUE;
    gu8PclkEdge = 0;
#ifdef DEVTYPE_AHD
    gstProductInfo.u32ProductType = XM350AI_60X20;
#endif
    gstProductInfo.u8SensorClk = SENSORCLK_27M;
    stComboDevAttr.input_mode = SENSCONT_MIPI;
    stComboDevAttr.mipi_attr.MipiCtrl = 0x001400b8;
    stComboDevAttr.mipi_attr.lane = MIPI_2LANE;  // mipi的lane数
    stComboDevAttr.mipi_attr.depth = MIPI_10BIT; // sensor点深度
    if (stComboDevAttr.input_mode == SENSCONT_DVP) {
      gstProductInfo.u8SensorClk = SENSORCLK_37x125M;
    }
    if (gstProductInfo.u8RsltType == P1080_) {
      if (gstProductInfo.u8StdType == PALS) {
        stComboDevAttr.mipi_attr.snsAllPixs = 2640; // sensor输出总点数
      } else {
        stComboDevAttr.mipi_attr.snsAllPixs = 2200; // sensor输出总点数
      }
      stComboDevAttr.mipi_attr.snsAllLine = 1125; // sensor输出总行数

      stComboDevAttr.mipi_attr.snsActiveLine = 1088;
      stComboDevAttr.mipi_attr.snsActivePixs = 1928;

      stComboDevAttr.mipi_attr.bMpDvpclk = 74250000; //芯片内部并行取点时钟
    } else                                           // 720P
    {
      gstProductInfo.u8SensorClk = SENSORCLK_37x125M;
      if (gstProductInfo.u8StdType == PALS) {
        stComboDevAttr.mipi_attr.snsAllPixs = 1980; // sensor输出总点数
      } else {
        stComboDevAttr.mipi_attr.snsAllPixs = 1650; // sensor输出总点数
      }
      stComboDevAttr.mipi_attr.snsAllLine = 750; // sensor输出总行数
      stComboDevAttr.mipi_attr.snsActiveLine = 728;
      stComboDevAttr.mipi_attr.snsActivePixs = 1288;
      stComboDevAttr.mipi_attr.bMpDvpclk = 37125000; //芯片内部并行取点时钟

      stVenAttr.enPclk = PCLK_37x125M;
      XM_MPI_VENC_SetScaler(&stVenAttr);
    }
    stComboDevAttr.mipi_attr.bAllPixs =
        stComboDevAttr.mipi_attr.snsAllPixs; //芯片内部并行取点总点数
    stComboDevAttr.mipi_attr.snsMpOutclk =
        stComboDevAttr.mipi_attr.bMpDvpclk * au8Bit[gstProductInfo.SnsBwide] /
        (1 << stComboDevAttr.mipi_attr.lane) /
        (1 << stComboDevAttr.mipi_attr.lane);    // sensor输出的mipi
    stComboDevAttr.mipi_attr.delay = 0x01010101; // mipi内部delay
    break;
#endif
  case SENSOR_CHIP_SC307E:
    gstProductInfo.u8StdType =
        (gstParamIn.u8Vstd == VSTDNULL) ? PALS : gstParamIn.u8Vstd;
    gstProductInfo.u8RsltType =
        P1080_; //(gstParamIn.enRslt==PNULL)?P1080_:gstParamIn.enRslt;
    gstProductInfo.bHsyncRecEn = XM_TRUE;
    gu8PclkEdge = 0;
    gstProductInfo.SnsBwide = SENSBWIDE_10BIT;
    gstProductInfo.u8SensorClk = SENSORCLK_27M;
    gstProductInfo.SnsConnect = SENSCONT_MIPI; // SENSCONT_DVP;
    stComboDevAttr.input_mode = gstProductInfo.SnsConnect;
    stComboDevAttr.mipi_attr.MipiCtrl = 0x001400b8;
    stComboDevAttr.mipi_attr.lane = MIPI_2LANE;               // mipi的lane数
    stComboDevAttr.mipi_attr.depth = gstProductInfo.SnsBwide; // sensor点深度
    if (gstProductInfo.u8StdType == PALS) {
      stComboDevAttr.mipi_attr.snsAllPixs = 2640; // sensor输出总点数
    } else {
      stComboDevAttr.mipi_attr.snsAllPixs = 2200; // sensor输出总点数
    }
    stComboDevAttr.mipi_attr.snsAllLine = 1125; // sensor输出总行数

    stComboDevAttr.mipi_attr.snsActiveLine = 1088;
    stComboDevAttr.mipi_attr.snsActivePixs = 1928;
    stComboDevAttr.mipi_attr.bMpDvpclk = 74250000; //芯片内部并行取点时钟
    stComboDevAttr.mipi_attr.bAllPixs =
        stComboDevAttr.mipi_attr.snsAllPixs; //芯片内部并行取点总点数
    stComboDevAttr.mipi_attr.snsMpOutclk =
        stComboDevAttr.mipi_attr.bMpDvpclk * au8Bit[gstProductInfo.SnsBwide] /
        (1 << stComboDevAttr.mipi_attr.lane) /
        (1 << stComboDevAttr.mipi_attr.lane);    // sensor输出的mipi
    stComboDevAttr.mipi_attr.delay = 0x01010101; // mipi内部delay

    if (gstParamIn.u8SnsInterface) {
      stComboDevAttr.input_mode = gstParamIn.u8SnsInterface - 1;
    }
    break;
  case SENSOR_CHIP_SC2145:
    gstProductInfo.u8StdType =
        (gstParamIn.u8Vstd == VSTDNULL) ? PALS : gstParamIn.u8Vstd;
    gstProductInfo.u8RsltType =
        P1080_; //(gstParamIn.enRslt==PNULL)?P1080_:gstParamIn.enRslt;
    gstProductInfo.bHsyncRecEn = XM_TRUE;
    gu8PclkEdge = 0;
    gstProductInfo.SnsBwide = SENSBWIDE_10BIT;
    gstProductInfo.u8SensorClk = SENSORCLK_37x125M;
    stComboDevAttr.input_mode = SENSCONT_MIPI;
    stComboDevAttr.mipi_attr.MipiCtrl = 0x001400b8;
    stComboDevAttr.mipi_attr.lane = MIPI_2LANE;               // mipi的lane数
    stComboDevAttr.mipi_attr.depth = gstProductInfo.SnsBwide; // sensor点深度
    if (gstProductInfo.u8StdType == PALS) {
      stComboDevAttr.mipi_attr.snsAllPixs = 2640; // sensor输出总点数
    } else {
      stComboDevAttr.mipi_attr.snsAllPixs = 2200; // sensor输出总点数
    }
    stComboDevAttr.mipi_attr.snsAllLine = 1125; // sensor输出总行数

    stComboDevAttr.mipi_attr.snsActiveLine = 1088;
    stComboDevAttr.mipi_attr.snsActivePixs = 1928;
    stComboDevAttr.mipi_attr.bMpDvpclk = 74250000; //芯片内部并行取点时钟
    stComboDevAttr.mipi_attr.bAllPixs =
        stComboDevAttr.mipi_attr.snsAllPixs; //芯片内部并行取点总点数
    stComboDevAttr.mipi_attr.snsMpOutclk =
        stComboDevAttr.mipi_attr.bMpDvpclk * au8Bit[gstProductInfo.SnsBwide] /
        (1 << stComboDevAttr.mipi_attr.lane) /
        (1 << stComboDevAttr.mipi_attr.lane);    // sensor输出的mipi
    stComboDevAttr.mipi_attr.delay = 0x04040404; // mipi内部delay
    break;
  case SENSOR_CHIP_IMX307:
    gstProductInfo.u8StdType =
        (gstParamIn.u8Vstd == VSTDNULL) ? PALS : gstParamIn.u8Vstd;
    gstProductInfo.u8RsltType = P1080_;
    gu8PclkEdge = 0;
    gstProductInfo.SnsBwide = SENSBWIDE_12BIT;
    gstProductInfo.bHsyncRecEn = XM_TRUE;
    gstProductInfo.u8SensorClk = SENSORCLK_37x125M;
    stComboDevAttr.input_mode = SENSCONT_MIPI;
    stComboDevAttr.mipi_attr.lane = MIPI_4LANE;  // mipi的lane数
    stComboDevAttr.mipi_attr.depth = MIPI_12BIT; // sensor点深度
    stComboDevAttr.mipi_attr.MipiCtrl = 0x000800b8;
    if (gstProductInfo.u8StdType == PALS) {
      stComboDevAttr.mipi_attr.snsAllPixs = 2640; // sensor输出总点数
    } else {
      stComboDevAttr.mipi_attr.snsAllPixs = 2200; // sensor输出总点数
    }
    stComboDevAttr.mipi_attr.snsAllLine = 1125; // sensor输出总行数

    stComboDevAttr.mipi_attr.snsActiveLine = 1088;
    stComboDevAttr.mipi_attr.snsActivePixs = 1928;

    stComboDevAttr.mipi_attr.bMpDvpclk = 74250000; //芯片内部并行取点时钟

    stComboDevAttr.mipi_attr.bAllPixs =
        stComboDevAttr.mipi_attr.snsAllPixs; //芯片内部并行取点总点数
    stComboDevAttr.mipi_attr.snsMpOutclk =
        stComboDevAttr.mipi_attr.bMpDvpclk * 12 /
        (1 << stComboDevAttr.mipi_attr.lane) /
        (1 << stComboDevAttr.mipi_attr.lane);    // sensor输出的mipi
    stComboDevAttr.mipi_attr.delay = 0x02020202; // mipi内部delay
    break;
  case SENSOR_CHIP_F37:
    gstProductInfo.u8StdType =
        (gstParamIn.u8Vstd == VSTDNULL) ? PALS : gstParamIn.u8Vstd;
    gstProductInfo.u8RsltType =
        P1080_; //(gstParamIn.enRslt==PNULL)?P1080_:gstParamIn.enRslt;
    gstProductInfo.bHsyncRecEn = XM_TRUE;
    gu8PclkEdge = 0;
    gstProductInfo.SnsBwide = SENSBWIDE_10BIT;
    gstProductInfo.u8SensorClk = SENSORCLK_27M;
    stComboDevAttr.input_mode = SENSCONT_MIPI;
    stComboDevAttr.mipi_attr.MipiCtrl = 0x001400b8;
    stComboDevAttr.mipi_attr.lane = MIPI_2LANE;               // mipi的lane数
    stComboDevAttr.mipi_attr.depth = gstProductInfo.SnsBwide; // sensor点深度
    if (gstProductInfo.u8StdType == PALS) {
      stComboDevAttr.mipi_attr.snsAllPixs = 5280; // sensor输出总点数
    } else {
      stComboDevAttr.mipi_attr.snsAllPixs = 4400; // sensor输出总点数
    }
    stComboDevAttr.mipi_attr.snsAllLine = 1125; // sensor输出总行数

    stComboDevAttr.mipi_attr.snsActiveLine = 1088;
    stComboDevAttr.mipi_attr.snsActivePixs = 1928;
    stComboDevAttr.mipi_attr.bMpDvpclk = 148500000; //芯片内部并行取点时钟
    stComboDevAttr.mipi_attr.bAllPixs =
        stComboDevAttr.mipi_attr.snsAllPixs; //芯片内部并行取点总点数
    stComboDevAttr.mipi_attr.snsMpOutclk =
        stComboDevAttr.mipi_attr.bMpDvpclk * au8Bit[gstProductInfo.SnsBwide] /
        (1 << stComboDevAttr.mipi_attr.lane) /
        (1 << stComboDevAttr.mipi_attr.lane);    // sensor输出的mipi
    stComboDevAttr.mipi_attr.delay = 0x02020202; // mipi内部delay
    break;
  case SENSOR_CHIP_SC2335:
    gstProductInfo.u8StdType =
        (gstParamIn.u8Vstd == VSTDNULL) ? PALS : gstParamIn.u8Vstd;
    gstProductInfo.u8RsltType = P1080_;
    gstProductInfo.bHsyncRecEn = XM_TRUE;
    gu8PclkEdge = 0;
    gstProductInfo.SnsBwide = SENSBWIDE_10BIT;
    gstProductInfo.u8SensorClk = SENSORCLK_27M;
    gstProductInfo.SnsConnect = SENSCONT_MIPI;
    stComboDevAttr.input_mode = gstProductInfo.SnsConnect;
    stComboDevAttr.mipi_attr.MipiCtrl = 0x001400b8;
    stComboDevAttr.mipi_attr.lane = MIPI_2LANE;               // mipi的lane数
    stComboDevAttr.mipi_attr.depth = gstProductInfo.SnsBwide; // sensor点深度
    if (gstProductInfo.u8StdType == NRT12p5) {
      stComboDevAttr.mipi_attr.snsAllPixs = 5280; // sensor输出总点数
    } else if (gstProductInfo.u8StdType == NRT15) {
      stComboDevAttr.mipi_attr.snsAllPixs = 2640 * 25 / 15; // sensor输出总点数
    } else if (gstProductInfo.u8StdType == PALS) {
      stComboDevAttr.mipi_attr.snsAllPixs = 2640; // sensor输出总点数
    } else {
      stComboDevAttr.mipi_attr.snsAllPixs = 3300; // sensor输出总点数
    }
    stComboDevAttr.mipi_attr.snsAllLine = 1125; // sensor输出总行数

    stComboDevAttr.mipi_attr.snsActiveLine = 1088;
    stComboDevAttr.mipi_attr.snsActivePixs = 1928;
    if (gstProductInfo.u8FunExt == 0x10) {
      stComboDevAttr.mipi_attr.bMpDvpclk = 37125000; //芯片内部并行取点时钟
    } else {
      stComboDevAttr.mipi_attr.bMpDvpclk = 74250000; //芯片内部并行取点时钟
    }
    stComboDevAttr.mipi_attr.bAllPixs =
        stComboDevAttr.mipi_attr.snsAllPixs; //芯片内部并行取点总点数
    stComboDevAttr.mipi_attr.snsMpOutclk =
        stComboDevAttr.mipi_attr.bMpDvpclk * au8Bit[gstProductInfo.SnsBwide] /
        (1 << stComboDevAttr.mipi_attr.lane) /
        (1 << stComboDevAttr.mipi_attr.lane);    // sensor输出的mipi
    stComboDevAttr.mipi_attr.delay = 0x01010101; // mipi内部delay
    cmos_get_sns_attr(&stSnsAttr);
    stSnsAttr.u8InputMode = stComboDevAttr.input_mode;
    cmos_set_sns_attr(&stSnsAttr);
    break;
  case SENSOR_CHIP_MIS2003:
    gstProductInfo.u8StdType =
        (gstParamIn.u8Vstd == VSTDNULL) ? PALS : gstParamIn.u8Vstd;
    gstProductInfo.u8RsltType = P1080_;
    gstProductInfo.bHsyncRecEn = XM_TRUE;
    gu8PclkEdge = 0;
    gstProductInfo.SnsBwide = SENSBWIDE_10BIT;
    gstProductInfo.u8SensorClk = SENSORCLK_37x125M;
    stComboDevAttr.input_mode = SENSCONT_DVP;

    stComboDevAttr.mipi_attr.MipiCtrl = 0x001400b8;
    stComboDevAttr.mipi_attr.lane = MIPI_2LANE;               // mipi的lane数
    stComboDevAttr.mipi_attr.depth = gstProductInfo.SnsBwide; // sensor点深度
    stComboDevAttr.mipi_attr.snsAllPixs =
        (gstProductInfo.u8StdType == PALS) ? 2640 : 2200;
    stComboDevAttr.mipi_attr.snsAllLine = 1125; // sensor输出总行数
    stComboDevAttr.mipi_attr.snsActiveLine = 1088;
    stComboDevAttr.mipi_attr.snsActivePixs = 1928;
    stComboDevAttr.mipi_attr.bMpDvpclk = 74250000; //芯片内部并行取点时钟
    stComboDevAttr.mipi_attr.bAllPixs =
        stComboDevAttr.mipi_attr.snsAllPixs; //芯片内部并行取点总点数
    stComboDevAttr.mipi_attr.snsMpOutclk =
        stComboDevAttr.mipi_attr.bMpDvpclk * au8Bit[gstProductInfo.SnsBwide] /
        (1 << stComboDevAttr.mipi_attr.lane) /
        (1 << stComboDevAttr.mipi_attr.lane);    // sensor输出的mipi
    stComboDevAttr.mipi_attr.delay = 0x02020202; // mipi内部delay
    break;
  case SENSOR_CHIP_SP2305:
    gstProductInfo.u8StdType =
        (gstParamIn.u8Vstd == VSTDNULL) ? PALS : gstParamIn.u8Vstd;
    gstProductInfo.u8RsltType =
        P1080_; //(gstParamIn.enRslt==PNULL)?P1080_:gstParamIn.enRslt;
    gstProductInfo.bHsyncRecEn = XM_TRUE;
    gu8PclkEdge = 0;
    gstProductInfo.SnsBwide = SENSBWIDE_10BIT;
    gstProductInfo.u8SensorClk = SENSORCLK_27M;
    stComboDevAttr.input_mode = SENSCONT_MIPI;
    stComboDevAttr.mipi_attr.MipiCtrl = 0x001400b8;
    stComboDevAttr.mipi_attr.lane = MIPI_2LANE;               // mipi的lane数
    stComboDevAttr.mipi_attr.depth = gstProductInfo.SnsBwide; // sensor点深度
    if (gstProductInfo.u8StdType == PALS) {
      stComboDevAttr.mipi_attr.snsAllPixs = 2640; // sensor输出总点数
    } else {
      stComboDevAttr.mipi_attr.snsAllPixs = 2200; // sensor输出总点数
    }
    stComboDevAttr.mipi_attr.snsAllLine = 1125; // sensor输出总行数

    stComboDevAttr.mipi_attr.snsActiveLine = 1088;
    stComboDevAttr.mipi_attr.snsActivePixs = 1928;
    stComboDevAttr.mipi_attr.bMpDvpclk = 74250000; //芯片内部并行取点时钟
    stComboDevAttr.mipi_attr.bAllPixs =
        stComboDevAttr.mipi_attr.snsAllPixs; //芯片内部并行取点总点数
    stComboDevAttr.mipi_attr.snsMpOutclk =
        stComboDevAttr.mipi_attr.bMpDvpclk * au8Bit[gstProductInfo.SnsBwide] /
        (1 << stComboDevAttr.mipi_attr.lane) /
        (1 << stComboDevAttr.mipi_attr.lane);    // sensor输出的mipi
    stComboDevAttr.mipi_attr.delay = 0x01010101; // mipi内部delay
    break;
  case SENSOR_CHIP_SC335E:
    gstProductInfo.u8StdType =
        (gstParamIn.u8Vstd == VSTDNULL) ? PALS : gstParamIn.u8Vstd;
    sensor_set_reslotionType(gstParamIn.enRslt);
    gstProductInfo.u8RsltType = P5M_;
    gstProductInfo.u8SensorClk = SENSORCLK_24M;
    gstProductInfo.SnsBwide = SENSBWIDE_10BIT;
    gstProductInfo.bHsyncRecEn = XM_TRUE;
    gu8PclkEdge = 0;

    stComboDevAttr.input_mode = SENSCONT_MIPI;
    stComboDevAttr.mipi_attr.lane = MIPI_2LANE;  // mipi的lane数
    stComboDevAttr.mipi_attr.depth = MIPI_10BIT; // sensor点深度
    stComboDevAttr.mipi_attr.MipiCtrl = 0x001400b8;
    stComboDevAttr.mipi_attr.snsAllLine = 2640;    //	15fps
    stComboDevAttr.mipi_attr.snsAllPixs = 4000;    //	   sensor输出总点数
    stComboDevAttr.mipi_attr.snsActiveLine = 1952; // sensor输出总行数
    stComboDevAttr.mipi_attr.snsActivePixs = 2600; // sensor输出总点数
    stComboDevAttr.mipi_attr.bMpDvpclk = 158400000; //芯片内部并行取点时钟
    stComboDevAttr.mipi_attr.bAllPixs =
        stComboDevAttr.mipi_attr.snsAllPixs; //芯片内部并行取点总点数
    stComboDevAttr.mipi_attr.snsMpOutclk =
        stComboDevAttr.mipi_attr.bMpDvpclk * 10 /
        (1 << stComboDevAttr.mipi_attr.lane) / 2; // sensor输出的mipi
    stComboDevAttr.mipi_attr.delay = 0x04040404;  // mipi内部delay
    break;
  case SENSOR_CHIP_SC5239:
    gstProductInfo.u8StdType =
        (gstParamIn.u8Vstd == VSTDNULL) ? PALS : gstParamIn.u8Vstd;
    if (gstParamIn.enRslt == P1080_) {
      gstParamIn.enRslt = P1536_;
    }
    if (gstParamIn.enRslt == P1536_) {
      gstProductInfo.u8RsltType = P1536_;
    } else {
      gstProductInfo.u8RsltType = P4M_;
    }
    gstProductInfo.u8SensorClk = SENSORCLK_18M;
    gstProductInfo.SnsBwide = SENSBWIDE_12BIT;
    gstProductInfo.bHsyncRecEn = XM_TRUE;
    stComboDevAttr.input_mode = SENSCONT_MIPI;
    stComboDevAttr.mipi_attr.lane = MIPI_2LANE;  // mipi的lane数
    stComboDevAttr.mipi_attr.depth = MIPI_10BIT; // sensor点深度
    gu8PclkEdge = 0;
    stComboDevAttr.mipi_attr.MipiCtrl = 0x001400b8;
#if (defined DEVTYPE_AHD)
    if (gstProductInfo.u8StdType == PALS) {
      stComboDevAttr.mipi_attr.snsAllPixs = 3120; // sensor输出总点数
    } else {
      stComboDevAttr.mipi_attr.snsAllPixs = 2600; // sensor输出总点数
    }
    stVenAttr.enPclk = PCLK_117M;
    XM_MPI_VENC_SetScaler(&stVenAttr);
#else
    if (gstProductInfo.u8RsltType == P1536_) {
      stComboDevAttr.mipi_attr.snsAllPixs = 3120; // sensor输出总点数 (25fps)
    } else {
      stComboDevAttr.mipi_attr.snsAllPixs =
          3900; // 3120      	//sensor输出总点数 (20fps)
    }
#endif
    stComboDevAttr.mipi_attr.snsAllLine = 1500;    // sensor输出总行数
    stComboDevAttr.mipi_attr.snsActiveLine = 1448; // sensor输出总行数
    stComboDevAttr.mipi_attr.snsActivePixs = 2560; // sensor输出总点数
    stComboDevAttr.mipi_attr.bMpDvpclk = 117000000; //芯片内部并行取点时钟
    stComboDevAttr.mipi_attr.bAllPixs =
        stComboDevAttr.mipi_attr.snsAllPixs; //芯片内部并行取点总点数
    stComboDevAttr.mipi_attr.snsMpOutclk =
        stComboDevAttr.mipi_attr.bMpDvpclk * 10 /
        (1 << stComboDevAttr.mipi_attr.lane) / 2; // sensor输出的mipi

    stComboDevAttr.mipi_attr.delay = 0x00000000; // mipi内部delay
    break;
  case SENSOR_CHIP_SC5332:
    if (gstParamIn.enRslt == 4)
      gstProductInfo.u8RsltType = 4;
    else
      gstProductInfo.u8RsltType = 5;
    gstProductInfo.u8SensorClk = SENSORCLK_27M;
    gstProductInfo.SnsBwide = 1;
    gstProductInfo.bHsyncRecEn = 1;
    gu8PclkEdge = 0;
    stComboDevAttr.input_mode = 1;
    stComboDevAttr.mipi_attr.lane = 1;
    stComboDevAttr.mipi_attr.depth = 1;
    stComboDevAttr.mipi_attr.MipiCtrl = 0x1400B8;
    if (gstProductInfo.u8RsltType == 4) {
      stComboDevAttr.mipi_attr.snsMpOutclk = 90000000;
      stComboDevAttr.mipi_attr.snsAllLine = 1500;
      stComboDevAttr.mipi_attr.snsAllPixs = 3840;
      stComboDevAttr.mipi_attr.snsActiveLine = 1448;
      stComboDevAttr.mipi_attr.snsActivePixs = 2560;
      stComboDevAttr.mipi_attr.bMpDvpclk = 144000000;
      stComboDevAttr.mipi_attr.bAllPixs = 3840;
    } else {
      stComboDevAttr.mipi_attr.snsAllPixs = 3000;
      stComboDevAttr.mipi_attr.snsAllLine = 2640;
      stComboDevAttr.mipi_attr.snsActivePixs = 2600;
      stComboDevAttr.mipi_attr.snsActiveLine = 1952;
      stComboDevAttr.mipi_attr.bMpDvpclk = 118800000;
      stComboDevAttr.mipi_attr.bAllPixs = 3000;

      // ?
      stComboDevAttr.mipi_attr.snsMpOutclk =
          stComboDevAttr.mipi_attr.bMpDvpclk * au8Bit[gstProductInfo.SnsBwide] /
          (1 << stComboDevAttr.mipi_attr.lane) /
          (1 << stComboDevAttr.mipi_attr.lane); // sensor输出的mipi

      /*
    stComboDevAttr.mipi_attr.snsMpOutclk =
        stComboDevAttr.mipi_attr.bMpDvpclk *
            (unsigned int)(unsigned __int8)v53[gstProductInfo.SnsBwide - 4] >>
        SLOBYTE(stComboDevAttr.mipi_attr.lane) >>
        SLOBYTE(stComboDevAttr.mipi_attr.lane);*/
    }
    stComboDevAttr.mipi_attr.delay = 0x1010101;
    break;
  case SENSOR_CHIP_SC4236:
    gstProductInfo.u8StdType =
        (gstParamIn.u8Vstd == VSTDNULL) ? PALS : gstParamIn.u8Vstd;
    gstProductInfo.u8RsltType = P1536_;
    gstProductInfo.u8SensorClk = SENSORCLK_18M;
    gstProductInfo.SnsBwide = SENSBWIDE_12BIT;
    gstProductInfo.bHsyncRecEn = XM_TRUE;
    stComboDevAttr.input_mode = SENSCONT_MIPI;
    stComboDevAttr.mipi_attr.lane = MIPI_2LANE;  // mipi的lane数
    stComboDevAttr.mipi_attr.depth = MIPI_10BIT; // sensor点深度
    gu8PclkEdge = 0;
    stComboDevAttr.mipi_attr.MipiCtrl = 0x001400b8;
#if (defined DEVTYPE_AHD)
    if (gstProductInfo.u8StdType == PALS) {
      stComboDevAttr.mipi_attr.snsAllPixs = 3120; // sensor输出总点数
    } else {
      stComboDevAttr.mipi_attr.snsAllPixs = 2600; // sensor输出总点数
    }
#else
    // gstProductInfo.u8StdType = PALS;//XM530的3M实际帧率最高25帧
    stComboDevAttr.mipi_attr.snsAllPixs = 3120; // sensor输出总点数
#endif
    // stVenAttr.enPclk = PCLK_117M;
    stComboDevAttr.mipi_attr.snsAllLine = 1500;    // sensor输出总行数
    stComboDevAttr.mipi_attr.snsActiveLine = 1448; // sensor输出总行数
    stComboDevAttr.mipi_attr.snsActivePixs = 2560; // sensor输出总点数
    stComboDevAttr.mipi_attr.bMpDvpclk = 117000000; //芯片内部并行取点时钟
    stComboDevAttr.mipi_attr.bAllPixs =
        stComboDevAttr.mipi_attr.snsAllPixs; //芯片内部并行取点总点数
    stComboDevAttr.mipi_attr.snsMpOutclk =
        stComboDevAttr.mipi_attr.bMpDvpclk * 10 /
        (1 << stComboDevAttr.mipi_attr.lane) / 2; // sensor输出的mipi
    stComboDevAttr.mipi_attr.delay = 0x00000000;  // mipi内部delay
    break;
  case SENSOR_CHIP_SC5235:
    if (gstParamIn.enRslt == P4M_)
      gstProductInfo.u8RsltType = P4M_;
    else
      gstProductInfo.u8RsltType = P5M_;
    gstProductInfo.u8SensorClk = SENSORCLK_24M;
    gstProductInfo.SnsBwide = SENSBWIDE_10BIT;
    gstProductInfo.bHsyncRecEn = XM_TRUE;
    gu8PclkEdge = 0;

    stComboDevAttr.input_mode = SENSCONT_MIPI;
    stComboDevAttr.mipi_attr.lane = MIPI_2LANE;  // mipi的lane数
    stComboDevAttr.mipi_attr.depth = MIPI_10BIT; // sensor点深度
    stComboDevAttr.mipi_attr.MipiCtrl = 0x001400b8;
    if (gstProductInfo.u8RsltType == P4M_) {
      stComboDevAttr.mipi_attr.snsMpOutclk = 90000000; // sensor输出的mipi
      stComboDevAttr.mipi_attr.snsAllLine = 1500;      // sensor输出总行数
      stComboDevAttr.mipi_attr.snsAllPixs = 3840;      // sensor输出总点数
      stComboDevAttr.mipi_attr.snsActiveLine = 1448;   // sensor输出总行数
      stComboDevAttr.mipi_attr.snsActivePixs = 2560;   // sensor输出总点数

      stComboDevAttr.mipi_attr.bMpDvpclk = 144000000; //芯片内部并行取点时钟
      stComboDevAttr.mipi_attr.bAllPixs = 3840; //芯片内部并行取点总点数
    } else                                      // 5M
    {
      // stComboDevAttr.mipi_attr.snsAllLine = 1980; //sensor输出总行数
      stComboDevAttr.mipi_attr.snsAllLine = 2640;
      stComboDevAttr.mipi_attr.snsAllPixs = 4000;    // sensor输出总点数
      stComboDevAttr.mipi_attr.snsActiveLine = 1952; // sensor输出总行数
      stComboDevAttr.mipi_attr.snsActivePixs = 2600; // sensor输出总点数

      stComboDevAttr.mipi_attr.bMpDvpclk = 158400000; //芯片内部并行取点时钟
      stComboDevAttr.mipi_attr.bAllPixs =
          stComboDevAttr.mipi_attr.snsAllPixs; //芯片内部并行取点总点数
      stComboDevAttr.mipi_attr.snsMpOutclk =
          stComboDevAttr.mipi_attr.bMpDvpclk * 10 /
          (1 << stComboDevAttr.mipi_attr.lane) / 2; // sensor输出的mipi
    }
    stComboDevAttr.mipi_attr.delay = 0x50505050; // mipi内部delay
    break;
  case SENSOR_CHIP_IMX335:
    gstProductInfo.u8StdType =
        (gstParamIn.u8Vstd == VSTDNULL) ? PALS : gstParamIn.u8Vstd;
    sensor_set_reslotionType(gstParamIn.enRslt);
    gstProductInfo.u8RsltType = P5M_;
    gstProductInfo.u8SensorClk = SENSORCLK_24M;
    gstProductInfo.SnsBwide = SENSBWIDE_12BIT;
    gstProductInfo.bHsyncRecEn = XM_TRUE;
    gu8PclkEdge = 0;
    stComboDevAttr.input_mode = SENSCONT_MIPI;
    stComboDevAttr.mipi_attr.lane = MIPI_4LANE;  // mipi的lane数
    stComboDevAttr.mipi_attr.depth = MIPI_12BIT; // sensor点深度
    stComboDevAttr.mipi_attr.MipiCtrl = 0x001400b8;
    stComboDevAttr.mipi_attr.snsAllLine = 4950;    // 15fps
    stComboDevAttr.mipi_attr.snsAllPixs = 4000;    // sensor输出总点数
    stComboDevAttr.mipi_attr.snsActiveLine = 1952; // sensor输出总行数
    stComboDevAttr.mipi_attr.snsActivePixs = 2600; // sensor输出总点数

    stComboDevAttr.mipi_attr.bMpDvpclk = 297000000; //芯片内部并行取点时钟
    stComboDevAttr.mipi_attr.bAllPixs =
        stComboDevAttr.mipi_attr.snsAllPixs; //芯片内部并行取点总点数
    stComboDevAttr.mipi_attr.snsMpOutclk =
        stComboDevAttr.mipi_attr.bMpDvpclk * 12 / 4 / 2; // sensor输出的mipi
    stComboDevAttr.mipi_attr.delay = 0x09090909;         // mipi内部delay
    break;
  case SENSOR_CHIP_K03:
    if (gstParamIn.enRslt == P4M_)
      gstProductInfo.u8RsltType = P4M_;
    else
      gstProductInfo.u8RsltType = P5M_;
    gstProductInfo.u8SensorClk = SENSORCLK_24M;
    gstProductInfo.SnsBwide = SENSBWIDE_10BIT;
    gstProductInfo.bHsyncRecEn = XM_TRUE;
    gu8PclkEdge = 0;

    stComboDevAttr.input_mode = SENSCONT_MIPI;
    stComboDevAttr.mipi_attr.lane = MIPI_4LANE;  // mipi的lane数
    stComboDevAttr.mipi_attr.depth = MIPI_10BIT; // sensor点深度
    stComboDevAttr.mipi_attr.MipiCtrl = 0x001400b8;
    if (gstProductInfo.u8RsltType == P4M_) {
      stComboDevAttr.mipi_attr.snsMpOutclk = 180000000; // sensor输出的mipi
      stComboDevAttr.mipi_attr.snsAllLine = 1500;       // sensor输出总行数
      stComboDevAttr.mipi_attr.snsAllPixs = 3840;       // sensor输出总点数
      stComboDevAttr.mipi_attr.snsActiveLine = 1448;    // sensor输出总行数
      stComboDevAttr.mipi_attr.snsActivePixs = 2560;    // sensor输出总点数

      stComboDevAttr.mipi_attr.bMpDvpclk = 144000000; //芯片内部并行取点时钟
      stComboDevAttr.mipi_attr.bAllPixs = 3840; //芯片内部并行取点总点数
    } else                                      // 5M
    {
      // stComboDevAttr.mipi_attr.snsAllLine = 1980; //sensor输出总行数
      stComboDevAttr.mipi_attr.snsAllLine = 2640;
      stComboDevAttr.mipi_attr.snsAllPixs = 4000;    // sensor输出总点数
      stComboDevAttr.mipi_attr.snsActiveLine = 1952; // sensor输出总行数
      stComboDevAttr.mipi_attr.snsActivePixs = 2600; // sensor输出总点数

      stComboDevAttr.mipi_attr.bMpDvpclk = 158400000; //芯片内部并行取点时钟
      stComboDevAttr.mipi_attr.bAllPixs =
          stComboDevAttr.mipi_attr.snsAllPixs; //芯片内部并行取点总点数
      stComboDevAttr.mipi_attr.snsMpOutclk =
          stComboDevAttr.mipi_attr.bMpDvpclk * 10 / 4 / 2; // sensor输出的mipi
    }
    stComboDevAttr.mipi_attr.delay = 0x50505050; // mipi内部delay
    break;
  case SENSOR_CHIP_AUGE:
    gstProductInfo.u8StdType =
        (gstParamIn.u8Vstd == VSTDNULL) ? PALS : gstParamIn.u8Vstd;
    gstProductInfo.u8RsltType =
        (gstParamIn.enRslt == PNULL) ? P1536_ : gstParamIn.enRslt;
    gstProductInfo.SnsBwide = SENSBWIDE_10BIT;
    gstProductInfo.bHsyncRecEn = XM_TRUE;
    gu8PclkEdge = 0;
    gstProductInfo.u8SensorClk = SENSORCLK_27M;
    stComboDevAttr.input_mode = SENSCONT_MIPI;
    stComboDevAttr.mipi_attr.MipiCtrl = 0x001400b8;
    stComboDevAttr.mipi_attr.lane = MIPI_2LANE;  // mipi的lane数
    stComboDevAttr.mipi_attr.depth = MIPI_10BIT; // sensor点深度

    stComboDevAttr.mipi_attr.snsAllPixs = 2880; // sensor输出总点数

    stComboDevAttr.mipi_attr.snsAllLine = 1350; // sensor输出总行数
    stComboDevAttr.mipi_attr.snsActivePixs = 2304 + 8;
    stComboDevAttr.mipi_attr.snsActiveLine = 1296 + 8;
    stComboDevAttr.mipi_attr.bMpDvpclk = 97200000; //芯片内部并行取点时钟
    stComboDevAttr.mipi_attr.bAllPixs =
        stComboDevAttr.mipi_attr.snsAllPixs; //芯片内部并行取点总点数
    stComboDevAttr.mipi_attr.snsMpOutclk =
        stComboDevAttr.mipi_attr.bMpDvpclk * au8Bit[gstProductInfo.SnsBwide] /
        (1 << stComboDevAttr.mipi_attr.lane) /
        (1 << stComboDevAttr.mipi_attr.lane);    // sensor输出的mipi
    stComboDevAttr.mipi_attr.delay = 0x04040404; // hardware of huiYun
    break;
  case SENSOR_CHIP_SC3035:
    gstProductInfo.u8StdType =
        (gstParamIn.u8Vstd == VSTDNULL) ? PALS : gstParamIn.u8Vstd;
    gstProductInfo.u8RsltType = P1536_;
    gstProductInfo.SnsBwide = SENSBWIDE_12BIT;
    gstProductInfo.bHsyncRecEn = XM_TRUE;
    gu8PclkEdge = 0;
    gstProductInfo.u8SensorClk = SENSORCLK_27M;

    stComboDevAttr.input_mode = SENSCONT_DVP;
    break;
  case SENSOR_CHIP_APOLLO:
    gstProductInfo.u8StdType =
        (gstParamIn.u8Vstd == VSTDNULL) ? PALS : gstParamIn.u8Vstd;
    gstProductInfo.u8RsltType =
        (gstParamIn.enRslt == PNULL) ? P4K_ : gstParamIn.enRslt;
    gstProductInfo.SnsBwide = SENSBWIDE_12BIT;
    gstProductInfo.bHsyncRecEn = XM_TRUE;
    gu8PclkEdge = 0;
    gstProductInfo.u8SensorClk = SENSORCLK_37x125M;
    stComboDevAttr.input_mode = SENSCONT_MIPI;
    stComboDevAttr.mipi_attr.MipiCtrl = 0x001400b8;
    stComboDevAttr.mipi_attr.lane = MIPI_4LANE;  // mipi的lane数
    stComboDevAttr.mipi_attr.depth = MIPI_12BIT; // sensor点深度

    if (gstProductInfo.u8StdType == PALS) {
      stComboDevAttr.mipi_attr.snsAllPixs = 2200; // sensor输出总点数
    } else {
      stComboDevAttr.mipi_attr.snsAllPixs = 2200; // sensor输出总点数
    }
    stComboDevAttr.mipi_attr.snsAllLine = 2250; // sensor输出总行数
    stComboDevAttr.mipi_attr.snsActivePixs = 1920 + 8;
    stComboDevAttr.mipi_attr.snsActiveLine = 2160 + 8;
    stComboDevAttr.mipi_attr.bMpDvpclk = 74250000; //芯片内部并行取点时钟
    stComboDevAttr.mipi_attr.bAllPixs =
        stComboDevAttr.mipi_attr.snsAllPixs; //芯片内部并行取点总点数
    stComboDevAttr.mipi_attr.snsMpOutclk =
        stComboDevAttr.mipi_attr.bMpDvpclk * au8Bit[gstProductInfo.SnsBwide] /
        (1 << stComboDevAttr.mipi_attr.lane) /
        (1 << stComboDevAttr.mipi_attr.lane);    // sensor输出的mipi
    stComboDevAttr.mipi_attr.delay = 0x01010101; // mipi内部delay
    break;
  default:
    gstProductInfo.u32SensorType = SENSOR_CHIP_AR0330;
    gstProductInfo.u8RsltType = P1536_;
    gstProductInfo.u8SensorClk = SENSORCLK_37x125M;
    gstProductInfo.bHsyncRecEn = XM_TRUE;
    break;
  }
  gstProductInfo.SnsConnect = stComboDevAttr.input_mode;
  cmos_get_sns_attr(&stSnsAttr);
  stSnsAttr.u8InputMode = gstProductInfo.SnsConnect;
  cmos_set_sns_attr(&stSnsAttr);
  sensor_set_chip(gstProductInfo.u32SensorType);
  gstProductInfo.u8Encoder = TXVENC_BUTT;
#ifdef DEVTYPE_AHD
  gstProductInfo.u8Encoder = TXVENC_AHD;
#endif
  OHTER_DATA_S stOtherData;
  if (ExtCfg_Other_Get(&stOtherData) == XM_SUCCESS) {
    gstProductInfo.u8StdType = stOtherData.u8VencVstd & 0x0F;
    gstProductInfo.u8Encoder = (stOtherData.u8VencVstd >> 4) & 0x0F;
    if (stOtherData.u8Rslt != 0xFF)
      gstProductInfo.u8RsltType = stOtherData.u8Rslt;
  }
  SysPLL_Init(0, &gstProductInfo);
  if (XM_MPI_MIPI_SetDevAttr(0, MIPI_SET_DEV_ATTR, &stComboDevAttr) !=
      XM_SUCCESS) {
    ERR("XM_MPI_MIPI_SetDevAttr failed!\n");
  }
#if (!defined DEVTYPE_AHD)
  camera_init(&(gstProductInfo.u32ProductType));
#endif

#ifdef SNS1_XM330Y
  gstProductInfo.u32ProductType = DOUBLE_XM650AI_XM330Y;
#endif

  DEBUG("DspChip:%s\n",
        (gstProductInfo.u32DSPType == DSP_XM550) ? "XM550" : "XM530");
  DEBUG("SnsIF:%s\n",
        (gstProductInfo.SnsConnect == SENSCONT_MIPI) ? "MIPI" : "DVP");
}

extern void XM_MPI_ISP_VsyncRun();

#if 0
static int getLocalTime() {
#ifdef SOC_SYSTEM
  static time_t sTimter = 0;
  time_t timer; // time_t就是long int 类型
  timer = time(NULL);
  if (timer > sTimter) {
    sTimter = timer;
    return 1;
  }
#endif
  return 0;
}
#endif

#ifdef SOC_NONE
static XM_S32 aeAwbInfo_save() {
  XM_U32 u32Addr;
  ISP_INNER_STATE_INFO_S stInnerStateInfo;
  ISP_WB_INFO_S stWBInfo;
  ExtCfg_BaseAddrGet(0, ROM_FUN_AEAWBINFO, &u32Addr);
#if 0
  static XM_U8 su8Ct = 0;
  XM_U32 u32Tmp;
  XM_U16 u16Tmp;
  u32Tmp = 0;
  if (++su8Ct > 100) {
    su8Ct = 0;
    DEBUG(" ~~~~~~~~~########\n");
    DBG("Addr: ");
    PrintHex(8, u32Addr);
    ENTER();
    ExtCfg_Read_V2(1, u32Addr, (XM_U8 *)&u32Tmp);
    DBG("Size: ");
    PrintInt(8, u32Tmp);
    ENTER();
    ExtCfg_Read_V2(4, u32Addr + 1, (XM_U8 *)&u32Tmp);
    DBG("Exp: ");
    PrintInt(8, u32Tmp);
    ENTER();
    ExtCfg_Read_V2(4, u32Addr + 5, (XM_U8 *)&u32Tmp);
    DBG("Shut: ");
    PrintInt(8, u32Tmp);
    ENTER();
    ExtCfg_Read_V2(4, u32Addr + 9, (XM_U8 *)&u32Tmp);
    DBG("Again: ");
    PrintInt(8, u32Tmp);
    ENTER();
    ExtCfg_Read_V2(4, u32Addr + 13, (XM_U8 *)&u32Tmp);
    DBG("Dgain: ");
    PrintInt(8, u32Tmp);
    ENTER();
    ExtCfg_Read_V2(4, u32Addr + 17, (XM_U8 *)&u32Tmp);
    DBG("IspDgain: ");
    PrintInt(8, u32Tmp);
    ENTER();

    ExtCfg_Read_V2(2, u32Addr + 37, (XM_U8 *)&u16Tmp);
    DBG("RGain: ");
    PrintInt(8, u16Tmp);
    ENTER();
    ExtCfg_Read_V2(2, u32Addr + 39, (XM_U8 *)&u16Tmp);
    DBG("GGain: ");
    PrintInt(8, u16Tmp);
    ENTER();
    ExtCfg_Read_V2(2, u32Addr + 41, (XM_U8 *)&u16Tmp);
    DBG("BGain: ");
    PrintInt(8, u16Tmp);
    ENTER();
    ExtCfg_Read_V2(2, u32Addr + 43, (XM_U8 *)&u16Tmp);
    DBG("CT: ");
    PrintInt(8, u16Tmp);
    ENTER();
  }

#endif
  XM_MPI_ISP_QueryInnerStateInfo(gIspDevApp, &stInnerStateInfo);
  XM_MPI_ISP_QueryWBInfo(&stWBInfo);

#if 0
  DEBUG("Now: ~~~~~\n");
  DBG("Exp: ");
  PrintInt(8, stInnerStateInfo.u32Exposure);
  ENTER();
  DBG("Shut: ");
  PrintInt(8, stInnerStateInfo.u32ExposureTime);
  ENTER();
  DBG("Again: ");
  PrintInt(8, stInnerStateInfo.u32AnalogGain);
  ENTER();
  DBG("Dgain: ");
  PrintInt(8, stInnerStateInfo.u32DigitalGain);
  ENTER();
  DBG("IspDgain: ");
  PrintInt(8, stInnerStateInfo.u32IspDGain);
  ENTER();
  DBG("RGain: ");
  PrintInt(8, (XM_U32)stWBInfo.u16Rgain);
  ENTER();
  DBG("GGain: ");
  PrintInt(8, (XM_U32)stWBInfo.u16Ggain);
  ENTER();
  DBG("BGain: ");
  PrintInt(8, (XM_U32)stWBInfo.u16Bgain);
  ENTER();
  DBG("CT: ");
  PrintInt(8, (XM_U32)stWBInfo.u16ColorTemp);
  ENTER();
  ENTER();
  ENTER();
#endif

  u32Addr += 1;
  ExtCfg_Write_V2(4, u32Addr, (XM_U8 *)&stInnerStateInfo.u32Exposure);
  u32Addr += 4;

  ExtCfg_Write_V2(4, u32Addr, (XM_U8 *)&stInnerStateInfo.u32ExposureTime);
  u32Addr += 4;

  ExtCfg_Write_V2(4, u32Addr, (XM_U8 *)&stInnerStateInfo.u32AnalogGain);
  u32Addr += 4;

  ExtCfg_Write_V2(4, u32Addr, (XM_U8 *)&stInnerStateInfo.u32DigitalGain);
  u32Addr += 4;

  ExtCfg_Write_V2(4, u32Addr, (XM_U8 *)&stInnerStateInfo.u32IspDGain);
  u32Addr += 4;

  u32Addr += 16;
  ExtCfg_Write_V2(2, u32Addr, (XM_U8 *)&stWBInfo.u16Rgain);
  u32Addr += 2;

  ExtCfg_Write_V2(2, u32Addr, (XM_U8 *)&stWBInfo.u16Ggain);
  u32Addr += 2;

  ExtCfg_Write_V2(2, u32Addr, (XM_U8 *)&stWBInfo.u16Bgain);
  u32Addr += 2;

  ExtCfg_Write_V2(2, u32Addr, (XM_U8 *)&stWBInfo.u16ColorTemp);
  u32Addr += 2;

  return 0;
}
#endif

#if 0
#define DSCALER_XDELTA (0x30078000)
#define DSCALER_YDELTA (0x30078004)
#define DSCALER_BYPASS_MODE (0x30078008)

#define VIDEO_CUT (0x30081000)
#define HOR_CUT_BEGIN (VIDEO_CUT + 0x000)
#define HOR_CUT_END (VIDEO_CUT + 0x004)
#define VER_CUT_BEGIN (VIDEO_CUT + 0x008)
#define VER_CUT_END (VIDEO_CUT + 0x00C)

#define VIDEO_STORE (0x30084000)
#define Y_LINE_ADDR_INC (VIDEO_STORE + 0x020)
#define Y_LINE_NUM_PIX (VIDEO_STORE + 0x024)
#define C_LINE_ADDR_INC (VIDEO_STORE + 0x028)
#define C_LINE_NUM_PIX (VIDEO_STORE + 0x02C)
#define STORE_EN (VIDEO_STORE + 0x034)
#define TOTAL_FRAME_NUM (VIDEO_STORE + 0x038)
#define Y_ADDR0 (VIDEO_STORE + 0x040)
#define C_ADDR0 (VIDEO_STORE + 0x0C0)
#define Y_FLIP_ADDR0 (VIDEO_STORE + 0x140)
#define C_FLIP_ADDR0 (VIDEO_STORE + 0x1C0)
static void SysVideoStore_Init() {
  XM_U32 u32Width = 1280;
  XM_U32 u32Height = 720;
  XM_U8 u8i, u8BufNum = 10; //存储多少帧 可配置
  XM_U32 u32PhyAddr = 0x86400000;
  XM_U32 u32YAddr, u32CAddr, u32YSize, u32CSize, u32CutStartH, u32CutStartV;
  u32CutStartH = 0;
  u32CutStartV = 0;

  XM_MPI_ISP_SetRegister(Y_LINE_ADDR_INC, u32Width >> 3);
  XM_MPI_ISP_SetRegister(Y_LINE_NUM_PIX, u32Width);

  XM_MPI_ISP_SetRegister(C_LINE_ADDR_INC, u32Width >> 3);
  XM_MPI_ISP_SetRegister(C_LINE_NUM_PIX, u32Width);

  u32YSize = u32Width * u32Height;
  u32CSize = u32Width * u32Height >> 1;
  DEBUG("VideoStore:\n");
  for (u8i = 0; u8i < u8BufNum; u8i++) {
    u32YAddr = u32PhyAddr + (u32YSize + u32CSize) * u8i;
    u32CAddr = u32YAddr + u32YSize;

    u32YAddr = (u32YAddr - 0x80000000) >> 3;
    u32CAddr = (u32CAddr - 0x80000000) >> 3;
    XM_MPI_ISP_SetRegister(Y_ADDR0 + 4 * u8i, u32YAddr);
    XM_MPI_ISP_SetRegister(C_ADDR0 + 4 * u8i, u32CAddr);

    PrintHex(8, u32YAddr);
    DBG(" ");
    PrintHex(8, u32CAddr);
    DBG(" ");
    u32YAddr += (u32Width * (u32Height - 1) >> 3);
    u32CAddr += (u32Width * (u32Height / 2 - 1) >> 3);
    XM_MPI_ISP_SetRegister(Y_FLIP_ADDR0 + 4 * u8i, u32YAddr);
    XM_MPI_ISP_SetRegister(C_FLIP_ADDR0 + 4 * u8i, u32CAddr);

    PrintHex(8, u32YAddr);
    DBG(" ");
    PrintHex(8, u32CAddr);
    DBG(" ");
    ENTER();
  }
  XM_MPI_ISP_SetRegister(HOR_CUT_BEGIN, u32CutStartH);
  XM_MPI_ISP_SetRegister(HOR_CUT_END, u32CutStartH + u32Width);

  XM_MPI_ISP_SetRegister(VER_CUT_BEGIN, u32CutStartV);
  XM_MPI_ISP_SetRegister(VER_CUT_END, u32CutStartV + u32Height);

  //配置 down 默认关闭
  XM_MPI_ISP_SetRegister(DSCALER_XDELTA, 0x400);
  XM_MPI_ISP_SetRegister(DSCALER_YDELTA, 0x400);
  XM_MPI_ISP_SetRegister(DSCALER_BYPASS_MODE, 1);
  XM_MPI_ISP_SetRegister(TOTAL_FRAME_NUM, (XM_U32)u8BufNum);
  XM_MPI_ISP_SetRegister(STORE_EN, 1);
}
#endif

#if (defined SOC_SYSTEM) || (defined SOC_ALIOS)
static XM_VOID *ISP_Run_SocSystem(XM_VOID) {
  XM_MPI_ISP_Run(gIspDevApp);
  return XM_NULL;
}

static XM_S32 GetIspWndRect(XM_U8 u8Fps, RECT_S *pstRect) {
  if (pstRect == NULL)
    return XM_FAILURE;
  switch (gstProductInfo.u32SensorType) {
  case SENSOR_CHIP_SP140A:
    pstRect->s32X = 1;
    pstRect->s32Y = 0x06;
    pstRect->u32Height = 726;
    pstRect->u32Width = 1286;
    break;
  case SENSOR_CHIP_H62:
    pstRect->s32X = 7;
    pstRect->s32Y = 0x10;
    pstRect->u32Height = 726;
    pstRect->u32Width = 1286;
    break;
  case SENSOR_CHIP_OV9732:
    pstRect->s32X = 0x02;
    pstRect->s32Y = 0x02;
    pstRect->u32Height = 726;
    pstRect->u32Width = 1286;
    break;
  case SENSOR_CHIP_H65:
    pstRect->s32X = 2;
    pstRect->s32Y = 0x15;
    pstRect->u32Height = 726;
    pstRect->u32Width = 1286;
    break;
  case SENSOR_CHIP_SC1235:
#ifdef RES_960
    pstRect->s32X = 2;
    pstRect->s32Y = 9;
    pstRect->u32Height = 966;
    pstRect->u32Width = 1286;
#else
    pstRect->s32X = 3;
    pstRect->s32Y = 15;
    pstRect->u32Height = 726;
    pstRect->u32Width = 1286;
#endif
    break;
  case SENSOR_CHIP_SC2235:
    pstRect->s32X = 1;
    pstRect->s32Y = 15;
    pstRect->u32Height = 1088;
    pstRect->u32Width = 1928;
    break;
  case SENSOR_CHIP_SC2235E:
    pstRect->s32X = 1;
    pstRect->s32Y = 6;
    pstRect->u32Height = 1088;
    pstRect->u32Width = 1920;
    break;
  case SENSOR_CHIP_SC2235P:
    if (gstProductInfo.u8RsltType == P1080_) {
#if 1 // Mipi
      pstRect->s32X = 1;
      pstRect->s32Y = 2;
      pstRect->u32Height = 1088;
      pstRect->u32Width = 1920;
#else
      pstRect->s32X = 1;
      pstRect->s32Y = 15;
      pstRect->u32Height = 1088;
      pstRect->u32Width = 1928;
#endif
    } else {
      pstRect->s32X = 1;
      pstRect->s32Y = 2;
      pstRect->u32Height = 728;
      pstRect->u32Width = 1288;
    }

    break;
  case SENSOR_CHIP_SC307E:
    if (gstProductInfo.SnsConnect == SENSCONT_DVP) {
      pstRect->s32X = 1;
      pstRect->s32Y = 0x15;
    } else {
      pstRect->s32X = 1;
      pstRect->s32Y = 4;
    }
    pstRect->u32Height = 1088;
    pstRect->u32Width = 1920;
    break;
  case SENSOR_CHIP_SC2145:
    pstRect->s32X = 1;
    pstRect->s32Y = 0;
    pstRect->u32Height = 1088;
    pstRect->u32Width = 1920;
    break;
  case SENSOR_CHIP_IMX307: // Mipi
    pstRect->s32X = 2;
    pstRect->s32Y = 7;
    pstRect->u32Height = 1086;
    pstRect->u32Width = 1930;
    break;
  case SENSOR_CHIP_F37: // Mipi
    pstRect->s32X = 3;
    pstRect->s32Y = 4;
    pstRect->u32Height = 1088;
    pstRect->u32Width = 1920;
    break;
  case SENSOR_CHIP_SC2335:
    pstRect->s32X = 1;
    pstRect->s32Y = 0;
    pstRect->u32Height = 1080;
    pstRect->u32Width = 1920;
    break;
  case SENSOR_CHIP_MIS2003:
  case SENSOR_CHIP_SP2305: // Mipi
    pstRect->s32X = 3;
    pstRect->s32Y = 2;
    pstRect->u32Height = 1088;
    pstRect->u32Width = 1920;
    break;
    break;
  case SENSOR_CHIP_SC2145H:
    pstRect->s32X = 3;
    pstRect->s32Y = 0x0D;
    pstRect->u32Height = 726;
    pstRect->u32Width = 1286;
    break;
  case SENSOR_CHIP_IMX323:
    pstRect->s32X = 0xC2;
    pstRect->s32Y = 0x14;
    pstRect->u32Height = 1086;
    pstRect->u32Width = 1926;
    break;
  case SENSOR_CHIP_SC4236:
    pstRect->s32X = 0x1;
    pstRect->s32Y = 0x04;
    pstRect->u32Height = 1296;
    pstRect->u32Width = 2300;
    break;
  case SENSOR_CHIP_SC5239:
    pstRect->s32X = 0x1;
    pstRect->s32Y = 0x04;
    if (gstProductInfo.u8RsltType == P1536_) {
      pstRect->u32Height = 1296;
      pstRect->u32Width = 2300;
    } else {
      pstRect->u32Height = 1448;
      pstRect->u32Width = 2300;
    }
    break;
  case SENSOR_CHIP_SC5332:
    if (gstProductInfo.u8RsltType == 4) {
      pstRect->s32X = 1;
      pstRect->s32Y = 0;
      pstRect->u32Height = 1448;
      pstRect->u32Width = 2560;
    } else {
      pstRect->s32X = 1;
      pstRect->s32Y = 0;
      pstRect->u32Width = 2600;
      pstRect->u32Height = 1952;
    }
    break;
  case SENSOR_CHIP_AR0330:
    pstRect->s32X = 1;
    pstRect->s32Y = 1;
    pstRect->u32Height = 1542;
    pstRect->u32Width = 2054;
    break;
  case SENSOR_CHIP_K03:
    if (gstProductInfo.u8RsltType == P4M_) {
      pstRect->s32X = 0x1;
      pstRect->s32Y = 0x0;
      pstRect->u32Height = 1448;
      pstRect->u32Width = 2560;
    } else {
      pstRect->s32X = 0x1;
      pstRect->s32Y = 0x0;
      pstRect->u32Height = 1954;
      pstRect->u32Width = 2600;
    }
    break;
  case SENSOR_CHIP_IMX335:
    if (gstProductInfo.u8RsltType == P4M_) {
      pstRect->s32X = 0x1;
      pstRect->s32Y = 0x0;
      pstRect->u32Height = 1448;
      pstRect->u32Width = 2560;
    } else {
      pstRect->s32X = 0x2;
      pstRect->s32Y = 0x1;
      pstRect->u32Height = 1954;
      pstRect->u32Width = 2600;
    }
    break;
  case SENSOR_CHIP_SC5235:
  case SENSOR_CHIP_SC335E:
    if (gstProductInfo.u8RsltType == P4M_) {
      pstRect->s32X = 0x1;
      pstRect->s32Y = 0x0;
      pstRect->u32Height = 1448;
      pstRect->u32Width = 2560;
    } else {
      pstRect->s32X = 0x1;
      pstRect->s32Y = 0x0;
      pstRect->u32Height = 1954;
      pstRect->u32Width = 2600;
    }
    break;
  case SENSOR_CHIP_AUGE:
    pstRect->s32X = 0x1;
    pstRect->s32Y = 0x00;
    pstRect->u32Width = 2304;
    pstRect->u32Height = 1296 + 8;
    break;
  case SENSOR_CHIP_SC3035:
    pstRect->s32X = 0x1;
    pstRect->s32Y = 0x11;
    pstRect->u32Width = 2048 + 8;
    pstRect->u32Height = 1536 + 8;
    break;
  case SENSOR_CHIP_APOLLO:
    pstRect->s32X = 0x01;
    pstRect->s32Y = 0x00;
    pstRect->u32Width = 1920;
    pstRect->u32Height = 2160 + 8;
    break;
  default:
    pstRect->s32X = 0;
    pstRect->s32Y = 0;
    pstRect->u32Height = 1280;
    pstRect->u32Width = 720;
    ERR("Unknow sensor type, VI failed!");
    return XM_FAILURE;
    break;
  }
  return XM_SUCCESS;
}
#else

// 0:Go on(run)    1:Stop
static XM_S32 ispStab_Deal(STABDEAL_S *pstStabDeal, XM_U16 u16FrameCnt) {
  static XM_U8 su8StabFlg = 0;
  XM_U32 u32Val = 0;
  XM_MPI_ISP_StabStats(&u32Val);
  u32Val = u32Val >> 6;

  // 图像稳定判断
  if ((su8StabFlg == 0) && ((u16FrameCnt == pstStabDeal->u16FmRunNum) ||
                            (u32Val <= (XM_U32)gu16Tolerance))) {
    su8StabFlg = 1;
#ifdef DEBUG_ON
    DEBUG("Statb!!!!\n");
#endif
    ExtCfg_AeAwbInit(1, gstProductInfo.u8StdType - 1);
    ExtCfg_Write_V2(2, (XM_U32)(pstStabDeal->u32IspAddrData - 3),
                    (XM_U8 *)&u16FrameCnt);
    ExtCft_WriteIsp(0, (XM_U32)(pstStabDeal->u32IspAddrData),
                    (XM_U16)(pstStabDeal->u8IspRegNum));
    if (pstStabDeal->u8DealAfterStab)
      return 1;
  }
  return 0;
}

static XM_VOID *ISP_Run_SocNone(XM_VOID) {
  XM_U32 u32Val = 0;
  XM_U16 u16FrameCnt = 0;
  STABDEAL_S stStabDeal;

  ExtCfg_StabDeal(&stStabDeal);
  Write_IspReg(0x300F004C, 0);
  while (gu8LoopIsp) {
    u32Val = Read_IspReg(0x300F004C);
    if (u32Val == 1) {
      Write_IspReg(0x300F004C, 0);
      if (u16FrameCnt < 0xFFFF)
        u16FrameCnt++;

      XM_MPI_ISP_VsyncRun();
      aeAwbInfo_save();
      XM_MPI_ISP_Run(gIspDevApp);
      if (ispStab_Deal(&stStabDeal, u16FrameCnt)) {
        return XM_NULL;
      }
    }
  }
  return XM_NULL;
}

#endif

static XM_S32 ispSampleMain() {
  XM_S32 s32Ret;
  ISP_PUB_ATTR_S stPubAttr;
  // 5M
  if (gstProductInfo.u8RsltType == P5M_) {
    stPubAttr.u8FrameRate = (gstProductInfo.u8StdType == PALS) ? 25 : 30;
  }
  // 4M
  else if (gstProductInfo.u8RsltType == P4M_) {
#if (defined DEVTYPE_AHD)
    stPubAttr.u8FrameRate = (gstProductInfo.u8StdType == PALS) ? 25 : 30;
#else
    if (gstProductInfo.u32DSPType == DSP_XM550) {
      stPubAttr.u8FrameRate = (gstProductInfo.u8StdType == PALS) ? 25 : 30;
    } else {
      stPubAttr.u8FrameRate = 20;
    }
#endif
  }
  // 3M
  else if (gstProductInfo.u8RsltType == P1536_) {
#if (defined DEVTYPE_AHD)
    stPubAttr.u8FrameRate = (gstProductInfo.u8StdType == PALS) ? 25 : 30;
#else
    stPubAttr.u8FrameRate = 25;
#endif
  }
  // 2M/1M
  else {
    stPubAttr.u8FrameRate = (gstProductInfo.u8StdType == PALS) ? 25 : 30;
  }
  DEBUG("Vstd:%s[%dfps]\n",
        (gstProductInfo.u8StdType == NTSCS) ? "NTSC" : "PAL",
        stPubAttr.u8FrameRate);
  stPubAttr.enBayer = BAYER_RGGB;
  stPubAttr.u8SnsHDRmark = 0;
  stPubAttr.u8Vpixel = 0;
  stPubAttr.u8Upscaleren = 0;
  stPubAttr.u8SnsBwide = (XM_U8)gstProductInfo.SnsBwide;
  stPubAttr.stVFAttr.opType = OP_TYPE_AUTO;
  s32Ret = ExtCft_GetIspWndRect(stPubAttr.u8FrameRate, &(stPubAttr.stWndRect));

#if (defined SOC_SYSTEM) || (defined SOC_ALIOS)
  if (s32Ret != XM_SUCCESS) {
    GetIspWndRect(stPubAttr.u8FrameRate, &(stPubAttr.stWndRect));
  }

  if (gstParamIn.u8Mode) {
    VI_CHN_ATTR_S stAttr;
    VI_EXT_CHN_ATTR_S stViExAttr;
    stAttr.stCapRect.s32X = stPubAttr.stWndRect.s32X;
    stAttr.stCapRect.s32Y = stPubAttr.stWndRect.s32Y;
    stAttr.stCapRect.u32Height = stPubAttr.stWndRect.u32Height;
    stAttr.stCapRect.u32Width = stPubAttr.stWndRect.u32Width;

    switch (gstProductInfo.u8RsltType) {
    case P1080_:
      stAttr.stDestSize.u32Width = 1920;
      stAttr.stDestSize.u32Height = 1080;
      break;
#if 0
			case P1536_:	stAttr.stDestSize.u32Width = 2048;
						stAttr.stDestSize.u32Height = 1536;
						break;
#else
    case P1536_:
      stAttr.stDestSize.u32Width = 2304;
      stAttr.stDestSize.u32Height = 1296;
      break;
#endif
    case P960_:
      stAttr.stDestSize.u32Width = 1280;
      stAttr.stDestSize.u32Height = 960;
      break;
    case P4M_:
      stAttr.stDestSize.u32Width = 2560;
      stAttr.stDestSize.u32Height = 1440;
      break;
    case P5M_:
      stAttr.stDestSize.u32Width = 2600;
      stAttr.stDestSize.u32Height = 1954;
      break;
    default:
      stAttr.stDestSize.u32Width = 1280;
      stAttr.stDestSize.u32Height = 720;
      break;
    }
#if 1 // 1920*520*60fps
    if (stAttr.stCapRect.u32Height < 960) {
      stAttr.stDestSize.u32Height = (stAttr.stCapRect.u32Height >> 3) << 3;
    }
#endif
    stViExAttr.stDestSize.u32Height = 576;
    stViExAttr.stDestSize.u32Width = 704;

    XM_MPI_VI_SetChnAttr(0, &stAttr);
    XM_MPI_VI_SetExtChnAttr(1, &stViExAttr);
    XM_MPI_VI_SetExtChnAttr(2, &stViExAttr);
    XM_MPI_VI_EnableChn(0);
    XM_MPI_VI_EnableChn(1);
    XM_MPI_VI_EnableChn(2);
    Write_IspReg(0x30078008, 1);
    Write_IspReg(0x30078014, 1);
    Write_IspReg(0x30078020, 1);
#if (defined SNS1_XM330Y)
    stViExAttr.stDestSize.u32Width = 1920;  // 1280
    stViExAttr.stDestSize.u32Height = 1080; // 720
    XM_MPI_VI_SetExtChnAttr(3, &stViExAttr);
    XM_MPI_VI_EnableChn(3);
#endif
  }
#endif
  s32Ret = XM_MPI_ISP_MemInit(gIspDevApp);
  if (s32Ret != XM_SUCCESS) {
    ERR("XM_MPI_ISP_MemInit failed!\n");
    return XM_FAILURE;
  }
  s32Ret = sensor_register_callback(gIspDevApp);
  if (s32Ret != XM_SUCCESS) {
    ERR("sensor_register_callback failed!\n");
    return XM_FAILURE;
  }
  s32Ret = XM_MPI_ISP_SetPubAttr(gIspDevApp, &stPubAttr);
  if (s32Ret != XM_SUCCESS) {
    ERR("XM_MPI_ISP_SetPubAttr failed!\n");
    return XM_FAILURE;
  }
  VI_WinSet(1, 0, 0, 0, 0);
  s32Ret = XM_MPI_ISP_Init(gIspDevApp);
  if (s32Ret != XM_SUCCESS) {
    ERR("XM_MPI_ISP_Init failed!\n");
    return XM_FAILURE;
  }
  ExtCfg_IspDataInit(gstProductInfo.u8Encoder, gstProductInfo.u8StdType - 1);
  ExtCfg_PN_IspSet(gstProductInfo.u8Encoder, gstProductInfo.u8StdType - 1);
#ifdef SOC_NONE
  XM_MPI_ISP_Run(gIspDevApp);
#endif
  XM_MPI_MIPI_SetDevAttr(0, MIPI_SET_RUN, NULL); // 如果DVP 无操作
  Write_IspReg(FRONT_V_VIE_EN, 0x1);
  camera_set_vstd(0, (XM_U32)(gstProductInfo.u8StdType));
  return XM_SUCCESS;
}

static void reset_stop() {
#ifdef SOC_NONE
  XM_MPI_ISP_SetRegister(0x2000011C, 0x01); // ISP
  XM_MPI_ISP_SetRegister(0x200000E4, 0x01); // TIMER34
  XM_MPI_ISP_SetRegister(0x200000A4, 0x01); // I2C0
#else
  if (gstParamIn.u8Mode) {
    // VideoStore
    XM_MPI_ISP_SetRegister(0x30084034, 0x00);
    XM_MPI_ISP_SetRegister(0x30084038, 0x00);
    XM_MPI_ISP_SetRegister(0x30084008, 0x00);
  }
#endif
}

static void RomRsltInfo_Print() {
#ifdef SOC_SYSTEM
  XM_U32 u32Tmp, u32Addr;
  XM_U16 u16Tmp;
  if (ExtCfg_BaseAddrGet(0, ROM_FUN_AEAWBINFO, &u32Addr) == XM_SUCCESS) {
    printf("AeAwbInfo: %d\n", u32Addr);
    ExtCfg_Read_V2(4, u32Addr + 1, (XM_U8 *)&u32Tmp);
    printf("Exp: %d\n", u32Tmp);
    ExtCfg_Read_V2(4, u32Addr + 5, (XM_U8 *)&u32Tmp);
    printf("Shut: %d\n", u32Tmp);
    ExtCfg_Read_V2(4, u32Addr + 9, (XM_U8 *)&u32Tmp);
    printf("Again: %d\n", u32Tmp);
    ExtCfg_Read_V2(4, u32Addr + 13, (XM_U8 *)&u32Tmp);
    printf("Dgain: %d\n", u32Tmp);
    ExtCfg_Read_V2(4, u32Addr + 17, (XM_U8 *)&u32Tmp);
    printf("IspDgain: %d\n", u32Tmp);

    ExtCfg_Read_V2(2, u32Addr + 37, (XM_U8 *)&u16Tmp);
    printf("RGain: %d\n", u16Tmp);
    ExtCfg_Read_V2(2, u32Addr + 39, (XM_U8 *)&u16Tmp);
    printf("GGain: %d\n", u16Tmp);
    ExtCfg_Read_V2(2, u32Addr + 41, (XM_U8 *)&u16Tmp);
    printf("BGain: %d\n", u16Tmp);
    ExtCfg_Read_V2(2, u32Addr + 43, (XM_U8 *)&u16Tmp);
    printf("CT: %d\n", u16Tmp);
  }

  if (ExtCfg_BaseAddrGet(0, ROM_FUN_STABDEAL, &u32Addr) == XM_SUCCESS) {
    u16Tmp = 0;
    ExtCfg_Read_V2(1, u32Addr, (XM_U8 *)&u16Tmp);
    printf("size: %d\n", u16Tmp);
    u16Tmp = 0;
    ExtCfg_Read_V2(1, u32Addr + 1, (XM_U8 *)&u16Tmp);
    printf("Deal: %d\n", u16Tmp);
    u16Tmp = 0;
    ExtCfg_Read_V2(2, u32Addr + 2, (XM_U8 *)&u16Tmp);
    printf("FrameNum: %d\n", u16Tmp);
    u16Tmp = 0;
    ExtCfg_Read_V2(2, u32Addr + 4, (XM_U8 *)&u16Tmp);
    printf("StabFrameID: %d\n", u16Tmp);
  }
#endif
}

static XM_S32 SocSystem_Init() {
#if (defined SOC_SYSTEM) && ((!defined SOC_XMSDK) && (!defined SOC_ALIOS))
  XM_S32 s32Ret;
  ispSignalInit();
  if ((Create_Isplock()) == 0) {
    ERR("Isp is Running ,Exit!\n");
    return -1;
  }
  if (gstParamIn.u8Mode) {
    // Open_IspDev();
    s32Ret = XM_MPI_SYS_Init();
    if (s32Ret != XM_SUCCESS) {
      ERR("XM_MPI_SYS_Init failed!\n");
      return -1;
    }
  }
#endif
  return 0;
}

#if (defined DEVTYPE_AHD)
static XM_S32 VencTX_Set(XM_U8 u8VencTypeTx, XM_U8 u8StdType) {
  if (u8VencTypeTx == TXVENC_BUTT) {
    return XM_FAILURE;
  }
  XM_U32 u32Val;
  XM_MPI_ISP_GetRegister(ANALOG_CTRL, &u32Val);
  XM_MPI_ISP_SetRegister(ANALOG_CTRL, u32Val | 0x01); // Enable DAC

  if (ExtCfg_GetExtCfgFlg()) {
    if (ExtCfg_VencSet(0) != 1) {
      XM_MPI_VENCTX_SetMode(u8VencTypeTx, u8StdType, gstProductInfo.u8RsltType);
    }
    ExtCfg_VencSet(1 << (u8VencTypeTx * 2 + u8StdType - 1));
  }
  sensor_set_encoderinfo(&u8VencTypeTx);
  return XM_SUCCESS;
}
#endif
static XM_S32 Isp_Run() {
#if (defined SOC_SYSTEM) || (defined SOC_ALIOS)
// 1. Creat camera_setting
#if (defined DEVTYPE_IPC) && ((!defined SOC_XMSDK) && (!defined SOC_ALIOS)) && \
    (!defined SNS1_XM330Y)

#else
  camera_set_ircut_mode(0); // 红外灯同步
  CAMERA_COLOR stColor;
  stColor.Acutance = 8;
  stColor.Brightness = 50;
  stColor.Contrast = 50;
  stColor.Saturation = 50;
  stColor.Hue = 50;
  camera_set_color(0, &stColor);
#endif
  // 2. Run ISP
  if (0 !=
      pthread_create(&gIspPid, 0, (void *(*)(void *))ISP_Run_SocSystem, NULL)) {
    ERR("pthread_create XM_MPI_ISP_Run failed!\n");
    return XM_FAILURE;
  }
#else
  ISP_Run_SocNone();
#endif
  gu8IspOk = 1;
#if (!defined DEVTYPE_IPC)
  if (pthread_create(&gIspDebugCom, NULL, (void *(*)(void *))Com_process,
                     NULL)) {
    ERR("error creating Com_process thread\n");
    return 0;
  }
#endif
  return XM_SUCCESS;
}

static void usagePrint(XM_U8 u8PrintEn) {
#if (!defined SOC_XMSDK) && (!defined SOC_ALIOS)
  static XM_U8 su8PrintOk = 0;
  if ((su8PrintOk < 1) && u8PrintEn) {
    printf("usage: ./isp -m [0/1] -r [0/1/2...] -v [p/n]: ./isp -m 0 -r 0 -v p "
           "-e 0\n");
    printf("\t-m\t--- mode:0 VI/SYS set in Sofia   1: VI/SYS set in ISP\r\n");
    printf("\t-r\t--- resolution: 0:720P 1:1080P 2:960P 3:3M 4:4M 5:5M\r\n");
    printf("\t-v\t--- vstd: p:PAL n:NTSC f:15fps\r\n");
    printf("\t-e\t--- ipc encoder: (1:H.265  other:H.264)\r\n");
    printf("\t-i\t--- sensor vi interface: (0:by src 1:DVP 2:MIPI)\r\n");
  }
  if (u8PrintEn)
    su8PrintOk = 1;
#endif
}
static int usage(int argc, char *argv[]) {
  gstParamIn.u8Vstd = VSTDNULL;
  gstParamIn.enRslt = PNULL;
  gstParamIn.u8IpcVenc = IPC_Hx264;
  gstParamIn.u8SnsInterface = 0;
  DEBUG("[ISP_SDK]BUILD TIME:"__DATE__
        " "__TIME__
        "\r\n");
#ifdef OSC_24M
  DEBUG("OSC:24M\n");
#else
  DEBUG("OSC:12M\n");
#endif
#if (defined SOC_SYSTEM) || (defined SOC_ALIOS)
  gstParamIn.u8Mode = 0; // 1:默认支持独立跑
#if (defined DEVTYPE_IPC) || (defined SOC_XMSDK)
  gstParamIn.u8Mode = 0;
#else
  // gstParamIn.u8Mode = 1;
  ;
#endif
  if (argc < 2) {
    usagePrint(1);
  }
  int s32Tmp, pIdx = 1;
  int argcTmp = (argc >= 1) ? argc - 1 : 0;
  while (argcTmp > 0) {
    if (*(argv[pIdx] + 0) != '-') {
      goto FAILED_RET;
    }
    switch (*(argv[pIdx] + 1)) {
    case 'M':
    case 'm':
      if (--argcTmp < 1) {
        goto FAILED_RET;
      }
      pIdx += 1;
      gstParamIn.u8Mode = atoi(argv[pIdx]);
      argcTmp -= 1;
      pIdx += 1;
      break;
    case 'R':
    case 'r':
      if (--argcTmp < 1) {
        goto FAILED_RET;
      }
      pIdx += 1;
      gstParamIn.enRslt = atoi(argv[pIdx]);
      argcTmp -= 1;
      pIdx += 1;
      break;
    case 'V':
    case 'v':
      if (--argcTmp < 1) {
        goto FAILED_RET;
      }
      pIdx += 1;
      if ((*argv[pIdx] == 'n') || (*argv[pIdx] == 'N')) {
        gstParamIn.u8Vstd = NTSCS;
      } else if ((*argv[pIdx] == 'p') || (*argv[pIdx] == 'P')) {
        gstParamIn.u8Vstd = PALS;
      } else if ((*argv[pIdx] == 'f') || (*argv[pIdx] == 'F')) {
        gstParamIn.u8Vstd = NRT15;
      }
      argcTmp -= 1;
      pIdx += 1;
      break;
    case 'C':
    case 'c':
      if (--argcTmp < 1) {
        goto FAILED_RET;
      }
      pIdx += 1;
      Mipi_Check = atoi(argv[pIdx]);
      printf("Mipi_Check = %d\n", Mipi_Check);
      argcTmp -= 1;
      pIdx += 1;
      break;
    case 'e':
    case 'E':
      if (--argcTmp < 1) {
        goto FAILED_RET;
      }
      pIdx += 1;
      s32Tmp = atoi(argv[pIdx]);
      if (s32Tmp == 1) {
        gstParamIn.u8IpcVenc = IPC_Hx265;
      }
      argcTmp -= 1;
      pIdx += 1;
      break;
    case 'i':
    case 'I':
      if (--argcTmp < 1) {
        goto FAILED_RET;
      }
      pIdx += 1;
      s32Tmp = atoi(argv[pIdx]);
      s32Tmp = CLIP3(s32Tmp, 0, 2);
      gstParamIn.u8SnsInterface = (XM_U8)s32Tmp;
      argcTmp -= 1;
      pIdx += 1;
      break;
    default:
      goto FAILED_RET;
      break;
    }
  }
  printf("SYS/VI set in\t[ %s ]\n", (gstParamIn.u8Mode) ? "isp" : "Sofia");
  printf("Vstd_init\t[ %s ]\n", (gstParamIn.u8Vstd == NTSCS) ? "NTSC" : "PAL");
  if (gstParamIn.enRslt != PNULL) {
    printf("Rslt\t[ %d ]\n", gstParamIn.enRslt);
  }
  printf("IPC_Venc:\t[ %s ]\n",
         (gstParamIn.u8IpcVenc == IPC_Hx265) ? "H.265" : "H.264");
  printf("Sns_IF:\t[ %s ]\n",
         (gstParamIn.u8SnsInterface == 0)
             ? "By src"
             : ((gstParamIn.u8SnsInterface == 1) ? "DVP" : "MIPI"));
  printf("\n");
  return 0;
FAILED_RET:
  ERR("argvIdx[%d] argc remain[%d] err!\n", pIdx, argcTmp);
  usagePrint(1);
  return -1;
#else
  gstParamIn.u8Mode = 0;
  return 0;
#endif
}

#if 0
static XM_S32 lcd_init() {
#ifdef TWOCHANNEL
  int i = 0;
  int ret = 0;
  char *y_addr = NULL;
  char *uv_addr = NULL;
  int s32Ret = 0;
  MPP_CHN_S stSrcChn;
  MPP_CHN_S stDestChn;

  // XM_MPI_SYS_Init();
  // system("isp &");
  // system("tp2823_app &");

  // sleep(5);

  VI_CHN_ATTR_S stAttr;
  VI_EXT_CHN_ATTR_S stExtAttr;
  stAttr.stCapRect.s32X = 2;
  stAttr.stCapRect.s32Y = 2;
  stAttr.stCapRect.u32Width = 1280;
  stAttr.stCapRect.u32Height = 720;
  stAttr.stDestSize.u32Width = 1280;
  stAttr.stDestSize.u32Height = 720;
  XM_MPI_VI_SetChnAttr(0, &stAttr);
  XM_MPI_VI_EnableChn(0);

  stExtAttr.stDestSize.u32Width = 480;
  stExtAttr.stDestSize.u32Height = 272;
  XM_MPI_VI_SetExtChnAttr(1, &stExtAttr);
  XM_MPI_VI_EnableChn(1);

  VO_PUB_ATTR_S pubAttr;
  VO_CHN_ATTR_S chnAttr;

  pubAttr.u32BgColor = 0;
  pubAttr.enIntfType = VO_INTF_LCD;
  pubAttr.enIntfSync = VO_OUTPUT_480x272_60;
  XM_MPI_VO_SetPubAttr(0, &pubAttr);

  chnAttr.u32Priority = 0;
  chnAttr.stRect.s32X = 0;
  chnAttr.stRect.s32Y = 0;
  chnAttr.stRect.u32Width = 480;
  chnAttr.stRect.u32Height = 272;
  XM_MPI_VO_SetChnAttr(0, 0, &chnAttr);
  XM_MPI_VO_EnableChn(0, 0);

  stSrcChn.enModId = XM_ID_VIU;
  stSrcChn.s32DevId = 0;
  stSrcChn.s32ChnId = 1;
  stDestChn.enModId = XM_ID_VOU;
  stDestChn.s32DevId = 0;
  stDestChn.s32ChnId = 0;
  XM_MPI_SYS_Bind(&stSrcChn, &stDestChn);
#endif
  return 0;
}

static void testPatter(XM_U8 u8Rslt, XM_U8 u8Vstd) {
  XM_U32 u32TotalSizeH, u32TotalSizeV, u32ActiveH, u32ActiveV;
  XM_U8 u8TstPatternEn = 0;
  if ((u8Rslt == 1) && (u8Vstd == NTSCS)) // 2M_NTSC
  {
    u32TotalSizeH = 2200;
    u32TotalSizeV = 1125;
    u32ActiveH = 1938;
    u32ActiveV = 1100;
    u8TstPatternEn = 1;
    DEBUG("testPatter_2M_NTSC~~~\n");
  } else if ((u8Rslt == 0) && (u8Vstd == PALS)) // 1M_PAL
  {
    if (gstProductInfo.u8SensorClk == SENSORCLK_37x125M) {
      u32TotalSizeH = 1980;
      u32TotalSizeV = 750;
      u32ActiveH = 1290;
      u32ActiveV = 730;
    } else {
      u32TotalSizeH = 1920;
      u32TotalSizeV = 750;
      u32ActiveH = 1290;
      u32ActiveV = 730;
    }
    u8TstPatternEn = 1;
    XM_MPI_ISP_SetRegister(0x30088010, 0x307);
    DEBUG("testPatter_1M_PAL~~~\n");
  } else if ((u8Rslt == 0) && (u8Vstd == NTSCS)) // 1M_NTSC
  {
    if (gstProductInfo.u8SensorClk == SENSORCLK_37x125M) {
      u32TotalSizeH = 1650;
      u32TotalSizeV = 750;
      u32ActiveH = 1290;
      u32ActiveV = 730;
    } else {
      u32TotalSizeH = 1600;
      u32TotalSizeV = 750;
      u32ActiveH = 1290;
      u32ActiveV = 730;
    }
    u8TstPatternEn = 1;
    XM_MPI_ISP_SetRegister(0x30088010, 0x307);
    DEBUG("testPatter_1M_NTSC~~~\n");
  }
  if (u8TstPatternEn) {
    XM_MPI_ISP_SetRegister(0x30000080, u32TotalSizeH);
    XM_MPI_ISP_SetRegister(0x30000084, u32TotalSizeV);
    XM_MPI_ISP_SetRegister(0x30000088, u32ActiveH);
    XM_MPI_ISP_SetRegister(0x3000008C, u32ActiveV);
    XM_MPI_ISP_SetRegister(0x30000000, 1);
  }
}
#endif

static int ispMsgInit(XM_VOID) {
#if ((defined SOC_SYSTEM) || (defined SOC_ALIOS))
#if (defined DEVTYPE_IPC) && ((!defined SOC_XMSDK) && (!defined SOC_ALIOS))
  //创建一个消息处理线程，接收库函数中发过来的消息
  gu8IspOk = 0;
  if (pthread_create(&gIspDebug, NULL, (void *(*)(void *))ispMsgReceive,
                     NULL)) {
    ERR("error creating ispMsgReceive thread\n");
    return -1;
  }
  if (pthread_create(&gIspTool, NULL, (void *(*)(void *))toolMsgReceive,
                     NULL)) {
    ERR("error creating toolMsgReceive thread\n");
    return -1;
  }
  DEBUG("msg init ok!\n");
#endif
#endif
  return 0;
}

#if 0
// u8Mode(0:Start 1:End)
XM_S32 showT(XM_U8 u8Mode) {
  static XM_U8 su8First = 1;
  static struct timeval first_time;
  struct timeval second_time;
  if (u8Mode == 0) {
    gettimeofday(&first_time, NULL);
  } else if ((u8Mode == 2)) {
    if (su8First) {
      su8First = 0;
      gettimeofday(&second_time, NULL);
      printf("%d T: %d us~~~\n", u8Mode,
             (second_time.tv_sec - first_time.tv_sec) * 1000000 +
                 second_time.tv_usec - first_time.tv_usec);
    }
  } else {
    gettimeofday(&second_time, NULL);
    printf("%d T: %d us~~~\n", u8Mode,
           (second_time.tv_sec - first_time.tv_sec) * 1000000 +
               second_time.tv_usec - first_time.tv_usec);
  }
  return XM_SUCCESS;
}
#endif

#if (defined SOC_XMSDK) || (defined SOC_ALIOS)
XM_S32 SysIspRun_Exit(XM_VOID) {
  if (gIspPid) {
    pthread_join(gIspPid, NULL);
  }

  if (gIspDebugCom) {
    pthread_join(gIspDebugCom, NULL);
  }
  return XM_SUCCESS;
}

int ispApp(int argc, char *argv[])
#else
int main(int argc, char *argv[])
#endif
{
  XM_U8 *pu8Buff = 0x00000000;
  ispMsgInit();
  if (usage(argc, argv) < 0) {
    goto RET;
  }
  if (SocSystem_Init() != 0) {
    goto RET;
  }
  reset_stop();
  GPIO_Init();
  I2C_Init();
  pu8Buff = Get_CfgBufferAddr();
  if (ExtCfg_Init(pu8Buff) != XM_SUCCESS) {
    goto RET;
  }
  RomRsltInfo_Print();
  GetProductInfo();
#if (defined DEVTYPE_AHD)
  VencTX_Set(gstProductInfo.u8Encoder, gstProductInfo.u8StdType);
#endif
  if (ispSampleMain() != XM_SUCCESS) {
    goto RET;
  }
  lumaVideo_Set(0, 1280, 720);
  DEBUG("Init over ~~~~~~~~\n");
  // lcd_init();
#if (!(defined DEVTYPE_AHD))
  if (Mipi_Check == 1) {
    Mipi_check();
    SysRegExit();
    return 0;
  }
#endif
  Isp_Run();
  // camera_set_blc_v2(1,25);  测试背光补偿
RET:
  DEBUG("isp_sample end!\n");
#if 0 // 移动侦测测试示例
#if (!defined SOC_XMSDK)
  camera_set_dnc_mode(3); // 智能警戒(双光、软光敏)
  while (gu8LoopIsp) {
    vda_sample();
    sleep(1);
  }
#endif
#endif
  SysRegExit();
  return 0;
}
