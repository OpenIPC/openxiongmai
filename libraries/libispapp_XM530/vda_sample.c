#include "mpi_ae.h"
#include "mpi_isp.h"
#include "mpi_vdam.h"
#include "xm_math.h"
#include "xm_print.h"
#include "xm_sns_ctrl.h"
#include "xm_type.h"
#include <fcntl.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>

// 移动侦测
static int vda_movAttrSet() {
  XM_U8 u8i;
  XM_U8 u8Enable, u8Level, u8RatioThresh, u8Chn = 0;
  VDA_MOTION_MOVE_INIT stVdaMoveAttr;
  /*
  移动侦测: 	配置属性
  */
  // 7*7 窗口(对应bit为1时判断对应窗口)
  XM_U32 au32Win[18] = {0x7f, // 1th行(7列)
                        0x7f, // 2th行(7列)
                        0x7f, // 3th行(7列)
                        0x7f, // 4th行(7列)
                        0x7f, // 5th行(7列)
                        0x7f, // 6th行(7列)
                        0x7f, // 7th行(7列)
                        0x0,  0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
  u8Enable = 1;
  u8Level = 3;
  u8Level = CLIP3(u8Level, 1, 6); // 灵敏度
  u8RatioThresh = 2;
  u8RatioThresh = CLIP3(u8RatioThresh, 0, 100); // 触发比例阈值

  XM_MPI_VDA_GetMovAttr(u8Chn, &stVdaMoveAttr);
  stVdaMoveAttr.MoEnble = (u8Enable > 0) ? 1 : 0;
  stVdaMoveAttr.u8RatioThresh = u8RatioThresh;
  switch (u8Level) {
  case 1:
    stVdaMoveAttr.MoRatio = VDA_MOV_RAT1;
    break;
  case 2:
    stVdaMoveAttr.MoRatio = VDA_MOV_RAT2;
    break;
  case 4:
    stVdaMoveAttr.MoRatio = VDA_MOV_RAT4;
    break;
  case 5:
    stVdaMoveAttr.MoRatio = VDA_MOV_RAT5;
    break;
  case 6:
    stVdaMoveAttr.MoRatio = VDA_MOV_RAT6;
    break;
  case 3:
  default:
    stVdaMoveAttr.MoRatio = VDA_MOV_RAT3;
    break;
  }
  stVdaMoveAttr.MoDframe = 20; //间隔帧
  for (u8i = 0; u8i < 18; u8i++) {
    stVdaMoveAttr.MoWinset[u8i] = au32Win[u8i];
  }
  if (XM_MPI_VDA_SetMovAttr(0, &stVdaMoveAttr) == XM_FAILURE) {
    ERR("XM_MPI_VDA_SetMovAttr failed!\n");
    return XM_FAILURE;
  }
  return XM_SUCCESS;
}

// 视频遮挡
static int vda_shiAttrSet(void) {
  XM_U8 s32Ret, Shratio;
  XM_U32 ShAlarmD, ShAlarmAf;
  XM_U8 u8Enable, u8Level;
  VDA_MOTION_SHELTER_INIT stVdaShiAttr;
  /*
  视频遮挡:	配置属性
  */
  u8Enable = 1;
  u8Level = 3;
  u8Level = CLIP3(u8Level, 1, 6);

  if (XM_MPI_VDA_GetShAttr(0, &stVdaShiAttr) != XM_SUCCESS) {
    ERR("XM_MPI_VDA_GetShAttr failed!\n");
    return XM_FAILURE;
  }
  stVdaShiAttr.ShLevel = u8Level;
  stVdaShiAttr.ShEnble = u8Enable;
  switch (stVdaShiAttr.ShLevel) {
  case XM_VDA_LEV1:
    Shratio = VDA_SHL_RAT1;
    ShAlarmAf = 16000;
    ShAlarmD = 160000;
    break;
  case XM_VDA_LEV2:
    Shratio = VDA_SHL_RAT2;
    ShAlarmAf = 18000;
    ShAlarmD = 180000;
    break;
  case XM_VDA_LEV3:
    Shratio = VDA_SHL_RAT3;
    ShAlarmAf = 20000;
    ShAlarmD = 200000;
    break;
  case XM_VDA_LEV4:
    Shratio = VDA_SHL_RAT4;
    ShAlarmAf = 26000;
    ShAlarmD = 220000;
    break;
  case XM_VDA_LEV5:
    Shratio = VDA_SHL_RAT5;
    ShAlarmAf = 32000;
    ShAlarmD = 260000;
    break;
  case XM_VDA_LEV6:
    Shratio = VDA_SHL_RAT6;
    ShAlarmAf = 38000;
    ShAlarmD = 300000;
    break;
  default:
    Shratio = VDA_SHL_RAT3;
    ShAlarmAf = 20000;
    ShAlarmD = 200000;
    break;
  }
  stVdaShiAttr.ShRatio = Shratio;
  stVdaShiAttr.ShAlarmAf = ShAlarmAf;
  stVdaShiAttr.ShAlarmD = ShAlarmD;
  stVdaShiAttr.ShDframe = 20; //间隔帧
  s32Ret = XM_MPI_VDA_SetShAttr(0, &stVdaShiAttr);
  if (s32Ret == XM_FAILURE) {
    ERR("XM_MPI_VDA_SetShAttr failed!\n");
    return XM_FAILURE;
  }
  return XM_SUCCESS;
}

static XM_S32 vda_init() {
  XM_U8 vdaChn = 0;
  XM_MPI_VDA_CreatChn(vdaChn);
  // 移动侦测
  vda_movAttrSet();
  // 视频遮挡
  vda_shiAttrSet();
  // Destory
  // XM_MPI_VDA_DestroyChn(vdaChn);

  return XM_SUCCESS;
}

XM_S32 vda_sample() {
  static XM_U8 su8First = 1;
  XM_S32 s32Ret;
  if (su8First) {
    su8First = 0;
    vda_init();
  }
  // 移动侦测结果
  VDA_MOTION_MOVE_RESULT stVdaMoveRslt;
  s32Ret = XM_MPI_VDA_GetMovData(0, &stVdaMoveRslt);
  if (s32Ret == XM_FAILURE) {
    ERR("XM_MPI_VDA_GetMovData failed!\n");
    return XM_FAILURE;
  }
  if (stVdaMoveRslt.MoReflag) {
    DEBUG("[VDA] motion detection: Flag[%d] ratio[%d]\n",
          stVdaMoveRslt.MoReflag, s32Ret);
  } else {
    printf("\n");
  }
  // 视频遮挡结果
  XM_U8 u8VdaShiRslt;
  XM_MPI_VDA_GetShdMark(0, &u8VdaShiRslt);
  // printf("[VDA] Video shade: %d \n", u8VdaShiRslt);

  return XM_SUCCESS;
}
