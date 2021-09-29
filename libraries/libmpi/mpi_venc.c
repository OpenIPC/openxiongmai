#define _POSIX_C_SOURCE 199506L
#include <signal.h>

#include <stdio.h>
#include <pthread.h>
#include <string.h>

#include "re_comm_venc.h"
#include "re_type.h"
#include "re_venc.h"
#include "re_vi.h"

static VENC_CHN_ATTR_S chn_attrs[VENC_MAX_CHN_NUM];
static VENC_PARAM_REF_EX_S chn_ref_ex[VENC_MAX_CHN_NUM];
static VENC_CHN_PARAM_S chn_params[VENC_MAX_CHN_NUM];

#define IS_H264(chn) (BITMSK(chn) & h264_msk_chn)
#define IS_JPEG(chn) (BITMSK(chn) & jpeg_msk_chn)
static int h264_msk_chn;
static int jpeg_msk_chn;

int venc_ioctl(XM_U32 cmd, void *arg);

int XM_MPI_VENC_Init() {
  sigset_t set;

  sigemptyset(&set);
  sigaddset(&set, SIGIO);
  pthread_sigmask(SIG_BLOCK, &set, NULL);

  return XM_SUCCESS;
}

XM_S32 XM_MPI_VENC_CreateChn(VENC_CHN VeChn, const VENC_CHN_ATTR_S *pstAttr) {
  struct {
    VENC_CHN chn;
    const VENC_CHN_ATTR_S *attr;
  } arg = {
      .chn = VeChn,
      .attr = pstAttr,
  };

  if (VeChn < VENC_MAX_CHN_NUM) {
    memcpy(&chn_attrs[VeChn], pstAttr, sizeof(VENC_CHN_ATTR_S));
    PAYLOAD_TYPE_E payload = pstAttr->stVeAttr.enType;
    if (payload == PT_H264) {
      h264_msk_chn |= BITMSK(VeChn);
      XM_S32 ret = venc_ioctl(0x40084501, &arg);
      VENC_CHN_PARAM_S pstParam = {
          .u32QuarterPixelMv = 2,
          .u32MinIQp = 0,
      };
      XM_MPI_VENC_SetH264Param(VeChn, &pstParam);
      XM_MPI_VI_SetFrmRate(VeChn, pstAttr->stRcAttr.stAttrH264Cbr.u32SrcFrmRate,
                           pstAttr->stRcAttr.stAttrH264Cbr.fr32DstFrmRate);
      return ret;
    }
    if (payload == PT_JPEG) {
      jpeg_msk_chn |= BITMSK(VeChn);
      return venc_ioctl(0x40084502, &arg);
    }
  } else {
    printf("chn %d not support, max is %d", VeChn, VENC_MAX_CHN_NUM - 1);
  }
  return XM_FAILURE;
}

XM_S32 XM_MPI_VENC_DestroyChn(VENC_CHN VeChn) {
  VENC_CHN *chn;
  int cmd;

  if (VeChn >= VENC_MAX_CHN_NUM) {
    printf("chn %d not support, max is %d", VeChn, VENC_MAX_CHN_NUM - 1);
    return XM_FAILURE;
  }

  chn = &VeChn;
  if (IS_H264(VeChn)) {
    cmd = 0x4517;
  } else {
    if (!IS_JPEG(VeChn))
      return XM_FAILURE;
    cmd = 0x4518;
  }
  return venc_ioctl(cmd, chn);
}

XM_S32 XM_MPI_VENC_GETRefParamEx(VENC_CHN VeChn,
                                 VENC_PARAM_REF_EX_S *pstRefParam) {
  if (VeChn < VENC_MAX_CHN_NUM) {
    memcpy(pstRefParam, &chn_ref_ex[VeChn], sizeof(VENC_PARAM_REF_EX_S));
    return XM_SUCCESS;
  } else {
    printf("chn %d not support, max is %d", VeChn, VENC_MAX_CHN_NUM - 1);
    return XM_FAILURE;
  }
}

XM_S32 XM_MPI_VENC_GetChnAttr(VENC_CHN VeChn, VENC_CHN_ATTR_S *pstAttr) {
  XM_S32 result;

  if (VeChn < VENC_MAX_CHN_NUM) {
    memcpy(pstAttr, &chn_attrs[VeChn], sizeof(VENC_CHN_ATTR_S));
    result = XM_SUCCESS;
  } else {
    printf("chn %d not support, max is %d", VeChn, VENC_MAX_CHN_NUM - 1);
    result = XM_FAILURE;
  }
  return result;
}

XM_S32 XM_MPI_VENC_GetFd(VENC_CHN VeChn) { return XM_SUCCESS; }

XM_S32 XM_MPI_VENC_GetH264Param(VENC_CHN VeChn, VENC_CHN_PARAM_S *pstParam) {
  if (VeChn < VENC_MAX_CHN_NUM) {
    memcpy(pstParam, &chn_params[VeChn], sizeof(VENC_CHN_PARAM_S));
    return XM_SUCCESS;
  } else {
    printf("chn %d not support, max is %d", VeChn, VENC_MAX_CHN_NUM - 1);
    return XM_FAILURE;
  }
}

XM_S32 XM_MPI_VENC_GetStream(VENC_CHN VeChn, VENC_STREAM_S *pstStream,
                             XM_BOOL bBlockFlag) {
  struct {
    VENC_CHN chn;
    XM_BOOL flag;
    VENC_STREAM_S *stream;
  } arg = {
      .chn = VeChn,
      .stream = pstStream,
      .flag = bBlockFlag != 0,
  };

  int cmd;

  if (VeChn < VENC_MAX_CHN_NUM) {
    if (IS_H264(VeChn)) {
      cmd = 0x400C4509;
    } else {
      if (!IS_JPEG(VeChn))
        return XM_FAILURE;
      cmd = 0x400C4511;
    }
    return venc_ioctl(cmd, &arg);
  }
  printf("chn %d not support, max is %d", VeChn, VENC_MAX_CHN_NUM - 1);
  return XM_FAILURE;
}

XM_S32 XM_MPI_VENC_Query(VENC_CHN VeChn, VENC_CHN_STAT_S *pstStat) {
  return XM_SUCCESS;
}

XM_S32 XM_MPI_VENC_ReleaseStream(VENC_CHN VeChn, VENC_STREAM_S *pstStream) {
  struct {
    VENC_CHN chn;
    int null;
    VENC_STREAM_S *stream;
  } arg = {
      .chn = VeChn,
      .stream = pstStream,
  };
  int cmd;

  if (VeChn < VENC_MAX_CHN_NUM) {
    if (IS_H264(VeChn)) {
      cmd = 0x400C4510;
    } else {
      if (!IS_JPEG(VeChn))
        return XM_FAILURE;
      cmd = 0x400C4512;
    }
    return venc_ioctl(cmd, &arg);
  }
  printf("chn %d not support, max is %d", VeChn, VENC_MAX_CHN_NUM - 1);
  return XM_FAILURE;
}

XM_S32 XM_MPI_VENC_RequestIDR(VENC_CHN VeChn) {
  if (VeChn < VENC_MAX_CHN_NUM) {
    if (IS_H264(VeChn))
      return venc_ioctl(0x40044515, &VeChn);
  } else {
    printf("chn %d not support, max is %d", VeChn, VENC_MAX_CHN_NUM - 1);
  }
  return XM_FAILURE;
}

XM_S32 XM_MPI_VENC_SelectData(void) { return venc_ioctl(0x4516, 0); }

XM_S32 XM_MPI_VENC_SetChnAttr(VENC_CHN VeChn, const VENC_CHN_ATTR_S *pstAttr) {
  if (VeChn >= VENC_MAX_CHN_NUM) {
    printf("chn %d not support, max is %d", VeChn, VENC_MAX_CHN_NUM - 1);
    return XM_FAILURE;
  }

  memcpy(&chn_attrs[VeChn], pstAttr, sizeof(VENC_CHN_ATTR_S));
  XM_MPI_VI_SetFrmRate(VeChn, pstAttr->stRcAttr.stAttrH264Cbr.u32SrcFrmRate,
                       pstAttr->stRcAttr.stAttrH264Cbr.fr32DstFrmRate);
  if (IS_H264(VeChn)) {
    struct {
      VENC_CHN chn;
      const VENC_CHN_ATTR_S *attr;
    } h264_arg = {
        .chn = VeChn,
        .attr = pstAttr,
    };
    return venc_ioctl(0x404C4513, &h264_arg);
  } else {
    if (!IS_JPEG(VeChn))
      return XM_FAILURE;
    return venc_ioctl(0x404C4514, &chn_attrs[VeChn]);
  }
}

XM_S32 XM_MPI_VENC_SetH264Param(VENC_CHN VeChn,
                                const VENC_CHN_PARAM_S *pstParam) {
  struct {
    VENC_CHN chn;
    const VENC_CHN_PARAM_S *param;
  } arg = {
      .chn = VeChn,
      .param = pstParam,
  };

  if (VeChn < VENC_MAX_CHN_NUM - 1) {
    memcpy(&chn_params[VeChn], pstParam, sizeof(VENC_CHN_PARAM_S));
    return venc_ioctl(0x40084519, &arg);
  } else {
    printf("chn %d not support, max is %d", VeChn, VENC_MAX_CHN_NUM - 1);
    return XM_FAILURE;
  }
}

XM_S32 XM_MPI_VENC_SetRefParamEx(VENC_CHN VeChn,
                                 VENC_PARAM_REF_EX_S *pstRefParam) {
  struct {
    VENC_CHN chn;
    VENC_PARAM_REF_EX_S *param;
  } arg = {
      .chn = VeChn,
      .param = pstRefParam,
  };

  if (VeChn < VENC_MAX_CHN_NUM) {
    memcpy(&chn_ref_ex[VeChn], pstRefParam, sizeof(VENC_PARAM_REF_EX_S));
    return venc_ioctl(0x40084520, &arg);
  } else {
    printf("chn %d not support, max is %d", VeChn, VENC_MAX_CHN_NUM - 1);
    return XM_FAILURE;
  }
}

XM_S32 XM_MPI_VENC_StartRecvPic(VENC_CHN VeChn) {
  int cmd;

  if (VeChn < VENC_MAX_CHN_NUM) {
    if (IS_H264(VeChn)) {
      cmd = 0x40044503;
    } else {
      if (!IS_JPEG(VeChn))
        return XM_FAILURE;
      cmd = 0x40044505;
    }
    return venc_ioctl(cmd, &VeChn);
  }
  printf("chn %d not support, max is %d", VeChn, VENC_MAX_CHN_NUM - 1);
  return XM_FAILURE;
}

XM_S32 XM_MPI_VENC_StartRecvPicEx(VENC_CHN VeChn,
                                  VENC_RECV_PIC_PARAM_S *pstRecvParam) {
  return XM_SUCCESS;
}
