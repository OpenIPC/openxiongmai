#include "re_common.h"
#include "re_type.h"
#include <re_vi.h>

#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>

#define FATAL(msg)                                                             \
  fprintf(stderr, "vi - %s(%d): " msg "\n", __FUNCTION__, __LINE__)
#define FATAL_ARG(msg, ...)                                                    \
  fprintf(stderr, "vi - %s(%d): " msg "\n", __FUNCTION__, __LINE__, __VA_ARGS__)

static int fd_vi = -1;

XM_S32 XM_MPI_VI_INIT(void) {
  if (fd_vi != -1 || (fd_vi = open("/dev/vi", 0), fd_vi >= 0)) {
    ioctl(fd_vi, 0x5606u, 0);
    return XM_SUCCESS;
  }

  FATAL("open error");
  return XM_SUCCESS;
}

XM_S32 XM_MPI_VI_SetChnAttr(VI_CHN ViChn, const VI_CHN_ATTR_S *pstAttr) {
  if (ViChn > 0) {
    FATAL_ARG("not support ViChn = %d, fd_vi = %d", ViChn, fd_vi);
    return XM_FAILURE;
  }

  if (fd_vi != -1 || (fd_vi = open("/dev/vi", 0), fd_vi >= 0)) {
    struct {
      XM_U32 left, top;
      XM_U32 right, bottom;
    } cap = {
        .left = pstAttr->stCapRect.s32X,
        .top = pstAttr->stCapRect.s32Y,
        .right = pstAttr->stCapRect.s32X + pstAttr->stCapRect.u32Width,
        .bottom = pstAttr->stCapRect.s32Y + pstAttr->stCapRect.u32Height,
    };

    ioctl(fd_vi, 0x40105600u, &cap);

    struct {
      VI_CHN chn;
      XM_U32 width;
      XM_U32 heght;
    } size = {
        .chn = ViChn,
        .width = pstAttr->stDestSize.u32Width,
        .heght = pstAttr->stDestSize.u32Height,
    };
    ioctl(fd_vi, 0x400C5601u, &size);

    return XM_SUCCESS;
  }

  FATAL("open error");
  return XM_FAILURE;
}

XM_S32 XM_MPI_VI_GetChnAttr(VI_CHN ViChn, VI_CHN_ATTR_S *pstAttr) {
  return XM_SUCCESS;
}

XM_S32 XM_MPI_VI_EnableChn(VI_CHN ViChn) {
  if (ViChn >= VENC_MAX_CHN_NUM) {
    FATAL_ARG("not support ViChn = %d, fd_vi = %d", ViChn, fd_vi);
    return XM_FAILURE;
  }

  if (fd_vi != -1 || (fd_vi = open("/dev/vi", 0), fd_vi >= 0)) {
    ioctl(fd_vi, 0x40045602u, &ViChn);
    return XM_SUCCESS;
  }

  FATAL("open error");
  return XM_FAILURE;
}

XM_S32 XM_MPI_VI_DisableChn(VI_CHN ViChn) {
  if (ViChn >= VENC_MAX_CHN_NUM) {
    FATAL_ARG("not support ViChn = %d, fd_vi = %d", ViChn, fd_vi);
    return XM_FAILURE;
  }

  if (fd_vi != -1 || (fd_vi = open("/dev/vi", 0), fd_vi >= 0)) {
    ioctl(fd_vi, 0x40045603u, &ViChn);
    return XM_SUCCESS;
  }

  FATAL("open error");
  return XM_FAILURE;
}

/*
 * Frame record of xm5x0_vi.ko: chn, pool, luma and chroma phys, width and
 * height, plus a 64-bit PTS on XM530. That makes 32 bytes on XM530
 * (0xC0205604 get, 0x40205605 release) and 24 on XM510 (0xC0185604,
 * 0x40185605). The size that works is remembered.
 */
typedef struct {
  VI_CHN chn;
  XM_U32 u32PoolId;
  XM_U32 u32PhyAddr_0;
  XM_U32 u32PhyAddr_1;
  XM_U32 u32Width;
  XM_U32 u32Height;
  XM_U64 u64Pts; /* XM530 only */
} vi_frame_t;

#define VI_CMD(dir, nr, size)                                                  \
  (((XM_U32)(dir) << 30) | ((size) << 16) | ('V' << 8) | (nr))
#define VI_GET_FRAME 4
#define VI_RELEASE_FRAME 5

static const XM_U32 vi_frame_sizes[] = {sizeof(vi_frame_t),
                                        sizeof(vi_frame_t) - sizeof(XM_U64)};
static int vi_abi = -1;

XM_S32 XM_MPI_VI_GetFrame(VI_CHN ViChn, VIDEO_FRAME_INFO_S *pstFrameInfo) {
  vi_frame_t arg;
  int i;

  if (ViChn >= VENC_MAX_CHN_NUM) {
    FATAL_ARG("not support ViChn = %d, fd_vi = %d", ViChn, fd_vi);
    return XM_FAILURE;
  }

  if (fd_vi == -1 && (fd_vi = open("/dev/vi", 0), fd_vi < 0)) {
    FATAL("open error");
    return XM_FAILURE;
  }

  /* No frame yet and an unknown record size both fail, so the size is
   * only settled by a call that returns a frame. */
  for (i = vi_abi < 0 ? 0 : vi_abi; i < 2; i++) {
    memset(&arg, 0, sizeof(arg));
    arg.chn = ViChn;
    if (ioctl(fd_vi, VI_CMD(3, VI_GET_FRAME, vi_frame_sizes[i]), &arg) == 0 &&
        arg.u32PhyAddr_0) {
      vi_abi = i;
      pstFrameInfo->u32PoolId = arg.u32PoolId;
      pstFrameInfo->stVFrame.u32PhyAddr[0] = arg.u32PhyAddr_0;
      pstFrameInfo->stVFrame.u32PhyAddr[1] = arg.u32PhyAddr_1;
      pstFrameInfo->stVFrame.u32Width = arg.u32Width;
      pstFrameInfo->stVFrame.u32Height = arg.u32Height;
      pstFrameInfo->stVFrame.u64pts = arg.u64Pts;
      return XM_SUCCESS;
    }
    if (vi_abi >= 0)
      break;
  }
  return XM_FAILURE;
}

XM_S32 XM_MPI_VI_ReleaseFrame(VI_CHN ViChn, VIDEO_FRAME_INFO_S *pstFrameInfo) {
  if (ViChn >= VENC_MAX_CHN_NUM || vi_abi < 0) {
    FATAL_ARG("not support ViChn = %d, fd_vi = %d", ViChn, fd_vi);
    return XM_FAILURE;
  }

  vi_frame_t arg = {
      .chn = ViChn,
      .u32PoolId = pstFrameInfo->u32PoolId,
      .u32PhyAddr_0 = pstFrameInfo->stVFrame.u32PhyAddr[0],
      .u32PhyAddr_1 = pstFrameInfo->stVFrame.u32PhyAddr[1],
      .u32Width = pstFrameInfo->stVFrame.u32Width,
      .u32Height = pstFrameInfo->stVFrame.u32Height,
      .u64Pts = pstFrameInfo->stVFrame.u64pts,
  };
  return ioctl(fd_vi, VI_CMD(1, VI_RELEASE_FRAME, vi_frame_sizes[vi_abi]),
               &arg);
}

XM_S32 XM_MPI_VI_SetExtChnAttr(VI_CHN ViChn,
                               const VI_EXT_CHN_ATTR_S *pstExtChnAttr);

XM_S32 XM_MPI_VI_GetExtChnAttr(VI_CHN ViChn, VI_EXT_CHN_ATTR_S *pstExtChnAttr) {
  return XM_SUCCESS;
}

static const int PAL[25] = {
    1,         0x1001,    0x20101,   0x41041,   0x108421,  0x111111,  0x224489,
    0x249249,  0x1249249, 0x5294A5,  0x15294A5, 0x555555,  0x1AAAAAA, 0xAD6B5A,
    0x1AD6B5A, 0xDB6DB6,  0x1DB6DB6, 0x1DDBB76, 0x1EEEEEE, 0x1EF7BDE, 0x1FBEFBE,
    0x1FDFEFE, 0x1FFEFFE, 0x1FFFFFE, 0x1FFFFFF};

static const int NTSC[30] = {
    1,         1001,      20101,     41041,     0x108421,  0x111111,  0x224489,
    0x249249,  0x1249249, 0x5294A5,  0x15294A5, 0x555555,  0x1AAAAAA, 0xAD6B5A,
    0x1AD6B5A, 0xDB6DB6,  0x1DB6DB6, 0x1DDBB76, 0x1EEEEEE, 0x1EF7BDE, 0x1FBEFBE,
    0x1FDFEFE, 0x1FFEFFE, 0x1FFFFFE, 0x1FFFFFF

};

XM_S32 XM_MPI_VI_SetFrmRate(VI_CHN ViChn, XM_U32 srcFrmRate,
                            XM_U32 dstFrmRate) {
  if (srcFrmRate == 0 || dstFrmRate > srcFrmRate || ViChn >= VENC_MAX_CHN_NUM) {
    FATAL_ARG("not support ViChn = %d, dstFrmRate = %d, srcFrmRate = %d", ViChn,
              dstFrmRate, srcFrmRate);
    return XM_FAILURE;
  }

  if (srcFrmRate != 30 && srcFrmRate != 25) {
    int ratio = dstFrmRate / srcFrmRate;
    srcFrmRate = 25;
    dstFrmRate = ratio;
  }

  if ((fd_vi == -1) && (fd_vi = open("/dev/vi", 0), fd_vi < 0)) {
    FATAL("open error");
    return XM_FAILURE;
  }

  struct {
    VI_CHN chn;
    XM_U32 src_fps;
    XM_U32 dst_fps;
    int PAL[25];
    int NTSC[30];
  } arg = {
      .chn = ViChn,
      .src_fps = srcFrmRate - 1,
  };
  memcpy(arg.PAL, PAL, sizeof(arg.PAL));
  memcpy(arg.NTSC, NTSC, sizeof(arg.NTSC));
  if (srcFrmRate == 25)
    arg.dst_fps = PAL[dstFrmRate - 1];
  else
    arg.dst_fps = NTSC[dstFrmRate - 1];

  ioctl(fd_vi, 0x400C5607u, &arg);
  return XM_SUCCESS;
}
