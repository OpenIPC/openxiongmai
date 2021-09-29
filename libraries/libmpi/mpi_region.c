#include <re_region.h>

#include <fcntl.h>
#include <stdio.h>
#include <sys/ioctl.h>

#define MODULE "vpss"
#include "common.h"

#define OPEN_RGN                                                               \
  if ((fd_rgn == -1) && (fd_rgn = open("/dev/rgn", 0), fd_rgn < 0)) {          \
    FATAL("mmz dev Open Failed");                                              \
    return XM_FAILURE;                                                         \
  }

#define CHECK_VICHN                                                            \
  if (ViChn > 1) {                                                             \
    FATAL_ARG("not support ViChn = %d, fd_rng = %d", ViChn, fd_rgn);           \
    return XM_FAILURE;                                                         \
  }

static int fd_rgn = 1;

XM_S32 XM_MPI_RGN_INIT(void) {
  OPEN_RGN;

  ioctl(fd_rgn, 0x5201u, 0);
  return XM_SUCCESS;
}

// TODO: incomplete
XM_S32 XM_MPI_RGN_Create(VI_CHN ViChn, const RGN_ATTR_S *pstRegion) {
  OPEN_RGN;
  CHECK_VICHN;

  switch (pstRegion->enType) {
  case COVER_RGN: {
    if (pstRegion->u32Handle >= COVER_MAX_NUM) {
      FATAL_ARG("not support Handle = %d", pstRegion->u32Handle);
      return XM_FAILURE;
    }
    // TODO
  };

  case OVERLAY_RGN: {
    if (pstRegion->u32Handle >= OVERLAY_MAX_NUM) {
      FATAL_ARG("not support Handle = %d", pstRegion->u32Handle);
      return XM_FAILURE;
    }
    // TODO
  }

  default:
    FATAL("unknown enType");
    return XM_FAILURE;
  }

  return XM_SUCCESS;
}

XM_S32 XM_MPI_RGN_Destroy(VI_CHN ViChn, const RGN_ATTR_S *pstRegion) {
  OPEN_RGN;
  CHECK_VICHN;

  switch (pstRegion->enType) {
  case COVER_RGN: {
    if (pstRegion->u32Handle >= COVER_MAX_NUM) {
      FATAL_ARG("not support Handle = %d", pstRegion->u32Handle);
      return XM_FAILURE;
    }
    // TODO: zero attrs
    return ioctl(fd_rgn, 0x40185208, &pstRegion->u32Handle);
  };

  case OVERLAY_RGN: {
    if (pstRegion->u32Handle >= OVERLAY_MAX_NUM) {
      FATAL_ARG("not support Handle = %d", pstRegion->u32Handle);
      return XM_FAILURE;
    }
    // TODO: zero attrs
    struct {
      VI_CHN chn;
      XM_U32 handle;
    } arg = {
        .chn = ViChn,
        .handle = pstRegion->u32Handle,
    };
    return ioctl(fd_rgn, 0x40105203, &arg);
  }

  default:
    FATAL("unknown enType");
    return XM_FAILURE;
  }

  return XM_SUCCESS;
}

XM_S32 XM_MPI_RGN_GetAttr(VI_CHN ViChn, RGN_ATTR_S *pstRegion);
XM_S32 XM_MPI_RGN_SetAttr(VI_CHN ViChn, const RGN_ATTR_S *pstRegion);

XM_S32 XM_MPI_RGN_SetBitMap(VI_CHN ViChn, const BITMAP_S *pstBitmap) {
  OPEN_RGN;
  CHECK_VICHN;

  if (pstBitmap->u32Handle > 0xf) {
    FATAL_ARG("not support Handle = %d", pstBitmap->u32Handle);
    return XM_FAILURE;
  }

  if (!pstBitmap->pData) {
    FATAL("pData is NULL");
    return XM_FAILURE;
  }

  struct {
    VI_CHN chn;
    XM_U32 handle;
    XM_VOID *data;
    XM_U32 size;
  } arg = {
      .chn = ViChn,
      .handle = pstBitmap->u32Handle,
      .data = pstBitmap->pData,
      .size = pstBitmap->u32Width * pstBitmap->u32Height / 8,
  };
  return ioctl(fd_rgn, 0x40105205u, &arg);
}

XM_S32 XM_MPI_RGN_Enable(VI_CHN ViChn, XM_U32 Handle, RGN_TYPE_E enType) {
  OPEN_RGN;
  CHECK_VICHN;

  struct {
    VI_CHN chn;
    XM_U32 handle;
    RGN_TYPE_E type;
  } arg = {
      .chn = ViChn,
      .handle = Handle,
      .type = enType,
  };

  switch (enType) {
  case COVER_RGN: {
    if (Handle >= COVER_MAX_NUM) {
      FATAL_ARG("not support Handle = %d", Handle);
      return XM_FAILURE;
    }
    return ioctl(fd_rgn, 0x400C520A, &arg);
  };

  case OVERLAY_RGN: {
    if (Handle >= OVERLAY_MAX_NUM) {
      FATAL_ARG("not support Handle = %d", Handle);
      return XM_FAILURE;
    }
    return ioctl(fd_rgn, 0x400C5206, &arg);
  }

  default:
    FATAL("unknown enType");
    return XM_FAILURE;
  }
}

// TODO: check code why it's the same with RGN_Enable?
XM_S32 XM_MPI_RGN_Disable(VI_CHN ViChn, XM_U32 Handle, RGN_TYPE_E enType) {
  OPEN_RGN;
  CHECK_VICHN;

  struct {
    VI_CHN chn;
    XM_U32 handle;
    RGN_TYPE_E type;
  } arg = {
      .chn = ViChn,
      .handle = Handle,
      .type = enType,
  };

  switch (enType) {
  case COVER_RGN: {
    if (Handle >= COVER_MAX_NUM) {
      FATAL_ARG("not support Handle = %d", Handle);
      return XM_FAILURE;
    }
    return ioctl(fd_rgn, 0x400C520A, &arg);
  };

  case OVERLAY_RGN: {
    if (Handle >= OVERLAY_MAX_NUM) {
      FATAL_ARG("not support Handle = %d", Handle);
      return XM_FAILURE;
    }
    return ioctl(fd_rgn, 0x400C5206, &arg);
  }

  default:
    FATAL("unknown enType");
    return XM_FAILURE;
  }
}
