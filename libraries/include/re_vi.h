#ifndef RE_VI_H
#define RE_VI_H

#include "re_comm_vi.h"

XM_S32 XM_MPI_VI_INIT(void);
XM_S32 XM_MPI_VI_SetChnAttr(VI_CHN ViChn, const VI_CHN_ATTR_S *pstAttr);
XM_S32 XM_MPI_VI_GetChnAttr(VI_CHN ViChn, VI_CHN_ATTR_S *pstAttr);

XM_S32 XM_MPI_VI_EnableChn(VI_CHN ViChn);
XM_S32 XM_MPI_VI_DisableChn(VI_CHN ViChn);

XM_S32 XM_MPI_VI_GetFrame(VI_CHN ViChn, VIDEO_FRAME_INFO_S *pstFrameInfo);
XM_S32 XM_MPI_VI_ReleaseFrame(VI_CHN ViChn, VIDEO_FRAME_INFO_S *pstFrameInfo);

XM_S32 XM_MPI_VI_SetExtChnAttr(VI_CHN ViChn,
                               const VI_EXT_CHN_ATTR_S *pstExtChnAttr);
XM_S32 XM_MPI_VI_GetExtChnAttr(VI_CHN ViChn, VI_EXT_CHN_ATTR_S *pstExtChnAttr);

XM_S32 XM_MPI_VI_SetFrmRate(VI_CHN ViChn, XM_U32 srcFrmRate, XM_U32 dstFrmRate);

#endif /* RE_VI_H */
