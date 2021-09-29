#ifndef RE_VENC_H
#define RE_VENC_H

#include "re_common.h"
#include "re_comm_video.h"
#include "re_comm_venc.h"

XM_S32 XM_MPI_VENC_Init(void);
XM_S32 XM_MPI_VENC_CreateGroup(VENC_GRP VeGroup);
XM_S32 XM_MPI_VENC_DestroyGroup(VENC_GRP VeGroup);

XM_S32 XM_MPI_VENC_CreateChn(VENC_CHN VeChn, const VENC_CHN_ATTR_S *pstAttr);
XM_S32 XM_MPI_VENC_DestroyChn(VENC_CHN VeChn);

XM_S32 XM_MPI_VENC_RegisterChn(VENC_GRP VeGroup, VENC_CHN VeChn );
XM_S32 XM_MPI_VENC_UnRegisterChn(VENC_CHN VeChn);

XM_S32 XM_MPI_VENC_StartRecvPic(VENC_CHN VeChn);
XM_S32 XM_MPI_VENC_StartRecvPicEx(VENC_CHN VeChn, VENC_RECV_PIC_PARAM_S *pstRecvParam);
XM_S32 XM_MPI_VENC_StopRecvPic(VENC_CHN VeChn);

XM_S32 XM_MPI_VENC_Query(VENC_CHN VeChn, VENC_CHN_STAT_S *pstStat);

XM_S32 XM_MPI_VENC_SetChnAttr( VENC_CHN VeChn, const VENC_CHN_ATTR_S *pstAttr);
XM_S32 XM_MPI_VENC_GetChnAttr( VENC_CHN VeChn, VENC_CHN_ATTR_S *pstAttr);

XM_S32 XM_MPI_VENC_SetH264Param(VENC_CHN VeChn, const VENC_CHN_PARAM_S *pstParam);
XM_S32 XM_MPI_VENC_GetH264Param(VENC_CHN VeChn, VENC_CHN_PARAM_S *pstParam);

XM_S32 XM_MPI_VENC_GetStream(VENC_CHN VeChn, VENC_STREAM_S *pstStream, XM_BOOL bBlockFlag);
XM_S32 XM_MPI_VENC_ReleaseStream(VENC_CHN VeChn, VENC_STREAM_S *pstStream);

XM_S32 XM_MPI_VENC_GetFd(VENC_CHN VeChn);

XM_S32 XM_MPI_VENC_RequestIDR(VENC_CHN VeChn);

XM_S32 XM_MPI_VENC_GETRefParamEx(VENC_CHN VeChn,VENC_PARAM_REF_EX_S *pstRefParam);

XM_S32 XM_MPI_VENC_SetRefParamEx(VENC_CHN VeChn,VENC_PARAM_REF_EX_S *pstRefParam);

XM_S32 XM_MPI_VENC_SelectData(void);

#endif /* RE_VENC_H */
