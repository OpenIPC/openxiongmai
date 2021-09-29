#ifndef MPI_REGION_H
#define MPI_REGION_H

#include "re_comm_region.h"

XM_S32 XM_MPI_RGN_INIT(void);
XM_S32 XM_MPI_RGN_Create(VI_CHN ViChn, const RGN_ATTR_S *pstRegion);
XM_S32 XM_MPI_RGN_Destroy(VI_CHN ViChn, const RGN_ATTR_S *pstRegion);

XM_S32 XM_MPI_RGN_GetAttr(VI_CHN ViChn, RGN_ATTR_S *pstRegion);
XM_S32 XM_MPI_RGN_SetAttr(VI_CHN ViChn, const RGN_ATTR_S *pstRegion);

XM_S32 XM_MPI_RGN_SetBitMap(VI_CHN ViChn, const BITMAP_S *pstBitmap);

XM_S32 XM_MPI_RGN_Enable(VI_CHN ViChn, XM_U32 Handle, RGN_TYPE_E enType);
XM_S32 XM_MPI_RGN_Disable(VI_CHN ViChn, XM_U32 Handle, RGN_TYPE_E enType);

#endif /* MPI_REGION_H */
