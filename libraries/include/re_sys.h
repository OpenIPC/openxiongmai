#ifndef RE_SYS_H
#define RE_SYS_H

#include "re_comm_sys.h"
#include "re_common.h"
#include "re_type.h"

XM_S32 XM_MPI_SYS_Init(XM_VOID);
/*
** u64Base is the global PTS of the system.
** ADVICE:
** 1. Better to call XM_MPI_SYS_GetCurPts on the host board to get the u64Base.
** 2. When the linux starts up, call XM_MPI_SYS_InitPtsBase to set the init pts.
** 3. When media system is running, synchronize the PTS one time per minute.
**    And should call XM_MPI_SYS_SyncPts.
*/

/* alloc mmz memory in user context                                         */
XM_S32 XM_MPI_SYS_MmzAlloc(XM_U32 *pu32PhyAddr, XM_VOID **ppVirtAddr,
                           const XM_CHAR *strMmb, const XM_CHAR *strZone,
                           XM_U32 u32Len);

/* free mmz memory in user context                                          */
XM_S32 XM_MPI_SYS_MmzFree(XM_U32 u32PhyAddr, XM_VOID *pVirtAddr);

/* flush cache */
/*
** Call the mmap function to map physical address to virtual address
** The system function mmap is too complicated, so we packge it.
*/
XM_VOID *XM_MPI_SYS_Mmap(XM_U32 u32PhyAddr, XM_U32 u32Size);
XM_S32 XM_MPI_SYS_Munmap(XM_VOID *pVirAddr, XM_U32 u32Size);

XM_S32 XM_MPI_SYS_MmzReset(void);

XM_S32 XM_MPI_SYS_MmzGetAddr(XM_U32 *pu32PhyAddr, void **ppVitAddr,
                             const XM_CHAR *pstrMmb, const XM_CHAR *pstrZone,
                             XM_U32 *pu32Len);

#endif /* RE_SYS_H */
