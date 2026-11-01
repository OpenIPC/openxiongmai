/* Motion and cover detection (XM_MPI_VDA_*): the settings are kept, and no
 * motion or cover is reported yet. */
#include <string.h>

#include "mpi_isp.h"
#include "mpi_vdam.h"

#define VDA_CHNS 2

static VDA_MOTION_MOVE_INIT mov[VDA_CHNS];
static VDA_MOTION_SHELTER_INIT sh[VDA_CHNS];

void XM_MPI_VDA_CreatChn(XM_U8 VdaChn) { (void)VdaChn; }
void XM_MPI_VDA_DestroyChn(XM_U8 VdaChn) { (void)VdaChn; }

int XM_MPI_VDA_SetMovAttr(XM_U8 channel, VDA_MOTION_MOVE_INIT *pstApInfo) {
  if (channel >= VDA_CHNS || !pstApInfo)
    return XM_FAILURE;
  mov[channel] = *pstApInfo;
  return XM_SUCCESS;
}

void XM_MPI_VDA_GetMovAttr(XM_U8 channel, VDA_MOTION_MOVE_INIT *gstApInfo) {
  if (channel < VDA_CHNS && gstApInfo)
    *gstApInfo = mov[channel];
}

int XM_MPI_VDA_GetMovData(XM_U8 channel, VDA_MOTION_MOVE_RESULT *pstApInfo) {
  if (channel >= VDA_CHNS || !pstApInfo)
    return XM_FAILURE;
  memset(pstApInfo, 0, sizeof(*pstApInfo));
  return 0;
}

int XM_MPI_VDA_SetShAttr(XM_U8 channel, VDA_MOTION_SHELTER_INIT *pstApInfo) {
  if (channel >= VDA_CHNS || !pstApInfo)
    return XM_FAILURE;
  sh[channel] = *pstApInfo;
  return XM_SUCCESS;
}

int XM_MPI_VDA_GetShAttr(XM_U8 channel, VDA_MOTION_SHELTER_INIT *pstApInfo) {
  if (channel >= VDA_CHNS || !pstApInfo)
    return XM_FAILURE;
  *pstApInfo = sh[channel];
  return XM_SUCCESS;
}

void XM_MPI_VDA_GetShdMark(XM_U8 channel, XM_U8 *shdmark) {
  (void)channel;
  if (shdmark)
    *shdmark = 0;
}
