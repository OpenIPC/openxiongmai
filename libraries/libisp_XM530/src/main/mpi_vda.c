/* Motion detection (XM_MPI_VDA_*) on the AE library's 7 x 7 luma grid, as
 * the vendor library behaves on the lab XM550 (#76):
 *
 *  - every MoDframe + 1 grids, with MoEnble set, the grid is compared with
 *    the one of the analysis before; a zone moved when it differs from that
 *    by more than its 1/MoRatio;
 *  - MoVdaBmark[r] bit c marks a moved zone, every zone; the share of moved
 *    zones among those MoWinset[r] bit c enables, in percent, is the count
 *    and what XM_MPI_VDA_GetMovData returns, and MoReflag is set when it
 *    reaches u8RatioThresh;
 *  - results gather (OR) until read; a read doesn't clear them, the next
 *    analysis after it does.
 *
 * Cover detection keeps its settings and reports nothing yet.
 */
#include <pthread.h>
#include <string.h>

#include "isp_main.h"
#include "mpi_vdam.h"

#define VDA_CHNS 2

static pthread_mutex_t vda_lock = PTHREAD_MUTEX_INITIALIZER;
static VDA_MOTION_MOVE_INIT mov[VDA_CHNS] = {
    {.u8RatioThresh = 2, .MoRatio = 6, .MoDframe = 20},
    {.u8RatioThresh = 2, .MoRatio = 6, .MoDframe = 20}};
static VDA_MOTION_SHELTER_INIT sh[VDA_CHNS];

struct mov_state {
  VDA_MOTION_MOVE_RECDATA ref;
  VDA_MOTION_MOVE_RESULT res;
  XM_U32 frames;
  int have_ref, read;
  XM_U8 percent;
};
static struct mov_state ms[VDA_CHNS];

static void analyse(int chn, const ISP_AE_STAT_2_S *grid) {
  const VDA_MOTION_MOVE_INIT *m = &mov[chn];
  struct mov_state *s = &ms[chn];
  if (!m->MoEnble || ++s->frames <= m->MoDframe)
    return;
  s->frames = 0;
  if (s->have_ref && m->MoRatio) {
    if (s->read) {
      memset(&s->res, 0, sizeof(s->res));
      s->percent = 0;
      s->read = 0;
    }
    XM_U32 moved = 0, watched = 0;
    for (int r = 0; r < AE_ZONE_ROW; r++)
      for (int c = 0; c < AE_ZONE_COLUMN; c++) {
        const XM_U32 cur = grid->au32MeteringWin[r][c];
        const XM_U32 ref = s->ref.MoChvall[r][c];
        const XM_U32 diff = cur > ref ? cur - ref : ref - cur;
        const int hit = diff > ref / m->MoRatio;
        if (hit)
          s->res.MoVdaBmark[r] |= 1u << c;
        if (m->MoWinset[r] & (1u << c)) {
          watched++;
          moved += hit;
        }
      }
    const XM_U8 percent = watched ? (XM_U8)(moved * 100 / watched) : 0;
    if (percent > s->percent)
      s->percent = percent;
    s->res.MoVdaCount = s->percent;
    if (percent >= m->u8RatioThresh && moved)
      s->res.MoReflag = 1;
  }
  memcpy(s->ref.MoChvall, grid->au32MeteringWin, sizeof(s->ref.MoChvall));
  s->have_ref = 1;
}

void vda_frame(const ISP_AE_STAT_2_S *grid) {
  pthread_mutex_lock(&vda_lock);
  for (int i = 0; i < VDA_CHNS; i++)
    analyse(i, grid);
  pthread_mutex_unlock(&vda_lock);
}

void XM_MPI_VDA_CreatChn(XM_U8 VdaChn) { (void)VdaChn; }
void XM_MPI_VDA_DestroyChn(XM_U8 VdaChn) { (void)VdaChn; }

int XM_MPI_VDA_SetMovAttr(XM_U8 channel, VDA_MOTION_MOVE_INIT *pstApInfo) {
  if (channel >= VDA_CHNS || !pstApInfo)
    return XM_FAILURE;
  pthread_mutex_lock(&vda_lock);
  mov[channel] = *pstApInfo;
  pthread_mutex_unlock(&vda_lock);
  return XM_SUCCESS;
}

void XM_MPI_VDA_GetMovAttr(XM_U8 channel, VDA_MOTION_MOVE_INIT *gstApInfo) {
  if (channel >= VDA_CHNS || !gstApInfo)
    return;
  pthread_mutex_lock(&vda_lock);
  *gstApInfo = mov[channel];
  pthread_mutex_unlock(&vda_lock);
}

int XM_MPI_VDA_GetMovData(XM_U8 channel, VDA_MOTION_MOVE_RESULT *pstApInfo) {
  if (channel >= VDA_CHNS || !pstApInfo)
    return XM_FAILURE;
  pthread_mutex_lock(&vda_lock);
  *pstApInfo = ms[channel].res;
  const int percent = ms[channel].percent;
  ms[channel].read = 1;
  pthread_mutex_unlock(&vda_lock);
  return percent;
}

int XM_MPI_VDA_SetShAttr(XM_U8 channel, VDA_MOTION_SHELTER_INIT *pstApInfo) {
  if (channel >= VDA_CHNS || !pstApInfo)
    return XM_FAILURE;
  pthread_mutex_lock(&vda_lock);
  sh[channel] = *pstApInfo;
  pthread_mutex_unlock(&vda_lock);
  return XM_SUCCESS;
}

int XM_MPI_VDA_GetShAttr(XM_U8 channel, VDA_MOTION_SHELTER_INIT *pstApInfo) {
  if (channel >= VDA_CHNS || !pstApInfo)
    return XM_FAILURE;
  pthread_mutex_lock(&vda_lock);
  *pstApInfo = sh[channel];
  pthread_mutex_unlock(&vda_lock);
  return XM_SUCCESS;
}

void XM_MPI_VDA_GetShdMark(XM_U8 channel, XM_U8 *shdmark) {
  (void)channel;
  if (shdmark)
    *shdmark = 0;
}
