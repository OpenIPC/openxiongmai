/* The 3A libraries, which are majestic's: the ISP library takes their
 * functions from XM_MPI_ISP_AELibRegCallBack and XM_MPI_ISP_AWBLibRegCallBack
 * and runs them every frame, as the vendor library does:
 *
 *   wait for the frame;
 *   AWB: it reads the AWB statistics itself, and returns the gains and the
 *        colour matrix, which go to AWB_BASE and the CSM;
 *   AE:  it takes the luma grids made from what the AWB library read, sets
 *        the sensor's exposure and gain itself, and returns the ISP's gain.
 *
 * The 3A libraries keep their state in buffers the ISP library gives them as
 * handles.
 */
#include <pthread.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "isp_main.h"
#include "mpi_ae.h"
#include "mpi_awb.h"

#define DBG(fmt, ...) fprintf(stderr, "libisp: " fmt "\n", ##__VA_ARGS__)

static ISP_AE_REGISTER_S ae_reg;
static ISP_AWB_REGISTER_S awb_reg;
static ALG_LIB_S ae_lib, awb_lib;
static int registered;

/* the 3A libraries' state */
static XM_U32 ae_state[0x1000 / 4], awb_state[0x4000 / 4];

static ISP_AE_INFO_S ae_info;
static ISP_AE_RESULT_S ae_res = {
    .u32IspDgain = 0x100, .u32IspDgainShift = 8, .u32Iso = 100};
static ISP_AWB_STAT_1_S awb_stat1;
static ISP_AWB_RESULT_GAIN_S awb_gain = {
    .au32WhiteBalanceGain = {0x100, 0x100, 0x100}};
static ISP_AWB_RESULT_CC_S awb_cc;
static ISP_AWB_RESULT_BK_S awb_bk;
/* the AWB library's own per-zone output (the vendor library gives it a part
 * of the AWB handle's buffer) */
static XM_U16 awb_rgb[4096];

static XM_U32 zone_sites, dgain_1x = 0x100;
static XM_U8 frame_rate;
static pthread_t thread;
static int started;

int isp_3a_register(ISP_DEV dev) {
  if (registered)
    return XM_SUCCESS;
  if (XM_MPI_ISP_AELibRegCallBack(dev, &ae_lib, &ae_reg) != XM_SUCCESS ||
      !ae_reg.stAeExpFunc.pfn_ae_run) {
    DBG("XM_MPI_ISP_AELibRegCallBack failed");
    return XM_FAILURE;
  }
  if (XM_MPI_ISP_AWBLibRegCallBack(&awb_lib, &awb_reg) != XM_SUCCESS ||
      !awb_reg.stAwbExpFunc.pfn_awb_run) {
    DBG("XM_MPI_ISP_AWBLibRegCallBack failed");
    return XM_FAILURE;
  }
  /* Command 1 gives each library its handle; their inits refuse to run
   * before it. */
  XM_U32 arg[16] = {0};
  if (ae_reg.stAeExpFunc.pfn_ae_ctrl &&
      ae_reg.stAeExpFunc.pfn_ae_ctrl(dev, (XM_S32)(intptr_t)ae_state, 1, arg) !=
          XM_SUCCESS)
    DBG("pfn_ae_ctrl failed");
  if (awb_reg.stAwbExpFunc.pfn_awb_ctrl &&
      awb_reg.stAwbExpFunc.pfn_awb_ctrl((XM_S32)(intptr_t)awb_state, 1, arg) !=
          XM_SUCCESS)
    DBG("pfn_awb_ctrl failed");
  registered = 1;
  return XM_SUCCESS;
}

/* The AWB statistics window (from the AWB library) in 32 x 16 zones. */
static void awb_window(const ISP_AWB_WDW_ATTR_S *w) {
  const XM_U32 h0 = w->u32HBegin, h1 = w->u32HEnd;
  const XM_U32 v0 = w->u32VBegin, v1 = w->u32VEnd;
  if (h1 <= h0 || v1 <= v0)
    return;
  isp_write(REG_AWB_CTRL, 1);
  isp_write(REG_AWB_LIMIT, 0xfff);
  for (int k = 0; k <= AWB_COLS; k++)
    isp_write(REG_AWB_HGRID + 4 * k, h0 + k * (h1 - h0) / AWB_COLS);
  /* the last row ends a line short, as the vendor library has it */
  for (int k = 0; k <= AWB_ROWS; k++)
    isp_write(REG_AWB_VGRID + 4 * k,
              v0 + k * (v1 - v0) / AWB_ROWS - (k == AWB_ROWS));
  /* a zone's pixels of each colour */
  zone_sites = ((h1 - h0) / AWB_COLS / 2) * ((v1 - v0) / AWB_ROWS / 2);
}

int isp_3a_init(ISP_DEV dev, const ISP_PUB_ATTR_S *pub) {
  if (!registered)
    return XM_FAILURE;
  frame_rate = pub->u8FrameRate;
  if (isp_mode)
    dgain_1x = isp_mode->dgain_1x;
  ISP_AE_PARAM_S ae_param = {.SensorId = 0,
                             .u32MaxIspDgain = 0x400,
                             .u32MinIspDgain = 0x100,
                             .u32IspDgainShift = 8,
                             .u8FrameRate = pub->u8FrameRate};
  if (ae_reg.stAeExpFunc.pfn_ae_init &&
      ae_reg.stAeExpFunc.pfn_ae_init(dev, (XM_S32)(intptr_t)ae_state,
                                     &ae_param) != XM_SUCCESS)
    DBG("pfn_ae_init failed");
  memset(&awb_bk, 0, sizeof(awb_bk));
  if (awb_reg.stAwbExpFunc.pfn_awb_init &&
      awb_reg.stAwbExpFunc.pfn_awb_init((XM_S32)(intptr_t)awb_state, &awb_bk,
                                        pub->stWndRect.u32Width,
                                        pub->stWndRect.u32Height) != XM_SUCCESS)
    DBG("pfn_awb_init failed");
  awb_window(&awb_bk.stWin);
  return XM_SUCCESS;
}

static void awb_apply(void) {
  for (int i = 0; i < 3; i++)
    isp_write(REG_AWB_GAIN + 4 * i, awb_gain.au32WhiteBalanceGain[i]);
  if (awb_cc.bChange) {
    for (int i = 0; i < 12; i++)
      isp_write(REG_CSM + 4 * i,
                (XM_U32)(XM_S32)(XM_S16)awb_cc.au16ColorMatrix[i]);
    awb_cc.bChange = XM_FALSE;
  }
  if (awb_bk.stWin.bChange) {
    awb_window(&awb_bk.stWin);
    awb_bk.stWin.bChange = XM_FALSE;
  }
}

static void *run_3a(void *arg) {
  (void)arg;
  XM_U32 n = 0, dgain = 0;
  const int debug = getenv("LIBISP_DEBUG") != NULL;
  ISP_AWB_INFO_S awb_info = {.pstAwbStat1 = &awb_stat1};
  ISP_AWB_RESULT_S awb_res = {.pstRlt_Gain = &awb_gain,
                              .pstRlt_Cc = &awb_cc,
                              .pstRlt_Bk = &awb_bk,
                              .pstRlt_RGB = awb_rgb};
  for (;;) {
    if (isp_wait_frame() < 0) {
      usleep(10000);
      continue;
    }
    n++;
    awb_info.u32Rsv = ae_res.u32Rsv;
    if (awb_reg.stAwbExpFunc.pfn_awb_run(0, &awb_info, &awb_res,
                                         (XM_S32)ae_res.u32Iso) == XM_SUCCESS)
      awb_apply();

    ae_info.u32FrameCnt = n;
    ae_info.u8FrameRate = frame_rate;
    /* every bin 0x80, as the vendor library gives it */
    for (int i = 0; i < 16; i++)
      ae_info.stAeStat1.au32MeteringHist[i] = 0x80;
    stats_grid(awb_gain.au32WhiteBalanceGain, zone_sites, &ae_info.stAeStat2,
               &ae_info.stAeStat3);
    if (ae_reg.stAeExpFunc.pfn_ae_run(0, 0, &ae_info, &ae_res, !(n & 1)) ==
        XM_SUCCESS) {
      if (debug && n % 50 == 0)
        DBG("frame %u: zones %u %u %u, V2 %u %u, R %x G %x B %x, AE %x %x %x "
            "%x "
            "%x",
            n, ae_info.stAeStat2.au32MeteringWin[3][2],
            ae_info.stAeStat2.au32MeteringWin[3][3],
            ae_info.stAeStat2.au32MeteringWin[3][4],
            ae_info.stAeStat3.au16MeteringWinV2[8][14],
            ae_info.stAeStat3.au16MeteringWinV2[8][15],
            awb_gain.au32WhiteBalanceGain[0], awb_gain.au32WhiteBalanceGain[1],
            awb_gain.au32WhiteBalanceGain[2], ae_res.u32IspDgain,
            ae_res.u32IspDgainShift, ae_res.u32Iso, ae_res.u32Rsv,
            ae_res.u32UpdateCfg);
      const XM_U32 g = (ae_res.u32IspDgain & 0xffff) * dgain_1x >> 8;
      if (g != dgain) {
        isp_write(REG_DGAIN, g);
        dgain = g;
      }
    }
  }
  return NULL;
}

int isp_3a_start(void) {
  if (!registered || started)
    return registered ? XM_SUCCESS : XM_FAILURE;
  if (pthread_create(&thread, NULL, run_3a, NULL))
    return XM_FAILURE;
  started = 1;
  return XM_SUCCESS;
}
