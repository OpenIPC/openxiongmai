// SPDX-License-Identifier: MIT
/*
 * The V200 series' chip families: the same pipeline and steps, with their
 * kernel modules' own struct versions, register places and clock access.
 * Each sensor tuning names the family it was captured on.
 */
#include <string.h>

#include "sns_tuning.h"
#include "xmv200_priv.h"

static const struct xmv200_chip chips[] = {
    {
        .name = "xm530v200",
        .engine = "xm530v200",
        .vi_window = 1,
        .vi_sync2 = 1,
        .hevc_reset = 0x10002,
        .isp_pool = 1,
        .vb_count = 3,
        .rx = 0x4000,
        .vicu = 0x40000,
        .vi_ctrl_chn = 1,
        .comm_n = 9,
    },
    {
        /* XM535D1/D2: the PLL through sys.ko; isp.ko's offline BPU wants
         * each frame wait answered; the longer 0x490e/0x4980 */
        .name = "xm535",
        .engine = "xm535",
        .sys_clocks = 1,
        .frame_answer = 1,
        .vi_mode = 1,
        .chn_attr_idx = 1,
        .hevc_reset = 0x04000000,
        .vb_mode = 1,
        .vb_count = 2,
        .venc_v2 = 1,
        .rx = 0x4000,
        .vicu = 0x40000,
        .vi_ctrl_chn = 1,
        .comm_n = 9,
        .comm_ib = 1,
    },
    {
        /* XM535W1: the XM530V200's isp.ko, the XM535's clocks and encoder;
         * no VPSS, vi.ko scales */
        .name = "xm535w1",
        .engine = "xm535",
        .sys_clocks = 1,
        .frame_answer = 1,
        .vi_window = 1,
        .vi_sync2 = 1,
        .hevc_reset = 0x04000000,
        .vb_mode = 1,
        .vb_count = 2,
        .rx = 0x4000,
        .vicu = 0x40000,
        .vi_ctrl_chn = 1,
        .comm_n = 9,
        .vi_scaler = 1,
        .vi_format = 2,
    },
    {
        /* XM535D1/D2 with an online BPU (the tunings captured with one ISP,
         * FunExt 0): isp.ko's VICU feeds the BPU straight, which isp.ko and
         * vpss.ko never hand to the VPSS; vi.ko scales the BPU's picture, as
         * on the XM535W1 (the vendor loads vi.ko for it) */
        .name = "xm535",
        .engine = "xm535",
        .sys_clocks = 1,
        .frame_answer = 1,
        .vi_mode = 1,
        .chn_attr_idx = 1,
        .hevc_reset = 0x04000000,
        .vb_mode = 1,
        .vb_count = 2,
        .venc_v2 = 1,
        .rx = 0x4000,
        .vicu = 0x40000,
        .vi_ctrl_chn = 1,
        .comm_n = 9,
        .comm_ib = 1,
        .vi_scaler = 1,
        .vi_format = 2,
    },
    {
        /* XM533: the XM530V200's ISP with VI's blocks moved; its own
         * encoder; no VPSS, vi.ko scales */
        .name = "xm533",
        .engine = "xm533",
        .frame_answer = 1,
        .vi_window = 1,
        .vi_sync2 = 1,
        .hevc_reset = 0x10002,
        .vb_count = 3,
        .rx = 0x0000,
        .vicu = 0x10000,
        .vi_ctrl = 0xf0000,
        .comm_n = 10,
        .vi_scaler = 1,
    },
};

const struct xmv200_chip *xmv200_chip_of(const struct xmv200_sns_tuning *t) {
  /* an XM535D1/D2 tuning with the BPU online (0x4962 word 2's low byte 0) */
  if (t->chip && !strcmp(t->chip, "xm535") && !(t->comm[2] & 0xff))
    return &chips[3];
  for (unsigned i = 0; i < sizeof(chips) / sizeof(chips[0]); i++)
    if (t->chip && !strcmp(t->chip, chips[i].name))
      return &chips[i];
  return &chips[0];
}
