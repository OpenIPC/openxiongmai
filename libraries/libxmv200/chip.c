// SPDX-License-Identifier: MIT
/*
 * The V200 series' chip families: the same pipeline and steps, with their
 * kernel modules' own struct versions and clock access. Each sensor tuning
 * names the family it was captured on.
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
    },
};

const struct xmv200_chip *xmv200_chip_of(const struct xmv200_sns_tuning *t) {
  for (unsigned i = 0; i < sizeof(chips) / sizeof(chips[0]); i++)
    if (t->chip && !strcmp(t->chip, chips[i].name))
      return &chips[i];
  return &chips[0];
}
