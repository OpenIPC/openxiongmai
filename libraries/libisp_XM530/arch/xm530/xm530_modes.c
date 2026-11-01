/* The modes the library has start-up tables for, by picture size. */
#include <stddef.h>

#include "xm530_start.h"

static const struct xm530_mode modes[] = {
    /* DIGITAL_GAIN 0x109 at 1x: what the vendor library writes for an ISP
     * gain of 0x100 (it scales the AE library's gain by 0x109 >> 8). */
    {"IMX335 2600x1954", 2600, 1954, 0x109, xm530_imx335_init,
     xm530_imx335_post, xm530_imx335_run, xm530_imx335_attr},
};

const struct xm530_mode *xm530_mode_find(XM_U32 width, XM_U32 height) {
  for (size_t i = 0; i < sizeof(modes) / sizeof(modes[0]); i++)
    if (modes[i].width == width && modes[i].height == height)
      return &modes[i];
  return NULL;
}
