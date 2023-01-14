#include "xm_i2c.h"
#include "xm_print.h"
#include "xm_sns_ctrl.h"
#include "xm_type.h"
extern void SysDelay_ms(XM_S32 ms);

static const XM_U16 gau16SnsInit_sc3335[][2] = {

{0x103, 1},
{0x100, 0},
{0x36E9, 0x80},
{0x36F9, 0x80},
{0x301F, 0x1A},
{0x3200, 0},
{0x3201, 0},
{0x3202, 0},
{0x3203, 0},
{0x3204, 9},
{0x3205, 0xB},
{0x3206, 5},
{0x3207, 0x1B},
{0x3208, 9},
{0x3209, 8},
{0x320A, 5},
{0x320B, 0x18},
{0x320C, 5},
{0x320D, 0xA0},
{0x3210, 0},
{0x3211, 2},
{0x3212, 0},
{0x3213, 2},
{0x3253, 4},
{0x3301, 4},
{0x3302, 0x10},
{0x3304, 0x40},
{0x3306, 0x40},
{0x3309, 0x50},
{0x330B, 0xB6},
{0x330E, 0x29},
{0x3310, 6},
{0x3314, 0x96},
{0x331E, 0x39},
{0x331F, 0x49},
{0x3320, 9},
{0x3333, 0x10},
{0x334C, 1},
{0x3364, 0x17},
{0x3367, 1},
{0x3390, 4},
{0x3391, 8},
{0x3392, 0x38},
{0x3393, 5},
{0x3394, 9},
{0x3395, 0x16},
{0x33AC, 0xC},
{0x33AE, 0x1C},
{0x3622, 0x16},
{0x3637, 0x22},
{0x363A, 0x1F},
{0x363C, 5},
{0x3670, 0xE},
{0x3674, 0xB0},
{0x3675, 0x88},
{0x3676, 0x68},
{0x3677, 0x84},
{0x3678, 0x85},
{0x3679, 0x86},
{0x367C, 0x18},
{0x367D, 0x38},
{0x367E, 8},
{0x367F, 0x18},
{0x3690, 0x43},
{0x3691, 0x43},
{0x3692, 0x44},
{0x369C, 0x18},
{0x369D, 0x38},
{0x36EA, 0x34},
{0x36EB, 0xD},
{0x36EC, 0x1C},
{0x36ED, 0x34},
{0x36FA, 0x34},
{0x36FB, 0},
{0x36FC, 0x10},
{0x36FD, 0x34},
{0x3908, 0x82},
{0x391F, 0x18},
{0x3E01, 0xA8},
{0x3E02, 0x20},
{0x3F09, 0x48},
{0x4505, 8},
{0x4509, 0x20},
{0x4800, 0x44},
{0x4819, 7},
{0x481B, 4},
{0x481D, 0xD},
{0x481F, 3},
{0x4821, 9},
{0x4823, 3},
{0x4825, 3},
{0x4827, 3},
{0x4829, 5},
{0x5799, 0},
{0x59E0, 0x60},
{0x59E1, 8},
{0x59E2, 0x3F},
{0x59E3, 0x18},
{0x59E4, 0x18},
{0x59E5, 0x3F},
{0x59E6, 6},
{0x59E7, 2},
{0x59E8, 0x38},
{0x59E9, 0x10},
{0x59EA, 0xC},
{0x59EB, 0x10},
{0x59EC, 4},
{0x59ED, 2},
{0x36E9, 0x20},
{0x36F9, 0x20},
{0x100, 1},
{0x00, 50},   // 50ms

};

XM_U32 sensor_getlist_sc3335(XM_U16 *pu16Num) {
  DEBUG("------------- SC3335 3M 25fps  init ok! (@202301)----------------\n");
  *pu16Num = sizeof(gau16SnsInit_sc3335) / sizeof(gau16SnsInit_sc3335[0]);
  return (XM_U32)gau16SnsInit_sc3335;
}
