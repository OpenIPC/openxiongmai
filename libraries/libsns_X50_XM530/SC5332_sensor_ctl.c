#include "xm_i2c.h"
#include "xm_print.h"
#include "xm_sns_ctrl.h"
#include "xm_type.h"
extern void SysDelay_ms(XM_S32 ms);

static const XM_U16 gau16SnsInit_sc5332[][2] = {
    {0x103, 1},     {0x100, 0},     {0x36E9, 0x80}, {0x36F9, 0x80},
    {0x301F, 6},    {0x3200, 0},    {0x3201, 0},    {0x3202, 0},
    {0x3203, 0},    {0x3204, 0xA},  {0x3205, 0x2B}, {0x3206, 7},
    {0x3207, 0xA3}, {0x3208, 0xA},  {0x3209, 0x28}, {0x320A, 7},
    {0x320B, 0xA0}, {0x320C, 7},    {0x320D, 0x53}, {0x3210, 0},
    {0x3211, 2},    {0x3212, 0},    {0x3213, 2},    {0x3301, 8},
    {0x3302, 0x10}, {0x3303, 8},    {0x3304, 0x40}, {0x3306, 0x40},
    {0x3308, 0x18}, {0x3309, 0x88}, {0x330B, 0xAA}, {0x330D, 0x20},
    {0x330E, 0x20}, {0x330F, 8},    {0x3310, 4},    {0x331C, 2},
    {0x331E, 0x31}, {0x331F, 0x79}, {0x3333, 0x10}, {0x3356, 0x39},
    {0x3364, 0x17}, {0x3390, 0xC},  {0x3391, 0x1C}, {0x3392, 0x3C},
    {0x3393, 0xF},  {0x3394, 0x19}, {0x3395, 0x20}, {0x33AF, 0x4A},
    {0x33B5, 0x10}, {0x3622, 0x16}, {0x3630, 0xB0}, {0x3631, 0x80},
    {0x3633, 0x73}, {0x3634, 0x43}, {0x3637, 0x12}, {0x3638, 8},
    {0x363B, 0},    {0x3670, 0xA},  {0x3674, 0xB0}, {0x3675, 0xBA},
    {0x3676, 0xBF}, {0x367C, 0xC},  {0x367D, 0x3C}, {0x3690, 0x43},
    {0x3691, 0x53}, {0x3692, 0x53}, {0x369C, 0x1C}, {0x369D, 0x3C},
    {0x36EA, 0x35}, {0x36EB, 0xC},  {0x36EC, 0xC},  {0x36ED, 0x24},
    {0x36FA, 0x35}, {0x36FC, 1},    {0x36FD, 0x27}, {0x3E01, 0xF7},
    {0x3E02, 0},    {0x3F09, 0x47}, {0x4505, 9},    {0x4509, 0x10},
    {0x4819, 9},    {0x481B, 5},    {0x481D, 0x14}, {0x481F, 4},
    {0x4821, 0xA},  {0x4823, 5},    {0x4825, 4},    {0x4827, 5},
    {0x4829, 8},    {0x5787, 0x10}, {0x5788, 6},    {0x578A, 0x10},
    {0x578B, 6},    {0x5790, 0x10}, {0x5791, 0x10}, {0x5792, 0},
    {0x5793, 0x10}, {0x5794, 0x10}, {0x5795, 0},    {0x5799, 0},
    {0x57C7, 0x10}, {0x57C8, 6},    {0x57CA, 0x10}, {0x57CB, 6},
    {0x57D0, 0x10}, {0x57D1, 0x10}, {0x57D2, 0},    {0x57D3, 0x10},
    {0x57D4, 0x10}, {0x57D5, 0},    {0x57D9, 0},    {0x59E0, 0x60},
    {0x59E1, 8},    {0x59E2, 0x3F}, {0x59E3, 0x18}, {0x59E4, 0x18},
    {0x59E5, 0x3F}, {0x59E6, 6},    {0x59E7, 2},    {0x59E8, 0x38},
    {0x59E9, 0x10}, {0x59EA, 0xC},  {0x59EB, 0x10}, {0x59EC, 4},
    {0x59ED, 2},    {0x36E9, 0x20}, {0x36F9, 0x20}, {0x100, 1},
};

void sensor_init_sc5332(void) {
  XM_U16 u16Num, u16i;
  u16Num = sizeof(gau16SnsInit_sc5332) / sizeof(gau16SnsInit_sc5332[0]);
  sensor_write_register(
      (XM_U32)gau16SnsInit_sc5332[0][0],
      (XM_U32)gau16SnsInit_sc5332[0][1]); // reset all registers
  SysDelay_ms(50);
  for (u16i = 1; u16i < u16Num; u16i++) {
    sensor_write_register((XM_U32)gau16SnsInit_sc5332[u16i][0],
                          (XM_U32)gau16SnsInit_sc5332[u16i][1]);
  }
  SysDelay_ms(20);
  DEBUG("------------- sc5332 5M 15fps  init ok! "
        "(@20200226_158x4)----------------\n");
}
