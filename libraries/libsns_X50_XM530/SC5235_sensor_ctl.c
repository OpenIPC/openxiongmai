#include "xm_i2c.h"
#include "xm_print.h"
#include "xm_sns_ctrl.h"
#include "xm_type.h"
extern void SysDelay_ms(XM_S32 ms);

static const XM_U16 gau16SnsInit_sc5235[][2] = {
    // ExtClk: 24M MCLK 158.4M
    {0x0103, 0x01}, {0x0100, 0x00},

    {0x3039, 0xc0}, // bypass pll1
    {0x3029, 0xb4}, // bypass pll2

    {0x302a, 0x75}, {0x302b, 0x00}, {0x302c, 0x00}, {0x3037, 0x26},
    {0x3038, 0x66}, {0x303a, 0x35}, {0x303b, 0x0a}, {0x303c, 0x0e},
    {0x303d, 0x33}, {0x3200, 0x00}, {0x3201, 0x00}, {0x3202, 0x00},
    {0x3203, 0x00}, {0x3204, 0x0a}, {0x3205, 0x33}, {0x3206, 0x07},
    {0x3207, 0xa7}, {0x3208, 0x0a}, {0x3209, 0x28}, {0x320a, 0x07},
    {0x320b, 0xa0}, {0x320c, 0x07}, {0x320d, 0xd0}, // 4000
    {0x320e, 0x07}, {0x320f, 0xbc},                 // 1980
    {0x3210, 0x00}, {0x3211, 0x04}, {0x3212, 0x00}, {0x3213, 0x04},
    {0x3235, 0x0f}, {0x3236, 0x9c}, {0x3301, 0x1c}, {0x3303, 0x20},
    {0x3304, 0x10}, {0x3306, 0x58}, {0x3308, 0x10}, {0x3309, 0x60},
    {0x330a, 0x00}, {0x330b, 0xb8}, {0x330d, 0x30}, {0x330e, 0x20},
    {0x3314, 0x14}, {0x3315, 0x02}, {0x331b, 0x83}, {0x331e, 0x19},
    {0x331f, 0x59}, {0x3320, 0x01}, {0x3321, 0x04}, {0x3326, 0x00},
    {0x3332, 0x22}, {0x3333, 0x20}, {0x3334, 0x40}, {0x3350, 0x22},
    {0x3359, 0x22}, {0x335c, 0x22}, {0x3364, 0x05}, {0x3366, 0xc8},
    {0x3367, 0x08}, {0x3368, 0x03}, {0x3369, 0x00}, {0x336a, 0x00},
    {0x336b, 0x00}, {0x336c, 0x01}, {0x336d, 0x40}, {0x337f, 0x03},
    {0x338f, 0x40}, {0x33ae, 0x22}, {0x33af, 0x22}, {0x33b0, 0x22},
    {0x33b4, 0x22}, {0x33b6, 0x07}, {0x33b7, 0x17}, {0x33b8, 0x20},
    {0x33b9, 0x20}, {0x33ba, 0x44}, {0x3614, 0x00}, {0x3620, 0x28},
    {0x3621, 0xac}, {0x3622, 0xf6}, {0x3623, 0x08}, {0x3624, 0x47},
    {0x3625, 0x0b}, {0x3630, 0x30}, {0x3631, 0x88}, {0x3632, 0x18},
    {0x3633, 0x34}, {0x3634, 0x86}, {0x3635, 0x4d}, {0x3636, 0x21},
    {0x3637, 0x20}, {0x3638, 0x18}, {0x3639, 0x09}, {0x363a, 0x83},
    {0x363b, 0x02}, {0x363c, 0x07}, {0x363d, 0x03}, {0x3670, 0x00},
    {0x3677, 0x86}, {0x3678, 0x86}, {0x3679, 0xa8}, {0x367e, 0x08},
    {0x367f, 0x18}, {0x3905, 0x98}, {0x3907, 0x01}, {0x3908, 0x11},
    {0x390a, 0x00}, {0x391c, 0x9f}, {0x391d, 0x00}, {0x391e, 0x01},
    {0x391f, 0xc0}, {0x3e00, 0x00}, {0x3e01, 0xf7}, {0x3e02, 0x00},
    {0x3e03, 0x0b}, {0x3e06, 0x00}, {0x3e07, 0x80}, {0x3e08, 0x03},
    {0x3e09, 0x20}, {0x3e1e, 0x30}, {0x3e26, 0x20}, {0x3f00, 0x0d},
    {0x3f02, 0x05}, {0x3f04, 0x02}, {0x3f05, 0xaa}, {0x3f06, 0x21},
    {0x3f08, 0x04}, {0x4500, 0x5d}, {0x4502, 0x10}, {0x4509, 0x10},
    {0x4809, 0x01}, {0x4837, 0x19}, {0x5000, 0x20}, {0x5002, 0x00},
    {0x6000, 0x20}, {0x6002, 0x00},

    {0x3652, 0x00}, // lane ��ʱ�� 100ps/step��Ĭ�� 3��b100
    {0x3039, 0x20}, // enable pll1
    {0x3029, 0x35}, // enable pll2

    {0x0100, 0x01}};

void sensor_init_sc5235(void) {
  XM_U16 u16Num, u16i;
  u16Num = sizeof(gau16SnsInit_sc5235) / sizeof(gau16SnsInit_sc5235[0]);
  sensor_write_register(
      (XM_U32)gau16SnsInit_sc5235[0][0],
      (XM_U32)gau16SnsInit_sc5235[0][1]); // reset all registers
  SysDelay_ms(50);
  for (u16i = 1; u16i < u16Num; u16i++) {
    sensor_write_register((XM_U32)gau16SnsInit_sc5235[u16i][0],
                          (XM_U32)gau16SnsInit_sc5235[u16i][1]);
  }
  DEBUG("------------- SC5235 5M 15fps  init ok! "
        "(@20180312_158x4)----------------\n");
}
