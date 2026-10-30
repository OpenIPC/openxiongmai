// SPDX-License-Identifier: MIT
/* SmartSens SC1235, SC2145H and SC2235 exposure and gain (smartsens.h). */
#include <math.h>

#include "smartsens.h"
#include "xmisp_priv.h"

#define REG_LINES_HI 0x3e01
#define REG_LINES_LO 0x3e02
#define REG_COARSE 0x3e08
#define REG_FINE 0x3e09
#define REG_GROUP 0x3812
#define GROUP_START 0x00
#define GROUP_END 0x30
#define REG_UPDATE 0x3903
#define REG_EXP_CTRL 0x3314 /* written with the exposure, always 0x02 */
#define FINE_ONE 0x10
#define FINE_MAX 0x1f

float xmisp_ss_gain(uint32_t coarse, uint32_t fine) {
  unsigned k = 0;
  for (coarse >>= 2; coarse & 1 && k < 5; coarse >>= 1)
    k++;
  if (fine < FINE_ONE)
    fine = FINE_ONE;
  return (float)(1u << k) * (float)(fine & FINE_MAX) / (float)FINE_ONE;
}

void xmisp_ss_gain_regs(float gain, unsigned coarse_max, uint32_t *coarse,
                        uint32_t *fine) {
  unsigned k = 0;
  while (k < coarse_max && gain >= (float)(2u << k))
    k++;
  int f = (int)roundf(gain / (float)(1u << k) * (float)FINE_ONE);
  if (f > FINE_MAX && k < coarse_max) {
    k++;
    f = FINE_ONE; /* rounds up into the next doubling */
  }
  f = f < FINE_ONE ? FINE_ONE : f > FINE_MAX ? FINE_MAX : f;
  *coarse = (4u << k) - 1;
  *fine = (uint32_t)f;
}

int xmisp_ss_get(struct xmisp *isp, uint32_t *lines, float *gain) {
  uint32_t hi, lo, c, f;
  if (xmisp_sensor_read(isp, REG_LINES_HI, &hi) < 0 ||
      xmisp_sensor_read(isp, REG_LINES_LO, &lo) < 0 ||
      xmisp_sensor_read(isp, REG_COARSE, &c) < 0 ||
      xmisp_sensor_read(isp, REG_FINE, &f) < 0)
    return -1;
  *lines = hi << 4 | lo >> 4;
  *gain = xmisp_ss_gain(c, f);
  return 0;
}

/* The writes of the range a gain falls in. */
static int ranges_write(struct xmisp *isp, const struct xmisp_ss_range *r,
                        unsigned n, float gain) {
  if (!n)
    return 0;
  unsigned i = 0;
  while (i + 1 < n && gain >= r[i + 1].from)
    i++;
  for (unsigned w = 0; w < XMISP_SS_WRITES && r[i].reg[w]; w++)
    if (xmisp_sensor_write(isp, r[i].reg[w], r[i].val[w]) < 0)
      return -1;
  return 0;
}

static int update(struct xmisp *isp) {
  return xmisp_sensor_write(isp, REG_UPDATE, 0x84) < 0 ||
                 xmisp_sensor_write(isp, REG_UPDATE, 0x04) < 0
             ? -1
             : 0;
}

int xmisp_ss_set(struct xmisp *isp, const struct xmisp_ss *ss, uint32_t lines,
                 float *gain) {
  uint32_t c, f;
  xmisp_ss_gain_regs(*gain, ss->coarse_max, &c, &f);
  const float g = xmisp_ss_gain(c, f);
  if (update(isp) < 0 || xmisp_sensor_write(isp, REG_EXP_CTRL, 0x02) < 0 ||
      xmisp_sensor_write(isp, REG_LINES_HI, (lines >> 4) & 0xff) < 0 ||
      xmisp_sensor_write(isp, REG_LINES_LO, (lines << 4) & 0xf0) < 0 ||
      update(isp) < 0 || xmisp_sensor_write(isp, REG_GROUP, GROUP_START) < 0 ||
      ranges_write(isp, ss->analog, ss->n_analog, g) < 0 ||
      xmisp_sensor_write(isp, REG_GROUP, GROUP_END) < 0 ||
      xmisp_sensor_write(isp, REG_COARSE, c) < 0 ||
      xmisp_sensor_write(isp, REG_FINE, f) < 0 ||
      ranges_write(isp, ss->post, ss->n_post, g) < 0)
    return -1;
  *gain = g;
  return 0;
}
