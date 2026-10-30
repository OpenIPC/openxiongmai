// SPDX-License-Identifier: MIT
/*
 * Host test of the sensor 3A drivers on a mock bus. For the sensors
 * measured only under emulation, each driver must write the codes the
 * vendor daemon's 3A wrote for the gains they encode, with the analogue
 * settings it wrote alongside them, and the exposure registers it wrote at
 * its longest exposure (qemu-bench traces, the vendor's dark ramp). For
 * every driver, what it writes reads back, and its gain steps rise with
 * the gain asked for and stay near it. Run: make -C libraries/libxmisp test
 */
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "../script.h"
#include "../xmisp.h"
#include "../xmisp_priv.h"

static uint16_t regs[0x10000]; /* AR0130 data is 16-bit */
static unsigned writes;
static uint32_t fail_reg; /* a register whose writes fail; 0 none */

int xmisp_sensor_write(struct xmisp *isp, uint32_t reg, uint32_t val) {
  if (fail_reg && reg == fail_reg)
    return -1;
  regs[reg & 0xffff] = (uint16_t)val;
  writes++;
  return 0;
}
int xmisp_sensor_read(struct xmisp *isp, uint32_t reg, uint32_t *val) {
  *val = regs[reg & 0xffff];
  return 0;
}

static int failures;

static void fail(const char *who, const char *fmt, double a, double b) {
  printf("FAIL: %s: ", who);
  printf(fmt, a, b);
  printf("\n");
  failures++;
}

/* A register and the value the vendor wrote; reg 0 ends a list. */
struct rv {
  uint16_t reg;
  uint8_t val;
};

/* Sets the gain that the codes in want encode, at lines; every register in
 * want must then hold its value. */
static void step(const char *who, const struct xmisp_sensor *sn, uint32_t lines,
                 const struct rv *want) {
  for (const struct rv *w = want; w->reg; w++)
    regs[w->reg] = w->val;
  uint32_t l;
  float g;
  if (sn->get_exposure(NULL, &l, &g) < 0) {
    fail(who, "get_exposure failed%.0f%.0f", 0, 0);
    return;
  }
  memset(regs, 0, sizeof(regs));
  float set = g;
  if (sn->set_exposure(NULL, lines, &set) < 0) {
    fail(who, "set_exposure failed%.0f%.0f", 0, 0);
    return;
  }
  if (fabsf(set - g) > 1e-4f)
    fail(who, "gain %.4f came back as %.4f", g, set);
  for (const struct rv *w = want; w->reg; w++)
    if (regs[w->reg] != w->val) {
      printf("FAIL: %s at %.4fx: 0x%04x = 0x%02x, the vendor wrote 0x%02x\n",
             who, g, w->reg, regs[w->reg], w->val);
      failures++;
    }
}

/* Over the gain range: the steps read back, rise, and stay within 7% of
 * the gain asked for (the coarsest step, 1/16 at the bottom of a
 * doubling, is 6.25%). */
static void sweep(const char *who, const struct xmisp_sensor *sn) {
  float last = 0.0f;
  for (float want = 1.0f; want <= sn->gain_max; want *= 1.01f) {
    memset(regs, 0, sizeof(regs));
    float g = want;
    uint32_t lines = sn->lines_max, l;
    float back;
    if (sn->set_exposure(NULL, lines, &g) < 0 ||
        sn->get_exposure(NULL, &l, &back) < 0) {
      fail(who, "exposure failed at %.3fx%.0f", want, 0);
      return;
    }
    if (l != lines)
      fail(who, "%.0f lines read back as %.0f", lines, l);
    if (fabsf(back - g) > 1e-4f)
      fail(who, "gain %.4f read back as %.4f", g, back);
    if (g < last)
      fail(who, "gain fell to %.4f after %.4f", g, last);
    if (fabsf(g / want - 1.0f) > 0.07f)
      fail(who, "%.4fx asked, %.4fx set", want, g);
    last = g;
  }
  if (last < sn->gain_max * 0.95f)
    fail(who, "reaches %.3fx of %.3fx", last, sn->gain_max);
}

/* The vendor daemon's dark ramps under emulation, step by step: the codes
 * it wrote, and the settings it had written by then. */
#define RV(...) ((const struct rv[]){__VA_ARGS__, {0, 0}})

static void soi(void) {
  static const uint8_t ramp[] = {0x06, 0x0e, 0x14, 0x1c, 0x23,
                                 0x2a, 0x32, 0x39, 0x40};
  for (unsigned i = 0; i < sizeof(ramp); i++) {
    step("JX-H42", &xmisp_sensor_h42, 748,
         RV({0x00, ramp[i]}, {0x01, 0xec}, {0x02, 0x02}));
    step("JX-H65 720p", &xmisp_sensor_h65_720p, 746,
         RV({0x00, ramp[i]}, {0x1f, ramp[i] < 0x20 ? 0x10 : 0x00}, {0x01, 0xea},
            {0x02, 0x02}));
  }
  /* 960p's own ramp, and its longest exposure */
  static const uint8_t ramp960[] = {0x06, 0x0f, 0x15, 0x1d, 0x24,
                                    0x2b, 0x32, 0x3a, 0x40};
  for (unsigned i = 0; i < sizeof(ramp960); i++)
    step("JX-H65 960p", &xmisp_sensor_h65_960p, 996,
         RV({0x00, ramp960[i]}, {0x1f, ramp960[i] < 0x20 ? 0x10 : 0x00},
            {0x01, 0xe4}, {0x02, 0x03}));
}

static void sc1235(void) {
  const struct xmisp_sensor *s = &xmisp_sensor_sc1235_720p;
  const char *n = "SC1235 720p";
  step(n, s, 746,
       RV({0x3e08, 0x03}, {0x3e09, 0x16}, {0x3631, 0x84}, {0x3301, 0x04},
          {0x3633, 0x2f}, {0x5781, 0x04}, {0x5785, 0x18}, {0x3e01, 0x2e},
          {0x3e02, 0xa0}, {0x3314, 0x02}));
  step(n, s, 746, RV({0x3e08, 0x03}, {0x3e09, 0x1e}, {0x3301, 0x04}));
  step(n, s, 746,
       RV({0x3e08, 0x07}, {0x3e09, 0x14}, {0x3631, 0x88}, {0x3301, 0x10},
          {0x3633, 0x23}));
  step(n, s, 746, RV({0x3e08, 0x07}, {0x3e09, 0x1c}, {0x3301, 0x10}));
  step(n, s, 746,
       RV({0x3e08, 0x0f}, {0x3e09, 0x13}, {0x3631, 0x88}, {0x3301, 0xb0},
          {0x3633, 0x43}));
  step(n, s, 746, RV({0x3e08, 0x0f}, {0x3e09, 0x1a}, {0x3301, 0xb0}));
  step(n, s, 746,
       RV({0x3e08, 0x1f}, {0x3e09, 0x12}, {0x3301, 0xb0}, {0x5781, 0x04},
          {0x5785, 0x18}));
  step(n, s, 746,
       RV({0x3e08, 0x1f}, {0x3e09, 0x19}, {0x5781, 0x02}, {0x5785, 0x08}));
  step(n, s, 746, RV({0x3e08, 0x1f}, {0x3e09, 0x1f}, {0x5781, 0x02}));

  s = &xmisp_sensor_sc1235_960p;
  n = "SC1235 960p";
  step(n, s, 996,
       RV({0x3e08, 0x03}, {0x3e09, 0x10}, {0x3631, 0x84}, {0x3301, 0x05},
          {0x3633, 0x2f}, {0x5781, 0x04}, {0x5785, 0x18}, {0x3e01, 0x3e},
          {0x3e02, 0x40}));
  step(n, s, 996, RV({0x3e08, 0x03}, {0x3e09, 0x1f}, {0x3301, 0x05}));
  step(n, s, 996,
       RV({0x3e08, 0x07}, {0x3e09, 0x15}, {0x3631, 0x88}, {0x3301, 0x1f},
          {0x3633, 0x23}));
  step(n, s, 996, RV({0x3e08, 0x07}, {0x3e09, 0x1d}, {0x3301, 0x1f}));
  step(n, s, 996,
       RV({0x3e08, 0x0f}, {0x3e09, 0x14}, {0x3631, 0x88}, {0x3301, 0xff},
          {0x3633, 0x43}));
  step(n, s, 996,
       RV({0x3e08, 0x1f}, {0x3e09, 0x12}, {0x3301, 0xff}, {0x5781, 0x04}));
  step(n, s, 996,
       RV({0x3e08, 0x1f}, {0x3e09, 0x1a}, {0x5781, 0x02}, {0x5785, 0x08}));
  /* 960p passed 9x and 13x only: at 12.5x it keeps the lower setting */
  step(n, s, 996, RV({0x3e08, 0x1f}, {0x3e09, 0x19}, {0x5781, 0x04}));
}

/* A write failing inside the SmartSens group hold still releases it. */
static void group_release(void) {
  memset(regs, 0, sizeof(regs));
  fail_reg = 0x3301;
  float g = 2.5f;
  if (xmisp_sensor_sc1235_720p.set_exposure(NULL, 746, &g) == 0)
    fail("SC1235", "set_exposure succeeded with 0x3301 failing%.0f%.0f", 0, 0);
  if (regs[0x3812] != 0x30)
    fail("SC1235", "group held after a failed write: 0x3812 = %.0f%.0f",
         regs[0x3812], 0);
  fail_reg = 0;
}

static void sc2145h(void) {
  const struct xmisp_sensor *s = &xmisp_sensor_sc2145h;
  const char *n = "SC2145H";
  step(n, s, 746,
       RV({0x3e08, 0x03}, {0x3e09, 0x16}, {0x3633, 0x57}, {0x3301, 0x04},
          {0x3631, 0x84}, {0x3620, 0x28}, {0x3e01, 0x2e}, {0x3e02, 0xa0}));
  step(n, s, 746,
       RV({0x3e08, 0x07}, {0x3e09, 0x14}, {0x3633, 0x43}, {0x3301, 0x08},
          {0x3631, 0x86}, {0x3620, 0x08}));
  step(n, s, 746, RV({0x3e08, 0x07}, {0x3e09, 0x1c}, {0x3301, 0x08}));
  step(n, s, 746,
       RV({0x3e08, 0x0f}, {0x3e09, 0x13}, {0x3633, 0x53}, {0x3301, 0x09}));
  step(n, s, 746,
       RV({0x3e08, 0x1f}, {0x3e09, 0x12}, {0x3633, 0x53}, {0x3301, 0x20}));
  step(n, s, 746, RV({0x3e08, 0x1f}, {0x3e09, 0x1f}, {0x3301, 0x20}));
}

static void sc2235(void) {
  const struct xmisp_sensor *s = &xmisp_sensor_sc2235;
  const char *n = "SC2235";
  step(n, s, 1121,
       RV({0x3e08, 0x03}, {0x3e09, 0x10}, {0x3631, 0x84}, {0x3301, 0x05},
          {0x366f, 0x2f}, {0x3e01, 0x46}, {0x3e02, 0x10}));
  step(n, s, 1121, RV({0x3e08, 0x03}, {0x3e09, 0x1b}, {0x3301, 0x05}));
  step(n, s, 1121,
       RV({0x3e08, 0x07}, {0x3e09, 0x12}, {0x3631, 0x88}, {0x3301, 0x15},
          {0x366f, 0x2f}));
  step(n, s, 1121, RV({0x3e08, 0x0f}, {0x3e09, 0x18}, {0x3301, 0x15}));
  step(n, s, 1121,
       RV({0x3e08, 0x1f}, {0x3e09, 0x10}, {0x3631, 0x88}, {0x3301, 0x19},
          {0x366f, 0x2f}));
  step(n, s, 1121, RV({0x3e08, 0x1f}, {0x3e09, 0x17}, {0x3301, 0x19}));
  step(n, s, 1121,
       RV({0x3e08, 0x1f}, {0x3e09, 0x1f}, {0x3631, 0x88}, {0x3301, 0xff},
          {0x366f, 0x3c}));
}

static void sc1135(void) {
  static const uint8_t ramp720[] = {0x08, 0x0e, 0x15, 0x1c, 0x34,
                                    0x3b, 0x72, 0x7a, 0x7f};
  static const uint8_t ramp960[] = {0x01, 0x09, 0x0f, 0x16, 0x1d,
                                    0x34, 0x3c, 0x73, 0x7b, 0x7f};
  for (unsigned i = 0; i < sizeof(ramp720); i++) {
    const int low = ramp720[i] < 0x10;
    step("SC1135 720p", &xmisp_sensor_sc1135_720p, 746,
         RV({0x3e09, ramp720[i]}, {0x3e08, 0x00}, {0x3630, low ? 0xb8 : 0x70},
            {0x3631, low ? 0x82 : 0x8e}, {0x3e01, 0x2e}, {0x3e02, 0xa0}));
  }
  for (unsigned i = 0; i < sizeof(ramp960); i++) {
    const int low = ramp960[i] < 0x10;
    step("SC1135 960p", &xmisp_sensor_sc1135_960p, 996,
         RV({0x3e09, ramp960[i]}, {0x3e08, 0x00}, {0x3630, low ? 0xb8 : 0x70},
            {0x3631, low ? 0x82 : 0x8e}, {0x3e01, 0x3e}, {0x3e02, 0x40}));
  }
}

static void ov(void) {
  static const uint16_t ramp9750[] = {0x086, 0x0b8, 0x0fd, 0x15b, 0x1de,
                                      0x291, 0x387, 0x4d9, 0x6aa, 0x7ff};
  for (unsigned i = 0; i < sizeof(ramp9750) / sizeof(*ramp9750); i++)
    step("OV9750", &xmisp_sensor_ov9750, 984,
         RV({0x3508, ramp9750[i] >> 8}, {0x3509, ramp9750[i] & 0xff},
            {0x3500, 0x00}, {0x3501, 0x3d}, {0x3502, 0x80}));
  static const uint8_t ramp9732[] = {0x16, 0x1e, 0x28, 0x38, 0x4c,
                                     0x68, 0x90, 0xc8, 0xf8};
  for (unsigned i = 0; i < sizeof(ramp9732); i++)
    step("OV9732", &xmisp_sensor_ov9732, 748,
         RV({0x350b, ramp9732[i]}, {0x3501, 0x2e}, {0x3502, 0xc0},
            {0x3208, 0xa0}));
}

int main(void) {
  soi();
  sc1235();
  group_release();
  sc2145h();
  sc2235();
  sc1135();
  ov();

  static const struct {
    const char *name;
    const struct xmisp_sensor *sn;
  } all[] = {
      {"JX-H62", &xmisp_sensor_h62},
      {"AR0130", &xmisp_sensor_ar0130},
      {"JX-H42", &xmisp_sensor_h42},
      {"JX-H65 720p", &xmisp_sensor_h65_720p},
      {"JX-H65 960p", &xmisp_sensor_h65_960p},
      {"SC1135 720p", &xmisp_sensor_sc1135_720p},
      {"SC1135 960p", &xmisp_sensor_sc1135_960p},
      {"SC1235 720p", &xmisp_sensor_sc1235_720p},
      {"SC1235 960p", &xmisp_sensor_sc1235_960p},
      {"SC2145H", &xmisp_sensor_sc2145h},
      {"SC2235", &xmisp_sensor_sc2235},
      {"OV9750", &xmisp_sensor_ov9750},
      {"OV9732", &xmisp_sensor_ov9732},
  };
  for (unsigned i = 0; i < sizeof(all) / sizeof(*all); i++)
    sweep(all[i].name, all[i].sn);

  /* Every sensor script with a driver keeps its frame: exposure within
   * it, and a line time that fits 25 fps. */
  for (const struct xmisp_script *const *s = xmisp_scripts; *s; s++) {
    const struct xmisp_sensor *sn = (*s)->sensor;
    if (!sn)
      continue;
    const double frame_lines = 1e9 / 25.0 / sn->line_ns;
    if (sn->lines_max >= frame_lines)
      fail((*s)->name, "%.0f lines in a %.1f-line frame", sn->lines_max,
           frame_lines);
  }

  if (failures)
    printf("%d failures\n", failures);
  else
    printf("sensor drivers: ok\n");
  return failures != 0;
}
