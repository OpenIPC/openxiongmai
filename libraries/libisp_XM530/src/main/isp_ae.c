/* The open auto-exposure (system.openAe), in place of majestic's.
 *
 * It holds what the vendor AE holds, measured on the lab XM550 (IMX335):
 * the mean of the 7 x 7 luma grid, weighted by the AE attributes' table,
 * over 256 (the "average luminance" the AE reports, 0-255), within the
 * attributes' tolerance of their compensation. Majestic moves the
 * compensation with the light (55 in a lit room, 51 in a dark one), and
 * the attributes are read every frame, so it follows.
 *
 * The exposure goes to the sensor through the callbacks its library
 * registers (libisp_ae_sensor_register): lines first, then the sensor's
 * analog gain, its digital gain, and the ISP's for the rest, each taken to
 * the sensor's nearest step. libxmisp's ae.c moves lines x gain.
 *
 * The result goes where the vendor AE keeps its own in its state buffer,
 * which majestic's inner-state query reads (its metrics, automatic
 * day/night and the compensation it sets): lines, the three gains and the
 * average luminance. */
#include <dlfcn.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#pragma GCC visibility push(hidden)
#include "ae.h" /* libxmisp's, built in (xmisp_ae.c) */
#pragma GCC visibility pop
#include "isp_main.h"
#include "mpi_ae.h"

/* The vendor AE's state, found by comparing it with the inner-state query
 * in two lights: lines and the gains (1024 is 1x) as words, the average
 * luminance as a byte */
#define ST_LINES 0xd0
#define ST_AGAIN 0xd4
#define ST_DGAIN 0xd8
#define ST_ISPDGAIN 0xdc
#define ST_AVELUM 0x109
#define ST_AT_MAX 0x10c     /* a byte: no exposure left */
#define ST_EXPOSURE 0xe0    /* lines x the three gains, Q10: what it reports */
#define ST_AGAIN_STEP 0x610 /* the analog gain as the sensor's table step */
#define ST_LINES2 0x60c     /* lines again */

/* frames from a change until the statistics show it */
#define AE_INTERVAL 2

#define DBG(fmt, ...) fprintf(stderr, "libisp: " fmt "\n", ##__VA_ARGS__)

static AE_SENSOR_EXP_FUNC_S sns;
static int have_sns;
static AE_SENSOR_DEFAULT_S dft;
static struct xmisp_ae ae;
static int active;
static XM_S32 (*get_attr)(ISP_DEV, ISP_EXPOSURE_ATTR_S *);
static XM_U32 again_now = 1024, dgain_now = 1024, ispdgain_now = 1024;
static XM_U32 again_step;

__attribute__((visibility("default"))) XM_S32
libisp_ae_sensor_register(ISP_DEV IspDev, SENSOR_ID SensorId,
                          const AE_SENSOR_REGISTER_S *pstRegister) {
  (void)IspDev;
  (void)SensorId;
  if (!pstRegister)
    return XM_FAILURE;
  sns = pstRegister->stSnsExp;
  have_sns = 1;
  return XM_SUCCESS;
}

int open_ae_active(void) { return active; }

static XM_U32 st_word(const void *state, XM_U32 off) {
  XM_U32 v;
  memcpy(&v, (const XM_U8 *)state + off, sizeof(v));
  return v;
}

static void st_put(void *state, XM_U32 off, XM_U32 v) {
  memcpy((XM_U8 *)state + off, &v, sizeof(v));
}

static XM_U32 umin(XM_U32 a, XM_U32 b) { return a < b ? a : b; }

/* The limits of each gain, Q10: the sensor's, within the attributes' */
struct limits {
  XM_U32 again, dgain, ispdgain, system;
};

static struct limits limits(const ISP_EXPOSURE_ATTR_S *a) {
  const XM_U32 shift = dft.u32ISPDgainShift ? dft.u32ISPDgainShift : 8;
  struct limits l = {
      .again = dft.u32MaxAgain ? dft.u32MaxAgain : 1024,
      .dgain = dft.u32MaxDgain ? dft.u32MaxDgain : 1024,
      .ispdgain = dft.u32MaxISPDgain ? dft.u32MaxISPDgain << 10 >> shift : 1024,
  };
  l.system = 0;
  if (a) {
    const ISP_AE_ATTR_S *u = &a->stAuto;
    if (u->stAGainRange.u32Max >= 1024)
      l.again = umin(l.again, u->stAGainRange.u32Max);
    if (u->stDGainRange.u32Max >= 1024)
      l.dgain = umin(l.dgain, u->stDGainRange.u32Max);
    if (u->stISPDGainRange.u32Max >= 1024)
      l.ispdgain = umin(l.ispdgain, u->stISPDGainRange.u32Max);
    /* the most gain of all: 128x on the lab IMX335, where the vendor AE
     * stopped, with the ISP's at 2.09x of its 8x */
    if (u->stSysGainRange.u32Max >= 1024)
      l.system = u->stSysGainRange.u32Max;
  }
  return l;
}

static float gain_max(const struct limits *l) {
  const float g = (float)l->again / 1024 * ((float)l->dgain / 1024) *
                  ((float)l->ispdgain / 1024);
  const float sys = (float)l->system / 1024;
  return l->system && sys < g ? sys : g;
}

/* Lines and a total gain to the sensor and the ISP, each at its nearest
 * step; *applied is the gain all of them took */
static void apply(XM_U32 lines, float gain, const struct limits *l,
                  float *applied) {
  AE_SENSOR_GAININFO_S ag = {.u32SnsTimes = 1024}, dg = {.u32SnsTimes = 1024};
  float want = gain * 1024;
  XM_U32 a = (XM_U32)(want < l->again ? want : l->again);
  if (sns.pfn_cmos_again_calc_table)
    sns.pfn_cmos_again_calc_table(a < 1024 ? 1024 : a, &ag);
  if (!ag.u32SnsTimes)
    ag.u32SnsTimes = 1024;
  want = gain * 1024 * 1024 / ag.u32SnsTimes;
  XM_U32 d = (XM_U32)(want < l->dgain ? want : l->dgain);
  if (sns.pfn_cmos_dgain_calc_table)
    sns.pfn_cmos_dgain_calc_table(d < 1024 ? 1024 : d, &dg);
  if (!dg.u32SnsTimes)
    dg.u32SnsTimes = 1024;
  want = gain * 1024 * 1024 * 1024 / ((float)ag.u32SnsTimes * dg.u32SnsTimes);
  XM_U32 isp = (XM_U32)(want + 0.5f);
  if (isp < 1024)
    isp = 1024;
  if (isp > l->ispdgain)
    isp = l->ispdgain;
  if (sns.pfn_cmos_inttime_update)
    sns.pfn_cmos_inttime_update(lines);
  if (sns.pfn_cmos_gains_update)
    sns.pfn_cmos_gains_update(ag.u32GainDb, dg.u32GainDb);
  again_now = ag.u32SnsTimes;
  again_step = ag.u32GainDb;
  dgain_now = dg.u32SnsTimes;
  ispdgain_now = isp;
  *applied = (float)again_now / 1024 * ((float)dgain_now / 1024) *
             ((float)ispdgain_now / 1024);
}

/* Whether the sensor's gains are in the units the open AE drives: 1024 is
 * 1x, in its limits and in what its tables return. Not every sensor
 * library's are (Auge's digital gain takes 128 for 1x, and its analog
 * table returns 32 times its input): those keep majestic's AE until their
 * units are known. */
static int gains_in_q10(void) {
  AE_SENSOR_GAININFO_S g;
  if (dft.u32MinAgain != 1024 || (dft.u32MinDgain && dft.u32MinDgain != 1024))
    return 0;
  if (sns.pfn_cmos_again_calc_table) {
    memset(&g, 0, sizeof(g));
    sns.pfn_cmos_again_calc_table(1024, &g);
    if (g.u32SnsTimes != 1024)
      return 0;
    if (dft.u32MaxAgain >= 2048) {
      memset(&g, 0, sizeof(g));
      sns.pfn_cmos_again_calc_table(2048, &g);
      if (g.u32SnsTimes < 1800 || g.u32SnsTimes > 2048)
        return 0;
    }
  }
  if (sns.pfn_cmos_dgain_calc_table && dft.u32MaxDgain > 1024) {
    memset(&g, 0, sizeof(g));
    sns.pfn_cmos_dgain_calc_table(1024, &g);
    if (g.u32SnsTimes != 1024)
      return 0;
  }
  return 1;
}

static XM_U8 fps_now;

/* The lines the exposure may have: the sensor's at its frame rate, within
 * the attributes' exposure time (microseconds) */
static void line_range(const ISP_EXPOSURE_ATTR_S *a, XM_U32 *lo, XM_U32 *hi) {
  *lo = dft.u32MinIntTime ? dft.u32MinIntTime : 1;
  *hi = dft.u32MaxIntTime;
  if (!a || !dft.u32LinesPer500ms)
    return;
  const ISP_AE_RANGE_S *t = &a->stAuto.stExpTimeRange;
  const XM_U64 per = dft.u32LinesPer500ms; /* lines in 500000 us */
  const XM_U32 tlo = (XM_U32)((t->u32Min * per + 499999) / 500000);
  const XM_U32 thi = (XM_U32)(t->u32Max * per / 500000);
  if (tlo > *lo && tlo <= *hi)
    *lo = tlo;
  if (thi < *hi && thi >= *lo)
    *hi = thi;
}

/* The target: the attributes' compensation, the sensor's for none */
static float target_of(const ISP_EXPOSURE_ATTR_S *a) {
  if (a && a->stAuto.u8Compensation)
    return a->stAuto.u8Compensation;
  return dft.u8AeCompensation ? dft.u8AeCompensation : 50;
}

/* Take the exposure up from what the state holds (the last run's, or what
 * majestic's AE set while it had the exposure) */
static void take_over(void *state, const struct limits *l) {
  XM_U32 lines = st_word(state, ST_LINES);
  float g = (float)st_word(state, ST_AGAIN) / 1024 *
            ((float)st_word(state, ST_DGAIN) / 1024) *
            ((float)st_word(state, ST_ISPDGAIN) / 1024);
  if (lines < ae.cfg.lines_min || lines > ae.cfg.lines_max)
    lines = ae.cfg.lines_max;
  if (!(g >= 1.0f && g <= ae.cfg.gain_max))
    g = 1.0f;
  float applied;
  apply(lines, g, l, &applied);
  xmisp_ae_init(&ae, &ae.cfg, lines, applied);
}

int open_ae_init(XM_U8 fps, void *state) {
  active = 0;
  if (!open_3a_wanted("LIBISP_OPEN_AE", "openAe"))
    return 0;
  if (!have_sns || !sns.pfn_cmos_get_ae_default ||
      !sns.pfn_cmos_inttime_update || !sns.pfn_cmos_gains_update) {
    DBG("open AE: the sensor library gave no AE callbacks; majestic's AE");
    return 0;
  }
  get_attr = (XM_S32 (*)(ISP_DEV, ISP_EXPOSURE_ATTR_S *))dlsym(
      RTLD_DEFAULT, "XM_MPI_ISP_GetExposureAttr");
  memset(&dft, 0, sizeof(dft));
  if (sns.pfn_cmos_get_ae_default(&dft)) {
    DBG("open AE: the sensor's AE defaults failed; majestic's AE");
    return 0;
  }
  if (sns.pfn_cmos_fps_set && fps)
    sns.pfn_cmos_fps_set(fps, &dft);
  fps_now = fps;
  if (!gains_in_q10()) {
    DBG("open AE: the sensor's gains are not in 1024-for-1x units (analog "
        "%u-%u, digital %u-%u); majestic's AE",
        dft.u32MinAgain, dft.u32MaxAgain, dft.u32MinDgain, dft.u32MaxDgain);
    return 0;
  }
  ISP_EXPOSURE_ATTR_S attr, *a = NULL;
  if (get_attr && get_attr(0, &attr) == XM_SUCCESS)
    a = &attr;
  const struct limits l = limits(a);
  struct xmisp_ae_cfg cfg = {
      .target = target_of(a),
      .tolerance = 0.05f,
      .gain_max = gain_max(&l),
      .interval = AE_INTERVAL,
  };
  line_range(a, &cfg.lines_min, &cfg.lines_max);
  if (!cfg.lines_max || cfg.lines_max < cfg.lines_min) {
    DBG("open AE: the sensor's lines %u-%u are no range; majestic's AE",
        cfg.lines_min, cfg.lines_max);
    return 0;
  }
  ae.cfg = cfg;
  take_over(state, &l);
  active = 1;
  DBG("open AE: lines %u-%u, gain to %.1fx, target %.0f", cfg.lines_min,
      cfg.lines_max, cfg.gain_max, cfg.target);
  return 1;
}

/* One frame: the grid in, the ISP gain and ISO out (res), the state for
 * majestic. 0 for a frame majestic's AE is to run: manual or bypassed
 * exposure, which it does itself; the open AE takes the exposure up again
 * from what it left when they end. */
int open_ae_frame(const ISP_AE_INFO_S *info, void *state,
                  ISP_AE_RESULT_S *res) {
  static int handed;
  ISP_EXPOSURE_ATTR_S attr, *a = NULL;
  if (get_attr && get_attr(0, &attr) == XM_SUCCESS)
    a = &attr;
  if (a && (a->bByPass || a->enOpType == OP_TYPE_MANUAL)) {
    if (!handed++)
      DBG("open AE: %s exposure: majestic's AE has it",
          a->bByPass ? "bypassed" : "manual");
    return 0;
  }
  const struct limits l = limits(a);

  /* A new frame rate: the sensor's lines and flicker period with it */
  const XM_U8 fps = isp_pub()->u8FrameRate;
  if (fps && fps != fps_now && sns.pfn_cmos_fps_set) {
    sns.pfn_cmos_fps_set(fps, &dft);
    fps_now = fps;
  }
  ae.cfg.gain_max = gain_max(&l);
  line_range(a, &ae.cfg.lines_min, &ae.cfg.lines_max);
  ae.cfg.target = target_of(a);
  ae.cfg.tolerance = a && a->stAuto.u8Tolerance
                         ? (float)a->stAuto.u8Tolerance / ae.cfg.target
                         : 0.05f;
  if (a) {
    const ISP_ANTIFLICKER_S *f = &a->stAuto.stAntiflicker;
    xmisp_ae_flicker(&ae, f->bEnable && f->u8Frequency
                              ? (float)dft.u32LinesPer500ms / f->u8Frequency
                              : 0);
  }
  if (handed) {
    handed = 0;
    DBG("open AE: automatic exposure again");
    take_over(state, &l);
  }
  /* An exposure the limits no longer allow is planned again, even within
   * the tolerance */
  if (ae.lines < ae.cfg.lines_min || ae.lines > ae.cfg.lines_max ||
      ae.gain > ae.cfg.gain_max * 1.001f)
    ae.replan = 1;

  /* the weighted mean, as the vendor AE's average luminance */
  XM_U64 sum = 0, wsum = 0;
  for (int r = 0; r < AE_ZONE_ROW; r++)
    for (int c = 0; c < AE_ZONE_COLUMN; c++) {
      const XM_U32 w = a ? a->stAuto.au8Weight[r][c] : 1;
      sum += (XM_U64)w * info->stAeStat2.au32MeteringWin[r][c];
      wsum += w;
    }
  const float mean = wsum ? (float)sum / wsum / 256 : 0;

  XM_U32 lines;
  float gain, applied;
  static XM_U32 frames;
  if (xmisp_ae_update(&ae, mean, &lines, &gain)) {
    apply(lines, gain, &l, &applied);
    xmisp_ae_applied(&ae, lines, applied);
  }
  if (getenv("LIBISP_DEBUG") && ++frames % 50 == 0)
    DBG("open AE: mean %.1f target %.0f tol %.2f, lines %u (%u-%u) gain "
        "%.2f (again %u dgain %u isp %u), max %.1f",
        mean, ae.cfg.target, ae.cfg.tolerance, ae.lines, ae.cfg.lines_min,
        ae.cfg.lines_max, ae.gain, again_now, dgain_now, ispdgain_now,
        ae.cfg.gain_max);
  const float total = ae.gain;
  res->u32IspDgain = ispdgain_now >> 2; /* 8.8 */
  res->u32IspDgainShift = 8;
  res->u32Iso = (XM_U32)(total * 100 + 0.5f);

  st_put(state, ST_LINES, ae.lines);
  st_put(state, ST_LINES2, ae.lines);
  st_put(state, ST_AGAIN, again_now);
  st_put(state, ST_DGAIN, dgain_now);
  st_put(state, ST_ISPDGAIN, ispdgain_now);
  st_put(state, ST_EXPOSURE,
         (XM_U32)((XM_U64)ae.lines * again_now * dgain_now / 1024 *
                  ispdgain_now / 1024));
  st_put(state, ST_AGAIN_STEP, again_step);
  ((XM_U8 *)state)[ST_AVELUM] = (XM_U8)(mean > 255 ? 255 : mean + 0.5f);
  ((XM_U8 *)state)[ST_AT_MAX] =
      ae.lines >= ae.cfg.lines_max && ae.gain >= ae.cfg.gain_max * 0.99f;
  return 1;
}
