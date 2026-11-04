/* The open auto white balance (system.openAwb), in place of majestic's,
 * for the sensors whose neutrals are measured: the IMX335 so far.
 *
 * Measured on the lab XM550 against a colour checker's grey patches under
 * 3000-5990 K bulbs and cloudy daylight, with an orange chair and wood in
 * view, which trap the vendor AWB (its grey-world over the calibration's
 * wide polygon read about 2850 K in daylight, and turned the chart's greys
 * blue; 35-40% off in R/G and B/G):
 *  - neutrals lie on a line in log R/G against log B/G, within 0.004 for
 *    the bulbs and 0.03 for daylight;
 *  - an orange surface in daylight lies on that line too, far out at the
 *    warm end, so a limit on how warm a neutral can be is needed as well as
 *    the distance from the line;
 *  - within the band the neutrals are the coolest group with support, a
 *    coloured surface only pulling warmer: sorted by R/G, the first window
 *    12% wide with at least 4 zones and a fifth of them. Its median R/G and
 *    B/G were within 1.5% of the chart at every light.
 * The gains are their inverse, G at 1x. With too few such zones (a coloured
 * light) the gains are held, as the vendor AWB holds them. The colour
 * temperature is the R/G's place among the measured neutrals, and the colour
 * matrix the sensor's, at its three temperatures, blended linearly in mired
 * between the two around it, as the vendor's are. */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "isp_main.h"
#include "mpi_awb.h"

#define DBG(fmt, ...) fprintf(stderr, "libisp: " fmt "\n", ##__VA_ARGS__)

/* A sensor's neutrals, against a chart: log(B/G) = slope log(R/G) + at0,
 * from slope and at0's points, R/G falling as the light gets bluer */
#define NEUTRALS 4
struct neutrals {
  SENSOR_ID id; /* SENSOR_CHIP */
  float slope, at0;
  float band_warm, band_cool; /* log B/G either side of the line */
  float r_min, r_max;         /* a neutral's R/G: r_max is about 2700 K */
  float r[NEUTRALS], kelvin[NEUTRALS];
};

static const struct neutrals sensors[] = {
    {155 /* IMX335 */,
     -0.800f,
     -1.187f,
     0.05f,
     0.10f,
     0.50f,
     0.90f,
     {0.810f, 0.729f, 0.677f, 0.644f},
     {3000, 4000, 5000, 5990}},
};
static const struct neutrals *sensor;

#define CLIP_LEVEL 3686 /* 0.9 of 12 bits: a channel's mean at the clip */
#define DARK_LEVEL 64   /* a G mean below it is too dark to count */
#define SMOOTH 0.05f    /* of the way to the measured gains, a frame */
/* frames after the start the gains hold, while the exposure settles: dark
 * or clipped first frames pulled them away and back */
#define START_HOLD 25
#define GROUP_WIDTH 0.12f /* log R/G, the coolest group's */
#define GROUP_MIN 4       /* its zones, and at least a fifth of them */

static AWB_SENSOR_EXP_FUNC_S sns;
static int have_sns;
static AWB_SENSOR_DEFAULT_S dft;
static ISP_AWB_CALIBRATION_V2_S cal;
static int active;
static int inside_sign[8];
static float log_r, log_b; /* the gains set, natural log */
static int ct_now;

/* The gains it set, kept in the block /dev/isp shares, which outlives the
 * process: majestic's start resets the ISP (its VI init), so a restart
 * starts from these. Past the AE statistics the block keeps at 0x2000. */
#define SHM_AWB_OPEN 0x3f00
#define AWB_MAGIC 0x4f415742 /* "OAWB" */
struct kept {
  XM_U32 magic;
  float log_r, log_b;
};

static struct kept *kept(void) {
  char *shm = isp_shm();
  return shm ? (struct kept *)(shm + SHM_AWB_OPEN) : NULL;
}

__attribute__((visibility("default"))) XM_S32
libisp_awb_sensor_register(ISP_DEV IspDev, SENSOR_ID SensorId,
                           const AWB_SENSOR_REGISTER_S *pstRegister) {
  (void)IspDev;
  (void)SensorId;
  if (!pstRegister)
    return XM_FAILURE;
  sns = pstRegister->stSnsExp;
  have_sns = 1;
  return XM_SUCCESS;
}

int open_awb_active(void) { return active; }

/* The side of each line a point (Q12 R/G, B/G) is on */
static void sides(float r, float b, int out[8]) {
  for (int k = 0; k < 8; k++)
    out[k] = cal.A[k] * r + cal.B[k] * b + cal.C[k] * 4096.0f > 0;
}

static int inside(float r, float b) {
  int s[8];
  sides(r, b, s);
  return !memcmp(s, inside_sign, sizeof(s));
}

/* The colour temperature of a neutral's R/G: linear in mired against log
 * R/G between the measured neutrals, and on past the ends */
static int colour_temp(float r) {
  const struct neutrals *n = sensor;
  const float l = logf(r);
  int i = 0;
  while (i < NEUTRALS - 2 && r < n->r[i + 1])
    i++;
  const float l0 = logf(n->r[i]), l1 = logf(n->r[i + 1]);
  const float m0 = 1e6f / n->kelvin[i], m1 = 1e6f / n->kelvin[i + 1];
  const float mired = m0 + (l - l0) / (l1 - l0) * (m1 - m0);
  return mired < 50 ? 20000 : (int)(1e6f / mired);
}

/* The sensor's matrix at ct: blended in mired between the two around it */
static void matrix(int ct, XM_U16 out[12]) {
  const ISP_COLORMATRIX_AUTO_S *m = &dft.stCcm;
  const XM_U16 *lo, *hi;
  float tlo, thi;
  if (ct >= m->u16HighColorTemp) {
    memcpy(out, m->au16HighCCM, sizeof(m->au16HighCCM));
    return;
  }
  if (ct <= m->u16LowColorTemp) {
    memcpy(out, m->au16LowCCM, sizeof(m->au16LowCCM));
    return;
  }
  if (ct >= m->u16MidColorTemp) {
    lo = m->au16MidCCM, hi = m->au16HighCCM;
    tlo = m->u16MidColorTemp, thi = m->u16HighColorTemp;
  } else {
    lo = m->au16LowCCM, hi = m->au16MidCCM;
    tlo = m->u16LowColorTemp, thi = m->u16MidColorTemp;
  }
  const float t = (1e6f / ct - 1e6f / tlo) / (1e6f / thi - 1e6f / tlo);
  for (int i = 0; i < 12; i++) {
    const float a = (XM_S16)lo[i], c = (XM_S16)hi[i];
    out[i] = (XM_U16)(XM_S16)lroundf(a + t * (c - a));
  }
}

int open_awb_init(void) {
  active = 0;
  if (!open_3a_wanted("LIBISP_OPEN_AWB", "openAwb"))
    return 0;
  const SENSOR_ID id = isp_sensor_id();
  sensor = NULL;
  for (unsigned i = 0; i < sizeof(sensors) / sizeof(sensors[0]); i++)
    if (sensors[i].id == id)
      sensor = &sensors[i];
  if (!sensor) {
    DBG("open AWB: sensor %d's neutrals are not measured yet; majestic's AWB",
        (int)id);
    return 0;
  }
  if (!have_sns || !sns.pfn_cmos_get_awb_default) {
    DBG("open AWB: the sensor library gave no AWB callbacks; majestic's AWB");
    return 0;
  }
  memset(&dft, 0, sizeof(dft));
  if (sns.pfn_cmos_get_awb_default(&dft) || !dft.pstRbTable) {
    DBG("open AWB: the sensor's AWB defaults failed; majestic's AWB");
    return 0;
  }
  memcpy(&cal, dft.pstRbTable, sizeof(cal));
  if (!cal.init_gain[0] || !cal.init_gain[2]) {
    DBG("open AWB: the sensor's calibration has no initial gains; majestic's "
        "AWB");
    return 0;
  }
  /* inside is the initial gains' grey point's side of every line */
  const float r0 = 1024.0f / cal.init_gain[0], b0 = 1024.0f / cal.init_gain[2];
  sides(r0 * 4096, b0 * 4096, inside_sign);
  log_r = logf(cal.init_gain[0] / 1024.0f);
  log_b = logf(cal.init_gain[2] / 1024.0f);
  /* After a restart, from the gains the last run kept, if they are sane
   * (0.5x to 8x); the calibration's on a cold start */
  const struct kept *k = kept();
  const char *from = "the calibration";
  if (k && k->magic == AWB_MAGIC && fabsf(k->log_r) < 2.08f &&
      fabsf(k->log_b) < 2.08f) {
    log_r = k->log_r;
    log_b = k->log_b;
    from = "the last run";
  }
  ct_now = colour_temp(expf(-log_r));
  active = 1;
  DBG("open AWB: start gains R %.2f B %.2f, %d K, from %s", expf(log_r),
      expf(log_b), ct_now, from);
  return 1;
}

static int by_float(const void *a, const void *b) {
  const float x = *(const float *)a, y = *(const float *)b;
  return x < y ? -1 : x > y;
}

struct grey {
  float r, b;
};

static int by_r(const void *a, const void *b) {
  return by_float(&((const struct grey *)a)->r, &((const struct grey *)b)->r);
}

/* The coolest group with support, of n zones sorted by R/G: 0 for none */
static int coolest(const struct grey *g, int n, int *first) {
  const int need = n / 5 > GROUP_MIN ? n / 5 : GROUP_MIN;
  const float width = expf(GROUP_WIDTH);
  for (int i = 0, j = 0; i < n; i++) {
    while (j < n && g[j].r <= g[i].r * width)
      j++;
    if (j - i >= need) {
      *first = i;
      return j - i;
    }
  }
  return 0;
}

/* One frame, on the zones read just before (sites: a zone's pixels of one
 * colour). 1 with new gains and matrix */
int open_awb_frame(XM_U32 sites, ISP_AWB_RESULT_GAIN_S *gain,
                   ISP_AWB_RESULT_CC_S *cc) {
  const XM_U32(*z)[3] = stats_zones();
  const double clip = (double)CLIP_LEVEL * (sites ? sites : 1);
  const double dark = (double)DARK_LEVEL * (sites ? sites : 1);
  static struct grey g[AWB_ZONES];
  static float bs[AWB_ZONES];
  int n = 0, clipped = 0;
  for (int i = 0; i < AWB_ZONES; i++) {
    const double R = z[i][0], G = z[i][1] / 2.0, B = z[i][2];
    if (G <= 0)
      continue;
    if (R >= clip || G >= clip || B >= clip) {
      clipped++;
      continue;
    }
    if (G < dark || R <= 0 || B <= 0)
      continue;
    const float r = (float)(R / G), b = (float)(B / G);
    if (r < sensor->r_min || r > sensor->r_max || !inside(r * 4096, b * 4096))
      continue;
    const float d = logf(b) - (sensor->slope * logf(r) + sensor->at0);
    if (d < -sensor->band_warm || d > sensor->band_cool)
      continue;
    g[n++] = (struct grey){r, b};
  }
  static XM_U32 frames;
  ++frames;
  int first = 0, group = 0;
  if (frames > START_HOLD) {
    qsort(g, n, sizeof(g[0]), by_r);
    group = coolest(g, n, &first);
  }
  if (group) {
    for (int i = 0; i < group; i++)
      bs[i] = g[first + i].b;
    qsort(bs, group, sizeof(bs[0]), by_float);
    const float r = g[first + group / 2].r, b = bs[group / 2];
    log_r += SMOOTH * (logf(1 / r) - log_r);
    log_b += SMOOTH * (logf(1 / b) - log_b);
    ct_now = colour_temp(expf(-log_r));
    struct kept *kp = kept();
    if (kp) {
      kp->log_r = log_r;
      kp->log_b = log_b;
      kp->magic = AWB_MAGIC;
    }
  }
  if (getenv("LIBISP_DEBUG") && frames % 50 == 0)
    DBG("open AWB: %d neutral zones, group %d, %d clipped, gains R %.3f B "
        "%.3f, %d K",
        n, group, clipped, expf(log_r), expf(log_b), ct_now);
  gain->au32WhiteBalanceGain[0] = (XM_U32)lroundf(expf(log_r) * 256);
  gain->au32WhiteBalanceGain[1] = 256;
  gain->au32WhiteBalanceGain[2] = (XM_U32)lroundf(expf(log_b) * 256);
  XM_U16 m[12];
  matrix(ct_now, m);
  if (memcmp(m, cc->au16ColorMatrix, sizeof(m))) {
    memcpy(cc->au16ColorMatrix, m, sizeof(m));
    cc->bChange = XM_TRUE;
  }
  return 1;
}
