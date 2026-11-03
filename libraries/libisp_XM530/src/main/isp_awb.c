/* The open auto white balance (LIBISP_OPEN_AWB=1), in place of majestic's,
 * for the sensors whose calibration is understood: the IMX335 so far.
 *
 * Measured against the vendor AWB on the lab XM550 under 4000 K and
 * 3000-6000 K bulbs:
 *  - the grey zones are those whose R/G and B/G (Q12) lie inside the
 *    polygon of the sensor calibration's eight lines A x + B y + C = 0, on
 *    the side its initial gains lie on, less any zone with a channel near
 *    the clip;
 *  - the gains are the inverse of their summed R/G and B/G, G at 1x;
 *  - the colour temperature is a position along the calibration's table of
 *    distances: the grey point (Q10) projected at key / 512 of a turn, from
 *    (ini_y, ini_x), plus key (fitted to the vendor's matrices within 14
 *    on the IMX335, and so far checked on no other sensor);
 *  - the colour matrix is the sensor's, at its three temperatures, blended
 *    linearly in mired between the two around it and held at the ends.
 * The vendor's gains were within 1.7% of these at every temperature. With
 * too few grey zones (a coloured light) the gains are held, as the vendor
 * AWB holds them. */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "isp_main.h"
#include "mpi_awb.h"

#define DBG(fmt, ...) fprintf(stderr, "libisp: " fmt "\n", ##__VA_ARGS__)

/* sensors whose calibration this reads right (SENSOR_CHIP ids) */
static const SENSOR_ID known[] = {155 /* IMX335 */};

#define GREY_MIN 16     /* zones, fewer and the gains are held */
#define CLIP_LEVEL 3686 /* 0.9 of 12 bits: a channel's mean at the clip */
#define SMOOTH 0.05f    /* of the way to the measured gains, a frame */

static AWB_SENSOR_EXP_FUNC_S sns;
static int have_sns;
static AWB_SENSOR_DEFAULT_S dft;
static ISP_AWB_CALIBRATION_V2_S cal;
static int active;
static int inside_sign[8];
static float log_r, log_b; /* the gains set, natural log */
static int ct_now;

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

/* The colour temperature of a grey point (R/G, B/G) */
static int colour_temp(float r, float b) {
  const float th = 2 * (float)M_PI * cal.key / 512;
  const float d = (r * 1024 - cal.ini_y) * cosf(th) +
                  (b * 1024 - cal.ini_x) * sinf(th) + cal.key;
  int n = 0;
  while (n < 16 && (n == 0 || cal.dis[n] > cal.dis[n - 1]))
    n++;
  if (n < 2)
    return 5000;
  if (d <= cal.dis[0])
    return cal.val[0];
  for (int i = 0; i + 1 < n; i++)
    if (d <= cal.dis[i + 1])
      return (int)(cal.val[i] + (d - cal.dis[i]) /
                                    (cal.dis[i + 1] - cal.dis[i]) *
                                    (cal.val[i + 1] - cal.val[i]));
  return cal.val[n - 1];
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
  const char *e = getenv("LIBISP_OPEN_AWB");
  active = 0;
  if (!e || strcmp(e, "1"))
    return 0;
  const SENSOR_ID id = isp_sensor_id();
  int ok = 0;
  for (unsigned i = 0; i < sizeof(known) / sizeof(known[0]); i++)
    ok |= known[i] == id;
  if (!ok) {
    DBG("open AWB: sensor %d's calibration is not known yet; majestic's AWB",
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
  ct_now = colour_temp(r0, b0);
  active = 1;
  DBG("open AWB: start gains R %.2f B %.2f, %d K", expf(log_r), expf(log_b),
      ct_now);
  return 1;
}

/* One frame, on the zones read just before (sites: a zone's pixels of one
 * colour). 1 with new gains and matrix */
int open_awb_frame(XM_U32 sites, ISP_AWB_RESULT_GAIN_S *gain,
                   ISP_AWB_RESULT_CC_S *cc) {
  const XM_U32(*z)[3] = stats_zones();
  const double clip = (double)CLIP_LEVEL * (sites ? sites : 1);
  double sr = 0, sg = 0, sb = 0;
  int n = 0, clipped = 0;
  for (int i = 0; i < AWB_ZONES; i++) {
    const double R = z[i][0], G = z[i][1] / 2.0, B = z[i][2];
    if (G <= 0)
      continue;
    if (R >= clip || G >= clip || B >= clip) {
      clipped++;
      continue;
    }
    if (!inside((float)(R / G * 4096), (float)(B / G * 4096)))
      continue;
    sr += R, sg += G, sb += B, n++;
  }
  static XM_U32 frames;
  if (n >= GREY_MIN) {
    const float r = (float)(sr / sg), b = (float)(sb / sg);
    log_r += SMOOTH * (logf(1 / r) - log_r);
    log_b += SMOOTH * (logf(1 / b) - log_b);
    ct_now = colour_temp(expf(-log_r), expf(-log_b));
  }
  if (getenv("LIBISP_DEBUG") && ++frames % 50 == 0)
    DBG("open AWB: %d grey zones, %d clipped, gains R %.3f B %.3f, %d K", n,
        clipped, expf(log_r), expf(log_b), ct_now);
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
