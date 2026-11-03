/* The XM_MPI_ISP_* calls the ISP application, the sensor libraries and
 * majestic make.
 *
 * Start-up replays the vendor library's writes for the sensor and mode
 * (arch/xm530), with the 3D noise reduction's buffers in an "Isp" MMZ block
 * of our own. The attributes are kept and returned; the per-frame tuning
 * that the vendor library makes from them is still to come, so the start-up
 * values stay.
 */
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include "isp_main.h"
#include "mpi_sys.h"

/* majestic's AE library: the sensor's frame rate, with the MIPI receiver
 * stopped around the change */
XM_S32 XM_MPI_AE_SetFrameRate(ISP_DEV IspDev, XM_U8 u8Fps);

#define DBG(fmt, ...) fprintf(stderr, "libisp: " fmt "\n", ##__VA_ARGS__)

/* the "Isp" block and the start-up's geometry, from Init */
static XM_U32 geo[XM530_GEO_NUM];
/* what the vendor library has built in for this sensor, if anything */
static const struct xm530_sensor *sns_start;
/* the attributes, set and read from several threads */
static pthread_mutex_t attr_lock = PTHREAD_MUTEX_INITIALIZER;
static ISP_PUB_ATTR_S pub;
static ISP_SENSOR_REGISTER_S sensor;
static SENSOR_ID sensor_id = -1;
static int mem_init, init_done, run_started;
static XM_U32 mmz_phys;
static XM_VOID *mmz_virt;
static XM_U32 vsync_saved;

static void attr_start(const struct xm530_sensor *s);
static void attr_gamma(void);
static void attr_snapshot(struct isp_tune_attrs *a);

const ISP_PUB_ATTR_S *isp_pub(void) { return &pub; }
/* the Run loop's tuning reads it while the sensor library may register */
SENSOR_ID isp_sensor_id(void) {
  return __atomic_load_n(&sensor_id, __ATOMIC_RELAXED);
}

XM_S32 XM_MPI_ISP_MemInit(ISP_DEV IspDev) {
  /* majestic calls it again later, from another thread */
  if (mem_init)
    return XM_SUCCESS;
  if (isp_open() < 0) {
    DBG("cannot open /dev/isp");
    return XM_FAILURE;
  }
  if (isp_3a_register(IspDev) != XM_SUCCESS)
    return XM_FAILURE;
  mem_init = 1;
  return XM_SUCCESS;
}

XM_S32 XM_MPI_ISP_SensorRegCallBack(SENSOR_ID SensorId,
                                    const ISP_SENSOR_REGISTER_S *pstRegister) {
  if (!pstRegister)
    return XM_FAILURE;
  sensor = *pstRegister;
  __atomic_store_n(&sensor_id, SensorId, __ATOMIC_RELAXED);
  return XM_SUCCESS;
}

XM_S32 XM_MPI_ISP_SensorUnRegCallBack(SENSOR_ID SensorId) {
  if (SensorId != isp_sensor_id())
    return XM_FAILURE;
  memset(&sensor, 0, sizeof(sensor));
  __atomic_store_n(&sensor_id, (SENSOR_ID)-1, __ATOMIC_RELAXED);
  return XM_SUCCESS;
}

/* 0 disables the frame interrupt, 1 enables it, 2 restores what 0 found */
XM_S32 XM_MPI_ISP_SetVsyncIntEn(ISP_DEV IspDev, XM_U8 u8Mode) {
  (void)IspDev;
  switch (u8Mode) {
  case 0:
    vsync_saved = isp_read(REG_VSYNC_INT);
    isp_write(REG_VSYNC_INT, 0);
    return XM_SUCCESS;
  case 1:
    isp_write(REG_VSYNC_INT, 1);
    return XM_SUCCESS;
  case 2:
    isp_write(REG_VSYNC_INT, vsync_saved);
    return XM_SUCCESS;
  }
  return XM_FAILURE;
}

XM_S32 XM_MPI_ISP_SetPubAttr(ISP_DEV IspDev, const ISP_PUB_ATTR_S *pstPubAttr) {
  if (!pstPubAttr)
    return XM_FAILURE;
  XM_MPI_ISP_SetVsyncIntEn(IspDev, 0);
  /* the buffers are the library's */
  const XM_U32 phys = pub.u32PhyAddr, err = pub.u32ErrPhyAddr;
  const XM_U32 size = pub.u32Size, err_size = pub.u32ErrSize;
  const XM_U8 bufs = pub.u8BufFNum;
  pub = *pstPubAttr;
  pub.u32PhyAddr = phys, pub.u32ErrPhyAddr = err;
  pub.u32Size = size, pub.u32ErrSize = err_size;
  pub.u8BufFNum = bufs;
  XM_MPI_AE_SetFrameRate(IspDev, pub.u8FrameRate);
  XM_MPI_ISP_SetVsyncIntEn(IspDev, 2);
  return XM_SUCCESS;
}

XM_S32 XM_MPI_ISP_GetPubAttr(ISP_DEV IspDev, ISP_PUB_ATTR_S *pstPubAttr) {
  (void)IspDev;
  if (!pstPubAttr)
    return XM_FAILURE;
  *pstPubAttr = pub;
  return XM_SUCCESS;
}

/* The 3D noise reduction's MMZ block: its frame, motion and error planes
 * (xm530_planes()). The vendor's block is up to 1 KiB larger than the
 * three; a page more covers it. */
static XM_S32 nr3d_buffers(void) {
  struct xm530_planes p;
  xm530_planes(pub.stWndRect.u32Width, pub.stWndRect.u32Height, &p);
  const XM_U32 frame = p.frame;
  const XM_U32 err_w = p.err_w, err_h = p.err_h;
  const XM_U32 size = p.err_off + p.err + 4096;
  if (XM_MPI_SYS_MmzAlloc(&mmz_phys, &mmz_virt, "Isp", NULL, size) !=
          XM_SUCCESS ||
      !mmz_phys) {
    DBG("cannot allocate the 3D NR buffers (%u bytes)", size);
    return XM_FAILURE;
  }
  pub.u32PhyAddr = mmz_phys;
  pub.u32Size = frame;
  pub.u32ErrPhyAddr = mmz_phys + p.err_off;
  pub.u32ErrSize = err_h << 16 | err_w;
  pub.u8BufFNum = 1;
  return XM_SUCCESS;
}

/* The vendor library's start-up for any sensor (xm530_start.h): the
 * template, the geometry of the picture, the attributes from the sensor's
 * defaults, and what it has built in for some sensors. */
XM_S32 XM_MPI_ISP_Init(ISP_DEV IspDev) {
  if (!mem_init || init_done)
    return init_done ? XM_SUCCESS : XM_FAILURE;
  const XM_U32 w = pub.stWndRect.u32Width, h = pub.stWndRect.u32Height;
  /* the window registers take 12 bits, and the tone mapping grid not
   * every size */
  if (w > 4095 || h > 4095 || xm530_geometry(w, h, 0, geo)) {
    DBG("no start-up for %ux%u", w, h);
    return XM_FAILURE;
  }
  if (nr3d_buffers() != XM_SUCCESS)
    return XM_FAILURE;
  const SENSOR_ID id = isp_sensor_id();
  sns_start = NULL;
  for (const struct xm530_sensor *s = xm530_sensors; s->id; s++)
    if ((SENSOR_ID)s->id == id)
      sns_start = s;
  DBG("sensor %d, %ux%u%s", (int)id, w, h,
      sns_start ? "" : ", no built-in tuning");
  attr_start(sns_start);
  const XM_U32 mmz8 = (mmz_phys - 0x80000000) / 8;
  xm530_geometry(w, h, mmz8, geo);
  isp_write_table(xm530_init, mmz8, geo);
  if (sns_start)
    isp_write_table(sns_start->init, mmz8, geo);
  attr_gamma();
  if (isp_3a_init(IspDev, &pub) != XM_SUCCESS)
    return XM_FAILURE;
  isp_write_table(xm530_post, mmz8, geo);
  if (sns_start)
    isp_write_table(sns_start->post, mmz8, geo);
  init_done = 1;
  return XM_SUCCESS;
}

XM_S32 XM_MPI_ISP_Run(ISP_DEV IspDev) {
  (void)IspDev;
  if (!init_done || run_started)
    return XM_FAILURE;
  run_started = 1;
  isp_write_table(xm530_run, (mmz_phys - 0x80000000) / 8, geo);
  if (sns_start)
    isp_write_table(sns_start->run, (mmz_phys - 0x80000000) / 8, geo);
  if (isp_3a_start() != XM_SUCCESS)
    return XM_FAILURE;
  /* The tuning by gain, every 7 frames or so as the vendor's loop; Run never
   * returns. */
  for (;;) {
    usleep(280000);
    const XM_U32 iso = isp_3a_iso();
    if (!iso)
      continue;
    struct isp_tune_attrs a;
    attr_snapshot(&a);
    isp_tune(iso, &a);
  }
  return XM_SUCCESS;
}

XM_S32 XM_MPI_ISP_Exit(ISP_DEV IspDev) {
  (void)IspDev;
  return XM_SUCCESS;
}

/* Mirror and flip go to the sensor. */
static ISP_CHN_ATTR_S chn;

XM_S32 XM_MPI_ISP_SetChnAttr(ISP_CHN_ATTR_S *pstChnAttr) {
  if (!pstChnAttr)
    return XM_FAILURE;
  chn = *pstChnAttr;
  if (!chn.bMode && sensor.stSnsExp.pfn_cmos_set_mirror_flip)
    return sensor.stSnsExp.pfn_cmos_set_mirror_flip(chn.bMirror, chn.bFlip);
  return XM_SUCCESS;
}

XM_S32 XM_MPI_ISP_GetChnAttr(ISP_CHN_ATTR_S *pstChnAttr) {
  if (!pstChnAttr)
    return XM_FAILURE;
  *pstChnAttr = chn;
  return XM_SUCCESS;
}

XM_S32 XM_MPI_ISP_GetNrInfo(ISP_NR_INFO_S *pstNRInfo) {
  if (!pstNRInfo)
    return XM_FAILURE;
  memset(pstNRInfo, 0, sizeof(*pstNRInfo));
  return XM_SUCCESS;
}

/* The attributes, kept as set, and until then as the vendor library's Init
 * set them. */
struct attr_store {
  const char *name;
  void *p;
  size_t size;
};
static const struct attr_store *attr_stores(void);

static const struct attr_store *attr_find(const char *name) {
  for (const struct attr_store *s = attr_stores(); s->name; s++)
    if (!strcmp(s->name, name))
      return s;
  return NULL;
}

/* cq: const, where the SDK headers have it; apply: what takes effect at
 * once, under the lock, so the registers end as the stored attribute */
#define ATTR(name, type, cq, apply)                                            \
  static type attr_##name;                                                     \
  XM_S32 XM_MPI_ISP_Set##name(cq type *p) {                                    \
    if (!p)                                                                    \
      return XM_FAILURE;                                                       \
    pthread_mutex_lock(&attr_lock);                                            \
    attr_##name = *p;                                                          \
    apply;                                                                     \
    pthread_mutex_unlock(&attr_lock);                                          \
    return XM_SUCCESS;                                                         \
  }                                                                            \
  XM_S32 XM_MPI_ISP_Get##name(type *p) {                                       \
    if (!p)                                                                    \
      return XM_FAILURE;                                                       \
    pthread_mutex_lock(&attr_lock);                                            \
    *p = attr_##name;                                                          \
    pthread_mutex_unlock(&attr_lock);                                          \
    return XM_SUCCESS;                                                         \
  }
#define ATTR_DEV(name, type, cq)                                               \
  static type attr_##name;                                                     \
  XM_S32 XM_MPI_ISP_Set##name(ISP_DEV IspDev, cq type *p) {                    \
    (void)IspDev;                                                              \
    if (!p)                                                                    \
      return XM_FAILURE;                                                       \
    pthread_mutex_lock(&attr_lock);                                            \
    attr_##name = *p;                                                          \
    pthread_mutex_unlock(&attr_lock);                                          \
    return XM_SUCCESS;                                                         \
  }                                                                            \
  XM_S32 XM_MPI_ISP_Get##name(ISP_DEV IspDev, type *p) {                       \
    (void)IspDev;                                                              \
    if (!p)                                                                    \
      return XM_FAILURE;                                                       \
    pthread_mutex_lock(&attr_lock);                                            \
    *p = attr_##name;                                                          \
    pthread_mutex_unlock(&attr_lock);                                          \
    return XM_SUCCESS;                                                         \
  }

/* The gamma curve, node by node (10 bits); disabled, a straight line. */
static void gamma_write(const ISP_GAMMA_ATTR_S *g) {
  for (int i = 0; i < GAMMA_NODE_NUMBER; i++) {
    const XM_U32 line = i * 1024 / (GAMMA_NODE_NUMBER - 1);
    isp_write(REG_GAMMA + 4 * i,
              g->bEnable ? g->u16Table[i] : (line > 0x3ff ? 0x3ff : line));
  }
}

static void gamma_apply(const ISP_GAMMA_ATTR_S *g) {
  if (init_done)
    gamma_write(g);
}

/* No saturation is black and white at once: Cb and Cr gain 0. Otherwise
 * the tuning pass sets them from the saturation and the gain. */
static void csc_apply(const ISP_CSC_ATTR_S *c) {
  if (init_done && c->bEnable && !c->stManual.u8SatuVal) {
    isp_write(REG_CBCR, 0);
    isp_write(REG_CBCR + 4, 0);
  }
}

ATTR(GammaAttr, ISP_GAMMA_ATTR_S, const, gamma_apply(p))
ATTR(BlackLevelAttr, ISP_BLACKLVL_ATTR_S, const, )
ATTR(SaturationAttr, ISP_SATURATION_ATTR_S, const, )
ATTR(DyDefectPixelAttr, ISP_DYDPC_ATTR_S, , )
ATTR(SharpenAttr, ISP_SHARPEN_ATTR_S, const, )
ATTR(SharpenV2Attr, ISP_SHARPENV2_ATTR_S, const, )
ATTR(NRAttr, ISP_2DNR_ATTR_S, const, )
ATTR(3DNrAttr, ISP_3DNR_ATTR_S, const, )
ATTR(3DNrV2Attr, ISP_3DNRV2_ATTR_S, const, )
ATTR(CSCAttr, ISP_CSC_ATTR_S, const, csc_apply(p))
ATTR(AntiFalseColorAttr, ISP_ANTI_FALSECOLOR_S, const,
     if (init_done) isp_antifalsecolor(p))
ATTR(ChromaAttr, ISP_CHROMA_ATTR_S, , if (init_done) isp_chroma(p))
ATTR_DEV(DRCAttr, ISP_DRC_ATTR_S, const)
ATTR_DEV(DeFogAttr, ISP_DEFOG_ATTR_S, const)
ATTR_DEV(DCIAttr, ISP_DCI_ATTR_S, )

#define STORE(name, type) {#name, &attr_##name##Attr, sizeof(type)}
static const struct attr_store *attr_stores(void) {
  static const struct attr_store stores[] = {
      STORE(Gamma, ISP_GAMMA_ATTR_S),
      STORE(BlackLevel, ISP_BLACKLVL_ATTR_S),
      STORE(Saturation, ISP_SATURATION_ATTR_S),
      STORE(DyDefectPixel, ISP_DYDPC_ATTR_S),
      STORE(Sharpen, ISP_SHARPEN_ATTR_S),
      STORE(SharpenV2, ISP_SHARPENV2_ATTR_S),
      STORE(NR, ISP_2DNR_ATTR_S),
      STORE(3DNr, ISP_3DNR_ATTR_S),
      STORE(3DNrV2, ISP_3DNRV2_ATTR_S),
      STORE(CSC, ISP_CSC_ATTR_S),
      STORE(AntiFalseColor, ISP_ANTI_FALSECOLOR_S),
      STORE(Chroma, ISP_CHROMA_ATTR_S),
      STORE(DRC, ISP_DRC_ATTR_S),
      STORE(DeFog, ISP_DEFOG_ATTR_S),
      STORE(DCI, ISP_DCI_ATTR_S),
      {NULL, NULL, 0},
  };
  return stores;
}

/* Where the vendor library takes these attributes from the sensor's
 * defaults: its AGC table and its gamma (tools/xm-isp-trace/start530.py,
 * MAPPED, checks this on every trace) */
static void sensor_defaults(void) {
  ISP_CMOS_DEFAULT_S d;
  if (!sensor.stSnsExp.pfn_cmos_get_isp_default)
    return;
  memset(&d, 0, sizeof(d));
  if (sensor.stSnsExp.pfn_cmos_get_isp_default(&d))
    return;
  if (d.stGamma.bValid)
    memcpy(attr_GammaAttr.u16Table, d.stGamma.au16Gamma,
           sizeof(attr_GammaAttr.u16Table));
  if (!d.stAgcTbl.bValid)
    return;
  const ISP_CMOS_AGC_TABLE_S *t = &d.stAgcTbl;
  memcpy(attr_BlackLevelAttr.stAuto.au16Blc, t->au16Blc, sizeof(t->au16Blc));
  memcpy(attr_SaturationAttr.stAuto.au8Sat, t->au8Saturation,
         sizeof(t->au8Saturation));
  memcpy(attr_SharpenAttr.stAuto.au8SharpenD, t->au8SharpenAltD,
         sizeof(t->au8SharpenAltD));
  memcpy(attr_SharpenAttr.stAuto.au8SharpenUd, t->au8SharpenAltUd,
         sizeof(t->au8SharpenAltUd));
  memcpy(attr_SharpenAttr.stAuto.au8SharpenKd, t->au8SharpenKd,
         sizeof(t->au8SharpenKd));
  memcpy(attr_NRAttr.stAuto.au8Thresh, t->au8SnrThresh,
         sizeof(t->au8SnrThresh));
  memcpy(attr_3DNrAttr.stAuto.au8TfStrength, t->au83DTfStrength,
         sizeof(t->au83DTfStrength));
  memcpy(attr_3DNrAttr.stAuto.au8SfStrength, t->au83DSfStrength,
         sizeof(t->au83DSfStrength));
  memcpy(attr_DyDefectPixelAttr.stAuto.au8Sth, t->au8DyDpc,
         sizeof(t->au8DyDpc));
  for (int i = 0; i < ISP_AUTO_STENGTH_NUM; i++)
    attr_CSCAttr.stAuto.au8ContrVal[i] = (XM_U8)t->au16Ytrangamma[i];
}

/* the attributes Init starts with: the template's, the sensor's defaults,
 * and what the vendor library has built in for the sensor */
static void attr_start(const struct xm530_sensor *s) {
  pthread_mutex_lock(&attr_lock);
  for (const struct xm530_attr *a = xm530_attr; a->name; a++) {
    const struct attr_store *st = attr_find(a->name);
    if (st)
      memcpy(st->p, a->w, st->size < sizeof(a->w) ? st->size : sizeof(a->w));
  }
  sensor_defaults();
  for (const struct xm530_patch *p = s ? s->attr : NULL; p && p->name; p++) {
    const struct attr_store *st = attr_find(p->name);
    if (st && p->len <= sizeof(p->data) && p->offset + p->len <= st->size)
      memcpy((XM_U8 *)st->p + p->offset, p->data, p->len);
  }
  pthread_mutex_unlock(&attr_lock);
}

/* the gamma Init leaves: the sensor's */
static void attr_gamma(void) {
  pthread_mutex_lock(&attr_lock);
  gamma_write(&attr_GammaAttr);
  pthread_mutex_unlock(&attr_lock);
}

static void attr_snapshot(struct isp_tune_attrs *a) {
  pthread_mutex_lock(&attr_lock);
  a->nr = attr_NRAttr;
  a->sharpen = attr_SharpenAttr;
  a->sharpen2 = attr_SharpenV2Attr;
  a->black = attr_BlackLevelAttr;
  a->saturation = attr_SaturationAttr;
  a->csc = attr_CSCAttr;
  a->dpc = attr_DyDefectPixelAttr;
  a->defog = attr_DeFogAttr;
  pthread_mutex_unlock(&attr_lock);
}
