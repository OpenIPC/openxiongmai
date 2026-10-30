/* SPDX-License-Identifier: MIT */
/*
 * Sensor drivers: what the vendor's libsns (built into its isp daemon)
 * does to a sensor over I2C, per sensor, as functions libxmv200 calls. A
 * tuning (sns_tuning.h) still holds the ISP's side of a mode; where a
 * sensor has a driver, its init, frame rate, exposure, gain and flip come
 * from the driver instead of the tuning's recorded runs.
 *
 * Units are the vendor's: exposure in lines; analog gain as an index into
 * the sensor's coarse steps (1x, 2x, 4x, ...), from again_calc(); the
 * sensor's fine digital gain in 1/32 (0x20 = 1x), from dgain_calc().
 */
#ifndef XMV200_SNS_H
#define XMV200_SNS_H

#include <stdint.h>

#include "sns_tuning.h"

/* a sensor on a channel; the vendor's functions differ for channel 1 */
struct xmv200_sns_dev {
  int bus;      /* /dev/i2cN */
  uint8_t addr; /* 7-bit */
  uint32_t chn;
};

/* what the vendor's cmos_set_sns_attr gives a sensor: how it is wired and
 * the mode it runs */
struct xmv200_sns_attr {
  int dvp;      /* parallel, not MIPI */
  int one_lane; /* MIPI on one lane */
  int p720;     /* 1280x720, where the sensor has it */
  uint32_t fps;
};

struct xmv200_sns_mode {
  const char *name;
  struct xmv200_sns_attr attr; /* what selects it (fps unused) */
  const struct xmv200_sns_reg *regs;
  int n;
};

/* the AE's limits in a mode, as the vendor's cmos_get_ae_default and
 * frame rate functions give them */
struct xmv200_sns_ae {
  uint32_t full_lines; /* the frame length (VTS) */
  uint32_t max_lines;  /* the longest exposure */
  uint32_t again_min, again_max; /* 1/32: 0x20 = 1x */
  uint32_t dgain_min, dgain_max;
};

/* what a tuning's sensor runs take: the mode, and the AE's start */
struct xmv200_sns_ctx {
  const struct xmv200_sns_mode *mode;
  struct xmv200_sns_attr attr; /* with the frame rate */
  uint32_t exp_lines;          /* the exposure the tuning starts from */
  struct xmv200_sns_ae *ae;    /* what the frame rate sets */
};

struct xmv200_sns_drv {
  const char *name;
  const struct xmv200_sns_mode *modes;
  int n_modes;
  /* the mode's registers and what the vendor writes after them */
  void (*init)(const struct xmv200_sns_dev *d, const struct xmv200_sns_mode *m,
               const struct xmv200_sns_attr *a);
  void (*ae_default)(const struct xmv200_sns_attr *a, struct xmv200_sns_ae *ae);
  /* line length and frame length for fps; updates ae */
  void (*set_fps)(const struct xmv200_sns_dev *d,
                  const struct xmv200_sns_attr *a, uint32_t fps,
                  struct xmv200_sns_ae *ae);
  /* frame length alone (slow shutter) */
  void (*set_vts)(const struct xmv200_sns_dev *d, uint32_t vts,
                  struct xmv200_sns_ae *ae);
  void (*set_exposure)(const struct xmv200_sns_dev *d, uint32_t lines);
  /* gain in 1/32 to what the sensor takes: the analog step's index and its
   * gain, the digital gain's register value and its gain */
  void (*again_calc)(uint32_t gain, uint32_t *idx, uint32_t *real);
  void (*dgain_calc)(uint32_t gain, uint32_t *reg, uint32_t *real);
  void (*set_gain)(const struct xmv200_sns_dev *d, uint32_t again_idx,
                   uint32_t dgain_reg);
  void (*set_flip)(const struct xmv200_sns_dev *d, int mirror, int flip);
  /* the tuning's sensor runs: how many the vendor's libsns makes, and
   * what each is (NULL: init, frame rate, the AE's start) */
  int n_runs; /* 0: 3 */
  void (*run)(const struct xmv200_sns_dev *d, int n,
              const struct xmv200_sns_ctx *c);
};

/* the driver for a sensor the registry names ("SC2336"), or NULL */
const struct xmv200_sns_drv *xmv200_sns_driver(const char *name);
/* the driver's mode for attr, or the one whose registers a tuning's first
 * run holds; NULL if none */
const struct xmv200_sns_mode *
xmv200_sns_mode_for(const struct xmv200_sns_drv *drv,
                    const struct xmv200_sns_attr *a,
                    const struct xmv200_sns_tuning *t);

/* the driver and mode a tuning was captured with: name's, or the driver
 * with a mode of the same registers; NULL if there is none */
const struct xmv200_sns_mode *
xmv200_sns_find(const char *name, const struct xmv200_sns_tuning *t,
                const struct xmv200_sns_drv **drv);

/* the tuning's sensor run n, by the driver; the default: run 0 the mode's
 * init, run 1 the frame rate, run 2 the exposure the tuning starts from and
 * 1x gain */
void xmv200_sns_run(const struct xmv200_sns_drv *drv,
                    const struct xmv200_sns_dev *d, int n,
                    const struct xmv200_sns_ctx *c);
/* the steps of the default */
void xmv200_sns_init_step(const struct xmv200_sns_drv *drv,
                          const struct xmv200_sns_dev *d,
                          const struct xmv200_sns_ctx *c);
void xmv200_sns_fps_step(const struct xmv200_sns_drv *drv,
                         const struct xmv200_sns_dev *d,
                         const struct xmv200_sns_ctx *c);
void xmv200_sns_ae_step(const struct xmv200_sns_drv *drv,
                        const struct xmv200_sns_dev *d,
                        const struct xmv200_sns_ctx *c);

extern const struct xmv200_sns_drv xmv200_sns_sc2336, xmv200_sns_sc2336p,
    xmv200_sns_sc2337p;

#endif
