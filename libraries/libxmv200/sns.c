// SPDX-License-Identifier: MIT
/* The sensor drivers (sns.h) by name, and their modes. */

#include <string.h>
#include <strings.h>

#include "sns.h"

/* one line a driver (sns_<sensor>.c) */
extern const struct xmv200_sns_drv xmv200_sns_sc2336;
extern const struct xmv200_sns_drv xmv200_sns_sc2336p;
extern const struct xmv200_sns_drv xmv200_sns_sc2337p;
extern const struct xmv200_sns_drv xmv200_sns_sc1336;
extern const struct xmv200_sns_drv xmv200_sns_sc1345;
extern const struct xmv200_sns_drv xmv200_sns_sc1346;
extern const struct xmv200_sns_drv xmv200_sns_sc1a4t;
extern const struct xmv200_sns_drv xmv200_sns_sc035gs;
extern const struct xmv200_sns_drv xmv200_sns_sc200ai;
extern const struct xmv200_sns_drv xmv200_sns_sc231ai;
extern const struct xmv200_sns_drv xmv200_sns_sc2331;
extern const struct xmv200_sns_drv xmv200_sns_sc2335;
extern const struct xmv200_sns_drv xmv200_sns_sc202cs;
extern const struct xmv200_sns_drv xmv200_sns_sc3235;
extern const struct xmv200_sns_drv xmv200_sns_sc3335;
extern const struct xmv200_sns_drv xmv200_sns_sc3338;
extern const struct xmv200_sns_drv xmv200_sns_sc401ai;
extern const struct xmv200_sns_drv xmv200_sns_sc4210;
extern const struct xmv200_sns_drv xmv200_sns_sc430ai;
extern const struct xmv200_sns_drv xmv200_sns_sc4336;
extern const struct xmv200_sns_drv xmv200_sns_sc500ai;
extern const struct xmv200_sns_drv xmv200_sns_sc5239ai;
extern const struct xmv200_sns_drv xmv200_sns_sc831hai;
extern const struct xmv200_sns_drv xmv200_sns_f37p;
extern const struct xmv200_sns_drv xmv200_sns_q03;
extern const struct xmv200_sns_drv xmv200_sns_q03p;
extern const struct xmv200_sns_drv xmv200_sns_h63;
extern const struct xmv200_sns_drv xmv200_sns_h63p;
extern const struct xmv200_sns_drv xmv200_sns_sc223a;
extern const struct xmv200_sns_drv xmv200_sns_sc223ap;
extern const struct xmv200_sns_drv xmv200_sns_sc2210;
extern const struct xmv200_sns_drv xmv200_sns_sc307c;
extern const struct xmv200_sns_drv xmv200_sns_sc307e;
extern const struct xmv200_sns_drv xmv200_sns_gc3003;
extern const struct xmv200_sns_drv xmv200_sns_imx307;
extern const struct xmv200_sns_drv xmv200_sns_imx335;
extern const struct xmv200_sns_drv xmv200_sns_mis2009;
extern const struct xmv200_sns_drv xmv200_sns_mis4001;
extern const struct xmv200_sns_drv xmv200_sns_mis40c1;
extern const struct xmv200_sns_drv xmv200_sns_mis5001;

extern const struct xmv200_sns_drv xmv200_sns_sc1346_xm533;
extern const struct xmv200_sns_drv xmv200_sns_sc1a4t_xm533;
extern const struct xmv200_sns_drv xmv200_sns_sc223a_xm533;
extern const struct xmv200_sns_drv xmv200_sns_sc231ai_xm533;
extern const struct xmv200_sns_drv xmv200_sns_sc2331_xm533;
extern const struct xmv200_sns_drv xmv200_sns_sc2336_xm533;
extern const struct xmv200_sns_drv xmv200_sns_sc3338_xm533;
extern const struct xmv200_sns_drv xmv200_sns_sc401ai_xm533;
extern const struct xmv200_sns_drv xmv200_sns_sc4336_xm533;
extern const struct xmv200_sns_drv xmv200_sns_sc500ai_xm533;

static const struct xmv200_sns_drv *const drivers[] = {
    &xmv200_sns_sc2336,
    &xmv200_sns_sc2336p,
    &xmv200_sns_sc2337p,
    &xmv200_sns_sc1336,
    &xmv200_sns_sc1345,
    &xmv200_sns_sc1346,
    &xmv200_sns_sc1a4t,
    &xmv200_sns_sc035gs,
    &xmv200_sns_sc200ai,
    &xmv200_sns_sc231ai,
    &xmv200_sns_sc2331,
    &xmv200_sns_sc2335,
    &xmv200_sns_sc202cs,
    &xmv200_sns_sc3235,
    &xmv200_sns_sc3335,
    &xmv200_sns_sc3338,
    &xmv200_sns_sc401ai,
    &xmv200_sns_sc4210,
    &xmv200_sns_sc430ai,
    &xmv200_sns_sc4336,
    &xmv200_sns_sc500ai,
    &xmv200_sns_sc5239ai,
    &xmv200_sns_sc831hai,
    &xmv200_sns_f37p,
    &xmv200_sns_q03,
    &xmv200_sns_q03p,
    &xmv200_sns_h63,
    &xmv200_sns_h63p,
    &xmv200_sns_sc223a,
    &xmv200_sns_sc223ap,
    &xmv200_sns_sc2210,
    &xmv200_sns_sc307c,
    &xmv200_sns_sc307e,
    &xmv200_sns_gc3003,
    &xmv200_sns_imx307,
    &xmv200_sns_imx335,
    &xmv200_sns_mis2009,
    &xmv200_sns_mis4001,
    &xmv200_sns_mis40c1,
    &xmv200_sns_mis5001,
    &xmv200_sns_sc1346_xm533,
    &xmv200_sns_sc1a4t_xm533,
    &xmv200_sns_sc223a_xm533,
    &xmv200_sns_sc231ai_xm533,
    &xmv200_sns_sc2331_xm533,
    &xmv200_sns_sc2336_xm533,
    &xmv200_sns_sc3338_xm533,
    &xmv200_sns_sc401ai_xm533,
    &xmv200_sns_sc4336_xm533,
    &xmv200_sns_sc500ai_xm533,
};

static const char *family_of(const struct xmv200_sns_drv *drv) {
  return drv->family ? drv->family : "xm530v200";
}

const struct xmv200_sns_drv *xmv200_sns_driver(const char *name,
                                               const char *family) {
  for (unsigned i = 0; name && i < sizeof(drivers) / sizeof(drivers[0]); i++)
    if (!strcasecmp(drivers[i]->name, name) &&
        !strcmp(family_of(drivers[i]), family))
      return drivers[i];
  return NULL;
}

static int same_regs(const struct xmv200_sns_mode *m,
                     const struct xmv200_sns_run *run) {
  if (m->n != run->n)
    return 0;
  for (int i = 0; i < m->n; i++)
    if (m->regs[i].reg != run->regs[i].reg ||
        m->regs[i].val != run->regs[i].val)
      return 0;
  return 1;
}

const struct xmv200_sns_mode *
xmv200_sns_mode_for(const struct xmv200_sns_drv *drv,
                    const struct xmv200_sns_attr *a,
                    const struct xmv200_sns_tuning *t) {
  if (!drv)
    return NULL;
  if (a) {
    for (int i = 0; i < drv->n_modes; i++) {
      const struct xmv200_sns_attr *m = &drv->modes[i].attr;

      if (m->dvp == a->dvp && m->p720 == a->p720 &&
          (a->dvp || m->one_lane == a->one_lane))
        return &drv->modes[i];
    }
    return NULL;
  }
  /* the mode the tuning was captured in */
  for (int i = 0; t && t->n_runs && i < drv->n_modes; i++)
    if (same_regs(&drv->modes[i], &t->runs[0]))
      return &drv->modes[i];
  return NULL;
}

const struct xmv200_sns_mode *
xmv200_sns_find(const char *name, const struct xmv200_sns_tuning *t,
                const struct xmv200_sns_drv **drv) {
  const struct xmv200_sns_mode *m;

  /* a driver is its chip family's libsns's, for that family's tunings */
  *drv = NULL;
  if (!t)
    return NULL;
  *drv = xmv200_sns_driver(name, t->chip);
  if ((m = xmv200_sns_mode_for(*drv, NULL, t)))
    return m;
  /* the driver the vendor ran: its libsns can take one part for another
   * (the X6C-WEQ's reads SC2337P and runs SC2336P's driver) */
  for (unsigned i = 0; i < sizeof(drivers) / sizeof(drivers[0]); i++)
    if (!strcmp(family_of(drivers[i]), t->chip) &&
        (m = xmv200_sns_mode_for(drivers[i], NULL, t))) {
      *drv = drivers[i];
      return m;
    }
  *drv = NULL;
  return NULL;
}

void xmv200_sns_init_step(const struct xmv200_sns_drv *drv,
                          const struct xmv200_sns_dev *d,
                          const struct xmv200_sns_ctx *c) {
  drv->init(d, c->mode, &c->attr);
}

void xmv200_sns_fps_step(const struct xmv200_sns_drv *drv,
                         const struct xmv200_sns_dev *d,
                         const struct xmv200_sns_ctx *c) {
  drv->ae_default(&c->attr, c->ae);
  drv->set_fps(d, &c->attr, c->attr.fps, c->ae);
}

void xmv200_sns_ae_step(const struct xmv200_sns_drv *drv,
                        const struct xmv200_sns_dev *d,
                        const struct xmv200_sns_ctx *c) {
  uint32_t idx, reg, real;

  drv->set_exposure(d, c->exp_lines);
  drv->again_calc(0x20, &idx, &real);
  drv->dgain_calc(0x20, &reg, &real);
  drv->set_gain(d, idx, reg);
}

void xmv200_sns_run(const struct xmv200_sns_drv *drv,
                    const struct xmv200_sns_dev *d, int n,
                    const struct xmv200_sns_ctx *c) {
  if (drv->run) {
    drv->run(d, n, c);
    return;
  }
  switch (n) {
  case 0:
    xmv200_sns_init_step(drv, d, c);
    break;
  case 1:
    xmv200_sns_fps_step(drv, d, c);
    break;
  case 2:
    xmv200_sns_ae_step(drv, d, c);
    break;
  }
}

static struct {
  uint16_t reg;
  uint8_t val;
} rec[512];
static int n_rec;

static void record(uint32_t reg, uint32_t val) {
  if (n_rec < (int)(sizeof(rec) / sizeof(rec[0]))) {
    rec[n_rec].reg = reg;
    rec[n_rec].val = val;
  }
  n_rec++;
}

uint32_t xmv200_sns_capture_fps(const struct xmv200_sns_drv *drv,
                                const struct xmv200_sns_mode *m,
                                const struct xmv200_sns_tuning *t,
                                const struct xmv200_sns_dev *d,
                                uint32_t fallback) {
  static const uint32_t rates[] = {12, 25, 30, 15, 20, 10, 24};
  uint32_t found = fallback;

  if (!drv || !m || t->n_runs < 2)
    return fallback;
  for (unsigned i = 0; i < sizeof(rates) / sizeof(rates[0]); i++) {
    struct xmv200_sns_ae ae = {0};
    struct xmv200_sns_ctx c = {m, m->attr, t->exp_lines, &ae};
    const struct xmv200_sns_run *r = &t->runs[1];
    int same;

    c.attr.fps = rates[i];
    xmv200_i2c_divert = record;
    xmv200_sns_run(drv, d, 0, &c);
    n_rec = 0;
    xmv200_sns_run(drv, d, 1, &c);
    xmv200_i2c_divert = NULL;
    same = n_rec == r->n;
    for (int k = 0; same && k < n_rec; k++)
      same = rec[k].reg == r->regs[k].reg && rec[k].val == r->regs[k].val;
    if (same) {
      found = rates[i];
      break;
    }
  }
  return found;
}
