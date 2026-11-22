// SPDX-License-Identifier: MIT
/*
 * check [FAMILY]: each tuning's sensor runs, done by its sensor driver
 * with the I2C recorded, against the runs the tuning recorded from the
 * vendor's ISP. A read answers what was last written there, else 0, as
 * qemu-bench's sensors do. Exit 1 if a driver's writes differ.
 */
#include <stdio.h>
#include <string.h>

#include "sns.h"

static struct {
  uint16_t reg;
  uint8_t val;
} wr[8192];
static int n_wr;

void (*xmv200_i2c_divert)(uint32_t reg, uint32_t val);

/* the sensor's registers, as written */
static uint8_t regs[0x10000];

int xmv200_i2c_write(int bus, uint8_t dev, uint32_t reg, int reg_bytes,
                     uint32_t val, int val_bytes) {
  regs[reg & 0xffff] = val;
  if (xmv200_i2c_divert) {
    xmv200_i2c_divert(reg, val);
    return 0;
  }
  if (n_wr < (int)(sizeof(wr) / sizeof(wr[0]))) {
    wr[n_wr].reg = reg;
    wr[n_wr].val = val;
  }
  n_wr++;
  return 0;
}

int xmv200_i2c_read(int bus, uint8_t dev, uint32_t reg, int reg_bytes,
                    uint32_t *val, int val_bytes) {
  *val = regs[reg & 0xffff];
  return 0;
}

int xmv200_reg_write(uint32_t addr, uint32_t val) { return 0; }
int xmv200_reg_read(uint32_t addr, uint32_t *val) {
  *val = 0;
  return 0;
}

static int same(const struct xmv200_sns_run *r, int verbose) {
  int ok = n_wr == r->n;

  for (int i = 0; ok && i < n_wr; i++)
    ok = wr[i].reg == r->regs[i].reg && wr[i].val == r->regs[i].val;
  if (!ok && verbose) {
    int n = n_wr > r->n ? n_wr : r->n, shown = 0;

    for (int i = 0; i < n && shown < 16; i++) {
      int a = i < n_wr, b = i < r->n;

      if (a && b && wr[i].reg == r->regs[i].reg && wr[i].val == r->regs[i].val)
        continue;
      printf("      #%d driver %04x=%02x recorded %04x=%02x\n", i,
             a ? wr[i].reg : 0, a ? wr[i].val : 0, b ? r->regs[i].reg : 0,
             b ? r->regs[i].val : 0);
      shown++;
    }
  }
  return ok;
}

int main(int argc, char **argv) {
  const char *family = argc > 1 ? argv[1] : NULL;
  int bad = 0;

  for (int i = 0; i < xmv200_n_sensors; i++) {
    const struct xmv200_sns *s = &xmv200_sensors[i];
    const struct xmv200_sns_tuning *t = s->tuning;
    const struct xmv200_sns_drv *drv;
    const struct xmv200_sns_mode *m;
    struct xmv200_sns_ae ae;
    struct xmv200_sns_ctx c;
    struct xmv200_sns_dev d = {0, 0x30, 0};
    int want, ok = 1;

    if (family && strcmp(t->chip, family))
      continue;
    memset(regs, 0, sizeof(regs));
    m = xmv200_sns_find(s->name, t, &drv);
    printf("%-9s %-9s ", t->chip, s->name);
    if (!m) {
      printf("no driver mode\n");
      continue;
    }
    want = xmv200_sns_n_runs(drv, m);
    printf("%s, %s: ", drv->name, m->name);
    if (t->n_runs != want) {
      printf("%d runs, the driver makes %d\n", t->n_runs, want);
      bad = 1;
      continue;
    }
    /* the frame rate the capture's board ran the sensor at, as the
     * pipeline finds it (and with the driver's state after that) */
    uint32_t fps = xmv200_sns_capture_fps(drv, m, t, &d, 12);

    memset(&ae, 0, sizeof(ae));
    c = (struct xmv200_sns_ctx){m, m->attr, t->exp_lines, &ae};
    c.attr.fps = fps;
    for (int n = 0; n < t->n_runs; n++) {
      n_wr = 0;
      xmv200_sns_run(drv, &d, n, &c);
      if (!same(&t->runs[n], 0)) {
        if (ok)
          printf("\n");
        printf("    run %d at %u fps: %d writes, %d recorded\n", n, fps, n_wr,
               t->runs[n].n);
        same(&t->runs[n], 1);
        ok = 0;
      }
    }
    if (ok)
      printf("%u fps: ", fps);
    printf(ok ? "same\n" : "");
    bad |= !ok;
  }
  return bad;
}
