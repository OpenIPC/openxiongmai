// SPDX-License-Identifier: MIT
/*
 * Host test of the sensor scripts: the XM510 template with each script's
 * parts and ISP values. The JX-H62's must be the start-up its trace gave as
 * one list; every script must probe a chip ID on a bus the interpreter
 * drives, and change only ISP registers the template writes. Run: make -C
 * libraries/libxmisp test
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../script.h"
#include "../xmisp.h"
#include "../xmisp_priv.h"

/* The sensor drivers' bus, which the scripts only point at. */
int xmisp_sensor_write(struct xmisp *isp, uint32_t reg, uint32_t val) {
  return -1;
}
int xmisp_sensor_read(struct xmisp *isp, uint32_t reg, uint32_t *val) {
  return -1;
}

extern const struct xmisp_op h62_720p_flat[];

static int failures;

static void check(int ok, const char *script, const char *what) {
  if (!ok) {
    printf("FAIL: %s: %s\n", script, what);
    failures++;
  }
}

static unsigned count(const struct xmisp_op *op, enum xmisp_op_type type) {
  unsigned n = 0;
  for (; op->type != XMISP_END; op++)
    n += op->type == type;
  return n;
}

static unsigned isp_count(const struct xmisp_script *sc) {
  unsigned n = 0;
  for (const struct xmisp_isp_val *v = sc->isp; v && v->addr; v++)
    n++;
  return n;
}

static int template_writes(uint32_t addr) {
  for (const struct xmisp_op *op = xmisp_xm510_start; op->type != XMISP_END;
       op++)
    if ((op->type == XMISP_ISP_W || op->type == XMISP_ISP_W_BUF) &&
        op->a == addr)
      return 1;
  return 0;
}

int main(void) {
  check(count(xmisp_xm510_start, XMISP_SENSOR) == XMISP_PARTS, "template",
        "one slot per part");
  for (unsigned i = 0; i < XMISP_PARTS; i++) {
    unsigned seen = 0;
    for (const struct xmisp_op *op = xmisp_xm510_start; op->type != XMISP_END;
         op++)
      seen += op->type == XMISP_SENSOR && op->a == i;
    check(seen == 1, "template", "a part's slot missing or repeated");
  }

  for (const struct xmisp_script *const *sp = xmisp_scripts; *sp; sp++) {
    const struct xmisp_script *sc = *sp;
    const unsigned n = xmisp_expand(xmisp_xm510_start, sc, NULL, 0);
    struct xmisp_op *start = malloc(n * sizeof(*start));
    check(start && xmisp_expand(xmisp_xm510_start, sc, start, n) == n &&
              start[n - 1].type == XMISP_END,
          sc->name, "expands");
    check(count(start, XMISP_SENSOR) == 0, sc->name, "a slot left over");
    check(sc->part[XMISP_PART_PROBE] &&
              count(sc->part[XMISP_PART_PROBE], XMISP_I2C_EXPECT) > 0,
          sc->name, "no chip ID in the probe");
    check(sc->part[XMISP_PART_INIT] &&
              count(sc->part[XMISP_PART_INIT], XMISP_I2C_W) > 0,
          sc->name, "no sensor init");
    check(sc->part[XMISP_PART_VI] &&
              count(sc->part[XMISP_PART_VI], XMISP_VI_IOW) > 0,
          sc->name, "no VI setup");
    check(sc->i2c_dev && sc->i2c_dev < 0x80, sc->name, "7-bit I2C address");
    check((sc->reg_bytes == 1 || sc->reg_bytes == 2) &&
              (sc->data_bytes == 1 || sc->data_bytes == 2),
          sc->name, "register and data widths of 1 or 2 bytes");
    for (const struct xmisp_isp_val *v = sc->isp; v && v->addr; v++)
      check(template_writes(v->addr), sc->name,
            "an ISP value for a register the template doesn't write");
    printf("%-24s %4u steps, %3u sensor writes, ISP values %u\n", sc->name, n,
           count(sc->part[XMISP_PART_INIT], XMISP_I2C_W) +
               count(sc->part[XMISP_PART_POST], XMISP_I2C_W),
           isp_count(sc));

    if (sc == &xmisp_h62_720p) {
      unsigned m = 0;
      while (h62_720p_flat[m].type != XMISP_END)
        m++;
      check(n == m + 1 && !memcmp(start, h62_720p_flat, n * sizeof(*start)),
            sc->name, "not the start-up its trace gave as one list");
    }
    free(start);
  }
  printf(failures ? "%d failed\n" : "ok\n", failures);
  return failures != 0;
}
