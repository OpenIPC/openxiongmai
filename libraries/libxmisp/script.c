// SPDX-License-Identifier: MIT
/* The SoC's start-up with a sensor's parts and values: see script.h. */
#include <stddef.h>

#include "script.h"

static int is_isp_write(const struct xmisp_op *op) {
  return op->type == XMISP_ISP_W || op->type == XMISP_ISP_W_BUF;
}

/* The script's value for an ISP write, if it is the register's last write
 * in the template. */
static uint32_t isp_val(const struct xmisp_script *sc,
                        const struct xmisp_op *op) {
  for (const struct xmisp_op *later = op + 1; later->type != XMISP_END; later++)
    if (is_isp_write(later) && later->a == op->a)
      return op->b;
  for (const struct xmisp_isp_val *v = sc->isp; v && v->addr; v++)
    if (v->addr == op->a)
      return v->val;
  return op->b;
}

unsigned xmisp_expand(const struct xmisp_op *soc,
                      const struct xmisp_script *script, struct xmisp_op *out,
                      unsigned max) {
  unsigned n = 0;
  for (const struct xmisp_op *op = soc;; op++) {
    if (op->type == XMISP_SENSOR) {
      const struct xmisp_op *p =
          op->a < XMISP_PARTS ? script->part[op->a] : NULL;
      for (; p && p->type != XMISP_END; p++, n++)
        if (n < max)
          out[n] = *p;
      continue;
    }
    if (n < max) {
      out[n] = *op;
      if (is_isp_write(op))
        out[n].b = isp_val(script, op);
      else if (op->type == XMISP_ISP_BUF && script->isp_buf)
        out[n].a = script->isp_buf;
    }
    n++;
    if (op->type == XMISP_END)
      return n;
  }
}
