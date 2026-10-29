/* SPDX-License-Identifier: MIT */
/* What libxmisp's sensor drivers may use of the SoC side. */
#ifndef XMISP_PRIV_H
#define XMISP_PRIV_H

#include <stdint.h>

struct xmisp;

/* The script's sensor, 8-bit registers and data. 0 or -1 (errno). */
int xmisp_sensor_write(struct xmisp *isp, uint32_t reg, uint32_t val);
int xmisp_sensor_read(struct xmisp *isp, uint32_t reg, uint32_t *val);

#endif
