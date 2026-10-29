/* SPDX-License-Identifier: MIT */
/*
 * libxmisp: an open ISP for Xiongmai SoCs, written from traces of the vendor
 * daemon, the register map and the hardware's behaviour.
 *
 * XM510 only so far: xmisp_start() brings up VI, the sensor and the ISP
 * from a sensor script and applies its fixed 3A state; xmisp_frame() runs
 * auto-exposure each frame (M2). White balance stays fixed.
 */
#ifndef XMISP_H
#define XMISP_H

#include "script.h"

struct xmisp;

/* Opens the devices: /dev/isp, /dev/xm_i2c, /dev/vi, /dev/mmz, /dev/rgn and
 * /dev/mem. NULL with errno set on failure. */
struct xmisp *xmisp_open(void);

/* Runs the first script whose sensor answers with its chip ID, then that
 * script's fixed 3A state. 0, or -1 with xmisp_error() saying why. */
int xmisp_start(struct xmisp *isp, const struct xmisp_script *const *scripts);

/* The script that started, or NULL. */
const struct xmisp_script *xmisp_script(const struct xmisp *isp);

/* /dev/isp: with O_ASYNC (fcntl) it sends SIGIO at each frame start. */
int xmisp_fd(const struct xmisp *isp);

/* Once a frame, at its start (SIGIO): reads the AE statistics and moves the
 * exposure. 1 if it changed, 0 if not (or AE is off), -1 if the statistics
 * or the sensor failed, with xmisp_error() saying which. */
int xmisp_frame(struct xmisp *isp);

/* 1 if AE runs: the script's sensor has a driver. */
int xmisp_ae_enabled(const struct xmisp *isp);

/* The last frame's mean luma (8-bit, linear) and the exposure set. */
void xmisp_exposure(const struct xmisp *isp, float *mean, uint32_t *lines,
                    float *gain);

/* What the vendor daemon does on its way out: 3D noise reduction off, so its
 * DMA stops before the next start resets VI. */
void xmisp_stop(struct xmisp *isp);

void xmisp_close(struct xmisp *isp);

const char *xmisp_error(const struct xmisp *isp);

/* The scripts built in, NULL-terminated. */
extern const struct xmisp_script *const xmisp_scripts[];

#endif
