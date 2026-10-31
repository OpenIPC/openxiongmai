/* SPDX-License-Identifier: MIT */
/*
 * awb.c: auto white balance as the vendor's isp daemon does it, on the
 * BPU's AWB statistics, with the sensor's calibration from its tuning's
 * shared page.
 */
#ifndef XMV200_AWB_H
#define XMV200_AWB_H

#include <stdint.h>

struct xmv200_isp_cfg;

/* the running ISP's channels: their calibration found in the tuning's
 * page; 0, or -1 when the page has none (AWB then stays off) */
int xmv200_awb_init(const struct xmv200_isp_cfg *c);
/* one frame of channel chn: its statistics read, the gains (isp.ko
 * 0x4965) and colour matrix (0x4966) sent when they change. 0 when it ran;
 * -1 when off, or the statistics had too few white zones (the gains then
 * stay as they were: at first the sensor's calibrated white) */
int xmv200_awb_frame(uint32_t chn);
void xmv200_awb_stop(void);
/* the sensor's total gain (100 = 1x), for the saturation by gain
 * (XM530V200's tunings: the chroma gain and the matrix's blend) */
void xmv200_awb_set_gain(uint32_t chn, uint32_t gain);
/* the chroma gain for that saturation, 0x100 = 1x: image.c's 0x49de U/V
 * base is scaled by it, as the daemon's frame thread does */
uint32_t xmv200_awb_chroma(uint32_t chn);
/* the gains last sent (0x4965 R, G, B, 0x100 = 1x): what the frame
 * thread sends with every frame where it answers them (XM533, XM535) */
int xmv200_awb_gains(uint32_t chn, uint32_t g[3]);

/* the last result of a channel: gains (0x100 = 1x) and colour temperature
 * (K); -1 when off */
int xmv200_awb_get(uint32_t chn, uint16_t gain[3], uint16_t *ct);

#endif
