// SPDX-License-Identifier: MIT
/*
 * The sensors and modes xmisp_start() tries, in order: each one's probe
 * runs until a chip ID matches. The vendor daemon's order: SOI at 0x30
 * (8-bit registers), SmartSens at 0x30 (16-bit), OmniVision at 0x36 with
 * the ID in 0x300a/0x300b, onsemi at 0x10, OmniVision with the ID in
 * 0x300b/0x300c, ImageDesign at 0x30 (8-bit).
 */
#include <stddef.h>

#include "script.h"
#include "xmisp.h"

extern const struct xmisp_script xmisp_h42_720p, xmisp_h65_720p,
    xmisp_sc1235_720p, xmisp_sc1245_720p, xmisp_sc2235_1080p, xmisp_ov9732_720p,
    xmisp_ar0130_720p, xmisp_ov9750_720p, xmisp_mis1002_720p;

const struct xmisp_script *const xmisp_scripts[] = {
    &xmisp_h62_720p,
    &xmisp_h42_720p,
    &xmisp_h65_720p,
    &xmisp_sc1235_720p,
    &xmisp_sc1245_720p,
    &xmisp_sc2235_1080p,
    &xmisp_ov9732_720p,
    &xmisp_ar0130_720p,
    &xmisp_ov9750_720p,
    &xmisp_mis1002_720p,
    NULL,
};
