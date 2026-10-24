/* SPDX-License-Identifier: MIT */
/* libxmvenc internals shared between its source files */
#ifndef XMVENC_PRIV_H
#define XMVENC_PRIV_H

#include "xmvenc.h"

void xmvenc__log(struct xmvenc *enc, const char *fmt, ...)
    __attribute__((format(printf, 2, 3)));

/* Encodes into the channel's stream buffer at off, writing at most limit
 * bytes (both multiples of 64) */
int xmvenc__encode_at(struct xmvenc_chn *chn, uint32_t phys_y,
                      uint32_t phys_c, uint32_t off, uint32_t limit,
                      struct xmvenc_frame *out);
uint32_t xmvenc__strm_size(const struct xmvenc_chn *chn);
/* The frame rate the channel actually runs at, for rate control: its
 * per-frame budget is the bitrate over this */
void xmvenc__frame_rate(struct xmvenc_chn *chn, double fps);



/* A VI channel, read from a pipe's thread (vi.c) */
struct xmvenc_vi {
  int fd, chn, held;
  uint32_t rec_size; /* 0 until a get has worked */
  uint32_t rec[8];   /* the record of the frame held, for its release */
  uint32_t phys_y, phys_c, width, height;
  uint64_t pts;   /* microseconds, CLOCK_MONOTONIC-based */
  uint32_t pts32; /* the low half of VI's last timestamp */
};

/* rec_size: the frame record's size if known (32 on XM530, 24 on XM510),
 * or 0 to find it out */
int xmvenc__vi_open(struct xmvenc_vi *vi, int chn, uint32_t rec_size);
void xmvenc__vi_close(struct xmvenc_vi *vi);
/* Takes the next frame, or fails when none is ready yet */
int xmvenc__vi_get(struct xmvenc_vi *vi);
void xmvenc__vi_release(struct xmvenc_vi *vi);
#endif
