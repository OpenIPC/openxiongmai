/* SPDX-License-Identifier: MIT */
/*
 * libxmvenc: H.264 and JPEG encoding on the Hantro 8290 (H1) encoder of
 * the XM530/XM550 and XM510, without vendor libraries
 * (OpenIPC/openxiongmai#13).
 *
 * The encoder is one piece of hardware: open it once per process. Each
 * channel is one stream (H.264 or JPEG) with its own buffers. Encodes on
 * different channels may come from different threads and take turns on
 * the hardware; one channel is encoded by one thread at a time.
 * xmvenc_chn_request_idr may be called from any thread.
 *
 * Needs an encoder driver (/dev/h264: vendor xm530_h264.ko or
 * xm510_h264.ko, or drivers/hx280enc), /dev/mmz and /dev/mem. The vendor
 * stack must not be encoding at the same time.
 */
#ifndef XMVENC_H
#define XMVENC_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

struct xmvenc;
struct xmvenc_chn;

/* A block of MMZ memory: physical address and an uncached mapping */
struct xmvenc_buf {
  uint32_t phys;
  uint8_t *virt;
  uint32_t size;
};

/* Opens the encoder and identifies the chip. Errors and notes go to log
 * (if not NULL), one line each without a newline. Returns NULL on
 * failure. */
struct xmvenc *xmvenc_open(void (*log)(const char *msg));
void xmvenc_close(struct xmvenc *enc);

/* "XM530/XM550" or "XM510", and the widest picture the encoder takes */
const char *xmvenc_chip(const struct xmvenc *enc);
int xmvenc_max_width(const struct xmvenc *enc);

/* Allocates a named MMZ block, or finds one of that name again: MMZ has
 * no free, so blocks outlive the process. Failing both, takes the block
 * named vendor (if not NULL) when it is big enough. */
int xmvenc_mmz_alloc(struct xmvenc *enc, struct xmvenc_buf *b,
                     const char *name, uint32_t size, const char *vendor);

enum xmvenc_codec { XMVENC_H264, XMVENC_JPEG };

struct xmvenc_chn_attr {
  enum xmvenc_codec codec;
  /* Picture size; width a multiple of 8, height even, 96 to max width.
   * stride is the luma row length in bytes, at least the width (0: the
   * width rounded up to 16). The input is NV12. */
  int width, height, stride;
  /* H.264 */
  int cabac; /* CABAC rather than CAVLC */
  int gop;   /* IDR interval in frames; 1 makes every frame an IDR */
  int qp;    /* fixed QP, or the starting QP with rate control */
  /* Frame-level rate control, when kbps > 0: QP within qmin..qmax */
  double kbps, fps;
  int qmin, qmax;
  /* JPEG quality, 1-100 */
  int quality;
  /* Allocate the reconstruction buffers for at least this many luma
   * bytes, so that later channels of up to that size can reuse them
   * (0: the picture's own size) */
  uint32_t min_luma;
};

/* Creates channel id (0-3; it names the channel's MMZ blocks). */
struct xmvenc_chn *xmvenc_chn_create(struct xmvenc *enc, int id,
                                     const struct xmvenc_chn_attr *attr);
void xmvenc_chn_destroy(struct xmvenc_chn *chn);

/* The next frame of the channel is an IDR, and the GOP restarts there */
void xmvenc_chn_request_idr(struct xmvenc_chn *chn);

struct xmvenc_frame {
  char type; /* 'I', 'P' or 'J' */
  int qp;    /* H.264 only */
  /* Software-written headers: SPS and PPS before an IDR, the JPEG header
   * up to SOS before a picture; otherwise hdr_len is 0. */
  const uint8_t *hdr;
  uint32_t hdr_len;
  /* What the hardware wrote: the slice NAL with its start code, or the
   * JPEG scan with EOI. Uncached memory, valid until the channel's next
   * encode. */
  const uint8_t *data;
  uint32_t len;
  uint32_t status; /* the encoder's status register after the frame */
  long us;         /* hardware time */
};

/* Encodes the NV12 picture at the given physical addresses. Blocks until
 * the frame is done; the input may be reused as soon as this returns.
 * Returns 0, or -1 when the encoder reports an error or the wait fails
 * (a signal interrupts it), with the encoder stopped. */
int xmvenc_encode(struct xmvenc_chn *chn, uint32_t phys_y, uint32_t phys_c,
                  struct xmvenc_frame *out);

#ifdef __cplusplus
}
#endif

#endif
