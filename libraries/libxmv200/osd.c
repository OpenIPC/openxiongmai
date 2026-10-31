// SPDX-License-Identifier: MIT
/*
 * Overlays on the streams: rgn.ko's (or open_rgn's) regions of kind 0,
 * which the OSD engine blends into each picture of the VENC channel they
 * are attached to before venc.ko queues it. A stream is VENC channel n.
 *
 * Every rgn ioctl takes 15 words: the handle, the kind, then the call's.
 */
#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "xmv200.h"

#define RGN_CREATE 0x403c5201
#define RGN_DESTROY 0x403c5202
#define RGN_SET_ATTR 0x403c5203
#define RGN_BITMAP 0x403c5204
#define RGN_ATTACH 0x403c5207
#define RGN_DETACH 0x403c5208
#define MOD_VENC 7

static int rgn_fd = -1;

static int rgn(unsigned long cmd, int handle, const uint32_t *w, int n) {
  uint32_t a[15] = {(uint32_t)handle, 0};

  if (rgn_fd < 0 && (rgn_fd = open("/dev/rgn", O_RDWR)) < 0)
    return -1;
  if (handle < 0 || handle >= XMV200_OSD_MAX)
    return -1;
  memcpy(a + 2, w, n * sizeof(uint32_t));
  return ioctl(rgn_fd, cmd, a) < 0 ? -1 : 0;
}

/* words 2..14 of create and set */
static int attr(const struct xmv200_osd *o, uint32_t w[13]) {
  if (o->format > XMV200_OSD_ENC_2BIT || o->scale_x > 2 || o->scale_y > 2)
    return -1;
  w[0] = o->format;
  w[1] = o->x & ~1u; /* chroma pairs */
  w[2] = o->y & ~1u;
  w[3] = w[5] = o->width;
  w[4] = w[6] = o->height;
  memcpy(w + 7, o->color, sizeof(o->color));
  w[11] = o->alpha;
  /* 1 or 2 across and down: 0 or 1 in either nibble */
  w[12] = (o->scale_y == 2) << 4 | (o->scale_x == 2);
  return 0;
}

int xmv200_osd_create(int handle, const struct xmv200_osd *o) {
  uint32_t w[13];

  return attr(o, w) ? -1 : rgn(RGN_CREATE, handle, w, 13);
}

int xmv200_osd_set(int handle, const struct xmv200_osd *o) {
  uint32_t w[13];

  return attr(o, w) ? -1 : rgn(RGN_SET_ATTR, handle, w, 13);
}

int xmv200_osd_bitmap(int handle, const void *data, uint32_t len) {
  uint32_t w[2] = {(uint32_t)(uintptr_t)data, len};

  return rgn(RGN_BITMAP, handle, w, 2);
}

int xmv200_osd_attach(int handle, int stream) {
  uint32_t w[3] = {MOD_VENC, 0, (uint32_t)stream};

  return rgn(RGN_ATTACH, handle, w, 3);
}

int xmv200_osd_detach(int handle, int stream) {
  uint32_t w[3] = {MOD_VENC, 0, (uint32_t)stream};

  return rgn(RGN_DETACH, handle, w, 3);
}

int xmv200_osd_destroy(int handle) { return rgn(RGN_DESTROY, handle, NULL, 0); }

uint32_t xmv200_osd_color(uint8_t r, uint8_t g, uint8_t b, uint8_t a) {
  /* BT.601, video range */
  int y = 16 + (66 * r + 129 * g + 25 * b + 128) / 256;
  int u = 128 + (-38 * r - 74 * g + 112 * b + 128) / 256;
  int v = 128 + (112 * r - 94 * g - 18 * b + 128) / 256;

  return (uint32_t)a << 24 | y << 16 | u << 8 | v;
}
