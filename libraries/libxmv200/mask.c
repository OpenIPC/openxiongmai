// SPDX-License-Identifier: MIT
/*
 * Privacy masks: filled rectangles over a sensor's picture, on every
 * stream that shows it. Each is one rgn.ko (open_rgn) cover per stream:
 * kind 1, attached to the VPSS channel that makes the stream from the
 * sensor's group (XM530V200, XM535D1/D2), or to the VI channel that makes
 * it (XM533, XM535W1). The VPSS or VI draws it into the channel's picture,
 * so it is in each stream before encoding, at that stream's scale.
 *
 * Cover words (after the handle and kind): x, y, w, h, colour, fill (1),
 * thickness; attach: module (8 VPSS, 0x10 VI), dev (the group), chn. The
 * colour is 0xYYUUVV: Sofia converts its RGB to that before rgn.ko.
 */
#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "xmv200.h"
#include "xmv200_priv.h"

#define RGN_CREATE 0x403c5201
#define RGN_DESTROY 0x403c5202
#define RGN_SET_ATTR 0x403c5203
#define RGN_ATTACH 0x403c5207
#define RGN_DETACH 0x403c5208
#define KIND_COVER 1
#define MOD_VPSS 8
#define MOD_VI 0x10

static int rgn_fd = -1;
static struct {
  int on;
  struct xmv200_mask m;
  uint32_t streams; /* the streams it has a cover on */
} masks[XMV200_MASK_MAX];

void xmv200_mask_reset(void) { memset(masks, 0, sizeof(masks)); }

/* the cover handles (covers have their own, apart from the OSD's
 * overlays): one per mask and stream, from the top, leaving the low ones to
 * an application's own covers */
static int handle(int id, int s) {
  return 255 - (id * XMV200_MAX_STREAMS + s);
}

static int rgn(unsigned long cmd, int h, const uint32_t *w, int n) {
  uint32_t a[15] = {(uint32_t)h, KIND_COVER};

  if (rgn_fd < 0 && (rgn_fd = open("/dev/rgn", O_RDWR)) < 0)
    return -1;
  memcpy(a + 2, w, n * sizeof(uint32_t));
  return ioctl(rgn_fd, cmd, a) < 0 ? -1 : 0;
}

/* the VPSS channel stream s is of sensor g: its place among the streams
 * with g (board.c sets them up in that order) */
static uint32_t vpss_chn(const struct xmv200_geom *g, int sensor, int s) {
  uint32_t vc = 0;

  for (int k = 0; k < s; k++)
    if (g->stream[k].sensors & 1u << sensor)
      vc++;
  return vc;
}

static uint32_t at(uint32_t permille, uint32_t size) {
  return (uint32_t)((uint64_t)(permille > 1000 ? 1000 : permille) * size /
                    1000) &
         ~1u;
}

/* the cover's words for stream s: the rectangle in its picture */
static int cover(const struct xmv200_geom *g, int s,
                 const struct xmv200_mask *m, uint32_t w[7]) {
  uint32_t W = g->stream[s].width, H = g->stream[s].height;
  uint32_t x0 = at(m->x, W), y0 = at(m->y, H);
  uint32_t x1 = at(m->x + m->width, W), y1 = at(m->y + m->height, H);

  if (x1 <= x0 || y1 <= y0)
    return -1;
  w[0] = x0;
  w[1] = y0;
  w[2] = x1 - x0;
  w[3] = y1 - y0;
  w[4] = m->color & 0xffffff;
  w[5] = 1; /* filled */
  w[6] = 2;
  return 0;
}

static void remove_covers(int id) {
  struct xmv200_geom g;
  int sensor = masks[id].m.sensor, live = !xmv200_layout_geom(&g);

  for (int s = 0; s < XMV200_MAX_STREAMS; s++)
    if (masks[id].streams & 1u << s) {
      /* where it is attached, as it was made */
      uint32_t a[3] = {live && g.vi ? MOD_VI : MOD_VPSS,
                       live && g.vi ? 0 : sensor,
                       !live ? 0 : g.vi ? (uint32_t)s : vpss_chn(&g, sensor, s)};

      rgn(RGN_DETACH, handle(id, s), a, 3);
      rgn(RGN_DESTROY, handle(id, s), NULL, 0);
    }
  masks[id].streams = 0;
  masks[id].on = 0;
}

int xmv200_mask_set(int id, const struct xmv200_mask *m) {
  struct xmv200_geom g;
  int r = 0;

  if (id < 0 || id >= XMV200_MASK_MAX || xmv200_layout_geom(&g) ||
      (int)m->sensor >= g.n_sensors || (g.vi && m->sensor))
    return -1;
  /* a moved mask on the same streams keeps its covers */
  if (masks[id].on && masks[id].m.sensor != m->sensor)
    remove_covers(id);
  for (int s = 0; s < g.n_streams; s++) {
    uint32_t w[7];
    int h = handle(id, s);

    if (!(g.stream[s].sensors & 1u << m->sensor))
      continue;
    if (cover(&g, s, m, w)) {
      r = -1;
      continue;
    }
    if (masks[id].streams & 1u << s) {
      r |= rgn(RGN_SET_ATTR, h, w, 7);
      continue;
    }
    uint32_t a[3] = {g.vi ? MOD_VI : MOD_VPSS, g.vi ? 0 : m->sensor,
                     g.vi ? (uint32_t)s : vpss_chn(&g, m->sensor, s)};

    if (rgn(RGN_CREATE, h, w, 7)) {
      r = -1;
      continue;
    }
    if (rgn(RGN_ATTACH, h, a, 3)) {
      rgn(RGN_DESTROY, h, NULL, 0);
      r = -1;
      continue;
    }
    masks[id].streams |= 1u << s;
  }
  masks[id].on = 1;
  masks[id].m = *m;
  return r;
}

int xmv200_mask_clear(int id) {
  if (id < 0 || id >= XMV200_MASK_MAX)
    return -1;
  remove_covers(id);
  return 0;
}
