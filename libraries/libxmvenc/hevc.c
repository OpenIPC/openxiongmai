// SPDX-License-Identifier: MIT
/*
 * H.265 on the XM530/XM550 Mali-V550, through the vendor driver
 * (mve_base.ko, /dev/mv500) and its firmware (OpenIPC/openxiongmai#27).
 *
 * Each channel is one firmware session on a /dev/mv500 descriptor of its
 * own; the driver schedules the sessions on the engine, independently of
 * the H.264 encoder. The firmware does the rate control, the GOP and the
 * bitstream: VPS, SPS and PPS come once, before the first frame, and are
 * kept here to go before every IDR.
 *
 * Per frame: the input picture is mapped by its physical address (the
 * driver finds the pages from the user address), both it and the part of
 * the stream buffer to write are registered, queued, and unregistered
 * once the firmware hands them back.
 */
#define _DEFAULT_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

#include "mvb.h"
#include "xmvenc_priv.h"

/* GET_EVENT timeout, and how long a frame may take in all */
#define EVENT_MS 100
#define FRAME_MS 2000

struct xmvenc_hevc {
  struct xmvenc *enc;
  int id, fd, fd_mem, width, height, stride, broken;
  struct xmvenc_buf *strm;
  uint8_t hdr[256]; /* VPS, SPS and PPS */
  uint32_t hdr_len;
  uint32_t fps_sent, fps_want; /* Q16 */
  uint32_t last_status;        /* of the last command turned down */
  union {
    uint8_t b[512];
    uint32_t w[128];
  } io;
};

static uint64_t now_us(void) {
  struct timespec ts;

  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (uint64_t)ts.tv_sec * 1000000 + ts.tv_nsec / 1000;
}

static uint64_t handle_of(const void *p) {
  return (uint64_t)(int64_t)(intptr_t)p;
}

/* One command; the reply data is left after the struct mvb_reply in io.
 * Returns its size, or -1, with last_status set if the driver turned the
 * command down. Status quiet is not logged. */
static int cmd(struct xmvenc_hevc *h, uint32_t nr, const void *payload,
               uint32_t size, uint32_t quiet) {
  struct mvb_req *req = (struct mvb_req *)h->io.b;
  struct mvb_reply *rep = (struct mvb_reply *)h->io.b;

  h->last_status = 0;
  if (size > sizeof(h->io) - sizeof(*req))
    return -1;
  memset(h->io.b, 0, sizeof(h->io));
  req->cmd = nr;
  req->size = size;
  if (size)
    memcpy(h->io.b + sizeof(*req), payload, size);
  if (ioctl(h->fd, MVB_IOC_CMD, h->io.b) < 0) {
    xmvenc__log(h->enc, "H.265 channel %d: command %u: %s", h->id, nr,
                strerror(errno));
    return -1;
  }
  if (rep->status[0] || rep->status[1]) {
    h->last_status = rep->status[0] ? rep->status[0] : ~0u;
    if (h->last_status != quiet)
      xmvenc__log(h->enc, "H.265 channel %d: command %u: status %#x %#x", h->id,
                  nr, rep->status[0], rep->status[1]);
    return -1;
  }
  if (rep->size > sizeof(h->io) - sizeof(*rep)) {
    xmvenc__log(h->enc, "H.265 channel %d: command %u: %u-byte reply", h->id,
                nr, rep->size);
    return -1;
  }
  return rep->size;
}

static const uint8_t *reply(const struct xmvenc_hevc *h) {
  return h->io.b + sizeof(struct mvb_reply);
}

/* Takes an event: its code, or 0 for none within EVENT_MS, or -1. The
 * buffer of an INPUT/OUTPUT_DONE event goes to b. */
static int event(struct xmvenc_hevc *h, struct mvb_buf *b) {
  uint32_t timeout = EVENT_MS;
  struct mvb_event ev;
  int n = cmd(h, MVB_GET_EVENT, &timeout, sizeof(timeout), 0);

  if (n < 0)
    return -1;
  if (n < (int)sizeof(ev))
    return 0;
  memcpy(&ev, reply(h), sizeof(ev));
  if (ev.code == MVB_EVENT_INPUT_DONE || ev.code == MVB_EVENT_OUTPUT_DONE) {
    if (ev.size < sizeof(*b) || n < (int)(sizeof(ev) + sizeof(*b))) {
      xmvenc__log(h->enc, "H.265 channel %d: event %u with %d bytes", h->id,
                  ev.code, n);
      return -1;
    }
    memcpy(b, reply(h) + sizeof(ev), sizeof(*b));
  }
  return ev.code ? ev.code : 0x10000; /* code 0: not one we wait for */
}

/* A firmware message through SET_PARAMETER or SET_CONFIG. The driver
 * turns these down until the session has been switched in: take events
 * until it has. Only switch events can come then, no buffers are
 * queued. */
static int fw_msg(struct xmvenc_hevc *h, uint32_t nr, uint32_t size, int n,
                  const uint32_t *w) {
  uint32_t msg[MVB_OPT_SIZE / 4] = {0};
  struct mvb_buf b;

  memcpy(msg, w, n * sizeof(*w));
  for (int tries = 0;; tries++) {
    if (cmd(h, nr, msg, size, MVB_STATUS_NOT_SCHEDULED) >= 0)
      return 0;
    if (h->last_status != MVB_STATUS_NOT_SCHEDULED)
      return -1;
    if (tries == 20) {
      xmvenc__log(h->enc, "H.265 channel %d: session never switched in", h->id);
      return -1;
    }
    if (event(h, &b) < 0)
      return -1;
  }
}

static int send_frame_rate(struct xmvenc_hevc *h) {
  const uint32_t m[] = {MVB_FW_BUFFER_PARAM, MVB_PARAM_FRAME_RATE, h->fps_want};

  if (fw_msg(h, MVB_SET_CONFIG, sizeof(m), 3, m))
    return -1;
  h->fps_sent = h->fps_want;
  return 0;
}

static int reg(struct xmvenc_hevc *h, uint32_t port, void *addr,
               uint32_t size) {
  struct mvb_register r;

  memset(&r, 0, sizeof(r));
  r.port = port;
  r.buf.handle = r.buf.addr = handle_of(addr);
  r.buf.size = size;
  if (port == MVB_PORT_INPUT) {
    r.buf.width = h->width;
    r.buf.height = h->height;
    r.buf.stride = h->stride;
    r.buf.planes_or_one = 1;
    r.buf.format = MVB_FORMAT_NV12;
  } else {
    r.buf.format = MVB_FORMAT_HEVC;
  }
  return cmd(h, MVB_REGISTER_BUFFER, &r, sizeof(r), 0) < 0 ? -1 : 0;
}

static void unreg(struct xmvenc_hevc *h, void *addr) {
  uint64_t handle = handle_of(addr);

  cmd(h, MVB_UNREGISTER_BUFFER, &handle, sizeof(handle), 0);
}

static int queue(struct xmvenc_hevc *h, uint32_t nr, void *addr,
                 uint32_t filled) {
  struct mvb_queue q;

  memset(&q, 0, sizeof(q));
  q.handle = q.addr = handle_of(addr);
  q.filled = filled;
  q.in_flags = filled ? MVB_IN_FLAGS : 0;
  return cmd(h, nr, &q, sizeof(q), 0) < 0 ? -1 : 0;
}

struct xmvenc_hevc *xmvenc__hevc_create(struct xmvenc *enc, int id, int fd_mem,
                                        const struct xmvenc_chn_attr *a,
                                        int stride, struct xmvenc_buf *strm) {
  struct xmvenc_hevc *h = calloc(1, sizeof(*h));
  uint32_t state = MVB_STATE_EXECUTING, proto;
  int kbps = a->kbps > 0;

  if (!h)
    return NULL;
  h->enc = enc;
  h->id = id;
  h->fd_mem = fd_mem;
  h->width = a->width;
  h->height = a->height;
  h->stride = stride;
  h->strm = strm;
  h->fps_want = (uint32_t)((a->fps > 0 ? a->fps : 25) * 65536);
  if ((h->fd = open("/dev/mv500", O_RDWR | O_CLOEXEC)) < 0) {
    xmvenc__log(enc, "/dev/mv500: %s", strerror(errno));
    free(h);
    return NULL;
  }
  if (cmd(h, MVB_CREATE_SESSION, MVB_ROLE_HEVC_ENCODER,
          sizeof(MVB_ROLE_HEVC_ENCODER), 0) < 0 ||
      cmd(h, MVB_ACTIVATE_SESSION, NULL, 0, 0) < 4)
    goto fail;
  memcpy(&proto, reply(h), sizeof(proto));
  if (proto >> 8 != 2) {
    xmvenc__log(enc, "H.265 firmware protocol %u.%u, not 2.x", proto >> 8,
                proto & 0xff);
    goto fail;
  }

  /* P frames after each IDR: the firmware keeps the GOP. It reads the
   * count as 16 bits and refuses 0xffff, so longer GOPs (a pipe that asks
   * for each IDR itself) get 32767. Without rate control the QP range is
   * the one fixed QP, under a target that never binds. */
  const uint32_t pframes[] = {MVB_FW_SET_OPTION, MVB_OPT_P_FRAMES,
                              a->gop - 1 < 32767 ? (uint32_t)a->gop - 1
                                                 : 32767};
  const uint32_t rc[] = {MVB_FW_BUFFER_PARAM, MVB_PARAM_RATE_CONTROL,
                         MVB_RC_VARIABLE,
                         kbps ? (uint32_t)(a->kbps * 1000) : 100000000u};
  const uint32_t qp[] = {MVB_FW_BUFFER_PARAM, MVB_PARAM_QP_RANGE,
                         (uint32_t)(kbps ? a->qmin : a->qp),
                         (uint32_t)(kbps ? a->qmax : a->qp)};
  if (send_frame_rate(h) ||
      fw_msg(h, MVB_SET_PARAMETER, MVB_OPT_SIZE, 3, pframes) ||
      fw_msg(h, MVB_SET_CONFIG, sizeof(rc), 4, rc) ||
      fw_msg(h, MVB_SET_PARAMETER, sizeof(qp), 4, qp) ||
      cmd(h, MVB_STATE_CHANGE, &state, sizeof(state), 0) < 0)
    goto fail;
  return h;
fail:
  close(h->fd);
  free(h);
  return NULL;
}

void xmvenc__hevc_destroy(struct xmvenc_hevc *h) {
  if (!h)
    return;
  /* The driver ends the session with the descriptor */
  close(h->fd);
  free(h);
}

void xmvenc__hevc_frame_rate(struct xmvenc_hevc *h, double fps) {
  if (fps > 0 && fps < 1000)
    h->fps_want = (uint32_t)(fps * 65536);
}

int xmvenc__hevc_encode(struct xmvenc_hevc *h, uint32_t phys_y, uint32_t phys_c,
                        uint32_t off, uint32_t limit, int idr,
                        struct xmvenc_frame *out) {
  uint32_t size = (uint32_t)h->stride * h->height * 3 / 2;
  uint32_t base = phys_y & ~0xfffu, map_len;
  uint8_t *map, *in, *dst = h->strm->virt + off;
  int in_done = 0, out_done = 0, ret = -1;
  uint64_t t0, deadline;

  memset(out, 0, sizeof(out[0]));
  if (h->broken) {
    xmvenc__log(h->enc, "H.265 channel %d: stuck in an earlier frame", h->id);
    return -1;
  }
  /* The firmware takes one plane: chroma right after the luma */
  if (phys_c != phys_y + (uint32_t)h->stride * h->height) {
    xmvenc__log(h->enc, "H.265 channel %d: chroma at +%u, not after luma",
                h->id, phys_c - phys_y);
    return -1;
  }
  /* Settings between frames: frame rate changes of over 2%, and an IDR
   * (a GOP reset makes the next frame one) */
  if (h->fps_want && (h->fps_want > h->fps_sent + h->fps_sent / 50 ||
                      h->fps_want + h->fps_want / 50 < h->fps_sent))
    send_frame_rate(h);
  if (idr) {
    const uint32_t m[] = {MVB_FW_BUFFER_PARAM, MVB_PARAM_GOP_RESET};

    if (fw_msg(h, MVB_SET_CONFIG, sizeof(m), 2, m))
      return -1;
  }

  map_len = ((phys_y - base) + size + 0xfff) & ~0xfffu;
  map = mmap(0, map_len, PROT_READ, MAP_SHARED, h->fd_mem, base);
  if (map == MAP_FAILED) {
    xmvenc__log(h->enc, "H.265 channel %d: mmap input: %s", h->id,
                strerror(errno));
    return -1;
  }
  in = map + (phys_y - base);
  if (reg(h, MVB_PORT_INPUT, in, size))
    goto unmap;
  if (reg(h, MVB_PORT_OUTPUT, dst, limit))
    goto unreg_in;
  t0 = now_us();
  deadline = t0 + FRAME_MS * 1000;
  if (queue(h, MVB_EMPTY_THIS_BUFFER, in, size) ||
      queue(h, MVB_FILL_THIS_BUFFER, dst, 0))
    goto stuck;
  while (!in_done || !out_done) {
    struct mvb_buf b;
    int ev = event(h, &b);

    if (ev < 0 || now_us() > deadline) {
      xmvenc__log(h->enc, "H.265 channel %d frame: %s", h->id,
                  ev < 0 ? "event failed" : "no frame in time");
      goto stuck;
    }
    if (ev == MVB_EVENT_INPUT_DONE) {
      in_done = 1;
    } else if (ev == MVB_EVENT_OUTPUT_DONE) {
      if (b.offset > limit || b.filled > limit - b.offset) {
        xmvenc__log(h->enc, "H.265 channel %d: %u bytes at %u, room %u", h->id,
                    b.filled, b.offset, limit);
        goto stuck;
      }
      if (b.flags & MVB_FLAG_CODEC_CONFIG) {
        /* parameter sets: keep them, and have the buffer back */
        if (b.filled <= sizeof(h->hdr)) {
          memcpy(h->hdr, dst + b.offset, b.filled);
          h->hdr_len = b.filled;
        }
        if (queue(h, MVB_FILL_THIS_BUFFER, dst, 0))
          goto stuck;
        continue;
      }
      if (!(b.flags & MVB_FLAG_FRAME_END)) {
        xmvenc__log(h->enc, "H.265 channel %d: frame doesn't fit %u bytes",
                    h->id, limit);
        goto stuck;
      }
      out->type = b.flags & MVB_FLAG_SYNC ? 'I' : 'P';
      out->data = dst + b.offset;
      out->len = b.filled;
      out_done = 1;
    }
  }
  out->us = (long)(now_us() - t0);
  if (out->type == 'I') {
    out->hdr = h->hdr;
    out->hdr_len = h->hdr_len;
  }
  ret = 0;
  goto unreg_out;
stuck:
  /* The firmware may still hold the buffers: no more frames on this
   * session (destroying the channel ends it) */
  h->broken = 1;
unreg_out:
  unreg(h, dst);
unreg_in:
  unreg(h, in);
unmap:
  munmap(map, map_len);
  return ret;
}
