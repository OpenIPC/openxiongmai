// SPDX-License-Identifier: MIT
/*
 * Pipes: a channel fed from VI by a thread of its own.
 *
 * Encoded frames queue up in the channel's stream buffer, used as a ring:
 * each frame is written after the previous one, or back at the start
 * when the end has too little room, and the encoder is told how much
 * room there is (it stops there, with "buffer full"). Packets are
 * released in order, which frees the ring from its oldest end.
 */
#define _DEFAULT_SOURCE
#include <errno.h>
#include <pthread.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

#include "xmvenc_priv.h"

#define DEPTH 32 /* packets queued at most */
#define ALIGN 64 /* frames start on 64-byte boundaries */

struct slot {
  uint32_t off;
  struct xmvenc_packet pkt;
};

struct xmvenc_pipe {
  struct xmvenc *enc;
  struct xmvenc_chn *chn;
  struct xmvenc_vi vi;
  int width, height;
  uint64_t period; /* microseconds between frames, 0: every VI frame */
  uint32_t ring;   /* the stream buffer's size */

  pthread_mutex_t ctl; /* start, stop and destroy, one at a time */
  pthread_mutex_t lock;
  pthread_cond_t cond;
  pthread_t thread;
  int joinable; /* a thread was started and not joined yet */
  int run;      /* the thread is to keep going */
  int frames;   /* frames still to encode, 0: no limit */
  /* Packets put, taken and released so far; slot i % DEPTH */
  unsigned put, got, rel;
  struct slot q[DEPTH];
  uint32_t head; /* where the next frame goes */
  uint32_t seq;
  /* Interval between encoded frames, smoothed, in microseconds (0: none
   * yet); rate control is fed the frame rate it gives */
  double interval;
  uint64_t last_pts;
  double fps; /* the configured rate, rate control's start */
  uint64_t gop_us;   /* keyframe interval in time, 0: in frames */
  uint64_t last_idr; /* timestamp of the last IDR, 0: none yet */
  struct xmvenc_pipe_stats stats;
};

static uint64_t now_us(void) {
  struct timespec ts;

  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (uint64_t)ts.tv_sec * 1000000 + ts.tv_nsec / 1000;
}

/* Room for the next frame, under the lock: 0 and the place, or -1. A
 * frame needs at least a quarter of the ring, or it would likely run out
 * of room half-way and be lost after its encode. */
static int reserve(struct xmvenc_pipe *p, uint32_t *off, uint32_t *limit) {
  uint32_t min = p->ring / 4, tail;

  if (p->put - p->rel >= DEPTH)
    return -1;
  if (p->put == p->rel) {
    *off = 0;
    *limit = p->ring;
    return 0;
  }
  tail = p->q[p->rel % DEPTH].off;
  if (p->head > tail) {
    /* free: from head to the end, and from the start to tail */
    if (p->ring - p->head >= min) {
      *off = p->head;
      *limit = p->ring - p->head;
      return 0;
    }
    *off = 0;
    *limit = tail;
  } else {
    /* head == tail with packets queued: full */
    *off = p->head;
    *limit = tail - p->head;
  }
  return *limit >= min ? 0 : -1;
}

/* Rate control budgets per frame: after each frame encoded, tell it the
 * rate frames really come at, which the configured fps may not be (a 15 fps
 * sensor). Gaps much longer than a frame (a stall) are not counted. */
static void measure_rate(struct xmvenc_pipe *p) {
  double gap = p->fps > 0 ? 4e6 / p->fps : 0;

  if (gap < 1e6)
    gap = 1e6;
  if (p->last_pts && p->vi.pts > p->last_pts &&
      p->vi.pts - p->last_pts < gap) {
    double d = (double)(p->vi.pts - p->last_pts);

    p->interval = p->interval ? p->interval * 0.9 + d * 0.1 : d;
    xmvenc__frame_rate(p->chn, 1e6 / p->interval);
  }
  p->last_pts = p->vi.pts;
}

static void *feeder(void *arg) {
  struct xmvenc_pipe *p = arg;
  uint64_t next = 0;
  int size_warned = 0;

  pthread_mutex_lock(&p->lock);
  while (p->run) {
    struct xmvenc_frame fr;
    uint32_t off, limit;
    uint64_t t;
    int ret;

    pthread_mutex_unlock(&p->lock);
    if (xmvenc__vi_get(&p->vi)) {
      usleep(5000);
      pthread_mutex_lock(&p->lock);
      continue;
    }
    t = now_us();
    if ((int)p->vi.width != p->width || (int)p->vi.height != p->height) {
      if (!size_warned++)
        xmvenc__log(p->enc, "VI channel %d delivers %ux%u, not %dx%d",
                    p->vi.chn, p->vi.width, p->vi.height, p->width,
                    p->height);
      xmvenc__vi_release(&p->vi);
      usleep(100000);
      pthread_mutex_lock(&p->lock);
      continue;
    }
    /* Keep the frame rate: take a frame once its time has (nearly) come */
    if (p->period && next && t + p->period / 4 < next) {
      xmvenc__vi_release(&p->vi);
      pthread_mutex_lock(&p->lock);
      continue;
    }
    if (p->period)
      next = next && t < next + p->period ? next + p->period : t + p->period;

    pthread_mutex_lock(&p->lock);
    ret = reserve(p, &off, &limit);
    if (ret) {
      /* The reader is behind: skip the frame, the stream stays valid;
       * the gap in seq shows it */
      p->stats.skipped++;
      p->seq++;
      pthread_mutex_unlock(&p->lock);
      xmvenc__vi_release(&p->vi);
      pthread_mutex_lock(&p->lock);
      continue;
    }
    pthread_mutex_unlock(&p->lock);

    /* The region is ours: the reader only frees others */
    /* Due a keyframe: half a frame early rather than one frame late */
    if (p->gop_us &&
        (!p->last_idr ||
         p->vi.pts + (uint64_t)(p->interval / 2) >= p->last_idr + p->gop_us))
      xmvenc_chn_request_idr(p->chn);
    ret = xmvenc__encode_at(p->chn, p->vi.phys_y, p->vi.phys_c, off, limit,
                            &fr);
    xmvenc__vi_release(&p->vi);

    pthread_mutex_lock(&p->lock);
    if (ret) {
      p->stats.failed++;
    } else {
      struct slot *s = &p->q[p->put % DEPTH];

      s->off = off;
      s->pkt.type = fr.type;
      s->pkt.qp = fr.qp;
      s->pkt.hdr = fr.hdr;
      s->pkt.hdr_len = fr.hdr_len;
      s->pkt.data = fr.data;
      s->pkt.len = fr.len;
      s->pkt.pts = p->vi.pts;
      s->pkt.seq = p->seq;
      p->head = off + ((fr.len + ALIGN - 1) & ~(ALIGN - 1u));
      p->put++;
      p->stats.encoded++;
      if (p->frames && --p->frames == 0)
        p->run = 0;
      measure_rate(p);
      /* Requested IDRs count too: the interval runs from the last one */
      if (fr.type == 'I')
        p->last_idr = p->vi.pts;
    }
    p->seq++;
    pthread_cond_broadcast(&p->cond);
  }
  /* wake readers waiting on a pipe that no longer runs */
  pthread_cond_broadcast(&p->cond);
  pthread_mutex_unlock(&p->lock);
  return NULL;
}

struct xmvenc_pipe *xmvenc_pipe_create(struct xmvenc *enc, int id, int vi_chn,
                                       const struct xmvenc_chn_attr *attr) {
  struct xmvenc_pipe *p = calloc(1, sizeof(*p));
  pthread_condattr_t ca;

  if (!p)
    return NULL;
  p->enc = enc;
  p->width = attr->width;
  p->height = attr->height;
  p->period = attr->fps > 0 ? (uint64_t)(1000000 / attr->fps) : 0;
  p->fps = attr->fps;
  /* XM510's encoder takes at most 1280 pixels a line */
  if (xmvenc__vi_open(&p->vi, vi_chn, xmvenc_max_width(enc) <= 1280 ? 24 : 32)) {
    xmvenc__log(enc, "/dev/vi: %s", strerror(errno));
    free(p);
    return NULL;
  }
  struct xmvenc_chn_attr a = *attr;
  /* With a keyframe interval in time, the pipe asks for every IDR itself */
  if (a.gop_us)
    a.gop = 1 << 30;
  p->gop_us = a.gop_us;
  if (!(p->chn = xmvenc_chn_create(enc, id, &a))) {
    xmvenc__vi_close(&p->vi);
    free(p);
    return NULL;
  }
  p->ring = xmvenc__strm_size(p->chn) & ~(ALIGN - 1u);
  pthread_mutex_init(&p->ctl, NULL);
  pthread_mutex_init(&p->lock, NULL);
  pthread_condattr_init(&ca);
  pthread_condattr_setclock(&ca, CLOCK_MONOTONIC);
  pthread_cond_init(&p->cond, &ca);
  pthread_condattr_destroy(&ca);
  return p;
}

void xmvenc_pipe_destroy(struct xmvenc_pipe *p) {
  if (!p)
    return;
  xmvenc_pipe_stop(p);
  xmvenc__vi_close(&p->vi);
  xmvenc_chn_destroy(p->chn);
  pthread_cond_destroy(&p->cond);
  pthread_mutex_destroy(&p->lock);
  pthread_mutex_destroy(&p->ctl);
  free(p);
}

int xmvenc_pipe_start(struct xmvenc_pipe *p, int frames) {
  int ret = 0;

  pthread_mutex_lock(&p->ctl);
  pthread_mutex_lock(&p->lock);
  p->frames = frames > 0 ? frames : 0;
  if (p->run) {
    pthread_mutex_unlock(&p->lock);
    pthread_mutex_unlock(&p->ctl);
    return 0;
  }
  pthread_mutex_unlock(&p->lock);
  /* a thread that ended by itself, after its frames */
  if (p->joinable) {
    pthread_join(p->thread, NULL);
    p->joinable = 0;
  }
  pthread_mutex_lock(&p->lock);
  p->run = 1;
  /* A gap while stopped is no frame interval, and the rate measured before
   * says nothing about the next run: start again from the configured one */
  p->last_pts = 0;
  p->interval = 0;
  p->last_idr = 0; /* a restarted stream starts with an IDR */
  xmvenc__frame_rate(p->chn, p->fps);
  pthread_mutex_unlock(&p->lock);
  if (pthread_create(&p->thread, NULL, feeder, p)) {
    pthread_mutex_lock(&p->lock);
    p->run = 0;
    pthread_mutex_unlock(&p->lock);
    xmvenc__log(p->enc, "pipe: no thread");
    ret = -1;
  } else {
    p->joinable = 1;
  }
  pthread_mutex_unlock(&p->ctl);
  return ret;
}

void xmvenc_pipe_stop(struct xmvenc_pipe *p) {
  pthread_mutex_lock(&p->ctl);
  pthread_mutex_lock(&p->lock);
  p->run = 0;
  pthread_cond_broadcast(&p->cond);
  pthread_mutex_unlock(&p->lock);
  if (p->joinable) {
    pthread_join(p->thread, NULL);
    p->joinable = 0;
  }
  pthread_mutex_unlock(&p->ctl);
}

struct xmvenc_chn *xmvenc_pipe_chn(struct xmvenc_pipe *p) { return p->chn; }

int xmvenc_pipe_get(struct xmvenc_pipe *p, struct xmvenc_packet *pkt,
                    int timeout_ms) {
  struct timespec ts;

  clock_gettime(CLOCK_MONOTONIC, &ts);
  if (timeout_ms > 0) {
    ts.tv_sec += timeout_ms / 1000;
    ts.tv_nsec += (long)(timeout_ms % 1000) * 1000000;
    if (ts.tv_nsec >= 1000000000) {
      ts.tv_sec++;
      ts.tv_nsec -= 1000000000;
    }
  }
  pthread_mutex_lock(&p->lock);
  while (p->got == p->put) {
    if (!p->run) {
      pthread_mutex_unlock(&p->lock);
      errno = EAGAIN;
      return -1;
    }
    if (timeout_ms == 0 ||
        (timeout_ms > 0 &&
         pthread_cond_timedwait(&p->cond, &p->lock, &ts) == ETIMEDOUT &&
         p->got == p->put)) {
      pthread_mutex_unlock(&p->lock);
      errno = ETIMEDOUT;
      return -1;
    }
    if (timeout_ms < 0)
      pthread_cond_wait(&p->cond, &p->lock);
  }
  *pkt = p->q[p->got % DEPTH].pkt;
  p->got++;
  pthread_mutex_unlock(&p->lock);
  return 0;
}

void xmvenc_pipe_release(struct xmvenc_pipe *p,
                         const struct xmvenc_packet *pkt) {
  (void)pkt;
  pthread_mutex_lock(&p->lock);
  if (p->rel != p->got)
    p->rel++;
  pthread_mutex_unlock(&p->lock);
}

void xmvenc_pipe_stats(struct xmvenc_pipe *p, struct xmvenc_pipe_stats *st) {
  pthread_mutex_lock(&p->lock);
  *st = p->stats;
  pthread_mutex_unlock(&p->lock);
}
