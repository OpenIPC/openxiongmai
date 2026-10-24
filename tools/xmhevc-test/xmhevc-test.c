// SPDX-License-Identifier: MIT
/*
 * xmhevc-test: encode H.265 on the XM530/XM550 Mali-V550 through the
 * vendor driver's /dev/mv500 (mve_base.ko), from a synthetic NV12
 * picture, without vendor libraries (OpenIPC/openxiongmai#27, H1).
 *
 * Needs mve_base.ko with its firmware, /dev/mmz and the H.264 encoder
 * driver (libxmvenc allocates the MMZ). Stop majestic first.
 *
 * Usage: xmhevc-test [-w width] [-h height] [-n frames] [-g gop]
 *                    [-f fps] [-b kbps] [-Q qmin:qmax] [-o file] [-v]
 *
 * Writes the stream (parameter sets, then one access unit per frame) to
 * -o, default /tmp/xmhevc.h265. -g is the IDR interval in frames, -b the
 * target of variable bitrate control (default 4096) with the QP within
 * -Q (default 24:32). -v logs every driver event.
 */
#define _DEFAULT_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "mvb.h"
#include "xmvenc.h"

static int fd_mv = -1, verbose;

/* Room for the largest request (8 + 208) and reply (12 + 4 + 120) */
static union {
  uint8_t b[512];
  uint32_t w[128];
} io;

static void log_line(const char *msg) { fprintf(stderr, "%s\n", msg); }

/* Sends one command; the reply data, if any, is left in io after the
 * struct mvb_reply. Returns the reply data size, or -1. */
static int mvb(uint32_t cmd, const void *payload, uint32_t size,
               uint32_t quiet_status) {
  struct mvb_req *req = (struct mvb_req *)io.b;
  struct mvb_reply *rep = (struct mvb_reply *)io.b;

  if (size > sizeof(io) - sizeof(*req))
    return -1;
  memset(io.b, 0, sizeof(io));
  req->cmd = cmd;
  req->size = size;
  if (size)
    memcpy(io.b + sizeof(*req), payload, size);
  if (ioctl(fd_mv, MVB_IOC_CMD, io.b) < 0) {
    fprintf(stderr, "mv500 command %u: %s\n", cmd, strerror(errno));
    return -1;
  }
  if (rep->status[0] || rep->status[1]) {
    if (rep->status[0] != quiet_status || rep->status[1])
      fprintf(stderr, "mv500 command %u: status %#x %#x\n", cmd, rep->status[0],
              rep->status[1]);
    return -1;
  }
  return rep->size;
}

static const uint8_t *reply_data(void) {
  return io.b + sizeof(struct mvb_reply);
}

static int fw_msg(uint32_t cmd, uint32_t size, int n, const uint32_t *w) {
  uint32_t msg[MVB_OPT_SIZE / 4] = {0};

  memcpy(msg, w, n * sizeof(*w));
  /* Until the session is first switched in, the driver turns these down
   * with status 8: take events until it is, then try again */
  for (int tries = 0;; tries++) {
    struct mvb_reply *rep = (struct mvb_reply *)io.b;
    uint32_t timeout = 100;

    if (mvb(cmd, msg, size, MVB_STATUS_NOT_SCHEDULED) >= 0)
      return 0;
    if (rep->status[0] != MVB_STATUS_NOT_SCHEDULED)
      return -1;
    if (tries == 20) {
      fprintf(stderr, "mv500 command %u: session never switched in\n", cmd);
      return -1;
    }
    if (mvb(MVB_GET_EVENT, &timeout, sizeof(timeout), 0) < 0)
      return -1;
  }
}

static uint64_t handle_of(const void *p) {
  return (uint64_t)(int64_t)(intptr_t)p;
}

/* A luma ramp with a bright square that moves 8 pixels a frame, over a
 * chroma ramp. Drawn in pic, then copied: MMZ is mapped uncached, and
 * drawing there byte by byte is slow. ramp holds w bytes. */
static void fill_pattern(uint8_t *mmz, uint8_t *pic, uint8_t *ramp, int w,
                         int h, int frame) {
  int bx = (frame * 8) % (w > 128 ? w - 128 : 1), by = h / 3;
  uint8_t *y = pic, *uv = pic + w * h;

  for (int i = 0; i < w; i++)
    ramp[i] = 16 + i * 200 / w;
  for (int j = 0; j < h; j++) {
    uint8_t *row = y + j * w, add = j * 19 / h;

    for (int i = 0; i < w; i++)
      row[i] = ramp[i] + add;
    if (j >= by && j < by + 128)
      memset(row + bx, 235, bx + 128 <= w ? 128 : w - bx);
  }
  for (int i = 0; i < w / 2; i++)
    ramp[i] = 64 + i * 128 / (w / 2);
  for (int j = 0; j < h / 2; j++) {
    uint8_t *row = uv + j * w, cr = 64 + j * 128 / (h / 2);

    for (int i = 0; i < w / 2; i++) {
      row[2 * i] = ramp[i];
      row[2 * i + 1] = cr;
    }
  }
  memcpy(mmz, pic, w * h * 3 / 2);
}

static uint32_t in_size(int w, int h) {
  uint32_t need = w * h * 3 / 2, hd = 1920 * 1088 * 3 / 2;

  return need > hd ? need : hd;
}

static int parse_range(const char *s, int *lo, int *hi) {
  return sscanf(s, "%d:%d", lo, hi) == 2 && *lo >= 0 && *lo <= *hi && *hi <= 51
             ? 0
             : -1;
}

int main(int argc, char **argv) {
  int w = 1920, h = 1080, frames = 25, gop = 25, fps = 25, qmin = 24, qmax = 32;
  long kbps = 4096;
  const char *out_path = "/tmp/xmhevc.h265";
  struct xmvenc_buf in = {0}, out = {0};
  struct xmvenc *enc;
  uint8_t *pic;
  FILE *f;
  int c, ret = 1, n, sent = 1, done = 0, in_busy, bytes = 0;

  while ((c = getopt(argc, argv, "w:h:n:g:f:b:Q:o:v")) != -1)
    switch (c) {
    case 'w':
      w = atoi(optarg);
      break;
    case 'h':
      h = atoi(optarg);
      break;
    case 'n':
      frames = atoi(optarg);
      break;
    case 'g':
      gop = atoi(optarg);
      break;
    case 'f':
      fps = atoi(optarg);
      break;
    case 'b':
      kbps = atol(optarg);
      break;
    case 'Q':
      if (parse_range(optarg, &qmin, &qmax)) {
        fprintf(stderr, "-Q wants qmin:qmax within 0..51\n");
        return 2;
      }
      break;
    case 'o':
      out_path = optarg;
      break;
    case 'v':
      verbose = 1;
      break;
    default:
      fprintf(stderr,
              "usage: %s [-w width] [-h height] [-n frames] "
              "[-g gop] [-f fps] [-b kbps] [-Q qmin:qmax] "
              "[-o file] [-v]\n",
              argv[0]);
      return 2;
    }
  if (w < 64 || h < 64 || w % 16 || h % 2 || frames < 1 || gop < 1 || fps < 1 ||
      fps > 240 || kbps < 1 || kbps > 100000) {
    fprintf(stderr, "bad size, frame count, gop, fps or bitrate\n");
    return 2;
  }

  if (!(pic = malloc(w * h * 3 / 2 + w)))
    return 1;
  if (!(enc = xmvenc_open(log_line))) {
    free(pic);
    return 1;
  }
  /* MMZ blocks outlive the process and keep their first size: make the
   * input big enough for 1080p whatever this run's size */
  if (xmvenc_mmz_alloc(enc, &in, "xmhevc-in", in_size(w, h), NULL) ||
      xmvenc_mmz_alloc(enc, &out, "xmhevc-out", 1 << 20, NULL)) {
    fprintf(stderr, "MMZ allocation failed\n");
    goto close_enc;
  }
  if (!(f = fopen(out_path, "wb"))) {
    perror(out_path);
    goto close_enc;
  }
  if ((fd_mv = open("/dev/mv500", O_RDWR)) < 0) {
    perror("/dev/mv500");
    goto close_file;
  }

  if (mvb(MVB_CREATE_SESSION, MVB_ROLE_HEVC_ENCODER,
          sizeof(MVB_ROLE_HEVC_ENCODER), 0) < 0 ||
      mvb(MVB_ACTIVATE_SESSION, NULL, 0, 0) < 4)
    goto close_mv;
  {
    uint32_t proto;

    memcpy(&proto, reply_data(), 4);
    fprintf(stderr, "firmware protocol %u.%u\n", proto >> 8, proto & 0xff);
  }
  if (mvb(MVB_READ_HW_INFO, NULL, 0, 0) >= 12) {
    uint32_t hw[3];

    memcpy(hw, reply_data(), sizeof(hw));
    fprintf(stderr, "hardware %08x, %u core(s), fuse %#x\n", hw[1], hw[2],
            hw[0]);
  }

  {
    struct mvb_register r = {0};

    r.port = MVB_PORT_INPUT;
    r.buf.handle = r.buf.addr = handle_of(in.virt);
    r.buf.size = w * h * 3 / 2;
    r.buf.width = r.buf.stride = w;
    r.buf.height = h;
    r.buf.planes_or_one = 1;
    r.buf.format = MVB_FORMAT_NV12;
    if (mvb(MVB_REGISTER_BUFFER, &r, sizeof(r), 0) < 0)
      goto close_mv;
    memset(&r, 0, sizeof(r));
    r.port = MVB_PORT_OUTPUT;
    r.buf.handle = r.buf.addr = handle_of(out.virt);
    r.buf.size = out.size;
    r.buf.format = MVB_FORMAT_HEVC;
    if (mvb(MVB_REGISTER_BUFFER, &r, sizeof(r), 0) < 0)
      goto close_mv;
  }

  /* The vendor library's order and choice of command */
  {
    const uint32_t rate[] = {MVB_FW_BUFFER_PARAM, MVB_PARAM_FRAME_RATE,
                             (uint32_t)fps << 16};
    const uint32_t pframes[] = {MVB_FW_SET_OPTION, MVB_OPT_P_FRAMES,
                                (uint32_t)gop - 1};
    const uint32_t rc[] = {MVB_FW_BUFFER_PARAM, MVB_PARAM_RATE_CONTROL,
                           MVB_RC_VARIABLE, (uint32_t)(kbps * 1000)};
    const uint32_t qp[] = {MVB_FW_BUFFER_PARAM, MVB_PARAM_QP_RANGE,
                           (uint32_t)qmin, (uint32_t)qmax};

    if (fw_msg(MVB_SET_CONFIG, sizeof(rate), 3, rate) ||
        fw_msg(MVB_SET_PARAMETER, MVB_OPT_SIZE, 3, pframes) ||
        fw_msg(MVB_SET_CONFIG, sizeof(rc), 4, rc) ||
        fw_msg(MVB_SET_PARAMETER, sizeof(qp), 4, qp))
      goto close_mv;
  }

  {
    struct mvb_queue q = {0};
    uint32_t state = MVB_STATE_EXECUTING;

    fill_pattern(in.virt, pic, pic + w * h * 3 / 2, w, h, 0);
    q.handle = q.addr = handle_of(in.virt);
    q.filled = w * h * 3 / 2;
    q.in_flags = MVB_IN_FLAGS;
    if (mvb(MVB_EMPTY_THIS_BUFFER, &q, sizeof(q), 0) < 0)
      goto close_mv;
    memset(&q, 0, sizeof(q));
    q.handle = q.addr = handle_of(out.virt);
    if (mvb(MVB_FILL_THIS_BUFFER, &q, sizeof(q), 0) < 0 ||
        mvb(MVB_STATE_CHANGE, &state, sizeof(state), 0) < 0)
      goto close_mv;
  }
  in_busy = 1;

  /* One input and one output buffer in flight: refill the picture when
   * the input comes back, and requeue the output once it is written */
  for (int idle = 0; done < frames;) {
    uint32_t timeout = 200;
    struct mvb_event ev;
    struct mvb_buf b;

    if ((n = mvb(MVB_GET_EVENT, &timeout, sizeof(timeout), 0)) < 0)
      goto close_mv;
    if (n < (int)sizeof(ev)) {
      if (++idle == 10) {
        fprintf(stderr, "no events for 2 s: %d of %d frames out\n", done,
                frames);
        goto close_mv;
      }
      continue;
    }
    memcpy(&ev, reply_data(), sizeof(ev));
    if (verbose)
      fprintf(stderr, "event %u, %u bytes\n", ev.code, ev.size);
    if (ev.code != MVB_EVENT_INPUT_DONE && ev.code != MVB_EVENT_OUTPUT_DONE) {
      if (++idle == 50) {
        fprintf(stderr, "no buffers back: %d of %d frames out\n", done, frames);
        goto close_mv;
      }
      continue;
    }
    idle = 0;
    if (ev.size < sizeof(b)) {
      fprintf(stderr, "event %u: short buffer, %u bytes\n", ev.code, ev.size);
      goto close_mv;
    }
    memcpy(&b, reply_data() + sizeof(ev), sizeof(b));

    if (ev.code == MVB_EVENT_INPUT_DONE) {
      in_busy = 0;
    } else {
      if (b.offset > out.size || b.filled > out.size - b.offset) {
        fprintf(stderr, "output: %u bytes at %u in a %u-byte buffer\n",
                b.filled, b.offset, out.size);
        goto close_mv;
      }
      if (fwrite(out.virt + b.offset, 1, b.filled, f) != b.filled) {
        perror(out_path);
        goto close_mv;
      }
      bytes += b.filled;
      fprintf(stderr, "out %u bytes at %u, flags %#x%s\n", b.filled, b.offset,
              b.flags,
              b.flags & MVB_FLAG_CODEC_CONFIG ? " (parameter sets)"
              : b.flags & MVB_FLAG_SYNC       ? " (IDR)"
                                              : "");
      if (b.flags & MVB_FLAG_FRAME_END)
        done++;
      if (done < frames) {
        struct mvb_queue q = {0};

        q.handle = q.addr = handle_of(out.virt);
        if (mvb(MVB_FILL_THIS_BUFFER, &q, sizeof(q), 0) < 0)
          goto close_mv;
      }
    }
    if (!in_busy && sent < frames) {
      struct mvb_queue q = {0};

      fill_pattern(in.virt, pic, pic + w * h * 3 / 2, w, h, sent);
      q.handle = q.addr = handle_of(in.virt);
      q.filled = w * h * 3 / 2;
      q.in_flags = MVB_IN_FLAGS;
      if (mvb(MVB_EMPTY_THIS_BUFFER, &q, sizeof(q), 0) < 0)
        goto close_mv;
      in_busy = 1;
      sent++;
    }
  }
  fprintf(stderr, "%d frames, %d bytes to %s\n", done, bytes, out_path);
  ret = 0;

close_mv:
  close(fd_mv);
close_file:
  if (fclose(f) && !ret) {
    perror(out_path);
    ret = 1;
  }
close_enc:
  xmvenc_close(enc);
  free(pic);
  return ret;
}
