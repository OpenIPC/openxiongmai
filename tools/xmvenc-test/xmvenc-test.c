// SPDX-License-Identifier: MIT
/*
 * xmvenc-test: encode H.264 or JPEG on the XM530/XM550/XM510 Hantro 8290
 * (H1) encoder with libxmvenc, from a synthetic NV12 picture or a VI
 * channel, without vendor libraries.
 *
 * Needs the encoder driver (/dev/h264, vendor xm530_h264.ko/xm510_h264.ko
 * or hx280enc) and /dev/mmz. Stop majestic first: the encoder serves one
 * user at a time.
 *
 * Usage: xmvenc-test [-w width] [-h height] [-q qp] [-n frames]
 *                    [-g gop] [-c] [-v chn] [-d dump.nv12] [-j quality]
 *                    [-b kbps] [-f fps] [-Q qmin:qmax] [-o file]
 *                    [-p [-J quality] [-r frames] [-S ms] [-G ms]] [-B kib]
 *
 * -g sets the IDR interval (1: every frame is IDR), -c selects CABAC.
 * The colour bars move 8 pixels a frame, so P frames carry motion.
 * -v takes the pictures from a VI channel instead: frames are encoded
 * straight from the capture buffers, at the size VI delivers. VI keeps
 * capturing after majestic stops (channel 0 is the main stream).
 * -d saves the input picture of the first frame, to compare against.
 * -j encodes JPEG pictures at that quality (1-100) instead of H.264;
 * more than one frame gives concatenated JPEGs.
 * -b turns on frame-level rate control for that bitrate at -f frames per
 * second (default 25), with the QP kept within -Q (default 20:45); -q is
 * then the starting QP and the PPS pic_init_qp.
 * -p encodes the -v channel through a libxmvenc pipe (its own thread and
 * queue); -J adds JPEG snapshots of VI channel 1 at that quality, about
 * one a second, to <file>.jpg, -r requests an IDR once that many
 * packets have been read, and -S makes the reader that much slower per packet, so that
 * the queue fills, and -G sets the keyframe interval in milliseconds of
 * capture time instead of -g frames. -B sets the stream buffer size in KiB.
 */
#define _DEFAULT_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <math.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include "xmvenc.h"

static int fd_mem = -1;

/* EBU-style colour bars over a luma ramp, NV12 */
static void fill_pattern(uint8_t *y, uint8_t *uv, int w, int h, int stride,
                         int shift) {
  static const uint8_t bars[8][3] = {
      {235, 128, 128}, {210, 16, 146}, {170, 166, 16}, {145, 54, 34},
      {106, 202, 222}, {81, 90, 240},  {41, 240, 110}, {16, 128, 128},
  };

  for (int j = 0; j < h; j++)
    for (int i = 0; i < w; i++) {
      const uint8_t *c = bars[(i + shift) % w * 8 / w];
      y[j * stride + i] = j < h * 3 / 4 ? c[0] : 16 + 219 * i / w;
    }
  for (int j = 0; j < h / 2; j++)
    for (int i = 0; i < w / 2; i++) {
      const uint8_t *c = bars[(i * 2 + shift) % w * 8 / w];
      int grey = j >= h * 3 / 8;
      uv[j * stride + 2 * i] = grey ? 128 : c[1];
      uv[j * stride + 2 * i + 1] = grey ? 128 : c[2];
    }
}

/* VI frame record: 32 bytes on XM530 (xm530_vi.ko), 24 on XM510, which
 * has no timestamp. Getting a frame waits briefly for one; it stays ours,
 * and VI fills its other buffers, until it is released. */
struct vi_frame {
  uint32_t chn, pool, phys_y, phys_c, width, height;
  uint64_t pts;
};
/* Frame delivery per channel: majestic turns it off when it exits, while
 * the capture hardware itself keeps running. */
#define VI_IOC_ENABLE_CHN _IOW('V', 2, uint32_t)
#define VI_IOC_DISABLE_CHN _IOW('V', 3, uint32_t)
#define VI_CMD(dir, nr, size)                                                  \
  (((uint32_t)(dir) << 30) | ((size) << 16) | ('V' << 8) | (nr))
#define VI_IOC_GET_FRAME(size) VI_CMD(3, 4, size)
#define VI_IOC_RELEASE_FRAME(size) VI_CMD(1, 5, size)

/* VI state for the exit handler: whatever path main() leaves by, a frame
 * the tool still holds is given back and the channel is switched off
 * again, as majestic left it. */
static struct {
  int fd, held;
  uint32_t chn, rec_size; /* rec_size: 0 until a get has worked */
  struct vi_frame frame;
} vi = {-1, 0, 0, 0, {0}};

static void vi_release(void) {
  if (vi.held)
    ioctl(vi.fd, VI_IOC_RELEASE_FRAME(vi.rec_size), &vi.frame);
  vi.held = 0;
}

/* -J: VI channel 1, enabled as well */
static uint32_t vi_sub = 1;
static int vi_sub_enabled;

static void vi_cleanup(void) {
  if (vi.fd < 0)
    return;
  vi_release();
  ioctl(vi.fd, VI_IOC_DISABLE_CHN, &vi.chn);
  if (vi_sub_enabled)
    ioctl(vi.fd, VI_IOC_DISABLE_CHN, &vi_sub);
  close(vi.fd);
  vi.fd = -1;
}

/* Takes the next frame of the channel into vi.frame. The record size is
 * settled by the first get that returns a frame: the other size is
 * rejected, but so is a get when no frame is ready yet. */
static int vi_get(void) {
  static const uint32_t sizes[] = {32, 24};

  /* At 15 fps the next frame can be 66 ms away, and the driver waits only
   * briefly: retry for up to a second. */
  for (int tries = 0; tries < 100; tries++) {
    for (int i = 0; i < 2; i++) {
      uint32_t size = vi.rec_size ? vi.rec_size : sizes[i];

      memset(&vi.frame, 0, sizeof(vi.frame));
      vi.frame.chn = vi.chn;
      alarm(2);
      int ret = ioctl(vi.fd, VI_IOC_GET_FRAME(size), &vi.frame);
      alarm(0);
      if (!ret && vi.frame.phys_y && vi.frame.phys_c && vi.frame.width &&
          vi.frame.height) {
        vi.rec_size = size;
        vi.held = 1;
        return 0;
      }
      if (vi.rec_size)
        break;
    }
    usleep(10000);
  }
  fprintf(stderr, "VI channel %u: no frame\n", vi.chn);
  return -1;
}

/* Writes the NV12 picture at the given physical addresses, cropped to
 * width x height, to a file. */
static int dump_nv12(const char *path, uint32_t phys_y, uint32_t phys_c,
                     int width, int height, int stride) {
  FILE *f = fopen(path, "wb");
  int ok = f != NULL;

  for (int plane = 0; ok && plane < 2; plane++) {
    uint32_t phys = plane ? phys_c : phys_y, base = phys & ~0xfffu;
    size_t len = (size_t)stride * (plane ? height / 2 : height);
    uint8_t *p = mmap(0, len + (phys - base), PROT_READ, MAP_SHARED, fd_mem,
                      base);
    uint8_t *row = malloc(width);

    if (p == MAP_FAILED || !row) {
      ok = 0;
      break;
    }
    for (int j = 0; ok && j < (plane ? height / 2 : height); j++) {
      memcpy(row, p + (phys - base) + (size_t)j * stride, width);
      ok = fwrite(row, 1, width, f) == (size_t)width;
    }
    free(row);
    munmap(p, len + (phys - base));
  }
  if (!f || fclose(f) || !ok) {
    perror(path);
    return -1;
  }
  return 0;
}

/* A positive finite number, all of the argument; -1 otherwise */
static double parse_rate(const char *arg) {
  char *end;
  double v = strtod(arg, &end);

  return end != arg && *end == '\0' && isfinite(v) && v > 0 ? v : -1;
}

static void onalrm(int s) { (void)s; }

static void log_stderr(const char *msg) { fprintf(stderr, "%s\n", msg); }

static int read_delay_ms; /* -S */

static int write_packet(FILE *f, const struct xmvenc_packet *pkt) {
  return fwrite(pkt->hdr, 1, pkt->hdr_len, f) == pkt->hdr_len &&
                 fwrite(pkt->data, 1, pkt->len, f) == pkt->len
             ? 0
             : -1;
}

/*
 * -p: encode through a pipe, the library's own thread, while this thread
 * writes the packets out. -J adds a second pipe taking JPEG snapshots of
 * VI channel 1, about one a second, into <file>.jpg. -r requests an IDR
 * after that many packets.
 */
static int run_pipes(struct xmvenc *enc, const struct xmvenc_chn_attr *attr,
                     int vi_chn, int frames, int snap_q, int sub_w, int sub_h,
                     int idr_at, const char *out) {
  struct xmvenc_pipe *pm, *ps = NULL;
  struct xmvenc_pipe_stats st;
  struct xmvenc_packet pkt;
  char jpg[256];
  FILE *f, *fj = NULL;
  uint64_t last_pts = 0, bytes = 0;
  const uint8_t *first_data = NULL;
  int n, snaps = 0, started = 0, every = attr->fps >= 1 ? (int)attr->fps : 1;

  if (!(pm = xmvenc_pipe_create(enc, 0, vi_chn, attr)))
    return 1;
  if (snap_q) {
    struct xmvenc_chn_attr ja = {.codec = XMVENC_JPEG,
                                 .width = sub_w,
                                 .height = sub_h,
                                 .stride = sub_w,
                                 .quality = snap_q};
    snprintf(jpg, sizeof(jpg), "%s.jpg", out);
    if (!(ps = xmvenc_pipe_create(enc, 1, vi_sub, &ja)) ||
        !(fj = fopen(jpg, "wb")))
      return 1;
  }
  if (!(f = fopen(out, "wb")) || xmvenc_pipe_start(pm, frames))
    return 1;
  for (n = 0; n < frames; n++) {
    if (xmvenc_pipe_get(pm, &pkt, 2000)) {
      fprintf(stderr, "packet %d: none within 2 s\n", n);
      return 1;
    }
    if (!first_data)
      first_data = pkt.data;
    if (write_packet(f, &pkt)) {
      perror(out);
      return 1;
    }
    printf("packet %u (%c) qp %d: %u bytes at %+d, pts %llu (+%lld)\n",
           pkt.seq, pkt.type, pkt.qp, pkt.hdr_len + pkt.len,
           (int)(pkt.data - first_data),
           (unsigned long long)pkt.pts,
           last_pts ? (long long)(pkt.pts - last_pts) : 0LL);
    last_pts = pkt.pts;
    bytes += pkt.hdr_len + pkt.len;
    /* the next frame encoded after this request is an IDR; with a slow
     * reader (-S) that is a frame further on than packet n + 1 */
    if (n + 1 == idr_at) {
      printf("requesting an IDR\n");
      xmvenc_chn_request_idr(xmvenc_pipe_chn(pm));
    }
    xmvenc_pipe_release(pm, &pkt);
    if (read_delay_ms)
      usleep(read_delay_ms * 1000);
    if (ps && n % every == 0 && xmvenc_pipe_start(ps, 1) == 0)
      started++;
    /* the snapshots done so far, and at the end all of them */
    while (ps && snaps < started &&
           xmvenc_pipe_get(ps, &pkt, n + 1 < frames ? 0 : 2000) == 0) {
      if (write_packet(fj, &pkt)) {
        perror(jpg);
        return 1;
      }
      printf("snapshot %d: %u bytes\n", snaps++, pkt.hdr_len + pkt.len);
      xmvenc_pipe_release(ps, &pkt);
    }
  }
  xmvenc_pipe_stats(pm, &st);
  xmvenc_pipe_destroy(pm);
  printf("wrote %s: %d packets, %llu bytes; encoded %u, skipped %u, failed "
         "%u\n",
         out, n, (unsigned long long)bytes, st.encoded, st.skipped, st.failed);
  if (fclose(f))
    return 1;
  if (ps) {
    xmvenc_pipe_stats(ps, &st);
    xmvenc_pipe_destroy(ps);
    printf("wrote %s: %d snapshots; encoded %u, skipped %u, failed %u\n", jpg,
           snaps, st.encoded, st.skipped, st.failed);
    if (fclose(fj))
      return 1;
  }
  return 0;
}

int main(int argc, char **argv) {
  int width = 704, height = 576, qp = 26, frames = 1, gop = 30, cabac = 0,
      vi_chn = -1, jpeg_q = 0, qmin = 20, qmax = 45, pipe = 0, snap_q = 0,
      idr_at = 0, sub_w = 0, sub_h = 0, strm_kib = 0, gop_ms = 0, opt;
  double kbps = 0, fps = 25;
  const char *out = "xmvenc-test.h264", *dump = NULL;
  struct xmvenc *enc;
  struct xmvenc_chn *chn;
  struct xmvenc_buf in;

  while ((opt = getopt(argc, argv, "w:h:q:n:g:cv:d:j:b:f:Q:o:pJ:r:B:S:G:")) != -1) {
    switch (opt) {
    case 'w': width = atoi(optarg); break;
    case 'h': height = atoi(optarg); break;
    case 'q': qp = atoi(optarg); break;
    case 'n': frames = atoi(optarg); break;
    case 'g': gop = atoi(optarg); break;
    case 'c': cabac = 1; break;
    case 'v': vi_chn = atoi(optarg); break;
    case 'd': dump = optarg; break;
    case 'j': jpeg_q = atoi(optarg); break;
    case 'b':
      kbps = parse_rate(optarg);
      break;
    case 'f':
      fps = parse_rate(optarg);
      break;
    case 'Q':
      if (sscanf(optarg, "%d:%d", &qmin, &qmax) != 2)
        qmin = -1;
      break;
    case 'o': out = optarg; break;
    case 'p': pipe = 1; break;
    case 'J': snap_q = atoi(optarg); break;
    case 'r': idr_at = atoi(optarg); break;
    case 'B': strm_kib = atoi(optarg); break;
    case 'S': read_delay_ms = atoi(optarg); break;
    case 'G': {
      char *end;
      long v = strtol(optarg, &end, 10);

      gop_ms = end != optarg && *end == '\0' && v > 0 && v <= 3600000 ? v : -1;
      break;
    }
    default:
      fprintf(stderr,
              "usage: %s [-w width] [-h height] [-q qp] [-n frames] "
              "[-g gop] [-c] [-v chn] [-d dump.nv12] [-j quality] "
              "[-b kbps] [-f fps] [-Q qmin:qmax] [-o file] "
              "[-p [-J quality] [-r frames] [-S ms] [-G ms]] [-B kib]\n",
              argv[0]);
      return 2;
    }
  }
  if (frames < 1 || jpeg_q < 0 || jpeg_q > 100 || kbps < 0 || fps <= 0 ||
      qmin < 0 || qmax > 51 || qmin > qmax || snap_q < 0 || snap_q > 100 ||
      ((pipe || snap_q || idr_at) && (vi_chn < 0 || jpeg_q)) ||
      ((snap_q || idr_at || read_delay_ms || gop_ms) && !pipe) ||
      strm_kib < 0 || read_delay_ms < 0 || gop_ms < 0) {
    fprintf(stderr, "bad arguments\n");
    return 2;
  }
  /* An alarm bounds each wait: a signal interrupts it */
  struct sigaction sa;
  memset(&sa, 0, sizeof(sa));
  sa.sa_handler = onalrm;
  sigaction(SIGALRM, &sa, 0);

  if (vi_chn >= 0) {
    vi.chn = vi_chn;
    if ((vi.fd = open("/dev/vi", O_RDWR)) < 0) {
      perror("/dev/vi");
      return 1;
    }
    if (ioctl(vi.fd, VI_IOC_ENABLE_CHN, &vi.chn)) {
      perror("VI enable");
      return 1;
    }
    atexit(vi_cleanup);
    if (vi_get())
      return 1;
    vi_release();
    width = vi.frame.width;
    height = vi.frame.height;
    printf("VI channel %d: %dx%d\n", vi_chn, width, height);
    if (snap_q) {
      if (ioctl(vi.fd, VI_IOC_ENABLE_CHN, &vi_sub)) {
        perror("VI enable");
        return 1;
      }
      vi_sub_enabled = 1;
      vi.chn = vi_sub;
      if (vi_get())
        return 1;
      vi_release();
      sub_w = vi.frame.width;
      sub_h = vi.frame.height;
      vi.chn = vi_chn;
      printf("VI channel %u: %dx%d\n", vi_sub, sub_w, sub_h);
    }
  }

  if (!(enc = xmvenc_open(log_stderr)))
    return 1;
  if ((fd_mem = open("/dev/mem", O_RDWR | O_SYNC)) < 0) {
    perror("/dev/mem");
    return 1;
  }
  int mbw = (width + 15) / 16, mbh = (height + 15) / 16;
  /* VI buffers are packed: the stride is the width */
  int stride = vi_chn >= 0 ? width : mbw * 16;
  uint32_t luma = stride * mbh * 16, pic_size = luma * 3 / 2;
  /* MMZ has no free and blocks are found again by name, so size them for
   * at least the 5 MP sensor: later runs at any size up to that reuse
   * them. With majestic stopped this is 23 MB of the 26 MB left. */
  uint32_t luma_max =
      xmvenc_max_width(enc) < 2592 || luma > 2592 * 1952 ? luma : 2592 * 1952;
  struct xmvenc_chn_attr attr = {
      .codec = jpeg_q ? XMVENC_JPEG : XMVENC_H264,
      .width = width,
      .height = height,
      .stride = stride,
      .cabac = cabac,
      .gop = gop,
      .qp = qp,
      .kbps = jpeg_q ? 0 : kbps,
      .fps = fps,
      .qmin = qmin,
      .qmax = qmax,
      .quality = jpeg_q,
      .min_luma = luma_max,
      .strm_size = (uint32_t)strm_kib * 1024,
      .gop_us = (uint64_t)gop_ms * 1000,
  };
  if (pipe)
    return run_pipes(enc, &attr, vi_chn, frames, snap_q, sub_w, sub_h, idr_at,
                     out);
  if (vi_chn < 0 &&
      xmvenc_mmz_alloc(enc, &in, "xmvenc_in", luma_max * 3 / 2, NULL))
    return 1;
  if (!(chn = xmvenc_chn_create(enc, 0, &attr)))
    return 1;
  FILE *f = fopen(out, "wb");
  if (!f) {
    perror(out);
    return 1;
  }

  /* The MMZ mapping is uncached: build each picture in normal memory and
   * copy it over, byte stores there take seconds per frame. */
  uint8_t *pic = vi_chn < 0 ? malloc(pic_size) : NULL;
  if (vi_chn < 0 && !pic) {
    perror("malloc");
    return 1;
  }
  uint64_t total = 0, hdr_total = 0;
  for (int n = 0; n < frames; n++) {
    struct xmvenc_frame fr;
    uint32_t in_y, in_c;

    if (vi_chn >= 0) {
      if (vi_get())
        return 1;
      if ((int)vi.frame.width != width || (int)vi.frame.height != height) {
        fprintf(stderr, "VI size changed to %ux%u\n", vi.frame.width,
                vi.frame.height);
        return 1;
      }
      in_y = vi.frame.phys_y;
      in_c = vi.frame.phys_c;
    } else {
      memset(pic, 0, pic_size);
      fill_pattern(pic, pic + luma, width, height, stride, n * 8);
      memcpy(in.virt, pic, pic_size);
      in_y = in.phys;
      in_c = in.phys + luma;
    }

    if (n == 0 && dump && dump_nv12(dump, in_y, in_c, width, height, stride))
      return 1;

    alarm(2);
    int ret = xmvenc_encode(chn, in_y, in_c, &fr);
    alarm(0);
    if (ret)
      return 1;
    /* The frame is done: the input picture can go back to VI */
    vi_release();

    if (fwrite(fr.hdr, 1, fr.hdr_len, f) != fr.hdr_len ||
        fwrite(fr.data, 1, fr.len, f) != fr.len) {
      perror(out);
      return 1;
    }
    total += fr.len;
    hdr_total += fr.hdr_len;
    printf("frame %d (%c) qp %d: status 0x%08x, %u bytes, %ld us\n", n,
           fr.type, fr.qp, fr.status, fr.len, fr.us);
  }
  if (fclose(f)) {
    perror(out);
    return 1;
  }
  vi_cleanup();
  xmvenc_chn_destroy(chn);
  xmvenc_close(enc);
  if (jpeg_q)
    printf("wrote %s: %d JPEG pictures, %llu bytes (%dx%d, quality %d)\n", out,
           frames, (unsigned long long)(total + hdr_total), width, height,
           jpeg_q);
  else
    printf("wrote %s: %llu header + %llu slice bytes, %d frames (%dx%d, qp "
           "%d, gop %d, %s)\n",
           out, (unsigned long long)hdr_total, (unsigned long long)total,
           frames, width, height, qp, gop, cabac ? "CABAC" : "CAVLC");
  if (kbps > 0 && !jpeg_q)
    printf("rate control: target %.0f kbps at %.2f fps, got %.0f kbps\n", kbps,
           fps, total * 8.0 * fps / frames / 1000);
  return 0;
}
