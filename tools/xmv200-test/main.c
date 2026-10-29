// SPDX-License-Identifier: MIT
/* xmv200-test: bring-up steps of the V200 pipeline without vendor userspace */
#include <fcntl.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include "xmv200.h"

#ifdef REPLAY_DATA
int replay(int verbose);
#else
static int replay(int verbose) {
  fprintf(stderr, "built without REPLAY_DATA\n");
  return -1;
}
#endif

static int probe(void) {
  static const uint8_t addrs[] = {0x30, 0x32};

  for (int bus = 0; bus < 3; bus++)
    for (unsigned i = 0; i < sizeof(addrs); i++) {
      uint32_t hi, lo;

      if (xmv200_i2c_read(bus, addrs[i], 0x3107, 2, &hi, 1) ||
          xmv200_i2c_read(bus, addrs[i], 0x3108, 2, &lo, 1))
        continue;
      printf("i2c%d 0x%02x: id %02x%02x\n", bus, addrs[i], hi & 0xff,
             lo & 0xff);
    }
  return 0;
}

/* vpss.ko's 128-byte record: group, channel, then the command's fields */
struct vpss_rec {
  uint32_t grp, chn, timeout_ms, rsvd;
  uint32_t y, c, c2, fmt, stride, width, height, rsvd2;
  uint32_t pts_lo, pts_hi, vb, rsvd3[17];
};

#define VPSS_GET_CHN_FRAME 0x4080500e
#define VPSS_RELEASE_CHN_FRAME 0x4080500f
#define VPSS_SET_DEPTH 0x4080500d

static volatile int running = 1;

/* a 96x32 2-bit card at (16, 16) on each stream: red, green and white
 * bands, then a transparent quarter */
static void osd_card(int n) {
  static uint8_t bmp[32 * 96 / 4];
  static const uint8_t band[4] = {1, 2, 3, 0};
  struct xmv200_osd o = {
      .format = XMV200_OSD_2BIT,
      .x = 16,
      .y = 16,
      .width = 96,
      .height = 32,
      .color = {0, xmv200_osd_color(255, 0, 0, 255),
                xmv200_osd_color(0, 255, 0, 255),
                xmv200_osd_color(255, 255, 255, 255)},
  };

  for (int y = 0; y < 32; y++)
    for (int x = 0; x < 96; x++)
      bmp[y * 24 + x / 4] |= band[x / 24] << (6 - 2 * (x % 4));
  for (int s = 0; s < n; s++)
    if (xmv200_osd_create(s, &o) || xmv200_osd_bitmap(s, bmp, sizeof(bmp)) ||
        xmv200_osd_attach(s, s))
      fprintf(stderr, "osd %d: failed\n", s);
}

/* isp.ko's frame wait, as the vendor ISP thread runs it */
static void *isp_frames(void *p) {
  int fd = open("/dev/isp", O_RDWR);
  uint32_t arg[3];

  while (running && fd >= 0)
    if (ioctl(fd, 0x490a, arg) < 0)
      usleep(20000);
  return NULL;
}

static int grab(int grp, int chn, int count, const char *path) {
  int fd = open("/dev/vpss", O_RDWR), mem = open("/dev/mem", O_RDWR);
  FILE *out = path ? fopen(path, "wb") : NULL;

  if (fd < 0 || mem < 0)
    return -1;
  /* frames reach GetChnFrame only with a user queue on the channel */
  struct vpss_rec d = {.grp = grp, .chn = chn, .y = 2};
  ioctl(fd, VPSS_SET_DEPTH, &d);
  for (int i = 0; i < count; i++) {
    struct vpss_rec r = {.grp = grp, .chn = chn, .timeout_ms = 2000};
    int ret = ioctl(fd, VPSS_GET_CHN_FRAME, &r);

    if (ret < 0) {
      printf("frame %d: none (%d)\n", i, ret);
      continue;
    }
    printf("frame %d: %ux%u stride %u y %08x c %08x pts %u\n", i, r.width,
           r.height, r.stride, r.y, r.c, r.pts_lo);
    if (out && r.y && r.stride && r.height) {
      uint32_t len = r.stride * r.height * 3 / 2;
      uint32_t off = r.y & 0xfff;
      uint8_t *m = mmap(NULL, len + off, PROT_READ, MAP_SHARED, mem,
                        r.y & ~0xfffu);

      if (m != MAP_FAILED) {
        fwrite(m + off, 1, len, out);
        munmap(m, len + off);
      }
    }
    ioctl(fd, VPSS_RELEASE_CHN_FRAME, &r);
  }
  if (out)
    fclose(out);
  return 0;
}

int main(int argc, char **argv) {
  if (argc > 1 && !strcmp(argv[1], "probe"))
    return probe();
  if (argc > 1 && !strcmp(argv[1], "replay")) {
    pthread_t t;

    replay(argc > 2 && !strcmp(argv[2], "-v"));
    pthread_create(&t, NULL, isp_frames, NULL);
    grab(0, 0, 3, "/tmp/g0c0.yuv");
    grab(1, 0, 3, "/tmp/g1c0.yuv");
    grab(1, 1, 3, "/tmp/g1c1.yuv");
    running = 0;
    return 0;
  }
  if (argc > 1 && !strcmp(argv[1], "encode")) {
    /* the recorded board: main 2304x2592 (two sensors spliced) on venc
     * channel 0, sub 640x720 on channel 1 */
    static uint8_t buf[2 << 20];
    struct xmv200_venc *e[2];
    FILE *out[2] = {fopen("/tmp/main.h265", "wb"), fopen("/tmp/sub.h265", "wb")};
    int n = argc > 2 ? atoi(argv[2]) : 30, got[2] = {0, 0};
    pthread_t t;

    replay(0);
    pthread_create(&t, NULL, isp_frames, NULL);
    if (xmv200_venc_create(&e[0], 2304, 2592, 12, 35, 24) ||
        xmv200_venc_create(&e[1], 640, 720, 12, 35, 24)) {
      fprintf(stderr, "venc create failed\n");
      return 1;
    }
    for (int i = 0; i < 4 * n && (got[0] < n || got[1] < n); i++) {
      struct xmv200_frame f;
      uint32_t chn;
      int len;

      if (xmv200_frame_get(&f)) {
        printf("frame %d: none\n", i);
        continue;
      }
      chn = f.chn;
      len = chn < 2 ? xmv200_venc_encode(e[chn], &f, buf, sizeof(buf)) : -1;
      xmv200_frame_release(&f);
      if (len < 0) {
        printf("frame %d: chn %u len %d\n", i, chn, len);
        continue;
      }
      fwrite(buf, 1, len, out[chn]);
      if (got[chn]++ < 3 || i % 10 == 0)
        printf("chn %u frame %d: %d bytes\n", chn, got[chn], len);
    }
    fclose(out[0]);
    fclose(out[1]);
    running = 0;
    return 0;
  }
  if (argc > 2 && !strcmp(argv[1], "run")) {
    /* the board's pipeline from libxmv200 alone, its streams to files */
    static uint8_t buf[2 << 20];
    struct xmv200_board_info info;
    struct xmv200_venc *e[XMV200_MAX_STREAMS];
    FILE *out[XMV200_MAX_STREAMS];
    int n = argc > 3 ? atoi(argv[3]) : 30, got[XMV200_MAX_STREAMS] = {0};
    int done = 0;

    struct xmv200_layout l;
    int r;

    /* a board's name, or a layout file */
    if (strchr(argv[2], '/') || strstr(argv[2], ".conf")) {
      static char text[8192];
      FILE *lf = fopen(argv[2], "r");
      size_t len = lf ? fread(text, 1, sizeof(text) - 1, lf) : 0;

      if (lf)
        fclose(lf);
      text[len] = 0;
      r = !lf || xmv200_layout_parse(text, &l);
    } else {
      r = xmv200_board_layout(argv[2], &l);
    }
    if (r || xmv200_layout_start(&l, &info)) {
      fprintf(stderr, "board %s: start failed\n", argv[2]);
      return 1;
    }
    if (argc > 4 && !strcmp(argv[4], "osd"))
      osd_card(info.n_streams);
    for (int s = 0; s < info.n_streams; s++) {
      char path[32];

      snprintf(path, sizeof(path), "/tmp/stream%d.h265", s);
      out[s] = fopen(path, "wb");
      if (xmv200_venc_create(&e[s], info.stream[s].width,
                             info.stream[s].height, info.fps, 35,
                             2 * info.fps)) {
        fprintf(stderr, "venc %d: create failed\n", s);
        return 1;
      }
    }
    for (int i = 0; i < 4 * n * info.n_streams / 2 && done < info.n_streams;
         i++) {
      struct xmv200_frame f;
      int len;

      if (xmv200_frame_get(&f)) {
        printf("frame %d: none\n", i);
        continue;
      }
      xmv200_ae_frame(&f);
      len = (int)f.chn < info.n_streams
                ? xmv200_venc_encode(e[f.chn], &f, buf, sizeof(buf))
                : -1;
      xmv200_frame_release(&f);
      if (len < 0) {
        printf("frame %d: chn %u %ux%u not encoded\n", i, f.chn, f.width,
               f.height);
        continue;
      }
      fwrite(buf, 1, len, out[f.chn]);
      if (++got[f.chn] == n)
        done++;
      if (got[f.chn] <= 2 || got[f.chn] % 10 == 0)
        printf("stream %u frame %d: %d bytes\n", f.chn, got[f.chn], len);
    }
    for (int s = 0; s < info.n_streams; s++) {
      fclose(out[s]);
      printf("stream %d: %d frames\n", s, got[s]);
    }
    return 0;
  }
  if (argc > 2 && !strcmp(argv[1], "layout")) {
    /* a board's layout as text, to start a layout file from */
    struct xmv200_layout l;
    static char text[8192];

    if (xmv200_board_layout(argv[2], &l))
      return 1;
    xmv200_layout_format(&l, text, sizeof(text));
    fputs(text, stdout);
    return 0;
  }
  fprintf(stderr,
          "usage: %s probe | run <board|layout file> [frames [osd]] | layout <board>"
          " | replay | encode [n]\n",
          argv[0]);
  return 1;
}
