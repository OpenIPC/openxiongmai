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
  fprintf(stderr, "usage: %s probe|replay\n", argv[0]);
  return 1;
}
