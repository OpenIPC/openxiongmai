// SPDX-License-Identifier: MIT
/*
 * Pictures from the vendor VI driver (/dev/vi), for pipes.
 *
 * The frame record is 32 bytes on XM530 (xm530_vi.ko), with a timestamp,
 * and 24 on XM510, without. A get waits briefly for a frame; the frame
 * stays ours, and VI fills its other buffers, until it is released. The
 * caller passes the size when it knows the chip; otherwise it is settled
 * by the first get that returns a frame: the other size is rejected, but
 * so is a get when no frame is ready yet, and XM530's driver logs
 * "vi_ioctl: cmd error" for every 24-byte try.
 */
#define _DEFAULT_SOURCE
#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <string.h>
#include <sys/ioctl.h>
#include <time.h>
#include <unistd.h>

#include "xmvenc_priv.h"

#define VI_CMD(dir, nr, size)                                                  \
  (((uint32_t)(dir) << 30) | ((size) << 16) | ('V' << 8) | (nr))
#define VI_IOC_GET_FRAME(size) VI_CMD(3, 4, size)
#define VI_IOC_RELEASE_FRAME(size) VI_CMD(1, 5, size)

struct vi_rec {
  uint32_t chn, pool, phys_y, phys_c, width, height;
  uint64_t pts;
};

int xmvenc__vi_open(struct xmvenc_vi *vi, int chn, uint32_t rec_size) {
  memset(vi, 0, sizeof(*vi));
  vi->chn = chn;
  vi->rec_size = rec_size;
  vi->fd = open("/dev/vi", O_RDWR);
  return vi->fd < 0 ? -1 : 0;
}

void xmvenc__vi_close(struct xmvenc_vi *vi) {
  xmvenc__vi_release(vi);
  if (vi->fd >= 0)
    close(vi->fd);
  vi->fd = -1;
}

static uint64_t now_us(void) {
  struct timespec ts;

  clock_gettime(CLOCK_MONOTONIC, &ts);
  return (uint64_t)ts.tv_sec * 1000000 + ts.tv_nsec / 1000;
}

int xmvenc__vi_get(struct xmvenc_vi *vi) {
  static const uint32_t sizes[] = {32, 24};

  for (int i = 0; i < 2; i++) {
    uint32_t size = vi->rec_size ? vi->rec_size : sizes[i];
    struct vi_rec r;

    memset(&r, 0, sizeof(r));
    r.chn = vi->chn;
    if (ioctl(vi->fd, VI_IOC_GET_FRAME(size), &r) == 0 && r.phys_y &&
        r.phys_c && r.width && r.height) {
      vi->rec_size = size;
      vi->held = 1;
      memcpy(vi->rec, &r, sizeof(r));
      vi->phys_y = r.phys_y;
      vi->phys_c = r.phys_c;
      vi->width = r.width;
      vi->height = r.height;
      /* XM530's timestamp counts microseconds but is a sign-extended
       * 32-bit value that wraps every 71 minutes: carry its steps on
       * from the clock at the first frame. XM510 has none: take the
       * time of the get. */
      uint64_t now = now_us();

      if (size == 32 && vi->pts) {
        vi->pts += (uint32_t)r.pts - vi->pts32;
        /* After a pause of more than one wrap (a snapshot pipe, say), the
         * steps are short by whole wraps: the clock says how many */
        int64_t lag = (int64_t)(now - vi->pts);
        if (lag > INT32_MAX || lag < INT32_MIN)
          vi->pts += (uint64_t)((lag + (lag > 0 ? 1 : -1) * (1LL << 31)) /
                                (1LL << 32))
                     << 32;
      } else {
        vi->pts = now;
      }
      vi->pts32 = (uint32_t)r.pts;
      return 0;
    }
    if (vi->rec_size)
      break;
  }
  return -1;
}

void xmvenc__vi_release(struct xmvenc_vi *vi) {
  if (vi->held)
    ioctl(vi->fd, VI_IOC_RELEASE_FRAME(vi->rec_size), vi->rec);
  vi->held = 0;
}
