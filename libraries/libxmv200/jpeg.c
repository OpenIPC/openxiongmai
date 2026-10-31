// SPDX-License-Identifier: MIT
/*
 * JPEG snapshots on the XM530V200's encoder (jpeg.ko, /dev/jpeg, registers
 * at 0x21100000), as the vendor library in Sofia drives it (its job read
 * from qemu-bench's register log of a stock snapshot).
 *
 * A job is register writes through jpeg.ko's table (136 entries {offset,
 * value, changed}, written in order with entry 0, the start, last; an
 * offset may come more than once, so a call carries 135 writes):
 *   +0x060 / +0x064  the luma / chroma quantisers, a write each, natural
 *                    order: 1 << 24 | (0x40 + i) << 16 | 65536 / q
 *   +0x068 / +0x06c  the AC Huffman codes, 162 each (Annex K's, the
 *                    vendor's words as they are)
 *   +0x008 5 (NV12), +0x00c height << 16 | width, +0x014 the stride,
 *   +0x088 where the scan goes, +0x08c the buffer's end, +0x090 / +0x094
 *   the source's luma / chroma; the rest the vendor's constants
 *   +0x000 1: start; +0x044 the scan's length
 * The engine writes only the entropy-coded scan; the header (SOI, JFIF,
 * DQT, SOF0, DHT, SOS) and the EOI are ours.
 *
 * The XM533 and XM535s have no such engine: their snapshots are the Hantro
 * H1's (hx280enc, /dev/h264, its registers mapped), as the vendor library
 * drives it there (its job from qemu-bench's dump of a stock XM535D1
 * snapshot): the header ours up to an 8-byte aligned output pointer, the
 * rest of it (up to 7 bytes) in +0x058/+0x05c for the H1 to emit first,
 * their bits in +0x094[28:23]; the quantisers four to a word, big endian,
 * in the H1's order, at +0x100 (luma) and +0x140 (chroma); +0x03c the row
 * length << 12 | NV12 << 2; +0x038 the picture in macroblocks, JPEG mode,
 * intra, start; +0x060 the room in 8 bytes, done, the bits written. The
 * H1 ends with EOI.
 */
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include "xmv200.h"

#define JPEG_WAIT 0x80044a02
#define JPEG_SET_REGS 0x400c4a03
#define JPEG_TAKE 0x4a04
#define JPEG_GIVE 0x4a05
#define JPEG_RESET 0x4a06
#define N_REG 136
#define SCAN_ROOM 0x400 /* the scan's place in the buffer */

/* hx280enc: the registers' place and size; they are mapped through
 * /dev/mem, as Hantro's EWL does */
#define H1_IOCGHWOFFSET _IOR('k', 3, unsigned long *)
#define H1_IOCGHWIOSIZE _IOR('k', 4, unsigned int *)
#define H1_IOCWAITDONE _IO('k', 12)

struct jreg {
  uint32_t off, val, changed;
};

struct xmv200_jpeg {
  int fd;
  int h1;                /* the Hantro H1's, not jpeg.ko's */
  volatile uint32_t *hw; /* its registers */
  size_t hw_size;
  struct xmv200_buf buf;
  struct jreg t[N_REG];
  int n; /* writes queued, from entry 1 */
};

#include "jpeg_tables.h"

/* the H1's quantiser order: word k holds Q[h1_order[4k..4k+3]] */
static const uint8_t h1_order[64] = {
    0,  8,  16, 24, 1,  9,  17, 25, 32, 40, 48, 56, 33, 41, 49, 57,
    2,  10, 18, 26, 3,  11, 19, 27, 34, 42, 50, 58, 35, 43, 51, 59,
    4,  12, 20, 28, 5,  13, 21, 29, 36, 44, 52, 60, 37, 45, 53, 61,
    6,  14, 22, 30, 7,  15, 23, 31, 38, 46, 54, 62, 39, 47, 55, 63};

/* Annex K's tables at quality 50, natural order */
static const uint8_t std_q[2][64] = {
    {16, 11, 10, 16, 24,  40,  51,  61,  12, 12, 14, 19, 26,  58,  60,  55,
     14, 13, 16, 24, 40,  57,  69,  56,  14, 17, 22, 29, 51,  87,  80,  62,
     18, 22, 37, 56, 68,  109, 103, 77,  24, 35, 55, 64, 81,  104, 113, 92,
     49, 64, 78, 87, 103, 121, 120, 101, 72, 92, 95, 98, 112, 100, 103, 99},
    {17, 18, 24, 47, 99, 99, 99, 99, 18, 21, 26, 66, 99, 99, 99, 99,
     24, 26, 56, 99, 99, 99, 99, 99, 47, 66, 99, 99, 99, 99, 99, 99,
     99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99,
     99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99}};

static const uint8_t zigzag[64] = {
    0,  1,  8,  16, 9,  2,  3,  10, 17, 24, 32, 25, 18, 11, 4,  5,
    12, 19, 26, 33, 40, 48, 41, 34, 27, 20, 13, 6,  7,  14, 21, 28,
    35, 42, 49, 56, 57, 50, 43, 36, 29, 22, 15, 23, 30, 37, 44, 51,
    58, 59, 52, 45, 38, 31, 39, 46, 53, 60, 61, 54, 47, 55, 62, 63};

int xmv200_jpeg_create(struct xmv200_jpeg **pj, uint32_t max_width,
                       uint32_t max_height) {
  struct xmv200_jpeg *j = calloc(1, sizeof(*j));

  if (!j)
    return -1;
  j->fd = open("/dev/jpeg", O_RDWR);
  if (j->fd < 0) {
    unsigned long base = 0;
    unsigned int size = 0;
    int mem;

    j->fd = open("/dev/h264", O_RDWR);
    if (j->fd >= 0 && !ioctl(j->fd, H1_IOCGHWOFFSET, &base) &&
        !ioctl(j->fd, H1_IOCGHWIOSIZE, &size) && base && size &&
        (mem = open("/dev/mem", O_RDWR | O_SYNC)) >= 0) {
      j->hw = mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, mem, base);
      close(mem);
      if (j->hw == MAP_FAILED)
        j->hw = NULL;
      j->hw_size = size;
      j->h1 = j->hw != NULL;
    }
    if (!j->h1) {
      xmv200_jpeg_destroy(j);
      return -1;
    }
  }
  if (
      xmv200_mmz_alloc(&j->buf, "jpeg_stream",
                       SCAN_ROOM + max_width * max_height + 0x10000, 0)) {
    xmv200_jpeg_destroy(j);
    return -1;
  }
  if (!j->h1)
    ioctl(j->fd, JPEG_RESET, 0);
  *pj = j;
  return 0;
}

void xmv200_jpeg_destroy(struct xmv200_jpeg *j) {
  if (!j)
    return;
  if (j->hw)
    munmap((void *)j->hw, j->hw_size);
  if (j->fd >= 0)
    close(j->fd);
  xmv200_mmz_free(&j->buf);
  free(j);
}

/* queued writes out through the table, entry 0 a no-op (or the start) */
static int flush(struct xmv200_jpeg *j, int start) {
  j->t[0] = (struct jreg){0x000, start, 1};
  if (ioctl(j->fd, JPEG_SET_REGS, j->t) < 0)
    return -1;
  memset(j->t, 0, sizeof(j->t));
  j->n = 0;
  return 0;
}

static int w(struct xmv200_jpeg *j, uint32_t off, uint32_t val) {
  if (j->n == N_REG - 1 && flush(j, 0))
    return -1;
  j->t[++j->n] = (struct jreg){off, val, 1};
  return 0;
}

static uint8_t *put16(uint8_t *p, uint32_t v) {
  *p++ = v >> 8;
  *p++ = v & 0xff;
  return p;
}

/* SOI, JFIF, DQT, SOF0, DHT, SOS */
static int header(uint8_t *out, int cap, uint8_t q[2][64], uint32_t w_,
                  uint32_t h) {
  static const uint8_t jfif[] = {0xff, 0xd8, 0xff, 0xe0, 0x00, 0x10, 'J',
                                 'F',  'I',  'F',  0x00, 0x01, 0x01, 0x00,
                                 0x00, 0x01, 0x00, 0x01, 0x00, 0x00};
  uint8_t *p = out;

  if (cap < 20 + 134 + 19 + (int)sizeof(dht) + 14)
    return -1;
  memcpy(p, jfif, sizeof(jfif));
  p += sizeof(jfif);
  p = put16(put16(p, 0xffdb), 2 + 2 * 65);
  for (int t = 0; t < 2; t++) {
    *p++ = t;
    for (int i = 0; i < 64; i++)
      *p++ = q[t][zigzag[i]];
  }
  p = put16(put16(p, 0xffc0), 17);
  *p++ = 8;
  p = put16(put16(p, h), w_);
  *p++ = 3;
  static const uint8_t comps[] = {1, 0x22, 0, 2, 0x11, 1, 3, 0x11, 1};
  memcpy(p, comps, sizeof(comps));
  p += sizeof(comps);
  memcpy(p, dht, sizeof(dht));
  p += sizeof(dht);
  static const uint8_t sos[] = {0xff, 0xda, 0x00, 0x0c, 3, 1, 0x00, 2,
                                0x11, 3,    0x11, 0,    0x3f, 0};
  memcpy(p, sos, sizeof(sos));
  p += sizeof(sos);
  return p - out;
}

/* the H1's job: the header n bytes in out, the scan after it */
static int h1_encode(struct xmv200_jpeg *j, const struct xmv200_frame *f,
                     uint8_t q[2][64], uint8_t *out, int n, int cap) {
  volatile uint32_t *r = j->hw;
  uint32_t aligned = n & ~7u, nrem = n - aligned, mbw = (f->width + 15) / 16,
           mbh = (f->height + 15) / 16, len, st;
  uint64_t rem = 0;

  if (aligned + 0x10 > SCAN_ROOM)
    return -1;
  memcpy(j->buf.virt, out, aligned);
  for (uint32_t i = 0; i < nrem; i++)
    rem |= (uint64_t)out[aligned + i] << (56 - 8 * i);
  r[0x004 / 4] = 0;
  r[0x008 / 4] = 0xd01f; /* the bus: the vendor's */
  r[0x014 / 4] = j->buf.phys + aligned;
  r[0x02c / 4] = f->y;
  r[0x030 / 4] = f->c;
  r[0x034 / 4] = 0;
  r[0x03c / 4] = f->stride << 12 | 1 << 2; /* NV12 */
  r[0x058 / 4] = rem >> 32;
  r[0x05c / 4] = (uint32_t)rem;
  r[0x060 / 4] = (j->buf.size - aligned) >> 3 & ~7u; /* in 8 bytes */
  r[0x094 / 4] = nrem * 8 << 23;
  for (int t = 0; t < 2; t++)
    for (int k = 0; k < 16; k++)
      r[(0x100 + t * 0x40) / 4 + k] =
          (uint32_t)q[t][h1_order[4 * k]] << 24 |
          q[t][h1_order[4 * k + 1]] << 16 | q[t][h1_order[4 * k + 2]] << 8 |
          q[t][h1_order[4 * k + 3]];
  r[0x038 / 4] = 1u << 31 | mbw << 19 | mbh << 10 | 2 << 1 | 1 << 3 | 1;
  if (ioctl(j->fd, H1_IOCWAITDONE, 0) < 0)
    return -1;
  st = r[0x004 / 4];
  len = r[0x060 / 4] / 8;
  r[0x004 / 4] = 0;
  if (!(st & 1 << 2) || !len || aligned + len > j->buf.size ||
      (int)(aligned + len) + 2 > cap) {
    fprintf(stderr, "libxmv200: jpeg (H1) status %#x\n", st);
    return -1;
  }
  memcpy(out + aligned, j->buf.virt + aligned, len);
  n = aligned + len;
  if (out[n - 2] != 0xff || out[n - 1] != 0xd9) {
    out[n++] = 0xff;
    out[n++] = 0xd9;
  }
  return n;
}

int xmv200_jpeg_encode(struct xmv200_jpeg *j, const struct xmv200_frame *f,
                       int quality, uint8_t *out, int cap) {
  uint8_t q[2][64];
  uint32_t status = 0, len, scale;
  int n;

  if (quality < 1)
    quality = 1;
  if (quality > 100)
    quality = 100;
  scale = quality < 50 ? 5000 / quality : 200 - 2 * quality;
  for (int t = 0; t < 2; t++)
    for (int i = 0; i < 64; i++) {
      uint32_t v = (std_q[t][i] * scale + 50) / 100;

      q[t][i] = v < 1 ? 1 : v > 255 ? 255 : v;
    }
  if (f->width * f->height + SCAN_ROOM > j->buf.size ||
      (n = header(out, cap, q, f->width, f->height)) < 0)
    return -1;
  if (j->h1)
    return h1_encode(j, f, q, out, n, cap);

  if (ioctl(j->fd, JPEG_TAKE, 0) < 0)
    return -1;
  for (int i = 0; i < 64; i++) {
    w(j, 0x060, 1u << 24 | (0x40u + i) << 16 | 65536 / q[0][i]);
    w(j, 0x064, 1u << 24 | (0x40u + i) << 16 | 65536 / q[1][i]);
  }
  for (int i = 0; i < 162; i++) {
    w(j, 0x068, huff_luma[i]);
    w(j, 0x06c, huff_chroma[i]);
  }
  w(j, 0x004, 0);
  w(j, 0x008, 5);
  w(j, 0x00c, f->height << 16 | f->width);
  w(j, 0x010, 0);
  w(j, 0x014, f->stride);
  w(j, 0x018, 0x02710000);
  w(j, 0x024, 0);
  w(j, 0x028, 0);
  w(j, 0x030, 3);
  w(j, 0x034, 0x3f);
  w(j, 0x074, 0);
  w(j, 0x088, j->buf.phys + SCAN_ROOM);
  w(j, 0x08c, j->buf.phys + j->buf.size);
  w(j, 0x090, f->y);
  w(j, 0x094, f->c);
  w(j, 0x098, 0);
  w(j, 0x09c, 0);
  w(j, 0x0a0, 0);
  if (flush(j, 1) || ioctl(j->fd, JPEG_WAIT, &status) < 0 || !(status & 1)) {
    fprintf(stderr, "libxmv200: jpeg status %#x\n", status);
    ioctl(j->fd, JPEG_RESET, 0);
    ioctl(j->fd, JPEG_GIVE, 0);
    return -1;
  }
  len = 0;
  xmv200_reg_read(0x21100044, &len); /* the scan's length */
  ioctl(j->fd, JPEG_GIVE, 0);
  if (!len || len > j->buf.size - SCAN_ROOM || n + (int)len + 2 > cap)
    return -1;
  memcpy(out + n, j->buf.virt + SCAN_ROOM, len);
  n += len;
  out[n++] = 0xff;
  out[n++] = 0xd9;
  return n;
}
