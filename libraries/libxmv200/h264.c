// SPDX-License-Identifier: MIT AND Apache-2.0
/*
 * H.264 on the XM533's and XM535s' Hantro H1 (hx280enc, /dev/h264), which
 * their vendor library uses only for JPEG: no vendor job to follow, so the
 * H1's H.264 registers and the per-QP tables below are the ones Rockchip's
 * MPP programs its VEPU1 (the same H1) with (mpp/hal/vpu/h264e:
 * hal_h264e_vepu1_v2.c, hal_h264e_vpu_tbl.c; Copyright 2015 Rockchip
 * Electronics Co. LTD, Apache License 2.0).
 *
 * Baseline profile, CAVLC, one slice a picture, one reference. The SPS and
 * PPS are ours; the H1 writes each slice's NAL (header and data) from its
 * registers, so the SPS is what it assumes: POC type 2, a 16-bit frame_num.
 * As for JPEG, what we write goes up to an 8-byte aligned output pointer and
 * the rest (up to 7 bytes) to +0x058/+0x05c, for the H1 to emit first. The
 * reconstruction of each picture is the next one's reference: two frame
 * buffers in turn.
 */
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include "xmv200.h"
#include "xmv200_priv.h"

#define H1_IOCGHWOFFSET _IOR('k', 3, unsigned long *)
#define H1_IOCGHWIOSIZE _IOR('k', 4, unsigned int *)
#define H1_IOCWAITDONE _IO('k', 12)

/* MPP's tables (hal_h264e_vpu_tbl.c), by QP */
static const uint32_t prev_mode_favor[52] = {
    7,  7,  8,  8,  9,  9,  10, 10, 11,  12,  12,  13,  14,  15,  16,  17, 18,
    19, 20, 21, 22, 24, 25, 27, 29, 30,  32,  34,  36,  38,  41,  43,  46, 49,
    51, 55, 58, 61, 65, 69, 73, 78, 82,  87,  93,  98,  104, 110, 117, 124,
    132, 140};
static const uint32_t diff_mv_penalty[52] = {
    2,   2,   3,   3,   3,   4,   4,   4,   5,   6,   6,   7,   8,
    9,   10,  11,  13,  14,  16,  18,  20,  23,  26,  29,  32,  36,
    40,  45,  51,  57,  64,  72,  81,  91,  102, 114, 128, 144, 161,
    181, 203, 228, 256, 287, 323, 362, 406, 456, 512, 575, 645, 724};
static const uint32_t diff_mv_penalty4p[52] = {
    2,   2,   2,   3,   3,   3,   4,   4,   5,   5,   6,   7,   8,
    9,   10,  11,  12,  14,  16,  17,  20,  22,  25,  28,  31,  35,
    39,  44,  49,  55,  62,  70,  78,  88,  98,  110, 124, 139, 156,
    175, 197, 221, 248, 278, 312, 351, 394, 442, 496, 557, 625, 701};
static const uint32_t intra16_favor[52] = {
    24,   24,   24,   26,   27,   30,   32,   35,   39,   43,   48,
    53,   58,   64,   71,   78,   85,   93,   102,  111,  121,  131,
    142,  154,  167,  180,  195,  211,  229,  248,  271,  296,  326,
    361,  404,  457,  523,  607,  714,  852,  1034, 1272, 1588, 2008,
    2568, 3318, 4323, 5672, 7486, 9928, 13216, 17648};
static const uint32_t inter_favor[52] = {
    0,   0,   0,   0,   0,   0,   45,   48,   51,   53,   55,   60,   62,
    67,  69,  72,  78,  84,  90,  96,   110,  120,  135,  152,  170,  189,
    210, 235, 265, 297, 335, 376, 420,  470,  522,  572,  620,  670,  724,
    770, 820, 867, 915, 970, 1020, 1076, 1132, 1180, 1230, 1275, 1320, 1370};
static const uint32_t skip_sad_penalty[52] = {
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 233, 205, 182,
    163, 146, 132, 120, 109, 100, 92,  84,  78,  71,  66,  61,  56,
    52,  48,  44,  41,  38,  35,  32,  30,  27,  25,  23,  21,  19,
    17,  15,  14,  12,  11,  9,   8,   7,   5,   4,   3,   2,   1};

/* MPP's exp_golomb_signed: the bits of a signed Exp-Golomb code */
static uint32_t eg_bits(int32_t v) {
  int32_t t = 0;

  v = v > 0 ? 2 * v : -2 * v + 1;
  while (v >> ++t)
    ;
  return t * 2 - 1;
}

struct h264enc {
  int fd;
  volatile uint32_t *hw;
  size_t hw_size;
  struct xmv200_buf out, rec[2];
  uint32_t w, h, mbw, mbh, fps;
  uint32_t frame_num, idr_id;
  int cur; /* the reconstruction buffer this picture writes */
};

/* ---- the parameter sets ---- */

struct bits {
  uint8_t *p;
  int cap, len;
  uint32_t acc;
  int n;
};

static void put(struct bits *b, uint32_t v, int n) {
  while (n--) {
    b->acc = b->acc << 1 | ((v >> n) & 1);
    if (++b->n == 8) {
      if (b->len < b->cap)
        b->p[b->len] = b->acc;
      b->len++;
      b->n = 0;
      b->acc = 0;
    }
  }
}

static void ue(struct bits *b, uint32_t v) {
  int n = 0;

  for (uint32_t t = v + 1; t > 1; t >>= 1)
    n++;
  put(b, 0, n);
  put(b, v + 1, n + 1);
}

static void se(struct bits *b, int32_t v) {
  ue(b, v > 0 ? 2 * v - 1 : -2 * v);
}

static void trailing(struct bits *b) {
  put(b, 1, 1);
  while (b->n)
    put(b, 0, 1);
}

/* a NAL: start code, header, the RBSP with emulation prevention */
static int nal(uint8_t *out, int cap, uint8_t hdr, const uint8_t *r, int n) {
  int o = 0, zeros = 0;

  if (cap < 5 + n * 3 / 2)
    return -1;
  out[o++] = 0;
  out[o++] = 0;
  out[o++] = 0;
  out[o++] = 1;
  out[o++] = hdr;
  for (int i = 0; i < n; i++) {
    if (zeros == 2 && r[i] <= 3) {
      out[o++] = 3;
      zeros = 0;
    }
    out[o++] = r[i];
    zeros = r[i] ? 0 : zeros + 1;
  }
  return o;
}

static int sps_pps(const struct h264enc *e, uint8_t *out, int cap) {
  uint8_t r[64];
  struct bits b = {r, sizeof(r)};
  uint32_t cw = e->mbw * 16 - e->w, ch = e->mbh * 16 - e->h;
  int n, m;

  put(&b, 66, 8);   /* Baseline */
  put(&b, 0xc0, 8); /* constraint_set0/1 */
  put(&b, e->mbw * e->mbh > 3600 ? 40 : e->mbw * e->mbh > 1620 ? 31 : 30, 8);
  ue(&b, 0);  /* seq_parameter_set_id */
  ue(&b, 12); /* log2_max_frame_num - 4: the H1's 16 bits */
  ue(&b, 2);  /* pic_order_cnt_type */
  ue(&b, 1);  /* max_num_ref_frames */
  put(&b, 0, 1);
  ue(&b, e->mbw - 1);
  ue(&b, e->mbh - 1);
  put(&b, 1, 1); /* frame_mbs_only */
  put(&b, 1, 1); /* direct_8x8_inference */
  put(&b, cw || ch, 1);
  if (cw || ch) {
    ue(&b, 0);
    ue(&b, cw / 2);
    ue(&b, 0);
    ue(&b, ch / 2);
  }
  put(&b, 1, 1); /* VUI: timing only */
  put(&b, 0, 4); /* aspect, overscan, video signal, chroma location */
  put(&b, 1, 1);
  put(&b, 1000, 32);
  put(&b, e->fps * 2000, 32);
  put(&b, 1, 1); /* fixed_frame_rate */
  put(&b, 0, 5); /* HRDs, pic_struct, bitstream restriction */
  trailing(&b);
  if ((n = nal(out, cap, 0x67, r, b.len)) < 0)
    return -1;

  struct bits p = {r, sizeof(r)};

  ue(&p, 0);     /* pic_parameter_set_id */
  ue(&p, 0);     /* seq_parameter_set_id */
  put(&p, 0, 2); /* CAVLC, no bottom field POC */
  ue(&p, 0);     /* one slice group */
  ue(&p, 0);     /* num_ref_idx_l0_default_active - 1 */
  ue(&p, 0);
  put(&p, 0, 3); /* no weighted prediction */
  se(&p, 0);     /* pic_init_qp - 26 */
  se(&p, 0);     /* pic_init_qs - 26 */
  se(&p, 0);     /* chroma_qp_index_offset */
  put(&p, 1, 1); /* deblocking_filter_control_present */
  put(&p, 0, 2); /* constrained_intra_pred, redundant_pic_cnt_present */
  trailing(&p);
  if ((m = nal(out + n, cap - n, 0x68, r, p.len)) < 0)
    return -1;
  return n + m;
}

/* ---- the H1 ---- */

void xmv200_h264_destroy(struct h264enc *e) {
  if (!e)
    return;
  if (e->hw)
    munmap((void *)e->hw, e->hw_size);
  if (e->fd >= 0)
    close(e->fd);
  free(e);
}

struct h264enc *xmv200_h264_create(uint32_t w, uint32_t h, uint32_t fps) {
  struct h264enc *e = calloc(1, sizeof(*e));
  unsigned long base = 0;
  unsigned int size = 0;
  uint32_t frame;
  int mem;

  if (!e)
    return NULL;
  e->w = w;
  e->h = h;
  e->mbw = (w + 15) / 16;
  e->mbh = (h + 15) / 16;
  e->fps = fps ? fps : 25;
  frame = e->mbw * 16 * e->mbh * 16;
  e->fd = open("/dev/h264", O_RDWR);
  if (e->fd < 0 || ioctl(e->fd, H1_IOCGHWOFFSET, &base) ||
      ioctl(e->fd, H1_IOCGHWIOSIZE, &size) || !base || !size ||
      (mem = open("/dev/mem", O_RDWR | O_SYNC)) < 0)
    goto fail;
  e->hw = mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, mem, base);
  close(mem);
  if (e->hw == MAP_FAILED) {
    e->hw = NULL;
    goto fail;
  }
  e->hw_size = size;
  if (xmv200_mmz_alloc(&e->out, "h264_stream", frame + 0x1000, 0) ||
      xmv200_mmz_alloc(&e->rec[0], "h264_rec0", frame * 3 / 2, 0) ||
      xmv200_mmz_alloc(&e->rec[1], "h264_rec1", frame * 3 / 2, 0))
    goto fail;
  return e;
fail:
  xmv200_h264_destroy(e);
  return NULL;
}

void xmv200_h264_set_fps(struct h264enc *e, uint32_t fps) {
  if (fps)
    e->fps = fps;
}

int xmv200_h264_encode(struct h264enc *e, const struct xmv200_frame *f,
                       int idr, int qp, uint8_t *out, int cap) {
  volatile uint32_t *r = e->hw;
  uint32_t mbw = e->mbw, mbh = e->mbh, ysize = mbw * 16 * mbh * 16;
  uint32_t hdr = 0, aligned, nrem, len, st, v;
  uint32_t scaler = 200 / (mbw + mbh) ? 200 / (mbw + mbh) : 1;
  uint64_t rem = 0;
  int n;

  if (f->width != e->w || f->height != e->h)
    return -1;
  qp = qp < 0 ? 0 : qp > 51 ? 51 : qp;
  if (idr) {
    e->frame_num = 0;
    e->idr_id = (e->idr_id + 1) & 0xf;
    if ((n = sps_pps(e, e->out.virt, 0x200)) < 0)
      return -1;
    hdr = n;
  }
  aligned = hdr & ~7u;
  nrem = hdr - aligned;
  for (uint32_t i = 0; i < nrem; i++)
    rem |= (uint64_t)e->out.virt[aligned + i] << (56 - 8 * i);

  const struct xmv200_buf *rec = &e->rec[e->cur], *ref = &e->rec[!e->cur];

  r[0x004 / 4] = 0;
  r[0x008 / 4] = 0xd01f; /* the bus, as the vendor's JPEG */
  r[0x014 / 4] = e->out.phys + aligned;
  r[0x018 / 4] = 0; /* no NAL size table */
  r[0x01c / 4] = ref->phys;
  r[0x020 / 4] = ref->phys + ysize;
  r[0x024 / 4] = rec->phys;
  r[0x028 / 4] = rec->phys + ysize;
  r[0x02c / 4] = f->y;
  r[0x030 / 4] = f->c;
  r[0x034 / 4] = 0;
  r[0x03c / 4] = f->stride << 12 |
                 (e->w & 15 ? (16 - (e->w & 15)) / 4 : 0) << 10 |
                 (e->h & 15 ? 16 - (e->h & 15) : 0) << 6 | 1 << 2; /* NV12 */
  r[0x040 / 4] = 26u << 26 | e->idr_id << 1; /* PPS QP, deblocking 0/0 */
  r[0x044 / 4] = prev_mode_favor[qp] << 16 | e->frame_num;
  v = intra16_favor[qp];
  if (mbw * mbh > 3600)
    v |= 1u << 22; /* no quarter-pixel MVs */
  r[0x048 / 4] = v;
  r[0x04c / 4] = 1u << 30 | diff_mv_penalty[qp] << 20 |
                 diff_mv_penalty4p[qp] << 10 | diff_mv_penalty[qp];
  v = skip_sad_penalty[qp] * scaler;
  r[0x054 / 4] = (v > 255 ? 255 : v) << 24 | inter_favor[qp];
  r[0x058 / 4] = rem >> 32;
  r[0x05c / 4] = (uint32_t)rem;
  r[0x060 / 4] = (e->out.size - aligned) >> 3 & ~7u;
  r[0x064 / 4] = 0; /* no MAD QP adjustment */
  r[0x06c / 4] = (uint32_t)qp << 26 | (uint32_t)qp << 20 | (uint32_t)qp << 14;
  for (int i = 0; i < 10; i++)
    r[0x070 / 4 + i] = 0; /* no checkpoints */
  r[0x090 / 4] = 0;
  r[0x094 / 4] = nrem * 8 << 23;
  r[0x09c / 4] = 0;
  r[0x0a0 / 4] = 0;
  r[0x0cc / 4] = 0; /* CAVLC: no CABAC table */
  r[0x0d0 / 4] = 0;
  r[0x0e0 / 4] = mbw << 24 | mbw << 16 | mbh << 8 | mbh; /* no intra area */
  r[0x0f0 / 4] = mbw << 24 | mbw << 16 | mbh << 8 | mbh; /* no ROI */
  r[0x0f4 / 4] = mbw << 24 | mbw << 16 | mbh << 8 | mbh;
  r[0x0f8 / 4] = 10u << 28; /* zero MV favour */
  for (int i = 0; i < 128; i += 4) {
    r[0x180 / 4 + i / 4] = (uint32_t)i << 24 | (uint32_t)(i + 1) << 16 |
                           (uint32_t)(i + 2) << 8 | (uint32_t)(i + 3);
    uint32_t q[4];

    for (int k = 0; k < 4; k++)
      q[k] = eg_bits(i + k) > 255 ? 255 : eg_bits(i + k);
    r[0x200 / 4 + i / 4] = q[0] << 24 | q[1] << 16 | q[2] << 8 | q[3];
  }
  r[0x038 / 4] = 1u << 31 | mbw << 19 | mbh << 10 | (idr ? 1 : 0) << 3 |
                 3 << 1 | 1; /* H.264, start */
  if (ioctl(e->fd, H1_IOCWAITDONE, 0) < 0)
    return -1;
  st = r[0x004 / 4];
  len = r[0x060 / 4] / 8;
  r[0x004 / 4] = 0;
  if (!(st & 1 << 2) || !len || aligned + len > e->out.size ||
      (int)(aligned + len) > cap) {
    fprintf(stderr, "libxmv200: h264 status %#x\n", st);
    return -1;
  }
  memcpy(out, e->out.virt, aligned + len);
  e->frame_num = (e->frame_num + 1) & 0xffff;
  e->cur = !e->cur;
  return aligned + len;
}
