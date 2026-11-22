// SPDX-License-Identifier: MIT AND Apache-2.0
/*
 * H.264 on the XM533's and XM535s' Hantro H1 (hx280enc, /dev/h264), which
 * their vendor library uses only for JPEG. The H1 is the 8290 the XM530 and
 * XM550 carry (the same 0x1c4-byte window in their hx280enc), so it is
 * programmed as libxmvenc programs that one: Rockchip VEPU1's layout (MPP,
 * Apache License 2.0) with the 8290's differences measured on an XM550
 * (OpenIPC/openxiongmai#13): frame_num in 0x048[15:0], 0x044 a constant
 * per frame type, the vendor's constants in 0x04c/0x054/0x0f8 and
 * 0x180..0x1c0, no tables from 0x200.
 *
 * Baseline (CAVLC) or Main (CABAC, cabac_init_idc 0, its context tables
 * for every QP in MMZ at 0x0cc), one slice a picture, one reference. The
 * SPS and PPS are ours; the H1 writes each slice's NAL from its registers,
 * so the SPS is what it assumes: POC type 2, a 16-bit frame_num. As for
 * JPEG, what we write goes up to an 8-byte aligned output pointer and the
 * rest (up to 7 bytes) to +0x058/+0x05c, for the H1 to emit first. The
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

/* the XM530/XM550 vendor's values (libxmvenc's chip_xm530), [0] IDR, [1] P */
static const uint32_t ctrl1[2] = {0x006a00eb, 0x0081011d};
static const uint32_t ctrl3[2] = {0x00009c28, 0x0000c433};
static const uint32_t ctrl4[2] = {0x2d000087, 0x33000087};
static const uint32_t ext[6] = {0x00000003, 0x606a707a, 0x4148545a,
                                0x10112131, 0x11111100, 0x11111100};
static const uint32_t r1b8[2] = {0x000abcdd, 0x999acdee};
#define MVC 0x80002000
#define R1BC 0x64209bdf
#define R1C0 0x1b6db6db

/* libxmvenc's CABAC context initialisations (cabac_tbl.c) */
extern const signed char xmvenc__cabac_init_intra[460][2];
extern const signed char xmvenc__cabac_init[3][460][2];
#define CABAC_TBL_SIZE (52 * 2 * 464)

struct h264enc {
  int fd;
  volatile uint32_t *hw;
  size_t hw_size;
  struct xmv200_buf out, rec[2], cabac_tbl;
  int cabac, cabac_next; /* Main profile; from the next IDR */
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

  put(&b, e->cabac ? 77 : 66, 8);  /* Main or (Constrained) Baseline */
  put(&b, e->cabac ? 0 : 0xc0, 8); /* constraint_set0/1 */
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
  put(&p, e->cabac, 1); /* entropy_coding_mode */
  put(&p, 0, 1);        /* no bottom field POC */
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

/* the CABAC context states for every QP, intra then inter (cabac_init_idc
 * 0), 464 bytes each, in the H1's word order: libxmvenc's, which matches
 * the vendor stack's table */
static int cabac_table(uint8_t *dst) {
  uint8_t *t = calloc(1, CABAC_TBL_SIZE);

  if (!t)
    return -1;
  for (int qp = 0; qp < 52; qp++)
    for (int j = 0; j < 2; j++) {
      const signed char(*ctx)[2] =
          j ? xmvenc__cabac_init[0] : xmvenc__cabac_init_intra;

      for (int i = 0; i < 460; i++) {
        int pre = ((ctx[i][0] * qp) >> 4) + ctx[i][1];

        pre = pre < 1 ? 1 : pre > 126 ? 126 : pre;
        t[qp * 928 + j * 464 + i] =
            pre <= 63 ? (63 - pre) << 1 : ((pre - 64) << 1) | 1;
      }
    }
  /* each pair of 32-bit words swapped, and each byte-reversed */
  for (int k = 0; k < CABAC_TBL_SIZE; k += 8)
    for (int b = 0; b < 4; b++) {
      dst[k + b] = t[k + 7 - b];
      dst[k + 4 + b] = t[k + 3 - b];
    }
  free(t);
  return 0;
}

void xmv200_h264_set_cabac(struct h264enc *e, int on) { e->cabac_next = !!on; }

void xmv200_h264_destroy(struct h264enc *e) {
  if (!e)
    return;
  if (e->hw)
    munmap((void *)e->hw, e->hw_size);
  if (e->fd >= 0)
    close(e->fd);
  xmv200_mmz_free(&e->out);
  xmv200_mmz_free(&e->rec[0]);
  xmv200_mmz_free(&e->rec[1]);
  xmv200_mmz_free(&e->cabac_tbl);
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
  e->fd = -1; /* until it is opened: the failure path closes it */
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
  uint32_t hdr = 0, aligned, nrem, len, st;
  uint64_t rem = 0;
  int n;

  if (f->width != e->w || f->height != e->h)
    return -1;
  qp = qp < 0 ? 0 : qp > 51 ? 51 : qp;
  if (idr) {
    if (e->cabac_next != e->cabac) {
      if (e->cabac_next && !e->cabac_tbl.phys &&
          (xmv200_mmz_alloc(&e->cabac_tbl, "h264_cabac", CABAC_TBL_SIZE, 0) ||
           cabac_table(e->cabac_tbl.virt)))
        e->cabac_next = 0;
      e->cabac = e->cabac_next;
    }
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

  r[0x038 / 4] = 0;
  r[0x004 / 4] = 0;
  r[0x008 / 4] = 0xd01f; /* burst 16, clock gating, all swaps (vendor) */
  r[0x014 / 4] = e->out.phys + aligned;
  r[0x018 / 4] = 0; /* no NAL size table */
  r[0x01c / 4] = ref->phys;
  r[0x020 / 4] = ref->phys + ysize;
  r[0x024 / 4] = rec->phys;
  r[0x028 / 4] = rec->phys + ysize;
  r[0x02c / 4] = f->y;
  r[0x030 / 4] = f->c;
  r[0x034 / 4] = f->c + 1;
  r[0x03c / 4] = f->stride << 12 |
                 (e->w & 15 ? (16 - (e->w & 15)) / 4 : 0) << 10 |
                 (e->h & 15 ? 16 - (e->h & 15) : 0) << 6 | 1 << 2; /* NV12 */
  /* pic_init_qp (the PPS's 26), idr_pic_id (P pictures keep the IDR's) */
  r[0x040 / 4] = 26u << 26 | e->idr_id << 1;
  r[0x044 / 4] = ctrl1[!idr];
  /* deblocking as the vendor, quarter-pel MVs off, CABAC (init_idc 0) or
   * CAVLC, frame_num */
  r[0x048 / 4] = 0xc0400000 | (uint32_t)e->cabac << 18 | e->frame_num;
  r[0x04c / 4] = ctrl3[!idr];
  r[0x054 / 4] = ctrl4[!idr];
  r[0x058 / 4] = rem >> 32;
  r[0x05c / 4] = (uint32_t)rem;
  r[0x060 / 4] = (e->out.size - aligned) >> 3 & ~7u;
  r[0x064 / 4] = 0; /* no MAD QP adjustment */
  /* the picture's QP as init, min and max; the H1 codes slice_qp_delta
   * against 0x040's */
  r[0x06c / 4] = (uint32_t)qp << 26 | (uint32_t)qp << 20 | (uint32_t)qp << 14;
  for (int i = 0; i < 9; i++)
    r[0x070 / 4 + i] = 0; /* no checkpoints */
  r[0x094 / 4] = nrem * 8 << 23;
  r[0x098 / 4] = 0;
  r[0x0cc / 4] = e->cabac ? e->cabac_tbl.phys : 0;
  r[0x0d0 / 4] = 0;
  r[0x0d4 / 4] = 0x962b4c85;
  r[0x0d8 / 4] = 0x90901d50;
  r[0x0dc / 4] = 0x0000b694;
  /* intra area and ROIs outside the picture */
  r[0x0e0 / 4] = mbw << 24 | mbw << 16 | mbh << 8 | mbh;
  r[0x0f0 / 4] = mbw << 24 | mbw << 16 | mbh << 8 | mbh;
  r[0x0f4 / 4] = mbw << 24 | mbw << 16 | mbh << 8 | mbh;
  r[0x0f8 / 4] = MVC;
  for (int i = 0; i < 6; i++)
    r[0x180 / 4 + i] = ext[i];
  r[0x1b8 / 4] = r1b8[!idr];
  r[0x1bc / 4] = R1BC;
  r[0x1c0 / 4] = R1C0;
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
