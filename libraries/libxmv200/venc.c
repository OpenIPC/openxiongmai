// SPDX-License-Identifier: MIT
/*
 * H.265 on the V200 encoder engine (hevc.ko, /dev/h265, registers at
 * 0x21000000) fed by venc.ko's frame FIFO (/dev/venc_info).
 *
 * Frames: a VPSS channel bound to VENC (module 7) queues each picture in
 * venc.ko; 0xc0284902 takes one (40-byte record, a 2 s wait inside) and
 * 0x40284903 gives it back to its VB pool.
 *
 * Engine: one job encodes one picture as one slice. The job's register list
 * is 189 {value, write} pairs, entry n for the register at 4n; hevc.ko
 * writes them, starts the engine (1 at +0x004) and on its interrupt keeps
 * the slice length (+0x1d8) for 0xc0344802. 0x4803/0x4804 take and give
 * back the engine. The engine writes only slice_segment_data(); the
 * parameter sets and the slice header are ours.
 *
 * Reference and reconstruction share a ring of one picture plus three CTB
 * rows: each picture's reconstruction starts three rows below the last
 * one's, so it never overwrites rows the next picture still reads.
 */
#include <errno.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include "xmv200.h"

#define VENC_GET_FRAME 0xc0284902
#define VENC_RELEASE_FRAME 0x40284903
#define HEVC_LOCK 0x4803
#define HEVC_UNLOCK 0x4804
#define HEVC_JOB 0x40384805
#define HEVC_WAIT 0xc0344802
#define HEVC_RESET 0x40044806

#define NREGS 189
#define CTB 32
#define HDR_ROOM 0x100 /* the vendor's output starts this far in */

/* venc.ko's frame record */
struct venc_rec {
  uint32_t chn, y, width, height, stride, rsvd;
  uint32_t pts_lo, pts_hi, vb, rsvd2;
};

/* hevc.ko's job: only +4 (must be 0) and the list pointer are read; the rest
 * comes back from HEVC_WAIT, with the slice length at +44. */
struct hevc_job {
  uint32_t w[13];
  uint32_t regs;
};

/*
 * Engine registers that every picture writes the same: what the vendor
 * encoder programs for CTB 32, transforms up to 32, SAO and its intra/inter
 * decision tables. The fields we understand are set in venc_regs().
 */
static const uint16_t tmpl_off[] = {
    0x008, 0x00c, 0x010, 0x014, 0x018, 0x01c, 0x020, 0x024, 0x03c, 0x048,
    0x064, 0x068, 0x06c, 0x070, 0x074, 0x078, 0x07c, 0x080, 0x084, 0x088,
    0x08c, 0x090, 0x094, 0x098, 0x09c, 0x0a0, 0x0a4, 0x0a8, 0x0ac, 0x0b0,
    0x0b4, 0x0b8, 0x0bc, 0x0c0, 0x0c4, 0x0c8, 0x0cc, 0x0d0, 0x0d4, 0x0dc,
    0x0e0, 0x0e4, 0x0e8, 0x0ec, 0x0f0, 0x0f4, 0x0f8, 0x0fc, 0x100, 0x104,
    0x108, 0x10c, 0x110, 0x114, 0x118, 0x11c, 0x120, 0x124, 0x128, 0x12c,
    0x130, 0x134, 0x138, 0x13c, 0x140, 0x144, 0x148, 0x14c, 0x150, 0x154,
    0x158, 0x15c, 0x160, 0x164, 0x168, 0x16c, 0x170, 0x174, 0x178, 0x17c,
    0x180, 0x184, 0x188, 0x18c, 0x190, 0x194, 0x198, 0x19c, 0x1a0, 0x1a4,
    0x1a8, 0x1ac, 0x1b0, 0x1b4, 0x1b8, 0x1bc, 0x1c0, 0x1c4, 0x1c8, 0x1cc,
    0x1d0, 0x288, 0x28c, 0x290, 0x294, 0x298, 0x29c, 0x2a0, 0x2b0, 0x2c4,
    0x2c8, 0x2cc, 0x2dc, 0x2e0, 0x2e4, 0x2e8};
static const uint32_t tmpl_val[] = {
    0x3,        0x3,        0x3,        0x1,        0xe,        0x0,
    0x0,        0x0,        0x0,        0x100000,   0x0,        0x1f,
    0xb,        0x20c4146,  0x728824b,  0xc34e3d0,  0x11fff493, 0x145565da,
    0x1865b71d, 0x1e7e0862, 0x3fffffff, 0x3fffffff, 0x1,        0x28a1,
    0x8000a52,  0x0,        0x0,        0x0,        0x0,        0x0,
    0x0,        0x10,       0x0,        0x0,        0x8,        0x50000,
    0x16aa,     0x10,       0x22,       0x1,        0x0,        0x2,
    0xf0d0c0c,  0xb3,       0x180,      0x169,      0xbb,       0xc0,
    0xc5,       0x16c,      0xcd,       0x27d017,   0x285ffd,   0x2a4fee,
    0x29cfe9,   0x2b3003,   0x221ffd,   0x1e9ffd,   0xa,        0x76,
    0x28a,      0x145,      0xda,       0x66,       0x28f,      0x11c07d,
    0x31f000,   0x28f000,   0x245000,   0x35f000,   0x29a000,   0x20d000,
    0x14,       0x46,       0x15e,      0x1,        0x3,        0x5,
    0xe,        0x28,       0x1,        0x1a,       0x18171717, 0x17161616,
    0x1515,     0x383a4048, 0x4a,       0x1,        0x1,        0x180c,
    0x28282828, 0x20202020, 0x20202020, 0x2020,     0x10101010, 0x1010,
    0x1,        0x0,        0x0,        0x1,        0x0,        0x0,
    0x0,        0x0,        0x0,        0x0,        0x0,        0x0,
    0x1,        0x3e807d0,  0xbb80fa0,  0x138a1770, 0x40008,    0x100020,
    0x400080,   0x820739c};

/* the picture-type presets at 0x2a4..0x2d4: IDR, the first P, later P */
static const uint16_t type_off[] = {0x2a4, 0x2a8, 0x2ac, 0x2b4,
                                    0x2b8, 0x2bc, 0x2d0, 0x2d4};
static const uint32_t type_idr[] = {0x2f,    0x1c, 0x2b,      0x4,
                                    0x43332, 0x1,  0x8314e95, 0x15};
static const uint32_t type_p1[] = {0x2,    0x23, 0x26,      0x2d,
                                   0x1ddd, 0xa,  0x8317fbb, 0x1a};
static const uint32_t type_p[] = {0x3,    0x23, 0x2b,      0x2d,
                                  0x1ddd, 0xa,  0x8317fbb, 0x1a};
/* the same for pictures up to 1280 wide (the vendor's sub stream) */
static const uint32_t type_idr_s[] = {0x197,   0x1c, 0x2b,      0x4,
                                      0x2cccc, 0x1,  0x8314e95, 0x15};
static const uint32_t type_p1_s[] = {0x16,   0x23, 0x26,      0xa,
                                     0x13e9, 0xa,  0x8317fbb, 0x1a};
static const uint32_t type_p_s[] = {0x1c,   0x23, 0x2b,      0xa,
                                    0x13e9, 0xa,  0x8317fbb, 0x1a};

struct xmv200_venc {
  int fd_hevc, fd_mem;
  uint32_t width, height, h32, qp, gop, fps;
  struct xmv200_buf luma, chroma; /* reference/reconstruction rings */
  struct xmv200_buf stream;
  uint8_t *stream_map;
  uint32_t frame, recon, poc;
  uint32_t regs[NREGS][2];
};

/* ---- bitstream ---- */

struct bw {
  uint8_t *p;
  int cap, len; /* bytes */
  uint64_t acc;
  int nacc;
};

static void put(struct bw *b, uint32_t v, int n) {
  while (n--) {
    b->acc = (b->acc << 1) | ((v >> n) & 1);
    if (++b->nacc == 8) {
      if (b->len < b->cap)
        b->p[b->len] = (uint8_t)b->acc;
      b->len++;
      b->nacc = 0;
      b->acc = 0;
    }
  }
}

static void ue(struct bw *b, uint32_t v) {
  int n = 0;

  for (uint32_t t = v + 1; t > 1; t >>= 1)
    n++;
  put(b, 0, n);
  put(b, v + 1, n + 1);
}

static void se(struct bw *b, int v) { ue(b, v > 0 ? 2 * v - 1 : -2 * v); }

static void trailing(struct bw *b) {
  put(b, 1, 1);
  while (b->nacc)
    put(b, 0, 1);
}

/* start code + NAL header, then the RBSP with emulation prevention */
static int nal(uint8_t *out, int cap, int type, const uint8_t *rbsp, int len) {
  int n = 0, zeros = 0;

  if (cap < 6 + len * 3 / 2)
    return -1;
  out[n++] = 0, out[n++] = 0, out[n++] = 0, out[n++] = 1;
  out[n++] = type << 1;
  out[n++] = 1;
  for (int i = 0; i < len; i++) {
    if (zeros >= 2 && rbsp[i] <= 3) {
      out[n++] = 3;
      zeros = 0;
    }
    out[n++] = rbsp[i];
    zeros = rbsp[i] ? 0 : zeros + 1;
  }
  return n;
}

static void ptl(struct bw *b) {
  put(b, 0, 2);       /* profile_space */
  put(b, 0, 1);       /* tier */
  put(b, 1, 5);       /* Main */
  put(b, 0x60000000, 32); /* compatible with Main and Main 10 */
  put(b, 0, 4);       /* progressive/interlaced/non-packed/frame-only */
  put(b, 0, 32);
  put(b, 0, 12);
  put(b, 63, 8); /* level 2.1 as the vendor encoder writes it */
}

static int write_vps(struct xmv200_venc *v, uint8_t *rbsp, int cap) {
  struct bw b = {rbsp, cap};

  put(&b, 0, 4);      /* vps_id */
  put(&b, 3, 2);      /* base layer internal + available */
  put(&b, 0, 6);      /* max_layers_minus1 */
  put(&b, 0, 3);      /* max_sub_layers_minus1 */
  put(&b, 1, 1);      /* temporal_id_nesting */
  put(&b, 0xffff, 16);
  ptl(&b);
  put(&b, 0, 1);      /* sub_layer_ordering_info_present */
  ue(&b, 1);          /* max_dec_pic_buffering_minus1 */
  ue(&b, 0);
  ue(&b, 0);
  put(&b, 0, 6);      /* max_layer_id */
  ue(&b, 0);          /* num_layer_sets_minus1 */
  put(&b, 1, 1);      /* timing_info_present */
  put(&b, 1000, 32);
  put(&b, v->fps * 1000, 32);
  put(&b, 0, 1);
  ue(&b, 0);          /* num_hrd_parameters */
  put(&b, 0, 1);      /* extension */
  trailing(&b);
  return b.len;
}

static int write_sps(struct xmv200_venc *v, uint8_t *rbsp, int cap) {
  struct bw b = {rbsp, cap};

  put(&b, 0, 4);      /* vps_id */
  put(&b, 0, 3);      /* max_sub_layers_minus1 */
  put(&b, 1, 1);
  ptl(&b);
  ue(&b, 0);          /* sps_id */
  ue(&b, 1);          /* 4:2:0 */
  ue(&b, v->width);
  ue(&b, v->height);
  put(&b, 1, 1);      /* conformance window, all zero as the vendor's */
  ue(&b, 0);
  ue(&b, 0);
  ue(&b, 0);
  ue(&b, 0);
  ue(&b, 0);          /* bit depths */
  ue(&b, 0);
  ue(&b, 6);          /* log2_max_poc_lsb_minus4: 10 bits */
  put(&b, 0, 1);
  ue(&b, 1);
  ue(&b, 0);
  ue(&b, 0);
  ue(&b, 0);          /* min CB 8 */
  ue(&b, 2);          /* CTB 32 */
  ue(&b, 0);          /* min TB 4 */
  ue(&b, 3);          /* max TB 32 */
  ue(&b, 0);          /* transform hierarchy depths */
  ue(&b, 0);
  put(&b, 0, 1);      /* scaling lists */
  put(&b, 0, 1);      /* AMP */
  put(&b, 1, 1);      /* SAO */
  put(&b, 0, 1);      /* PCM */
  ue(&b, 1);          /* one short-term RPS: the previous picture */
  ue(&b, 1);
  ue(&b, 0);
  ue(&b, 0);
  put(&b, 1, 1);
  put(&b, 0, 1);      /* long-term refs */
  put(&b, 0, 1);      /* TMVP */
  put(&b, 0, 1);      /* strong intra smoothing */
  put(&b, 1, 1);      /* VUI: timing only */
  put(&b, 0, 8);
  put(&b, 1, 1);
  put(&b, 1000, 32);
  put(&b, v->fps * 1000, 32);
  put(&b, 0, 1);
  put(&b, 0, 1);      /* HRD */
  put(&b, 0, 1);      /* bitstream restriction */
  put(&b, 0, 1);      /* extension */
  trailing(&b);
  return b.len;
}

static int write_pps(uint8_t *rbsp, int cap) {
  struct bw b = {rbsp, cap};

  ue(&b, 0);
  ue(&b, 0);
  put(&b, 0, 1);      /* dependent slices */
  put(&b, 0, 1);      /* output flag */
  put(&b, 0, 3);
  put(&b, 0, 1);      /* sign data hiding */
  put(&b, 1, 1);      /* cabac_init_present */
  ue(&b, 0);
  ue(&b, 0);
  se(&b, 0);          /* init_qp 26 */
  put(&b, 0, 1);
  put(&b, 0, 1);      /* transform skip */
  put(&b, 1, 1);      /* cu_qp_delta */
  ue(&b, 0);
  se(&b, 0);
  se(&b, 0);
  put(&b, 0, 1);
  put(&b, 0, 1);      /* weighted pred */
  put(&b, 0, 1);
  put(&b, 0, 1);      /* transquant bypass */
  put(&b, 0, 1);      /* tiles */
  put(&b, 0, 1);      /* WPP */
  put(&b, 1, 1);      /* loop filter across slices */
  put(&b, 1, 1);      /* deblocking control */
  put(&b, 1, 1);      /* override enabled */
  put(&b, 0, 1);
  se(&b, 5);          /* beta */
  se(&b, -2);         /* tc */
  put(&b, 0, 1);
  put(&b, 0, 1);
  ue(&b, 0);
  put(&b, 0, 1);
  put(&b, 0, 1);
  trailing(&b);
  return b.len;
}

static int write_slice_header(struct xmv200_venc *v, int idr, uint8_t *rbsp,
                              int cap) {
  struct bw b = {rbsp, cap};

  put(&b, 1, 1);      /* first slice */
  if (idr)
    put(&b, 0, 1);    /* no_output_of_prior_pics */
  ue(&b, 0);          /* pps */
  ue(&b, idr ? 2 : 1);
  if (!idr) {
    put(&b, v->poc & 0x3ff, 10);
    put(&b, 1, 1);    /* the SPS's RPS */
  }
  put(&b, 1, 1);      /* SAO luma, chroma */
  put(&b, 1, 1);
  if (!idr) {
    put(&b, 0, 1);    /* num_ref_idx override */
    put(&b, 0, 1);    /* cabac_init_flag */
    ue(&b, 1);        /* five_minus_max_num_merge_cand */
  }
  se(&b, (int)v->qp - 26);
  put(&b, 1, 1);      /* deblocking override */
  put(&b, 0, 1);
  se(&b, 5);
  se(&b, -2);
  put(&b, 1, 1);      /* loop filter across slices */
  trailing(&b);       /* byte_alignment() */
  return b.len;
}

/* ---- engine ---- */

static void reg(struct xmv200_venc *v, uint32_t off, uint32_t val) {
  v->regs[off / 4][0] = val;
  v->regs[off / 4][1] = 1;
}

static void venc_regs(struct xmv200_venc *v) {
  uint32_t lsz = v->width * v->h32, band = v->width * CTB * 3;

  memset(v->regs, 0, sizeof(v->regs));
  for (unsigned i = 0; i < sizeof(tmpl_off) / sizeof(tmpl_off[0]); i++)
    reg(v, tmpl_off[i], tmpl_val[i]);
  reg(v, 0x034, v->width - 1);
  reg(v, 0x038, v->height - 1);
  reg(v, 0x044, v->qp);
  reg(v, 0x1e8, v->luma.phys); /* reference ring */
  reg(v, 0x1ec, v->chroma.phys);
  reg(v, 0x1f0, v->luma.phys); /* reconstruction ring (the same) */
  reg(v, 0x1f4, v->chroma.phys);
  for (uint32_t o = 0x214; o <= 0x228; o += 4)
    reg(v, o, v->width);
  reg(v, 0x230, lsz);
  reg(v, 0x234, lsz / 2);
  reg(v, 0x238, lsz + band);
  reg(v, 0x23c, (lsz + band) / 2);
  reg(v, 0x2c0, v->width > 1280);
  reg(v, 0x2d8, v->width > 1280);
}

int xmv200_venc_create(struct xmv200_venc **pv, uint32_t width,
                       uint32_t height, uint32_t fps, uint32_t qp,
                       uint32_t gop) {
  struct xmv200_venc *v = calloc(1, sizeof(*v));

  if (v)
    v->fd_hevc = v->fd_mem = -1;
  uint32_t lsz, band, reset = 0x10002;

  if (!v)
    return -1;
  v->width = width;
  v->height = height;
  v->h32 = (height + CTB - 1) & ~(CTB - 1);
  v->fps = fps ? fps : 25;
  v->qp = qp ? qp : 35;
  v->gop = gop ? gop : 2 * v->fps;
  lsz = width * v->h32;
  band = width * CTB * 3;
  v->fd_hevc = open("/dev/h265", O_RDWR);
  v->fd_mem = open("/dev/mem", O_RDWR | O_SYNC);
  if (v->fd_hevc < 0 || v->fd_mem < 0 ||
      xmv200_mmz_alloc(&v->luma, "vencWrapLuma", lsz + band, 0) ||
      xmv200_mmz_alloc(&v->chroma, "vencWrapChroma", (lsz + band) / 2, 0) ||
      xmv200_mmz_alloc(&v->stream, "stream_chn", lsz / 4 + HDR_ROOM, 0))
    goto fail;
  /* the slice data read back uncached */
  v->stream_map = mmap(NULL, v->stream.size, PROT_READ, MAP_SHARED, v->fd_mem,
                       v->stream.phys);
  if (v->stream_map == MAP_FAILED)
    goto fail;
  ioctl(v->fd_hevc, HEVC_RESET, &reset);
  venc_regs(v);
  *pv = v;
  return 0;
fail:
  xmv200_venc_destroy(v);
  return -1;
}

void xmv200_venc_destroy(struct xmv200_venc *v) {
  if (!v)
    return;
  if (v->stream_map && v->stream_map != MAP_FAILED)
    munmap(v->stream_map, v->stream.size);
  if (v->fd_hevc >= 0)
    close(v->fd_hevc);
  if (v->fd_mem >= 0)
    close(v->fd_mem);
  free(v);
}

void xmv200_venc_request_idr(struct xmv200_venc *v) { v->frame = 0; }

/* Encodes one picture from VPSS into out: parameter sets before an IDR,
 * then the slice. Returns the length, or -1. */
static int encode(struct xmv200_venc *v, uint32_t y, uint32_t c,
                  uint8_t *out, int cap) {
  uint32_t lsz = v->width * v->h32, band = v->width * CTB * 3;
  uint32_t ring = lsz + band;
  uint8_t rbsp[128];
  struct hevc_job job;
  int idr = v->frame % v->gop == 0, n = 0, k, len;
  const uint32_t *preset;

  if (idr) {
    v->poc = 0;
    if ((k = nal(out + n, cap - n, 32, rbsp, write_vps(v, rbsp, 128))) < 0)
      return -1;
    n += k;
    if ((k = nal(out + n, cap - n, 33, rbsp, write_sps(v, rbsp, 128))) < 0)
      return -1;
    n += k;
    if ((k = nal(out + n, cap - n, 34, rbsp, write_pps(rbsp, 128))) < 0)
      return -1;
    n += k;
  }
  k = nal(out + n, cap - n, idr ? 19 : 1, rbsp,
          write_slice_header(v, idr, rbsp, sizeof(rbsp)));
  if (k < 0)
    return -1;
  n += k;

  if (v->width > 1280)
    preset = idr ? type_idr : v->frame % v->gop == 1 ? type_p1 : type_p;
  else
    preset = idr ? type_idr_s : v->frame % v->gop == 1 ? type_p1_s : type_p_s;
  for (unsigned i = 0; i < sizeof(type_off) / sizeof(type_off[0]); i++)
    reg(v, type_off[i], preset[i]);
  reg(v, 0x040, !idr);
  reg(v, 0x1e0, y);
  reg(v, 0x1e4, c);
  reg(v, 0x200, v->stream.phys + HDR_ROOM);
  reg(v, 0x1d4, lsz / 4);
  /* the reference is the last reconstruction; this one starts three CTB
   * rows below it in the ring */
  uint32_t ref = v->recon, rec = idr ? 0 : (v->recon + ring - band) % ring;
  reg(v, 0x240, idr ? lsz : ref);
  reg(v, 0x244, (idr ? lsz : ref) / 2);
  reg(v, 0x248, rec);
  reg(v, 0x24c, rec / 2);

  memset(&job, 0, sizeof(job));
  job.regs = (uint32_t)(uintptr_t)v->regs;
  if (ioctl(v->fd_hevc, HEVC_LOCK, 0) < 0)
    return -1;
  if (ioctl(v->fd_hevc, HEVC_JOB, &job) < 0 ||
      ioctl(v->fd_hevc, HEVC_WAIT, &job) < 0) {
    ioctl(v->fd_hevc, HEVC_UNLOCK, 0);
    return -1;
  }
  ioctl(v->fd_hevc, HEVC_UNLOCK, 0);
  len = job.w[11];
  if (len <= 0 || (uint32_t)len > v->stream.size - HDR_ROOM || n + len > cap)
    return -1;
  memcpy(out + n, v->stream_map + HDR_ROOM, len);
  v->recon = rec;
  v->frame++;
  v->poc++;
  return n + len;
}

static int venc_fd = -1;

int xmv200_frame_get(struct xmv200_frame *f) {
  struct venc_rec r;

  if (venc_fd < 0)
    venc_fd = open("/dev/venc_info", O_RDWR);
  if (venc_fd < 0 || ioctl(venc_fd, VENC_GET_FRAME, &r) < 0)
    return -1;
  f->chn = r.chn;
  f->y = r.y;
  f->c = r.y + r.stride * r.height;
  f->width = r.width;
  f->height = r.height;
  f->stride = r.stride;
  f->pts = (uint64_t)r.pts_hi << 32 | r.pts_lo;
  memcpy(f->priv, &r, sizeof(r));
  return 0;
}

void xmv200_frame_release(struct xmv200_frame *f) {
  ioctl(venc_fd, VENC_RELEASE_FRAME, f->priv);
}

int xmv200_venc_encode(struct xmv200_venc *v, const struct xmv200_frame *f,
                       uint8_t *out, int cap) {
  if (f->width != v->width || f->height != v->height)
    return -1;
  return encode(v, f->y, f->c, out, cap);
}
