/*
 * xmvenc-test: encode H.264 on the XM530/XM550 Hantro 8290 (H1) encoder
 * from a synthetic NV12 picture, without vendor libraries. IDR and P
 * frames, CAVLC or CABAC, fixed QP.
 *
 * Software writes SPS and PPS; the hardware writes the slice (start code,
 * NAL header, slice header and data).
 *
 * Register layout: Rockchip VEPU1 (MPP hal_h264e_vepu1_reg_tbl.h), with
 * these 8290 differences, measured on an XM550 (OpenIPC/openxiongmai#13):
 *  - The window is 0x000-0x1c3 (HX280ENC_IOCGHWIOSIZE = 0x1c4); VEPU1's
 *    tables at 0x200 and up don't exist.
 *  - frame_num is in 0x048[15:0]; 0x044[15:0] is a per-frame-type constant
 *    (0x11d for P, 0xeb for IDR in the vendor stream).
 *  - 0x060 is written back with the coded size in bits.
 *  - 0x0fc reads 0x0b6f2a80 in every job; it is probably the synthesis
 *    configuration and is not decoded.
 *  - 0x180-0x194 hold constants the vendor writes for every job; their
 *    meaning is unknown (mainline H1 has a DMV penalty table there).
 *  - 0x198-0x1ac are written by the hardware: twelve 16-bit counters that
 *    add up to the macroblock count, a per-frame histogram.
 *  - 0x1b8-0x1c0 are configuration: 0x1b8 differs between P and IDR
 *    (plausibly per-bin QP deltas); 0x1bc and 0x1c0 are constant.
 * Completion: _IO('k', 12) on /dev/h264 sleeps until the frame interrupt;
 * the status is in 0x004 (bit 2 frame ready, bits 3-6 errors).
 *
 * Needs the encoder driver (/dev/h264, vendor xm530_h264.ko or hx280enc)
 * and /dev/mmz. Stop majestic first: the encoder serves one user at a time.
 *
 * Usage: xmvenc-test [-w width] [-h height] [-q qp] [-n frames]
 *                    [-g gop] [-c] [-v chn] [-d dump.nv12] [-j quality]
 *                    [-b kbps] [-f fps] [-Q qmin:qmax] [-o file]
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
 */
#define _DEFAULT_SOURCE
#include <errno.h>
#include <math.h>
#include <fcntl.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <time.h>
#include <unistd.h>

#include "cabac_tbl.h"

/* Encoder registers (byte offsets) */
#define REG_ID 0x000
#define REG_IRQ 0x004
#define REG_AXI 0x008
#define REG_ADDR_OUT 0x014
#define REG_ADDR_SIZE_TBL 0x018
#define REG_ADDR_REF_LUMA 0x01c
#define REG_ADDR_REF_CHROMA 0x020
#define REG_ADDR_REC_LUMA 0x024
#define REG_ADDR_REC_CHROMA 0x028
#define REG_ADDR_IN_LUMA 0x02c
#define REG_ADDR_IN_CB 0x030
#define REG_ADDR_IN_CR 0x034
#define REG_ENC_CTRL 0x038
#define REG_IN_IMG_CTRL 0x03c
#define REG_ENC_CTRL0 0x040
#define REG_ENC_CTRL1 0x044
#define REG_ENC_CTRL2 0x048
#define REG_ENC_CTRL3 0x04c
#define REG_ENC_CTRL4 0x054
#define REG_STR_HDR_REM_MSB 0x058
#define REG_STR_HDR_REM_LSB 0x05c
#define REG_STR_BUF_LIMIT 0x060
#define REG_MAD_CTRL 0x064
#define REG_QP_VAL 0x06c
#define REG_RLC_CTRL 0x094
#define REG_MB_CTRL 0x098
#define REG_ADDR_CABAC_TBL 0x0cc
#define REG_ADDR_MV_OUT 0x0d0
#define REG_RGB2YUV1 0x0d4
#define REG_RGB2YUV2 0x0d8
#define REG_RGB_MASK 0x0dc
#define REG_INTRA_AREA 0x0e0
#define REG_ROI1 0x0f0
#define REG_ROI2 0x0f4
#define REG_MVC 0x0f8
#define REG_WINDOW 0x1c4

#define IRQ_BIT (1u << 0)
#define IRQ_FRAME_READY (1u << 2)
#define IRQ_BUS_ERROR (1u << 3)
#define IRQ_RESET (1u << 4)
#define IRQ_BUFFER_FULL (1u << 5)
#define IRQ_TIMEOUT (1u << 6)

/* /dev/h264 ioctls ('k'), see drivers/hx280enc/hx280enc.h */
#define HX280ENC_IOCGHWOFFSET _IOR('k', 3, unsigned long *)
#define HX280ENC_IOCGHWIOSIZE _IOR('k', 4, unsigned int *)
#define HX280ENC_IOCWAITDONE _IO('k', 12)

/* /dev/mmz allocation. mmz.ko takes a record of name[32], size, phys and,
 * on XM530 only, a trailing word: 44 bytes there, 40 on XM510. Unknown
 * commands print "ioctl cmd error" but still return 0, so the address is
 * checked; the record size that works is remembered. Get-by-name fills
 * size, then phys. There is no free, only a reset of the whole zone, so
 * blocks from an earlier run are found again by name. */
#define MMZ_CMD(dir, nr, size)                                                 \
  (((uint32_t)(dir) << 30) | ((size) << 16) | ('M' << 8) | (nr))
#define MMZ_ALLOC 1
#define MMZ_GET 3

struct buf {
  uint32_t phys;
  uint8_t *virt;
  uint32_t size;
};

static int fd_mem = -1;

static int mmz_call(int nr, const char *name, uint32_t *size,
                    uint32_t *phys) {
  static const uint32_t sizes[] = {44, 40};
  static int fd_mmz = -1, abi = -1;
  struct {
    char name[32];
    uint32_t size;
    uint32_t phys;
    uint32_t rsvd;
  } arg;

  if (fd_mmz < 0 && (fd_mmz = open("/dev/mmz", O_RDWR)) < 0) {
    perror("/dev/mmz");
    return -1;
  }
  for (int i = abi < 0 ? 0 : abi; i < 2; i++) {
    memset(&arg, 0, sizeof(arg));
    strncpy(arg.name, name, sizeof(arg.name) - 1);
    arg.size = *size;
    if (ioctl(fd_mmz, MMZ_CMD(nr == MMZ_ALLOC ? 3 : 2, nr, sizes[i]), &arg) ==
            0 &&
        arg.phys >= 0x80000000u) {
      abi = i;
      *size = arg.size;
      *phys = arg.phys;
      return 0;
    }
    if (abi >= 0)
      break;
  }
  return -1;
}

/* Allocates a named block, or finds it again by name. Failing that, and
 * if given, takes the vendor stack's block of that name: majestic's
 * encoder buffers are idle while it is stopped, and on XM510 the zone has
 * no room for our own. */
static int mmz_alloc(struct buf *b, const char *name, uint32_t size,
                     const char *vendor) {
  uint32_t got = size, phys;
  int found = mmz_call(MMZ_ALLOC, name, &got, &phys) == 0;

  if (!found) {
    got = 0;
    found = mmz_call(MMZ_GET, name, &got, &phys) == 0 && got >= size;
  }
  if (!found && vendor) {
    got = 0;
    found = mmz_call(MMZ_GET, vendor, &got, &phys) == 0 && got >= size;
    if (found)
      printf("mmz: using the vendor's idle %s for %s\n", vendor, name);
  }
  if (!found) {
    fprintf(stderr,
            "mmz: no %u-byte block for %s (a smaller one from an earlier run? "
            "restart majestic to reset MMZ)\n",
            size, name);
    return -1;
  }
  b->phys = phys;
  b->size = size;
  b->virt = mmap(0, (size + 0xfff) & ~0xfff, PROT_READ | PROT_WRITE,
                 MAP_SHARED, fd_mem, b->phys);
  if (b->virt == MAP_FAILED) {
    perror("mmap mmz");
    return -1;
  }
  return 0;
}

/* Bit writer for SPS/PPS, with emulation prevention */
struct bw {
  uint8_t buf[64];
  int bytes, bits; /* bits pending in cur */
  uint32_t cur;
};

static void put_bits(struct bw *w, uint32_t v, int n) {
  while (n--) {
    w->cur = (w->cur << 1) | ((v >> n) & 1);
    if (++w->bits == 8) {
      w->buf[w->bytes++] = w->cur;
      w->cur = 0;
      w->bits = 0;
    }
  }
}

static void put_ue(struct bw *w, uint32_t v) {
  int len = 0;
  uint32_t x = v + 1;

  while (x >> (len + 1))
    len++;
  put_bits(w, 0, len);
  put_bits(w, x, len + 1);
}

static void put_se(struct bw *w, int v) {
  put_ue(w, v > 0 ? 2 * v - 1 : -2 * v);
}

static void put_trailing(struct bw *w) {
  put_bits(w, 1, 1);
  while (w->bits)
    put_bits(w, 0, 1);
}

/* Writes start code + NAL with emulation prevention, returns length */
static int put_nal(uint8_t *out, uint8_t nal_hdr, const struct bw *w) {
  int n = 0, zeros = 0;

  out[n++] = 0;
  out[n++] = 0;
  out[n++] = 0;
  out[n++] = 1;
  out[n++] = nal_hdr;
  for (int i = 0; i < w->bytes; i++) {
    if (zeros == 2 && w->buf[i] <= 3) {
      out[n++] = 3;
      zeros = 0;
    }
    out[n++] = w->buf[i];
    zeros = w->buf[i] ? 0 : zeros + 1;
  }
  return n;
}

/* Lowest level whose MaxFS fits the frame (H.264 table A-1) */
static int level_for(int mbs) {
  static const struct {
    int max_fs, level;
  } levels[] = {{1620, 30}, {3600, 31}, {5120, 32}, {8192, 40}, {22080, 50}};

  for (unsigned i = 0; i < sizeof(levels) / sizeof(levels[0]); i++)
    if (mbs <= levels[i].max_fs)
      return levels[i].level;
  return 51;
}

/* SPS/PPS matching what the hardware slice header assumes: frame_num is
 * 16 bits (log2_max_frame_num_minus4 = 12) and pic_order_cnt_type is 2,
 * as in the vendor stream. */
static int write_sps_pps(uint8_t *out, int width, int height, int qp,
                         int cabac) {
  struct bw w;
  int mbw = (width + 15) / 16, mbh = (height + 15) / 16, n;

  memset(&w, 0, sizeof(w));
  put_bits(&w, 77, 8); /* profile_idc: Main */
  put_bits(&w, 0, 8);  /* constraint flags */
  put_bits(&w, level_for(mbw * mbh), 8); /* level_idc */
  put_ue(&w, 0);       /* seq_parameter_set_id */
  put_ue(&w, 12);      /* log2_max_frame_num_minus4 */
  put_ue(&w, 2);       /* pic_order_cnt_type */
  put_ue(&w, 1);       /* max_num_ref_frames */
  put_bits(&w, 0, 1);  /* gaps_in_frame_num_value_allowed_flag */
  put_ue(&w, mbw - 1);
  put_ue(&w, mbh - 1);
  put_bits(&w, 1, 1); /* frame_mbs_only_flag */
  put_bits(&w, 1, 1); /* direct_8x8_inference_flag */
  if (width != mbw * 16 || height != mbh * 16) {
    put_bits(&w, 1, 1); /* frame_cropping_flag, in 2-pixel units */
    put_ue(&w, 0);
    put_ue(&w, (mbw * 16 - width) / 2);
    put_ue(&w, 0);
    put_ue(&w, (mbh * 16 - height) / 2);
  } else {
    put_bits(&w, 0, 1);
  }
  put_bits(&w, 0, 1); /* vui_parameters_present_flag */
  put_trailing(&w);
  n = put_nal(out, 0x67, &w);

  memset(&w, 0, sizeof(w));
  put_ue(&w, 0);      /* pic_parameter_set_id */
  put_ue(&w, 0);      /* seq_parameter_set_id */
  put_bits(&w, cabac, 1); /* entropy_coding_mode_flag */
  put_bits(&w, 0, 1); /* bottom_field_pic_order_in_frame_present_flag */
  put_ue(&w, 0);      /* num_slice_groups_minus1 */
  put_ue(&w, 0);      /* num_ref_idx_l0_default_active_minus1 */
  put_ue(&w, 0);      /* num_ref_idx_l1_default_active_minus1 */
  put_bits(&w, 0, 1); /* weighted_pred_flag */
  put_bits(&w, 0, 2); /* weighted_bipred_idc */
  put_se(&w, qp - 26); /* pic_init_qp_minus26 */
  put_se(&w, 0);       /* pic_init_qs_minus26 */
  put_se(&w, 0);       /* chroma_qp_index_offset */
  put_bits(&w, 1, 1);  /* deblocking_filter_control_present_flag */
  put_bits(&w, 0, 1);  /* constrained_intra_pred_flag */
  put_bits(&w, 0, 1);  /* redundant_pic_cnt_present_flag */
  put_trailing(&w);
  n += put_nal(out + n, 0x68, &w);
  return n;
}

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

/* CABAC context tables for every QP, intra then inter (cabac_init_idc),
 * 464 bytes each, in the word order the encoder reads. Matches the table
 * the vendor stack builds, apart from the 4 padding bytes per block. */
#define CABAC_TBL_SIZE (52 * 2 * 464)

static void write_cabac_table(uint8_t *dst, int cabac_init_idc) {
  static uint8_t t[CABAC_TBL_SIZE];

  memset(t, 0, sizeof(t));
  for (int qp = 0; qp < 52; qp++)
    for (int j = 0; j < 2; j++) {
      const signed char(*ctx)[2] =
          j ? h264_context_init[cabac_init_idc] : h264_context_init_intra;

      for (int i = 0; i < 460; i++) {
        int pre = ((ctx[i][0] * qp) >> 4) + ctx[i][1];

        pre = pre < 1 ? 1 : pre > 126 ? 126 : pre;
        t[qp * 928 + j * 464 + i] =
            pre <= 63 ? (63 - pre) << 1 : ((pre - 64) << 1) | 1;
      }
    }
  /* each pair of 32-bit words: swapped, and each byte-reversed */
  for (int k = 0; k < CABAC_TBL_SIZE; k += 8)
    for (int b = 0; b < 4; b++) {
      dst[k + b] = t[k + 7 - b];
      dst[k + 4 + b] = t[k + 3 - b];
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

static void vi_cleanup(void) {
  if (vi.fd < 0)
    return;
  vi_release();
  ioctl(vi.fd, VI_IOC_DISABLE_CHN, &vi.chn);
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

/*
 * JPEG (baseline, 4:2:0, ITU-T T.81). Software writes the header up to
 * and including SOS; the hardware writes the entropy-coded scan. The
 * quantisation tables are the T.81 Annex K examples scaled the IJG way,
 * and the Huffman tables are the Annex K ones, which the encoder uses.
 */
static const uint8_t jpeg_q_luma[64] = {
    16, 11, 10, 16, 24,  40,  51,  61,  12, 12, 14, 19, 26,  58,  60,  55,
    14, 13, 16, 24, 40,  57,  69,  56,  14, 17, 22, 29, 51,  87,  80,  62,
    18, 22, 37, 56, 68,  109, 103, 77,  24, 35, 55, 64, 81,  104, 113, 92,
    49, 64, 78, 87, 103, 121, 120, 101, 72, 92, 95, 98, 112, 100, 103, 99};
static const uint8_t jpeg_q_chroma[64] = {
    17, 18, 24, 47, 99, 99, 99, 99, 18, 21, 26, 66, 99, 99, 99, 99,
    24, 26, 56, 99, 99, 99, 99, 99, 47, 66, 99, 99, 99, 99, 99, 99,
    99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99,
    99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99, 99};
static const uint8_t jpeg_zigzag[64] = {
    0,  1,  8,  16, 9,  2,  3,  10, 17, 24, 32, 25, 18, 11, 4,  5,
    12, 19, 26, 33, 40, 48, 41, 34, 27, 20, 13, 6,  7,  14, 21, 28,
    35, 42, 49, 56, 57, 50, 43, 36, 29, 22, 15, 23, 30, 37, 44, 51,
    58, 59, 52, 45, 38, 31, 39, 46, 53, 60, 61, 54, 47, 55, 62, 63};
/* Huffman tables: 16 code-length counts, then the values */
static const uint8_t jpeg_dc_luma[] = {
    0, 1, 5, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0, 0, 0,
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
static const uint8_t jpeg_dc_chroma[] = {
    0, 3, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 0, 0, 0, 0,
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11};
static const uint8_t jpeg_ac_luma[] = {
    0,    2,    1,    3,    3,    2,    4,    3,    5,    5,    4,    4,
    0,    0,    1,    0x7d, 0x01, 0x02, 0x03, 0x00, 0x04, 0x11, 0x05, 0x12,
    0x21, 0x31, 0x41, 0x06, 0x13, 0x51, 0x61, 0x07, 0x22, 0x71, 0x14, 0x32,
    0x81, 0x91, 0xa1, 0x08, 0x23, 0x42, 0xb1, 0xc1, 0x15, 0x52, 0xd1, 0xf0,
    0x24, 0x33, 0x62, 0x72, 0x82, 0x09, 0x0a, 0x16, 0x17, 0x18, 0x19, 0x1a,
    0x25, 0x26, 0x27, 0x28, 0x29, 0x2a, 0x34, 0x35, 0x36, 0x37, 0x38, 0x39,
    0x3a, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4a, 0x53, 0x54, 0x55,
    0x56, 0x57, 0x58, 0x59, 0x5a, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68, 0x69,
    0x6a, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7a, 0x83, 0x84, 0x85,
    0x86, 0x87, 0x88, 0x89, 0x8a, 0x92, 0x93, 0x94, 0x95, 0x96, 0x97, 0x98,
    0x99, 0x9a, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7, 0xa8, 0xa9, 0xaa, 0xb2,
    0xb3, 0xb4, 0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xc2, 0xc3, 0xc4, 0xc5,
    0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xd2, 0xd3, 0xd4, 0xd5, 0xd6, 0xd7, 0xd8,
    0xd9, 0xda, 0xe1, 0xe2, 0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9, 0xea,
    0xf1, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8, 0xf9, 0xfa};
static const uint8_t jpeg_ac_chroma[] = {
    0,    2,    1,    2,    4,    4,    3,    4,    7,    5,    4,    4,
    0,    1,    2,    0x77, 0x00, 0x01, 0x02, 0x03, 0x11, 0x04, 0x05, 0x21,
    0x31, 0x06, 0x12, 0x41, 0x51, 0x07, 0x61, 0x71, 0x13, 0x22, 0x32, 0x81,
    0x08, 0x14, 0x42, 0x91, 0xa1, 0xb1, 0xc1, 0x09, 0x23, 0x33, 0x52, 0xf0,
    0x15, 0x62, 0x72, 0xd1, 0x0a, 0x16, 0x24, 0x34, 0xe1, 0x25, 0xf1, 0x17,
    0x18, 0x19, 0x1a, 0x26, 0x27, 0x28, 0x29, 0x2a, 0x35, 0x36, 0x37, 0x38,
    0x39, 0x3a, 0x43, 0x44, 0x45, 0x46, 0x47, 0x48, 0x49, 0x4a, 0x53, 0x54,
    0x55, 0x56, 0x57, 0x58, 0x59, 0x5a, 0x63, 0x64, 0x65, 0x66, 0x67, 0x68,
    0x69, 0x6a, 0x73, 0x74, 0x75, 0x76, 0x77, 0x78, 0x79, 0x7a, 0x82, 0x83,
    0x84, 0x85, 0x86, 0x87, 0x88, 0x89, 0x8a, 0x92, 0x93, 0x94, 0x95, 0x96,
    0x97, 0x98, 0x99, 0x9a, 0xa2, 0xa3, 0xa4, 0xa5, 0xa6, 0xa7, 0xa8, 0xa9,
    0xaa, 0xb2, 0xb3, 0xb4, 0xb5, 0xb6, 0xb7, 0xb8, 0xb9, 0xba, 0xc2, 0xc3,
    0xc4, 0xc5, 0xc6, 0xc7, 0xc8, 0xc9, 0xca, 0xd2, 0xd3, 0xd4, 0xd5, 0xd6,
    0xd7, 0xd8, 0xd9, 0xda, 0xe2, 0xe3, 0xe4, 0xe5, 0xe6, 0xe7, 0xe8, 0xe9,
    0xea, 0xf2, 0xf3, 0xf4, 0xf5, 0xf6, 0xf7, 0xf8, 0xf9, 0xfa};

/* Quality 1-100 to a table in raster order, as libjpeg does */
static void jpeg_scale_q(uint8_t *q, const uint8_t *base, int quality) {
  int scale = quality < 50 ? 5000 / quality : 200 - 2 * quality;

  for (int i = 0; i < 64; i++) {
    int v = (base[i] * scale + 50) / 100;
    q[i] = v < 1 ? 1 : v > 255 ? 255 : v;
  }
}

static uint8_t *put16(uint8_t *p, int v) {
  *p++ = v >> 8;
  *p++ = v;
  return p;
}

static uint8_t *put_dht(uint8_t *p, int class_id, const uint8_t *tbl) {
  int n = 0;

  for (int i = 0; i < 16; i++)
    n += tbl[i];
  p = put16(p, 0xffc4);
  p = put16(p, 2 + 1 + 16 + n);
  *p++ = class_id;
  memcpy(p, tbl, 16 + n);
  return p + 16 + n;
}

/* SOI, DQT, SOF0, DHT and SOS for a 4:2:0 picture; returns the length */
static int write_jpeg_header(uint8_t *out, int width, int height,
                             const uint8_t *qy, const uint8_t *qc) {
  uint8_t *p = out;

  p = put16(p, 0xffd8);
  for (int t = 0; t < 2; t++) {
    p = put16(p, 0xffdb);
    p = put16(p, 2 + 1 + 64);
    *p++ = t;
    for (int i = 0; i < 64; i++)
      *p++ = (t ? qc : qy)[jpeg_zigzag[i]];
  }
  p = put16(p, 0xffc0);
  p = put16(p, 2 + 6 + 3 * 3);
  *p++ = 8;
  p = put16(p, height);
  p = put16(p, width);
  *p++ = 3;
  const uint8_t comps[3][3] = {{1, 0x22, 0}, {2, 0x11, 1}, {3, 0x11, 1}};
  for (int c = 0; c < 3; c++) {
    memcpy(p, comps[c], 3);
    p += 3;
  }
  p = put_dht(p, 0x00, jpeg_dc_luma);
  p = put_dht(p, 0x10, jpeg_ac_luma);
  p = put_dht(p, 0x01, jpeg_dc_chroma);
  p = put_dht(p, 0x11, jpeg_ac_chroma);
  p = put16(p, 0xffda);
  p = put16(p, 2 + 1 + 3 * 2 + 3);
  *p++ = 3;
  const uint8_t sel[3][2] = {{1, 0x00}, {2, 0x11}, {3, 0x11}};
  for (int c = 0; c < 3; c++) {
    memcpy(p, sel[c], 2);
    p += 2;
  }
  *p++ = 0;  /* Ss */
  *p++ = 63; /* Se */
  *p++ = 0;  /* Ah, Al */
  return p - out;
}

/* The encoder's quantisation registers (0x100 luma, 0x140 chroma) hold
 * each table in this order, four entries per word, first in the MSB. */
static const uint8_t jpeg_q_reg_order[64] = {
    0, 8,  16, 24, 1, 9,  17, 25, 32, 40, 48, 56, 33, 41, 49, 57,
    2, 10, 18, 26, 3, 11, 19, 27, 34, 42, 50, 58, 35, 43, 51, 59,
    4, 12, 20, 28, 5, 13, 21, 29, 36, 44, 52, 60, 37, 45, 53, 61,
    6, 14, 22, 30, 7, 15, 23, 31, 38, 46, 54, 62, 39, 47, 55, 63};

/* Vendor values for the registers whose meaning isn't decoded, per chip
 * and frame type ([0] IDR, [1] P), as the vendor stack writes them
 * (OpenIPC/openxiongmai#13). The chips tell themselves apart by the
 * maximum picture width in 0x0fc[11:0]: 1280 on XM510, 2688 on XM550. */
struct chip {
  const char *name;
  uint32_t ctrl1[2], ctrl3[2], ctrl4[2], mvc;
  uint32_t ext[2][6]; /* 0x180-0x194 */
  uint32_t r1b8[2], r1bc, r1c0;
  uint32_t strm_size;    /* fits the vendor's stream buffer on XM510 */
  const char *vendor[6]; /* idle vendor blocks: stream, rec/ref, CABAC */
};

static const struct chip chip_xm530 = {
    "XM530/XM550",
    {0x006a00eb, 0x0081011d},
    {0x00009c28, 0x0000c433},
    {0x2d000087, 0x33000087},
    0x80002000,
    {{0x00000003, 0x606a707a, 0x4148545a, 0x10112131, 0x11111100, 0x11111100},
     {0x00000003, 0x606a707a, 0x4148545a, 0x10112131, 0x11111100,
      0x11111100}},
    {0x000abcdd, 0x999acdee},
    0x64209bdf,
    0x1b6db6db,
    1 << 20,
    {NULL, NULL, NULL, NULL, NULL, NULL},
};

static const struct chip chip_xm510 = {
    "XM510",
    {0x005c014a, 0x006302ee},
    {0x00013851, 0x00013851},
    {0x400000aa, 0x400000aa},
    0x00002000,
    {{0xa87088e8, 0xc8081070, 0x070b0b01, 0x800cba98, 0x76543210, 0},
     {0x6098ff28, 0x08406040, 0x070bff06, 0x800cba98, 0x76543210, 0}},
    {0, 0},
    0,
    0,
    0xfd000,
    {"h264buf_0", "LumRefBuffs_0:num[0]", "ChrRefBuffs_0:num[0]",
     "LumRefBuffs_0:num[1]", "ChrRefBuffs_0:num[1]",
     "CABAC_context_tables_0"},
};

/*
 * Frame-level rate control. Each frame has a budget of bitrate/fps bits,
 * and the bits spent over or under budget accumulate in a buffer that the
 * following P frames pay back over about a second. A frame's size
 * roughly halves for every 6 QP steps, so a P frame's QP moves by
 * 6*log2(actual/wanted), damped and capped at 2 per frame. IDR frames
 * aim at IDR_WEIGHT P budgets with a QP of their own, when there are P
 * frames to pay that back; with every frame an IDR (-g 1), they follow
 * the buffer the way P frames do.
 */
#define IDR_WEIGHT 4.0

struct rc {
  int on, qp_i, qp_p, qmin, qmax, gop;
  double budget, buffer, fps;
};

static int clampi(int v, int lo, int hi) {
  return v < lo ? lo : v > hi ? hi : v;
}

/* The size the next frame should have to pay the buffer back in about a
 * second, but never below a quarter of the budget */
static double rc_want(const struct rc *rc) {
  double want = rc->budget - rc->buffer / rc->fps;

  return want < rc->budget / 4 ? rc->budget / 4 : want;
}

static void rc_update(struct rc *rc, int idr, uint32_t bits) {
  rc->buffer += bits - rc->budget;
  if (idr && rc->gop > 1) {
    double d = 6 * log2(bits / (rc->budget * IDR_WEIGHT));

    rc->qp_i = clampi(rc->qp_i + clampi((int)lround(d / 2), -3, 3), rc->qmin,
                      rc->qmax);
  } else {
    int *qp = idr ? &rc->qp_i : &rc->qp_p;

    *qp = clampi(*qp + clampi((int)lround(3 * log2(bits / rc_want(rc))), -2, 2),
                 rc->qmin, rc->qmax);
  }
  /* keep IDR frames at least as good as P frames */
  if (rc->gop > 1 && rc->qp_i > rc->qp_p)
    rc->qp_i = rc->qp_p;
}

/* A positive finite number, all of the argument; -1 otherwise */
static double parse_rate(const char *arg) {
  char *end;
  double v = strtod(arg, &end);

  return end != arg && *end == '\0' && isfinite(v) && v > 0 ? v : -1;
}

static void onalrm(int s) { (void)s; }

/*
 * Waits for the frame. xm530_h264.ko and hx280enc have a blocking wait,
 * _IO('k', 12). xm510_h264.ko doesn't (its ioctls stop at 11): it signals
 * SIGIO through fasync instead. There the status register is polled; it
 * keeps its frame-ready and error bits after the driver's IRQ handler.
 * SIGIO is turned on only then, so that it can't interrupt the blocking
 * wait, and only so the driver has someone to signal.
 */
static int wait_frame(int fd, volatile uint32_t *regs) {
  static int poll_status;
  int ret = -1;

  if (!poll_status) {
    alarm(2);
    ret = ioctl(fd, HX280ENC_IOCWAITDONE);
    alarm(0);
    if (ret == 0 || errno != ENOTTY)
      return ret;
    poll_status = 1;
    signal(SIGIO, SIG_IGN);
    fcntl(fd, F_SETOWN, getpid());
    fcntl(fd, F_SETFL, fcntl(fd, F_GETFL) | O_ASYNC);
  }
  for (int i = 0; i < 2000; i++) {
    if (regs[REG_IRQ / 4] &
        (IRQ_FRAME_READY | IRQ_BUS_ERROR | IRQ_RESET | IRQ_BUFFER_FULL |
         IRQ_TIMEOUT))
      return 0;
    usleep(1000);
  }
  return -1;
}

int main(int argc, char **argv) {
  int width = 704, height = 576, qp = 26, frames = 1, gop = 30, cabac = 0,
      vi_chn = -1, jpeg_q = 0, opt;
  struct rc rc = {0, 0, 0, 20, 45, 0, 0, 0, 25};
  double kbps = 0;
  const char *out = "xmvenc-test.h264", *dump = NULL;
  volatile uint32_t *regs;
  struct buf in, strm, recon_y[2], recon_c[2], cabac_tbl = {0};
  unsigned long hwbase = 0;
  unsigned int hwsize = 0;
  int fd_enc, mbw, mbh, stride, ret;
  uint32_t irq, bits;

  while ((opt = getopt(argc, argv, "w:h:q:n:g:cv:d:j:b:f:Q:o:")) != -1) {
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
      rc.fps = parse_rate(optarg);
      break;
    case 'Q':
      if (sscanf(optarg, "%d:%d", &rc.qmin, &rc.qmax) != 2)
        rc.qmin = -1;
      break;
    case 'o': out = optarg; break;
    default:
      fprintf(stderr,
              "usage: %s [-w width] [-h height] [-q qp] [-n frames] "
              "[-g gop] [-c] [-v chn] [-d dump.nv12] [-j quality] "
              "[-b kbps] [-f fps] [-Q qmin:qmax] [-o file]\n",
              argv[0]);
      return 2;
    }
  }
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
  }

  /* The intra-area and ROI fields hold 8-bit macroblock counts. */
  if (width < 96 || height < 96 || width > 4080 || height > 4080 ||
      width % 8 || height % 2 || qp < 0 ||
      qp > 51 || frames < 1 || gop < 1 || jpeg_q < 0 || jpeg_q > 100 ||
      kbps < 0 || rc.fps <= 0 || rc.qmin < 0 || rc.qmax > 51 ||
      rc.qmin > rc.qmax) {
    fprintf(stderr, "bad size or qp\n");
    return 2;
  }
  if (kbps > 0 && !jpeg_q) {
    rc.on = 1;
    rc.budget = kbps * 1000 / rc.fps;
    rc.qp_i = rc.qp_p = clampi(qp, rc.qmin, rc.qmax);
    rc.gop = gop;
  }
  mbw = (width + 15) / 16;
  mbh = (height + 15) / 16;
  /* VI buffers are packed: the stride is the width */
  stride = vi_chn >= 0 ? width : mbw * 16;

  if ((fd_enc = open("/dev/h264", O_RDWR)) < 0) {
    perror("/dev/h264");
    return 1;
  }
  if (ioctl(fd_enc, HX280ENC_IOCGHWOFFSET, &hwbase) ||
      ioctl(fd_enc, HX280ENC_IOCGHWIOSIZE, &hwsize) || hwsize < REG_WINDOW) {
    fprintf(stderr, "encoder driver: no register window\n");
    return 1;
  }
  if ((fd_mem = open("/dev/mem", O_RDWR | O_SYNC)) < 0) {
    perror("/dev/mem");
    return 1;
  }
  regs = mmap(0, 0x1000, PROT_READ | PROT_WRITE, MAP_SHARED, fd_mem, hwbase);
  if (regs == MAP_FAILED) {
    perror("mmap regs");
    return 1;
  }
  printf("encoder id 0x%08x at 0x%08lx\n", regs[REG_ID / 4], hwbase);
  if ((regs[REG_ID / 4] >> 16) != 0x8290) {
    fprintf(stderr, "not a Hantro 8290\n");
    return 1;
  }
  if (regs[REG_ENC_CTRL / 4] & 1) {
    fprintf(stderr, "encoder busy: stop majestic first\n");
    return 1;
  }
  uint32_t max_width = regs[0x0fc / 4] & 0xfff;
  const struct chip *chip = max_width <= 1280 ? &chip_xm510 : &chip_xm530;
  printf("%s profile, max width %u\n", chip->name, max_width);
  if ((uint32_t)width > max_width) {
    fprintf(stderr, "%d is wider than this encoder's %u\n", width,
            max_width);
    return 1;
  }


  /* Buffers: NV12 input, stream, reconstruction and reference (separate
   * luma and chroma, as the vendor stack allocates them). */
  uint32_t luma = stride * mbh * 16, pic_size = luma * 3 / 2;
  /* MMZ has no free and blocks are found again by name, so size them for
   * at least the 5 MP sensor: later runs at any size up to that reuse
   * them. With majestic stopped this is 23 MB of the 26 MB left. */
  uint32_t luma_max = max_width < 2592 || luma > 2592 * 1952 ? luma
                                                             : 2592 * 1952;
  if ((vi_chn < 0 && mmz_alloc(&in, "xmvenc_in", luma_max * 3 / 2, NULL)) ||
      mmz_alloc(&strm, "xmvenc_strm", chip->strm_size, chip->vendor[0]) ||
      mmz_alloc(&recon_y[0], "xmvenc_rec_y", luma_max, chip->vendor[1]) ||
      mmz_alloc(&recon_c[0], "xmvenc_rec_c", luma_max / 2, chip->vendor[2]) ||
      mmz_alloc(&recon_y[1], "xmvenc_ref_y", luma_max, chip->vendor[3]) ||
      mmz_alloc(&recon_c[1], "xmvenc_ref_c", luma_max / 2, chip->vendor[4]))
    return 1;
  if (cabac) {
    if (mmz_alloc(&cabac_tbl, "xmvenc_cabac", CABAC_TBL_SIZE,
                  chip->vendor[5]))
      return 1;
    write_cabac_table(cabac_tbl.virt, 0);
  }
  uint8_t hdr[128];
  int hdr_len = jpeg_q ? 0 : write_sps_pps(hdr, width, height, qp, cabac);
  uint8_t jq_y[64], jq_c[64], jhdr[1024];
  int jhdr_len = 0;
  if (jpeg_q) {
    jpeg_scale_q(jq_y, jpeg_q_luma, jpeg_q);
    jpeg_scale_q(jq_c, jpeg_q_chroma, jpeg_q);
    jhdr_len = write_jpeg_header(jhdr, width, height, jq_y, jq_c);
  }
  FILE *f = fopen(out, "wb");
  if (!f || fwrite(hdr, 1, hdr_len, f) != (size_t)hdr_len) {
    perror(out);
    return 1;
  }

#define W(off, val) (regs[(off) / 4] = (val))
  /* The MMZ mapping is uncached: build each picture in normal memory and
   * copy it over, byte stores there take seconds per frame. */
  uint8_t *pic = vi_chn < 0 ? malloc(pic_size) : NULL;
  if (vi_chn < 0 && !pic) {
    perror("malloc");
    return 1;
  }
  uint64_t total = 0;
  int frame_num = 0, idr_pic_id = -1;
  for (int n = 0; n < frames; n++) {
    struct timespec t0, t1;
    /* The reconstruction of this frame is the reference of the next */
    int cur = n & 1, idr = n % gop == 0;

    if (idr) {
      frame_num = 0;
      idr_pic_id = (idr_pic_id + 1) % 16; /* consecutive IDRs differ */
    } else {
      frame_num = (frame_num + 1) & 0xffff; /* log2_max_frame_num = 16 */
    }

    uint32_t in_y, in_c;
    int fqp = rc.on ? (idr ? rc.qp_i : rc.qp_p) : qp;

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

    if (jpeg_q) {
      /* A JPEG job leaves the H.264-only registers alone, so clear the
       * window first (0x180 and up are the vendor's constants, kept). */
      for (int off = REG_IRQ; off < 0x180; off += 4)
        W(off, 0);
      W(REG_AXI, 0x0000d01f);
      W(REG_ADDR_OUT, strm.phys);
      W(REG_ADDR_IN_LUMA, in_y);
      W(REG_ADDR_IN_CB, in_c);
      W(REG_ADDR_IN_CR, in_c);
      /* row length, right fill in 4-pixel units, bottom fill, NV12 */
      W(REG_IN_IMG_CTRL, (stride << 12) | (((mbw * 16 - width) / 4) << 10) |
                             ((mbh * 16 - height) << 6) | (1 << 2));
      W(REG_STR_BUF_LIMIT, strm.size);
      W(REG_RGB2YUV1, 0x962b4c85);
      W(REG_RGB2YUV2, 0x90901d50);
      W(REG_RGB_MASK, 0x0000b694);
      /* The quantisation tables share 0x100-0x17c with other formats'
       * tables: select JPEG (not yet enabled) before writing them. */
      W(REG_ENC_CTRL, (1u << 31) | (mbw << 19) | (mbh << 10) | (1 << 3) |
                          (2 << 1));
      for (int i = 0; i < 16; i++) {
        const uint8_t *o = &jpeg_q_reg_order[i * 4];

        W(0x100 + i * 4, jq_y[o[0]] << 24 | jq_y[o[1]] << 16 |
                             jq_y[o[2]] << 8 | jq_y[o[3]]);
        W(0x140 + i * 4, jq_c[o[0]] << 24 | jq_c[o[1]] << 16 |
                             jq_c[o[2]] << 8 | jq_c[o[3]]);
      }
    } else {
      W(REG_ENC_CTRL, 0);
      W(REG_IRQ, 0);
      W(REG_AXI, 0x0000d01f); /* burst 16, clock gating, all swaps (vendor) */
      W(REG_ADDR_OUT, strm.phys);
      W(REG_ADDR_SIZE_TBL, 0);
      W(REG_ADDR_REF_LUMA, recon_y[!cur].phys);
      W(REG_ADDR_REF_CHROMA, recon_c[!cur].phys);
      W(REG_ADDR_REC_LUMA, recon_y[cur].phys);
      W(REG_ADDR_REC_CHROMA, recon_c[cur].phys);
      W(REG_ADDR_IN_LUMA, in_y);
      W(REG_ADDR_IN_CB, in_c);
      W(REG_ADDR_IN_CR, in_c + 1);
      /* row length, bottom overfill, input format 1 = NV12 */
      W(REG_IN_IMG_CTRL,
        (stride << 12) | ((mbh * 16 - height) << 6) | (1 << 2));
      /* pic_init_qp, idr_pic_id (P frames keep the last IDR's) */
      W(REG_ENC_CTRL0, ((uint32_t)qp << 26) | (idr_pic_id << 1));
      /* 8290: per-frame-type constant, vendor values */
      W(REG_ENC_CTRL1, chip->ctrl1[!idr]);
      /* deblocking mode as vendor, quarter-pel MV off, CABAC with
       * cabac_init_idc 0 or CAVLC, and (8290) frame_num */
      W(REG_ENC_CTRL2, 0xc0400000 | (cabac << 18) | frame_num);
      /* MV penalties and skip/inter favour, vendor values per frame type */
      W(REG_ENC_CTRL3, chip->ctrl3[!idr]);
      W(REG_ENC_CTRL4, chip->ctrl4[!idr]);
      W(REG_STR_HDR_REM_MSB, 0);
      W(REG_STR_HDR_REM_LSB, 0);
      W(REG_STR_BUF_LIMIT, (strm.size >> 3) & ~7u); /* as MPP's VEPU1 HAL */
      W(REG_MAD_CTRL, 0);
      /* fixed QP: init = min = max, no checkpoints */
      /* the frame's QP; the hardware writes slice_qp_delta against the
       * pic_init_qp in 0x040 */
      W(REG_QP_VAL, ((uint32_t)fqp << 26) | (fqp << 20) | (fqp << 14));
      for (int off = 0x070; off <= 0x090; off += 4)
        W(off, 0);
      W(REG_RLC_CTRL, 0); /* stream starts at bit 0 */
      W(REG_MB_CTRL, 0);
      W(REG_ADDR_CABAC_TBL, cabac ? cabac_tbl.phys : 0);
      W(REG_ADDR_MV_OUT, 0);
      W(REG_RGB2YUV1, 0x962b4c85);
      W(REG_RGB2YUV2, 0x90901d50);
      W(REG_RGB_MASK, 0x0000b694);
      /* intra area and ROIs outside the picture */
      /* left, right, top, bottom */
      uint32_t outside = (mbw << 24) | (mbw << 16) | (mbh << 8) | mbh;
      W(REG_INTRA_AREA, outside);
      W(REG_ROI1, outside);
      W(REG_ROI2, outside);
      W(REG_MVC, chip->mvc);
      /* 8290-only registers: constants the vendor writes for every job
       * (meaning unknown, see the top of this file); 0x1b8 differs per
       * frame type */
      for (int i = 0; i < 6; i++)
        W(0x180 + i * 4, chip->ext[!idr][i]);
      W(0x1b8, chip->r1b8[!idr]);
      W(0x1bc, chip->r1bc);
      W(0x1c0, chip->r1c0);
    }

    /* timeout IRQ, MB size, picture type (1 = IDR/intra), format (3 =
     * H.264, 2 = JPEG), enable */
    clock_gettime(CLOCK_MONOTONIC, &t0);
    W(REG_ENC_CTRL, (1u << 31) | (mbw << 19) | (mbh << 10) |
                        ((jpeg_q || idr) << 3) | ((jpeg_q ? 2 : 3) << 1) | 1);
    ret = wait_frame(fd_enc, regs);
    clock_gettime(CLOCK_MONOTONIC, &t1);
    irq = regs[REG_IRQ / 4];
    bits = regs[REG_STR_BUF_LIMIT / 4];
    if (!(irq & IRQ_FRAME_READY) ||
        (irq & (IRQ_BUS_ERROR | IRQ_RESET | IRQ_BUFFER_FULL | IRQ_TIMEOUT))) {
      fprintf(stderr, "frame %d: wait %d, irq 0x%08x, no frame:%s%s%s%s\n", n,
              ret, irq, irq & IRQ_BUS_ERROR ? " bus error" : "",
              irq & IRQ_BUFFER_FULL ? " buffer full" : "",
              irq & IRQ_TIMEOUT ? " timeout" : "",
              irq & IRQ_RESET ? " reset" : "");
      /* Stop the encoder before the exit handler gives a VI frame back:
       * an interrupted wait doesn't mean it stopped reading. */
      W(REG_ENC_CTRL, 0);
      W(REG_IRQ, 0);
      for (int i = 0; i < 100 && (regs[REG_ENC_CTRL / 4] & 1); i++)
        usleep(1000);
      return 1;
    }
    W(REG_IRQ, 0);
    /* The frame is done: the input picture can go back to VI */
    vi_release();

    uint32_t len = bits / 8;
    if (jpeg_q && fwrite(jhdr, 1, jhdr_len, f) != (size_t)jhdr_len) {
      perror(out);
      return 1;
    }
    if (fwrite(strm.virt, 1, len, f) != len) {
      perror(out);
      return 1;
    }
    total += len;
    if (rc.on)
      rc_update(&rc, idr, len * 8);
    printf("frame %d (%c) qp %d: irq 0x%08x, %u bytes, %ld us\n", n,
           jpeg_q ? 'J' : idr ? 'I' : 'P', jpeg_q ? 0 : fqp, irq, len,
           (long)((t1.tv_sec - t0.tv_sec) * 1000000 +
                  (t1.tv_nsec - t0.tv_nsec) / 1000));
  }
  if (fclose(f)) {
    perror(out);
    return 1;
  }
  vi_cleanup();
  if (jpeg_q)
    printf("wrote %s: %d JPEG pictures, %llu bytes (%dx%d, quality %d)\n", out,
           frames, (unsigned long long)total + frames * jhdr_len, width,
           height, jpeg_q);
  else
    printf("wrote %s: %d header + %llu slice bytes, %d frames (%dx%d, qp %d, "
           "gop %d, %s)\n",
           out, hdr_len, (unsigned long long)total, frames, width, height, qp,
           gop, cabac ? "CABAC" : "CAVLC");
  if (rc.on)
    printf("rate control: target %.0f kbps at %.2f fps, got %.0f kbps\n", kbps,
           rc.fps, total * 8.0 * rc.fps / frames / 1000);
  return 0;
}
