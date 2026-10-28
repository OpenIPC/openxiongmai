// SPDX-License-Identifier: MIT
/*
 * Boards: how sensors, VPSS and VENC are laid out, and the start-up the
 * vendor userspace does for it, in its order:
 *
 *   MMZ reset, VPSS/RGN init, HEVC reset, VB pools
 *   VPSS groups (bound to the ISP), their channels, the splice, VENC binds
 *   venc.ko channels
 *   sensor clocks, ISP reset, ISP clocks, sensor reset, MIPI receiver
 *   the ISP (isp.c)
 */
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "sns_tuning.h"
#include "xmv200.h"
#include "xmv200_priv.h"

struct reg {
  uint32_t addr, val;
};

#define SYS 0x20000000
#define VI 0x30000000

/* the ISP's PLL (the vendor's clock setup for 1080p30-class sensors):
 * unlock, VCO and dividers, reset and enable, lock; waits between */
static void isp_pll(uint32_t vco, uint32_t d44, uint32_t d48, uint32_t d50,
                    uint32_t d54) {
  xmv200_reg_write(SYS, 1);
  xmv200_reg_write(SYS + 0x10, 0x010a8000);
  xmv200_reg_write(SYS + 0x14, vco);
  xmv200_reg_write(SYS + 0x44, d44);
  xmv200_reg_write(SYS + 0x48, d48);
  xmv200_reg_write(SYS + 0x50, d50);
  xmv200_reg_write(SYS + 0x54, d54);
  xmv200_reg_write(SYS + 0x10, 0x800a8000);
  usleep(100);
  xmv200_reg_write(SYS + 0x10, 0x810a8000);
  usleep(10000);
  xmv200_reg_write(SYS, 0);
  xmv200_reg_write(SYS + 0x50490, 1);
}

/* the MIPI combo receiver: two sensors, two lanes each, RAW10 */
static const struct reg mipi_2x2lane[] = {
    {VI + 0x402c, 0},          {VI + 0x4008, 0x2b010044},
    {VI + 0x400c, 0x001600b8}, {VI + 0x4010, 0},
    {VI + 0x4004, 0x00080404}, {VI + 0x4094, 0x3ff},
    {VI + 0x409c, 0},          {VI + 0x40a0, 1},
    {VI + 0x40a4, 0},          {VI + 0x40b4, 0},
    {VI + 0x40b8, 1},          {VI + 0x40c8, 0},
    {VI + 0x40cc, 0},          {VI + 0x40e4, 0x38},
    {VI + 0x40e8, 1},          {VI + 0x40ec, 0},
    {VI + 0x41bc, 0},          {VI + 0x4240, 1},
    {VI + 0x4244, 0x440},      {VI + 0x4248, 1},
    {VI + 0x424c, 0x440},      {VI + 0x423c, 0},
    {VI + 0x4300, 0xf},        {VI + 0x4304, 0},
    {VI + 0x4308, 0xf},        {VI + 0x430c, 0x21},
    {VI + 0x4310, 0x21},
};

struct board {
  const char *name;
  struct xmv200_isp_cfg isp;
  /* the encoded streams: VPSS channel n of the groups in groups goes to
   * VENC channel n, spliced top to bottom when there are several */
  int n_streams;
  struct {
    uint32_t width, height; /* per group */
    uint32_t stream_buf;
    uint32_t groups; /* mask */
  } stream[2];
  uint32_t fps;
};

static const struct board boards[] = {
    {
        /* XM530V200 X6C-WEQ: two SC2336 one above the other, spliced into
         * one 2304x2592 picture */
        .name = "x6c-weq",
        .isp =
            {
                .tuning = &xmv200_tuning_sc2336,
                .n_chn = 2,
                .chn =
                    {
                        {.chn = 0, .x0 = 1, .y0 = 0, .width = 1930,
                         .height = 1080, .hts = 2640, .frame_lines = 2250,
                         .exposure = 750, .out_width = 2304,
                         .out_height = 1296, .ldc = 1},
                        {.chn = 1, .mipi = 1, .x0 = 3, .y0 = 3, .width = 1930,
                         .height = 1080, .hts = 2640, .frame_lines = 2250,
                         .exposure = 750, .out_width = 2304,
                         .out_height = 1296},
                    },
                .sensor = {{.bus = 0, .addr = 0x30, .flip = 0x00},
                           {.bus = 0, .addr = 0x32, .flip = 0x66}},
            },
        .n_streams = 2,
        .stream = {{2304, 1296, 0x38f400, 3}, {640, 360, 0xbb800, 3}},
        .fps = 12,
    },
    {
        /* XM530V200 X2-WQ2 (dual-lens zoom): the wide lens' SC2336 alone,
         * 1080p and 360p */
        .name = "x2-wq2",
        .isp =
            {
                .tuning = &xmv200_tuning_sc2336,
                .n_chn = 1,
                .chn = {{.chn = 0, .x0 = 1, .y0 = 0, .width = 1930,
                         .height = 1080, .hts = 2640, .frame_lines = 2250,
                         .exposure = 750, .out_width = 1920,
                         .out_height = 1080}},
                .sensor = {{.bus = 0, .addr = 0x30, .flip = 0x00}},
            },
        .n_streams = 2,
        .stream = {{1920, 1080, 0x13c680, 1}, {640, 360, 0xbb800, 1}},
        .fps = 12,
    },
};

static int popcount(uint32_t m) {
  int n = 0;

  for (; m; m >>= 1)
    n += m & 1;
  return n;
}

/* vpss.ko: one 128-byte record, {grp, chn, -, -, fields from +0x10} */
struct vpss_rec {
  uint32_t grp, chn, rsvd[2];
  uint32_t f[28];
};

#define VPSS_INIT 0x5000
#define VPSS_CREATE_GRP 0x40805001
#define VPSS_START_GRP 0x40045008
#define VPSS_SET_CHN_ATTR 0x40805004
#define VPSS_ENABLE_CHN 0x40805006
#define VPSS_SET_SPLICE 0x40805012
#define VPSS_BIND 0x4080500a

#define MOD_VPSS 8
#define MOD_VENC 7

/* sys.ko keeps the whole bind table: {src{mod,dev,chn}, dst{mod,dev,chn}} */
static uint32_t binds[64][6];

static int sys_bind(int fd, uint32_t src_mod, uint32_t src_dev,
                    uint32_t src_chn, uint32_t dst_mod, uint32_t dst_dev,
                    uint32_t dst_chn) {
  for (int i = 0; i < 64; i++)
    if (!binds[i][0]) {
      uint32_t e[6] = {src_mod, src_dev, src_chn, dst_mod, dst_dev, dst_chn};

      memcpy(binds[i], e, sizeof(e));
      return ioctl(fd, 0x400c5300, binds);
    }
  return -1;
}

static int vpss(int fd, uint32_t cmd, uint32_t grp, uint32_t chn,
                const uint32_t *f, int n) {
  struct vpss_rec r;

  memset(&r, 0, sizeof(r));
  r.grp = grp;
  r.chn = chn;
  if (n)
    memcpy(r.f, f, n * 4);
  if (ioctl(fd, cmd, &r) < 0) {
    fprintf(stderr, "libxmv200: vpss %#x grp %u chn %u failed\n", cmd, grp,
            chn);
    return -1;
  }
  return 0;
}

static int open_dev(const char *path) {
  int fd = open(path, O_RDWR);

  if (fd < 0)
    fprintf(stderr, "libxmv200: %s: no device\n", path);
  return fd;
}

int xmv200_board_start(const char *name, struct xmv200_board_info *info) {
  const struct board *b = NULL;
  int fd_vpss, fd_sys, fd_venc, fd_rgn, fd_hevc, n;
  uint32_t reset = 0x10002;

  for (unsigned i = 0; i < sizeof(boards) / sizeof(boards[0]); i++)
    if (!strcmp(boards[i].name, name))
      b = &boards[i];
  if (!b)
    return -1;
  n = b->isp.n_chn;

  fd_vpss = open_dev("/dev/vpss");
  fd_sys = open_dev("/dev/sys");
  fd_venc = open_dev("/dev/venc_info");
  fd_rgn = open_dev("/dev/rgn");
  fd_hevc = open_dev("/dev/h265");
  if (fd_vpss < 0 || fd_sys < 0 || fd_venc < 0 || fd_hevc < 0)
    return -1;

  /* the system: nothing allocates before the MMZ reset */
  if (xmv200_mmz_reset() || ioctl(fd_vpss, VPSS_INIT, 0) < 0)
    return -1;
  if (fd_rgn >= 0)
    ioctl(fd_rgn, 0x5200, 0);
  ioctl(fd_hevc, 0x40044806, &reset);

  /* VB: what VPSS writes (each stream, all groups in one block) and the
   * ISP's output to VPSS */
  struct xmv200_vb_pool pools[3];
  int np = 0;

  for (int s = 0; s < b->n_streams; s++)
    pools[np++] = (struct xmv200_vb_pool){
        b->stream[s].width * b->stream[s].height *
            popcount(b->stream[s].groups) * 3 / 2,
        3};
  pools[np++] = (struct xmv200_vb_pool){
      1920 * ((b->isp.chn[0].height + 15) & ~15u) * 3 / 2, 3};
  if (xmv200_vb_init(pools, np))
    return -1;

  /* VPSS: a group per sensor, fed by the ISP; its channels spliced top to
   * bottom into the streams and bound to VENC */
  for (int g = 0; g < n; g++) {
    uint32_t grp[7] = {4096, 2160, 0, 0, 0, 0, 1};

    if (vpss(fd_vpss, VPSS_CREATE_GRP, g, 0, grp, 7) ||
        ioctl(fd_vpss, VPSS_START_GRP, &g) < 0)
      return -1;
  }
  for (int g = 0; g < n; g++)
    for (int s = 0; s < b->n_streams; s++) {
      uint32_t attr[5] = {2, b->stream[s].width, b->stream[s].height,
                          b->fps, b->fps};
      uint32_t splice[2] = {g + 1, s};
      uint32_t bind[3] = {MOD_VENC, 0, s};

      if (!(b->stream[s].groups & 1u << g))
        continue;
      if (vpss(fd_vpss, VPSS_SET_CHN_ATTR, g, s, attr, 5) ||
          (popcount(b->stream[s].groups) > 1 &&
           vpss(fd_vpss, VPSS_SET_SPLICE, g, s, splice, 2)) ||
          vpss(fd_vpss, VPSS_ENABLE_CHN, g, s, NULL, 0) ||
          vpss(fd_vpss, VPSS_BIND, g, s, bind, 3) ||
          sys_bind(fd_sys, MOD_VPSS, g, s, MOD_VENC, 0, s) < 0)
        return -1;
    }

  /* venc.ko: its channels and their stream buffers; +0x58 starts them */
  ioctl(fd_venc, 0x4900, 0);
  for (int s = 0; s < b->n_streams; s++) {
    uint32_t cfg[65];

    memset(cfg, 0, sizeof(cfg));
    cfg[0] = s;
    cfg[1] = 1;
    cfg[2] = cfg[5] = 0x109; /* H.265 */
    cfg[3] = b->stream[s].width;
    cfg[4] = b->stream[s].height * popcount(b->stream[s].groups);
    cfg[8] = b->stream[s].stream_buf;
    cfg[10] = 0xf;
    cfg[22] = 1;
    if (ioctl(fd_venc, 0x41044901, cfg) < 0)
      return -1;
  }

  /* clocks: the sensors', the ISP out of reset, the ISP's PLL */
  xmv200_reg_write(SYS, 1);
  xmv200_reg_write(SYS + 0x74, 0xc104c103);
  xmv200_reg_write(SYS + 0x78, 0xc105c0ff);
  xmv200_reg_write(SYS, 0);
  xmv200_reg_write(SYS + 0x11c, 1);
  isp_pll(0x20b60000, 0x0d620d62, 0x051f051f, 0x051f0000, 0x051f0509);

  /* the sensors out of reset, the receiver */
  for (int c = 0; c < n; c++) {
    const struct xmv200_isp_sensor *s = &b->isp.sensor[c];

    xmv200_i2c_write(s->bus, s->addr, 0x0103, 2, 1, 1);
    xmv200_i2c_write(s->bus, s->addr, 0x0103, 2, 1, 1);
    xmv200_i2c_write(s->bus, s->addr, 0x0100, 2, 1, 1);
    if (c == 0)
      for (unsigned i = 0; i < sizeof(mipi_2x2lane) / sizeof(mipi_2x2lane[0]);
           i++)
        xmv200_reg_write(mipi_2x2lane[i].addr, mipi_2x2lane[i].val);
    xmv200_reg_write(VI, 2);
    xmv200_reg_write(VI + 0x18, c);
  }
  isp_pll(0x24848000, 0x0d410d41, 0x05090517, 0x05170000, 0x05090505);

  if (xmv200_isp_start(&b->isp))
    return -1;
  xmv200_ae_init(&b->isp);

  if (info) {
    memset(info, 0, sizeof(*info));
    info->n_streams = b->n_streams;
    info->fps = b->fps;
    for (int s = 0; s < b->n_streams; s++) {
      info->stream[s].width = b->stream[s].width;
      info->stream[s].height =
          b->stream[s].height * popcount(b->stream[s].groups);
    }
  }
  return 0;
}
