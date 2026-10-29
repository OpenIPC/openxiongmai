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

#include "board_prog.h"
#include "sns_tuning.h"
#include "xmv200.h"
#include "xmv200_priv.h"

extern const struct xmv200_prog xmv200_prog_x2w_wng;
extern const struct xmv200_prog xmv200_prog_x3_wq_b;

#define SYS 0x20000000
#define VI 0x30000000

/* the ISP's PLL. XM530V200 (/dev/mem): unlock, VCO and dividers, reset and
 * enable, lock, with waits between; p: the mode's 0x14, 0x44, 0x48, 0x50,
 * 0x54. XM535: the same through sys.ko (0x40085301 {register, value}, 0x10
 * first), then reading 0x10 back (0x40085302). */
static void isp_pll(const struct xmv200_chip *chip, int fd_sys,
                    const uint32_t *p) {
  static const uint32_t regs[] = {0x10, 0x44, 0x48, 0x50, 0x54};

  if (chip->sys_clocks) {
    uint32_t a[2];

    for (int i = 0; i < 5; i++) {
      a[0] = regs[i];
      a[1] = p[i];
      ioctl(fd_sys, 0x40085301, a);
    }
    a[0] = 0x10;
    a[1] = 0;
    ioctl(fd_sys, 0x40085302, a);
    return;
  }
  xmv200_reg_write(SYS, 1);
  xmv200_reg_write(SYS + 0x10, 0x010a8000);
  xmv200_reg_write(SYS + 0x14, p[0]);
  xmv200_reg_write(SYS + 0x44, p[1]);
  xmv200_reg_write(SYS + 0x48, p[2]);
  xmv200_reg_write(SYS + 0x50, p[3]);
  xmv200_reg_write(SYS + 0x54, p[4]);
  xmv200_reg_write(SYS + 0x10, 0x800a8000);
  usleep(100);
  xmv200_reg_write(SYS + 0x10, 0x810a8000);
  usleep(10000);
  xmv200_reg_write(SYS, 0);
  xmv200_reg_write(SYS + 0x50490, 1);
}

struct board {
  const char *name;
  /* a chip whose start-up is a board program (board_prog.h) */
  const char *chip;
  const struct xmv200_prog *prog;
  struct xmv200_isp_cfg isp;
  /* the encoded streams: VPSS channel n of the groups in groups goes to
   * VENC channel n, spliced top to bottom when there are several */
  int n_streams;
  struct {
    uint32_t width, height; /* per group */
    uint32_t stream_buf;
    uint32_t groups; /* mask */
  } stream[XMV200_MAX_STREAMS];
  uint32_t fps;
  /* a program board's sensors, for AE */
  int n_ae;
  struct xmv200_ae_sensor ae[3];
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
                        {.chn = 0, .x0 = 1, .y0 = 0, .out_width = 2304,
                         .out_height = 1296, .ldc = 1},
                        {.chn = 1, .mipi = 1, .x0 = 3, .y0 = 3, .out_width = 2304,
                         .out_height = 1296, .vi_idx = 1},
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
                .chn = {{.chn = 0, .x0 = 1, .y0 = 0, .out_width = 1920,
                         .out_height = 1080}},
                .sensor = {{.bus = 0, .addr = 0x30, .flip = 0x00}},
            },
        .n_streams = 2,
        .stream = {{1920, 1080, 0x13c680, 1}, {640, 360, 0xbb800, 1}},
        .fps = 12,
    },
    {
        /* XM535D1 X6E-WEQ: two SC2336 on their own receivers, spliced top
         * to bottom into 1920x2160, plus 640x720 */
        .name = "x6e-weq",
        .isp =
            {
                .tuning = &xmv200_tuning_sc2336_xm535,
                .n_chn = 2,
                .chn =
                    {
                        {.chn = 0, .x0 = 1, .y0 = 0, .out_width = 1920,
                         .out_height = 1080},
                        {.chn = 1, .x0 = 1, .y0 = 0, .out_width = 1920,
                         .out_height = 1080, .phy = 1, .vi_idx = 1},
                    },
                .sensor = {{.bus = 0, .addr = 0x30, .flip = 0x00},
                           {.bus = 0, .addr = 0x32, .flip = 0x00}},
            },
        .n_streams = 2,
        .stream = {{1920, 1080, 0x16c800, 3}, {640, 360, 0x38400, 3}},
        .fps = 12,
    },
    {
        /* XM535D2 X9-WE2Q: three SC2336, each its own picture, 1080p and
         * 360p. Two share receiver 0 and VI device 0 (on bus 2); the third,
         * camera 0, is on receiver 1 and VI device 1 (bus 0) */
        .name = "x9-we2q",
        .isp =
            {
                .tuning = &xmv200_tuning_sc2336_xm535,
                .n_chn = 3,
                .chn =
                    {
                        {.chn = 0, .x0 = 1, .y0 = 0, .out_width = 1920,
                         .out_height = 1080},
                        {.chn = 1, .x0 = 1, .y0 = 0, .out_width = 1920,
                         .out_height = 1080, .vi_idx = 1},
                        {.chn = 2, .x0 = 1, .y0 = 0, .out_width = 1920,
                         .out_height = 1080, .phy = 1, .vi_dev = 1},
                    },
                .sensor = {{.bus = 2, .addr = 0x30},
                           {.bus = 2, .addr = 0x32},
                           {.bus = 0, .addr = 0x30}},
            },
        /* VENC 2n and 2n + 1 are camera n's */
        .n_streams = 6,
        .stream = {{1920, 1080, 0x7e900, 4},
                   {640, 360, 0x38400, 4},
                   {1920, 1080, 0x7e900, 2},
                   {640, 360, 0x38400, 2},
                   {1920, 1080, 0x7e900, 1},
                   {640, 360, 0x38400, 1}},
        .fps = 12,
    },
};

/* program boards: the chip, the program and what VENC gets */
static const struct board prog_boards[] = {
    {
        /* XM535W1 X2W-WNG: one SC1346, its 720p encoded at 1080p, and
         * 800x448 */
        .name = "x2w-wng",
        .chip = "xm535w1",
        .prog = &xmv200_prog_x2w_wng,
        .n_streams = 2,
        .stream = {{1920, 1080, 0, 1}, {800, 448, 0, 1}},
        .fps = 12,
        .n_ae = 1,
        .ae = {{0, 0x30, 0, 0, 0, 1}},
    },
    {
        /* XM533 X3-WQ-B: one SC2337P, 2304x1296 and 800x448 */
        .name = "x3-wq-b",
        .chip = "xm533",
        .prog = &xmv200_prog_x3_wq_b,
        .n_streams = 2,
        .stream = {{2304, 1296, 0, 1}, {800, 448, 0, 1}},
        .fps = 12,
        .n_ae = 1,
        .ae = {{0, 0x30, 0, 0, 0, 1}},
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

/*
 * "auto": the SmartSens sensors on bus 0 at 0x30 and 0x32 (their chip ID
 * at 0x3107/0x3108) and the tuning for them. Two of a kind are spliced top
 * to bottom, as the X6C-WEQ's; one gives its own picture. Each stream has
 * a 640-wide sub stream beside it.
 */
static int chip_id(int bus, uint8_t addr, uint32_t *id) {
  uint32_t hi, lo;

  if (xmv200_i2c_read(bus, addr, 0x3107, 2, &hi, 1) ||
      xmv200_i2c_read(bus, addr, 0x3108, 2, &lo, 1))
    return -1;
  *id = (hi & 0xff) << 8 | (lo & 0xff);
  return 0;
}

static const struct xmv200_sns *sensor_named(const char *name) {
  for (int i = 0; i < xmv200_n_sensors; i++)
    if (!strcmp(xmv200_sensors[i].name, name))
      return &xmv200_sensors[i];
  return NULL;
}

static const struct xmv200_sns *probe(int bus, uint8_t addr) {
  uint32_t id, ext;

  if (chip_id(bus, addr, &id))
    return NULL;
  /* SC2336, SC2336P and SC2337P all read 0xcb3a in standby: streaming,
   * the P parts read 0x9b3a, and 0x801e tells them apart (the vendor ISP's
   * test) */
  if (id == 0xcb3a || id == 0x9b3a) {
    const char *name = "SC2336";

    xmv200_i2c_write(bus, addr, 0x0100, 2, 1, 1);
    usleep(3000);
    if (!chip_id(bus, addr, &id) && id == 0x9b3a &&
        !xmv200_i2c_read(bus, addr, 0x801e, 2, &ext, 1))
      name = ext & 0xf ? "SC2337P" : "SC2336P";
    xmv200_i2c_write(bus, addr, 0x0100, 2, 0, 1);
    if (sensor_named(name))
      return sensor_named(name);
    fprintf(stderr, "libxmv200: %s at 0x%02x: no tuning\n", name, addr);
    return NULL;
  }
  for (int i = 0; i < xmv200_n_sensors; i++)
    if (xmv200_sensors[i].chip_id == id)
      return &xmv200_sensors[i];
  fprintf(stderr, "libxmv200: sensor %04x at 0x%02x: no tuning\n", id, addr);
  return NULL;
}

static int auto_board(struct board *b) {
  static const uint8_t addrs[] = {0x30, 0x32};
  const struct xmv200_sns *s[2] = {NULL, NULL};
  int n = 0;

  /* the sensors answer only once they are clocked */
  xmv200_reg_write(SYS, 1);
  xmv200_reg_write(SYS + 0x74, 0xc104c103);
  xmv200_reg_write(SYS + 0x78, 0xc105c0ff);
  xmv200_reg_write(SYS, 0);
  usleep(10000);
  for (int i = 0; i < 2; i++)
    if ((s[n] = probe(0, addrs[i])) != NULL) {
      b->isp.sensor[n] = (struct xmv200_isp_sensor){0, addrs[i], 0};
      n++;
    }
  if (!n || (n == 2 && s[0] != s[1]))
    return -1;
  const struct xmv200_sns_tuning *t = s[0]->tuning;
  uint32_t w = t->width & ~0xfu, h = t->height & ~0x7u;

  memset(&b->isp.chn, 0, sizeof(b->isp.chn));
  b->name = "auto";
  b->isp.tuning = t;
  b->isp.n_chn = n;
  for (int c = 0; c < n; c++)
    b->isp.chn[c] = (struct xmv200_isp_chn){
        .chn = c, .mipi = c, .x0 = 1, .y0 = 0, .out_width = w,
        .out_height = h, .ldc = 0, .vi_idx = c};
  if (n == 2)
    b->isp.sensor[1].flip = 0x66; /* the second lens is mounted turned */
  b->n_streams = 2;
  b->stream[0].width = w;
  b->stream[0].height = h;
  b->stream[0].stream_buf = w * h * n / 4 + 0x10000;
  b->stream[0].groups = (1u << n) - 1;
  b->stream[1].width = 640;
  b->stream[1].height = (640 * h / w) & ~0x7u;
  b->stream[1].stream_buf = 0xbb800;
  b->stream[1].groups = (1u << n) - 1;
  b->fps = 12;
  fprintf(stderr, "libxmv200: %d x %s, %ux%u\n", n, s[0]->name, w, h * n);
  return 0;
}

int xmv200_board_start(const char *name, struct xmv200_board_info *info) {
  static struct board detected;
  const struct board *b = NULL;
  int fd_vpss, fd_sys, fd_venc, fd_rgn, fd_hevc, fd_vb, n;
  const struct xmv200_chip *chip;
  uint32_t reset;

  if (!strcmp(name, "auto")) {
    if (auto_board(&detected))
      return -1;
    b = &detected;
  }
  for (unsigned i = 0; !b && i < sizeof(boards) / sizeof(boards[0]); i++)
    if (!strcmp(boards[i].name, name))
      b = &boards[i];
  for (unsigned i = 0; !b && i < sizeof(prog_boards) / sizeof(prog_boards[0]);
       i++)
    if (!strcmp(prog_boards[i].name, name))
      b = &prog_boards[i];
  if (!b)
    return -1;
  if (b->prog) {
    xmv200_venc_set_engine(b->chip);
    if (xmv200_prog_run(b->prog) || xmv200_isp_frames_start())
      return -1;
    xmv200_ae_init_prog(b->ae, b->n_ae, b->prog);
    if (info) {
      memset(info, 0, sizeof(*info));
      info->n_streams = b->n_streams;
      info->fps = b->fps;
      for (int s = 0; s < b->n_streams; s++) {
        info->stream[s].width = b->stream[s].width;
        info->stream[s].height = b->stream[s].height;
      }
    }
    return 0;
  }
  n = b->isp.n_chn;
  chip = xmv200_chip_of(b->isp.tuning);
  xmv200_venc_set_engine(chip->engine);
  reset = chip->hevc_reset;

  fd_vpss = open_dev("/dev/vpss");
  fd_sys = open_dev("/dev/sys");
  fd_venc = open_dev("/dev/venc_info");
  fd_rgn = open_dev("/dev/rgn");
  fd_hevc = open_dev("/dev/h265");
  fd_vb = open_dev("/dev/vb");
  if (fd_vpss < 0 || fd_sys < 0 || fd_venc < 0 || fd_hevc < 0)
    return -1;

  /* the system: nothing allocates before the MMZ reset */
  if (xmv200_mmz_reset() || ioctl(fd_vpss, VPSS_INIT, 0) < 0)
    return -1;
  if (fd_rgn >= 0)
    ioctl(fd_rgn, 0x5200, 0);
  ioctl(fd_hevc, 0x40044806, &reset);

  /* VB: what VPSS writes (each stream, all groups in one block) and, on
   * the XM530V200, the ISP's output to VPSS */
  struct xmv200_vb_pool pools[XMV200_MAX_STREAMS + 1];
  int np = 0;
  uint32_t one = 1;

  if (chip->vb_mode && fd_vb >= 0)
    ioctl(fd_vb, 0x40044205, &one);
  for (int s = 0; s < b->n_streams; s++)
    pools[np++] = (struct xmv200_vb_pool){
        b->stream[s].width * b->stream[s].height *
            popcount(b->stream[s].groups) * 3 / 2,
        chip->vb_count};
  if (chip->isp_pool)
    pools[np++] = (struct xmv200_vb_pool){
        (b->isp.tuning->width & ~0xfu) *
            ((b->isp.tuning->height + 15) & ~15u) * 3 / 2,
        3};
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
    for (int s = 0, vc = 0; s < b->n_streams; s++) {
      /* stream s is the group's VPSS channel vc, and VENC channel s */
      uint32_t attr[5] = {2, b->stream[s].width, b->stream[s].height,
                          b->fps, b->fps};
      uint32_t splice[2] = {g + 1, s};
      uint32_t bind[3] = {MOD_VENC, 0, s};

      if (!(b->stream[s].groups & 1u << g))
        continue;
      if (vpss(fd_vpss, VPSS_SET_CHN_ATTR, g, vc, attr, 5) ||
          (popcount(b->stream[s].groups) > 1 &&
           vpss(fd_vpss, VPSS_SET_SPLICE, g, vc, splice, 2)) ||
          vpss(fd_vpss, VPSS_ENABLE_CHN, g, vc, NULL, 0) ||
          vpss(fd_vpss, VPSS_BIND, g, vc, bind, 3) ||
          sys_bind(fd_sys, MOD_VPSS, g, vc, MOD_VENC, 0, s) < 0)
        return -1;
      vc++;
    }

  /* venc.ko: its channels and their stream buffers */
  ioctl(fd_venc, 0x4900, 0);
  for (int s = 0; s < b->n_streams; s++) {
    uint32_t cfg[65], h = b->stream[s].height * popcount(b->stream[s].groups);

    memset(cfg, 0, sizeof(cfg));
    cfg[0] = s;
    cfg[1] = 1;
    cfg[2] = cfg[5] = 0x109; /* H.265 */
    cfg[3] = b->stream[s].width;
    cfg[4] = h;
    cfg[8] = b->stream[s].stream_buf;
    if (chip->venc_v2) {
      cfg[10] = 1;
      cfg[41] = b->stream[s].width; /* the encoded size */
      cfg[42] = h;
    } else {
      cfg[10] = 0xf;
      cfg[22] = 1; /* +0x58 starts it */
    }
    if (ioctl(fd_venc, 0x41044901, cfg) < 0)
      return -1;
  }

  /* clocks: the sensors' (XM530V200; the XM535's module loader sets them),
   * the ISP out of reset, the ISP's PLL */
  if (!chip->sys_clocks) {
    xmv200_reg_write(SYS, 1);
    xmv200_reg_write(SYS + 0x74, b->isp.tuning->sns_clk[0]);
    xmv200_reg_write(SYS + 0x78, b->isp.tuning->sns_clk[1]);
    xmv200_reg_write(SYS, 0);
  }
  xmv200_reg_write(SYS + 0x11c, 1);
  isp_pll(chip, fd_sys, b->isp.tuning->pll[0]);

  /* the sensors out of reset, each receiver's setup, each channel's VI */
  for (int c = 0; c < n; c++) {
    const struct xmv200_isp_sensor *s = &b->isp.sensor[c];

    xmv200_i2c_write(s->bus, s->addr, 0x0103, 2, 1, 1);
    xmv200_i2c_write(s->bus, s->addr, 0x0103, 2, 1, 1);
    xmv200_i2c_write(s->bus, s->addr, 0x0100, 2, 1, 1);
  }
  for (int c = 0; c < n; c++) {
    const struct xmv200_isp_chn *ch = &b->isp.chn[c];
    int first = 1;

    for (int k = 0; k < c; k++)
      if (b->isp.chn[k].phy == ch->phy)
        first = 0;
    for (int i = 0; first && i < b->isp.tuning->n_mipi; i++)
      xmv200_reg_write(b->isp.tuning->mipi[i].addr + ch->phy * 0x1000,
                       b->isp.tuning->mipi[i].val);
    /* XM535: the channel's VI device and its place there */
    xmv200_reg_write(VI, !chip->vi_mode ? 2
                         : ch->vi_dev   ? 6
                         : ch->vi_idx   ? 0xa
                                        : 2);
    xmv200_reg_write(VI + 0x18, chip->vi_mode ? c > 0 : c);
  }
  isp_pll(chip, fd_sys, b->isp.tuning->pll[1]);

  if (xmv200_isp_start(&b->isp))
    return -1;

  /* AE: each sensor in the first stream with its group, as its band */
  struct xmv200_ae_sensor where[3];

  for (int c = 0; c < n && c < 3; c++) {
    where[c] = (struct xmv200_ae_sensor){
        b->isp.sensor[c].bus, b->isp.sensor[c].addr, b->isp.chn[c].chn, 0, 0,
        1};
    for (int s = 0; s < b->n_streams; s++)
      if (b->stream[s].groups & 1u << c) {
        where[c].stream = s;
        where[c].band = popcount(b->stream[s].groups & ((1u << c) - 1));
        where[c].n_bands = popcount(b->stream[s].groups);
        break;
      }
  }
  xmv200_ae_init(&b->isp, where, chip->frame_answer);

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
