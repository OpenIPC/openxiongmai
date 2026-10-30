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
#include <strings.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include "sns_tuning.h"
#include "xmv200.h"
#include "xmv200_priv.h"


#define SYS 0x20000000
#define VI 0x30000000

/* the ISP's PLL, as the mode's program writes it: through /dev/mem with the
 * clock block unlocked and a wait after each control write (0x10), or
 * through sys.ko (0x40085301 {register, value}), then reading 0x10 back
 * (0x40085302) */
static void isp_pll(const struct xmv200_chip *chip, int fd_sys,
                    const struct xmv200_reg *p, int n) {
  if (chip->sys_clocks) {
    uint32_t a[2];

    for (int i = 0; i < n; i++) {
      a[0] = p[i].addr;
      a[1] = p[i].val;
      ioctl(fd_sys, 0x40085301, a);
    }
    a[0] = 0x10;
    a[1] = 0;
    ioctl(fd_sys, 0x40085302, a);
    return;
  }
  xmv200_reg_write(SYS, 1);
  for (int i = 0; i < n; i++) {
    xmv200_reg_write(SYS + p[i].addr, p[i].val);
    if (p[i].addr == 0x10)
      usleep(10000);
  }
  xmv200_reg_write(SYS, 0);
  if (chip->isp_pool) /* XM530V200 */
    xmv200_reg_write(SYS + 0x50490, 1);
}

struct board {
  const char *name;
  struct xmv200_isp_cfg isp;
  /* the encoded streams: VPSS channel n of the groups in groups goes to
   * VENC channel n, spliced top to bottom when there are several (without
   * VPSS, VI channel n scales the ISP's picture for it) */
  int n_streams;
  struct {
    uint32_t width, height; /* per group */
    uint32_t stream_buf;
    uint32_t groups; /* mask */
  } stream[XMV200_MAX_STREAMS];
  uint32_t fps;
};

/* the boards there are layouts for */
static const struct {
  const char *name;
  struct xmv200_layout l;
} boards[] = {
    {
        /* XM530V200 X6C-WEQ: two SC2336 one above the other, spliced into
         * one 2304x2592 picture */
        "x6c-weq",
        {.chip = "xm530v200",
         .n_sensors = 2,
         .sensor = {{"SC2336", 0, 0x30, 0x00, 1, 0, 2304, 1296, .ldc = 1},
                    {"SC2336", 0, 0x32, 0x66, 3, 3, 2304, 1296, .vi_idx = 1,
                     .mipi = 1}},
         .n_streams = 2,
         .stream = {{2304, 1296, 3}, {640, 360, 3}},
         .fps = 12},
    },
    {
        /* XM530V200 X2-WQ2 (dual-lens zoom): the wide lens' SC2336 alone,
         * 1080p and 360p */
        "x2-wq2",
        {.chip = "xm530v200",
         .n_sensors = 1,
         .sensor = {{"SC2336", 0, 0x30, 0x00, 1, 0, 1920, 1080}},
         .n_streams = 2,
         .stream = {{1920, 1080, 1}, {640, 360, 1}},
         .fps = 12},
    },
    {
        /* XM535D1 X6E-WEQ: two SC2336 on their own receivers, spliced top
         * to bottom into 1920x2160, plus 640x720 */
        "x6e-weq",
        {.chip = "xm535d1",
         .n_sensors = 2,
         .sensor = {{"SC2336", 0, 0x30, 0x00, 1, 0, 1920, 1080},
                    {"SC2336", 0, 0x32, 0x00, 1, 0, 1920, 1080, .phy = 1,
                     .vi_idx = 1}},
         .n_streams = 2,
         .stream = {{1920, 1080, 3}, {640, 360, 3}},
         .fps = 12},
    },
    {
        /* XM535D2 X9-WE2Q: three SC2336, each its own picture, 1080p and
         * 360p. Two share receiver 0 and VI device 0 (on bus 2); the third,
         * camera 0, is on receiver 1 and VI device 1 (bus 0). VENC 2n and
         * 2n + 1 are camera n's */
        "x9-we2q",
        {.chip = "xm535d2",
         .n_sensors = 3,
         .sensor = {{"SC2336", 2, 0x30, 0x00, 1, 0, 1920, 1080},
                    {"SC2336", 2, 0x32, 0x00, 1, 0, 1920, 1080, .vi_idx = 1},
                    {"SC2336", 0, 0x30, 0x00, 1, 0, 1920, 1080, .phy = 1,
                     .vi_dev = 1}},
         .n_streams = 6,
         .stream = {{1920, 1080, 4},
                    {640, 360, 4},
                    {1920, 1080, 2},
                    {640, 360, 2},
                    {1920, 1080, 1},
                    {640, 360, 1}},
         .fps = 12},
    },
    {
        /* XM535W1 X2W-WNG: one SC1346 (720p); VI makes 720p and 800x448 */
        "x2w-wng",
        {.chip = "xm535w1",
         .n_sensors = 1,
         .sensor = {{"SC1346", 0, 0x30, 0x00, 1, 0, 1280, 720}},
         .n_streams = 2,
         .stream = {{1280, 720, 1}, {800, 448, 1}},
         .fps = 12},
    },
    {
        /* XM533 X3-WQ-B: one SC2337P; VI makes 2304x1296 and 800x448 */
        "x3-wq-b",
        {.chip = "xm533",
         .n_sensors = 1,
         .sensor = {{"SC2337P", 0, 0x30, 0x00, 1, 0, 1920, 1086}},
         .n_streams = 2,
         .stream = {{2304, 1296, 1}, {800, 448, 1}},
         .fps = 12},
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
#define MOD_VI 0x10

/* vi.ko (no VPSS): its channels scale the ISP's output; 80-byte records
 * {-, chn, ...} */
#define VI_INIT 0x5600
#define VI_SET_CHN_ATTR 0x40505602 /* +0x10: x, y, in w, h, format, out w,
                                    * h, -, -, source fps, fps */
#define VI_ENABLE_CHN 0x40045603   /* {chn} */
#define VI_BIND 0x40505605         /* +0x10: {module, device, channel} */

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

/* the running chip, from the kernel's machine name */
const char *xmv200_chip_name(void) {
  static char name[32];
  char line[128];
  FILE *f;

  if (name[0])
    return name;
  f = fopen("/proc/cpuinfo", "r");
  while (f && fgets(line, sizeof(line), f)) {
    char *m = strstr(line, "xm5");

    if (strncmp(line, "Hardware", 8) || !m)
      continue;
    for (int i = 0; i < (int)sizeof(name) - 1 && m[i] > ' '; i++)
      name[i] = m[i];
    break;
  }
  if (f)
    fclose(f);
  return name[0] ? name : NULL;
}

/* the tuning family of a chip: the XM535D1 and D2 share theirs */
static const char *family_of(const char *chip) {
  if (!chip)
    return "xm530v200";
  if (!strcmp(chip, "xm535d1") || !strcmp(chip, "xm535d2"))
    return "xm535";
  return chip;
}

static const struct xmv200_sns *sensor_named(const char *name,
                                             const char *family) {
  for (int i = 0; i < xmv200_n_sensors; i++)
    if (!strcasecmp(xmv200_sensors[i].name, name) &&
        !strcmp(xmv200_sensors[i].tuning->chip, family))
      return &xmv200_sensors[i];
  return NULL;
}

/*
 * Probing: a SmartSens sensor's chip ID at 0x3107/0x3108, and the tuning
 * for it on this chip family.
 */
static int chip_id(int bus, uint8_t addr, uint32_t *id) {
  uint32_t hi, lo;

  if (xmv200_i2c_read(bus, addr, 0x3107, 2, &hi, 1) ||
      xmv200_i2c_read(bus, addr, 0x3108, 2, &lo, 1))
    return -1;
  *id = (hi & 0xff) << 8 | (lo & 0xff);
  return 0;
}

static const struct xmv200_sns *registered(uint32_t id, const char *family) {
  for (int i = 0; i < xmv200_n_sensors; i++)
    if (xmv200_sensors[i].chip_id == id &&
        !strcmp(xmv200_sensors[i].tuning->chip, family))
      return &xmv200_sensors[i];
  return NULL;
}

static const struct xmv200_sns *probe(int bus, uint8_t addr,
                                      const char *family) {
  uint32_t id, ext;

  /* not a SmartSens part: the drivers' probes (the registry has the chip
   * ID the driver gives) */
  if (chip_id(bus, addr, &id) || !id || id == 0xffff) {
    if (!(id = xmv200_sns_probe(bus, addr, family)))
      return NULL;
    if (!registered(id, family))
      fprintf(stderr, "libxmv200: sensor %04x at %d-%02x: no %s tuning\n",
              id, bus, addr, family);
    return registered(id, family);
  }
  /* SC2336, SC2336P and SC2337P all read 0xcb3a in standby: streaming,
   * the P parts read 0x9b3a, and 0x801e tells them apart (the vendor ISP's
   * test) */
  if (id == 0xcb3a || id == 0x9b3a) {
    const char *name = "SC2336";
    uint32_t sid;

    xmv200_i2c_write(bus, addr, 0x0100, 2, 1, 1);
    usleep(3000);
    if (!chip_id(bus, addr, &sid) && sid == 0x9b3a &&
        !xmv200_i2c_read(bus, addr, 0x801e, 2, &ext, 1))
      name = ext & 0xf ? "SC2337P" : "SC2336P";
    xmv200_i2c_write(bus, addr, 0x0100, 2, 0, 1);
    if (sensor_named(name, family))
      return sensor_named(name, family);
    /* else what this family's vendor ISP took that chip ID for (the
     * XM533's: SC2337P) */
    fprintf(stderr, "libxmv200: %s at %d-%02x: no %s tuning", name, bus, addr,
            family);
  }
  for (int i = 0; i < xmv200_n_sensors; i++)
    if (xmv200_sensors[i].chip_id == id &&
        !strcmp(xmv200_sensors[i].tuning->chip, family)) {
      if (id == 0xcb3a || id == 0x9b3a)
        fprintf(stderr, ", taking %s's\n", xmv200_sensors[i].name);
      return &xmv200_sensors[i];
    }
  fprintf(stderr, "libxmv200: sensor %04x at %d-%02x: no %s tuning\n", id, bus,
          addr, family);
  return NULL;
}

/* the sensors answer only once they are clocked */
static void sensor_clocks_on(const char *family) {
  const struct xmv200_chip *chip = NULL;
  uint32_t c74 = 0xc104c103, c78 = 0xc105c0ff;

  for (int i = 0; i < xmv200_n_sensors; i++)
    if (!strcmp(xmv200_sensors[i].tuning->chip, family)) {
      chip = xmv200_chip_of(xmv200_sensors[i].tuning);
      if (strcmp(family, "xm530v200")) {
        c74 = xmv200_sensors[i].tuning->sns_clk[0];
        c78 = xmv200_sensors[i].tuning->sns_clk[1];
      }
      break;
    }
  if (!chip || chip->sys_clocks) /* the module loader set them */
    return;
  xmv200_reg_write(SYS, 1);
  xmv200_reg_write(SYS + 0x74, c74);
  xmv200_reg_write(SYS + 0x78, c78);
  xmv200_reg_write(SYS, 0);
  usleep(10000);
}

/* "auto": the sensors on bus 0 at 0x30 and 0x32. Two of a kind are spliced
 * top to bottom (as the X6C-WEQ's and X6E-WEQ's), one gives its own
 * picture; each stream has a 640-wide one beside it. Without VPSS, the
 * first sensor alone. */
static int auto_layout(struct xmv200_layout *l) {
  uint8_t addrs[8] = {0x30, 0x32};
  const char *chip = xmv200_chip_name(), *family = family_of(chip);
  const struct xmv200_sns *s[2] = {NULL, NULL};
  const struct xmv200_chip *cp;
  int n = 0, na;

  /* SmartSens' addresses, then the other sensors' */
  na = 2 + xmv200_sns_probe_addrs(family, addrs + 2, 6);
  sensor_clocks_on(family);
  for (int i = 0; i < na && n < 2; i++)
    if ((s[n] = probe(0, addrs[i], family)) != NULL) {
      l->sensor[n] = (struct xmv200_layout_sensor){
          .model = s[n]->name, .bus = 0, .addr = addrs[i], .x0 = 1};
      n++;
    }
  if (!n || (n == 2 && s[0] != s[1]))
    return -1;
  cp = xmv200_chip_of(s[0]->tuning);
  if (cp->vi_scaler)
    n = 1;
  for (int c = 1; c < n; c++) {
    l->sensor[c].vi_idx = c;
    if (cp->vi_mode) {
      l->sensor[c].phy = c; /* XM535D1: a receiver each */
    } else {
      l->sensor[c].mipi = c;
      l->sensor[c].flip = 0x66; /* the second lens is mounted turned */
    }
  }

  const struct xmv200_sns_tuning *t = s[0]->tuning;
  uint32_t w = t->width & ~0xfu, h = t->height & ~0x7u;

  l->chip = chip;
  l->n_sensors = n;
  l->n_streams = 2;
  l->stream[0] = (struct xmv200_layout_stream){w, h, (1u << n) - 1};
  l->stream[1] = (struct xmv200_layout_stream){640, (640 * h / w) & ~0x7u,
                                               (1u << n) - 1};
  l->fps = 12;
  fprintf(stderr, "libxmv200: %s: %d x %s, %ux%u\n", chip ? chip : "?", n,
          s[0]->name, w, h * n);
  return 0;
}

int xmv200_board_layout(const char *name, struct xmv200_layout *l) {
  memset(l, 0, sizeof(*l));
  if (!strcmp(name, "auto"))
    return auto_layout(l);
  for (unsigned i = 0; i < sizeof(boards) / sizeof(boards[0]); i++)
    if (!strcmp(boards[i].name, name)) {
      *l = boards[i].l;
      return 0;
    }
  fprintf(stderr, "libxmv200: no board %s\n", name);
  return -1;
}

/* a layout as the pipeline runs it: the sensors' tuning for this chip, the
 * ISP channels and the streams */
static int layout_board(const struct xmv200_layout *l, struct board *b) {
  const char *family = family_of(l->chip ? l->chip : xmv200_chip_name());
  const struct xmv200_sns_tuning *t = NULL;
  const char *model = NULL;

  memset(b, 0, sizeof(*b));
  if (l->n_sensors < 1 || l->n_sensors > XMV200_MAX_SENSORS ||
      l->n_streams < 1 || l->n_streams > XMV200_MAX_STREAMS)
    return -1;
  sensor_clocks_on(family);
  for (int c = 0; c < l->n_sensors; c++) {
    const struct xmv200_layout_sensor *ls = &l->sensor[c];
    const struct xmv200_sns *sn = ls->model ? sensor_named(ls->model, family)
                                            : probe(ls->bus, ls->addr, family);

    if (!sn) {
      fprintf(stderr, "libxmv200: sensor %d (%s): no %s tuning\n", c,
              ls->model ? ls->model : "probed", family);
      return -1;
    }
    if (t && sn->tuning != t) {
      fprintf(stderr, "libxmv200: the sensors have to be of one kind\n");
      return -1;
    }
    t = sn->tuning;
    model = sn->name;
    b->isp.sensor[c] =
        (struct xmv200_isp_sensor){ls->bus, ls->addr, ls->flip};
    b->isp.chn[c] = (struct xmv200_isp_chn){
        .chn = c,
        .mipi = ls->mipi,
        .x0 = ls->x0,
        .y0 = ls->y0,
        .out_width = ls->width ? ls->width : t->width & ~0xfu,
        .out_height = ls->height ? ls->height : t->height & ~0x7u,
        .ldc = ls->ldc,
        .phy = ls->phy,
        .vi_dev = ls->vi_dev,
        .vi_idx = ls->vi_idx};
  }
  b->name = "layout";
  b->isp.tuning = t;
  b->isp.n_chn = l->n_sensors;
  b->n_streams = l->n_streams;
  b->fps = l->fps ? l->fps : 12;
  /* the sensor's driver, in the mode the tuning is of */
  b->isp.mode = xmv200_sns_find(model, t, &b->isp.drv);
  if (b->isp.mode) {
    struct xmv200_sns_dev d = {b->isp.sensor[0].bus, b->isp.sensor[0].addr, 0};

    b->isp.attr = b->isp.mode->attr;
    /* the frame rate the tuning's ISP program is for */
    b->isp.attr.fps = xmv200_sns_capture_fps(b->isp.drv, b->isp.mode, t, &d,
                                             b->fps);
  }
  for (int s = 0; s < l->n_streams; s++) {
    uint32_t w = l->stream[s].width, h = l->stream[s].height;
    uint32_t g = l->stream[s].sensors & ((1u << l->n_sensors) - 1);

    if (!g || !w || !h)
      return -1;
    b->stream[s].width = w;
    b->stream[s].height = h;
    b->stream[s].groups = g;
    b->stream[s].stream_buf = w * h * popcount(g) / 4 + 0x10000;
  }
  return 0;
}

int xmv200_board_start(const char *name, struct xmv200_board_info *info) {
  struct xmv200_layout l;

  if (xmv200_board_layout(name, &l))
    return -1;
  return xmv200_layout_start(&l, info);
}

int xmv200_layout_start(const struct xmv200_layout *l,
                        struct xmv200_board_info *info) {
  static struct board board;
  const struct board *b = &board;
  int fd_vpss, fd_sys, fd_venc, fd_rgn, fd_hevc, fd_vb, fd_vi, n;
  const struct xmv200_chip *chip;
  uint32_t reset;

  if (layout_board(l, &board))
    return -1;
  n = b->isp.n_chn;
  chip = xmv200_chip_of(b->isp.tuning);
  xmv200_venc_set_engine(chip->engine);
  reset = chip->hevc_reset;

  fd_vpss = chip->vi_scaler ? -1 : open_dev("/dev/vpss");
  fd_sys = open_dev("/dev/sys");
  fd_venc = open_dev("/dev/venc_info");
  fd_rgn = open_dev("/dev/rgn");
  fd_hevc = open_dev("/dev/h265");
  fd_vb = open_dev("/dev/vb");
  fd_vi = chip->vi_scaler ? open_dev("/dev/vi") : -1;
  if ((chip->vi_scaler ? fd_vi : fd_vpss) < 0 || fd_sys < 0 || fd_venc < 0 ||
      fd_hevc < 0)
    return -1;

  /* the system: nothing allocates before the MMZ reset */
  if (xmv200_mmz_reset())
    return -1;
  if (chip->vi_scaler ? ioctl(fd_vi, VI_INIT, 0) < 0
                      : ioctl(fd_vpss, VPSS_INIT, 0) < 0)
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
  for (int s = 0; s < b->n_streams; s++) {
    uint32_t size = b->stream[s].width * b->stream[s].height *
                    popcount(b->stream[s].groups) * 3 / 2;

    /* small pictures get two more: equal sizes share buffers (vb.ko takes
     * the best fit), and a faster sensor's would take them all */
    pools[np++] = (struct xmv200_vb_pool){
        size, chip->vb_count + (size < 0x100000 ? 2 : 0)};
  }
  if (chip->isp_pool)
    pools[np++] = (struct xmv200_vb_pool){
        (b->isp.tuning->width & ~0xfu) *
            ((b->isp.tuning->height + 15) & ~15u) * 3 / 2,
        3};
  if (xmv200_vb_init(pools, np))
    return -1;

  /* VPSS: a group per sensor, fed by the ISP; its channels spliced top to
   * bottom into the streams and bound to VENC (without VPSS, VI's channels
   * come after the ISP) */
  for (int g = 0; !chip->vi_scaler && g < n; g++) {
    uint32_t grp[7] = {4096, 2160, 0, 0, 0, 0, 1};

    if (vpss(fd_vpss, VPSS_CREATE_GRP, g, 0, grp, 7) ||
        ioctl(fd_vpss, VPSS_START_GRP, &g) < 0)
      return -1;
  }
  for (int g = 0; !chip->vi_scaler && g < n; g++)
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
  for (int s = 0; !chip->vi_scaler && s < b->n_streams; s++) {
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
  isp_pll(chip, fd_sys, b->isp.tuning->pll[0], b->isp.tuning->n_pll[0]);

  /* the sensors out of reset (SmartSens'; a driver of another's does its
   * own), each receiver's setup, each channel's VI */
  for (int c = 0; !(b->isp.drv && b->isp.drv->probe) && c < n; c++) {
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
    /* VI's input: on the XM535 the channel's VI device and its place */
    xmv200_reg_write(VI + chip->vi_ctrl, !chip->vi_mode ? 2
                                         : ch->vi_dev   ? 6
                                         : ch->vi_idx   ? 0xa
                                                        : 2);
    if (chip->vi_ctrl_chn)
      xmv200_reg_write(VI + chip->vi_ctrl + 0x18, chip->vi_mode ? c > 0 : c);
  }
  isp_pll(chip, fd_sys, b->isp.tuning->pll[1], b->isp.tuning->n_pll[1]);

  if (xmv200_isp_start(&b->isp))
    return -1;

  /* without VPSS: a VI channel per stream, scaling the ISP's picture, and
   * bound to VENC */
  for (int s = 0; chip->vi_scaler && s < b->n_streams; s++) {
    uint32_t a[20] = {0, s, 0, 0x20, 0, 0, b->isp.chn[0].out_width,
                      b->isp.chn[0].out_height & ~7u, chip->vi_format,
                      b->stream[s].width, b->stream[s].height, 0, 0, 25,
                      b->fps < 25 ? b->fps : 25};
    uint32_t on = s, bind[20] = {0, s, 0, 0, MOD_VENC, 0, s};

    if (ioctl(fd_vi, VI_SET_CHN_ATTR, a) < 0 ||
        ioctl(fd_vi, VI_ENABLE_CHN, &on) < 0 ||
        ioctl(fd_vi, VI_BIND, bind) < 0 ||
        sys_bind(fd_sys, MOD_VI, 0, s, MOD_VENC, 0, s) < 0) {
      fprintf(stderr, "libxmv200: vi chn %d failed\n", s);
      return -1;
    }
  }

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
