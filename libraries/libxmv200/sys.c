// SPDX-License-Identifier: MIT
/*
 * Registers, sensor I2C, MMZ and VB.
 *
 * /dev/mem: the vendor userspace writes the PLL and clock registers
 * (0x20000000), the MIPI receivers and the VI windows (0x30000000) itself.
 *
 * i2c.ko, /dev/i2cN: a 20-byte record {dev, reg, reg_bytes, val, val_bytes};
 * 1 writes, 3 reads (the value comes back in the record).
 *
 * mmz.ko, /dev/mmz: 'M' 0 resets (the vendor modules allocate nothing until
 * it has: "mmz: not init"); 44-byte records {name[32], size, phys, ...}.
 *
 * vb.ko, /dev/vb: 0x41044201 sets the pool table {count, {size, count}[32]},
 * 0x4200 allocates the pools from MMZ.
 */
#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include "xmv200.h"

static int open_once(int *fd, const char *path) {
  if (*fd < 0)
    *fd = open(path, O_RDWR);
  return *fd;
}

static int mem_fd = -1;

static volatile uint32_t *reg_map(uint32_t addr, void **page) {
  if (open_once(&mem_fd, "/dev/mem") < 0)
    return NULL;
  *page = mmap(NULL, 0x1000, PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd,
               addr & ~0xfffu);
  if (*page == MAP_FAILED)
    return NULL;
  return (volatile uint32_t *)((uint8_t *)*page + (addr & 0xffc));
}

int xmv200_reg_write(uint32_t addr, uint32_t val) {
  void *page;
  volatile uint32_t *r = reg_map(addr, &page);

  if (!r)
    return -1;
  *r = val;
  munmap(page, 0x1000);
  return 0;
}

int xmv200_reg_read(uint32_t addr, uint32_t *val) {
  void *page;
  volatile uint32_t *r = reg_map(addr, &page);

  if (!r)
    return -1;
  *val = *r;
  munmap(page, 0x1000);
  return 0;
}

struct i2c_rec {
  uint8_t dev;
  uint8_t pad[3];
  uint32_t reg;
  uint32_t reg_bytes;
  uint32_t val;
  uint32_t val_bytes;
};

#define I2C_WRITE 1
#define I2C_READ 3

static int i2c_fd[3] = {-1, -1, -1};

/* i2c.ko's devices: /dev/i2c0..2, or on the XM533 /dev/xm_i2c (bus 0) and
 * /dev/xm_i2c1..2 */
static int i2c_open(int bus) {
  char path[16];

  if (bus < 0 || bus > 2)
    return -1;
  if (i2c_fd[bus] >= 0)
    return i2c_fd[bus];
  snprintf(path, sizeof(path), "/dev/i2c%d", bus);
  if (access(path, F_OK)) {
    if (bus)
      snprintf(path, sizeof(path), "/dev/xm_i2c%d", bus);
    else
      snprintf(path, sizeof(path), "/dev/xm_i2c");
  }
  return open_once(&i2c_fd[bus], path);
}

void (*xmv200_i2c_divert)(uint32_t reg, uint32_t val);

int xmv200_i2c_write(int bus, uint8_t dev, uint32_t reg, int reg_bytes,
                     uint32_t val, int val_bytes) {
  struct i2c_rec r = {.dev = dev,
                      .reg = reg,
                      .reg_bytes = reg_bytes,
                      .val = val,
                      .val_bytes = val_bytes};
  int fd;

  if (xmv200_i2c_divert) {
    xmv200_i2c_divert(reg, val);
    return 0;
  }
  fd = i2c_open(bus);

  if (fd < 0)
    return -1;
  return ioctl(fd, I2C_WRITE, &r);
}

int xmv200_i2c_read(int bus, uint8_t dev, uint32_t reg, int reg_bytes,
                    uint32_t *val, int val_bytes) {
  struct i2c_rec r = {
      .dev = dev, .reg = reg, .reg_bytes = reg_bytes, .val_bytes = val_bytes};
  int fd = i2c_open(bus);

  if (fd < 0 || ioctl(fd, I2C_READ, &r) < 0)
    return -1;
  *val = r.val;
  return 0;
}

struct mmz_rec {
  char name[32];
  uint32_t size;
  uint32_t phys;
  uint32_t rsvd;
};

#define MMZ_RESET _IO('M', 0)
#define MMZ_ALLOC _IOWR('M', 1, struct mmz_rec)
#define MMZ_ALLOC_CACHED _IOWR('M', 4, struct mmz_rec)
#define MMZ_FREE _IOW('M', 2, struct mmz_rec)

static int mmz_fd = -1;

/* the blocks allocated since the last reset: only those are freed (after a
 * reset an address may be another block's) */
#define MMZ_LIVE 64
static uint32_t mmz_live[MMZ_LIVE];

int xmv200_mmz_reset(void) {
  if (open_once(&mmz_fd, "/dev/mmz") < 0)
    return -1;
  memset(mmz_live, 0, sizeof(mmz_live));
  return ioctl(mmz_fd, MMZ_RESET, 0);
}

void xmv200_mmz_free(struct xmv200_buf *b) {
  if (b->virt && b->virt != MAP_FAILED)
    munmap(b->virt, b->size);
  for (int i = 0; b->phys && mmz_fd >= 0 && i < MMZ_LIVE; i++)
    if (mmz_live[i] == b->phys) {
      struct mmz_rec r;

      memset(&r, 0, sizeof(r));
      r.phys = b->phys;
      r.size = b->size;
      /* the list allocator (the V200 series') frees it; the vendor's and
       * the sequential one keep it until the next reset */
      ioctl(mmz_fd, MMZ_FREE, &r);
      mmz_live[i] = 0;
      break;
    }
  memset(b, 0, sizeof(*b));
}

int xmv200_mmz_alloc(struct xmv200_buf *b, const char *name, uint32_t size,
                     int cached) {
  struct mmz_rec r;

  if (open_once(&mmz_fd, "/dev/mmz") < 0)
    return -1;
  memset(&r, 0, sizeof(r));
  strncpy(r.name, name, sizeof(r.name) - 1);
  r.size = size;
  if (ioctl(mmz_fd, cached ? MMZ_ALLOC_CACHED : MMZ_ALLOC, &r) < 0 || !r.phys)
    return -1;
  /* the vendor library maps the block through /dev/mmz itself */
  b->phys = r.phys;
  b->size = size;
  for (int i = 0; i < MMZ_LIVE; i++)
    if (!mmz_live[i]) {
      mmz_live[i] = r.phys;
      break;
    }
  b->virt = mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, mmz_fd,
                 r.phys);
  if (b->virt == MAP_FAILED) {
    b->virt = NULL;
    xmv200_mmz_free(b);
    return -1;
  }
  return 0;
}

struct vb_conf {
  uint32_t count;
  struct {
    uint32_t size;
    uint32_t count;
  } pool[XMV200_VB_MAX_POOLS];
};

#define VB_SET_CONF _IOW('B', 1, struct vb_conf)
#define VB_INIT _IO('B', 0)

static int vb_fd = -1;

int xmv200_vb_init(const struct xmv200_vb_pool *pools, int n) {
  struct vb_conf c;

  if (n < 1 || n > XMV200_VB_MAX_POOLS || open_once(&vb_fd, "/dev/vb") < 0)
    return -1;
  memset(&c, 0, sizeof(c));
  c.count = n;
  for (int i = 0; i < n; i++) {
    c.pool[i].size = pools[i].size;
    c.pool[i].count = pools[i].count;
  }
  if (ioctl(vb_fd, VB_SET_CONF, &c) < 0)
    return -1;
  return ioctl(vb_fd, VB_INIT, 0);
}
