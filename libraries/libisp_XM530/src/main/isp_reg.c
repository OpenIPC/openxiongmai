/* Register access through /dev/isp, and the helpers the vendor library
 * exports to the ISP application, the sensor libraries and majestic. */
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include "isp_main.h"

/* /dev/isp: _IOW('I', n, 12) on {addr, val, arg} */
#define ISP_IOC_WRITE 0x400c4901
#define ISP_IOC_READ 0x400c4902
#define ISP_IOC_WAIT 0x400c4904 /* arg: 0, the Bayer frame */

struct isp_rec {
  XM_U32 addr, val, arg;
};

static int isp_fd = -1;

int isp_open(void) {
  if (isp_fd < 0) {
    int fd = open("/dev/isp", O_RDWR | O_CLOEXEC);
    if (fd < 0)
      return -1;
    /* another thread may have opened it meanwhile */
    if (__sync_val_compare_and_swap(&isp_fd, -1, fd) != -1)
      close(fd);
  }
  return isp_fd;
}

/* The ISP (0x30000000, 2 MiB) and GPIO (0x10020000, 1 MiB) blocks are what
 * /dev/isp reaches. */
static int isp_reachable(XM_U32 addr) {
  return (addr >= 0x30000000 && addr < 0x30200000) ||
         (addr >= 0x10020000 && addr < 0x10120000);
}

static int isp_get(XM_U32 addr, XM_U32 *val) {
  struct isp_rec r = {addr, 0, 0};
  if (isp_open() < 0 || ioctl(isp_fd, ISP_IOC_READ, &r) < 0)
    return XM_FAILURE;
  *val = r.val;
  return XM_SUCCESS;
}

static int isp_set(XM_U32 addr, XM_U32 val) {
  struct isp_rec r = {addr, val, 0};
  if (isp_open() < 0 || ioctl(isp_fd, ISP_IOC_WRITE, &r) < 0)
    return XM_FAILURE;
  return XM_SUCCESS;
}

XM_U32 isp_read(XM_U32 addr) {
  XM_U32 v = 0;
  isp_get(addr, &v);
  return v;
}

void isp_write(XM_U32 addr, XM_U32 val) { isp_set(addr, val); }

/* The block /dev/isp shares (16 KiB, kept while the module is loaded). */
#define SHM_SIZE 0x4000
void *isp_shm(void) {
  static void *shm;
  if (!shm && isp_open() >= 0) {
    void *p =
        mmap(NULL, SHM_SIZE, PROT_READ | PROT_WRITE, MAP_SHARED, isp_fd, 0);
    if (p != MAP_FAILED)
      shm = p;
  }
  return shm;
}

int isp_wait_frame(void) {
  struct isp_rec r = {0, 0, 0};
  if (isp_open() < 0)
    return -1;
  return ioctl(isp_fd, ISP_IOC_WAIT, &r);
}

void isp_write_table(const struct xm530_reg *t, XM_U32 mmz8) {
  for (; !(t->flags & XM530_END); t++)
    isp_write(t->addr, t->flags & XM530_MMZ ? mmz8 + t->val : t->val);
}

/* The system controller's registers (PLLs, resets: the first 4 KiB at
 * 0x20000000) through /dev/mem; nothing else is reached that way. */
#define SYSCTL_BASE 0x20000000
#define SYSCTL_SIZE 0x1000

static int mem_access(XM_U32 addr, XM_U32 *val, int write) {
  const long page = sysconf(_SC_PAGESIZE);
  const XM_U32 base = addr & ~(XM_U32)(page - 1);
  if (addr < SYSCTL_BASE || addr >= SYSCTL_BASE + SYSCTL_SIZE)
    return XM_FAILURE;
  int fd = open("/dev/mem", O_RDWR | O_SYNC | O_CLOEXEC);
  if (fd < 0)
    return XM_FAILURE;
  volatile XM_U32 *p =
      mmap(NULL, page, PROT_READ | PROT_WRITE, MAP_SHARED, fd, base);
  close(fd);
  if (p == MAP_FAILED)
    return XM_FAILURE;
  if (write)
    p[(addr - base) / 4] = *val;
  else
    *val = p[(addr - base) / 4];
  munmap((void *)p, page);
  return XM_SUCCESS;
}

XM_S32 XM_MPI_ISP_SetRegister(XM_U32 u32Addr, XM_U32 u32Value) {
  if (u32Addr & 3)
    return XM_FAILURE;
  if (isp_reachable(u32Addr))
    return isp_set(u32Addr, u32Value);
  return mem_access(u32Addr, &u32Value, 1);
}

XM_S32 XM_MPI_ISP_GetRegister(XM_U32 u32Addr, XM_U32 *pu32Value) {
  if (!pu32Value || (u32Addr & 3))
    return XM_FAILURE;
  if (isp_reachable(u32Addr))
    return isp_get(u32Addr, pu32Value);
  return mem_access(u32Addr, pu32Value, 0);
}

/* The other libraries' register access. The AWB library reads its
 * statistics through these, and the luma grids the AE library takes are
 * made from what it reads. */
XM_S32 Write_IspReg(XM_U32 u32Addr, XM_U32 u32Value) {
  stats_port_write(u32Addr, u32Value);
  isp_write(u32Addr, u32Value);
  return XM_SUCCESS;
}

XM_S32 Read_IspReg(XM_U32 u32Addr) {
  const XM_U32 v = isp_read(u32Addr);
  stats_port_read(u32Addr, v);
  return (XM_S32)v;
}

XM_S32 XM_MPI_ISP_Memncpy(XM_U8 *pDst, XM_U8 *pSrc, XM_U32 u32Num) {
  if (!pDst || !pSrc)
    return XM_FAILURE;
  memmove(pDst, pSrc, u32Num);
  return XM_SUCCESS;
}

XM_S32 ispMemnCpy(XM_U8 *pDst, XM_U8 *pSrc, XM_U32 u32Num) {
  return XM_MPI_ISP_Memncpy(pDst, pSrc, u32Num);
}

XM_VOID XM_MPI_ISP_Memset(XM_U8 *pu8Addr, XM_U8 u8Ch, XM_U32 u32Num) {
  if (pu8Addr)
    memset(pu8Addr, u8Ch, u32Num);
}

void SysDelay_ms(XM_S32 ms) {
  if (ms > 0)
    usleep((useconds_t)ms * 1000);
}

/* The ISP application's MIPI lane-delay check, a diagnostic mode only. */
XM_S32 Mipi_check_init(void) { return XM_FAILURE; }
XM_S32 Mipi_check_run(void) { return XM_FAILURE; }

/* Majestic's AE calls this (mode 1) before it slows the frame rate itself;
 * the vendor library's returns 0 and changes nothing a camera shows. */
XM_S32 IspSetSlowFrameRate(XM_U8 u8Mode) {
  (void)u8Mode;
  return XM_SUCCESS;
}
