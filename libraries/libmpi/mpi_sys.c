#include "re_type.h"
#include <re_region.h>
#include <re_sys.h>
#include <re_vi.h>

#include <fcntl.h>
#include <pthread.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>

#define FATAL(msg)                                                             \
  fprintf(stderr, "sys - %s(%d): " msg "\n", __FUNCTION__, __LINE__)

static int fd_mmz = -1;
static int fd_mem = -1;

XM_S32 XM_MPI_SYS_Init(XM_VOID) {
  if (XM_MPI_SYS_MmzReset() != XM_SUCCESS)
    return XM_FAILURE;

  if (XM_MPI_VI_INIT() != XM_SUCCESS)
    return XM_FAILURE;

  if (XM_MPI_RGN_INIT() != XM_SUCCESS)
    return XM_FAILURE;

  return XM_SUCCESS;
}

/*
 * mmz.ko takes a record of name[32], size and phys, plus a trailing word
 * on XM530: 40 bytes on XM510 (0xC0284D01 alloc, 0x80284D03 get) and 44
 * on XM530 (0xC02C4D01, 0x802C4D03). Get-by-name fills size before phys
 * on both. A module that doesn't know a command logs "ioctl cmd error"
 * but still returns 0, so a zero address means the call failed. The
 * record size that worked is remembered.
 */
#define MMZ_CMD(dir, nr, size) (((dir) << 30) | ((size) << 16) | ('M' << 8) | (nr))
#define MMZ_ALLOC 1
#define MMZ_GET 3

struct mmz_rec {
  char name[32];
  XM_U32 u32Size;
  XM_U32 u32PhyAddr;
  XM_U32 u32Rsvd; /* XM530 only */
};

static XM_S32 mmz_call_locked(int nr, const XM_CHAR *name, XM_U32 *pu32Size,
                              XM_U32 *pu32PhyAddr) {
  static const XM_U32 sizes[] = {sizeof(struct mmz_rec),
                                 sizeof(struct mmz_rec) - sizeof(XM_U32)};
  static int abi = -1;
  struct mmz_rec arg;
  int i;

  if ((fd_mmz == -1) && (fd_mmz = open("/dev/mmz", 0), fd_mmz < 0)) {
    FATAL("mmz dev Open Failed");
    return XM_FAILURE;
  }

  for (i = abi < 0 ? 0 : abi; i < 2; i++) {
    memset(&arg, 0, sizeof(arg));
    strncpy(arg.name, name, sizeof(arg.name) - 1);
    arg.u32Size = *pu32Size;
    if (ioctl(fd_mmz, MMZ_CMD(nr == MMZ_ALLOC ? 3 : 2, nr, sizes[i]), &arg) ==
            0 &&
        arg.u32PhyAddr) {
      abi = i;
      *pu32Size = arg.u32Size;
      *pu32PhyAddr = arg.u32PhyAddr;
      return XM_SUCCESS;
    }
    if (abi >= 0)
      break;
  }
  return XM_FAILURE;
}

/* Serialises the record-size probe, its cache and the /dev/mmz open */
static XM_S32 mmz_call(int nr, const XM_CHAR *name, XM_U32 *pu32Size,
                       XM_U32 *pu32PhyAddr) {
  static pthread_mutex_t lock = PTHREAD_MUTEX_INITIALIZER;
  XM_S32 ret;

  pthread_mutex_lock(&lock);
  ret = mmz_call_locked(nr, name, pu32Size, pu32PhyAddr);
  pthread_mutex_unlock(&lock);
  return ret;
}

XM_S32 XM_MPI_SYS_MmzAlloc(XM_U32 *pu32PhyAddr, XM_VOID **ppVirtAddr,
                           const XM_CHAR *strMmb, const XM_CHAR *strZone,
                           XM_U32 u32Len) {
  XM_U32 u32Size = u32Len;

  if (mmz_call(MMZ_ALLOC, strMmb, &u32Size, pu32PhyAddr) != XM_SUCCESS ||
      u32Size < u32Len) {
    FATAL("mmz alloc failed");
    return XM_FAILURE;
  }
  *ppVirtAddr = XM_MPI_SYS_Mmap(*pu32PhyAddr, u32Size);
  return XM_SUCCESS;
}

XM_S32 XM_MPI_SYS_MmzFree(XM_U32 u32PhyAddr, XM_VOID *pVirtAddr) {
  return XM_SUCCESS;
}

XM_VOID *XM_MPI_SYS_Mmap(XM_U32 u32PhyAddr, XM_U32 u32Size) {
  if ((fd_mem == -1) && (fd_mem = open("/dev/mem", 0x1002), fd_mem < 0)) {
    FATAL("mmz dev Open Failed");
    return (void *)XM_FAILURE;
  }

  char *pData =
      (char *)mmap(0, u32Size + 0xfff + ((u32PhyAddr & 0xfff) & 0xfffff000), 3,
                   1, fd_mem, u32PhyAddr & 0xFFFFF000);
  if (pData == MAP_FAILED) {
    FATAL("Memory Map Failed For Address");
  }

  return &pData[u32PhyAddr & 0xFFF];
}

XM_S32 XM_MPI_SYS_Munmap(XM_VOID *pVirAddr, XM_U32 u32Size) {
  munmap((void *)((unsigned int)pVirAddr & 0xFFFFF000),
         (u32Size + 4095 + ((XM_U32)pVirAddr & 0xFFF)) & 0xFFFFF000);
  return XM_SUCCESS;
}

XM_S32 XM_MPI_SYS_MmzReset(void) {

  if ((fd_mmz == -1) && (fd_mmz = open("/dev/mmz", 0), fd_mmz < 0)) {
    FATAL("mmz reset failed");
    return XM_FAILURE;
  }

  return ioctl(fd_mmz, 0x4D00u, 0);
}

XM_S32 XM_MPI_SYS_MmzGetAddr(XM_U32 *pu32PhyAddr, void **ppVitAddr,
                             const XM_CHAR *pstrMmb, const XM_CHAR *pstrZone,
                             XM_U32 *pu32Len) {
  *pu32Len = 0;
  if (mmz_call(MMZ_GET, pstrMmb, pu32Len, pu32PhyAddr) != XM_SUCCESS) {
    FATAL("mmz get addr failed");
    return XM_FAILURE;
  }
  *ppVitAddr = XM_MPI_SYS_Mmap(*pu32PhyAddr, *pu32Len);
  return XM_SUCCESS;
}
