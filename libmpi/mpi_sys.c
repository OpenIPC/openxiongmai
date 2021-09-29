#include "re_type.h"
#include <re_region.h>
#include <re_sys.h>
#include <re_vi.h>

#include <fcntl.h>
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

XM_S32 XM_MPI_SYS_MmzAlloc(XM_U32 *pu32PhyAddr, XM_VOID **ppVirtAddr,
                           const XM_CHAR *strMmb, const XM_CHAR *strZone,
                           XM_U32 u32Len) {
  struct {
    char name[32];
    XM_U32 u32Size;
    XM_U32 u32PhyAddr;
  } arg = {};

  if ((fd_mmz == -1) && (fd_mmz = open("/dev/mmz", 0), fd_mmz < 0)) {
    FATAL("mmz dev Open Failed");
    return XM_FAILURE;
  }

  memcpy(arg.name, strMmb, sizeof(arg.name));
  arg.u32Size = u32Len;
  if (ioctl(fd_mmz, 0xC0284D01, &arg)) {
    FATAL("mmz alloc failed");
    return XM_FAILURE;
  }
  *pu32PhyAddr = arg.u32PhyAddr;
  *ppVirtAddr = XM_MPI_SYS_Mmap(arg.u32PhyAddr, arg.u32Size);
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
  struct {
    char name[32];
    XM_U32 u32PhyAddr;
    XM_U32 u32Size;
  } arg = {};

  if ((fd_mmz == -1) && (fd_mmz = open("/dev/mmz", 0), fd_mmz < 0)) {
    FATAL("mmz dev Open Failed");
    return XM_FAILURE;
  }

  memcpy(arg.name, pstrMmb, sizeof(arg.name));
  if (ioctl(fd_mmz, 0x80284D03, &arg)) {
    FATAL("mmz get addr failed");
    return XM_FAILURE;
  }
  *pu32PhyAddr = arg.u32PhyAddr;
  *pu32Len = arg.u32Size;
  *ppVitAddr = XM_MPI_SYS_Mmap(arg.u32PhyAddr, arg.u32Size);
  return XM_SUCCESS;
}
