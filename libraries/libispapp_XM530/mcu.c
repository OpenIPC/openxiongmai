#ifdef SOC_SYSTEM
#include <fcntl.h>
#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#endif

#include "extCfg.h"
#include "ispPrivate.h"
#include "xm530_tmp.h"
#include "xm_print.h"

#ifdef SOC_NONE
#define UARTDR0 (*((volatile unsigned long *)0x10030000))
#define UARTFR0 (*((volatile unsigned long *)0x10030018))
#define UARTTIMEOUT 0X04000000
XM_U8 putbyte0(XM_U8 c) {
  XM_U32 timeout = 0;
  XM_U32 st = 0;

  while (1) {
    st = UARTFR0 & 0x00000020;
    if (st == 0) // FIFO未满，就填
    {
      UARTDR0 = (XM_U32)(c & 0xff);
      break;
    }

    if (timeout > UARTTIMEOUT) {
      return 0;
    }

    timeout++;
  }

  return 1;
}
#endif

XM_U8 putstr(XM_U8 uart_NO, char const *str) {
#if (defined SOC_SYSTEM) || (defined SOC_ALIOS)
  printf("%s", str);
  return 1;

#else
#ifdef DEBUG_ON
  XM_U8 flag = 0;
  for (; *str != '\0'; str++) {
    flag = putbyte0(*str);
    if (flag == 0)
      return 0;
  }
#endif
  return 1;
#endif
}

void PrintInt(unsigned char u8Num, int u32Data) {
#if (defined SOC_SYSTEM) || (defined SOC_ALIOS)
  printf("%d ", u32Data);
#else
#ifdef DEBUG_ON
  char strBuf[24];

  strBuf[0] = ' ';
  if (u32Data < 0) {
    strBuf[0] = '-';
    u32Data = u32Data * (-1);
  }
  u8Num = u8Num + 1;
  // strBuf[u8Num] = '\r';
  // strBuf[u8Num+1] = '\n';
  strBuf[u8Num] = ' ';
  strBuf[u8Num + 1] = ' ';
  strBuf[u8Num + 2] = 0;
  for (; u8Num > 1;) {
    u8Num--;
    strBuf[u8Num] = u32Data % 10 + '0';
    u32Data = u32Data / 10;
  }

  putstr(0, strBuf);
#endif
#endif
}

void PrintHex(unsigned char u8Num, int u32Data) {
#if (defined SOC_SYSTEM) || (defined SOC_ALIOS)
  printf("0x%x ", u32Data);
#else
#ifdef DEBUG_ON
  XM_S8 ch[24];
  XM_U8 u8tmp, u8i;
  putstr(0, "0x");

  for (u8i = 0; u8Num > 0; u8i++) {
    u8Num--;
    u8tmp = (u32Data >> (u8Num * 4)) & 0xf;
    if (u8tmp < 10)
      u8tmp += '0';
    else
      u8tmp = u8tmp - 10 + 'A';
    ch[u8i] = (XM_S8)u8tmp;
  }
  // ch[u8i]='\r';
  // ch[u8i+1]='\n';
  // ch[u8i+2]=0;
  ch[u8i] = 0;
  putstr(0, ch);
#endif
#endif
}

#ifdef SOC_SYSTEM
XM_U8 gu8LoopIsp = 1;

#define CFG_FILE "/home/extCfg.bin"
// XM_U8 gau8Buffer[4096];
/*
 * 函数说明:  读二进制文件
 *  参数描述: _fileName, 文件名称
 *             _buf, 读出来的数据存放位置
 *             _bufLen, 数据的长度信息
 *    返回值:  0, 成功
 *             -1, 失败
 *
 */
int readFile(const char *_fileName, void *_buf, int _bufLen) {

  FILE *fp = NULL;
  if (NULL == _buf || _bufLen <= 0)
    return (-1);

  fp = fopen(_fileName, "rb");
  if (NULL == fp) {
    return (-1);
  }

  fread(_buf, _bufLen, 1, fp);
  fclose(fp);
  return 0;
}

#define REG_SPACE_SIZE 0x1000
static XM_U8 gu81thFlg = 1;
static XM_U32 gau32Virtaddr[2];
static XM_S32 SysRegInit(XM_VOID) {
  int fd;
  fd = open("/dev/mem", O_RDWR | O_SYNC);
  if (fd < 0) {
    ERR("mem dev Open Failed\n");
    return -1;
  }
  gau32Virtaddr[0] =
      (XM_U32)mmap((void *)0, REG_SPACE_SIZE, PROT_READ | PROT_WRITE,
                   MAP_SHARED, fd, (0 & 0xfffff000));
  gau32Virtaddr[1] =
      (XM_U32)mmap((void *)0, REG_SPACE_SIZE, PROT_READ | PROT_WRITE,
                   MAP_SHARED, fd, (0x1000 & 0xfffff000));
  close(fd);
  gu81thFlg = 0;
  return XM_SUCCESS;
}
extern XM_S32 ispTool(void);
extern int Audio_testEn(XM_U8 u8En);
XM_S32 SysRegExit() {
#if (!defined SOC_XMSDK) && (!defined SOC_ALIOS)
  while (gu8LoopIsp) {
#if 0 // Debug
		ispTool();
#endif
    sleep(3);
  }

  if (gau32Virtaddr[0] != 0)
    munmap((void *)gau32Virtaddr[0], REG_SPACE_SIZE);
  if (gau32Virtaddr[1] != 0)
    munmap((void *)gau32Virtaddr[1], REG_SPACE_SIZE);
  gu81thFlg = 1;
  Audio_testEn(0);
  DEBUG("IspSample Exit!\n");
#endif
  return 0;
}

XM_S32 SysRegWriteTmp(XM_U32 phyaddr, XM_U32 val) {
  XM_U8 u8Idx;
  if (gu81thFlg)
    SysRegInit();

  u8Idx = (phyaddr < 0x1000) ? 0 : 1;
  *((volatile XM_U32 *)(gau32Virtaddr[u8Idx] + (phyaddr & 0x00000fff))) = val;
  return 0;
}

XM_S32 SysRegReadTmp(XM_U32 phyaddr, XM_U32 *val) {
  XM_U8 u8Idx;
  if (gu81thFlg)
    SysRegInit();

  u8Idx = (phyaddr < 0x1000) ? 0 : 1;
  *val = *((volatile XM_U32 *)(gau32Virtaddr[u8Idx] + (phyaddr & 0x00000fff)));
  return 0;
}

XM_S32 Write_IspRegSys(XM_U32 u32Addr, XM_U32 u32Value) {
  return Write_IspReg(u32Addr, u32Value);
}

XM_S32 Read_IspRegSys(XM_U32 u32Addr) { return Read_IspReg(u32Addr); }

#if 0
void SysDelay_ms(XM_S32 ms) { usleep(ms * 1000); }
#endif

#else
#define REG(addr) (*((volatile unsigned long *)(addr)))
XM_S32 SysRegExit() {
#if (defined SOC_NONE)
  while (1)
    ;
#endif
  return 0;
}

XM_S32 Write_IspRegSys(XM_U32 u32Addr, XM_U32 u32Value) {
  return XM_MPI_ISP_SetRegister(u32Addr, u32Value);
}

XM_S32 Read_IspRegSys(XM_U32 u32Addr) {
  XM_U32 u32Val;
  XM_MPI_ISP_GetRegister(u32Addr, &u32Val);
  return u32Val;
}

XM_S32 SysRegWriteTmp(XM_U32 phyaddr, XM_U32 val) {
  return XM_MPI_ISP_SetRegister(phyaddr, val);
}

XM_S32 SysRegReadTmp(XM_U32 phyaddr, XM_U32 *val) {
  return XM_MPI_ISP_GetRegister(phyaddr, val);
}
#endif

#ifdef SOC_NONE
#define TIMER_CLK (24000000)
#define TIMER4_BASE (0x100D0000)
#define TIMER4_LOAD (TIMER4_BASE + 0x20)
#define TIMER4_VALUE (TIMER4_BASE + 0x24)
#define TIMER4_CONTROL (TIMER4_BASE + 0x28)
#define TIMER4_BGLOAD (TIMER4_BASE + 0x38)

void SysDelay_ms(XM_S32 nms) {
  XM_U32 u32Tmp;
  unsigned int u32TimerPer;                     // 周期
  XM_MPI_ISP_SetRegister(TIMER4_CONTROL, 0x47); // Int  24/16 M	32Bit  oneShot

  // (24M/16) / 15000 ===>  T=1ms
  u32TimerPer = (unsigned int)TIMER_CLK / 16 / 1000;
  u32TimerPer = u32TimerPer * nms;
  XM_MPI_ISP_SetRegister(TIMER4_BGLOAD, u32TimerPer);
  XM_MPI_ISP_SetRegister(TIMER4_LOAD, u32TimerPer);
  // Timer4
  u32Tmp = 0;
  XM_MPI_ISP_GetRegister(TIMER4_CONTROL, &u32Tmp);
  XM_MPI_ISP_SetRegister(TIMER4_CONTROL, (u32Tmp | (1 << 7))); // Start Timer4
  while (1) {
    XM_MPI_ISP_GetRegister(TIMER4_VALUE, &u32Tmp);
    if (u32Tmp < 2)
      break;
  }
  XM_MPI_ISP_SetRegister(TIMER4_CONTROL, 0);
}
#endif

#define TIMER1_BASE (0x100C0000)
#define TIMER1_LOAD (TIMER1_BASE + 0x00)
#define TIMER1_VALUE (TIMER1_BASE + 0x04)
#define TIMER1_CONTROL (TIMER1_BASE + 0x08)
#define TIMER1_BGLOAD (TIMER1_BASE + 0x18)

void Timer1_Test(XM_U8 u8Mode, XM_U32 u32TimerPer) {
#if 0
#ifdef SOC_NONE
  static XM_U32 su32StartVal;
  XM_U32 u32Tmp;

  if (u8Mode > 0) // End
  {
    XM_MPI_ISP_GetRegister(TIMER1_VALUE, &u32TimerPer);
    XM_MPI_ISP_SetRegister(TIMER1_CONTROL, 0);

    u32Tmp = su32StartVal - u32TimerPer;
    u32TimerPer = (unsigned int)TIMER_CLK / 16 / 1000;
    u32Tmp = u32Tmp / u32TimerPer;
    if (u32Tmp >= 39) {
      DEBUG("T:\n");
      PrintInt(8, u32Tmp);
      ENTER();
    }
  }
  if (u8Mode != 1) // start
  {
    XM_MPI_ISP_SetRegister(TIMER1_CONTROL,
                           0x47); // Int  24/16 M	32Bit  oneShot
    XM_MPI_ISP_SetRegister(TIMER1_BGLOAD, u32TimerPer);
    XM_MPI_ISP_SetRegister(TIMER1_LOAD, u32TimerPer);

    // Timer4
    u32Tmp = 0;
    XM_MPI_ISP_GetRegister(TIMER1_CONTROL, &u32Tmp);
    XM_MPI_ISP_SetRegister(TIMER1_CONTROL, (u32Tmp | (1 << 7))); // Start Timer4
    su32StartVal = u32TimerPer;
  }
#endif
#endif
}

XM_U8 *Get_CfgBufferAddr() {
  XM_U8 *pu8Buff;
  pu8Buff = (XM_U8 *)BUFFER_START;

//#ifdef SOC_SYSTEM
#if 0
  XM_S32 s32Ret;
#if 0
  s32Ret = readFile(CFG_FILE, gau8Buffer, 4096);
  if (s32Ret < 0) {
    ERR("readFile failed!\n");
  }
  pu8Buff = gau8Buffer;
#else
  XM_U8 au8Buffer[6144];

  s32Ret = readFile(CFG_FILE, au8Buffer, 6144);
  if (s32Ret < 0) {
    ERR("readFile failed!\n");
  }

  ExtCfg_Write_V2(6144, 0, au8Buffer);
#endif
#endif
  return pu8Buff;
}

#define REG_FLAG (0x20000F2C)
void SysFlagSet(XM_U32 u32Flag) {
#ifdef SOC_NONE
  XM_MPI_ISP_SetRegister(REG_FLAG, u32Flag);
#endif
}
