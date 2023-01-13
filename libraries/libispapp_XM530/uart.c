#include "com.h"
#include "xm_print.h"
#ifndef SOC_ALIOS
#include <termios.h>

#include <fcntl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>
#endif
#include <string.h>

typedef unsigned int DWORD;

/************************************************************************
 * define dev file
 ************************************************************************/
#define DEV_COM "/dev/ttyAMA0"
#define DEV_FBD "/dev/null"
#define DEV_PTZ "/dev/ttyAMA1"
#define DEV_UART2 "/dev/ttyAMA2"
#define DEV_RTC "/dev/rtc"

#define DEV_PNX "/dev/pnx1500a"
#define DEV_TW NULL
#define DEV_BF "/dev/bf561"
#define DEV_UP "/dev/mtd1"
#define DEV_BAK "/dev/mtdblock2"
#define DEV_DOG "/dev/wdt2510"
#define DEV_MOUSE "/dev/psaux"
#define DEV_FLASH "/dev/misc/hi-flash"
#define DEV_ALM "/dev/misc/alarm"
//#define DEV_MOTOR		"/dev/misc/motor"
#define DEV_KEYBOARD "/dev/input/event0"

#define COMM_PURGE_TXABORT 0x0001 ///< 中止写操作
#define COMM_PURGE_RXABORT 0x0002 ///< 中止读操作
#define COMM_PURGE_TXCLEAR 0x0004 ///< 清空输出缓冲
#define COMM_PURGE_RXCLEAR 0x0008 ///< 清空输入缓冲

/// 串口属性结构
typedef struct COMM_ATTR {
  unsigned int baudrate;  ///< 实际的波特率值。
  unsigned char databits; ///< 实际的数据位数。
  unsigned char parity; ///< 奇偶校验选项，取comm_parity_t类型的枚举值。
  unsigned char stopbits; ///< 停止位数，取comm_stopbits_t类型的枚举值。
  unsigned char reserved; ///< 保留
} COMM_ATTR;

/// 串口停止位类型
enum comm_stopbits_t {
  COMM_ONESTOPBIT,   ///< 1 stop bit
  COMM_ONE5STOPBITS, ///< 1.5 stop bit
  COMM_TWOSTOPBITS   ///< 2 stop bit
};

/// 串口校验位类型
enum comm_parity_t {
  COMM_NOPARITY,   ///< No parity
  COMM_ODDPARITY,  ///< Odd parity
  COMM_EVENPARITY, ///< Even parity
  COMM_MARK,       ///<
  COMM_SPACE       ///<
};

#define IN_BUF_LEN 128 // 软件BUFFER大小定义

static volatile uint8 InBuf0[IN_BUF_LEN] = {0}; // 软件BUFFER
static volatile uint8 RxBufFull0 = 0;           // 软件BUFFER装满标志
static volatile uint8 RxBufHaveData0 = 0; // 软件BUFFER有无数据标志
static volatile uint8 inlast0 = 0;        // 软件BUFFER接收计数变量
static volatile uint8 getlast0 = 0;       // 软件BUFFER取数计数变量

extern eCOMERR eComErr;

#ifndef SOC_ALIOS
static int fd_com = -1;
static int g_comflag = 1; // default as general uart

int SetAttribute(int dev_fd, COMM_ATTR *pattr) {
  struct termios opt;
  COMM_ATTR *attr = pattr;

  if (dev_fd < 0) {
    return -1;
  }

  memset(&opt, 0, sizeof(struct termios));
  tcgetattr(dev_fd, &opt);
  cfmakeraw(&opt); /* set raw mode	*/

  /*
   * set speed
   */
  switch (attr->baudrate) {
  case 50:
    cfsetispeed(&opt, B50);
    cfsetospeed(&opt, B50);
    break;
  case 75:
    cfsetispeed(&opt, B75);
    cfsetospeed(&opt, B75);
    break;
  case 110:
    cfsetispeed(&opt, B110);
    cfsetospeed(&opt, B110);
    break;
  case 134:
    cfsetispeed(&opt, B134);
    cfsetospeed(&opt, B134);
    break;
  case 150:
    cfsetispeed(&opt, B150);
    cfsetospeed(&opt, B150);
    break;
  case 200:
    cfsetispeed(&opt, B200);
    cfsetospeed(&opt, B200);
    break;
  case 300:
    cfsetispeed(&opt, B300);
    cfsetospeed(&opt, B300);
    break;
  case 600:
    cfsetispeed(&opt, B600);
    cfsetospeed(&opt, B600);
    break;
  case 1200:
    cfsetispeed(&opt, B1200);
    cfsetospeed(&opt, B1200);
    break;
  case 1800:
    cfsetispeed(&opt, B1800);
    cfsetospeed(&opt, B1800);
    break;
  case 2400:
    cfsetispeed(&opt, B2400);
    cfsetospeed(&opt, B2400);
    break;
  case 4800:
    cfsetispeed(&opt, B4800);
    cfsetospeed(&opt, B4800);
    break;
  case 9600:
    cfsetispeed(&opt, B9600);
    cfsetospeed(&opt, B9600);
    break;
  case 19200:
    cfsetispeed(&opt, B19200);
    cfsetospeed(&opt, B19200);
    break;
  case 38400:
    cfsetispeed(&opt, B38400);
    cfsetospeed(&opt, B38400);
    break;
  case 57600:
    cfsetispeed(&opt, B57600);
    cfsetospeed(&opt, B57600);
    break;
  case 115200:
    cfsetispeed(&opt, B115200);
    cfsetospeed(&opt, B115200);
    break;
  case 230400:
    cfsetispeed(&opt, B230400);
    cfsetospeed(&opt, B230400);
    break;
  case 460800:
    cfsetispeed(&opt, B460800);
    cfsetospeed(&opt, B460800);
    break;
  case 500000:
    cfsetispeed(&opt, B500000);
    cfsetospeed(&opt, B500000);
    break;
  case 576000:
    cfsetispeed(&opt, B576000);
    cfsetospeed(&opt, B576000);
    break;
  case 921600:
    cfsetispeed(&opt, B921600);
    cfsetospeed(&opt, B921600);
    break;
  case 1000000:
    cfsetispeed(&opt, B1000000);
    cfsetospeed(&opt, B1000000);
    break;
  case 1152000:
    cfsetispeed(&opt, B1152000);
    cfsetospeed(&opt, B1152000);
    break;
  case 1500000:
    cfsetispeed(&opt, B1500000);
    cfsetospeed(&opt, B1500000);
    break;
  case 2000000:
    cfsetispeed(&opt, B2000000);
    cfsetospeed(&opt, B2000000);
    break;
  case 2500000:
    cfsetispeed(&opt, B2500000);
    cfsetospeed(&opt, B2500000);
    break;
  case 3000000:
    cfsetispeed(&opt, B3000000);
    cfsetospeed(&opt, B3000000);
    break;
  case 3500000:
    cfsetispeed(&opt, B3500000);
    cfsetospeed(&opt, B3500000);
    break;
  case 4000000:
    cfsetispeed(&opt, B4000000);
    cfsetospeed(&opt, B4000000);
    break;
  default:
    ERR("unsupported baudrate\n");
    break;
  }

  /*
   * set parity
   */
  switch (attr->parity) {
  case COMM_NOPARITY:       /* none			*/
    opt.c_cflag &= ~PARENB; /* disable parity	*/
    opt.c_iflag &= ~INPCK;  /* disable parity check	*/
    break;
  case COMM_ODDPARITY:     /* odd			*/
    opt.c_cflag |= PARENB; /* enable parity	*/
    opt.c_cflag |= PARODD; /* odd			*/
    opt.c_iflag |= INPCK;  /* enable parity check	*/
    break;
  case COMM_EVENPARITY:     /* even			*/
    opt.c_cflag |= PARENB;  /* enable parity	*/
    opt.c_cflag &= ~PARODD; /* even			*/
    opt.c_iflag |= INPCK;   /* enable parity check	*/
  default:
    ERR("unsupported parity\n");
    break;
  }

  /*
   * set data bits
   */
  opt.c_cflag &= ~CSIZE;
  switch (attr->databits) {
  case 5:
    opt.c_cflag |= CS5;
    break;
  case 6:
    opt.c_cflag |= CS6;
    break;
  case 7:
    opt.c_cflag |= CS7;
    break;
  case 8:
    opt.c_cflag |= CS8;
    break;
  default:
    ERR("unsupported data bits\n");
    break;
  }

  /*
   * set stop bits
   */
  opt.c_cflag &= ~CSTOPB;
  switch (attr->stopbits) {
  case COMM_ONESTOPBIT:
    opt.c_cflag &= ~CSTOPB;
    break;
    /*		case COMM_ONE5STOPBIT:
                            break;
    */
  case COMM_TWOSTOPBITS:
    opt.c_cflag |= CSTOPB;
    break;
  default:
    ERR("unsupported stop bits\n");
    break;
  }
  opt.c_cc[VTIME] = 0;
  opt.c_cc[VMIN] = 1; /* block until data arrive */

  tcflush(dev_fd, TCIOFLUSH);
  if (tcsetattr(dev_fd, TCSANOW, &opt) < 0) {
    ERR("tcsetattr\n");
    return -1;
  }

  return 0;
} /* end SetAttribute */

/************************************************************************
 * Set com device attribute
 *
 * ret :	= 0 success
 * 		< 0 false
 ************************************************************************/
int CommSetAttribute(COMM_ATTR *pattr) {
  if (g_comflag == 0) {
    return 0;
  }

  if (fd_com < 0) {
    fd_com = open(DEV_COM, O_RDWR | O_NONBLOCK);
    if (fd_com < 0) {
      ERR("Can't Open Com Dev");
      return -1;
    }
  }

  return SetAttribute(fd_com, pattr);
}

/************************************************************************
 * Open com device
 *
 * ret :	> 0  bytes by read
 * 		<= 0 false
 ************************************************************************/
int CommRead(void *pdata, DWORD nbytes) {
  if (g_comflag == 0) {
    return 0;
  }

  if (fd_com < 0) {
    fd_com = open(DEV_COM, O_RDWR | O_NONBLOCK);
    if (fd_com < 0) {
      ERR("Can't Open Com Dev");
      return -1;
    }
  }

  return read(fd_com, pdata, nbytes);
}

/************************************************************************
 * write data
 *
 * ret :	> 0  bytes by write
 * 		<= 0 false
 ************************************************************************/
int CommWrite(void *pdata, DWORD len) {
  if (g_comflag == 0) {
    return 0;
  }

  if (fd_com < 0) {
    fd_com = open(DEV_COM, O_RDWR | O_NONBLOCK);
    if (fd_com < 0) {
      ERR("Can't Open Com Dev");
      return -1;
    }
  }

  return write(fd_com, pdata, len);
}
#else
extern int CommWrite(void *pdata, DWORD len);
extern int CommRead(void *pdata, DWORD nbytes);
extern int CommSetAttribute(COMM_ATTR *pattr);

#endif

/************************************************************************
** Func Name	: putbyte0
** Discription	:
**
** Parameter	: uint8 c
** Return		: Null
************************************************************************/
uint8 putbyte(uint8 c) {
  CommWrite(&c, 1);
  return TRUE;
}

/************************************************************************
** Func Name	: getbyte(uint32 timeout)
** Discription	:
**
** Parameter	: timeout
** Return		: Null
************************************************************************/

uint8 getbyte(uint32 timeout) {
  uint8 Data;

  eComErr = E_RECV_OK;
  while (1) //查询软件BUFFER中是否有数据
  {
    if (timeout == 0) {
      eComErr = E_RECV_TIMEOUT;
      return 0xFF;
    }

    if (RxBufHaveData0)
      break;

    timeout--;
  }
  //软件BUFFER中有数据
  Data = InBuf0[getlast0];

  getlast0 = (getlast0 + 1) % IN_BUF_LEN;

  if (getlast0 == inlast0) {
    RxBufHaveData0 = 0; // BUF 中没有数据
    RxBufFull0 = 0;     //软件BUFFER中无数据
  }
  return Data;
}

/***********************************************************************
Function:	putdword
Description:串口发送字字符串
Input： 	ComNum:串口选择
                        c:数据
Output：	NO
Return：	NO
Others：	NO
***********************************************************************/
void putdword(unsigned char ComNum, unsigned int c) {
  unsigned char temp[4] = {0};

  temp[0] =
      (unsigned char)((c >> 24) &
                      0xff); //再程序中，从iic中返回的数据被定义为char型，此处temp[0],
                             // temp[1]为空，
  temp[1] = (unsigned char)((c >> 16) & 0xff);
  temp[2] =
      (unsigned char)((c >> 8) & 0xff); // temp[2],temp[3]为真实的返回的数据。
  temp[3] = (unsigned char)((c >> 0) & 0xff);

  CommWrite(temp, 4);
}

#define BUF_SIZE (64)
XM_S32 Init_UART0(XM_VOID) {
  COMM_ATTR stCommAttr;
  stCommAttr.baudrate = 115200;
  stCommAttr.databits = 8;
  stCommAttr.parity = COMM_NOPARITY;
  stCommAttr.stopbits = COMM_ONESTOPBIT;
  CommSetAttribute(&stCommAttr);
  return XM_SUCCESS;
}

void ISR_UART0(void) {
  uint8 au8Buf[BUF_SIZE] = {0};
  int s32NumRsv, s32NumTmp;
  s32NumTmp = 0;
  s32NumRsv = CommRead(au8Buf, BUF_SIZE);
  if (s32NumRsv > 0) {
#if 0
    for (s32NumTmp = 0; s32NumTmp < s32NumRsv; s32NumTmp++) {
      printf("0x%02x ", au8Buf[s32NumTmp]);
    }
    printf("\n");
#endif
    s32NumTmp = 0;
    if (!RxBufFull0) //接收软件BUFF未满
    {
      while (s32NumRsv) //读取数据直到接收FIFO为空
      {
        InBuf0[inlast0] = au8Buf[s32NumTmp++];
        s32NumRsv--;
        RxBufHaveData0 = 1; //有数据

        ++inlast0;

        if (inlast0 == IN_BUF_LEN)
          inlast0 = 0;

        if (inlast0 == getlast0) {
          RxBufFull0 = 1; //接收软件BUFF满
          break;
        }
      }
    }
    // printf("lst: %d %d %d\n", inlast0, getlast0, RxBufHaveData0);
    if (inlast0 != getlast0) //接收软件BUFF满，判断是否有数据取出
    {
      RxBufFull0 = 0; //接收BUFF满
    }
  }
}
