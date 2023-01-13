/*********************************************************************
Copyright (C), 2015-8-26, JuFeng.Tech. Co., Ltd.
File name: 	i2c.c
Author:
Versoin: 	       1.00
Data: 		2015-08-26
Desc:
Ohters:		// 其他说明
Function List:

**********************************************************************/
#ifdef SOC_SYSTEM
#include <fcntl.h>
#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#elif (defined SOC_ALIOS)
#include <fcntl.h>
#include <pthread.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#endif
#include "i2c.h"
#include "xm530_tmp.h"
#include "xm_i2c.h"
#include "xm_print.h"
#include "xm_type.h"

// static volatile I2C_REG_S *i2c_regs = (I2C_REG_S *)I2C_REGS_ADDR_BASE;

//#ifdef CHIP_FPGA
#if 0
void I2C_Init(void) {
#ifdef SOC_SYSTEM
  XM_MPI_ISP_SetRegister(I2C_REGS_CTRL, 0x01);
  XM_MPI_ISP_SetRegister(I2C_REGS_THIGH, 0xA8);
  XM_MPI_ISP_SetRegister(I2C_REGS_TLOW, 0xA8);
  XM_MPI_ISP_SetRegister(I2C_REGS_TSU_STA, 0xA8);
  XM_MPI_ISP_SetRegister(I2C_REGS_THD_STA, 0xA8);
  XM_MPI_ISP_SetRegister(I2C_REGS_TSU_STOP, 0xA8);
  XM_MPI_ISP_SetRegister(I2C_REGS_TDAT_CHG, 0x50);
  XM_MPI_ISP_SetRegister(I2C_REGS_TDAT_SMP, 0xFC);
  XM_MPI_ISP_SetRegister(I2C_REGS_ADDRESS, 0x10);
#else
  i2c_regs->iic_ctrl = 0x01;
  i2c_regs->iic_thigh = 0xA8;
  i2c_regs->iic_tlow = 0xA8;
  i2c_regs->iic_tsu_sta = 0xA8;
  i2c_regs->iic_thd_sta = 0xA8;
  i2c_regs->iic_tsu_stop = 0xA8;
  i2c_regs->iic_tdat_chg = 0x50;
  i2c_regs->iic_tdat_smp = 0xFC;
  i2c_regs->iic_address =
      0x10; // slave address  为0x20的bit1 ~
            // bit7位，即0x20右移一位后得到的值0x10   为slave  address.
#endif
  DEBUG("I2C_Init Ok! ~~\n");
}

static unsigned char Wait_Read_Statue(void) {
  volatile unsigned long time = 0;
#ifdef SOC_SYSTEM
  XM_U32 u32Val;
  XM_MPI_ISP_GetRegister(I2C_REGS_CTRL, &u32Val);
  XM_MPI_ISP_SetRegister(I2C_REGS_CTRL, u32Val | (1) | (1 << 1) | (1 << 11));
  while (1) {
    XM_MPI_ISP_GetRegister(I2C_REGS_SOURCE_INT, &u32Val);
    if (u32Val & (1 << 5))
      break;
    if (time++ > IICTimeOut) {
      XM_MPI_ISP_SetRegister(I2C_REGS_CTRL, 0x01);
      XM_MPI_ISP_SetRegister(I2C_REGS_INT_CLR, 0xFF);
      return FALSE;
    }
  }
#else
  i2c_regs->iic_ctrl |= (1) | (1 << 1) | (1 << 11); //// 读操作
  /* 等待直至数据传输完毕 */
  while (!(i2c_regs->iic_source_int & (1 << 5) ? 1 : 0)) {
    if (time++ > IICTimeOut) {
      i2c_regs->iic_ctrl = 0x01;
      i2c_regs->iic_int_clr = 0xFF;
      //		    putstr(0,"\n\r Return I2c read false");
      return FALSE;
    }
  }
#endif
  return TRUE;
}

static unsigned char Wait_Write_Statue(void) {
  unsigned char sRet = 1;
  volatile unsigned long time = 0;
#ifdef SOC_SYSTEM
  XM_U32 u32Val;
  XM_MPI_ISP_GetRegister(I2C_REGS_CTRL, &u32Val);
  XM_MPI_ISP_SetRegister(I2C_REGS_CTRL, u32Val | (1 << 1) | (1 << 3));
  while (1) {
    XM_MPI_ISP_GetRegister(I2C_REGS_SOURCE_INT, &u32Val);
    if (u32Val & (1 << 4))
      break;
    if (time++ > IICTimeOut) {
      XM_MPI_ISP_SetRegister(I2C_REGS_CTRL, 0x01);
      XM_MPI_ISP_SetRegister(I2C_REGS_INT_CLR, 0xFF);
      sRet = 0;
      break;
    }
  }
  XM_MPI_ISP_GetRegister(I2C_REGS_CTRL, &u32Val);
  u32Val |= (1 << 6);
  XM_MPI_ISP_SetRegister(I2C_REGS_CTRL, u32Val);

  XM_MPI_ISP_GetRegister(I2C_REGS_CTRL, &u32Val);
  u32Val &= ~(1 << 6);
  XM_MPI_ISP_SetRegister(I2C_REGS_CTRL, u32Val);

  XM_MPI_ISP_GetRegister(I2C_REGS_INT_CLR, &u32Val);
  XM_MPI_ISP_SetRegister(I2C_REGS_INT_CLR, u32Val | (1 << 4));
#else
  i2c_regs->iic_ctrl |= (1 << 1) | (1 << 3); //// 写操作
  /* 等待直至数据传输完毕 */
  while (!(
      i2c_regs->iic_source_int & (1 << 4)
          ? 1
          : 0)) //数据发送完后会产生txfifo传输完成中断,具体的是iic_source_int的bit4置1.
                //注意此处不是产生fifo空中断。
  {
    if (time++ > IICTimeOut) {
      i2c_regs->iic_ctrl = 0x01;
      i2c_regs->iic_int_clr = 0xFF;
      putstr(0, "I2C TimeOut!\r\n");
      sRet = 0;
      break;
    }
  }
  i2c_regs->iic_ctrl |= (1 << 6); //  高电平删除txfifo中的数据
  i2c_regs->iic_ctrl &=
      ~(1 << 6); //  清完txfifo中的数据后再将对应位恢复为低电平。
  i2c_regs->iic_int_clr |= (1 << 4); //  清中断
#endif
  return sRet;
}

int XM_I2C_Ioctl(int cmd, I2C_DATA_S *pstI2CData) {
  XM_U8 u8i;
  XM_U32 u32Val;
#ifdef SOC_SYSTEM
  XM_MPI_ISP_SetRegister(I2C_REGS_TXNUM,
                         pstI2CData->addr_byte_num + pstI2CData->data_byte_num);
  XM_MPI_ISP_SetRegister(I2C_REGS_ADDRESS, pstI2CData->dev_addr);
  if (cmd == CMD_I2C_WRITE) {
    for (u8i = pstI2CData->addr_byte_num; u8i > 0;) {
      u8i--;
      XM_MPI_ISP_SetRegister(I2C_REGS_TXFIFO,
                             (pstI2CData->reg_addr >> (u8i * 8)) & 0xff);
    }
    for (u8i = pstI2CData->data_byte_num; u8i > 0;) {
      u8i--;
      XM_MPI_ISP_SetRegister(I2C_REGS_TXFIFO,
                             (pstI2CData->data >> (u8i * 8)) & 0xff);
    }
    XM_MPI_ISP_SetRegister(I2C_REGS_TXFIFO, 0x00);
    Wait_Write_Statue();
  } else if (cmd == CMD_I2C_READ) {
    XM_MPI_ISP_SetRegister(I2C_REGS_TXNUM, pstI2CData->addr_byte_num);
    XM_MPI_ISP_SetRegister(I2C_REGS_RXNUM, pstI2CData->data_byte_num);

    for (u8i = pstI2CData->addr_byte_num; u8i > 0;) {
      u8i--;
      XM_MPI_ISP_SetRegister(I2C_REGS_TXFIFO,
                             (pstI2CData->reg_addr >> (u8i * 8)) & 0xff);
    }
    XM_MPI_ISP_SetRegister(I2C_REGS_TXFIFO, 0x00);
    Wait_Write_Statue();
    Wait_Read_Statue();
    for (u8i = pstI2CData->data_byte_num, pstI2CData->data = 0x00; u8i > 0;
         u8i--) {
      XM_MPI_ISP_GetRegister(I2C_REGS_RXFIFO, &u32Val);
      pstI2CData->data =
          (pstI2CData->data << (8 * (pstI2CData->data_byte_num - u8i))) |
          u32Val;
    }
    XM_MPI_ISP_GetRegister(I2C_REGS_CTRL, &u32Val);
    XM_MPI_ISP_SetRegister(I2C_REGS_CTRL,
                           u32Val | (1 << 7)); // 高电平删除rxfifo中的数据

    while (1) {
      XM_MPI_ISP_GetRegister(I2C_REGS_RXFIFO_NUM, &u32Val);
      if (u32Val == 0)
        break;
    }
    XM_MPI_ISP_GetRegister(I2C_REGS_CTRL, &u32Val);
    XM_MPI_ISP_SetRegister(I2C_REGS_CTRL,
                           u32Val & ~(1 << 7)); // 删除完成后恢复相应位为低电平
    XM_MPI_ISP_GetRegister(I2C_REGS_INT_CLR, &u32Val);
    XM_MPI_ISP_SetRegister(I2C_REGS_INT_CLR,
                           u32Val | (1 << 5)); // 清rxfifo读完成中断
  }
#else
  static volatile I2C_REG_S *pstI2CReg = (I2C_REG_S *)I2C_REGS_ADDR_BASE;

  pstI2CReg->iic_tx_num = pstI2CData->addr_byte_num + pstI2CData->data_byte_num;
  pstI2CReg->iic_address = pstI2CData->dev_addr;

  if (cmd == CMD_I2C_WRITE) {
    for (u8i = pstI2CData->addr_byte_num; u8i > 0;) {
      u8i--;
      pstI2CReg->iic_txfifo = (pstI2CData->reg_addr >> (u8i * 8)) & 0xff;
    }
    for (u8i = pstI2CData->data_byte_num; u8i > 0;) {
      u8i--;
      pstI2CReg->iic_txfifo = (pstI2CData->data >> (u8i * 8)) & 0xff;
    }
    pstI2CReg->iic_txfifo = 0x00;
    Wait_Write_Statue();
  } else if (cmd == CMD_I2C_READ) {
    pstI2CReg->iic_tx_num = pstI2CData->addr_byte_num;
    pstI2CReg->iic_rx_num = pstI2CData->data_byte_num;

    for (u8i = pstI2CData->addr_byte_num; u8i > 0;) {
      u8i--;
      pstI2CReg->iic_txfifo = (pstI2CData->reg_addr >> (u8i * 8)) & 0xff;
    }
    pstI2CReg->iic_txfifo = 0x00;
    Wait_Write_Statue();
    Wait_Read_Statue();
    for (u8i = pstI2CData->data_byte_num, pstI2CData->data = 0x00; u8i > 0;
         u8i--) {
      pstI2CData->data =
          (pstI2CData->data << (8 * (pstI2CData->data_byte_num - u8i))) |
          (pstI2CReg->iic_rxfifo);
    }
    pstI2CReg->iic_ctrl |= (1 << 7); // 高电平删除rxfifo中的数据
    while (1) {
      XM_MPI_ISP_GetRegister(I2C_REGS_RXFIFO_NUM, &u32Val);
      if (u32Val == 0)
        break;
    }
    pstI2CReg->iic_ctrl &= ~(1 << 7); // 删除完成后恢复相应位为低电平
    pstI2CReg->iic_int_clr |= (1 << 5); // 清rxfifo读完成中断
  }
#endif
  return XM_SUCCESS;
}

#else // Chip
#ifdef SOC_SYSTEM
#define I2C_DEV "/dev/xm_i2c"
static XM_S32 gI2CDevFd = -1;
static XM_S32 OpenDev() {
  gI2CDevFd = open(I2C_DEV, 0);
  if (gI2CDevFd < 0) {
    ERR("Open hi_i2c error!\n");
    return -1;
  }
  return 0;
}
static XM_S32 CloseDev() {
  if (gI2CDevFd >= 0) {
    close(gI2CDevFd);
  }
  return 0;
}

void I2C_Init(void) { ; }

int XM_I2C_Ioctl(int cmd, I2C_DATA_S *pstI2CData) {
  XM_S32 s32Ret;
  if (gI2CDevFd < 0) {
    OpenDev();
  }

  if (cmd == CMD_I2C_WRITE) {
    s32Ret = ioctl(gI2CDevFd, CMD_I2C_WRITE, pstI2CData);
    if (s32Ret) {
      ERR("xm_i2c write faild!\n");
      CloseDev();
      return -1;
    }
  } else {
    s32Ret = ioctl(gI2CDevFd, CMD_I2C_READ, pstI2CData);
    if (s32Ret) {
      ERR("xm_i2c read faild!\n");
      CloseDev();
      return -1;
    }
  }
  return XM_SUCCESS;
}
#elif (defined SOC_ALIOS)
extern int i2c_write(unsigned char i2cnum, unsigned char devaddr,
                     unsigned int regaddr, unsigned int reglen,
                     unsigned int data, unsigned int datalen);
extern int i2c_read(unsigned char i2cnum, unsigned char devaddr,
                    unsigned int regaddr, unsigned int reglen,
                    unsigned int datalen);

void I2C_Init(void) { ; }

int XM_I2C_Ioctl(int cmd, I2C_DATA_S *pstI2CData) {
  if (pstI2CData == NULL)
    return XM_FAILURE;

  if (cmd == CMD_I2C_WRITE) {
    i2c_write(0, pstI2CData->dev_addr, pstI2CData->reg_addr,
              pstI2CData->addr_byte_num, pstI2CData->data,
              pstI2CData->data_byte_num);
  } else {
    pstI2CData->data =
        i2c_read(0, pstI2CData->dev_addr, pstI2CData->reg_addr,
                 pstI2CData->addr_byte_num, pstI2CData->data_byte_num);
  }
  return XM_SUCCESS;
}

#else
void I2C_Init(void) {
  i2c_regs->iic_ctrl = 0x01;
  i2c_regs->iic_thigh = 0xA8;
  i2c_regs->iic_tlow = 0xA8;
  i2c_regs->iic_tsu_sta = 0xA8;
  i2c_regs->iic_thd_sta = 0xA8;
  i2c_regs->iic_tsu_stop = 0xA8;
  i2c_regs->iic_tdat_chg = 0x50;
  i2c_regs->iic_tdat_smp = 0xFC;
  i2c_regs->iic_address =
      0x10; // slave address  为0x20的bit1 ~
            // bit7位，即0x20右移一位后得到的值0x10   为slave  address.
  DEBUG("I2C_Init Ok! ~~\n");
}

static unsigned char Wait_Read_Statue(void) {
  volatile unsigned long time = 0;
  i2c_regs->iic_ctrl |= (1) | (1 << 1) | (1 << 11); //// 读操作
  /* 等待直至数据传输完毕 */
  while (!(i2c_regs->iic_source_int & (1 << 5) ? 1 : 0)) {
    if (time++ > IICTimeOut) {
      i2c_regs->iic_ctrl = 0x01;
      i2c_regs->iic_int_clr = 0xFF;
      //		    putstr(0,"\n\r Return I2c read false");
      return FALSE;
    }
  }
  return TRUE;
}

static unsigned char Wait_Write_Statue(void) {
  unsigned char sRet = 1;
  volatile unsigned long time = 0;
  i2c_regs->iic_ctrl |= (1 << 1) | (1 << 3); //// 写操作
  /* 等待直至数据传输完毕 */
  while (!(
      i2c_regs->iic_source_int & (1 << 4)
          ? 1
          : 0)) //数据发送完后会产生txfifo传输完成中断,具体的是iic_source_int的bit4置1.
                //注意此处不是产生fifo空中断。
  {
    if (time++ > IICTimeOut) {
      i2c_regs->iic_ctrl = 0x01;
      i2c_regs->iic_int_clr = 0xFF;
      putstr(0, "I2C TimeOut!\r\n");
      sRet = 0;
      break;
    }
  }
  i2c_regs->iic_ctrl |= (1 << 6); //  高电平删除txfifo中的数据
  i2c_regs->iic_ctrl &=
      ~(1 << 6); //  清完txfifo中的数据后再将对应位恢复为低电平。
  i2c_regs->iic_int_clr |= (1 << 4); //  清中断
  return sRet;
}

int XM_I2C_Ioctl(int cmd, I2C_DATA_S *pstI2CData) {
  XM_U8 u8i;
  XM_U32 u32Val;
  static volatile I2C_REG_S *pstI2CReg = (I2C_REG_S *)I2C_REGS_ADDR_BASE;

  pstI2CReg->iic_tx_num = pstI2CData->addr_byte_num + pstI2CData->data_byte_num;
  pstI2CReg->iic_address = pstI2CData->dev_addr;

  if (cmd == CMD_I2C_WRITE) {
    for (u8i = pstI2CData->addr_byte_num; u8i > 0;) {
      u8i--;
      pstI2CReg->iic_txfifo = (pstI2CData->reg_addr >> (u8i * 8)) & 0xff;
    }
    for (u8i = pstI2CData->data_byte_num; u8i > 0;) {
      u8i--;
      pstI2CReg->iic_txfifo = (pstI2CData->data >> (u8i * 8)) & 0xff;
    }
    pstI2CReg->iic_txfifo = 0x00;
    Wait_Write_Statue();
  } else if (cmd == CMD_I2C_READ) {
    pstI2CReg->iic_tx_num = pstI2CData->addr_byte_num;
    pstI2CReg->iic_rx_num = pstI2CData->data_byte_num;

    for (u8i = pstI2CData->addr_byte_num; u8i > 0;) {
      u8i--;
      pstI2CReg->iic_txfifo = (pstI2CData->reg_addr >> (u8i * 8)) & 0xff;
    }
    pstI2CReg->iic_txfifo = 0x00;
    Wait_Write_Statue();
    Wait_Read_Statue();
    for (u8i = pstI2CData->data_byte_num, pstI2CData->data = 0x00; u8i > 0;
         u8i--) {
      pstI2CData->data =
          (pstI2CData->data << (8 * (pstI2CData->data_byte_num - u8i))) |
          (pstI2CReg->iic_rxfifo);
    }
    pstI2CReg->iic_ctrl |= (1 << 7); // 高电平删除rxfifo中的数据
    while (1) {
      XM_MPI_ISP_GetRegister(I2C_REGS_RXFIFO_NUM, &u32Val);
      if (u32Val == 0)
        break;
    }
    pstI2CReg->iic_ctrl &= ~(1 << 7); // 删除完成后恢复相应位为低电平
    pstI2CReg->iic_int_clr |= (1 << 5); // 清rxfifo读完成中断
  }
  return XM_SUCCESS;
}

#endif
#endif
