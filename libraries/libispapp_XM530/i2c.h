/*********************************************************************
Copyright (C), 2015-8-26, JuFeng.Tech. Co., Ltd.
File name: 	i2c.h
Author:
Versoin: 	       1.00
Data: 		2015-08-26
Desc:
Ohters:		// 其他说明
Function List:

**********************************************************************/

#ifndef __I2C_H__
#define __I2C_H__

typedef unsigned char __u8;
typedef unsigned int __u16;

#define __16BIT_REGISTER 1
#define __16BIT_DATA 1

#define WRDATA (1)
#define RDDATA (2)

#define I2C_REGS_ADDR_BASE 0x10000000
#define I2C_REGS_CTRL (I2C_REGS_ADDR_BASE + 0x00)
#define I2C_REGS_TRANS_ORDER (I2C_REGS_ADDR_BASE + 0x04)
#define I2C_REGS_BUS_WIDTH (I2C_REGS_ADDR_BASE + 0x08)
#define I2C_REGS_ADDRESS (I2C_REGS_ADDR_BASE + 0x0C)
#define I2C_REGS_TXFIFO (I2C_REGS_ADDR_BASE + 0x10)
#define I2C_REGS_RXFIFO (I2C_REGS_ADDR_BASE + 0x14)
#define I2C_REGS_TXNUM (I2C_REGS_ADDR_BASE + 0x18)
#define I2C_REGS_RXNUM (I2C_REGS_ADDR_BASE + 0x1C)
#define I2C_REGS_THIGH (I2C_REGS_ADDR_BASE + 0x20)
#define I2C_REGS_TLOW (I2C_REGS_ADDR_BASE + 0x24)
#define I2C_REGS_TSU_STA (I2C_REGS_ADDR_BASE + 0x28)
#define I2C_REGS_THD_STA (I2C_REGS_ADDR_BASE + 0x2C)
#define I2C_REGS_TSU_STOP (I2C_REGS_ADDR_BASE + 0x30)
#define I2C_REGS_TDAT_CHG (I2C_REGS_ADDR_BASE + 0x34)
#define I2C_REGS_TDAT_SMP (I2C_REGS_ADDR_BASE + 0x38)
#define I2C_REGS_WAIT_TIME (I2C_REGS_ADDR_BASE + 0x3C)
#define I2C_REGS_INT_CLR (I2C_REGS_ADDR_BASE + 0x44)
#define I2C_REGS_SOURCE_INT (I2C_REGS_ADDR_BASE + 0x48)

#define I2C_REGS_RXFIFO_NUM (I2C_REGS_ADDR_BASE + 0x54)

#define TRUE 1
#define FALSE 0
#define IICTimeOut 0x4000

void I2C_Init(void);

typedef struct i2c_regs {
  volatile unsigned long iic_ctrl;        // 0xffff9000
  volatile unsigned long iic_trans_order; // 0xffff9004
  volatile unsigned long iic_bus_width;   // 0xffff9008
  volatile unsigned long iic_address;     // 0xffff900c
  volatile unsigned long iic_txfifo;      // 0xffff9010
  volatile unsigned long iic_rxfifo;      // 0xffff9014
  volatile unsigned long iic_tx_num;      // 0xffff9018
  volatile unsigned long iic_rx_num;      // 0xffff901c
  volatile unsigned long iic_thigh;       // 0xffff9020
  volatile unsigned long iic_tlow;        // 0xffff9024
  volatile unsigned long iic_tsu_sta;     // 0xffff9028
  volatile unsigned long iic_thd_sta;     // 0xffff902c
  volatile unsigned long iic_tsu_stop;    // 0xffff9030
  volatile unsigned long iic_tdat_chg;    // 0xffff9034
  volatile unsigned long iic_tdat_smp;    // 0xffff9038
  volatile unsigned long iic_wait_time;   // 0xffff903c
  volatile unsigned long iic_int_mask;    // 0xffff9040
  volatile unsigned long iic_int_clr;     // 0xffff9044
  volatile unsigned long iic_source_int;  // 0xffff9048
  volatile unsigned long iic_int_reg;     // 0xffff904c
  volatile unsigned long iic_txfifo_num;  // 0xffff9050
  volatile unsigned long iic_rxfifo_num;  // 0xffff9054
} I2C_REG_S;

#endif
