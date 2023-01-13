/*********************************************************************
Copyright (C), 2015-8-26, JuFeng.Tech. Co., Ltd.
File name: 	com.h
Author:		
Versoin: 	       1.00
Data: 		2015-08-26
Desc:		����Դ�ļ���ʵ�ִ���ͨ��
Ohters:		// ����˵��
Function List:	

**********************************************************************/
#ifndef __COM_H__
#define __COM_H__

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#define uint8 unsigned char
#define uint32 unsigned int
#define uint16 unsigned short
#define ISP_USED	(0)
#define UART_0		(0)
#define COM_TimeOut0		(0x00000001) 
#define COM_TimeOut1		(0x00010000) 
typedef enum {
    E_RECV_OK,
    E_RECV_TIMEOUT
}eCOMERR;


#define TRUE			1
#define FALSE			0



/**************************** ���ú��������� *******************************/

/*
 ָ���
 */
#define COM_CMDLEN			13	//( AA BB CC CC CC CC DD DD DD DD xx xx xx) 

// ���ڴ���ģʽ
#define CMD_DEBUGMODE		0XFE
#define CMD_BOOTMODE		0x95
#define CMD_VISCAMIN_MODE		0x80
#define CMD_VISCAMAX_MODE		0x8F

/***** DEBUG ģʽָ��� *****/

#define CMD_DEBUG_REGWRTIE		0x05
#define CMD_DEBUG_REGREAD		0x06


#define CMD_SENSOR_WRITE        0x07
#define CMD_SENSOR_READ         0x08

#define CMD_MOTOR_WRITE			0x09
#define CMD_MOTOR_READ			0x0A

#define CMD_SPI_WRITE			0x0B
#define CMD_SPI_READ			0x0C

#define CMD_FLASH_WRITE			0x0F
#define CMD_FLASH_READ			0x10

#define CMD_DEBUG_FLASHWRTIE	0x0aa   //0x
#define CMD_DEBUG_FLASHREAD		0x0ab
#define CMD_DEBUG_FLASHERASE	0x0ac

#define CMD_DEBUG_TIMEOUTPARA_SET	0x01A
#define CMD_DEBUG_FLASHPARA_SET		0x01B

#define CMD_DEBUG_BAUDRATE_SET		0x01C
#define CMD_DEBUG_BOOTMODE_EN_SET	0x0D
#define CMD_DEBUG_FLASHPRO_SET		0x0E

#define CMD_DEBUG_AHD			0x5a

/*
UART ���ԣ�
1��REG ��д��
2��FLASH byte��д(��У�飬 ʹ��)��
3��FLASH BLOCK ERASE(��У��, ʹ��)��

4����ģ�鳬ʱ������ѯ������(��У��)��
5��FLASH �������ã�����ָ������ã�(��У��)��

6������������(��У��)��Ĭ�ϲ�����9600��;
7��UART BootLoaderģʽʹ�ܣ�����λ�����ƣ�(��У��)��
8��FLASHд�������ܣ�����λ�����ƣ�(��У��)��
      ����Ϊchip protect;
      �ֶϵ����Ͷϵ粻�������֣�

*/




/***** bootloader ģʽ���� *****/
#define CMD_BOOT_FLASH_WRITE		0x22
#define	CMD_BOOT_FLASH_READ			0x42

#define	CMD_BOOT_FLASH_CHECKSUM		0x52
#define	CMD_BOOT_FLASH_BLANKCHECK	0x62

#define	CMD_BOOT_FLASH_ERASE		0x82
#define CMD_BOOT_CHIP_ERASE			0xA2







/******************��������**************************/







/******************��������**************************/

void Com_Debugprocess_Anomaly(uint8 uart_NO);
void Com_FlashProcess(uint8 uart_NO);

uint8 getbyte(uint32 timeout);
void ISR_UART0(void);
void putdword(unsigned char ComNum, unsigned int c);




#endif

