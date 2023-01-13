/******************************************************************************
 Copyright (C), 2015-2020, XM. Co., Ltd.
******************************************************************************
File Name	: xm540_tmp.h
Version 		: Initial Draft
Author		: XM Isp software group
Created 		: 2015/6/27

Description 	: The common data type defination
Function List	:
History :
1.Date		: 2015/6/27
  Author		: Lycai
  Modification	: creat
******************************************************************************/
#ifndef _XM540_TMP_H_
#define _XM540_TMP_H_
#include "xm_i2c.h"
#include "xm_ssp.h"
#include "xm_type.h"
#include "xm_ae_comm.h"
#include "xm_awb_comm.h"

#include "xm_comm_sns.h"
#include "xm_comm_3a.h"

#define VI_BASE		(0x00A00000)

#define VI_H_BEGIN	(VI_BASE+0x098)
#define VI_H_END		(VI_BASE+0x09C)
#define VI_V_BEGIN	(VI_BASE+0x0A0)
#define VI_V_END		(VI_BASE+0x0A4)

/*************************************************************************
��������:	����VI�ü�����(H��V)
�������:	u8Mode:
					0: Real Data(ʵ��ֵ)
					1: ���ڱ�׼ֵ����ƫ��
					2: Refresh
				u8Mirror:
					1: Mirror
				u8Flip:
					1: Flip
				u16ValH: ˮƽ����ֵ(bit15:Ϊ����)    [0, 0xFFFE]
					0~0x7FFF			:
					0x8000 ~ 0xFFFE	:  <0

				u16ValV: ��ֱ����ֵ(bit15:Ϊ����)	  [0, 0xFFFE]
					0~0x7FFF			:
					0x8000 ~ 0xFFFE	:  <0	
note:  
	u16ValH/u16ValV = 0xFFFF ʱ��׼��д��
*************************************************************************/
XM_S32 VI_WinSet(XM_U8 u8Mode, XM_U8 u8Mirror, XM_U8 u8Flip,
						XM_U16 u16ValH, XM_U16 u16ValV);

int XM_I2C_Ioctl(int cmd, I2C_DATA_S *pstI2CData);
int XM_SPI_Ioctl(int cmd, XM_U32* pu32Data);

void I2C_Write(XM_U32 addr, XM_U32 data);
void PrintHex(unsigned char u8Num, XM_U64 u64Data);
void PrintInt(unsigned char u8Num,int u32Data);
void SysDelay_ms(unsigned int nms);

XM_S32 SysGetProductInfo_Ptr(XM_PRODUCT_INFO **pstProductInfo);



/***********************************************************************
��������:	SysReadFromFlash
��������:	��ȡFLash����
�������:	pu8Data: �����ַ
				u32Addr: ��ȡ��ַ
				u32Len:	��ȡ����
�������:	��
���ز���:	1:	�ɹ�
				0: 	����
Note:Lycai
***********************************************************************/
XM_BOOL SysReadFromFlash(XM_U8 *pu8Data, XM_U32 u32Addr, XM_U32 u32Len);
/**********************************************************************
��������:	����ɫͬ��
�������:	u8BurstMode:  0  No CSync
							1  have CSync
				u8ColorMode:	0 BW
							1 Color
�������:	��
���ز���:	0: Success
				-1: Failure
**********************************************************************/
XM_S32 XM_MPI_VENC_SetColor(XM_U8 u8BurstMode, XM_U8 u8ColorMode);

#endif

