/******************************************************************************
 Copyright (C), 2015-2020, XM. Co., Ltd.
******************************************************************************
File Name	: mpi_smartVda.h  (libsmart.a  mpi)
Version 		: Initial Draft
Author		: XM Isp software group
Created 		: 2017/9/25

Description 	: The common data type defination
Function List	:
History :
1.Date		: 2017/9/25
  Author		: Lycai
  Modification	: creat
******************************************************************************/
#ifndef _MPI_SMARTVDA_H_
#define _MPI_SMARTVDA_H_
#include "xm_type.h"



typedef struct stResult
{
    XM_U32 uiFlag;    /*1����Ŀ��  0����Ŀ��*/
	XM_U8 reserved[16];
}Move_Detect_Result_S;

typedef struct xmMove_Detect_Para_S//�ƶ�����������
{
	XM_U8 Enable;//���ʹ�� 0:��ʹ�� 1:ʹ��
	XM_U8 Sensitivity;//������
	XM_U8 reserved[16];
}Move_Detect_Para_S;




//�����ƶ������Ľӿ�
XM_S32 MoveDetect_Creat(int nChannel);
XM_S32 MoveDetect_SetParam(int nChannel,Move_Detect_Para_S *pstParam);
XM_S32 MoveDetect_GetResult(int nChannel,Move_Detect_Result_S *pstResult);//���� 1:��Ŀ��  0:��Ŀ��
XM_S32 MoveDetect_Destory(int nChannel);








#endif


