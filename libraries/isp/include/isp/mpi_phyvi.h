/******************************************************************************
 Copyright (C), 2015-2020, XM. Co., Ltd.
******************************************************************************
File Name	: mpi_mipi.h
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
#ifndef __MPI_MIPI_H__
#define __MPI_MIPI_H__
#include "xm_defines.h"
#include "xm_common.h"


typedef struct _mipi_dev_attr_s
{
	XM_MIPI_LANE       	lane;                 /* MIPI lane num */
    XM_SENSOR_BWIDE     depth;          /* Depth: 8/10/12/ bit */
    
	XM_U32   snsMpInclk;	//�͸�sensor��mclkʱ��
	XM_U32   snsMpOutclk;	//sensor�����mipiʱ��
	XM_U32   snsAllLine;	//sensor���������
	XM_U32   snsAllPixs;	//sensor����ܵ���

	XM_U32   snsActiveLine;	//sensor�����Ч����
	XM_U32   snsActivePixs;	//sensor�����Ч����

	XM_U32   bMpDvpclk;	//оƬ�ڲ�����ȡ��ʱ��
	XM_U32   bAllPixs;	//оƬ�ڲ�����ȡ���ܵ���
	XM_U32   delay;		//mipi�ڲ�delay
	XM_U32   MipiCtrl;	//[23:16]:ctrl_dly(��� sotʱ�� delay:ctrl_dly*2);[7:0]:sot_data(B8);[8]:clk_ctrl(0:CKp;1:CKn) [15:9]:����
}MIPI_DEV_ATTR_S;


typedef struct _combo_dev_attr_s
{
	XM_SENSOR_CONT input_mode;
	MIPI_DEV_ATTR_S mipi_attr;
}COMBO_DEV_ATTR_S;

typedef enum xm_mipi_cmd
{
	MIPI_SET_DEV_ATTR = 0,
	MIPI_SET_OUTPUT_CLK_EDGE = 1,
	MIPI_RESET_MIPI = 2,
	MIPI_UNRESET_MIPI = 3,
	MIPI_SET_RUN = 4,
//	MIPI_RESET_SENSOR = 5,
//	MIPI_UNRESET_SENSOR = 6,
	MIPI_BUTT
}MIPI_CMD;

typedef struct _xm_mipi_ck_s
{
	unsigned int mipi_ck_div;
	unsigned int mipi_ck_count;
	unsigned int mipi_ck_all_v;
	unsigned int mipi_ck_all_h;
	unsigned int mipi_ck_en;
}MIPI_CK_CMD;



XM_S32 XM_MPI_MIPI_GetDevAttr(ISP_DEV IspDev, COMBO_DEV_ATTR_S *pstComboDevAttr);
XM_S32 XM_MPI_MIPI_SetDevAttr(ISP_DEV IspDev, MIPI_CMD enCmd, const COMBO_DEV_ATTR_S *pstComboDevAttr);

/*****************************************************************************************************
函数名称: XM_MPI_MIPI_RefreshFV
函数功能: MIPI强制刷新行场信号
输入参数: 		u32DelayMs:刷新时间(ms)
				>0: 配置的时间
				0:  内部自动决定
			u32TotalSizeV
				>0: 总行数
				=0: 仅刷新信号
输出参数:	无
返回参数:		0: 成功
			-1: 失败
*****************************************************************************************************/
XM_S32 XM_MPI_MIPI_RefreshFV(XM_U32 u32DelayMs, XM_U32 u32TotalSizeV);
#endif /*__MPI_ISP_H__ */

	
