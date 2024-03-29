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
    
	XM_U32   snsMpInclk;	//送给sensor的mclk时钟
	XM_U32   snsMpOutclk;	//sensor输出的mipi时钟
	XM_U32   snsAllLine;	//sensor输出总行数
	XM_U32   snsAllPixs;	//sensor输出总点数

	XM_U32   snsActiveLine;	//sensor输出有效行数
	XM_U32   snsActivePixs;	//sensor输出有效点数

	XM_U32   bMpDvpclk;	//芯片内部并行取点时钟
	XM_U32   bAllPixs;	//芯片内部并行取点总点数
	XM_U32   delay;		//mipi内部delay
	XM_U32   MipiCtrl;	//[23:16]:ctrl_dly(检测 sot时间 delay:ctrl_dly*2);[7:0]:sot_data(B8);[8]:clk_ctrl(0:CKp;1:CKn) [15:9]:保留
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
鍑芥暟鍚嶇О: XM_MPI_MIPI_RefreshFV
鍑芥暟鍔熻兘: MIPI寮哄埗鍒锋柊琛屽満淇″彿
杈撳叆鍙傛暟: 		u32DelayMs:鍒锋柊鏃堕棿(ms)
				>0: 閰嶇疆鐨勬椂闂�
				0:  鍐呴儴鑷姩鍐冲畾
			u32TotalSizeV
				>0: 鎬昏鏁�
				=0: 浠呭埛鏂颁俊鍙�
杈撳嚭鍙傛暟:	鏃�
杩斿洖鍙傛暟:		0: 鎴愬姛
			-1: 澶辫触
*****************************************************************************************************/
XM_S32 XM_MPI_MIPI_RefreshFV(XM_U32 u32DelayMs, XM_U32 u32TotalSizeV);
#endif /*__MPI_ISP_H__ */

	
