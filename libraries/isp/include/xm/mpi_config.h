#ifndef _MPI_CONFIG_H
#define _MPI_CONFIG_H

#include "xm_type.h"
#include "xm_math.h"
#include "mpi_ae.h"
#include "mpi_awb.h"
#include "mpi_isp.h"

#include "IspToolsFlash.h"
//#include "xm540_isp.h"

#include "xm_print.h"

typedef struct _xm_menu_cfgfile_
{
	XM_U8 u8MenuEn;			
	XM_U8 u8MenuExpMode;		
	XM_U8 u8MenuDncMode;		
	XM_U8 u8MenuBlcEn;	    
	XM_U8 u8MenuAeTgt;		  
	XM_U8 u8MenuDncThr;	
	XM_U8 u8MenuWdr;
	XM_U8 u8MenuImgStyle;
	XM_U8 u8MenuAgcEn;	
	XM_U8 u8MenuAgcMax;	
	XM_U8 u8MenuNrlvlCol;
	XM_U8 u8MenuNrlvlBw;
	XM_U8 u8MenuIrcutMode;
	XM_U8 u8MenuWbRb;      
	XM_U8 u8MenuWbMg;      
	XM_U8 u8MenuMirror;		
	XM_U8 u8MenuFlip;			
	XM_U8 u8MenuAntiflicker;
	XM_U8 u8MenuIrcutSwap;
	XM_U8 u8MenuBrightness;
	XM_U8 u8MenuContrast;	
	XM_U8 u8MenuSaturation;
	XM_U8 u8MenuHue;	
	XM_U8 u8MenuAcutance;
	XM_U8 u8MenuSawtooth;
	XM_U8 u8MenuAntiFc; 
	XM_U8 u8MenuBurstBw;
	XM_U8 u8MenuVStdType;
	XM_U8 u8MenuVEncType;
	XM_U8 u8MenuRsltType;
	XM_U8 u8MenuEShutter;	// ����������
	XM_U8 u8MenuHLC;		// ǿ������
}XM_MENU_CFGFILE;

typedef enum {
    MODE_USE_MENU = 0,
    MODE_NO_MENU  = 1,	// DebugMode
} XM_MENU_MODE;


/**********************************************************************
��������:	����ģʽ(����/��ȡ)
�������:	enMode:
					0: 	����ʹ�ò˵��ļ�
					1:	�޲˵�(����ģʽ)
�������:	penMode:
					0: 	����ʹ�ò˵��ļ�
					1:	�޲˵�(����ģʽ)
���ز���:	-1		: ʧ��
				����	: �ɹ�
Note:			Lycai
**********************************************************************/
XM_S32 XM_MPI_MENU_SetMode(XM_MENU_MODE enMode);
XM_S32 XM_MPI_MENU_GetMode(XM_MENU_MODE *penMode);


/**********************************************************************
��������:	����˵������ļ�(=> NowConfigFile)
�������:	u32Cmd:
					0		: ALL
					����	: ����ĳ��
				s32Data:
					-1:		��ȡ��ǰȻ��д��
					>0:		ֱ��д��ָ��ֵ
�������:	��
���ز���:	-1		: ʧ��
				����	: �ɹ�
Note:			Lycai
**********************************************************************/
XM_S32 XM_MPI_MENU_SaveCfg(XM_U32 u32Cmd, XM_S32 s32Data);


/****************************************************************************
��������:	��������(=> default ConfigFile)
�������:	pu32Cmd:	���
				pu16Data:	������ֵ
				u8Num:		�������
�������:	��
���ز���:	0: Success
				-1: Failure
				>0:	WriteNum
Note:			Lycai
****************************************************************************/
XM_S32 XM_MPI_MENU_SaveDefCfg(XM_U32 *pu32Cmd, XM_U16 *pu16Data, XM_U8 u8Num);


/**********************************************************************
��������:	����˵������ļ�(=> NowConfiFile)
�������:	u8Mode:	0	To NowCofnig(will Save to Falsh)
						1	To DefaultConfig(will Save to Falsh)
						0x10:	To NowCofnig(Not Save)
				u32Cmd:		ָ��(��ַ)
				*pu32Data:	��Ӧֵ
				
�������:	��
���ز���:	-1		: ʧ��
				����	: �ɹ�
Note:			Lycai
**********************************************************************/
XM_S32 XM_MPI_MENU_SetCfg(XM_U8 u8Mode,
										XM_U32 u32Cmd, XM_U32 *pu32Data);


/**********************************************************************
��������:	�����ļ�ͬ��
�������:	u8Mode
					0:	default configFile	-> Config
					1:	const default data	-> Config
					2:	Auto
**********************************************************************/
XM_S32 XM_MPI_MENU_CfgSync(XM_U8 u8Mode);



/**********************************************************************
��������:	���ز˵������ļ�
�������:	u32Cmd
					0:		ALL
					����: 	����ĳ��
�������:	��
���ز���:	-1		: ʧ��
				����	: �ɹ�
Note:			Lycai
**********************************************************************/
XM_S32 XM_MPI_MENU_LoadCfg(XM_U32 u32Cmd);



/**********************************************************************
��������:	�����ļ�ͬ��
�������:	u8Mode
					0:	default configFile 	-> Config
					1:    const default data 	-> Config
�������:	��
���ز���:	0: Success
				-1: Failure
**********************************************************************/
XM_S32 XM_MPI_MENU_SetDefault(XM_U8 u8Mode);



#endif

