/******************************************************************************
 Copyright (C), 2015-2020, XM. Co., Ltd.
******************************************************************************
File Name	: mpi_isp.h
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


#ifndef __MPI_ISP_H__
#define __MPI_ISP_H__

#include "xm_comm_isp.h"
#include "xm_comm_sns.h"
#include "xm_comm_3a.h"
#include "xm_comm_video.h"


/* Firmware Main Operation */
XM_S32 XM_MPI_ISP_Init(ISP_DEV IspDev);
XM_S32 XM_MPI_ISP_MemInit(ISP_DEV IspDev);
XM_S32 XM_MPI_ISP_Run(ISP_DEV IspDev);
XM_S32 XM_MPI_ISP_Exit(ISP_DEV IspDev);

/*******************************************************************************************
��������: XM_MPI_ISP_SetVsyncIntEn
��������: �������볡�ж�ʹ��
�������: 		IspDev:   IspDev
			u8Mode:
				0:Disable(ǿ��)    
				1:Enable(ǿ��)    
				2:�ָ�(ǿ�ƹ� ֮ǰ��״̬)
�������: ��
���ز���: -1: ʧ��
		0: �ɹ�
*******************************************************************************************/
XM_S32 XM_MPI_ISP_SetVsyncIntEn(ISP_DEV IspDev, XM_U8 u8Mode);

XM_S32 XM_MPI_ISP_GetChnAttr(ISP_CHN_ATTR_S *pstChnAttr);
XM_S32 XM_MPI_ISP_SetChnAttr(ISP_CHN_ATTR_S *pstChnAttr);

XM_S32 XM_MPI_ISP_GetFrame(VIDEO_FRAME_INFO_S *pstFrameInfo);
XM_S32 XM_MPI_ISP_ReleaseFrame(VIDEO_FRAME_INFO_S *pstFrameInfo);

XM_S32 XM_MPI_ISP_SensorRegCallBack(SENSOR_ID SensorId, const ISP_SENSOR_REGISTER_S *pstRegister);
XM_S32 XM_MPI_ISP_SensorUnRegCallBack(SENSOR_ID SensorId);


XM_S32 XM_MPI_ISP_SetModuleControl(const ISP_MODULE_CTRL_U *punModCtrl);
XM_S32 XM_MPI_ISP_GetModuleControl(ISP_MODULE_CTRL_U *punModCtrl);

XM_S32 XM_MPI_ISP_SetGammaAttr(const ISP_GAMMA_ATTR_S *pstGammaAttr);
XM_S32 XM_MPI_ISP_GetGammaAttr(ISP_GAMMA_ATTR_S *pstGammaAttr);


XM_S32 XM_MPI_ISP_SetBlackLevelAttr(const ISP_BLACKLVL_ATTR_S *pstBlackLevel);
XM_S32 XM_MPI_ISP_GetBlackLevelAttr(ISP_BLACKLVL_ATTR_S *pstBlackLevel);

/*******************************************************************************************
��������: XM_MPI_ISP_SetRegister
��������: ���üĴ���Ϊָ��ֵ
�������: u32Addr: ��Ҫ�����ĵ�ַ
		u32Value: �õ�ַ��Ҫ���õ�ֵ
�������: ��
���ز���: -1: ʧ��
		0: �ɹ�
ע��
	���ϵͳ��������ӳ��
*******************************************************************************************/
XM_S32 XM_MPI_ISP_SetRegister(XM_U32 u32Addr, XM_U32 u32Value);

/*******************************************************************************************
��������: XM_MPI_ISP_GetRegister
��������: ��ȡ�Ĵ�����ֵ
�������: u32Addr: ��Ҫ�����ĵ�ַ
		pu32Value: ��Ŷ�ȡ����ֵ
�������: ��
���ز���: -1: ʧ��
		0: �ɹ�
ע��
	���ϵͳ��������ӳ��
*******************************************************************************************/
XM_S32 XM_MPI_ISP_GetRegister(XM_U32 u32Addr, XM_U32 *pu32Value);

XM_S32 XM_MPI_ISP_SetPubAttr(ISP_DEV IspDev, const ISP_PUB_ATTR_S *pstPubAttr);
XM_S32 XM_MPI_ISP_GetPubAttr(ISP_DEV IspDev, ISP_PUB_ATTR_S *pstPubAttr);

XM_S32 XM_MPI_ISP_SetSaturationAttr(const ISP_SATURATION_ATTR_S *pstSatAttr);
XM_S32 XM_MPI_ISP_GetSaturationAttr(ISP_SATURATION_ATTR_S *pstSatAttr);

// DPC
XM_S32 XM_MPI_ISP_SetStDefectPixelAttr(ISP_STDPC_ATTR_S *pstStDPAttr);
XM_S32 XM_MPI_ISP_GetStDefectPixelAttr(ISP_STDPC_ATTR_S *pstStDPAttr);
XM_S32 XM_MPI_ISP_SetDyDefectPixelAttr(ISP_DYDPC_ATTR_S *pstDyDPAttr);
XM_S32 XM_MPI_ISP_GetDyDefectPixelAttr(ISP_DYDPC_ATTR_S *pstDyDPAttr);


XM_S32 XM_MPI_ISP_SetSharpenAttr(const ISP_SHARPEN_ATTR_S *pstSharpenAttr);
XM_S32 XM_MPI_ISP_GetSharpenAttr(ISP_SHARPEN_ATTR_S *pstSharpenAttr);

XM_S32 XM_MPI_ISP_GetSharpenV2Attr(ISP_SHARPENV2_ATTR_S *pstSharpenAttr);
XM_S32 XM_MPI_ISP_SetSharpenV2Attr(const ISP_SHARPENV2_ATTR_S *pstSharpenAttr);

XM_S32 XM_MPI_ISP_SetNRAttr(const ISP_2DNR_ATTR_S *pstNRAttr);
XM_S32 XM_MPI_ISP_GetNRAttr(ISP_2DNR_ATTR_S *pstNRAttr);

XM_S32 XM_MPI_ISP_Set3DNrAttr(const ISP_3DNR_ATTR_S *pstNRAttr);
XM_S32 XM_MPI_ISP_Get3DNrAttr(ISP_3DNR_ATTR_S *pstNRAttr);

XM_S32 XM_MPI_ISP_Set3DNrV2Attr(const ISP_3DNRV2_ATTR_S *pstNRAttr);
XM_S32 XM_MPI_ISP_Get3DNrV2Attr(ISP_3DNRV2_ATTR_S *pstNRAttr);

XM_S32 XM_MPI_ISP_GetNrInfo(ISP_NR_INFO_S *pstNRInfo);

XM_S32 XM_MPI_ISP_SetCSCAttr(const ISP_CSC_ATTR_S *pstCSCAttr);
XM_S32 XM_MPI_ISP_GetCSCAttr(ISP_CSC_ATTR_S *pstCSCAttr);

// ȥα��
XM_S32 XM_MPI_ISP_SetAntiFalseColorAttr(const ISP_ANTI_FALSECOLOR_S *pstAntiFC);
XM_S32 XM_MPI_ISP_GetAntiFalseColorAttr(ISP_ANTI_FALSECOLOR_S *pstAntiFC);

XM_S32 XM_MPI_ISP_SetChromaAttr(ISP_CHROMA_ATTR_S *pstChromaAttr);
XM_S32 XM_MPI_ISP_GetChromaAttr(ISP_CHROMA_ATTR_S *pstChromaAttr);

XM_S32 XM_MPI_ISP_FpnInit(XM_U8 u8Mode);
XM_S32 XM_MPI_ISP_AWB_REFRESH(ISP_DEV IspDev);

//*pu32AeErr:  err*64
XM_S32 XM_MPI_ISP_StabStats(XM_U32 *pu32AeErr);


XM_S32 XM_MPI_ISP_SetDRCAttr(ISP_DEV IspDev, const ISP_DRC_ATTR_S *pstDRC);
XM_S32 XM_MPI_ISP_GetDRCAttr(ISP_DEV IspDev, ISP_DRC_ATTR_S *pstDRC);

XM_S32 XM_MPI_ISP_SetDeFogAttr(ISP_DEV IspDev, const ISP_DEFOG_ATTR_S *pstDefogAttr);
XM_S32 XM_MPI_ISP_GetDeFogAttr(ISP_DEV IspDev, ISP_DEFOG_ATTR_S *pstDefogAttr);

// ע�᳡�жϴ����Ļص�����
XM_S32 XM_MPI_ISP_SetVsyncCallback(ISP_DEV IspDev, ISP_VSYNC_CALBAK_S *pstVsyncCalBak);
XM_S32 XM_MPI_ISP_GetVsyncCallback(ISP_DEV IspDev, ISP_VSYNC_CALBAK_S *pstVsyncCalBak);


/*******************************************************************************************
��������: XM_MPI_ISP_Memncpy
��������: �ڴ濽����ָ���ֽ�����
�������: pSrc: ԭʼ�����ڴ��ַ
			u32Num: ���ݵĴ�С���ֽڣ�
�������: pDst: ������Ŀ���ַ
���ز���: -1: ʧ��
		0: �ɹ�
*******************************************************************************************/
XM_S32 XM_MPI_ISP_Memncpy(XM_U8 *pDst, XM_U8 *pSrc, XM_U32 u32Num);


/*******************************************************************************************
��������: XM_MPI_ISP_Memset
��������: ָ���ڴ棬ָ����С�����ó�ָ��ֵ
�������: pu8Addr: ��Ҫ���������ݵ�ַ
			u8Ch: ָ���ڴ���Ҫ���óɵ�ֵ
			u32Num: ��ַ��С
�������: ��
���ز���: -1: ʧ��
		0: �ɹ�
*******************************************************************************************/
XM_VOID XM_MPI_ISP_Memset(XM_U8 *pu8Addr,XM_U8 u8Ch, XM_U32 u32Num);


XM_S32 XM_MPI_ISP_GetDCIAttr(ISP_DEV IspDev, ISP_DCI_ATTR_S *pstDciAttr);
XM_S32 XM_MPI_ISP_SetDCIAttr(ISP_DEV IspDev, ISP_DCI_ATTR_S *pstDciAttr);

#endif /*__MPI_ISP_H__ */

