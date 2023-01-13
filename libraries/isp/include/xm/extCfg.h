#ifndef _EXTCFG_H_
#define _EXTCFG_H_

#define DEBUG_ON	(1)
//#define RES_960		(1)
#include "xm_type.h"
#include "xm_common.h"
#include "xm_comm_isp.h"

#define FUN_NUM  	(35)

#define BUFFER_START	(0x800)
#define ROM_BASE  		(0x32000)
#define MARK_ID			(0xA55A5AA5)
#define MARK_ID_TO_SYS		(0xA55A5AB5)

// ����ģ���ַ
#define ROM_COM_MARK		(0)
#define ROM_MD_VER			(4)
#define ROM_MD_SNSNUM		(7)
#define ROM_MD_SNSBASE		(8)

#define COM_FUN_REG_F		(0)
#define COM_FUN_REG_B		(1)
#define COM_FUN_MENU		(2)




//  ģ���ڵĵ�ַƫ��(����ÿ��sensor������ʼ��)  gau32SnsBaseAddr
#define ROM_OFST_SNSID			(0)
#define ROM_OFST_FUNMASK		(1)
#define ROM_OFST_SNSI2C			(5)
#define ROM_OFST_ISPCUT			(9)

#define ROM_OFST_SNSCHECK		(21)


// Sensor I2C
#define I2C_OFST_RESOLUTION	(3)
// ISP CUT
#define ISPCUT_OFST_X_PAL		(0)
#define ISPCUT_OFST_Y_PAL		(2)
#define ISPCUT_OFST_WIDHT		(4)
#define ISPCUT_OFST_HEIGHT		(6)



#define SUPPORT_NUM		(8)


#define ROM_FUN_SNSI2C			(0)
#define ROM_FUN_ISPCUT			(1)
#define ROM_FUN_ISPINIT_FRONT		(2)
#define ROM_FUN_SNSINIT			(3)
#define ROM_FUN_FPS				(4)
#define ROM_FUN_MF				(5)
#define ROM_FUN_AEDEF			(6)
#define ROM_FUN_TABAGAIN		(7)
#define ROM_FUN_TABDGAIN		(8)
#define ROM_FUN_UPDATEGAIN		(9)
#define ROM_FUN_UPDATESHUT		(10)
#define ROM_FUN_ISPINIT			(11)
#define ROM_FUN_PN_ISPSET		(12)
#define ROM_FUN_AWB			(13)
#define ROM_FUN_CCM			(14)
#define ROM_FUN_SHARP			(15)
#define ROM_FUN_2DNR			(16)
#define ROM_FUN_3DNR			(17)
#define ROM_FUN_DYDPC			(18)
#define ROM_FUN_BLC				(19)
#define ROM_FUN_CON				(20)
#define ROM_FUN_SAT				(21)
#define ROM_FUN_CHROMA			(22)
#define ROM_FUN_AFC				(23)
#define ROM_FUN_NR3D			(24)
#define ROM_FUN_GAMMATAB		(25)
#define ROM_FUN_LUM				(26)
#define ROM_FUN_OTHER			(27)
#define ROM_FUN_GAINMODE		(28)
#define ROM_FUN_VENC			(29)
#define ROM_FUN_AEAWBINIT		(30)
#define ROM_FUN_AEAWBINFO		(31)
#define ROM_FUN_STABDEAL		(32)
#define ROM_FUN_GAINLOGIC		(33)
#define ROM_FUN_SHUTLOGIC		(34)


typedef enum {
// SmartSns
	SMS_LINE = 0x10,
	SMS_TAB  = 0x11,   // ��׼���
	SMS_TAB1 = 0x15,  // +�������߼�
	SMS_TAB2 = 0x19,  // +�������߼�
	SMS_TAB3 = 0x1D,  // +�����¡������߼�
 //����
	SOI_TAB = 0x21, // 

// Sony
	SONY_TAB = 0x31, // 

// Ov
	OV_LINE = 0x40, 	// ����
	OV_TAB = 0x41,	// ���
// Ps
	PS_LINE	= 0x50,

} GAIN_MODE_TYPE;


typedef struct _gamma_data_dn_s
{
	XM_U8 u8Num;			// <= 3  gamma����
	XM_U8 au8Idx[3];			// =u8Num
	XM_U32 au32Exp[2];		// =u8Num-1
	XM_U32 u32ThrDlt;
}GAMMA_DATA_DN_S;


typedef struct _lum_data_dn_s
{
	XM_U8 u8Num;			// <= 4
	XM_U8 au8Lum[3];			// =u8Num
	XM_U32 au32Exp[3];		// =u8Num
}LUM_DATA_DN_S;



typedef struct _other_data_s
{
	XM_U8 u8Size;
	XM_U8 u8Gain50;
	XM_U8 u8Gain100;
	XM_U8 u8CscLum;
	XM_U8 u8CscSat;
	XM_U8 u8CscContrast;
	XM_U8 u8CscHue;
	XM_U8 u8FlipMirror;
	XM_U8 u8VencVstd;
	XM_U8 u8Rslt;		// 0xFF: ���ڲ�����
	XM_U8 u8Rsv[6];
}OHTER_DATA_S;



// �����߼�(������ģʽ���ñ���)
typedef struct _HT_LOGIC_DT
{
	XM_U8 u8En;
	XM_U8 u8MinGain;
	XM_U16 u16Reg[2];		// L  H
	XM_U32 u32LimitUp;
	XM_U32 u32LimitDn;
/************************************************
����ģʽʱ
	XM_U8 u8AgainSft;		// Again cal����λ(��4λ<����4λ>)
	XM_U8 u8AgainMin;		// ��С����
	XM_U16 u16Rsv[2];		// ������
	XM_U32 u32Rsv1;		// ������
	XM_U32 u32Rsv2;		// ������

************************************************/
}HT_LOGIC_DT;


typedef struct _OV_AE_DATA
{
	XM_U32 u32Hold;
	XM_U32 u32Again;
	XM_U32 u32Dgain;
	XM_U32 u32Blc;
}AE_OV_TAB_DT;

typedef struct _AE_LINEAR_DT
{
	XM_U8 u8AgainSft;		// Again cal����λ(��4λ<����4λ>)
	XM_U8 u8AgainMin;		// ��С����
}AE_LINEAR_DT;


typedef union _GAIN_MODE_DT_U
{
	HT_LOGIC_DT stHtLgc;  		// SmartSns �����߼�
 	AE_LINEAR_DT stLinear;		// ��������
 	AE_OV_TAB_DT stOvTab;		// OV
}GAIN_MODE_DT;


typedef struct xm_AE_INIT_S
{
 	XM_U8 u8Speed;
	XM_U8 u8Tolerance;
 	XM_U8 u8Speed_Stab;
	XM_U8 u8Tolerance_Stab;
	XM_U32 u32UpdateCfg_Stab;
	XM_U8 u8AntiFlicker;
	XM_U8 u8AntiFlicker_Freq;
	XM_U8 u8BlackDelayFrame;
	XM_U8 u8WhiteDelayFrame;
	XM_U8 u8ExpMode;	// 0: Auto 1:Manual
	XM_U32 u32ExpManual;
	XM_U32 u32MinAGain;
	XM_U32 u32MaxAGain;
	XM_U32 u32MinDGain;
	XM_U32 u32MaxDGain;
	XM_U32 u32MinIspDGain;
	XM_U32 u32MaxIspDGain;
}AE_INIT_S;

typedef struct xm_AWB_INIT_S
{
 	XM_U8 u8HighColorTemp;
	XM_U8 u8LowColorTemp;
}AWB_INIT_S;


typedef struct xm_NR3D_INIT_S
{
	XM_U8 u8Size;
	XM_U8 u8FrameCnt;
	XM_U8 u8Ctrl;
	XM_U16 u16Width;
	XM_U16 u16Height;
	XM_U16 u16ChangePoint;
	XM_U32 u32PhyAddr;
	XM_U32 u32BaseAddr[8];
	XM_U32 u32YAddr;
	XM_U32 u32ErrAddr;
}NR3D_INIT_S;


typedef struct xm_STABDEAL_S
{
	XM_U8 u8Size;
	XM_U8 u8DealAfterStab;
	XM_U16 u16FmRunNum;
	XM_U16 u16StabFmID;
	XM_U8 u8IspRegNum;
	XM_U32 u32IspAddrData;
}STABDEAL_S;

XM_S32 ExtCfg_BaseAddrGet(XM_U8 u8Sns, XM_U32 u32Module, XM_U32 *pu32Addr);

XM_S32 ExtCft_Init();

//����ֵ:   0: ƥ������		-1: û��ƥ����
XM_S32 ExtCfg_IsLock();

/************************************************************************
��������:	����SensorID ��ȡ�ⲿ�����ļ���sensor����
�������:	u8SnsID: Ŀ��sensor ID
�������:	*pu8SnsIdx  �������ļ�������
���ز���:	0:		���ڸ�sensor
				-1: 		�����ڸ�sensor
************************************************************************/
XM_S32 ExtCft_GetSnsIdx(XM_U8 u8SnsID,XM_U8 *pu8SnsIdx);


// ����sensor ������ (ȷ��ƥ����Sensor)
XM_S32 ExtCfg_SetSnsIdx(XM_U8 u8SnsIdx, XM_U8 u8SnsAdWidth,XM_U8 u8SnsDtWidth);

// ��ȡSensorID
XM_S32 ExtCft_GetSnsId(XM_U8 *pu8SnsId);

// ��ȡ�����ļ��е�Sensor����
XM_S32 ExtCft_GetSnsNum(XM_U8 *pu8Num);

/*********************************************************
��������:	��ȡISP�ü�����
�������:	u8Fps:	֡��ģʽ(25/30/50/60)
�������:	pstRect:
					ISP�ü�����
���ز���:	-1:		��֧��
				0:		֧��
*********************************************************/
XM_S32 ExtCft_GetIspWndRect(XM_U8 u8Fps,RECT_S* pstRect);

//��ȡ�ֱ���
XM_S32 ExtCft_GetSnsResolution(XM_U8 *pu8Res);

//u8Mode:    	0: Common Funciton
//			1: Sensor Function
//��֤�Ƿ�֧�ָù���(�����Ƿ�ʹ���ⲿ����)
XM_S32 ExtCfg_CheckFun(XM_U8 u8Mode, XM_U32 u32Fun);

XM_S32 ExtCfg_PN_IspSet(XM_U8 u8Encode, XM_U8 u8VstdMode);

//u8DnMode:   	0: Color		1:BW
//u8XviEn:		0:Disable		1:Enable
XM_S32 ExtCfg_SetMode(XM_U8 u8XviEn, XM_U8 u8DnMode);

XM_S32 ExtCfg_IspInit();


/************************************************************************
��������:	��ȡGamma ����
�������:	*pu8Mode:		0:Color	1:Bw
�������:	*pu8Mode
					0: ����    1:����
				*pstGamm:
					��Ӧģʽ(Color/Bw)�Ĳ���
ע:	   ÿ��ģʽGamma��� 3��
************************************************************************/
XM_S32 ExtCfg_Gamma_Get(XM_U8 *pu8Mode,GAMMA_DATA_DN_S*pstGamm);
/************************************************************************
��������:	��ȡGammaTable
�������:	u8GmId:   ID(��u8GmId - 0x40   ��)       
�������:	*pu16Tab:
					Gamma��
ע:
	u8GmId>= 0x40
************************************************************************/
XM_S32 ExtCfg_GammaTab_Get(XM_U8 u8GmId,XM_U16 *pu16Tab);


/************************************************************************
��������:	��ȡlum����
�������:	��
�������:	��
���ز���: 	-1: ��֧��
				����: ����ֵ			
************************************************************************/
XM_S32 ExtCfg_Lum_Get();


// ��ȡOther����
XM_S32 ExtCfg_Other_Get(OHTER_DATA_S *pstData);


// ��ʼ��ISP �Ĵ���
XM_S32 ExtCfg_IspRegInit(XM_U8 u8Mode);



//u8DnMode:   	0: Color		1:BW
//u8XviEn:		0:Disable		1:Enable
XM_S32 ExtCfg_SetMode(XM_U8 u8XviEn, XM_U8 u8DnMode);

//��Flash/E2PROM�ж�ȡ
XM_S32 ExtCfg_Read_V2(XM_U8 u8Bytes, XM_U32 u32Add,XM_U8 *pu8Data);

XM_S32 ExtCfg_Write_V2(XM_U16 u16Bytes, XM_U32 u32Add,XM_U8 *pu8Data);


//д��Flash/E2PROM
XM_S32 ExtCfg_Write(XM_U8 u8Bytes, XM_U32 u32Addr,XM_U8 *pu8Data);



//��������: �������ļ���ȡ���� д��Isp�Ĵ���
//u32Addr:  ������ʼ��ַ(�����ļ�)
//u8Num: �Ĵ�������
//u8Mode: 0:Read From E2		1:Read From RAM
XM_S32 ExtCft_WriteIsp(XM_U8 u8Mode, XM_U32 u32Addr, XM_U16 u16Num);

/************************************************************************
��������:	ExtCft_GainModeGet
�������:	u8Mode:
					0:  ��ȡGainMode
					1:  ��ȡGainLogic��ַ
�������:	*pu32Data:
					GainMode  			(u8Mode == 0)
					GainLogic BaseAddr  	(u8Mode == 1)
���ز���:	-1:  failed/ not exist
				0:    Ok/exit
************************************************************************/
XM_S32 ExtCft_GainModeGet(XM_U8 u8Mode, XM_U32 *pu32Data);



/***********************************************************************
��������:	ExtCft_GetVersion
��������:	��ȡ�ⲿ���ð汾��Ϣ
�������:	��
�������:	ָ��3���ֽڵ�buffer ��ַ
					��(1Byte)
					��(1Byte)
					��(1Byte)
���ز���:	0:	�ɹ�
				-1: 	����
Note:Lycai
***********************************************************************/
XM_S32 ExtCft_GetVersion(XM_U8 *pau8Data);



/***********************************************************************
��������:	ExtCfg_VencSet
��������:	��ͬ���롢��Ƶ֧�ֽ����ⲿ���ò���
�������:	u8VencMode: ��ǰ��ʽ
					bit0: AHD_PAL
					bit1: AHD_NTSC
					bit2: CVI_PAL
					bit3: CVI_NTSC
					bit4: TVI_PAL
					bit5: TVI_NTSC
					bit6: CVBS_PAL
					bit7: CVBS_NTSC

�������:	��
���ز���:	0:	�ɹ�
				-1: 	����
Note:Lycai
***********************************************************************/
XM_S32 ExtCfg_VencSet(XM_U8 u8VencMode);

XM_S32 ExtCfg_Read_RAM(XM_U8 u8Bytes, XM_U32 u32Add,XM_U8 *pu8Data);

XM_S32 ExtCfg_CCM_Set(ISP_COLORMATRIX_AUTO_S *pstCCM);

XM_S32 ExtCfg_IspDataInit(XM_U8 u8Venc, XM_U8 u8Std);

XM_S32 ExtCfg_Init(XM_U8 *pu8Buffer);

XM_S32 ExtCfg_Nr3DInit(NR3D_INIT_S *pstNr3D);

XM_S32 ExtCfg_StabDeal(STABDEAL_S *pstStabDeal);

XM_S32 ExtCfg_AeAwbInit(XM_U8 u8Mode, XM_U8 u8Vstd);

/***********************************************************************
��������:	ExtCfg_GetExtCfgFlg
��������:	�ж��Ƿ���Ҫ����
�������:	��
�������:	��
���ز���:	0:	��������(ǰ��(ROM)�����ù�)
				1:    ��Ҫ����(ǰ��δ���ù�)
				2: 	��Ҫ����(�����ñ�)
				-1: 	����
Note:Lycai

	ROM ��ISP�����ظ����ò�������ǰ�ж�
***********************************************************************/
XM_S32 ExtCfg_GetExtCfgFlg();


void PrintInt(unsigned char u8Num,int u32Data);
void PrintHex(unsigned char u8Num,int u32Data);

#endif
