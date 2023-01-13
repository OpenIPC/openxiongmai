//
//  "$Id: MotionDetect.h 4 2009-06-11 13:01:43Z liwj $"
//
//  Copyright (c)2008-2008, ZheJiang JuFeng Technology Stock CO.LTD.
//  All Rights Reserved.
//
//	Description:	
//	Revisions:		Year-Month-Day  SVN-Author  Modification
//

#ifndef __PAL_MOTIONDETECT_H__
#define __PAL_MOTIONDETECT_H__

#ifdef __cplusplus
extern "C" {
#endif

/// \defgroup MotionDetectAPI API Motion Detect
/// ������������ȵĶ�̬���ӿڡ�
///	\n ��������ͼ:
/// \code
///    ============================
///                  |
///         *MotionDetectGetCaps
///                  |
///          MotionDetectCreate
///     +------------|
///     |   MotionDetectGetState
///     | MotionDetectSetParameter
///     |   MotionDetectGetResult
///     |   MotionDetectShowHint
///     +------------|
///         MotionDetectDestory
///                  |
///    ============================
/// \endcode
/// @{

/// ��̬������
typedef struct MOTION_DETECT_PARAM
{
	/// ��̬�������жȵ���
	int		iLevel;

	/// ��̬�����������ݵ�ÿһλ��Ӧһ����������飬��1��ʾ��Ҫ�ڸ�����鶯
	/// ̬��⣬��0��ʾ����Ҫ�ڸ�����鶯̬��⡣ÿһ����һ��unsigned int��ʾ������ߵ�
	/// ���Ӧ���λ�����ϵ��µ��ж�Ӧ���±��0��17��֧�ֵ�������֮���Ӧ������
	/// λ��Ч��
	unsigned int	win[18];

	/// �����ȵľ���ֵ
	unsigned char	sensitiveness;

	/// ֡���
	unsigned char	fieldinterval;

	/// ʹ�ܿ��أ��Ƿ�ʹ�ܣ�Ϊ1��ʾʹ�ܣ�Ϊ0ʱ��ʾ���ã��������������ԡ�
	int		enable;
}MOTION_DETECT_PARAM;

/// ��̬�����
typedef struct MOTION_DETECT_RESULT
{
	/// �ж�̬��ⷢ�����������ݵ�ÿһλ��Ӧһ����������飬��1��ʾ�ڸ������
	/// ��̬���ɹ�����0��ʾ������鶯̬���ʧ�ܡ�ÿһ����һ��unsigned int��ʾ�������
	/// �Ŀ������λ�����ϵ��µ��ж�Ӧ���±��0��17��֧�ֵ�������֮���Ӧ������
	/// λ��Ч��
	unsigned int	win[18];
}MOTION_DETECT_RESULT;

/// ��Ƶ��̬������Խṹ
typedef struct MOTION_DETECT_CAPS
{
	unsigned int Enabled;			///< ��1��ʾ֧�ֶ�̬��⣬��0��ʾ��֧�ֶ�̬��⡣
	unsigned int GridLines;		///< ��̬����������Ҫ���ֳɵ�������
	unsigned int GridRows;			///< ��̬����������Ҫ���ֳɵ�������
	unsigned char  Result;			///< �Ƿ��ܵõ�ÿ������ļ������
	unsigned char  Hint;				///< �Ƿ����ڼ��ɹ�ʱ������ʾ��
}MOTION_DETECT_CAPS;


/// ������̬����豸
/// 
/// \param ��
/// \retval <0 ����ʧ��
/// \retval 0 �����ɹ�
int MotionDetectCreate(void);


/// ���ٶ�̬����豸
/// 
/// \param ��
/// \retval <0 ����ʧ��
/// \retval 0 ���ٳɹ�
int MotionDetectDestory(void);


/// ִ�ж�̬��⡣�������ء�
/// 
/// \param [out] pData ָ��һ��unsigned intֵ��ָ�룬unsigned intֵ�Ǹ�����������ͨ����ƽ״̬
///        �����롣��ͨ���ڵ�λ����ͨ���ڸ�λ���ߵ�ƽ��1���͵�ƽ��0�������ڵ�ͨ
///        ����0��
/// \retval 0  ���ɹ�
/// \retval <0  ���ʧ��
int MotionDetectGetState(unsigned int* pData);


/// ��̬��Ⲷ�����ϸ�����
/// 
/// \param [in] channel ͨ���š�
/// \param [in] pResult ָ��̬�������ṹMOTION_DETECT_RESULT��ָ�롣
/// \retval 0  ���óɹ�
/// \retval <0  ����ʧ��
int MotionDetectGetResult(int channel, MOTION_DETECT_RESULT *pResult);


/// �����Ƿ��ڶ�̬���ɹ������������ʾ��
/// 
/// \param [in] channel ͨ���š�
/// \param [in] enable Ϊ1��ʾ��ʾ��Ϊ0ʱ��ʾ����ʾ��
/// \retval 0  ���óɹ�
/// \retval <0  ����ʧ��
int MotionDetectShowHint(int channel, int enable);


/// ִ�ж�̬��⡣�������ء�
/// 
/// \param [in] channel ͨ���š�
/// \param [in] pParam ָ��̬�������ṹMOTION_DETECT_PARAM��ָ�롣
/// \retval 0  ���óɹ�
/// \retval <0  ����ʧ��
int MotionDetectSetParameter(int channel, MOTION_DETECT_PARAM *pParam);


/// �õ���̬���֧�ֵ����ԡ�
/// 
/// \param [out] pCaps ָ��̬������ԽṹMOTION_DETECT_CAPS��ָ�롣
/// \retval 0  ��ȡ�ɹ���
/// \retval <0  ��ȡʧ�ܡ�
int MotionDetectGetCaps(MOTION_DETECT_CAPS * pCaps);

/// @} end of group

#ifdef __cplusplus
}
#endif

#endif 
