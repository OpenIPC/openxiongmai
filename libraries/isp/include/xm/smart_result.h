#ifndef __SMART_RESULT_H__
#define __SMART_RESULT_H__


#ifdef __cplusplus
#if __cplusplus
extern "C"
{
#endif
#endif /* __cplusplus */

typedef enum{
	SMART_MOVE_DETECT = 0,
	SMART_PRED,
}SMART_TYPE_E;
typedef struct stSmart_Result
{
	unsigned int cmd;//��ѯ���ÿbit����һ���㷨(�㷨˳����SMART_TYPE_E��˳�򱣳�һ��)
	unsigned int uiFlag; //��ѯ�����ÿbit����һ���㷨���(�㷨˳����SMART_TYPE_E��˳�򱣳�һ��)
	unsigned int reserved[16];
}Smart_Result_S;

XM_S32 Smart_GetResult(int nChannel,Smart_Result_S *pstResult) __attribute__((weak));

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* __cplusplus */

#endif 


