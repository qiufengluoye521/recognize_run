#ifndef	__HR_RECOGNIZE_H
#define __HR_RECOGNIZE_H

#include <stdint.h>

#define 	LEN_MAX_PEAK			8		/*�洢��ֵ��ĸ���*/
#define 	DEFAULT_HR				65		/*hr default value*/
#define		STATIC_STATE			0		/*��ֹ״̬*/
#define		TRANS_STATE				1		/*����״̬*/
#define		WALK_STATE				2		/*����״̬*/
#define		RUN_STATE				3		/*�ܲ�״̬*/
#define		DISTURB_STATE			4		/*����״̬*/


typedef struct {
	uint8_t pre_num;
	int32_t pre_value;
} peak_max;

typedef struct{
	uint8_t continuity;			/*������Ȩֵ*/
	uint8_t peak_amplitude;		/*��ֵ��СȨֵ*/
	uint8_t peak_width;			/*������Ȩֵ*/
	uint8_t move_factor;		/*�˶�����Ȩֵ*/
	uint8_t rise_factor;		/*����������Ȩֵ*/
} weighing;

void hr_recognize_fun(int32_t * ppg_fft_out,int32_t * gsensor_fft_out);

#endif	// #ifndef	__HR_RECOGNIZE_H

