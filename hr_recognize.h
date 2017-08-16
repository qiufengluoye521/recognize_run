#ifndef	__HR_RECOGNIZE_H
#define __HR_RECOGNIZE_H

#include <stdint.h>

#define 	LEN_MAX_PEAK			8		/*存储极值点的个数*/
#define 	DEFAULT_HR				65		/*hr default value*/
#define		STATIC_STATE			0		/*静止状态*/
#define		TRANS_STATE				1		/*过渡状态*/
#define		WALK_STATE				2		/*步行状态*/
#define		RUN_STATE				3		/*跑步状态*/
#define		DISTURB_STATE			4		/*干扰状态*/


typedef struct {
	uint8_t pre_num;
	int32_t pre_value;
} peak_max;

typedef struct{
	uint8_t continuity;			/*连续性权值*/
	uint8_t peak_amplitude;		/*峰值大小权值*/
	uint8_t peak_width;			/*波峰宽度权值*/
	uint8_t move_factor;		/*运动因素权值*/
	uint8_t rise_factor;		/*上升可能性权值*/
} weighing;

void hr_recognize_fun(int32_t * ppg_fft_out,int32_t * gsensor_fft_out);

#endif	// #ifndef	__HR_RECOGNIZE_H

