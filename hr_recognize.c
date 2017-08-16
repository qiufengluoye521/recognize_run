#include "hr_recognize.h"
#include <stdio.h>
#include <math.h>

#define REC_PARA_PATH "D:\\design\\C\\C_free\\recognize_run\\rec_para.txt"

FILE *rec_para_file;

static peak_max ppg_max_peak_buff[LEN_MAX_PEAK],gsensor_max_peak_buff[LEN_MAX_PEAK];
static uint8_t hr_last_3_buf[3]; 
static weighing weighing_parameter;
/*
* 0: static 1:trans	2:walk 3:run 4:disturb
*/
static uint8_t current_state_motion 	= 0;
static uint32_t fft_count				= 0;
static uint8_t is_first_3_hr_flag		= 1;		/*初始化后前3帧数据标志*/
uint8_t hr								= DEFAULT_HR;

// 连续性参数权值 
static float continual_heighing[LEN_MAX_PEAK];
static uint8_t continual_para_avg[LEN_MAX_PEAK];
static uint8_t continual_para_last[LEN_MAX_PEAK];
// 幅值参数权值 
static float peak_value_heighing[LEN_MAX_PEAK];
// gsensor(move)参数的权值 
static float move_factor_heighing[LEN_MAX_PEAK];
// 上升可能性权值
static float rise_factor_heighing[LEN_MAX_PEAK];
//计算最终权重
static float total_heighing[LEN_MAX_PEAK];


/*************************************************************************
*	static function
*/

/**
* @brief :recognize the current state
* 
* @para in: ppg_buf 			-> ppg raw data of one frame
* @para in: gsensor_buf 		-> gsensor raw data of one frame
* @para in: ppg_max_peak 		-> ppg max peak info
* @para in: gsensor_max_peak 	-> gsensor max peak info
* @para in: peak_num 			-> num of max peak in one frame
*
* @return: rec_result 			-> recognized current state
*
*/
static uint8_t current_motion_recognize(int32_t * 	ppg_buf,
										int32_t * 	gsensor_buf,
										peak_max * 	ppg_max_peak,
										peak_max *	gsensor_max_peak,
										uint8_t  	peak_num )
{
	/*决断gsensor是否存在2倍频*/
	uint8_t gsensor_2_times_flags = 0;
	/*极值点num 0~165*/
	uint8_t max_peak_num,scan_end;
	uint8_t is_disturb_flag = DISTURB_STATE;
	uint8_t i = 0;
	uint8_t rec_result =0;
	float gsensor_times;
	
//	printf("%d\n",gsensor_max_peak[peak_num].pre_value);
	
	if(ppg_max_peak[peak_num].pre_num < 45)
	{
		max_peak_num = 0;
	} else
	{
		max_peak_num = ppg_max_peak[peak_num].pre_num - 45;
	}
	// 极大值下方存在较小的值，认为些信号不是干扰信号，干扰信号
	// 的特征 一般低频全频段的值都较大 
	for(i=max_peak_num;i>0;i--)
	{
		if((float)ppg_buf[i] < (float)ppg_max_peak[peak_num].pre_value*2.0/3.0)
		{
			is_disturb_flag = STATIC_STATE;
			break;
		}
	}
	if(max_peak_num > 156)
	{
		scan_end = 166;
	} else
	{
		scan_end = max_peak_num + 10;
	}
	// 极大值上方10个点存在较小的值，认为些信号不是干扰信号，干扰信号
	// 的特征 一般极大值附近比较平缓，值都较大 
	for(i=0;i<scan_end;i++)
	{
		if((float)ppg_buf[i] < (float)ppg_max_peak[peak_num].pre_value*2.0/3.0)
		{
			is_disturb_flag = STATIC_STATE;
			break;
		}
	}
	if(DISTURB_STATE == is_disturb_flag)
	{
		rec_result = is_disturb_flag;
		return rec_result;
	} 
	
	/*******以上主要判断此帧是不是干扰信号，若不是则往下判断******/ 
	
//	/*最大值和次大值之间的倍数值，若为2的左右，则说明gsensor有明显倍频现象*/
//	gsensor_times = (float)(gsensor_max_peak[peak_num].pre_num / gsensor_max_peak[peak_num-1].pre_num);
//	if(gsensor_times >= 0.48) || (gsensor_times >= 0.48)

	/*gsensor 的最大值小于80，认为是静止状态*/
	if(gsensor_max_peak[peak_num].pre_value < 80)
	{
		rec_result = STATIC_STATE;
	} else if(gsensor_max_peak[peak_num].pre_value < 150)
	{
		rec_result = WALK_STATE;				// 步行状态 
	} else 
	{
		rec_result = RUN_STATE;					// 跑步状态 
	}
	
	return rec_result; 
}

/**
* @brief :recognize hr data int static state
* 
* @para in: ppg_pre 			-> ppg raw data of one frame
* @para in: para 				-> para weighing
* @para in: ppg_peak_buf 		-> ppg max peak info
* @para in: hr_buf 				-> last severia hr value
* @para in: flag 				-> first 3 frames or not
* @para in: peak_num 			-> num of max peak in one frame
*
* @return: hr_calculate 		-> recognized hr data of this frame
*
*/
static uint8_t static_process(int32_t 		* ppg_pre,
							  weighing 		para,
							  peak_max 		* ppg_peak_buf,
							  uint8_t 		* hr_buf,
							  uint8_t 		flag,
							  uint8_t       peak_num)
{
	float temp = 0;
	uint8_t i;
	uint8_t hr_last_avg = 0;
	float continual_temp = 0,max_temp = 0;
	uint8_t max_heighing_num = 0;
	uint8_t hr_calculate;
	
	temp = (float)(ppg_peak_buf[peak_num-1].pre_value * 1.5f);
	/*有一个明显的峰值点大于其它峰值点 */
	if(ppg_peak_buf[peak_num].pre_value > 2*ppg_peak_buf[peak_num-1].pre_value)
	{
		para.peak_amplitude		= 70;
		para.continuity			= 30;
	} else if(ppg_peak_buf[peak_num].pre_value > (int)temp)
	{
		para.continuity			= 40;
		para.peak_amplitude		= 60;	
	} else 
	{
		para.continuity			= 50;
		para.peak_amplitude		= 50;
	}
	
	/*计算连续性参数的权值 ---- continual_heighing 并归一化至100  */
	hr_last_avg = (hr_buf[0] + hr_buf[1] + hr_buf[2])/3;
	for(i=0;i<=peak_num;i++)
	{
		continual_para_last[i] 	= abs(ppg_peak_buf[i].pre_num - hr_buf[2]);
		continual_para_avg[i]	= abs(ppg_peak_buf[i].pre_num - hr_last_avg);
	} 
	
	max_temp = 0;
	for(i=0;i<=peak_num;i++)
	{
		continual_temp = (float)(continual_para_last[i] + continual_para_avg[i]);
		
		if(0 == continual_temp)
		{
			continual_temp = 0.1f;
		}
		continual_heighing[i] = 1.0 / continual_temp; 
		if(max_temp < continual_heighing[i])
		{
			max_temp = continual_heighing[i];
		}
	}
	for(i=0;i<=peak_num;i++)
	{
		continual_heighing[i] = continual_heighing[i]/max_temp*100;
	}
	
	/* 计算频域幅值参数的权值 */ 
	max_temp = 0;
	for(i=0;i<=peak_num;i++)
	{
		peak_value_heighing[i] = ppg_peak_buf[i].pre_value; 
		if(max_temp < peak_value_heighing[i])
		{
			max_temp = peak_value_heighing[i] ;
		}
	} 
	if(0 == max_temp)
	{
		max_temp = 0.1;
	}
	for(i=0;i<=peak_num;i++)
	{
		peak_value_heighing[i] = peak_value_heighing[i] / max_temp * 100;
	}
	
	/*计算最终权重*/
	max_temp = -65530;
	max_heighing_num = 0;
	for(i=0;i<=peak_num;i++)
	{
		total_heighing[i] = continual_heighing[i]*para.continuity  \
						+ peak_value_heighing[i]*para.peak_amplitude;
		if(max_temp < total_heighing[i])
		{
			max_temp 			= total_heighing[i];
			max_heighing_num	= i;
		}
	}
	
	hr_calculate = ppg_peak_buf[max_heighing_num].pre_num;
	
	/* 如果不是前3组数据，则做一个简单的滑动，前3组数据不用处理 */
	if(flag != 1)
	{
		if((hr_calculate - hr_buf[2]) > 4)
		{
			hr_calculate = hr_buf[2] + 3;
		} else if((hr_calculate - hr_buf[2]) < -3)
		{
			hr_calculate = hr_buf[2] - 2;
		}
	}
	
	if(hr_calculate < 45)
	{
		hr_calculate = 45;
	}
	
	return hr_calculate;
}

/**
* @brief :recognize hr data int walk run state
* 
* @para in: ppg_pre 			-> ppg raw data of one frame
* @para in: para 				-> para weighing
* @para in: ppg_peak_buf 		-> ppg max peak info
* @para in: gsensor_peak_buf 	-> gsensor max peak info
* @para in: hr_buf 				-> last severia hr value
* @para in: flag 				-> first 3 frames or not
* @para in: peak_num 			-> num of max peak in one frame
*
* @return: hr_calculate 		-> recognized hr data of this frame
*
*/
static uint8_t walk_run_process(int32_t 	* ppg_pre,
								weighing 	para,
								peak_max 	* ppg_peak_buf,
								peak_max 	* gsensor_peak_buf,
								uint8_t 	* hr_buf,
								uint8_t 	flag,
								uint8_t 	peak_num)
{
	float temp = 0,temp1 = 0;
	uint8_t i,j;
	uint8_t hr_last_avg = 0;
	float continual_temp = 0,max_temp = 0;
	uint8_t max_heighing_num = 0;
	uint8_t hr_calculate;
	float min_distance,current_distance; 
	int16_t peak_to_hr;
	uint8_t no_other_peak_flag; 
	float max_heighing_value;
	
	/*计算连续性参数的权值 ---- continual_heighing 并归一化至100  */
	hr_last_avg = (hr_buf[0] + hr_buf[1] + hr_buf[2])/3;
	for(i=0;i<=peak_num;i++)
	{
		continual_para_last[i] 	= abs(ppg_peak_buf[i].pre_num - hr_buf[2]);
		continual_para_avg[i]	= abs(ppg_peak_buf[i].pre_num - hr_last_avg);
	} 
	
	max_temp = 0;
	for(i=0;i<=peak_num;i++)
	{
		continual_temp = (float)(continual_para_last[i] + continual_para_avg[i]);
		
		if(0 == continual_temp)
		{
			continual_temp = 0.1f;
		}
		continual_heighing[i] = 1.0 / continual_temp; 
		if(max_temp < continual_heighing[i])
		{
			max_temp = continual_heighing[i];
		}
	}
	for(i=0;i<=peak_num;i++)
	{
		continual_heighing[i] = continual_heighing[i]/max_temp*100;
	}
	
	/*计算频域幅值参数的权值*/
	max_temp = 0;
	for(i=0;i<=peak_num;i++)
	{
		// 跑步时心率值小于60的可能性较小
		if(ppg_peak_buf[i].pre_num < 60)
		{
			peak_value_heighing[i] = 0.5 * ppg_peak_buf[i].pre_value;
		} else if(ppg_peak_buf[i].pre_num < 180)						// 心率在60~180 的概率最大
		{
			peak_value_heighing[i] = ppg_peak_buf[i].pre_value;
		} else															// 心率在180以上的可能性较小
		{
			peak_value_heighing[i] = 0.5 * ppg_peak_buf[i].pre_value;
		} 
		if(max_temp < peak_value_heighing[i])
		{
			max_temp = peak_value_heighing[i];
		} 
	}
	for(i=0;i<=peak_num;i++)
	{
		peak_value_heighing[i] = peak_value_heighing[i] / max_temp * 100;
	}
	
	/*计算gsensor(move)参数的权值*/
	min_distance 		= 240;
	current_distance	= 0;
	max_temp			= 0;
	for(i=0;i<=peak_num;i++)
	{
		min_distance = 240;
		for(j=(peak_num-3);j<=peak_num;j++)
		{
			current_distance = abs(ppg_peak_buf[i].pre_num - gsensor_peak_buf[j].pre_num);
			if(min_distance > current_distance)
			{
				min_distance = current_distance;
			}
			move_factor_heighing[i] = current_distance;
			if(0 == move_factor_heighing[i])
			{
				move_factor_heighing[i] = 1;
			}
		}
	}
	
	for(i=0;i<=peak_num;i++)
	{
		move_factor_heighing[i] = 1 / move_factor_heighing[i];
		if(max_temp < move_factor_heighing[i])
		{
			max_temp = move_factor_heighing[i];
		}
	}
	
	for(i=0;i<=peak_num;i++)
	{
		move_factor_heighing[i] = move_factor_heighing[i] / max_temp * 100;
	}
	
	// 计算rise_factor 参数权值  
	for(i=0;i<=peak_num;i++)
	{
		peak_to_hr = (int16_t)(ppg_peak_buf[i].pre_num - hr_last_avg);
		if(peak_to_hr < -5)
		{
			rise_factor_heighing[i] = 5;
		} else if(peak_to_hr < -1)
		{
			rise_factor_heighing[i] = 20;
		} else if(peak_to_hr < 2)
		{
			rise_factor_heighing[i] = 40;
		} else if(peak_to_hr < 6)
		{
			rise_factor_heighing[i] = 100;
		} else if(peak_to_hr < 15)
		{
			rise_factor_heighing[i] = 50;
		} else 
		{
			rise_factor_heighing[i] = 5;
		}
	}
	
	// 判断除了gsensor 引起的频率外还有无其它频率点，若没有，则可能此频率就是心率值
	no_other_peak_flag = 0;
	// 存在倍频
	temp = (float)((float)ppg_peak_buf[peak_num].pre_num / (float)ppg_peak_buf[peak_num-1].pre_num);
	// 存在倍频 
	if(((temp >= 0.47) && (temp <= 0.53)) || ((temp >= 1.8) && (temp <= 2.2)))
	{
		// ppg 和 gsensor频率值重合
		if((abs(ppg_peak_buf[peak_num].pre_num - gsensor_peak_buf[peak_num].pre_num) <= 3) \
		  && (abs(ppg_peak_buf[peak_num-1].pre_num - gsensor_peak_buf[peak_num-1].pre_num) <= 3))
  		{
  			if(ppg_peak_buf[peak_num-2].pre_value < (ppg_peak_buf[peak_num-1].pre_value/2))
  			{
			  	no_other_peak_flag = 1;
		  	}
		  	// gsensor 引起的3倍频
		  	temp1 = (float)((float)ppg_peak_buf[peak_num-2].pre_num/(float)ppg_peak_buf[peak_num].pre_num);
			if((temp1 < 3.2) && (temp1 >0.35)) 
			{
				if(ppg_peak_buf[peak_num-3].pre_value < ppg_peak_buf[peak_num-2].pre_value/2)
				{
					no_other_peak_flag = 1;
				}
			}
			temp1 = (float)((float)ppg_peak_buf[peak_num-2].pre_num/(float)ppg_peak_buf[peak_num-1].pre_num);
			if((temp1<3.2) && (temp1>0.35))
			{
				if(ppg_peak_buf[peak_num-3].pre_value < ppg_peak_buf[peak_num-2].pre_value/2)
				{
					no_other_peak_flag = 1;
				}
			}
  		}
		
		if((abs(ppg_peak_buf[peak_num].pre_num - gsensor_peak_buf[peak_num-1].pre_num) <= 3) \
		  && (abs(ppg_peak_buf[peak_num-1].pre_num - gsensor_peak_buf[peak_num].pre_num) <= 3))
		{
  			if((float)ppg_peak_buf[peak_num-2].pre_value < (float)(ppg_peak_buf[peak_num].pre_value/2))
  			{
			  	no_other_peak_flag = 1;
		  	}
		  	// gsensor 引起的3倍频
		  	temp1 = (float)((float)ppg_peak_buf[peak_num-2].pre_num/(float)ppg_peak_buf[peak_num].pre_num);
			if((temp1 < 3.2) && (temp1 >0.35)) 
			{
				if(ppg_peak_buf[peak_num-3].pre_value < ppg_peak_buf[peak_num-2].pre_value/2)
				{
					no_other_peak_flag = 1;
				}
			}
			temp1 = (float)((float)ppg_peak_buf[peak_num-2].pre_num/(float)ppg_peak_buf[peak_num-1].pre_num);
			if((temp1<3.2) && (temp1>0.35))
			{
				if(ppg_peak_buf[peak_num-3].pre_value < ppg_peak_buf[peak_num-2].pre_value/2)
				{
					no_other_peak_flag = 1;
				}
			}
  		}
		 
	}
	
	if(1 == no_other_peak_flag)
	{
		no_other_peak_flag 	= 0;
		para.move_factor	= 0;
//		printf("no_other_peak    "); 
//	} else
//	{
//		printf("have other peak    ");
	}
	
	/* 总的权值 */
	max_heighing_value	= - 65530;
	max_heighing_num 	= 0;
	for(i=0;i<=peak_num;i++)
	{
		total_heighing[i] = continual_heighing[i]*para.continuity + \
		 					peak_value_heighing[i]*para.peak_amplitude - \
	 						move_factor_heighing[i]*para.move_factor + \
							 rise_factor_heighing[i]*para.rise_factor;
	 	if(max_heighing_value < total_heighing[i])
	 	{
	 		max_heighing_value = total_heighing[i];
	 		max_heighing_num = i;
	 	}
	}
	
	hr_calculate = ppg_peak_buf[max_heighing_num].pre_num;
	
	// 如果不是前3组数据，则做一个简单的滑动，前3组数据不用处理
	if(flag != 1)
	{
		if(hr_calculate - hr_buf[2] > 5)
		{
			hr_calculate = hr_buf[2] + 3;
		} else if(hr_calculate - hr_buf[2] < -4)
		{
			hr_calculate = hr_buf[2] - 2;
		}
	}
 
 	if(hr_calculate < 45)
 	{
	 	hr_calculate = 45;
 	}
 	
 	return hr_calculate;
	
}




/*************************************************************************
*	public function
*/

void hr_recognize_init(void)
{
	uint8_t i=0;
	for(i=0;i<LEN_MAX_PEAK;i++)
	{
		ppg_max_peak_buff[i].pre_num				= 0;
		ppg_max_peak_buff[i].pre_value				= 0;
		gsensor_max_peak_buff[i].pre_num			= 0;
		gsensor_max_peak_buff[i].pre_value			= 0;
	}

	hr_last_3_buf[0]					= DEFAULT_HR;
	hr_last_3_buf[1]					= DEFAULT_HR;
	hr_last_3_buf[2]					= DEFAULT_HR;
	weighing_parameter.continuity		= 20;
	weighing_parameter.peak_amplitude	= 20;
	weighing_parameter.peak_width 		= 20;
	weighing_parameter.move_factor		= 20;
	weighing_parameter.rise_factor		= 20;
	fft_count							= 0;
} 

void hr_recognize_fun(int32_t * ppg_fft_out,int32_t * gsensor_fft_out)
{
	uint8_t i = 0,j = 0,jj = 0;
	uint8_t k = 0;
	
	rec_para_file = fopen(REC_PARA_PATH,"a");	
	
	if(0 == fft_count)
	{
		hr_recognize_init();
	}
	if(117 == fft_count)
	{
		printf("hello");
	}
	fft_count ++;
	
	for(i=0;i<LEN_MAX_PEAK;i++)
	{
		ppg_max_peak_buff[i].pre_num				= 0;
		ppg_max_peak_buff[i].pre_value				= 0;
		gsensor_max_peak_buff[i].pre_num			= 0;
		gsensor_max_peak_buff[i].pre_value			= 0;
	}
	

	/*recognize the max peak of ppg*/
	for(j=3;j<166-3;j++)
	{
		// 决断极值条件
		if((ppg_fft_out[j] >= ppg_fft_out[j-1]) && (ppg_fft_out[j-1] >= ppg_fft_out[j-2]) && (ppg_fft_out[j-2] >= ppg_fft_out[j-3]) \
			&& (ppg_fft_out[j] >= ppg_fft_out[j+1]) && (ppg_fft_out[j+1] >= ppg_fft_out[j+2]) && (ppg_fft_out[j+2] >= ppg_fft_out[j+3]))
		{
			for(jj = 0;jj < LEN_MAX_PEAK;jj++)
			{
				if(ppg_fft_out[j] > ppg_max_peak_buff[jj].pre_value)
				{
					if(jj > 0)
					{
						ppg_max_peak_buff[jj-1] = ppg_max_peak_buff[jj];
					}
					ppg_max_peak_buff[jj].pre_num	= j + 45;
					ppg_max_peak_buff[jj].pre_value	= ppg_fft_out[j];
				}
			}
		}
	}
	
	/*recognize the max peak of gsensor*/
	for(j=3;j<166-3;j++)
	{
		// 决断极值条件
		if( (gsensor_fft_out[j]>=gsensor_fft_out[j-1]) && (gsensor_fft_out[j-1]>=gsensor_fft_out[j-2]) \
	 		&& (gsensor_fft_out[j]>=gsensor_fft_out[j+1]) && (gsensor_fft_out[j+1]>=gsensor_fft_out[j+2]) )
 		{
 			for(jj = 0;jj < LEN_MAX_PEAK;jj++)
			{
				if(gsensor_fft_out[j] > gsensor_max_peak_buff[jj].pre_value)
				{
					if(jj > 0)
					{
						gsensor_max_peak_buff[jj-1] = gsensor_max_peak_buff[jj];
					}
					gsensor_max_peak_buff[jj].pre_num	= j + 45;
					gsensor_max_peak_buff[jj].pre_value	= gsensor_fft_out[j];
				}
			}
	 	}
		
	}
	
	/*judge the current state*/
	current_state_motion = current_motion_recognize(ppg_fft_out,gsensor_fft_out,ppg_max_peak_buff,gsensor_max_peak_buff,LEN_MAX_PEAK-1);

	switch (current_state_motion)
	{
		case STATIC_STATE:
			weighing_parameter.continuity 			= 40;
			weighing_parameter.peak_amplitude		= 60;
			weighing_parameter.peak_width			= 0;
			weighing_parameter.move_factor			= 0;
			weighing_parameter.rise_factor			= 0;
			if(fft_count <= 3)
			{
				weighing_parameter.continuity 		= 0;
				is_first_3_hr_flag					= 1; 
			} else
			{
				is_first_3_hr_flag					= 0;
			}
			hr = static_process(ppg_fft_out,weighing_parameter,ppg_max_peak_buff,hr_last_3_buf,is_first_3_hr_flag,LEN_MAX_PEAK-1);
			printf("hr_%d :%d\r\n",fft_count,hr);
			hr_last_3_buf[0] = hr_last_3_buf[1];
			hr_last_3_buf[1] = hr_last_3_buf[2];
			hr_last_3_buf[2] = hr;
			// buff没有填满，先填相同的值
			if(0 == hr_last_3_buf[1]) 
			{
				hr_last_3_buf[1] = hr_last_3_buf[2];
				hr_last_3_buf[0] = hr_last_3_buf[1];
			} else if(0 == hr_last_3_buf[0])
			{
				hr_last_3_buf[i] = hr_last_3_buf[1];
			}
		break;
		case TRANS_STATE:
		break;
		case WALK_STATE:
			weighing_parameter.continuity 			= 30;
			weighing_parameter.peak_amplitude		= 20;
			weighing_parameter.peak_width			= 0;
			weighing_parameter.move_factor			= 30;
			weighing_parameter.rise_factor			= 20;
			if(fft_count <= 3)
			{
				weighing_parameter.continuity 		= 0;
				is_first_3_hr_flag					= 1; 
			} else
			{
				is_first_3_hr_flag					= 0;
			}
			hr = walk_run_process(ppg_fft_out,weighing_parameter,ppg_max_peak_buff,gsensor_max_peak_buff,hr_last_3_buf,is_first_3_hr_flag,LEN_MAX_PEAK-1);
			printf("hr_%d :%d\r\n",fft_count,hr);
			hr_last_3_buf[0] = hr_last_3_buf[1];
			hr_last_3_buf[1] = hr_last_3_buf[2];
			hr_last_3_buf[2] = hr;
			// buff没有填满，先填相同的值
			if(0 == hr_last_3_buf[1]) 
			{
				hr_last_3_buf[1] = hr_last_3_buf[2];
				hr_last_3_buf[0] = hr_last_3_buf[1];
			} else if(0 == hr_last_3_buf[0])
			{
				hr_last_3_buf[i] = hr_last_3_buf[1];
			}
		break;
		case RUN_STATE:
			weighing_parameter.continuity 			= 25;
			weighing_parameter.peak_amplitude		= 10;
			weighing_parameter.peak_width			= 0;
			weighing_parameter.move_factor			= 40;
			weighing_parameter.rise_factor			= 25;
			if(fft_count <= 3)
			{
				weighing_parameter.continuity 		= 0;
				is_first_3_hr_flag					= 1; 
			} else
			{
				is_first_3_hr_flag					= 0;
			}
			hr = walk_run_process(ppg_fft_out,weighing_parameter,ppg_max_peak_buff,gsensor_max_peak_buff,hr_last_3_buf,is_first_3_hr_flag,LEN_MAX_PEAK-1);
			printf("hr_%d :%d\r\n",fft_count,hr);
			hr_last_3_buf[0] = hr_last_3_buf[1];
			hr_last_3_buf[1] = hr_last_3_buf[2];
			hr_last_3_buf[2] = hr;
			// buff没有填满，先填相同的值
			if(0 == hr_last_3_buf[1]) 
			{
				hr_last_3_buf[1] = hr_last_3_buf[2];
				hr_last_3_buf[0] = hr_last_3_buf[1];
			} else if(0 == hr_last_3_buf[0])
			{
				hr_last_3_buf[i] = hr_last_3_buf[1];
			}
			
		break;
		case DISTURB_STATE:
			hr = hr_last_3_buf[2];
			printf("hr_%d :%d\r\n",fft_count,hr);
			hr_last_3_buf[0] = hr_last_3_buf[1];
			hr_last_3_buf[1] = hr_last_3_buf[2];
			hr_last_3_buf[2] = hr;
			// buff没有填满，先填相同的值
			if(0 == hr_last_3_buf[1]) 
			{
				hr_last_3_buf[1] = hr_last_3_buf[2];
				hr_last_3_buf[0] = hr_last_3_buf[1];
			} else if(0 == hr_last_3_buf[0])
			{
				hr_last_3_buf[i] = hr_last_3_buf[1];
			}
		break; 
	}
	
	fprintf(rec_para_file,"%d\t%d\t%d\n", fft_count,current_state_motion, hr); 
	fclose(rec_para_file);   
	
	return;
} 
