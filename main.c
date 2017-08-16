#include <stdio.h>
#include "public_data.h" 
#include "iir_filter.h"
#include "dft_process.h"
#include <malloc.h>
#include "hr_recognize.h"

#define F_PATH_FILTER "D:\\design\\C\\C_free\\recognize_run\\filter_data.txt"
FILE *filter_file;
int32_t fft_filter_data[9513] = {0};




int main(int argc, char *argv[])
{
	uint16_t fft_data_len = 0;
	uint16_t gsensor_len = 0;
	uint16_t i = 0;
	
	fft_out_process_reg(hr_recognize_fun);
	
	fft_data_len = sizeof(fft_time_data) / sizeof(uint16_t);
	//printf("data len is :%d\r\n",fft_data_len);
	gsensor_len = sizeof(gsensor_data) / sizeof(uint8_t);
	if(fft_data_len != gsensor_len)
	{
		printf("ppg length is not same as gsensor len \r\n");
		return 0;
	} else
	{
		printf("length is %d,start processing!!!\r\n",fft_data_len);
	}
	
	
//	filter_file = fopen(F_PATH_FILTER,"a");
	
	for(i=0;i<fft_data_len;i++)
	{
		// process with iir filter
//        fft_filter_data[i] = iir_filter_process(fft_time_data[i],gsensor_data[i],40,200,1);
		iir_filter_process(fft_time_data[i],gsensor_data[i],40,200,1);
//        fprintf(filter_file,"%d\n", fft_filter_data[i]); 
        //printf("filter data is :%d\r\n",fft_filter_data[i]);
//        iir_filter_process_v2(fft_time_data[i],gsensor_data[i],40,200,0); 
        // process with dft
  		//fft_square_bw_v2(40,200,fft_time_data[i]);
  		fft_square_bw_v2(40,200,0);
	}
//	fclose(filter_file);	
	
	return 0;
}
