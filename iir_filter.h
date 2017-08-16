#ifndef __IIR_FILTER_H_
#define __IIR_FILTER_H_

#include <stdint.h>

// filter coeff
#define FIR1_B1        1
#define FIR1_B2        0
#define FIR1_B3        -1
#define IIR1_A2        -113
#define IIR1_A3        52
#define FIR2_B1        1
#define FIR2_B2        0
#define FIR2_B3        -1
#define IIR2_A2        -78
#define IIR2_A3        36

#define HRM_FLT_SETTLE          150
#define FFT_WIN_SIZE            256


typedef struct {
    uint16_t flt_in_dly;
    uint16_t flt_in_dly2;
    int32_t sec1_dly;
    int32_t sec1_dly2;
    int32_t sec1_out;   //David Zou,20161227
    int32_t fir2_dly;   //David Zou,20161227
    int32_t fir2_dly2;  //David Zou,20161227
    int32_t sec2_dly;
    int32_t sec2_dly2;
    int32_t out;
} iir_data_t;

typedef struct { 
    uint16_t    hr_times_8;
    uint32_t    data_cnt;
    uint32_t    dc_value;
    iir_data_t  iir_data;
    iir_data_t  iir_data_gsen;
    uint16_t    raw_data_fifo [4];
    uint16_t    fft_phase[166];
    uint16_t    fft_phase_start[166];
    int32_t     hrm_fft_out[166][2];
    /** 
	* 为了代码方便书写，加一个fft_out 的矢量和数组，
	* 如需内存优化，可去掉些数据，并作相应变化，下同
	*/
    int32_t 	hrm_fft_out_vector[166];
    int32_t     gsen_fft_out[166][2];
    int32_t 	gsen_fft_out_vector[166];
    int32_t     flt_data_fifo[FFT_WIN_SIZE];
    int32_t     gsen_data_fifo[FFT_WIN_SIZE];
    uint16_t    ind,start_ind;
    int32_t     motion_sum;
} alg_data_t;

int32_t iir_filter_process(uint16_t data_ppg,uint8_t data_gsensor,uint8_t low_fre,uint8_t high_fre,uint8_t abnormal_jump_remove);
int32_t iir_filter_process_v2(uint16_t data_ppg,uint8_t data_gsensor,uint8_t low_fre,uint8_t high_fre, uint8_t abnormal_jump_remove);
void iir_filter_para_init(void);

#endif      // __IIR_FILTER_H_
