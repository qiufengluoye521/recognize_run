#ifndef __DFT_PROCESS_H_
#define __DFT_PROCESS_H_

#include <stdint.h>

typedef void (*fft_out_process_cb_t)(int32_t * ppg_fft_out,int32_t * gsensor_fft_out);

void fft_square_bw_v2(uint8_t low_fre,uint8_t high_fre,int32_t data);
void send_fft_fre_init(void);
void fft_data_init(void);
void fft_out_process_reg(fft_out_process_cb_t fun);

#endif          // #ifndef __DFT_PROCESS_H_

