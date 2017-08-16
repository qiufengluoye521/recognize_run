#include "dft_process.h"
#include "iir_filter.h"
#include <math.h>
#include <stdio.h>
//#include "debug_macros.h"
//#include "app_scheduler.h"
//#include "square_root.h"
//#include "app_timer.h"


#define FFT_SEND_TIMER_MS                           40          // fft data send intenal---- ms
#define APP_TIMER_PRESCALER                         0
#define TIMER_FFT_SEND_INTERVAL                     APP_TIMER_TICKS(FFT_SEND_TIMER_MS, APP_TIMER_PRESCALER)
//APP_TIMER_DEF(m_fft_data_send_timer_id);            //fft 的频域值发送定时器

#define F_PATH "D:\\design\\C\\C_free\\recognize_run\\c_result.txt"  

extern alg_data_t * alg_data;
uint8_t fifo_temp_index                 = 0;
uint8_t fre_len                         = 0;
static uint8_t  fft_data_send_buf[20]   = {0};
static uint16_t fft_fre_data[166]       = {0};
uint16_t fft_fre_send_len               = 0;

FILE *c_result_file;


uint8_t  const sin_table[375] = {  0 ,  1 ,  2 ,   3 ,  4 ,  5 ,  6 ,  7,   9,  10,  11,  12,  13 , 14,  15,  16, \
                                 17 , 18 , 19 ,  20 , 21 , 22 , 23 , 25,  26,  27,  28,  29,  30 , 31,  32,  33, \
                                 34 , 35 , 36 ,  37 , 38 , 39 , 40 , 41,  43,  44,  45,  46,  47 , 48,  49,  50, \
                                 51 , 52 , 53 ,  54 , 55 , 56 , 57 , 58,  59,  60,  61,  62,  63 , 64,  65,  67, \
                                 68 , 69 , 70 ,  71 , 72 , 73 , 74 , 75,  76,  77,  78,  79,  80 , 81,  82,  83, \
                                 84 , 85 , 86 ,  87 , 88 , 89 , 90 , 91,  92,  93,  94,  95,  96 , 97,  98,  99, \
                                100 ,101 ,102 , 103 ,104 ,105 ,106 ,107, 108, 109, 110, 111, 111 ,112, 113, 114, \
                                115 ,116 ,117 , 118 ,119 ,120 ,121 ,122, 123, 124, 125, 126, 127 ,127, 128, 129, \
                                130 ,131 ,132 , 133 ,134 ,135 ,136 ,137, 138, 138, 139, 140, 141 ,142, 143, 144, \
                                145 ,146 ,146 , 147 ,148 ,149 ,150 ,151, 152, 152, 153, 154, 155 ,156, 157, 158, \
                                158 ,159 ,160 , 161 ,162 ,163 ,163 ,164, 165, 166, 167, 167, 168 ,169, 170, 171, \
                                171 ,172 ,173 , 174 ,175 ,175 ,176 ,177, 178, 178, 179, 180, 181 ,181, 182, 183, \
                                184 ,184 ,185 , 186 ,187 ,187 ,188 ,189, 190, 190, 191, 192, 192 ,193, 194, 194, \
                                195 ,196 ,196 , 197 ,198 ,199 ,199 ,200, 201, 201, 202, 202, 203 ,204, 204, 205, \
                                206 ,206 ,207 , 208 ,208 ,209 ,209 ,210, 211, 211, 212, 212, 213 ,214, 214, 215, \
                                215 ,216 ,216 , 217 ,218 ,218 ,219 ,219, 220, 220, 221, 221, 222 ,222, 223, 223, \
                                224 ,224 ,225 , 225 ,226 ,226 ,227 ,227, 228, 228, 229, 229, 230 ,230, 231, 231, \
                                232 ,232 ,233 , 233 ,233 ,234 ,234 ,235, 235, 235, 236, 236, 237 ,237, 237, 238, \
                                238 ,239 ,239 , 239 ,240 ,240 ,240 ,241, 241, 242, 242, 242, 243 ,243, 243, 243, \
                                244 ,244 ,244 , 245 ,245 ,245 ,246 ,246, 246, 246, 247, 247, 247 ,248, 248, 248, \
                                248 ,249 ,249 , 249 ,249 ,249 ,250 ,250, 250, 250, 250, 251, 251 ,251, 251, 251, \
                                252 ,252 ,252 , 252 ,252 ,252 ,253 ,253, 253, 253, 253, 253, 253 ,253, 254, 254, \
                                254 ,254 ,254 , 254 ,254 ,254 ,254 ,254, 254, 255, 255, 255, 255 ,255, 255, 255, \
                                255 ,255, 255 , 255 ,255 ,255 ,255 };


extern void lhb_ble_send_data(uint8_t *data, uint16_t len);

static fft_out_process_cb_t fft_out_process_cb = NULL;

void fft_out_process_reg(fft_out_process_cb_t fun)
{
	fft_out_process_cb = fun;
}

static void fft_fre_init(void)
{
    uint32_t err_code   = 0;
    
    fft_fre_send_len    = 0;
//    err_code = app_timer_stop(m_fft_data_send_timer_id);
//    APP_ERROR_CHECK(err_code);
}                                 

static void Fft_fre_send_process(void * p_contex)
{
    int32_t fre_int_send = 0;
    uint32_t err_code    = 0;
    //LHB_PRINT(".");
    fft_data_send_buf[0] = 0xAC;                            // 频率数据 0xAC 开头
    uint8_t i = 0;
    for(i=1;i<17;i=i+2)
    {
        if(fft_fre_send_len < 166)       //数据未发送完
        {
            fft_data_send_buf[i]    = fft_fre_data[fft_fre_send_len] >> 8;
            fft_data_send_buf[i+1]  = fft_fre_data[fft_fre_send_len] >> 0;
//            fft_data_send_buf[i]    = fre_int_send >> 8;
//            fft_data_send_buf[i+1]  = fre_int_send >> 0;
//            i += 1;
        } else{
            break;
        }
        fft_fre_send_len++;        
    }
    fft_data_send_buf[17] = 0;
    fft_data_send_buf[18] = ((fft_fre_send_len+2)/8) >> 8;
    fft_data_send_buf[19] = ((fft_fre_send_len+2)/8);
 //   lhb_ble_send_data(fft_data_send_buf, 20);
    
    if(fft_fre_send_len >= 166)
    {
        //__BKPT(0);
        fft_fre_send_len = 0;
//        err_code = app_timer_stop(m_fft_data_send_timer_id);
//        APP_ERROR_CHECK(err_code);
    }
    
}

void fft_data_init(void)
{
    uint16_t ii = 0;
    for (ii=0;ii<166;ii++) {        
        alg_data->fft_phase[ii] = 0;
        alg_data->fft_phase_start[ii] = 0;
        alg_data->hrm_fft_out[ii][0] = 0;
        alg_data->hrm_fft_out[ii][1] = 0;
    }

    for (ii=0;ii<166;ii++) {
        alg_data->gsen_fft_out[ii][0] = 0;
        alg_data->gsen_fft_out[ii][1] = 0;
    }

    for (ii=0;ii<FFT_WIN_SIZE;ii++) {
        alg_data->flt_data_fifo[ii] = 0;
        alg_data->gsen_data_fifo[ii] = 0;       //find
    }
    
    alg_data->data_cnt = 0;
    alg_data->start_ind = 0;
    fft_fre_init();
//    hr_process_data_init();
}

void send_fft_fre_init(void)
{
    uint32_t err_code = 0;
//    err_code = app_timer_create(&m_fft_data_send_timer_id, APP_TIMER_MODE_REPEATED, Fft_fre_send_process);
//    APP_ERROR_CHECK(err_code);
}
                                
void fft_square_bw_v2(uint8_t low_fre,uint8_t high_fre,int32_t data)
{
    uint8_t ii              = 0;
    int16_t phase_sin_1q    = 0;
    int16_t phase_cos_1q    = 0;
    int16_t sin_c           = 0;
    int16_t cos_c           = 0;
    int16_t temp_c          = 0;
    uint32_t fre_sqr        = 0;
    uint32_t this_pow1      = 0;
    uint32_t this_pow2      = 0;    
    uint32_t fre_sqr_root   = 0;
    uint32_t err_code       = 0;
    uint8_t tem_stop		= 0;
    
    
    if (alg_data->data_cnt > HRM_FLT_SETTLE) 
    //if(1)
    { // do main frame fft //////////   
        for (ii=0;ii<166;ii++) 
        {
            alg_data->fft_phase[ii] = alg_data->fft_phase[ii] + 45 + ii;
            if (alg_data->fft_phase[ii] >= 1500) alg_data->fft_phase[ii] = alg_data->fft_phase[ii] - 1500;

            phase_sin_1q = (alg_data->fft_phase[ii] % 375);  // 375 = 1500/4
//            if(alg_data->fft_phase[ii] >= 1125)
//            {
//                phase_sin_1q = alg_data->fft_phase[ii] -1125 ;
//            }else if(alg_data->fft_phase[ii] >= 750){
//                phase_sin_1q = alg_data->fft_phase[ii] -750 ;
//            }else if(alg_data->fft_phase[ii] >= 375){
//                phase_sin_1q = alg_data->fft_phase[ii] -375 ;
//            }else{
//                phase_sin_1q = alg_data->fft_phase[ii] ;
//            }
                                
            phase_cos_1q = 374 - phase_sin_1q;               //374 = 1500/4	- 1
            sin_c = sin_table[phase_sin_1q];
            cos_c = sin_table[phase_cos_1q];
            
            
            if (alg_data->fft_phase[ii] < 375) 
            {
                sin_c = sin_c;
                cos_c = cos_c;
            } else if (alg_data->fft_phase[ii] < 750) {
                temp_c = sin_c;
                sin_c = cos_c;
                cos_c = -temp_c;
            } else if (alg_data->fft_phase[ii] < 1125) {
                sin_c = -sin_c;
                cos_c = -cos_c;
            } else {
                temp_c = sin_c;
                sin_c = -cos_c;
                cos_c = temp_c;
            }
//            alg_data->hrm_fft_out[ii][0] = alg_data->hrm_fft_out[ii][0] + cos_c*alg_data->iir_data.out;
//            alg_data->hrm_fft_out[ii][1] = alg_data->hrm_fft_out[ii][1] + sin_c*alg_data->iir_data.out;

            alg_data->hrm_fft_out[ii][0] = alg_data->hrm_fft_out[ii][0] + cos_c*alg_data->iir_data.out;
            alg_data->hrm_fft_out[ii][1] = alg_data->hrm_fft_out[ii][1] + sin_c*alg_data->iir_data.out;

//            alg_data->hrm_fft_out[ii][0] = alg_data->hrm_fft_out[ii][0] + cos_c*data;
//            alg_data->hrm_fft_out[ii][1] = alg_data->hrm_fft_out[ii][1] + sin_c*data;

            alg_data->gsen_fft_out[ii][0] = alg_data->gsen_fft_out[ii][0] + cos_c*alg_data->iir_data_gsen.out;
            alg_data->gsen_fft_out[ii][1] = alg_data->gsen_fft_out[ii][1] + sin_c*alg_data->iir_data_gsen.out;

//            alg_data->gsen_fft_out[ii][0] = alg_data->gsen_fft_out[ii][0] + cos_c*data;
//            alg_data->gsen_fft_out[ii][1] = alg_data->gsen_fft_out[ii][1] + sin_c*data;
//           printf("%d\t%d\n",alg_data->hrm_fft_out[ii][0],alg_data->hrm_fft_out[ii][1]);

        } //for (ii=0;ii<60;ii++)
        if (alg_data->ind == (FFT_WIN_SIZE - 1))
            fifo_temp_index = 0;
        else
            fifo_temp_index = alg_data->ind + 1;

//        if (alg_data->data_cnt > HRM_FLT_SETTLE+1) 
//        {
//            alg_data->motion_sum = alg_data->motion_sum + abs(alg_data->iir_data_gsen.out - pre_gsen_out);
//        }
		if(alg_data->data_cnt >= 406)
		{
			tem_stop = 1;
		}
		if(alg_data->data_cnt >= 407)
		{
			tem_stop = 1;
		}
    }    // (alg_data->data_cnt > HRM_FLT_SETTLE)
    
    
    if (alg_data->data_cnt > (FFT_WIN_SIZE + HRM_FLT_SETTLE)) {
        for (ii=0;ii<166;ii++) {
            alg_data->fft_phase_start[ii] = alg_data->fft_phase_start[ii] + 45 + ii;		
            if (alg_data->fft_phase_start[ii] >= 1500) alg_data->fft_phase_start[ii] = alg_data->fft_phase_start[ii] - 1500;

            phase_sin_1q = (alg_data->fft_phase_start[ii] % 375);  // 375 = 1500/4					
//            if(alg_data->fft_phase_start[ii] >= 1125){
//                phase_sin_1q = alg_data->fft_phase_start[ii] -1125 ;
//            }else if(alg_data->fft_phase_start[ii] >= 750){
//                phase_sin_1q = alg_data->fft_phase_start[ii] -750 ;
//            }else if(alg_data->fft_phase_start[ii] >= 375){
//                phase_sin_1q = alg_data->fft_phase_start[ii] -375 ;							
//            }else{
//                phase_sin_1q = alg_data->fft_phase_start[ii] ;	
//            }		
                                        
            phase_cos_1q = 374 - phase_sin_1q;               //374 = 1500/4	- 1
            sin_c = sin_table[phase_sin_1q];
            cos_c = sin_table[phase_cos_1q];								
            if (alg_data->fft_phase_start[ii] < 375) {
                  sin_c = sin_c;
                  cos_c = cos_c;
            } else if (alg_data->fft_phase_start[ii] < 750) {
                      temp_c = sin_c;
                  sin_c = cos_c;
                  cos_c = -temp_c;
            } else if (alg_data->fft_phase_start[ii] < 1125) {
                  sin_c = -sin_c;
                  cos_c = -cos_c;
            } else {
                      temp_c = sin_c;
                  sin_c = -cos_c;
                  cos_c = temp_c;
            }
            alg_data->hrm_fft_out[ii][0] = alg_data->hrm_fft_out[ii][0] - cos_c*alg_data->flt_data_fifo[alg_data->start_ind];
            alg_data->hrm_fft_out[ii][1] = alg_data->hrm_fft_out[ii][1] - sin_c*alg_data->flt_data_fifo[alg_data->start_ind];
            //printf("FFT: %d\t%d\t%d\t%d\t%d\t%d\t%d\n", alg_data->data_cnt, ii, alg_data->hrm_fft_out[ii][0], alg_data->hrm_fft_out[ii][1], cos_c, sin_c, alg_data->flt_data_fifo[alg_data->start_ind]);      
            alg_data->gsen_fft_out[ii][0] = alg_data->gsen_fft_out[ii][0] - cos_c*alg_data->gsen_data_fifo[alg_data->start_ind];
            alg_data->gsen_fft_out[ii][1] = alg_data->gsen_fft_out[ii][1] - sin_c*alg_data->gsen_data_fifo[alg_data->start_ind];
        }       // 	for (ii=0;ii<60;ii++) {
    
        if (alg_data->start_ind == (FFT_WIN_SIZE - 1))
            fifo_temp_index = 0;
        else
            fifo_temp_index = alg_data->start_ind + 1;

        alg_data->motion_sum = alg_data->motion_sum - \
              abs(alg_data->gsen_data_fifo[fifo_temp_index] - alg_data->gsen_data_fifo[alg_data->start_ind]);
    }

    if ((alg_data->data_cnt >= ( HRM_FLT_SETTLE + 50) ) && (6 == ((alg_data->data_cnt - HRM_FLT_SETTLE) % 25)))
//	if (alg_data->data_cnt >= ( HRM_FLT_SETTLE + FFT_WIN_SIZE)) 
//	if (0 == (alg_data->data_cnt % 25))
	{
   	//if(1)  {
    	c_result_file = fopen(F_PATH,"a");
        for (ii=0;ii<166;ii++) {    	
            this_pow1 = ((alg_data->hrm_fft_out[ii][0]) >> 13)*((alg_data->hrm_fft_out[ii][0]) >> 13)  \
                    + ((alg_data->hrm_fft_out[ii][1]) >> 13)*((alg_data->hrm_fft_out[ii][1]) >> 13);
//        	this_pow1 = sqrt(this_pow1);
			alg_data->hrm_fft_out_vector[ii] = this_pow1; 

//            this_pow2 = ((alg_data->gsen_fft_out[ii][0])/128)*((alg_data->gsen_fft_out[ii][0])/128)  \
//                    + ((alg_data->gsen_fft_out[ii][1])/128)*((alg_data->gsen_fft_out[ii][1])/128);
            this_pow2 = ((alg_data->gsen_fft_out[ii][0]) >> 13)*((alg_data->gsen_fft_out[ii][0]) >> 13)  \
                    + ((alg_data->gsen_fft_out[ii][1]) >> 13)*((alg_data->gsen_fft_out[ii][1]) >> 13);
//           	this_pow2 = sqrt(this_pow2);
			alg_data->gsen_fft_out_vector[ii] = this_pow2; 
			
			//printf("FFT: %d\t%d\t%d\t%d\t%d\t%d\t%d\n", alg_data->data_cnt, ii, alg_data->hrm_fft_out[ii][0], alg_data->hrm_fft_out[ii][1], cos_c, sin_c, alg_data->flt_data_fifo[alg_data->start_ind]);      
//            fprintf(c_result_file,"%d\t%d\t%d\t%d\t%d\t%d\t%d\n", alg_data->data_cnt, ii, alg_data->hrm_fft_out[ii][0], alg_data->hrm_fft_out[ii][1], cos_c, sin_c, alg_data->flt_data_fifo[alg_data->start_ind]);    
//			fprintf(c_result_file,"%d\t%d\t%d\t%d\t%d\t%d\t%d\n", alg_data->data_cnt, ii, alg_data->gsen_fft_out[ii][0], alg_data->gsen_fft_out[ii][1], cos_c, sin_c, alg_data->gsen_data_fifo[alg_data->start_ind]);    
			fprintf(c_result_file,"%d\t%d\n", this_pow1, this_pow2);    
//			fprintf(c_result_file,"%d\t%d\t%d\t%d\n", alg_data->hrm_fft_out[ii][0], alg_data->hrm_fft_out[ii][1],alg_data->gsen_fft_out[ii][0], alg_data->gsen_fft_out[ii][1]);			 
        }
        /* call back recognize hr data through this_pow1 and this_pow2*/
//        fft_out_process_cb(alg_data->hrm_fft_out,alg_data->gsen_fft_out); 
		fft_out_process_cb(alg_data->hrm_fft_out_vector,alg_data->gsen_fft_out_vector); 
        fclose(c_result_file);
        //c_result_file = NULL;
		 
    }

     if (alg_data->data_cnt > HRM_FLT_SETTLE) {
        alg_data->flt_data_fifo[alg_data->start_ind] = alg_data->iir_data.out;		
        alg_data->gsen_data_fifo[alg_data->start_ind] = alg_data->iir_data_gsen.out;
        alg_data->ind++; 
        alg_data->start_ind++;
        if (alg_data->ind > (FFT_WIN_SIZE-1)) alg_data->ind = 0;
        if (alg_data->start_ind > (FFT_WIN_SIZE-1)) alg_data->start_ind = 0;
    } 
     
    alg_data->data_cnt ++;
    
}

