#include "iir_filter.h"
//#include "debug_macros.h"

//int16_t new_raw_data;
static int32_t iir_data_out;
alg_data_t m_alg_data_t;
alg_data_t * alg_data = &m_alg_data_t;

void iir_filter_para_init(void)
{
    alg_data->iir_data.fir2_dly     = 0;
    alg_data->iir_data.fir2_dly2    = 0;
    alg_data->iir_data.sec1_dly     = 0;
    alg_data->iir_data.sec1_dly2    = 0;
    alg_data->iir_data.sec1_out     = 0;
    alg_data->iir_data.fir2_dly     = 0;
    alg_data->iir_data.fir2_dly2    = 0;
    alg_data->iir_data.sec2_dly     = 0;
    alg_data->iir_data.sec2_dly2    = 0;
    alg_data->iir_data.out          = 0;
}

int32_t iir_filter_process(uint16_t data_ppg,uint8_t data_gsensor,uint8_t low_fre,uint8_t high_fre,uint8_t abnormal_jump_remove)
{
    static int32_t fir1         = 0;
    static int32_t fir2         = 0;
    static int32_t fir1_gsensor = 0;
    static int32_t fir2_gsensor = 0;
//    alg_data->fifo_num = 0;
//      alg_data->clip_cnt = 0;
    // ppg data process
    fir1 = data_ppg*FIR1_B1 \
           + (alg_data->iir_data.flt_in_dly)*FIR1_B2 \
           + (alg_data->iir_data.flt_in_dly2)*FIR1_B3;
    alg_data->iir_data.flt_in_dly2 = alg_data->iir_data.flt_in_dly;
    alg_data->iir_data.flt_in_dly  = data_ppg;

    alg_data->iir_data.sec1_out = fir1 - ((IIR1_A2*(alg_data->iir_data.sec1_dly) + \
                               IIR1_A3*(alg_data->iir_data.sec1_dly2))>>6);
    alg_data->iir_data.sec1_dly2 = alg_data->iir_data.sec1_dly;
    alg_data->iir_data.sec1_dly = alg_data->iir_data.sec1_out;

    fir2 = alg_data->iir_data.sec1_out*FIR2_B1 + \
                              (alg_data->iir_data.fir2_dly)*FIR2_B2 + \
                              (alg_data->iir_data.fir2_dly2)*FIR2_B3;
    alg_data->iir_data.fir2_dly2 = alg_data->iir_data.fir2_dly;
    alg_data->iir_data.fir2_dly = alg_data->iir_data.sec1_out;

    alg_data->iir_data.out = fir2 - ((IIR2_A2*(alg_data->iir_data.sec2_dly) + \
                          IIR2_A3*(alg_data->iir_data.sec2_dly2))>>6);
    alg_data->iir_data.sec2_dly2 = alg_data->iir_data.sec2_dly;
    alg_data->iir_data.sec2_dly = alg_data->iir_data.out;

    //data_fre[i] = alg_data->iir_data.out;   // 
    iir_data_out = alg_data->iir_data.out;
    
    // gsensor data process
    fir1_gsensor = data_gsensor*FIR1_B1 \
           + (alg_data->iir_data_gsen.flt_in_dly)*FIR1_B2 \
           + (alg_data->iir_data_gsen.flt_in_dly2)*FIR1_B3;
    alg_data->iir_data_gsen.flt_in_dly2 = alg_data->iir_data_gsen.flt_in_dly;
    alg_data->iir_data_gsen.flt_in_dly  = data_gsensor;

    alg_data->iir_data_gsen.sec1_out = fir1_gsensor - ((IIR1_A2*(alg_data->iir_data_gsen.sec1_dly) + \
                               IIR1_A3*(alg_data->iir_data_gsen.sec1_dly2))>>6);
    alg_data->iir_data_gsen.sec1_dly2 = alg_data->iir_data_gsen.sec1_dly;
    alg_data->iir_data_gsen.sec1_dly = alg_data->iir_data_gsen.sec1_out;

    fir2_gsensor = alg_data->iir_data_gsen.sec1_out*FIR2_B1 + \
                              (alg_data->iir_data_gsen.fir2_dly)*FIR2_B2 + \
                              (alg_data->iir_data_gsen.fir2_dly2)*FIR2_B3;
    alg_data->iir_data_gsen.fir2_dly2 = alg_data->iir_data_gsen.fir2_dly;
    alg_data->iir_data_gsen.fir2_dly = alg_data->iir_data_gsen.sec1_out;

    alg_data->iir_data_gsen.out = fir2_gsensor - ((IIR2_A2*(alg_data->iir_data_gsen.sec2_dly) + \
                          IIR2_A3*(alg_data->iir_data_gsen.sec2_dly2))>>6);
    alg_data->iir_data_gsen.sec2_dly2 = alg_data->iir_data_gsen.sec2_dly;
    alg_data->iir_data_gsen.sec2_dly = alg_data->iir_data_gsen.out;
    
    // remove the jump points
    if(1 == abnormal_jump_remove)
    {
    	if(alg_data->iir_data.out > 16000)
    	{
    		alg_data->iir_data.out = 16000;
	    }else if(alg_data->iir_data.out < - 16000)
	    {
	    	alg_data->iir_data.out = -16000;
    	}
    }

    //data_fre[i] = alg_data->iir_data_gsen.out;   // 
    //iir_data_out = alg_data->iir_data_gsen.out;
    iir_data_out = alg_data->iir_data.out;
    
    return iir_data_out;

}

int32_t iir_filter_process_v2(uint16_t data_ppg,uint8_t data_gsensor,uint8_t low_fre,uint8_t high_fre, uint8_t abnormal_jump_remove)
{
    static int32_t fir1 = 0;
    static int32_t fir2 = 0;
    static int32_t iir_out_fifo [20] = {0};
    static int32_t raw_data_diff_fifo [7];
    static start_ind = 0;
    static end_ind = 19;
    static int32_t flt_cnt = 0;
    int32_t iir_out = 0;
    uint8_t    abnormal_jump = 0;
    int8_t  ii;
    static uint16_t data_time_pre = 0;
    static int32_t abnormal_jump_ind = 0, abnormal_jump_ind_ref = 0;
    static uint8_t abnormal_jump_cnt = 0;

    for (ii = 5; ii >= 1; ii--) {
    	raw_data_diff_fifo[ii+1] = raw_data_diff_fifo[ii];	    	
    }   
    if (flt_cnt > 0) 
        raw_data_diff_fifo[1] = (data_ppg - data_time_pre);	 
    abnormal_jump_ind = raw_data_diff_fifo[6] + raw_data_diff_fifo[1] + raw_data_diff_fifo[2] \
                      + raw_data_diff_fifo[3] + raw_data_diff_fifo[4] + raw_data_diff_fifo[5];
    abnormal_jump_ind = abs(abnormal_jump_ind);
    if (flt_cnt > 0)
	    abnormal_jump_ind_ref = (abnormal_jump_ind_ref*(flt_cnt-1) + abnormal_jump_ind + (flt_cnt>>1))/flt_cnt;
    else
        abnormal_jump_ind_ref = abnormal_jump_ind;
        
    if (abnormal_jump_ind > 4*abnormal_jump_ind_ref) {
    	abnormal_jump_cnt = 1;
    } else if (abnormal_jump_cnt == 60){
    	abnormal_jump_cnt = 0;
    } else if (abnormal_jump_cnt > 0) {
    	abnormal_jump_cnt ++;
    }

    abnormal_jump = (abnormal_jump_cnt > 0)? 1:0;

    fir1 = data_ppg*FIR1_B1 \
           + (alg_data->iir_data.flt_in_dly)*FIR1_B2 \
           + (alg_data->iir_data.flt_in_dly2)*FIR1_B3;
    alg_data->iir_data.flt_in_dly2 = alg_data->iir_data.flt_in_dly;
    alg_data->iir_data.flt_in_dly  = data_ppg;

    alg_data->iir_data.sec1_out = fir1 - ((IIR1_A2*(alg_data->iir_data.sec1_dly) + \
                               IIR1_A3*(alg_data->iir_data.sec1_dly2))>>6);
    alg_data->iir_data.sec1_dly2 = alg_data->iir_data.sec1_dly;
    alg_data->iir_data.sec1_dly = alg_data->iir_data.sec1_out;

    fir2 = alg_data->iir_data.sec1_out*FIR2_B1 + \
                          (alg_data->iir_data.fir2_dly)*FIR2_B2 + \
                          (alg_data->iir_data.fir2_dly2)*FIR2_B3;
    alg_data->iir_data.fir2_dly2 = alg_data->iir_data.fir2_dly;
    alg_data->iir_data.fir2_dly = alg_data->iir_data.sec1_out;

    iir_out = fir2 - ((IIR2_A2*(alg_data->iir_data.sec2_dly) + \
                      IIR2_A3*(alg_data->iir_data.sec2_dly2))>>6);
    alg_data->iir_data.sec2_dly2 = alg_data->iir_data.sec2_dly;
    alg_data->iir_data.sec2_dly = iir_out; 
	
	flt_cnt++;
//	printf("Remove data at %d  %d  %d  %d %d  %d %d\n", flt_cnt, data_time, data_time_pre, abnormal_jump_ind, abnormal_jump_ind_ref, \
//	                                    abnormal_jump, abnormal_jump);
//	printf("fifo: %d %d %d %d %d %d %d\n", raw_data_diff_fifo[6], raw_data_diff_fifo[5], raw_data_diff_fifo[4], raw_data_diff_fifo[3], \
//	                                    raw_data_diff_fifo[2], raw_data_diff_fifo[1], raw_data_diff_fifo[0] );	
	if (flt_cnt >=21) {   //fifo full,begin to output
	    if ((abnormal_jump == 0) || (abnormal_jump_remove == 0)) {
 	        alg_data->iir_data.out = iir_out_fifo[end_ind];	   		
    	} else {
	    	alg_data->iir_data.out = 0;
	    	printf("Remove data at %d\n", flt_cnt);
	    }
		iir_out_fifo[start_ind] = iir_out;
		start_ind++;
		end_ind++;
		if (start_ind > 20)
		    start_ind = 0;
		if (end_ind > 20)
		    end_ind = 0; 	
	}

    //data_fre[i] = alg_data->iir_data.out;   //  
    data_time_pre = data_ppg;
	
	//    printf("FIlter out: %d %d %d %d %d %d %d %d\n", \
	//       data_time, fir1, IIR1_A2, ((IIR1_A2*(alg_data->iir_data.sec1_dly) + \
    //                               IIR1_A3*(alg_data->iir_data.sec1_dly2))>>6), \
	//       alg_data->iir_data.sec1_out, alg_data->iir_data.sec1_dly, \
	//		 IIR1_A2*(alg_data->iir_data.sec1_dly), alg_data->iir_data.out);            
        
}

