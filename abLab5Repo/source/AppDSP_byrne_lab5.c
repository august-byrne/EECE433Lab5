
/*******************************************************************************************
* AppDSP.c
* This is an example of one data processing task that does some real-time digital processing.
* Edits for EECE433 lab 5 were done by August Byrne.
*
* 02/10/2017 Todd Morton
* 04/03/2019 Todd Morton
* 05/22/2021 August Byrne
*******************************************************************************************/
/******************************************************************************************
* Include files
*******************************************************************************************/
#include "MCUType.h"
#include "app_cfg.h"
#include "os.h"
#include "I2S.h"
#include "TLV320AIC3007.h"
#include "K65TWR_GPIO.h"
#include "AppDSP.h"
#include "K65DMA.h"
/*****************************************************************************************************
* Defined constants for processing
*****************************************************************************************************/
#define FFT_LENGTH      512
//Supported Lengths: 32, 64, 128, 256, 512, 1024, 2048
                                //Must be <= DSP_SAMPLES_PER_BLOCK unless zero padded
#define NUM_STAGES        4
#define SAMPLE_RATE_HZ  48000
//#define FREQ_BIN_SIZE   SAMPLE_RATE_HZ/FFT_LENGTH
/******************************************************************************************/
static DSP_BLOCK_T dspInBuffer[DSP_NUM_IN_CHANNELS][DSP_NUM_BLOCKS];
static DSP_BLOCK_T dspOutBuffer[DSP_NUM_OUT_CHANNELS][DSP_NUM_BLOCKS];
static INT8U dspStopReqFlag = 0;
static OS_SEM dspFullStop;

/*****************************************************************************************************
* Public Function Prototypes
*****************************************************************************************************/
//IIR variables
static  arm_biquad_casd_df1_inst_f32  IIRLeft;
static  arm_biquad_casd_df1_inst_f32  IIRRight;
static  arm_biquad_casd_df1_inst_q31  IIRLeft_q31;
static  arm_biquad_casd_df1_inst_q31  IIRRight_q31;
//Each biquad IIR stage has 4 state variables.
//Each filter of 4 Biquad stages has 4*4=16 state variables.
static float32_t LeftState[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
static float32_t RightState[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
static q31_t LeftState_q31[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
static q31_t RightState_q31[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

static q31_t iirCoeffQ31[NUM_STAGES*5];
static q31_t iirCoeffQ31Scaled[NUM_STAGES*5];

static float32_t iirCoeffF32[] = {
		0.866091276638422,	-0.715205228839432,	0.866091276638422,	//b values for stage 1
		0.715205228839433,	-0.732182553276844,						//a for stage 1
		0.866091276638422,	-0.715205228839432,	0.866091276638422,	//b values for stage 2
		0.715205228839433,	-0.732182553276844,						//a for stage 2
		0.866091276638422,	-0.715205228839432,	0.866091276638422,	//b values for stage 3
		0.715205228839433,	-0.732182553276844,						//a for stage 3
		0.866091276638422,	-0.715205228839432,	0.866091276638422,	//b values for stage 3
		0.715205228839433,	-0.732182553276844,						//a for stage 4
};

static float32_t cosineVals[DSP_SAMPLES_PER_BLOCK];
/*******************************************************************************************
* Private Function Prototypes
*******************************************************************************************/
static void  dspTask(void *p_arg);
static CPU_STK dspTaskStk[APP_CFG_DSP_TASK_STK_SIZE];
static OS_TCB dspTaskTCB;
static DSP_PARAMS_T dspParams;
static const INT8U dspCodeToSize[4] = {16,20,24,32};
static const INT16U dspCodeToRate[11] = {48000,32000,24000,19200,16000,13700,
                                         12000,10700,9600,8700,8000};
/*******************************************************************************************
* DSPInit()- Initializes all dsp requirements - CODEC,I2S,DMA, and sets initial sample rate
*            and sample size.
*******************************************************************************************/
void DSPInit(void){
    OS_ERR os_err;
    //float32_t Fc = 4000;
    //float32_t V1;
    //int Q = (NUM_TAPS-1)/2;

    for(int i=0;i<DSP_SAMPLES_PER_BLOCK;i++){
    	cosineVals[i] = arm_cos_f32(2*PI*i*20000/48000);
    }

    //initialize IIR filter instance for both channels
    //arm_biquad_cascade_df1_init_f32(&IIRLeft,NUM_STAGES,&iirCoeffF32[0],&LeftState[0]);
    //arm_biquad_cascade_df1_init_f32(&IIRRight,NUM_STAGES,&iirCoeffF32[0],&RightState[0]);


    //convert IIR filter coefficients to Q31
    arm_float_to_q31(&iirCoeffF32[0],&iirCoeffQ31[0],(NUM_STAGES*5));
    //scale to get TOO LARGE coefficients
    //arm_scale_q31(&iirCoeffQ31[0],1,4,&iirCoeffQ31Scaled[0],(NUM_STAGES*5));
    //initialize IIR filter instance for both channels
    arm_biquad_cascade_df1_init_q31(&IIRLeft_q31,NUM_STAGES,&iirCoeffQ31[0],&LeftState_q31[0],0);
    arm_biquad_cascade_df1_init_q31(&IIRRight_q31,NUM_STAGES,&iirCoeffQ31[0],&RightState_q31[0],0);

    OSTaskCreate(&dspTaskTCB,
                "DSP Task ",
                dspTask,
                (void *) 0,
                APP_CFG_DSP_TASK_PRIO,
                &dspTaskStk[0],
                (APP_CFG_DSP_TASK_STK_SIZE / 10u),
                APP_CFG_DSP_TASK_STK_SIZE,
                0,
                0,
                (void *) 0,
                (OS_OPT_TASK_STK_CHK | OS_OPT_TASK_STK_CLR),
                &os_err);

    OSSemCreate(&dspFullStop, "DMA Stopped", 0, &os_err);
    CODECInit();
    I2SInit(DSP_SSIZE_CODE_32BIT);
    DSPSampleRateSet(CODEC_SRATE_CODE_48K);
    DSPSampleSizeSet(DSP_SSIZE_CODE_32BIT);
    DMAInit(&dspInBuffer[0][0], &dspOutBuffer[0][0]);
    I2S_RX_ENABLE();
    I2S_TX_ENABLE();
}

/*******************************************************************************************
* dspTask
*******************************************************************************************/
static void dspTask(void *p_arg){
    OS_ERR os_err;
    INT8U buffer_index;
    (void)p_arg;
    while(1){
        DB0_TURN_OFF();                             /* Turn off debug bit while waiting */
        buffer_index = DMAInPend(0, &os_err);
        DB0_TURN_ON();
        // DSP code goes here.
/*
        //convert the input signal from Q31 to floating-point
        arm_q31_to_float(&dspInBuffer[DSP_LEFT_CH][buffer_index].samples[0],&InBufferLeft[0],DSP_SAMPLES_PER_BLOCK);
        arm_q31_to_float(&dspInBuffer[DSP_RIGHT_CH][buffer_index].samples[0],&InBufferRight[0],DSP_SAMPLES_PER_BLOCK);
        // IIR floating-point filtering process
        arm_biquad_cascade_df1_f32(&IIRLeft,&InBufferLeft[0],&OutBufferLeft[0],DSP_SAMPLES_PER_BLOCK);
        arm_biquad_cascade_df1_f32(&IIRRight,&InBufferRight[0],&OutBufferRight[0],DSP_SAMPLES_PER_BLOCK);
        //convert the output signal from floating-point to Q31
        arm_float_to_q31(&OutBufferLeft[0],&dspOutBuffer[DSP_LEFT_CH][buffer_index].samples[0],DSP_SAMPLES_PER_BLOCK);
        arm_float_to_q31(&OutBufferRight[0],&dspOutBuffer[DSP_RIGHT_CH][buffer_index].samples[0],DSP_SAMPLES_PER_BLOCK);
*/
        // IIR floating-point filtering process
        arm_biquad_cascade_df1_q31(&IIRLeft_q31,&dspInBuffer[DSP_LEFT_CH][buffer_index].samples[0],&dspOutBuffer[DSP_LEFT_CH][buffer_index].samples[0],DSP_SAMPLES_PER_BLOCK);
        arm_biquad_cascade_df1_q31(&IIRRight_q31,&dspInBuffer[DSP_RIGHT_CH][buffer_index].samples[0],&dspOutBuffer[DSP_RIGHT_CH][buffer_index].samples[0],DSP_SAMPLES_PER_BLOCK);


        if((buffer_index == 1)&&(dspStopReqFlag == 1)){
            OSSemPost(&dspFullStop,OS_OPT_POST_1,&os_err);
        }
    }
}

/*******************************************************************************************
* DSPSampleSizeSet
* To set sample size you must set word size on both the CODEC and I2S
* Note: Does not change DMA or buffer word size which can be changed independently.
*******************************************************************************************/
void DSPSampleSizeSet(INT8U size_code){
    (void)CODECSetSampleSize(size_code);
    I2SWordSizeSet(size_code);
    dspParams.ssize = dspCodeToSize[size_code];
}
/*******************************************************************************************
* DSPSampleSizeGet
* To read current sample size code
*******************************************************************************************/
INT8U DSPSampleSizeGet(void){
    return dspParams.ssize;
}
/*******************************************************************************************
* DSPSampleRateGet
* To read current sample rate code
*******************************************************************************************/
INT16U DSPSampleRateGet(void){
    return dspParams.srate;
}
/*******************************************************************************************
* DSPSampleRateSet
* To set sample rate you set the rate on the CODEC
*******************************************************************************************/
void DSPSampleRateSet(INT8U rate_code){
    (void)CODECSetSampleRate(rate_code);
    dspParams.srate = dspCodeToRate[rate_code];
}
/*******************************************************************************************
* DSPStart
* Enable DMA to fill block with samples
*******************************************************************************************/
void DSPStartReq(void){

    dspStopReqFlag = 0;
    DMAStart();
    CODECEnable();
    CODECSetPage(0x00);
    CODECDefaultConfig();
    CODECHeadphoneOutOn();

}
/*******************************************************************************************
* DSPStop
* Disable DA after input/output buffers are full
*******************************************************************************************/
void DSPStopReq(void){

    dspStopReqFlag = 1;
    DMAStopFull();

}
/****************************************************************************************
 * DSP signal when buffer is full and DMA stopped
 * 04/16/2020 TDM
 ***************************************************************************************/

void DSPStopFullPend(OS_TICK tout, OS_ERR *os_err_ptr){
    OSSemPend(&dspFullStop, tout, OS_OPT_PEND_BLOCKING,(void *)0, os_err_ptr);
}
/****************************************************************************************
 * Return a pointer to the requested buffer
 * 04/16/2020 TDM
 ***************************************************************************************/

INT32S *DSPBufferGet(BUFF_ID_T buff_id){
    INT32S *buf_ptr = (void*)0;
    if(buff_id == LEFT_IN){
        buf_ptr = (INT32S *)&dspInBuffer[DSP_LEFT_CH][0];
    }else if(buff_id == RIGHT_IN){
        buf_ptr = (INT32S *)&dspInBuffer[DSP_RIGHT_CH][0];
    }else if(buff_id == RIGHT_OUT){
        buf_ptr = (INT32S *)&dspOutBuffer[DSP_RIGHT_CH][0];
    }else if(buff_id == LEFT_OUT){
        buf_ptr = (INT32S *)&dspOutBuffer[DSP_LEFT_CH][0];
    }else{
    }
    return buf_ptr;
}


