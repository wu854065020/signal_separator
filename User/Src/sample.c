#include <string.h>
#include "sample.h"
#include "main.h"
#include "adc.h"
#include "tim.h"
#include "myfft.h"
#include "packge.h"
#include "sample_config.h"
#include "blackmanharris2048.h"
#include "calculate.h"

extern DMA_HandleTypeDef hdma_adc1;

uint32_t g_testTime = 0;

uint8_t isUseWindow = 0;
SampleState g_sampleState = SAMPLE_INIT;
uint16_t g_signalAdc[SIGNAL_NUM] = {0};
float g_signalFFT[2*SIGNAL_NUM] = {0};
float g_signalVolt[SIGNAL_NUM] = {0};
uint32_t g_baseFreq[2];
WaveType g_waveType[2] = {3, 3};


// void sampleInit(void)
// {
//     HAL_DMA_RegisterCallback(&hdma_adc1, HAL_DMA_XFER_CPLT_CB_ID, sampleDmaCallback);
// }

void sampleSignal(void)
{
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)g_signalAdc, SIGNAL_NUM);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
}

void sampleDmaCallback(void)
{
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    HAL_ADC_Stop_DMA(&hadc1);
    memset(g_signalFFT, 0, sizeof(g_signalFFT));
    for (uint16_t i=0;i<SIGNAL_NUM;i++)
    {
        if (isUseWindow){
            g_signalFFT[2*i] = (float)g_signalAdc[i] * 3.3f / 4096.0f * blackmanHarris2048[i];
        } else {
            g_signalFFT[2*i] = (float)g_signalAdc[i] * 3.3f / 4096.0f;
        }
    }
    g_sampleState = SAMPLE_FINISH;
}

void sampleLoop(void)
{
    uint32_t lastTime = 0;
    uint32_t nowTime = 0;
    switch (g_sampleState)
    {
        case SAMPLE_INIT:
            sampleSignal();
            g_sampleState = SAMPLEING;
            break;
        case SAMPLE_FINISH:
            lastTime = HAL_GetTick();
            MyFFT(g_signalFFT);
            GetFFTMag(g_signalFFT, g_signalVolt);
            nowTime = HAL_GetTick();
            g_testTime = nowTime - lastTime;
            for (uint16_t i=0;i<SIGNAL_NUM;i++) {
                SendJustFloat(g_signalVolt[i],(float)g_signalAdc[i] * 3.3f / 4096.0f);
            }
            g_sampleState = FFT_FINISH;
            break;
        case FFT_FINISH:
            getBaseFreqAndType(g_signalVolt, g_waveType, g_baseFreq);
            g_sampleState = GET_WARE_FINISH;
            break;
        case GET_WARE_FINISH:
            break;
        default:
            break;
    }
}



