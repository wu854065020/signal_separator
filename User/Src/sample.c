#include <string.h>
#include <math.h>
#include "sample.h"
#include "main.h"
#include "adc.h"
#include "tim.h"
#include "myfft.h"
#include "packge.h"
#include "sample_config.h"
#include "blackmanharris2048.h"
#include "calculate.h"
#include "pid.h"
#include "arm_math.h"
#include "ad9833.h"

extern DMA_HandleTypeDef hdma_adc1;

uint8_t g_syncSample = 0;
pid_struct_t g_phasePid[2] = {0};
float g_phase[2][2] = {0};
float g_deltaPhase[2] = {0};
float g_deltaFreq[2] = {0};
uint16_t g_baseIndex[2];
uint16_t g_outIndex[2];

uint32_t g_testTime = 0;

uint8_t isUseWindow = 0;
SampleState g_sampleState = 10;
uint16_t g_signalAdc1[SIGNAL_NUM] = {0};
uint16_t g_signalAdc2[SIGNAL_NUM] = {0};
uint16_t g_signalAdc3[SIGNAL_NUM] = {0};
float g_signalFFT[2*SIGNAL_NUM] = {0};
float g_signalVolt[SIGNAL_NUM] = {0};
uint32_t g_baseFreq[2];
WaveType g_waveType[2] = {3, 3};



void sampleInit(void)
{
    pid_init(&g_phasePid[0], 0.07f, 0.0003f, 0.1f, 1.0f, 2.0f, 0.0f);
    pid_init(&g_phasePid[1], 0.1f, 0.01f, 0.0f, 2.0f, 4.0f, 0.0f);
}

void sampleSignal(void)
{
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)g_signalAdc1, SIGNAL_NUM);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
}

void sampleDmaCallback(void)
{
    if (g_sampleState == SAMPLEING) {
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
        HAL_ADC_Stop_DMA(&hadc1);
        memset(g_signalFFT, 0, sizeof(g_signalFFT));
        for (uint16_t i=0;i<SIGNAL_NUM;i++) {
            if (isUseWindow){
                g_signalFFT[2*i] = (float)g_signalAdc1[i] * 3.3f / 4096.0f * blackmanHarris2048[i];
            } else {
                g_signalFFT[2*i] = (float)g_signalAdc1[i] * 3.3f / 4096.0f;
            }
        }
        g_sampleState = SAMPLE_FINISH;
    } else if (g_sampleState == PHASE_LOCKING) {
        g_syncSample |= 0x01;
    }
}
// ADC2
void channel1SampleCallBack(void)
{
    g_syncSample |= 0x02;
}
// ADC3
void channel2SampleCallBack(void)
{
    g_syncSample |= 0x04;
}

void phaseLockStart(void)
{
    g_syncSample = 0x00;
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)g_signalAdc1, SIGNAL_NUM);
    HAL_ADC_Start_DMA(&hadc2, (uint32_t *)g_signalAdc2, SIGNAL_NUM);
    HAL_ADC_Start_DMA(&hadc3, (uint32_t *)g_signalAdc3, SIGNAL_NUM);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
}

void phaseLockStop(void)
{
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    HAL_ADC_Stop_DMA(&hadc1);
    HAL_ADC_Stop_DMA(&hadc2);
    HAL_ADC_Stop_DMA(&hadc3);
}

void phaseLockLoop(void)
{
    float tempMax;
    if (g_sampleState == PHASE_LOCKING) {
        if (g_syncSample == 0x07) {
            HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
            HAL_ADC_Stop_DMA(&hadc1);
            HAL_ADC_Stop_DMA(&hadc2);
            HAL_ADC_Stop_DMA(&hadc3);
            // base freq
            memset(g_signalFFT, 0, sizeof(g_signalFFT));
            for (uint16_t i=0;i<SIGNAL_NUM;i++) {
                g_signalFFT[2*i] = (float)g_signalAdc1[i] * 3.3f / 4096.0f;
            }
            MyFFT(g_signalFFT);
            GetFFTMag(g_signalFFT, g_signalVolt);
            getBaseFreqMag(g_signalVolt, g_baseIndex, &tempMax);
            g_phase[0][0] = atan2f(g_signalFFT[2*g_baseIndex[0]], g_signalFFT[2*g_baseIndex[0]+1]);
            g_phase[1][0] = atan2f(g_signalFFT[2*g_baseIndex[1]], g_signalFFT[2*g_baseIndex[1]+1]);
            // channel 1
            memset(g_signalFFT, 0, sizeof(g_signalFFT));
            for (uint16_t i=0;i<SIGNAL_NUM;i++) {
                g_signalFFT[2*i] = (float)g_signalAdc2[i] * 3.3f / 4096.0f;
            }
            MyFFT(g_signalFFT);
            GetFFTMag(g_signalFFT, g_signalVolt);
            getMaxValue(g_signalVolt+2, FFT_NUM/2-2, 1, g_outIndex, &tempMax);
            g_outIndex[0] += 2;
            g_phase[0][1] = atan2f(g_signalFFT[2*g_outIndex[0]], g_signalFFT[2*g_outIndex[0]+1]);
            // channel 2
            memset(g_signalFFT, 0, sizeof(g_signalFFT));
            for (uint16_t i=0;i<SIGNAL_NUM;i++) {
                g_signalFFT[2*i] = (float)g_signalAdc3[i] * 3.3f / 4096.0f;
            }
            MyFFT(g_signalFFT);
            GetFFTMag(g_signalFFT, g_signalVolt);
            getMaxValue(g_signalVolt+2, FFT_NUM/2-2, 1, g_outIndex, &tempMax);
            g_outIndex[1] += 2;
            g_phase[1][1] = atan2f(g_signalFFT[2*g_outIndex[1]], g_signalFFT[2*g_outIndex[1]+1]);
            g_deltaPhase[0] = g_phase[0][1] - g_phase[0][0];
            g_deltaPhase[1] = g_phase[1][1] - g_phase[1][0];
            if (g_deltaPhase[0] > PI) {
                g_deltaPhase[0] -= 2*PI;
            } else if (g_deltaPhase[0] < -PI) {
                g_deltaPhase[0] += 2*PI;
            }
            if (g_deltaPhase[1] > PI) {
                g_deltaPhase[1] -= 2*PI;
            } else if (g_deltaPhase[1] < -PI) {
                g_deltaPhase[1] += 2*PI;
            }
            g_deltaFreq[0] = pid_calc(&g_phasePid[0], 0, g_deltaPhase[0]);
            g_deltaFreq[1] = pid_calc(&g_phasePid[1], 0, g_deltaPhase[1]);
            AD9833_SetFrequency(g_baseFreq[0] - g_deltaFreq[0]);
            g_syncSample == 0x00;
            phaseLockStart();
        }
    }
}

uint32_t last_tick = 0;
uint32_t current_tick = 0;
uint32_t delta_tick = 0;
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
            for (uint16_t i=0; i<SIGNAL_NUM; i++)
            {
                g_signalFFT[2*i+1] = 0;
            }
            MyFFT(g_signalFFT);
            GetFFTMag(g_signalFFT, g_signalVolt);
            nowTime = HAL_GetTick();
            g_testTime = nowTime - lastTime;
            for (uint16_t i=0;i<SIGNAL_NUM;i++) {
                SendJustFloat(g_signalVolt[i],(float)g_signalAdc1[i] * 3.3f / 4096.0f);
            }
            g_sampleState = FFT_FINISH;
            break;
        case FFT_FINISH:
            getBaseFreqAndType(g_signalFFT, g_signalVolt, g_waveType, g_baseFreq);
            g_sampleState = GET_WARE_FINISH;
            break;
        case GET_WARE_FINISH:
            g_sampleState = PHASE_LOCKING;
            phaseLockStart();
            break;
        case PHASE_LOCKING:
            last_tick = HAL_GetTick();
            phaseLockLoop();
            current_tick = HAL_GetTick();
            delta_tick = current_tick - last_tick;
            // HAL_Delay((20-delta_tick>0)?(20-delta_tick):0);
            break;
        default:
            break;
    }
}



