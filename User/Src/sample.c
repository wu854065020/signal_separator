#include <string.h>
#include <math.h>
#include "sample.h"
#include "main.h"
#include "adc.h"
#include "tim.h"
#include "myfft.h"
#include "usart.h"
#include "packge.h"
#include "sample_config.h"
#include "blackmanharris2048.h"
#include "calculate.h"
#include "pid.h"
#include "arm_math.h"
#include "ad9833.h"
#include "key.h"

extern DMA_HandleTypeDef hdma_usart2_rx;
extern DMA_HandleTypeDef hdma_adc1;

#define UART2_RX_BUF_LEN 64
uint8_t g_communityRxBuf[UART2_RX_BUF_LEN];

uint8_t g_syncSample = 0;
pid_struct_t g_phasePid[2] = {0};
float g_refPhase[2] = {0};
float g_phase[2][2] = {0};
float g_phaseOffset[2] = {0};
float g_deltaPhase[2] = {0};
float g_lastDeltaPhase[2] = {0};
float g_deltaFreq[2] = {0};
uint16_t g_baseIndex[2];
uint16_t g_outIndex[2];
float g_phaseDiff = 0.0f;
uint8_t g_isGetMsg = 0;

PhaseLockState g_phaseLockState = PHASE_LOCKING;
uint32_t g_phaseLockWaitStartTime = 0;
uint32_t g_phaseLockWaitCurTime = 0;
WorkMode g_workMode = NORMAL_MODE;
uint32_t g_testTime = 0;
uint8_t g_KeyEnable = 0;
uint8_t isUseWindow = 0;
SampleState g_sampleState = SAMPLE_IDLE;
uint16_t g_signalAdc1[SIGNAL_NUM] = {0};
uint16_t g_signalAdc2[SIGNAL_NUM] = {0};
uint16_t g_signalAdc3[SIGNAL_NUM] = {0};
float g_signalFFT[2*SIGNAL_NUM] = {0};
float g_signalVolt[SIGNAL_NUM] = {0};
uint32_t g_baseFreq[2];
WaveType g_waveType[2] = {SINE, SINE};


void sampleInit(void)
{
    pid_init(&g_phasePid[0], 0.6f, 0.003f, 0.1f, 0.7f, 3.0f, 0.0f);
    pid_init(&g_phasePid[1], 0.4f, 0.002f, 0.1f, 0.7f, 3.0f, 0.0f);
    HAL_UART_Receive_DMA(&huart2, g_communityRxBuf, UART2_RX_BUF_LEN);
}

void communityUartCallBack(void)
{
    uint16_t len = UART2_RX_BUF_LEN - __HAL_DMA_GET_COUNTER(&hdma_usart2_rx);
    g_isGetMsg = 1;
    if (len == 4) {
        g_phaseDiff = *((float *)g_communityRxBuf);
    }
    HAL_UART_Receive_DMA(&huart2, g_communityRxBuf, UART2_RX_BUF_LEN);
}

void sampleSignal(void)
{
    __HAL_TIM_SET_AUTORELOAD(&htim1, 125-1);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 62);
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
    } else if (g_sampleState == PHASE_LOCK) {
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
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)g_signalAdc1, PHASE_LOCKED_FFT_NUM);
    HAL_ADC_Start_DMA(&hadc2, (uint32_t *)g_signalAdc2, PHASE_LOCKED_FFT_NUM);
    HAL_ADC_Start_DMA(&hadc3, (uint32_t *)g_signalAdc3, PHASE_LOCKED_FFT_NUM);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
}

void phaseLockStop(void)
{
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    HAL_ADC_Stop_DMA(&hadc1);
    HAL_ADC_Stop_DMA(&hadc2);
    HAL_ADC_Stop_DMA(&hadc3);
}

#define CLAMP(x, min, max) ((x)>(max)?(max):((x)<(min)?(min):(x)))
#define MYABS(x) ((x)>0?(x):-(x))
#define RAD_TO_ANGLE(x) ((180.0f/PI)*(x))
#define CLAMP_RAD(x) ((x)>PI?(x)-2*PI:((x)<-PI?(x)+2*PI:(x)))
float g_freq1OffsetRatio = (12.0f/90000.0f);
float g_freq2OffsetRatio =  (-1.0f/80000.0f);
float g_outOffset[2] = {0};

float get_delta_rad(float ang1, float ang2){
    float delta = ang1 - ang2;
    while (delta >= PI) {
        delta -= 2*PI;
    }
    while (delta < -PI) {
        delta += 2*PI;
    }
    return delta;
}

void phaseLockLoop(void)
{
    float tempMax;
    if (g_sampleState == PHASE_LOCK) {
        if (g_syncSample == 0x07) {
            HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
            HAL_ADC_Stop_DMA(&hadc1);
            HAL_ADC_Stop_DMA(&hadc2);
            HAL_ADC_Stop_DMA(&hadc3);
        // 还原波形类型
            if (g_waveType[0] == TRIANGLE) {
                AD9833_SetWaveform(&ad9833Channel1, wave_triangle);
            } else {
                AD9833_SetWaveform(&ad9833Channel1, wave_sine);
            }
            if (g_waveType[1] == TRIANGLE) {
                AD9833_SetWaveform(&ad9833Channel2, wave_triangle);
            } else {
                AD9833_SetWaveform(&ad9833Channel2, wave_sine);
            }
        // 还原波形基频
            AD9833_SetFrequency(&ad9833Channel1, g_baseFreq[0]);
            AD9833_SetFrequency(&ad9833Channel2, g_baseFreq[1]);
        // base freq
            memset(g_signalFFT, 0, sizeof(g_signalFFT));
            for (uint16_t i=0;i<PHASE_LOCKED_FFT_NUM;i++) {
                g_signalFFT[2*i] = (float)g_signalAdc1[i] * 3.3f / 4096.0f;
            }
            ALterFFT(g_signalFFT);
            GetFFTMag(g_signalFFT, g_signalVolt);
            getBaseFreqMag(g_signalVolt, g_baseIndex, &tempMax);
            g_phase[0][0] = atan2f(g_signalFFT[2*g_baseIndex[0]], g_signalFFT[2*g_baseIndex[0]+1]);
            g_phase[1][0] = atan2f(g_signalFFT[2*g_baseIndex[1]], g_signalFFT[2*g_baseIndex[1]+1]);
        // channel 1
            memset(g_signalFFT, 0, sizeof(g_signalFFT));
            for (uint16_t i=0;i<PHASE_LOCKED_FFT_NUM;i++) {
                g_signalFFT[2*i] = (float)g_signalAdc2[i] * 3.3f / 4096.0f;
            }
            ALterFFT(g_signalFFT);
            GetFFTMag(g_signalFFT, g_signalVolt);
            getMaxValue(g_signalVolt+2, PHASE_LOCKED_FFT_NUM/2-2, 1, g_outIndex, &tempMax);
            g_outIndex[0] += 2;
            g_phase[0][1] = atan2f(g_signalFFT[2*g_outIndex[0]], g_signalFFT[2*g_outIndex[0]+1]);
        // channel 2
            memset(g_signalFFT, 0, sizeof(g_signalFFT));
            for (uint16_t i=0;i<PHASE_LOCKED_FFT_NUM;i++) {
                g_signalFFT[2*i] = (float)g_signalAdc3[i] * 3.3f / 4096.0f;
            }
            ALterFFT(g_signalFFT);
            GetFFTMag(g_signalFFT, g_signalVolt);
            getMaxValue(g_signalVolt+2, PHASE_LOCKED_FFT_NUM/2-2, 1, g_outIndex+1, &tempMax);
            // for (uint16_t i=0;i<SIGNAL_NUM;i++) {
            //     SendJustFloat6(g_signalVolt[i],(float)g_signalAdc1[i] * 3.3f / 4096.0f);
            // }
        // 使用pid算法实现数字锁相，相位差作反馈，输出差分频率
            g_outIndex[1] += 2;
            g_phase[1][1] = atan2f(g_signalFFT[2*g_outIndex[1]], g_signalFFT[2*g_outIndex[1]+1]);
            g_lastDeltaPhase[0] = g_deltaPhase[0];
            g_lastDeltaPhase[1] = g_deltaPhase[1];
            g_deltaPhase[0] = g_phase[0][1] - g_phase[0][0];
            g_deltaPhase[1] = g_phase[1][1] - g_phase[1][0];
            if (g_workMode == PHASE_MODE) {
                if (g_phaseLockState == PHASE_LOCKING) {
                    g_phaseLockWaitCurTime = HAL_GetTick();
                    if (g_phaseLockWaitCurTime - g_phaseLockWaitStartTime > 1000) {

                        g_phaseLockState = PHASE_LOCKED;
                    }
                }
            }
            // if (g_deltaPhase[0] > PI) {
            //     g_deltaPhase[0] -= 2*PI;
            // } else if (g_deltaPhase[0] < -PI) {
            //     g_deltaPhase[0] += 2*PI;
            // }
            // if (g_deltaPhase[1] > PI) {
            //     g_deltaPhase[1] -= 2*PI;
            // } else if (g_deltaPhase[1] < -PI) {
            //     g_deltaPhase[1] += 2*PI;
            // }
            g_deltaFreq[0] = pid_calc(&g_phasePid[0], 0, get_delta_rad(g_deltaPhase[0], g_refPhase[0]));
            g_deltaFreq[1] = pid_calc(&g_phasePid[1], 0, get_delta_rad(g_deltaPhase[1], g_refPhase[1]));
            g_outOffset[0] = g_baseFreq[0] * g_freq1OffsetRatio;
            g_outOffset[1] = g_baseFreq[1] * g_freq2OffsetRatio;
            AD9833_SetFrequency(&ad9833Channel1, g_baseFreq[0] - g_deltaFreq[0] + g_outOffset[0]);
            AD9833_SetFrequency(&ad9833Channel2, g_baseFreq[1] - g_deltaFreq[1] + g_outOffset[1]);
            // AD9833_SetFrequency(F1);
            g_syncSample = 0x00;
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
    uint8_t key = 0;
    if (g_KeyEnable == 0x00) {
        key = key_scan();
        if (key == 0x01) {
            if (g_sampleState == PHASE_LOCK) {
                phaseLockStop();
            }
            g_sampleState = SAMPLE_INIT;
            g_KeyEnable = 0x01;
        }
    }
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
            // for (uint16_t i=0;i<SIGNAL_NUM;i++) {
            //     SendJustFloat(g_signalVolt[i],(float)g_signalAdc1[i] * 3.3f / 4096.0f);
            // }
            g_sampleState = FFT_FINISH;
            break;
        case FFT_FINISH:
            getBaseFreqAndType(g_signalFFT, g_signalVolt, g_waveType, g_baseFreq);
            g_sampleState = GET_WARE_FINISH;
            break;
        case GET_WARE_FINISH:
            g_sampleState = PHASE_LOCK;
            __HAL_TIM_SET_AUTORELOAD(&htim1, 500-1);
            __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 62);
            phaseLockStart();
            g_KeyEnable = 0x00;
            break;
        case PHASE_LOCK:
            last_tick = HAL_GetTick();
            phaseLockLoop();
            current_tick = HAL_GetTick();
            delta_tick = current_tick - last_tick;
            // HAL_Delay((20-delta_tick>0)?(20-delta_tick):0);
            break;
        case PHASE_OVER:
            g_sampleState = SAMPLE_IDLE;
            phaseLockStop();
            break;
        default:
            break;
    }
}

void changeWorkMode(WorkMode mode)
{
    if (mode == PHASE_MODE) {
        g_workMode = mode;
        g_phaseLockState = PHASE_LOCKING;
        g_phaseLockWaitStartTime = HAL_GetTick();
    }
}

