/*
 * @Author: wzccccccc
 * @Description: 信号处理和锁相环闭环部分
 * @Date: 2023-08-02 10:04:40
 * @LastEditors: wzccccccc wu854065020@gmail.com
 * @LastEditTime: 2024-04-01 16:40:23
 * @FilePath: \signal_separator\User\Src\sample.c
 */
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
#include "ad9833.h"
#include "key.h"
#include "ui_control.h"
#include "save_data.h"
/*
  本作品ADC采样可分为两部分，分别为分析信号成分，锁相环部分
  分析信号成分：由于输入信号为5kHz，最大频率为100kHz的三角波和正弦波，所以需要判断3、5次谐波来判断信号种类.
  5次谐波最大频率为100kHz，因此根据奈奎斯特定律，需要采样频率1.024MHz，使得分辨率为1kHz，搭配1024点FFT
  来分析信号成分。
  锁相环部分，由于相位只需要分析一次基波的相位即可，因此只要满足采集基波即可，采样频率设为256kHz，256点FFT，
  从而减少运算量，缩短PID控制周期，提高控制频率。
  还有一点很关键，对dds模块有要求，我们使用的ad9833在调节频率时，相位不会发生改变，同时响应速度快，符合题设
*/
/*
  阅读指南：建议先读sampleLoop函数，该函数为工作状态机，控制整个工作流程，里面实现的功能均已封装为函数，方便观看
  频率漂移部分可以先不看，没有也能用，只是因为太早做完了，求稳多做的功能
*/

#define TEST_FREQ 100000
uint32_t g_testFreq = 100000;

extern uint8_t g_testAuto;
extern DMA_HandleTypeDef hdma_adc1;

uint8_t g_syncSample = 0; //同步信号，三个ADC通道采样完成后，把对应标志位置1
pid_struct_t g_phasePid[2] = {0}; //双通道锁相环PID
float g_refPhase[2] = {0}; //期望相位
float g_phase[2][2] = {0}; //双通道相位
float g_phaseOffset[2] = {0}; //相位偏置（）
float g_deltaPhase[2] = {0};
float g_deltaBasePhase = 0;
float g_lastDeltaPhase[2] = {0};
float g_deltaFreq[2] = {0};
uint16_t g_baseIndex[2]; // 信号数组中基频对应的索引
uint16_t g_outIndex[2]; // 
float g_phaseDiff = 0.0f; // 无用变量
uint8_t g_isGetMsg = 0; // 无用变量

int8_t g_testOffset[2] = {2, 0};
float g_freqOffsetRatioOrigin[2] = {(2.60076431e-05f), (6.42714122e-06f)};
float g_freqOffsetRatio[2] = {(2.60076431e-05f), (6.42714122e-06f)};
static uint8_t steadyFlag = 0;  // 无用变量
static uint16_t g_steadyDiffCnt = 0; // 无用变量
float g_steadyDiff[2] = {0.0f}; // 无用变量，置为0，对结果无影响
float g_firstPhase = 0.0; // 初相位变量
PhaseLockState g_phaseLockState = PHASE_LOCKING; // 锁相环状态机
uint32_t g_phaseLockWaitStartTime = 0;
uint32_t g_phaseLockWaitCurTime = 0;
WorkMode g_workMode = NORMAL_MODE; // 工作模式状态机，不是很重要
uint32_t g_testTime = 0; // 测试变量，可以无视
uint8_t g_KeyEnable = 0; // 使能按键，工作中关闭，防止二次触发
uint8_t isUseWindow = 0; // 是否使用窗函数变量，本作品没用到窗函数，无视
SampleState g_sampleState = SAMPLE_IDLE; // 工作状态机，用于控制工作流程
uint16_t g_signalAdc1[SIGNAL_NUM] = {0}; // 加法器输出ADC采样信号数组
uint16_t g_signalAdc2[SIGNAL_NUM] = {0}; // 通道1输出ADC采样信号数组
uint16_t g_signalAdc3[SIGNAL_NUM] = {0}; // 通道2输出ADC采样信号数组
float g_signalFFT[2*SIGNAL_NUM] = {0}; // 加法器输出频谱数组，实部虚部交替存储
float g_signalVolt[SIGNAL_NUM] = {0}; // 加法器输出频谱幅值数组
uint32_t g_baseFreq[2]; // 两个输入通道信号基频
float g_totalFreq[2];
WaveType g_waveType[2] = {SINE, SINE}; // 两个输入通道信号波形种类

/**
 * @description: 初始化函数
 * @return {*}
 */
void sampleInit(void)
{
    SaveData temp = {0};
    uiSendOffset(g_testOffset[0], g_testOffset[1]);
    pid_init(&g_phasePid[0], 3.0f, 0.02f, 0.1f, 12.0f, 16.0f, 0.0f);
    pid_init(&g_phasePid[1], 3.0f, 0.02f, 0.1f, 12.0f, 16.0f, 0.0f);
    temp = loadOffsetData();
    g_freqOffsetRatio[0] = temp.freqOffset[0];
    g_freqOffsetRatio[1] = temp.freqOffset[1];
}

/**
 * @description: 保存漂移系数
 * @return {*}
 */
void reloadParam(void)
{
    g_freqOffsetRatio[0] = g_freqOffsetRatioOrigin[0];
    g_freqOffsetRatio[1] = g_freqOffsetRatioOrigin[1];
    SaveData t_saveData = {.freqOffset={g_freqOffsetRatioOrigin[0],g_freqOffsetRatioOrigin[1]}};
    saveOffsetData(t_saveData);
}

/**
 * @description: 开始采样
 * @return {*}
 */
void sampleSignal(void)
{
    // 此处使用PWM信号来触发ADC采样，能精准控制采样频率，该PWM信号同时触发三个ADC通道，保证同步采样
    __HAL_TIM_SET_AUTORELOAD(&htim1, 125-1); // 主频128MHz，分频系数125，采样频率1.024MHz
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 62); // 比较值随意，只要有方波形状来提供采样信号即可
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)g_signalAdc1, SIGNAL_NUM);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
}

/**
 * @description: 加法器输出的ADC采样通道回调函数
 * @return {*}
 */
void sampleDmaCallback(void)
{
    // 当状态机处于采样阶段（即还没开始锁相环闭环，需要分析信号成分的采样阶段），对信号FFT之后进入下一状态
    if (g_sampleState == SAMPLEING) {
        HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
        HAL_ADC_Stop_DMA(&hadc1);
        memset(g_signalFFT, 0, sizeof(g_signalFFT));
        for (uint16_t i=0;i<SIGNAL_NUM;i++) {
            if (isUseWindow){
                // 窗函数没用到，因为采样频率除以1024采样点，分辨率为1kHz，而信号为5kHz整数倍，不会发生频谱泄露
                g_signalFFT[2*i] = (float)g_signalAdc1[i] * 3.3f / 4096.0f * blackmanHarris2048[i];
            } else {
                g_signalFFT[2*i] = (float)g_signalAdc1[i] * 3.3f / 4096.0f;
            }
        }
        g_sampleState = SAMPLE_FINISH;
    } else {
        // 锁相环闭环阶段，需保证三个通道采集完成，故利用同步信号来判断
        g_syncSample |= 0x01;
    }
}

/**
 * @description: 输出通道1ADC通道回调函数
 * @return {*}
 */
// ADC2
void channel1SampleCallBack(void)
{
    g_syncSample |= 0x02;
}

/**
 * @description: 输出通道2ADC通道回调函数
 * @return {*}
 */
// ADC3
void channel2SampleCallBack(void)
{
    g_syncSample |= 0x04;
}

/**
 * @description: 锁相环周期采样开始
 * @return {*}
 */
void phaseLockStart(void)
{
    g_syncSample = 0x00; // 同步信号清零
    HAL_ADC_Start_DMA(&hadc1, (uint32_t *)g_signalAdc1, PHASE_LOCKED_FFT_NUM);
    HAL_ADC_Start_DMA(&hadc2, (uint32_t *)g_signalAdc2, PHASE_LOCKED_FFT_NUM);
    HAL_ADC_Start_DMA(&hadc3, (uint32_t *)g_signalAdc3, PHASE_LOCKED_FFT_NUM);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
}

/**
 * @description: 锁相环周期采样停止
 * @return {*}
 */
void phaseLockStop(void)
{
    HAL_TIM_PWM_Stop(&htim1, TIM_CHANNEL_1);
    HAL_ADC_Stop_DMA(&hadc1);
    HAL_ADC_Stop_DMA(&hadc2);
    HAL_ADC_Stop_DMA(&hadc3);
}

// 以下为一些数学函数，用途如名
#define CLAMP(x, min, max) ((x)>(max)?(max):((x)<(min)?(min):(x)))
#define MYABS(x) ((x)>0?(x):-(x))
#define RAD_TO_ANGLE(x) ((180.0f/PI)*(x))
#define CLAMP_RAD(x) ((x)>PI?(x)-2*PI:((x)<-PI?(x)+2*PI:(x)))
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

/**
 * @description: 无用函数，原本用来判断相位是否稳定
 * @return {*}
 */
void getSteady(void)
{
    static uint8_t steadyCnt = 0;
    if (steadyFlag == 0) {
        if (g_phasePid[0].err[0] < 0.07f && g_phasePid[1].err[0] < 0.07f && g_phasePid[0].err[0] > -0.07f && g_phasePid[1].err[0] > -0.07f) {
            steadyCnt++;
            if (steadyCnt > 10) {
                steadyFlag = 1;
                steadyCnt = 0;
            }
        } else {
            steadyCnt = 0;
        }
    }
}

#define MAX_SAMPLE_CNT 31
uint32_t g_offsetCntTick = 0;
uint8_t g_sampleCnt = 0;
float g_totalDeltaPhase[2] = {0}; // 两个通道测试时间内相位差与初始相位差的偏移
/**
 * @description: 自动获取频率漂移前，先进行采样
 * @return {*}
 */
void autoGetFreqOffsetStart(void)
{
    __HAL_TIM_SET_AUTORELOAD(&htim1, 500-1);
    __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 62);
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    g_sampleCnt = 0;
    g_offsetCntTick = 0;
    g_totalDeltaPhase[0] = 0;
    g_totalDeltaPhase[1] = 0;
    g_sampleState = AUTO_SETOFFSET;
    AD9833_SetFrequency(&ad9833Channel1, 80000.0f);
    AD9833_SetFrequency(&ad9833Channel2, 100000.0f);
    HAL_TIM_Base_Start(&htim2);
    phaseLockStart();
}

/**
 * @description: 自动获取频率漂移，我们发现输出与输入的频偏跟输入频率存在一定的线性关系，因此通过测量频率漂移与输入频率的比值，
 *              就可以在不同输入频率下计算出频率漂移，从而实现自动校准，减少频率漂移，实测即使没有锁相环，示波器上信号漂移也很小。
 *              该函数正常工作中不会被调用，可以最后在看。该函数通过串口屏发出指令调用，获得的频率漂移数据会保存在FLASH中。开机自动读取。
 * @return {*}
 */
void autoGetFreqOffset(void)
{
    float tempMax[2] = {0};
    if (g_syncSample == 0x07) {
        g_sampleCnt++;
        if (g_sampleCnt >= MAX_SAMPLE_CNT) {
            g_offsetCntTick = __HAL_TIM_GET_COUNTER(&htim2);
        }
        g_syncSample = 0x00;
        phaseLockStop();
    // base freq
        memset(g_signalFFT, 0, sizeof(g_signalFFT));
        for (uint16_t i=0;i<PHASE_LOCKED_FFT_NUM;i++) {
            g_signalFFT[2*i] = (float)g_signalAdc1[i] * 3.3f / 4096.0f;
        }
        ALterFFT(g_signalFFT);
        GetFFTMag(g_signalFFT, g_signalVolt);
        getBaseFreqMag(g_signalVolt, g_baseIndex, tempMax);
        g_phase[0][0] = atan2f(g_signalFFT[2*g_baseIndex[0]], g_signalFFT[2*g_baseIndex[0]+1]);
        g_phase[1][0] = atan2f(g_signalFFT[2*g_baseIndex[1]], g_signalFFT[2*g_baseIndex[1]+1]);
    // channel 1
        memset(g_signalFFT, 0, sizeof(g_signalFFT));
        for (uint16_t i=0;i<PHASE_LOCKED_FFT_NUM;i++) {
            g_signalFFT[2*i] = (float)g_signalAdc2[i] * 3.3f / 4096.0f;
        }
        ALterFFT(g_signalFFT);
        GetFFTMag(g_signalFFT, g_signalVolt);
        getMaxValue(g_signalVolt+2, PHASE_LOCKED_FFT_NUM/2-2, 1, g_outIndex, tempMax);
        g_outIndex[0] += 2;
        g_phase[0][1] = atan2f(g_signalFFT[2*g_outIndex[0]], g_signalFFT[2*g_outIndex[0]+1]);
    // channel 2
        memset(g_signalFFT, 0, sizeof(g_signalFFT));
        for (uint16_t i=0;i<PHASE_LOCKED_FFT_NUM;i++) {
            g_signalFFT[2*i] = (float)g_signalAdc3[i] * 3.3f / 4096.0f;
        }
        ALterFFT(g_signalFFT);
        GetFFTMag(g_signalFFT, g_signalVolt);
        getMaxValue(g_signalVolt+2, PHASE_LOCKED_FFT_NUM/2-2, 1, g_outIndex+1, tempMax);
        g_outIndex[1] += 2;
        g_phase[1][1] = atan2f(g_signalFFT[2*g_outIndex[1]], g_signalFFT[2*g_outIndex[1]+1]);
        g_lastDeltaPhase[0] = g_deltaPhase[0];
        g_lastDeltaPhase[1] = g_deltaPhase[1];
        g_deltaPhase[0] = g_phase[0][1] - g_phase[0][0];
        g_deltaPhase[1] = g_phase[1][1] - g_phase[1][0];
        if (g_sampleCnt >= 2) {
            g_totalDeltaPhase[0] += get_delta_rad(g_deltaPhase[0], g_lastDeltaPhase[0]);
            g_totalDeltaPhase[1] += get_delta_rad(g_deltaPhase[1], g_lastDeltaPhase[1]);
        }
        if (g_sampleCnt >= MAX_SAMPLE_CNT) {
            g_sampleCnt = 0;
            g_totalDeltaPhase[0] /= 2*PI;
            g_totalDeltaPhase[1] /= 2*PI;
            g_freqOffsetRatio[0] = g_totalDeltaPhase[0] / g_offsetCntTick * 1000000.0f / 80000.0f;
            g_freqOffsetRatio[1] = g_totalDeltaPhase[1] / g_offsetCntTick * 1000000.0f / TEST_FREQ;
            // AD9833_SetFrequency(&ad9833Channel1, 80000.0f + g_freqOffsetRatio[0] * 80000.0f);
            // AD9833_SetFrequency(&ad9833Channel2, TEST_FREQ + g_freqOffsetRatio[1] * TEST_FREQ);
            g_sampleState = SAMPLE_IDLE;
            SaveData temp = {0};
            temp.freqOffset[0] = g_freqOffsetRatio[0];
            temp.freqOffset[1] = g_freqOffsetRatio[1];
            saveOffsetData(temp);
            phaseLockStop();
            g_testAuto = 1;
        } else {
            phaseLockStart();
        }
    }
}

#define STEADY_CNT_MAX 10
uint32_t last_tick = 0;
uint32_t current_tick = 0;
uint32_t delta_tick = 0;
void phaseLockLoop(void)
{
    float tempMax[2];
    uint8_t n = g_baseFreq[1] / g_baseFreq[0];
    n = n>2?n:2;
    if (g_sampleState == PHASE_LOCK) {
        if (g_syncSample == 0x07) {
            current_tick = HAL_GetTick();
            delta_tick = current_tick - last_tick;
            last_tick = current_tick;
            phaseLockStop();
        // base freq
            memset(g_signalFFT, 0, sizeof(g_signalFFT));
            for (uint16_t i=0;i<PHASE_LOCKED_FFT_NUM;i++) {
                g_signalFFT[2*i] = (float)g_signalAdc1[i] * 3.3f / 4096.0f;
            }
            ALterFFT(g_signalFFT);
            GetFFTMag(g_signalFFT, g_signalVolt);
            getBaseFreqMag(g_signalVolt, g_baseIndex, tempMax);
            g_phase[0][0] = atan2f(g_signalFFT[2*g_baseIndex[0]], g_signalFFT[2*g_baseIndex[0]+1]);
            g_phase[1][0] = atan2f(g_signalFFT[2*g_baseIndex[1]], g_signalFFT[2*g_baseIndex[1]+1]);
        // channel 1
            memset(g_signalFFT, 0, sizeof(g_signalFFT));
            for (uint16_t i=0;i<PHASE_LOCKED_FFT_NUM;i++) {
                g_signalFFT[2*i] = (float)g_signalAdc2[i] * 3.3f / 4096.0f;
            }
            ALterFFT(g_signalFFT);
            GetFFTMag(g_signalFFT, g_signalVolt);
            getMaxValue(g_signalVolt+2, PHASE_LOCKED_FFT_NUM/2-2, 1, g_outIndex, tempMax);
            g_outIndex[0] += 2;
            g_phase[0][1] = atan2f(g_signalFFT[2*g_outIndex[0]], g_signalFFT[2*g_outIndex[0]+1]);
        // channel 2
            memset(g_signalFFT, 0, sizeof(g_signalFFT));
            for (uint16_t i=0;i<PHASE_LOCKED_FFT_NUM;i++) {
                g_signalFFT[2*i] = (float)g_signalAdc3[i] * 3.3f / 4096.0f;
            }
            ALterFFT(g_signalFFT);
            GetFFTMag(g_signalFFT, g_signalVolt);
            getMaxValue(g_signalVolt+2, PHASE_LOCKED_FFT_NUM/2-2, 1, g_outIndex+1, tempMax);
            g_outIndex[1] += 2;
            g_phase[1][1] = atan2f(g_signalFFT[2*g_outIndex[1]], g_signalFFT[2*g_outIndex[1]+1]);
            // for (uint16_t i=0;i<SIGNAL_NUM;i++) {
            //     SendJustFloat6(g_signalVolt[i],(float)g_signalAdc1[i] * 3.3f / 4096.0f);
            // }
        // 使用pid算法实现数字锁相，相位差作反馈，输出差分频率
            g_lastDeltaPhase[0] = g_deltaPhase[0];
            g_lastDeltaPhase[1] = g_deltaPhase[1];
            g_deltaPhase[0] = g_phase[0][1] - g_phase[0][0];
            g_deltaPhase[1] = g_phase[1][1] - g_phase[1][0];
            if (g_workMode == PHASE_MODE) {
                if (g_phaseLockState == PHASE_LOCKING) {
                    // getSteady();
                    // if (steadyFlag == 1) {
                    //     g_phaseLockState = PHASE_LOCKED;
                    // }
                    g_phaseLockState = PHASE_LOCKED;
                } else if (g_phaseLockState == PHASE_LOCKED) {
                    // g_steadyDiff[0] += g_phasePid[0].err[0];
                    // g_steadyDiff[1] += g_phasePid[1].err[0];
                    // g_steadyDiffCnt++;
                    // if (g_steadyDiffCnt >= STEADY_CNT_MAX) {
                    //     g_steadyDiff[0] /= STEADY_CNT_MAX;
                    //     g_steadyDiff[1] /= STEADY_CNT_MAX;
                    //     g_phaseLockState = GOT_PHASE_DIFF;
                    // }
                    g_phaseLockState = GOT_PHASE_DIFF;
                } else if (g_phaseLockState == GOT_PHASE_DIFF) {
                    while (g_phase[0][0] >= PI/n) {
                        g_phase[0][0] -= 2*PI/n;
                    }
                    while (g_phase[0][0] < -PI/n) {
                        g_phase[0][0] += 2*PI/n;
                    }
                    g_deltaBasePhase = n*g_phase[0][0] - g_phase[1][0];
                    if (g_deltaBasePhase > PI) {
                        g_deltaBasePhase -= 2*PI;
                    } else if (g_deltaBasePhase < -PI) {
                        g_deltaBasePhase += 2*PI;
                    }
                }
                // 这里是调节初相位功能，涉及数学公式运算，参考电赛报告，唯一需要注意的是，原本只需调一个通道相位，
                // 但为了稳定性考虑，将单个通道需要调节的相位除以2，并让一个通道超前半个调节相位，另一个通道滞后半个调节相位
                // g_steadyDiff没用到，我把它置0了，注意！！！
                float isGetPi = (n%2)?0:PI;
                g_deltaFreq[0] = pid_calc(&g_phasePid[0], 0, 
                    get_delta_rad(g_deltaPhase[0], g_refPhase[0]-g_deltaBasePhase/2/n+isGetPi/2/n+g_firstPhase/2/n-g_steadyDiff[0]));
                g_deltaFreq[1] = pid_calc(&g_phasePid[1], 0, 
                    get_delta_rad(g_deltaPhase[1], g_refPhase[1]-g_steadyDiff[1]+g_deltaBasePhase/2-isGetPi/2-g_firstPhase/2));
            } else if (g_workMode == NORMAL_MODE) {
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
            }
            g_outOffset[0] = g_baseFreq[0] * g_freqOffsetRatio[0];
            g_outOffset[1] = g_baseFreq[1] * g_freqOffsetRatio[1];
            g_totalFreq[0] = g_baseFreq[0] - g_deltaFreq[0] + g_outOffset[0];
            g_totalFreq[1] = g_baseFreq[1] - g_deltaFreq[1] + g_outOffset[1];
            AD9833_SetFrequency(&ad9833Channel1, g_totalFreq[0]);
            AD9833_SetFrequency(&ad9833Channel2, g_totalFreq[1]);
            // AD9833_SetFrequency(F1);
            g_syncSample = 0x00;
            phaseLockStart();
        }
    }
}

/**
 * @description: 工作状态机循环，放在主函数循环中，用于控制工作流程
 * @return {*}
 */
void sampleLoop(void)
{
    uint32_t lastTime = 0;
    uint32_t nowTime = 0;
    uint8_t key = 0;
    // 按下按键开始工作，同时暂时锁死按键，避免二次触发
    if (g_KeyEnable == 0x00) {
        key = key_scan();
        if (key == 0x01) {
            uiSendWaveInf(0, 0, 0, 0);
            if (g_sampleState == PHASE_LOCK) {
                phaseLockStop();
            }
            g_sampleState = SAMPLE_INIT; // 按键触发进入初始化
            g_KeyEnable = 0x01;
        }
    }
    switch (g_sampleState)
    {
        // 初始状态，开始采样，进入下一状态
        case SAMPLE_INIT:
            if (g_workMode != Test_MODE) {
                sampleSignal();
                g_sampleState = SAMPLEING;
            } else {
                AD9833_SetWaveform(&ad9833Channel1, wave_sine);
                AD9833_SetWaveform(&ad9833Channel2, wave_sine);
                AD9833_SetFrequency(&ad9833Channel1, g_testFreq);
                AD9833_SetFrequency(&ad9833Channel2, g_testFreq);
                g_sampleState = TEST_STATUS;
            }
            break;
        // 采样完成，进行FFT分析，进入下一状态
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
        // FFT分析完成，获取基频和波形种类，同时设置输出DDS的波形和频率，进入下一状态
        case FFT_FINISH:
            getBaseFreqAndType(g_signalFFT, g_signalVolt, g_waveType, g_baseFreq);
            uiSendWaveInf(g_baseFreq[0], g_baseFreq[1], g_waveType[0], g_waveType[1]);
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
            AD9833_SetFrequency(&ad9833Channel1, g_baseFreq[0]);
            AD9833_SetFrequency(&ad9833Channel2, g_baseFreq[1]);
            g_sampleState = GET_WARE_FINISH;
            break;
        // 设置采样频率为256kHz，采样点改为256点，开始锁相环闭环，进入下一状态
        case GET_WARE_FINISH:
            g_sampleState = PHASE_LOCK;
            __HAL_TIM_SET_AUTORELOAD(&htim1, 500-1);
            __HAL_TIM_SetCompare(&htim1, TIM_CHANNEL_1, 62);
            g_phasePid[0].i_out = 0;
            g_phasePid[1].i_out = 0;
            phaseLockStart();
            g_KeyEnable = 0x00;
            break;
        // 锁相环闭环阶段，调用锁相环函数
        case PHASE_LOCK:
            phaseLockLoop();
            delta_tick = current_tick - last_tick;
            // HAL_Delay((20-delta_tick>0)?(20-delta_tick):0);
            break;
        // 锁相环闭环结束，进入空闲状态
        case PHASE_OVER:
            g_sampleState = SAMPLE_IDLE;
            phaseLockStop();
            g_KeyEnable = 0x00;
            break;
        // 测试模式
        case TEST_STATUS:
            g_totalFreq[0] = g_testFreq  + g_testOffset[0];
            g_totalFreq[1] = g_testFreq  + g_testOffset[1];
            AD9833_SetFrequency(&ad9833Channel1, g_totalFreq[0]);
            AD9833_SetFrequency(&ad9833Channel2, g_totalFreq[1]);
            HAL_Delay(8);
            break;
        // 自动获取频率漂移
        case AUTO_SETOFFSET:
            autoGetFreqOffset();
            break;
        default:
            break;
    }
}

/**
 * @description: 切换工作模式
 * @param {WorkMode} mode
 * @return {*}
 */
void changeWorkMode(WorkMode mode)
{
    if (mode != g_workMode) {
        g_sampleState = PHASE_OVER;
        if (mode == PHASE_MODE) {
            g_steadyDiffCnt = 0;
            g_steadyDiff[0] = 0;
            g_steadyDiff[1] = 0;
            g_phaseLockState = PHASE_LOCKING;
            g_deltaBasePhase = 0;
        }
        g_workMode = mode;
    }
}

/**
 * @description: 设置初相位差
 * @param {float} phase
 * @return {*}
 */
void setFirstPhase(float phase)
{
    g_firstPhase = phase;
}

/**
 * @description: 设置频率漂移系数
 * @param {uint8_t} wave
 * @param {uint8_t} offset
 * @return {*}
 */
void setFreqOffsetRatio(uint8_t wave, uint8_t offset)
{
    g_testOffset[wave] = offset;
    g_freqOffsetRatio[wave] = (float)offset / g_testFreq;
}
