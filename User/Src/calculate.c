#include "calculate.h"
#include "sample_config.h"

// 三角波三次谐波为基波的1/9，五次谐波为基波的1/25
#define THRID_HARMONIC_THRESHOLD 0.08f
#define FIFTH_HARMONIC_THRESHOLD 0.02f 

// 两个数组分别存最大值和索引，下标0存索引偏小的，下标1存索引偏大的，即下标0存低频，下标1存高频
float g_maxValue[2];
uint16_t g_maxIndex[2];
float g_thirdHarmonic[2] = {0};
float g_fifthHarmonic[2] = {0};

// 该函数输入一个数组，并指定数组的长度。并指定最大值的数目，返回数组中前几个最大值的索引和最大值
void getMaxValue(float *input, uint16_t len, uint16_t maxNum, uint16_t *index, float *maxValue)
{
    for (uint16_t i=0;i<maxNum;i++)
    {
        maxValue[i] = 0;
        index[i] = 0;
    }
    for (uint16_t j=0;j<len;j++) {
        for (uint32_t i = 0; i < maxNum; i++) {
            if (input[j] > maxValue[i]) {
                for (uint32_t k = maxNum - 1; k > i; k--) {
                    maxValue[k] = maxValue[k - 1];
                    index[k] = index[k - 1];
                }
                maxValue[i] = input[j];
                index[i] = j;
                break;
            }
        }
    }
}

#define GET_FREQ(fftindex) ((float)fftindex*SAMPLE_FREQ/FFT_NUM)
#define GET_BASE_FREQ(fftindex) ((uint16_t)((GET_FREQ(fftindex)/5000.0f) + 0.5f) * 5000)
// 从题目可以推导出两个波形的基频应该是幅频曲线中最大的两个值，所以该函数先从幅频数组中找到两个最大值，
// 然后根据两最大值索引可以找到波形的谐波分量，然后根据谐波分量的幅值可以判断波形的类型
void getBaseFreqAndType(float *fftMag, WaveType *waveType, uint32_t *baseFreq)
{
    getMaxValue(fftMag+2, FFT_NUM/2-2, 2, g_maxIndex, g_maxValue);
    g_maxIndex[0] += 2; g_maxIndex[1] += 2;
    if (g_maxIndex[0] > g_maxIndex[1])
    {
        uint16_t tempIndex = g_maxIndex[0];
        float tempValue = g_maxValue[0];
        g_maxIndex[0] = g_maxIndex[1];
        g_maxValue[0] = g_maxValue[1];
        g_maxIndex[1] = tempIndex;
        g_maxValue[1] = tempValue;
    }
    baseFreq[0] = GET_BASE_FREQ(g_maxIndex[0]);
    baseFreq[1] = GET_BASE_FREQ(g_maxIndex[1]);
    uint16_t tmpindex = 0;
    getMaxValue(fftMag+g_maxIndex[0]*3-2, 5, 1, &tmpindex, g_thirdHarmonic);
    getMaxValue(fftMag+g_maxIndex[1]*3-2, 5, 1, &tmpindex, g_thirdHarmonic+1);
    getMaxValue(fftMag+g_maxIndex[0]*5-2, 5, 1, &tmpindex, g_fifthHarmonic);
    getMaxValue(fftMag+g_maxIndex[1]*5-2, 5, 1, &tmpindex, g_fifthHarmonic+1);
    // 若波2基频为波1基频的3倍，波2强行分析，波1减去波2的谐波分量再分析波形
    if ((baseFreq[1] / baseFreq[0] == 3) && (baseFreq[1] % baseFreq[0] == 0)) {
        if (g_thirdHarmonic[1] / g_maxValue[1] > THRID_HARMONIC_THRESHOLD           //判断波二3倍
            // && g_fifthHarmonic[1] / g_maxValue[1] > FIFTH_HARMONIC_THRESHOLD) {
        ) {
            waveType[1] = TRIANGLE;
            g_thirdHarmonic[0] -= ((g_thirdHarmonic[1] * 9) + g_fifthHarmonic[1] * 25) / 2;
            // if (g_thirdHarmonic[0] / g_maxValue[0] > THRID_HARMONIC_THRESHOLD
            if (g_fifthHarmonic[0] / g_maxValue[0] > FIFTH_HARMONIC_THRESHOLD       //判断波一5倍
                // && g_fifthHarmonic[0] / g_maxValue[0] > FIFTH_HARMONIC_THRESHOLD) {
            ) {
                waveType[0] = TRIANGLE;
            } else {
                waveType[0] = SINE;
            }
        } else {
            waveType[1] = SINE;
            // g_thirdHarmonic[0] -= ((g_thirdHarmonic[1] * 9) + g_fifthHarmonic[1] * 25) / 2;
            // if (g_thirdHarmonic[0] / g_maxValue[0] > THRID_HARMONIC_THRESHOLD && 
            if (g_fifthHarmonic[0] / g_maxValue[0] > FIFTH_HARMONIC_THRESHOLD) {    //判断波一5倍
                waveType[0] = TRIANGLE;
            } else {
                waveType[0] = SINE;
            }
        }
    // 若波2基频为波1基频的5倍，波2强行分析波形，波1减去波2的谐波分量后再分析波形
    } else if ((baseFreq[1] / baseFreq[0] == 5) && (baseFreq[1] % baseFreq[0] == 0)) {
        if (g_thirdHarmonic[1] / g_maxValue[1] > THRID_HARMONIC_THRESHOLD           //判断波二3倍
            // && g_fifthHarmonic[1] / g_maxValue[1] > FIFTH_HARMONIC_THRESHOLD) {
        ) {
            waveType[1] = TRIANGLE;
            g_fifthHarmonic[0] -= ((g_thirdHarmonic[1] * 9) + g_fifthHarmonic[1] * 25) / 2;
            if (g_thirdHarmonic[0] / g_maxValue[0] > THRID_HARMONIC_THRESHOLD       //判断波一3倍
                // && g_fifthHarmonic[0] / g_maxValue[0] > FIFTH_HARMONIC_THRESHOLD) {
            ) {
                waveType[0] = TRIANGLE;
            } else {
                waveType[0] = SINE;
            }
        } else {
            waveType[1] = SINE;
            if (g_thirdHarmonic[0] / g_maxValue[0] > THRID_HARMONIC_THRESHOLD       //判断波一3倍
                // && g_fifthHarmonic[0] / g_maxValue[0] > FIFTH_HARMONIC_THRESHOLD) {
            ) {
                waveType[0] = TRIANGLE;
            } else {
                waveType[0] = SINE;
            }
        }
    // 若波2基频为波1基频的5/3倍，波2强行分析波形，波1减去波2的谐波分量后再分析波形
    } else if (((float)baseFreq[1] / (float)baseFreq[0] > 5.0f/3 - 0.1f) && ((float)baseFreq[1] / (float)baseFreq[0] < 5.0f/3 + 0.1f)) {
        // if (g_thirdHarmonic[1] / g_maxValue[1] > THRID_HARMONIC_THRESHOLD
        if (g_fifthHarmonic[1] / g_maxValue[1] > FIFTH_HARMONIC_THRESHOLD           //判断波二5倍
            // && g_fifthHarmonic[1] / g_maxValue[1] > FIFTH_HARMONIC_THRESHOLD) {
        ) {
            waveType[1] = TRIANGLE;
            // g_thirdHarmonic[0] -= ((g_thirdHarmonic[1] * 9) + g_fifthHarmonic[1] * 25) / 2;
            if (g_thirdHarmonic[0] / g_maxValue[0] > THRID_HARMONIC_THRESHOLD       //判断波一3倍
                // && g_fifthHarmonic[0] / g_maxValue[0] > FIFTH_HARMONIC_THRESHOLD) {
            ) {
                waveType[0] = TRIANGLE;
            } else {
                waveType[0] = SINE;
            }
        } else {
            waveType[1] = SINE;
            if (g_thirdHarmonic[0] / g_maxValue[0] > THRID_HARMONIC_THRESHOLD       //判断波一3倍
                // && g_fifthHarmonic[0] / g_maxValue[0] > FIFTH_HARMONIC_THRESHOLD) {
            ) {
                waveType[0] = TRIANGLE;
            } else {
                waveType[0] = SINE;
            }
        }
    } else {
        if (g_thirdHarmonic[1] / g_maxValue[1] > THRID_HARMONIC_THRESHOLD           //判断波二3倍
            // && g_fifthHarmonic[1] / g_maxValue[1] > FIFTH_HARMONIC_THRESHOLD) {
        ) {
            waveType[1] = TRIANGLE;
            if (g_thirdHarmonic[0] / g_maxValue[0] > THRID_HARMONIC_THRESHOLD       //判断波一3倍
                // && g_fifthHarmonic[0] / g_maxValue[0] > FIFTH_HARMONIC_THRESHOLD) {
            ) {
                waveType[0] = TRIANGLE;
            } else {
                waveType[0] = SINE;
            }
        } else {
            waveType[1] = SINE;
            if (g_thirdHarmonic[0] / g_maxValue[0] > THRID_HARMONIC_THRESHOLD       //判断波一3倍
                // && g_fifthHarmonic[0] / g_maxValue[0] > FIFTH_HARMONIC_THRESHOLD) {
            ) {
                waveType[0] = TRIANGLE;
            } else {
                waveType[0] = SINE;
            }
        }
    }
}
