#include "calculate.h"
#include "sample_config.h"

// 三角波三次谐波为基波的1/9，五次谐波为基波的1/25，为了提高鲁棒性，把阈值调低了
#define THRID_HARMONIC_THRESHOLD 0.08f
#define FIFTH_HARMONIC_THRESHOLD 0.02f 

// 两个数组分别存最大值和索引，下标0存索引偏小的，下标1存索引偏大的，即下标0存低频，下标1存高频
float g_maxValue[2];
float g_maxFFT[2][2];
uint16_t g_maxIndex[2];
float g_thirdHarmonicMag[2] = {0};
float g_fifthHarmonicMag[2] = {0};
float g_thirdHarmonic[2][2] = {0};
float g_fifthHarmonic[2][2] = {0};

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

#define THIRD_NUM 1.0f/9
#define FIFTH_NUM 1.0f/25
#define GET_FREQ(fftindex) ((float)fftindex*SAMPLE_FREQ/FFT_NUM)
#define GET_BASE_FREQ(fftindex) ((uint16_t)((GET_FREQ(fftindex)/5000.0f) + 0.5f) * 5000)
/*
与下面找基频函数不同的是，这个是锁相环所使用的256点fft找基频
*/
void getBaseFreqMag(float *fftMag, uint16_t *index, float *maxValue)
{
    // 加2避免判断到直流分量
    getMaxValue(fftMag+2, PHASE_LOCKED_FFT_NUM/2-2, 2, index, maxValue);
    index[0] += 2; index[1] += 2;
    if (index[0] > index[1])
    {
        uint16_t tempIndex = index[0];
        float tempValue = maxValue[0];
        index[0] = index[1];
        maxValue[0] = maxValue[1];
        index[1] = tempIndex;
        maxValue[1] = tempValue;
    }
}

/*
从题目可以推导出两个波形的基频应该是幅频曲线中最大的两个值，所以该函数先从幅频数组中找到两个最大值，
然后根据两最大值索引可以找到波形的谐波分量，然后根据谐波分量的幅值可以判断波形的类型
但其实直接判断幅度不是很合理，因为波形幅度不是简单的相加，要考虑相位，但实测够用了，所以没做那么复杂
*/
void getBaseFreqAndType(float *fft, float *fftMag, WaveType *waveType, uint32_t *baseFreq)
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
    g_maxFFT[0][0] = fft[2*g_maxIndex[0]];
    g_maxFFT[0][1] = fft[2*g_maxIndex[0]+1];
    g_maxFFT[1][0] = fft[2*g_maxIndex[1]];
    g_maxFFT[1][1] = fft[2*g_maxIndex[1]+1];
    baseFreq[0] = GET_BASE_FREQ(g_maxIndex[0]);
    baseFreq[1] = GET_BASE_FREQ(g_maxIndex[1]);
    uint16_t tmpMax = 0;
    uint16_t tmpindex[4] = {0};
    getMaxValue(fftMag+g_maxIndex[0]*3-2, 5, 1, tmpindex, g_thirdHarmonicMag);
    getMaxValue(fftMag+g_maxIndex[1]*3-2, 5, 1, tmpindex+1, g_thirdHarmonicMag+1);
    getMaxValue(fftMag+g_maxIndex[0]*5-2, 5, 1, tmpindex+2, g_fifthHarmonicMag);
    getMaxValue(fftMag+g_maxIndex[1]*5-2, 5, 1, tmpindex+3, g_fifthHarmonicMag+1);
    g_thirdHarmonic[0][0] = fft[2*tmpindex[0]];
    g_thirdHarmonic[0][1] = fft[2*tmpindex[0]+1];
    g_thirdHarmonic[1][0] = fft[2*tmpindex[1]];
    g_thirdHarmonic[1][1] = fft[2*tmpindex[1]+1];
    g_fifthHarmonic[0][0] = fft[2*tmpindex[2]];
    g_fifthHarmonic[0][1] = fft[2*tmpindex[2]+1];
    g_fifthHarmonic[1][0] = fft[2*tmpindex[3]];
    g_fifthHarmonic[1][1] = fft[2*tmpindex[3]+1];
    // 若波2基频为波1基频的3倍，波2强行分析，波1减去波2的谐波分量再分析波形
    if ((baseFreq[1] / baseFreq[0] == 3) && (baseFreq[1] % baseFreq[0] == 0)) {
        // g_maxFFT[1][0] -= ();
        if (g_thirdHarmonicMag[1] / g_maxValue[1] > THRID_HARMONIC_THRESHOLD           //判断波二3倍
            // && g_fifthHarmonicMag[1] / g_maxValue[1] > FIFTH_HARMONIC_THRESHOLD) {
        ) {
            waveType[1] = TRIANGLE;
            g_thirdHarmonicMag[0] -= ((g_thirdHarmonicMag[1] * 9) + g_fifthHarmonicMag[1] * 25) / 2;
            // if (g_thirdHarmonicMag[0] / g_maxValue[0] > THRID_HARMONIC_THRESHOLD
            if (g_fifthHarmonicMag[0] / g_maxValue[0] > FIFTH_HARMONIC_THRESHOLD       //判断波一5倍
                // && g_fifthHarmonicMag[0] / g_maxValue[0] > FIFTH_HARMONIC_THRESHOLD) {
            ) {
                waveType[0] = TRIANGLE;
            } else {
                waveType[0] = SINE;
            }
        } else {
            waveType[1] = SINE;
            // g_thirdHarmonicMag[0] -= ((g_thirdHarmonicMag[1] * 9) + g_fifthHarmonicMag[1] * 25) / 2;
            // if (g_thirdHarmonicMag[0] / g_maxValue[0] > THRID_HARMONIC_THRESHOLD && 
            if (g_fifthHarmonicMag[0] / g_maxValue[0] > FIFTH_HARMONIC_THRESHOLD) {    //判断波一5倍
                waveType[0] = TRIANGLE;
            } else {
                waveType[0] = SINE;
            }
        }
    // 若波2基频为波1基频的5倍，波2强行分析波形，波1减去波2的谐波分量后再分析波形
    } else if ((baseFreq[1] / baseFreq[0] == 5) && (baseFreq[1] % baseFreq[0] == 0)) {
        if (g_thirdHarmonicMag[1] / g_maxValue[1] > THRID_HARMONIC_THRESHOLD           //判断波二3倍
            // && g_fifthHarmonicMag[1] / g_maxValue[1] > FIFTH_HARMONIC_THRESHOLD) {
        ) {
            waveType[1] = TRIANGLE;
            g_fifthHarmonicMag[0] -= ((g_thirdHarmonicMag[1] * 9) + g_fifthHarmonicMag[1] * 25) / 2;
            if (g_thirdHarmonicMag[0] / g_maxValue[0] > THRID_HARMONIC_THRESHOLD       //判断波一3倍
                // && g_fifthHarmonicMag[0] / g_maxValue[0] > FIFTH_HARMONIC_THRESHOLD) {
            ) {
                waveType[0] = TRIANGLE;
            } else {
                waveType[0] = SINE;
            }
        } else {
            waveType[1] = SINE;
            if (g_thirdHarmonicMag[0] / g_maxValue[0] > THRID_HARMONIC_THRESHOLD       //判断波一3倍
                // && g_fifthHarmonicMag[0] / g_maxValue[0] > FIFTH_HARMONIC_THRESHOLD) {
            ) {
                waveType[0] = TRIANGLE;
            } else {
                waveType[0] = SINE;
            }
        }
    // 若波2基频为波1基频的5/3倍，波2强行分析波形，波1减去波2的谐波分量后再分析波形
    } else if (((float)baseFreq[1] / (float)baseFreq[0] > 5.0f/3 - 0.1f) && ((float)baseFreq[1] / (float)baseFreq[0] < 5.0f/3 + 0.1f)) {
        // if (g_thirdHarmonicMag[1] / g_maxValue[1] > THRID_HARMONIC_THRESHOLD
        if (g_fifthHarmonicMag[1] / g_maxValue[1] > FIFTH_HARMONIC_THRESHOLD           //判断波二5倍
            // && g_fifthHarmonicMag[1] / g_maxValue[1] > FIFTH_HARMONIC_THRESHOLD) {
        ) {
            waveType[1] = TRIANGLE;
            // g_thirdHarmonicMag[0] -= ((g_thirdHarmonicMag[1] * 9) + g_fifthHarmonicMag[1] * 25) / 2;
            if (g_thirdHarmonicMag[0] / g_maxValue[0] > THRID_HARMONIC_THRESHOLD       //判断波一3倍
                // && g_fifthHarmonicMag[0] / g_maxValue[0] > FIFTH_HARMONIC_THRESHOLD) {
            ) {
                waveType[0] = TRIANGLE;
            } else {
                waveType[0] = SINE;
            }
        } else {
            waveType[1] = SINE;
            if (g_thirdHarmonicMag[0] / g_maxValue[0] > THRID_HARMONIC_THRESHOLD       //判断波一3倍
                // && g_fifthHarmonicMag[0] / g_maxValue[0] > FIFTH_HARMONIC_THRESHOLD) {
            ) {
                waveType[0] = TRIANGLE;
            } else {
                waveType[0] = SINE;
            }
        }
    } else {
        if (g_thirdHarmonicMag[1] / g_maxValue[1] > THRID_HARMONIC_THRESHOLD           //判断波二3倍
            // && g_fifthHarmonicMag[1] / g_maxValue[1] > FIFTH_HARMONIC_THRESHOLD) {
        ) {
            waveType[1] = TRIANGLE;
            if (g_thirdHarmonicMag[0] / g_maxValue[0] > THRID_HARMONIC_THRESHOLD       //判断波一3倍
                // && g_fifthHarmonicMag[0] / g_maxValue[0] > FIFTH_HARMONIC_THRESHOLD) {
            ) {
                waveType[0] = TRIANGLE;
            } else {
                waveType[0] = SINE;
            }
        } else {
            waveType[1] = SINE;
            if (g_thirdHarmonicMag[0] / g_maxValue[0] > THRID_HARMONIC_THRESHOLD       //判断波一3倍
                // && g_fifthHarmonicMag[0] / g_maxValue[0] > FIFTH_HARMONIC_THRESHOLD) {
            ) {
                waveType[0] = TRIANGLE;
            } else {
                waveType[0] = SINE;
            }
        }
    }
}
