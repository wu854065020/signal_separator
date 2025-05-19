/*
 * @Author: wzccccccc
 * @Date: 2023-08-02 17:31:56
 * @LastEditors: wzccccccc wu854065020@gmail.com
 * @LastEditTime: 2024-04-01 14:40:04
 * @FilePath: \signal_separator\User\Inc\calculate.h
 */
#ifndef CALCULATE_H_
#define CALCULATE_H_

#include <stdint.h>

typedef enum
{
    Init,
    SINE,
    TRIANGLE,
} WaveType;

// 获取波形的基频和波形类型，由于频率为5K的整数倍，故基频采用整形即可
void getMaxValue(float *input, uint16_t len, uint16_t maxNum, uint16_t *index, float *maxValue);
void getBaseFreqMag(float *fftMag, uint16_t *index, float *maxValue);
void getBaseFreqAndType(float *fft, float *fftMag, WaveType *waveType, uint32_t *baseFreq);

#endif
