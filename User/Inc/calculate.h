#ifndef CALCULATE_H_
#define CALCULATE_H_

#include <stdint.h>

typedef enum
{
    SINE = 0,
    TRIANGLE,
} WaveType;

// 获取波形的基频和波形类型，由于频率为5K的整数倍，故基频采用整形即可
void getBaseFreqAndType(float *fftMag, WaveType *waveType, uint32_t *baseFreq);

#endif
