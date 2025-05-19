/*
 * @Author: wzccccccc
 * @Date: 2023-08-02 10:04:38
 * @LastEditors: wzccccccc wu854065020@gmail.com
 * @LastEditTime: 2024-04-01 14:40:32
 * @FilePath: \signal_separator\User\Inc\sample.h
 */
#ifndef SAMPLE_H_
#define SAMPLE_H_

#include "arm_math.h"

#define DEG_TO_RAD (PI/180.0f)
typedef enum
{
    SAMPLE_IDLE = 0,
    SAMPLE_INIT,
    SAMPLEING,
    SAMPLE_FINISH,
    FFT_FINISH,
    GET_WARE_FINISH,
    PHASE_LOCK,
    PHASE_OVER,
    MORE_FREQ,
    TEST_STATUS,
    AUTO_SETOFFSET,
    
} SampleState;

typedef enum
{
    NORMAL_MODE = 0,
    PHASE_MODE,
    Test_MODE,
} WorkMode;

typedef enum
{
    PHASE_LOCKING,
    PHASE_LOCKED,
    GOT_PHASE_DIFF,
    PHASE_LOCK_OK,
} PhaseLockState;

void sampleInit(void);
void reloadParam(void);
void sampleSignal(void);
void sampleDmaCallback(void);
void phaseLockLoop(void);
void sampleLoop(void);
void channel1SampleCallBack(void);
void channel2SampleCallBack(void);
void changeWorkMode(WorkMode mode);
void setFirstPhase(float phase);
void setFreqOffsetRatio(uint8_t wave, uint8_t offset);
void autoGetFreqOffsetStart(void);
void autoGetFreqOffset(void);


#endif
