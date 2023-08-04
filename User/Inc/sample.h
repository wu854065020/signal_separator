#ifndef SAMPLE_H_
#define SAMPLE_H_

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
void sampleSignal(void);
void sampleDmaCallback(void);
void phaseLockLoop(void);
void sampleLoop(void);
void channel1SampleCallBack(void);
void channel2SampleCallBack(void);
void changeWorkMode(WorkMode mode);


#endif
