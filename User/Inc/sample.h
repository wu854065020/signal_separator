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
    PHASE_LOCKING,
    PHASE_OVER,
} SampleState;


void sampleInit(void);
void sampleSignal(void);
void sampleDmaCallback(void);
void phaseLockLoop(void);
void sampleLoop(void);
void channel1SampleCallBack(void);
void channel2SampleCallBack(void);


#endif
