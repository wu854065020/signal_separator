#ifndef SAMPLE_H_
#define SAMPLE_H_

typedef enum
{
    SAMPLE_INIT = 0,
    SAMPLEING,
    SAMPLE_FINISH,
    FFT_FINISH,
    GET_WARE_FINISH,
} SampleState;

// void sampleInit(void);
void sampleSignal(void);
void sampleDmaCallback(void);
void sampleLoop(void);

#endif
