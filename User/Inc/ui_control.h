#ifndef UI_CONTROL_H_
#define UI_CONTROL_H_

#include "stm32f4xx_hal.h"

#define PHASE_CONTROL_HEAD 0xA1
#define PHASE_CONTROL_LENGTH 6 
typedef __packed struct
{
    uint8_t head;
    uint8_t phase;
    uint8_t enable;
    uint8_t tail[3];
} PhaseControlFrame;

#define TEST_CONTROL_HEAD 0xA2
#define TEST_CONTROL_LENGTH 5
typedef __packed struct
{
    uint8_t head;
    uint8_t enable;
    uint8_t tail[3];
}TestControlFrame;



void uiControlInit(void);
void uiUartCallBack(void);
void uiSendWaveInf(uint32_t wave1Freq, uint32_t wave2Freq, uint8_t wave1Type, uint8_t wave2Type);

#endif
