#ifndef SAVE_DATA_H_
#define SAVE_DATA_H_

#include "stm32f4xx_hal.h"

typedef struct
{
    float freqOffset[2];
} SaveData;

void saveOffsetData(SaveData data);
SaveData loadOffsetData(void);


#endif
