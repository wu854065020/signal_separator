/*
 * @Author: wzccccccc
 * @Date: 2023-08-30 16:58:21
 * @LastEditors: wzccccccc wu854065020@gmail.com
 * @LastEditTime: 2024-04-01 14:40:35
 * @FilePath: \signal_separator\User\Inc\save_data.h
 */
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
