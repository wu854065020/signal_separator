/*
 * @Author: wzccccccc
 * @Date: 2023-08-30 16:58:21
 * @LastEditors: wzccccccc wu854065020@gmail.com
 * @LastEditTime: 2024-04-01 14:34:40
 * @FilePath: \signal_separator\User\Src\key.c
 */
#include "key.h"


uint8_t key_scan(void)
{
    uint8_t key = 0;
    if (!HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin))
    {
        HAL_Delay(10);
        if (!HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin))
        {
            key |= 0x01;
        }
    }
    return key;
}
