/*
 * @Author: wzccccccc
 * @Date: 2023-08-02 12:10:01
 * @LastEditors: wzccccccc wu854065020@gmail.com
 * @LastEditTime: 2024-04-01 14:34:32
 * @FilePath: \signal_separator\User\Src\packge.c
 */
#include "packge.h"
#include "usart.h"

uint8_t GetCheck(uint8_t *src, uint16_t len)
{
    uint8_t check = 0;
    while(len--)
    {
        check += *src;
        src++;
    }
    return check;
}

void SendJustFloat2(float input1, float input2)
{
    JustFloat2 justpack;
    justpack.data[0] = input1;
    justpack.data[1] = input2;
    justpack.tail[0] = 0x00;
    justpack.tail[1] = 0x00;
    justpack.tail[2] = 0x80;
    justpack.tail[3] = 0x7F;
    HAL_UART_Transmit(&huart1, (uint8_t *)&justpack, sizeof(JustFloat2), 0xFFFF);
}

void SendJustFloat6(float input1, float input2, float input3, float input4, float input5, float input6)
{
    JustFloat6 justpack;
    justpack.data[0] = input1;
    justpack.data[1] = input2;
    justpack.tail[0] = 0x00;
    justpack.tail[1] = 0x00;
    justpack.tail[2] = 0x80;
    justpack.tail[3] = 0x7F;
    HAL_UART_Transmit(&huart1, (uint8_t *)&justpack, sizeof(JustFloat6), 0xFFFF);
}


