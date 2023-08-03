#ifndef PACKGE_H
#define PACKGE_H
#include <stdint.h>

typedef __packed struct
{
    float data[2];
    uint8_t tail[4];
}JustFloat2;
typedef __packed struct
{
    float data[6];
    uint8_t tail[4];
}JustFloat6;

typedef __packed struct
{
    uint8_t header;     //0xA5
    float wave;
    uint8_t check;      
    uint8_t tail;       //0x5A
}TransPack;

uint8_t GetCheck(uint8_t *src, uint16_t len);
void SendJustFloat2(float input1, float input2);
void SendJustFloat6(float input1, float input2, float input3, float input4, float input5, float input6);

#endif
