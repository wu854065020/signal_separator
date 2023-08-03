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
