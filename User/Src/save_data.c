#include "save_data.h"
#include "stm32f4xx_hal_flash.h"
#include "stm32f4xx_hal_flash_ex.h"

#define SAVE_ADDRESS 0x08060000

void saveOffsetData(SaveData data)
{
    FLASH_EraseInitTypeDef My_Flash;  
	HAL_FLASH_Unlock();               
    __HAL_FLASH_DATA_CACHE_DISABLE();
    My_Flash.TypeErase = FLASH_TYPEERASE_SECTORS;
    My_Flash.Sector = FLASH_SECTOR_7;
    My_Flash.NbSectors = 1;      
    My_Flash.VoltageRange = FLASH_VOLTAGE_RANGE_3;                 
                
    uint32_t PageError = 0;
    HAL_FLASHEx_Erase(&My_Flash, &PageError);

    uint32_t *temp = NULL;
    temp = (uint32_t *)&data;
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, SAVE_ADDRESS, temp[0]);
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, SAVE_ADDRESS+4, temp[1]);
    __HAL_FLASH_DATA_CACHE_ENABLE();
    HAL_FLASH_Lock();    
}

SaveData loadOffsetData(void)
{
    SaveData ret = {0};
    uint32_t *pointData = NULL;
    pointData = (uint32_t *)&ret;
    pointData[0] = *(__IO uint32_t *)SAVE_ADDRESS;
    pointData[1] = *(__IO uint32_t *)(SAVE_ADDRESS+4);
    return ret;
}