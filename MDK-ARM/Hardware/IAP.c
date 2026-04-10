#include "IAP.h"

uint32_t IAP_ReadFlag(void)
{
    return *(uint32_t *)IAP_FLAG_ADDR;
}

void IAP_WriteFlag(uint32_t flag)
{
    FLASH_EraseInitTypeDef erase_init = {FLASH_TYPEERASE_SECTORS, FLASH_VOLTAGE_RANGE_3, FLASH_SECTOR_7, 1};
    uint32_t page_error;
    
    __disable_irq();
    HAL_FLASH_Unlock();
    
    HAL_FLASHEx_Erase(&erase_init, &page_error);
    
    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, IAP_FLAG_ADDR, flag);
    
    HAL_FLASH_Lock();
    __enable_irq();
}
