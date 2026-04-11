#include "IAP.h"

/**
 * @brief  读取升级标志位（APP端，地址和IAP完全一致）
 * @retval 标志值
 */
uint32_t IAP_ReadFlag(void)
{
    return *(volatile uint32_t *)IAP_FLAG_ADDR;
}

/**
 * @brief  写入升级标志位（适配STM32F407 HAL库）
 * @param  flag: 要写入的标志
 */
void IAP_WriteFlag(uint32_t flag)
{
    FLASH_EraseInitTypeDef EraseInitStruct = {0};
    uint32_t SectorError = 0;

    // 关闭中断
    __disable_irq();
    // Flash解锁
    HAL_FLASH_Unlock();

    EraseInitStruct.TypeErase = FLASH_TYPEERASE_SECTORS;
    EraseInitStruct.Sector = FLASH_SECTOR_2; // 对应 0x08008000
    EraseInitStruct.NbSectors = 1;
    EraseInitStruct.VoltageRange = FLASH_VOLTAGE_RANGE_3;

    // 执行擦除
    HAL_FLASHEx_Erase(&EraseInitStruct, &SectorError);

    HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                      IAP_FLAG_ADDR,
                      flag);

    // 锁定Flash
    HAL_FLASH_Lock();
    // 开启中断
    __enable_irq();
}

static uint8_t USART3_ReceiveDataCount = 0;
/**
 * @brief  检测连续5个0x7E，触发Bootloader（和你原代码逻辑完全一致）
 * @param  data: 串口接收到的字节
 */

__attribute__((optnone)) 
void check_Bootloader(uint8_t data)
{
    if (data == 0x7E)
    {
        if (++USART3_ReceiveDataCount >= 5)
        {
            USART3_ReceiveDataCount = 0;
            // 写入升级标志，复位
            IAP_WriteFlag(IAP_UPGRADE_FLAG);
            HAL_NVIC_SystemReset();
        }
    }
    else
    {
        USART3_ReceiveDataCount = 0;
    }
}