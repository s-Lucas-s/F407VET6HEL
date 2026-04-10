#ifndef __IAP_H
#define __IAP_H

// 1. 替换F1头文件为F4 HAL库头文件
#include "main.h"
#include "stm32f4xx_hal_flash.h"
#include "stm32f4xx_hal_flash_ex.h"

/**********************************************************
*** STM32F407VET6 Flash参数（根据你的芯片型号调整）
*** F407VET6：512KB Flash，每页2KB，共256页
*** 最后一页起始地址 = 0x08000000 + 512*1024 - 2*1024 = 0x0807FC00
**********************************************************/
#define FLASH_BASE_ADDR        0x08000000    // F407 Flash起始地址
#define FLASH_PAGE_SIZE        2048          // F407每页2KB（F1是1KB）
#define FLASH_TOTAL_SIZE       512 * 1024    // F407VET6总容量512KB

// IAP标志存储地址（最后一页起始 + 页内偏移0）
#define IAP_FLAG_PAGE_ADDR     (FLASH_BASE_ADDR + FLASH_TOTAL_SIZE - FLASH_PAGE_SIZE)
#define IAP_FLAG_ADDR          (IAP_FLAG_PAGE_ADDR + 0x00) // 存储升级标志

// 中断向量表偏移（F407适配，和F1逻辑一致）
#define NVIC_VectTab_FLASH_OFFSET	0x2000

// 升级标志定义（保留原代码，无修改）
#define IAP_UPGRADE_FLAG       0xABCD1234    // 代表“需要升级”
#define IAP_NORMAL_FLAG        0x00000000    // 代表“正常运行”

// 函数声明（新增Read函数实现）
uint32_t IAP_ReadFlag(void);
void IAP_WriteFlag(uint32_t flag);

#endif
