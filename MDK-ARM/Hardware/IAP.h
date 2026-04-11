#ifndef __IAP_H
#define __IAP_H

#include "stm32f4xx_hal.h"

#define IAP_FLAG_ADDR       0x08008000         // 标志位存储地址（扇区1起始地址）
#define APPLICATION_ADDRESS 0x0800C000         // APP程序起始地址：扇区2起始
#define NVIC_VECTTAB_OFFSET 0x0000C000         // APP向量表偏移

// 升级标志值（和IAP工程完全一致）
#define IAP_UPGRADE_FLAG 0xABCD1234 // 需要升级标志

// 函数声明
uint32_t IAP_ReadFlag(void);
void IAP_WriteFlag(uint32_t flag);
void check_Bootloader(uint8_t data);

#endif
