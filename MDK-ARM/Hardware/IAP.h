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

// Flash起始地址
// 0x08000000  +---------------------------+
//             |   IAP Bootloader程序区    |  扇区0，32KB，单片机上电优先运行，无任何预留空间
// 0x08008000  +---------------------------+
//             |   升级标志位专属区         |  扇区2，16KB，F4最小擦除单位，和IAP区无缝衔接
// 0x0800C000  +---------------------------+
//             |                           |
//             |                           |
//             |      用户APP程序区         |  扇区3-11，共1024-32-16=976KB，紧跟标志位区，最大化APP可用空间
//             |                           |
//             |                           |
// 0x080E0000  +---------------------------+  Flash最高地址（1MB）
