#ifndef __SYS_H
#define __SYS_H

/* ——————————include—————————— */
#include "main.h"        // 替换 stm32f10x.h，F407 HAL 库标准头文件
#include "Emm_V5.h"
#include "OLED.h"
#include "PID.h"
#include "usart.h"
#include "Serial.h"
#include "Serial1.h"
#include "Key.h"
#include "IAP.h"

/* ————————————变量———————————— */
extern bool Stop_flag;
extern int8_t Questionx;
extern bool Power_on_flag;

/* —————————————宏————————————— */
#define ABS(x)      ((x) >= 0 ? (x) : -(x))
#define Max_x_angle 180

/* ————————————函数———————————— */
float Check_angle(uint8_t addr);
void check_Bootloader(uint8_t data);

#endif
