#ifndef __Timer_H
#define __Timer_H

#include "main.h"  // 替换为F4 HAL库头文件，删除F1头文件

// 外部声明全局计时变量
extern uint32_t g_timer3_count;

// 外部声明定时器句柄（在main.c中定义）
extern TIM_HandleTypeDef htim3;

// 函数声明
void Timer_Init(void);

#endif
