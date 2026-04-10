#include "Timer.h"

// 【唯一定义】全局计时变量（仅此处定义，main.c不重复定义）
uint32_t g_timer3_count = 0;

/**
  * @brief  定时器初始化（启动TIM3中断）
  */
void Timer_Init(void)
{
    // 启动TIM3基础定时器和中断
    HAL_TIM_Base_Start_IT(&htim3);
}



