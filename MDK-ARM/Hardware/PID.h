#ifndef __PID_H
#define __PID_H

#include "stm32f4xx_hal.h"
#include <stdbool.h>
typedef enum {
    x, // X轴控制
    y  // Y轴控制
} Dimension_t;

extern float Target_Vertical_x; // X轴目标速度
extern float Target_Vertical_y; // Y轴目标速度

extern int8_t Questionx; // 当前题目编号（1-5）

void PID_Init(void);
void PID_SetTargetVertical(Dimension_t xy, float Target_Vertical_xy);
void PID_Control(float err_x, float err_y);

#endif
