#ifndef __PID_H
#define __PID_H

#include "main.h" // 仅替换sys.h为F407 HAL库标准头文件，其余完全不变

// 坐标结构体定义
typedef struct
{
    float x; // 横坐标
    float y; // 纵坐标
} Point2D;

extern float target_x;          // X轴目标坐标
extern float target_y;          // Y轴目标坐标
extern float err_x;
extern float err_y;

void PID_Init(void);
void PID_Control(float now_x, float now_y);

#endif
