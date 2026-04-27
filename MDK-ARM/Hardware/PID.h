#ifndef __PID_H
#define __PID_H

#include "main.h" // 仅替换sys.h为F407 HAL库标准头文件，其余完全不变

// 坐标结构体定义
typedef struct
{
    float x; // 横坐标
    float y; // 纵坐标
} Point2D;

typedef struct parameter_pid
{
    float kp;
    float ki;
    float kd;
} pid_t;

extern pid_t PID_Pool_X[4];
extern pid_t PID_Pool_Y[4];

extern float err_x;
extern float err_y;

void PID_Init(void);
void PID_Control(float in_target_x, float in_target_y);

#endif
