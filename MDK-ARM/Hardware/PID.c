#include "PID.h"
#include "Emm_V5.h"
#include <math.h>
#include "stm32F4xx_hal.h"

extern bool Stop_flag;            // 停机标志

/************************ 全局宏定义 ************************/
#define Integral_MAX 500000.0f // PID积分限幅最大值，防止积分饱和失控
#define MAX_SPEED 1800        // 电机最大输出限制，保护电机与机械结构
#define PI 3.1415926f          // 圆周率，用于圆形轨迹计算，备用参数

#define LASER_FIX_X    160.0f    // 你赛前标定的激光坐标值
#define LASER_FIX_Y    120.0f

#define Get_square(x) ((x) * (x)) // 计算平方
#define LIMIT_VALUE_SYMMETRIC(value, max_val) \
    do                                      \
    {                                       \
        if ((value) > (max_val))            \
        {                                   \
            (value) = (max_val);            \
        }                                   \
        else if ((value) < -(max_val))      \
        {                                   \
            (value) = -(max_val);           \
        }                                   \
    } while (0)
#define LIMIT_SYMMETRIC(value) LIMIT_VALUE_SYMMETRIC(value, Integral_MAX) // 积分专用限幅

/************************ 枚举与结构体定义 ************************/
typedef enum
{
    x, // X轴控制
    y  // Y轴控制
} Dimension_t;

typedef struct parameter_pid
{
    float kp;
    float ki;
    float kd;
} pid_t;

/************************ 全局PID参数实体 ************************/
pid_t Position_PID_x;
pid_t Position_PID_y;

/************************ 轨迹与前馈控制参数 ************************/
float Target_Vertical_x = 0.0f; // X轴目标运动速度（像素/帧）
float Target_Vertical_y = 0.0f; // Y轴目标运动速度（像素/帧）

float target_x = 0.0f; // X轴目标坐标
float target_y = 0.0f; // Y轴目标坐标

const int32_t Kf_x = 0; // X轴前馈系数
const int32_t Kf_y = 0; // Y轴前馈系数

/************************ PID内部静态变量 ************************/
static float xerr_last = 0.0f;
static float yerr_last = 0.0f;
static uint32_t time_count_last = 0;
static float integral_x = 0.0f;
static float integral_y = 0.0f;

/************************ 工具函数 ************************/
static inline int32_t LimitOutput(int32_t value)
{
    LIMIT_VALUE_SYMMETRIC(value, MAX_SPEED);
    return value;
}

int32_t Position_PID_Control(Dimension_t Dimension, float err, uint32_t dt)
{
    float pid_out = 0.0f;
    if (dt == 0)
    {
        dt = 1;
    }

    if (Dimension == x)
    {
        integral_x += err;
        LIMIT_VALUE_SYMMETRIC(integral_x, Integral_MAX);
        pid_out = Position_PID_x.kp * err + Position_PID_x.ki * integral_x + Position_PID_x.kd * (err - xerr_last) / dt;
    }
    else
    {
        integral_y += err;
        LIMIT_VALUE_SYMMETRIC(integral_y, Integral_MAX);
        pid_out = Position_PID_y.kp * err + Position_PID_y.ki * integral_y + Position_PID_y.kd * (err - yerr_last) / dt;
    }

    LIMIT_VALUE_SYMMETRIC(pid_out, MAX_SPEED);
    return (int32_t)(pid_out + 0.5f);
}

void PID_Init(void)
{
    Position_PID_x.kp = 2.5f;
    Position_PID_x.ki = 0.0f;
    Position_PID_x.kd = 0.0f;

    Position_PID_y.kp = 2.5f;
    Position_PID_y.ki = 0.0f;
    Position_PID_y.kd = 0.0f;
}

void Trajectory_Update(uint32_t dt)
{
    target_x += Target_Vertical_x * dt / 1000.0f;
    target_y += Target_Vertical_y * dt / 1000.0f;
}

void PID_Control(float target_x, float target_y)
{
    int32_t x_out;
    int32_t y_out;
    uint32_t time_count = HAL_GetTick();
    uint32_t interval_time = time_count - time_count_last;

    if (interval_time > 100)
    {
        interval_time = 100;
    }
    if (interval_time == 0)
    {
        interval_time = 1;
    }

    time_count_last = time_count;

    if (!Stop_flag)
    {
        xerr_last = 0.0f;
        yerr_last = 0.0f;
        integral_x = 0.0f;
        integral_y = 0.0f;
        target_x = 0.0f;
        target_y = 0.0f;
        return;
    }

    float err_x = LASER_FIX_X - target_x;
    float err_y = LASER_FIX_Y - target_y;

    int32_t pid_x = Position_PID_Control(x, err_x, interval_time);
    int32_t pid_y = Position_PID_Control(y, err_y, interval_time);

    int32_t feed_x = (int32_t)(Target_Vertical_x * Kf_x);
    int32_t feed_y = (int32_t)(Target_Vertical_y * Kf_y);

    x_out = pid_x + feed_x;
    y_out = pid_y + feed_y;

    // 输出限幅，保护电机
    LIMIT_VALUE_SYMMETRIC(x_out, MAX_SPEED);
    LIMIT_VALUE_SYMMETRIC(y_out, MAX_SPEED);

    if (x_out >= 0)
    {
        Emm_V5_Vel_Control(1, 1, (uint16_t)x_out, 0, 0);
    }
    else
    {
        Emm_V5_Vel_Control(1, 0, (uint16_t)(-x_out), 0, 0);
    }

   //Delay_us(1000);
   //HAL_Delay(1);

    if (y_out >= 0)
    {
        Emm_V5_Vel_Control(2, 0, (uint16_t)y_out, 0, 0);
    }
    else
    {
        Emm_V5_Vel_Control(2, 1, (uint16_t)(-y_out), 0, 0);
    }

    xerr_last = err_x;
    yerr_last = err_y;
}

