#include "PID.h"
#include "Emm_V5.h"
#include <math.h>
#include "stm32F4xx_hal.h"

extern bool Stop_flag;            // 停机标志

/************************ 全局宏定义 ************************/
#define Integral_MAX 500000.0f // PID积分限幅最大值，防止积分饱和失控
#define MAX_SPEED 16000        // 电机最大输出限制，保护电机与机械结构
#define PI 3.1415926f          // 圆周率，用于圆形轨迹计算，备用参数

#define Integration_Separation_Threshold 100.0f // 积分分离阈值

/************************ 枚举与结构体定义 ************************/
// 控制维度枚举：区分X轴/Y轴控制
typedef enum
{
    x, // X轴控制
    y  // Y轴控制
} Dimension_t;

// PID参数结构体：封装位置环PID的比例/积分/微分系数
typedef struct parameter_pid
{
    int32_t kp; // 比例系数：快速减小误差
    int32_t ki; // 积分系数：消除静态误差
    int32_t kd; // 微分系数：抑制震荡、提高响应速度
} pid_t;

typedef struct
{
    float y_prev;
} LPF_t;

/************************ PID内部静态变量 ************************/
static int32_t xerr_last = 0;        // X轴上一帧误差，用于微分计算
static int32_t yerr_last = 0;        // Y轴上一帧误差，用于微分计算
static uint32_t time_count_last = 0; // 上一帧控制时间戳，计算时间间隔dt
static int32_t integral_x = 0;       // X轴积分累加值
static int32_t integral_y = 0;       // Y轴积分累加值

static LPF_t lpf_now_x; // 视觉X坐标预处理滤波（内部用）
static LPF_t lpf_now_y; // 视觉Y坐标预处理滤波（内部用）
static LPF_t lpf_d_x;   // X轴PID微分滤波（内部用）
static LPF_t lpf_d_y;   // Y轴PID微分滤波（内部用）

/************************ 全局变量（仅需暴露给外部）************************/
pid_t Position_PID_x;
pid_t Position_PID_y;
float Target_Vertical_x = 0;
float Target_Vertical_y = 0;
float target_x = 0;
float target_y = 0;
const int32_t Kf_x = 10;
const int32_t Kf_y = 10;

/**
 * @brief  PID参数初始化函数
 * @note   上电默认清零PID参数，调试时再赋值
 * @retval 无
 */
void PID_Init(void)
{
    Position_PID_y.kp = 0;
    Position_PID_y.ki = 0;
    Position_PID_y.kd = 0;
    Position_PID_x.kp = 0;
    Position_PID_x.ki = 0;
    Position_PID_x.kd = 0;

    // 初始化内部滤波器
    lpf_now_x.y_prev = 0.0f;
    lpf_now_y.y_prev = 0.0f;
    lpf_d_x.y_prev = 0.0f;
    lpf_d_y.y_prev = 0.0f;
}

/************************ 工具宏定义 ************************/
#define Get_square(x) ((x) * (x)) // 计算平方
#define LIMIT_VALUE_SYMMETRIC(value, max_val) \
    do                                        \
    {                                         \
        if ((value) > (max_val))              \
        {                                     \
            (value) = (max_val);              \
        }                                     \
        else if ((value) < -(max_val))        \
        {                                     \
            (value) = -(max_val);             \
        }                                     \
    } while (0) // 对称限幅宏：限制数值在±max_val之间
#define LIMIT_SYMMETRIC(value) LIMIT_VALUE_SYMMETRIC(value, Integral_MAX) // 积分专用限幅

/**
 * @brief  一阶低通滤波更新（30帧，α=0.454）
 */
float LPF_Update(LPF_t *lpf, float x_raw)
{
    float y_filtered = 0.454f * x_raw + 0.546f * lpf->y_prev;
    lpf->y_prev = y_filtered;
    return y_filtered;
}

/**
 * @brief  单轴位置环PID控制计算
 * @param  Dimension: 控制维度 x/y
 * @param  err: 当前位置偏差（目标值-实际值）
 * @param  dt: 控制周期时间间隔（单位：ms）
 * @retval pid_out: PID计算输出值
 */
int32_t Position_PID_Control(Dimension_t Dimension, float err, uint32_t dt)
{
    float pid_out = 0;
    if (dt == 0)
        dt = 1; // 防止除0错误

    if (Dimension == x) // X轴PID计算
    {
        // 积分分离
        if (fabs(err) < Integration_Separation_Threshold) // 积分分离PID算法
        {
            integral_x += err;                               // 误差积分累加
            LIMIT_VALUE_SYMMETRIC(integral_x, Integral_MAX); // 积分限幅
        }

        // 不完全微分
        float raw_derivative = (err - xerr_last) / dt;
        float filtered_derivative = LPF_Update(&lpf_d_x, raw_derivative);
        // 位置环PID公式：输出 = Kp*误差 + Ki*积分 + Kd*微分
        pid_out = Position_PID_x.kp * err + Position_PID_x.ki * integral_x + Position_PID_x.kd * filtered_derivative;
        // 保存当前误差，用于下一帧微分计算
        xerr_last = err;
    }
    else // Y轴PID计算
    {
        // 积分分离
        if (fabs(err) < Integration_Separation_Threshold) // 积分分离PID算法
        {
            integral_y += err;                               // 误差积分累加
            LIMIT_VALUE_SYMMETRIC(integral_y, Integral_MAX); // 积分限幅
        }

        float raw_derivative = (err - yerr_last) / dt;
        float filtered_derivative = LPF_Update(&lpf_d_y, raw_derivative);
        // 位置环PID公式：输出 = Kp*误差 + Ki*积分 + Kd*微分
        pid_out = Position_PID_y.kp * err + Position_PID_y.ki * integral_y + Position_PID_y.kd * filtered_derivative;
        // 保存当前误差，用于下一帧微分计算
        yerr_last = err;
    }

    LIMIT_VALUE_SYMMETRIC(pid_out, MAX_SPEED); // PID输出限幅
    return (int32_t)(pid_out + 0.5f);
}

/**
 * @brief  主控制函数：视觉位置环+速度前馈总控
 * @param  now_x: 视觉识别的当前激光X坐标
 * @param  now_y: 视觉识别的当前激光Y坐标
 * @retval 无
 * @note   电赛核心控制逻辑：单闭环+前馈，稳定不耦合
 */
void PID_Control(float now_x, float now_y)
{
    int32_t x_out = 0;          // X轴最终电机输出
    int32_t y_out = 0;          // Y轴最终电机输出
    uint32_t time_count = 0;    // 当前时间戳
    uint32_t interval_time = 0; // 控制周期间隔时间

    float filtered_now_x = LPF_Update(&lpf_now_x, now_x);
    float filtered_now_y = LPF_Update(&lpf_now_y, now_y);
    // 获取定时器计时，计算时间间隔dt
    time_count = HAL_GetTick();
    interval_time = time_count - time_count_last;

    // 时间容错：防止异常大值/0值导致计算错误
    if (interval_time > 100)
        interval_time = 100;
    if (interval_time == 0)
        interval_time = 1;
    time_count_last = time_count;

    // 停机标志：停机时清零所有状态，防止重启漂移
    if (!Stop_flag)
    {
        xerr_last = 0;
        yerr_last = 0;
        integral_x = 0;
        integral_y = 0;
        target_x = 0;
        target_y = 0;
        lpf_now_x.y_prev = 0.0f;
        lpf_now_y.y_prev = 0.0f;
        lpf_d_x.y_prev = 0.0f;
        lpf_d_y.y_prev = 0.0f;
        return;
    }

    // ===================== 核心控制流程 =====================
    // 步骤1：更新轨迹目标点
    // Trajectory_Update(interval_time);

    // 步骤2：计算视觉位置误差 = 目标坐标 - 当前视觉坐标
    float err_x = target_x - filtered_now_x;
    float err_y = target_y - filtered_now_y;

    // 步骤3：位置环PID计算误差修正量
    int32_t pid_x = Position_PID_Control(x, err_x, interval_time);
    int32_t pid_y = Position_PID_Control(y, err_y, interval_time);

    // 步骤4：速度前馈计算：开环基础速度，提升跟随性
    int32_t feed_x = Target_Vertical_x * Kf_x;
    int32_t feed_y = Target_Vertical_y * Kf_y;

    // 步骤5：最终输出 = PID修正 + 速度前馈
    x_out = pid_x + feed_x;
    y_out = pid_y + feed_y;

    x_out /= 10;
    y_out /= 10;
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

    if (y_out >= 0)
    {
        Emm_V5_Vel_Control(2, 0, (uint16_t)y_out, 0, 0);
    }
    else
        Emm_V5_Vel_Control(2, 1, (uint16_t)-y_out, 0, 0);
}
