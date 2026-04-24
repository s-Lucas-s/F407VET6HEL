#include "PID.h"
#include "Emm_V5.h"
#include "Serial.h"
#include <math.h>

extern bool Start_flag; // 停机标志

/************************ 全局宏定义 ************************/
#define Integral_MAX 500000.0f // PID积分限幅最大值，防止积分饱和失控
#define MAX_SPEED 16000        // 电机最大输出限制，保护电机与机械结构

#define Integration_Separation_Threshold 100.0f // 积分分离阈值
/************************函数声明************************/

static void handle_PID_BasicQuestion1(float err_x, float err_y);
static void handle_PID_BasicQuestion2(float err_x, float err_y);
static void handle_PID_BasicQuestion3(float err_x, float err_y);
static void handle_PID_AdvancedQuestion1(float err_x, float err_y);
static void handle_PID_AdvancedQuestion2(float err_x, float err_y);
/************************ 枚举与结构体定义 ************************/
typedef void (*cmd_handler_PID_t)(float data_x, float data_y);
enum
{
    BasicQuestion1 = 0,
    BasicQuestion2,
    BasicQuestion3,
    AdvancedQuestion1,
    AdvancedQuestion2
};
static cmd_handler_PID_t cmd_Questionx[5] = {
    [BasicQuestion1] = handle_PID_BasicQuestion1,
    [BasicQuestion2] = handle_PID_BasicQuestion2,
    [BasicQuestion3] = handle_PID_BasicQuestion3,
    [AdvancedQuestion1] = handle_PID_AdvancedQuestion1,
    [AdvancedQuestion2] = handle_PID_AdvancedQuestion2
};


// PID参数结构体：封装位置环PID的比例/积分/微分系数
typedef struct parameter_pid
{
    int32_t kp; // 比例系数：快速减小误差
    int32_t ki; // 积分系数：消除静态误差
    int32_t kd; // 微分系数：抑制震荡、提高响应速度
} pid_t;

/************************ 变量 ************************/

pid_t Position_PID_x;
pid_t Position_PID_y;
float Target_Vertical_x = 0;
float Target_Vertical_y = 0;

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
}

/************************ 工具宏定义 ************************/
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

//积分分离宏：当误差较小时才进行积分，防止大误差时积分过多导致超调和震荡
#define INTEGRAL_SEPARATION(integral_err, err)                 \
    do                                                         \
    {                                                          \
        if (fabs(err) < Integration_Separation_Threshold)      \
        {                                                      \
            integral_err += err;                               \
            LIMIT_VALUE_SYMMETRIC(integral_err, Integral_MAX); \
        }                                                      \
    } while (0)

void PID_SetTargetVertical(Dimension_t xy, float Target_Vertical_xy)
{
    if (xy == x)
    {
        Target_Vertical_x = Target_Vertical_xy;
    }
    else if (xy == y)
    {
        Target_Vertical_y = Target_Vertical_xy;
    }
}
/**
 * @brief  主控制函数：视觉位置环+速度前馈总控
 * @param  now_x: 视觉识别的当前激光X坐标
 * @param  now_y: 视觉识别的当前激光Y坐标
 * @retval 无
 * @note   电赛核心控制逻辑：单闭环+前馈，稳定不耦合
 */
void PID_Control(float err_x, float err_y)
{

    if ((Questionx - 1) <= (-1) || (Questionx - 1) >= 5)
    {
        return;
    }
    cmd_Questionx[Questionx - 1](err_x, err_y);
}

void handle_PID_BasicQuestion1(float err_x, float err_y)
{
    // 第一题不做处理
}

#define LPF_Q2 0.454f // 一阶低通滤波系数，抑制微分噪声
void handle_PID_BasicQuestion2(float err_x, float err_y)
{
    const int32_t Kf_x = 10; // 前馈速度参数
    const int32_t Kf_y = 10; // 前馈速度参数

    static int32_t integral_x = 0;       // X轴积分累加值
    static int32_t integral_y = 0;       // Y轴积分累加值
    static float xerr_last = 0;          // X轴上一周期误差，用于计算微分（D项）
    static float yerr_last = 0;          // Y轴上一周期误差，用于计算微分（D项）
    static float last_d_x = 0;           // X轴上一周期滤波后的微分值（不完全微分用）
    static float last_d_y = 0;           // Y轴上一周期滤波后的微分值（不完全微分用）
    static uint32_t time_count_last = 0; // 上一周期时间戳

    int32_t x_out = 0; // X轴最终输出给电机的控制量
    int32_t y_out = 0; // Y轴最终输出给电机的控制量

    float raw_derivative_x = 0;      // X轴原始微分（未滤波，噪声大）
    float raw_derivative_y = 0;      // Y轴原始微分（未滤波，噪声大）
    float filtered_derivative_x = 0; // X轴滤波后微分（低通滤波，抑制抖动）
    float filtered_derivative_y = 0; // Y轴滤波后微分（低通滤波，抑制抖动）

    uint32_t time_count = 0;    // 当前系统时间戳（ms）
    uint32_t interval_time = 0; // PID控制周期（时间间隔，ms）

    // ===================== 视觉误差滤波（已注释，视觉端已滤波） =====================
    // 已知视觉已经滤波过了，此滤波暂存。
    // float err_x = LPF * err_x + (1.0f - LPF) * last_err_x;
    // float err_y = LPF * err_y + (1.0f - LPF) * last_err_y;
    // last_err_x = err_x;
    // last_err_y = err_y;

    // ===================== 获取PID控制周期 =====================
    // 获取当前定时器时间，计算与上一次PID的时间间隔
    time_count = HAL_GetTick();
    interval_time = time_count - time_count_last;

    // 时间容错：防止异常大值 / 0 导致计算爆炸
    if (interval_time > 100)
        interval_time = 100; // 最大周期限制100ms
    if (interval_time == 0)
        interval_time = 1;        // 避免除以0
    time_count_last = time_count; // 更新上一次时间

    // ===================== 停机状态清零 =====================
    // 停机标志：停机时清零所有PID状态，防止重启漂移、抖动
    if (!Start_flag)
    {
        xerr_last = 0;
        yerr_last = 0;
        integral_x = 0;
        integral_y = 0;
        return;
    }

    // ===================== 积分分离 =====================
    // 误差大时不积分 → 防止超调、震荡
    // 误差小时积分   → 消除静差
    INTEGRAL_SEPARATION(integral_x, err_x);
    INTEGRAL_SEPARATION(integral_y, err_y);

    // ===================== 不完全微分（带低通滤波） =====================
    // 1. 计算原始微分：误差变化率 = (当前误差 - 上次误差) / 时间
    raw_derivative_x = (err_x - xerr_last) / interval_time;
    raw_derivative_y = (err_y - yerr_last) / interval_time;

    // 2. 一阶低通滤波：平滑微分，抑制高频噪声（解决震荡、啸叫）
    filtered_derivative_x = LPF_Q2 * raw_derivative_x + (1.0f - LPF_Q2) * last_d_x;
    filtered_derivative_y = LPF_Q2 * raw_derivative_y + (1.0f - LPF_Q2) * last_d_y;

    // 3. 保存本次滤波后微分，供下周期使用
    last_d_x = filtered_derivative_x;
    last_d_y = filtered_derivative_y;

    // ===================== PID位置环计算 =====================
    int32_t pid_x = Position_PID_x.kp * err_x                    // P项：根据误差快速响应
                    + Position_PID_x.ki * integral_x             // I项：消除稳态误差
                    + Position_PID_x.kd * filtered_derivative_x; // D项：阻尼，抑制震荡（已滤波）

    int32_t pid_y = Position_PID_y.kp * err_y + Position_PID_y.ki * integral_y + Position_PID_y.kd * filtered_derivative_y;

    // ===================== 前馈控制（提高响应速度） =====================
    int32_t feed_x = Target_Vertical_x * Kf_x;
    int32_t feed_y = Target_Vertical_y * Kf_y;

    // ===================== 最终输出 = PID + 前馈 =====================
    x_out = pid_x + feed_x;
    y_out = pid_y + feed_y;

    // 缩小输出量级（适配电机转速范围）
    // x_out /= 10;
    // y_out /= 10;

    // ===================== 限幅：防止输出过大导致飞车/震荡 =====================
    LIMIT_VALUE_SYMMETRIC(x_out, MAX_SPEED);
    LIMIT_VALUE_SYMMETRIC(y_out, MAX_SPEED);

    // ===================== 输出给Emm42 V5闭环驱动 =====================
    // X轴电机控制：根据输出正负判断方向
    if (x_out >= 0)
    {
        Emm_V5_Vel_Control(1, 1, (uint16_t)x_out, 0, 0);
    }
    else
    {
        Emm_V5_Vel_Control(1, 0, (uint16_t)(-x_out), 0, 0);
    }

    // Y轴电机控制：根据输出正负判断方向
    if (y_out >= 0)
    {
        Emm_V5_Vel_Control(2, 0, (uint16_t)y_out, 0, 0);
    }
    else
    {
        Emm_V5_Vel_Control(2, 1, (uint16_t)(-y_out), 0, 0);
    }

    // ===================== 更新上一周期误差（给下一次微分用） =====================
    xerr_last = err_x;
    yerr_last = err_y;
}

void handle_PID_BasicQuestion3(float err_x, float err_y)
{
}

void handle_PID_AdvancedQuestion1(float err_x, float err_y)
{
}

void handle_PID_AdvancedQuestion2(float err_x, float err_y)
{
}