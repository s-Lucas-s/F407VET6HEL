#include "PID.h"
#include "Emm_V5.h"
#include <math.h>
#include "stm32F4xx_hal.h"
#include "SMS_STS.h"
#include "sys.h"

extern bool Stop_flag;            // 停机标志


/************************ 全局宏定义 ************************/
#define Integral_MAX 500000.0f // PID积分限幅最大值，防止积分饱和失控
#define MAX_SPEED 1800        // 电机最大输出限制，保护电机与机械结构
#define PI 3.1415926f          // 圆周率，用于圆形轨迹计算，备用参数

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
#define LIMIT_SYMMETRIC(value) LIMIT_VALUE_SYMMETRIC(value, Integral_MAX) // 积分专用限幅\

#define LASER_FIX_X 320.0f // 你赛前标定的激光坐标值
#define LASER_FIX_Y 240.0f

/************************ 枚举与结构体定义 ************************/
typedef enum
{
    x, // X轴控制
    y  // Y轴控制
} Dimension_t;

// 定义一个包含所有题目PID参数的数组池
pid_t PID_Pool_X[4]; 
pid_t PID_Pool_Y[4];

extern int8_t Questionx; // 引用外部题号变量

/************************ PID内部静态变量 ************************/
static float xerr_last = 0.0f;
static float yerr_last = 0.0f;
static uint32_t time_count_last = 0;
static float integral_x = 0.0f;
static float integral_y = 0.0f;
float err_x = 0.0f;
float err_y = 0.0f;
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

    // 将 dt 转换为秒
    float dt_s = dt / 1000.0f;
    uint8_t q_idx = Questionx > 0 ? (Questionx - 1) : 0; // 防越界保护

    if (Dimension == x) // PID_X算法
    {
        integral_x += err * dt_s;
        LIMIT_VALUE_SYMMETRIC(integral_x, Integral_MAX);
        pid_out = PID_Pool_X[q_idx].kp * err + PID_Pool_X[q_idx].ki * integral_x + PID_Pool_X[q_idx].kd * (err - xerr_last) / dt_s;
    }
    else // PID_Y算法
    {
        integral_y += err * dt_s;
        LIMIT_VALUE_SYMMETRIC(integral_y, Integral_MAX);
        pid_out = PID_Pool_Y[q_idx].kp * err + PID_Pool_Y[q_idx].ki * integral_y + PID_Pool_Y[q_idx].kd * (err - yerr_last) / dt_s;
    }

    LIMIT_VALUE_SYMMETRIC(pid_out, MAX_SPEED);
    return (int32_t)(pid_out >= 0 ? pid_out + 0.5f : pid_out - 0.5f); // 四舍五入取整
}
void PID_Init(void)
{
    // ====== 第一、二题（静态/基础打靶）：快、准、不超调 ======
    PID_Pool_X[0].kp = 0.0f; PID_Pool_X[0].ki = 0.0f; PID_Pool_X[0].kd = 0.0f;
    PID_Pool_Y[0].kp = 0.01f; PID_Pool_Y[0].ki = 0.0f; PID_Pool_Y[0].kd = 0.0f;
    
    // 第2题同理...
    PID_Pool_X[1] = PID_Pool_X[0]; 
    PID_Pool_Y[1] = PID_Pool_Y[0];
    
    // ====== 第三题（动态画圆跟随）：柔和、加前馈补偿 ======
    PID_Pool_X[2].kp = 0.050f; PID_Pool_X[2].ki = 0.01f; PID_Pool_X[2].kd = 0.050f;
    PID_Pool_Y[2].kp = 0.800f; PID_Pool_Y[2].ki = 0.05f; PID_Pool_Y[2].kd = 0.050f;
    
    // 预留第四题...
    PID_Pool_X[3] = PID_Pool_X[0]; 
    PID_Pool_Y[3] = PID_Pool_Y[0];
}

// === 舵机安全驱动封装函数 ===
// 无论未来是PID调用，还是按键手动调用，统一走这个接口，能100%防止硬件受损
void Set_Servo_Y_Pos(float pos)
{
    if (pos > 1000.0f) pos = 1000.0f;
    if (pos <   0.0f)  pos =   0.0f;
    // 发送绝对位置包：ID=1，采用较高速度避免滞后，加速度0
    WritePosEx(1, (int16_t)pos, 90, 0); 
}

void PID_Control(float in_target_x, float in_target_y)
{
    int32_t x_out;
    int32_t y_out;
    uint32_t time_count = HAL_GetTick();
    uint32_t interval_time = time_count - time_count_last;

    // ===== 极其关键的更新：限制控制帧率 =====
    // 如果主循环跑得太快（间隔在 20ms 以内），直接跳出！
    // 否则串口会被瞬间撑爆，导致舵机因为没时间执行命令而被“卡死”。步进也是同理。
    // if (interval_time < 20)
    // {
    //     return;
    // }

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
        
        Emm_V5_Stop_Now(1, 0); // 停止步进电机
        
        // servo1 是舵机，无需发停止
        return;
    }

    err_x =  in_target_x;
    err_y =  in_target_y;

    // 添加死区限制，防止摄像头像素误差引起的电机高频抖动（根据实际情况可调整此值，当前设定2个像素）
    // if (fabsf(err_x) < 2.0f)
    //     err_x = 0.0f;
    // if (fabsf(err_y) < 2.0f)
    //     err_y = 0.0f;

    int32_t pid_x = Position_PID_Control(x, err_x, interval_time);
    int32_t pid_y = Position_PID_Control(y, err_y, interval_time);

    x_out = pid_x ;
    y_out = pid_y ;
    // 输出限幅，保护电机
    LIMIT_VALUE_SYMMETRIC(x_out, MAX_SPEED);
    // Y轴的 MAX_SPEED 若用于舵机PID可能是扭矩、角度范围或角速度，假设它仍然可用
    LIMIT_VALUE_SYMMETRIC(y_out, MAX_SPEED);

    // 对于2号电机（步进电机，通常控制云台底座X轴，偏航角，接收速度包）：
    // 对于X轴：
    if (x_out >= 0)
    {
        Emm_V5_Vel_Control(1, 1, (uint16_t)x_out, 0, 0); // addr=1, dir=1
    }
    else
    {
        Emm_V5_Vel_Control(1, 0, (uint16_t)(-x_out), 0, 0); // addr=1, dir=0
    }

    // 对于1号电机（SM40BL总线舵机，控制Y轴，直接接收绝对位置指令）：
    // === 转化为增量式位置控制 ===
    // 我们将算出来的y_out直接作为这一次的“增量补偿”，加给记录本里舵机的上一次角度。
    // 这本质上融合了PI位置式闭环的特性，并且让舵机这种执行器能呈现“平滑跟踪”效果。
    static float servo_pos_y = 300.0f; // 记录舵机的绝对位置，默认中心400
    
    // 把y_out当做这一帧的角度增量！完全由 PID 参数接管这部分力度！
    // 只要调大 Kp 就会直接反馈到角度增量的跨度上 
    servo_pos_y += (float)y_out; 
    
    // 调用安全控制层
    Set_Servo_Y_Pos(servo_pos_y);

    xerr_last = err_x;
    yerr_last = err_y;
}


