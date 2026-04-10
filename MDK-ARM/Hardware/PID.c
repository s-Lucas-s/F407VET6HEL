#include "PID.h"
#include "Emm_V5.h"
#include <math.h>
#include "stm32F4xx_hal.h"

extern uint32_t g_timer3_count;   // 定时器计数变量
extern bool Stop_flag;            // 停机标志

/************************ 全局宏定义 ************************/
#define Integral_MAX 500000.0f // PID积分限幅最大值，防止积分饱和失控
#define MAX_SPEED 16000        // 电机最大输出限制，保护电机与机械结构
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
#define LIMIT_SYMMETRIC(value) LIMIT_VALUE_SYMMETRIC(value, Integral_MAX) // 积分专用限幅
/************************ 透视变换单应性矩阵 ************************/
float H[3][3]; // 3x3透视变换矩阵，用于视觉坐标→实际坐标转换

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

void PID_Control(float now_x, float now_y)
{
    int32_t x_out;
    int32_t y_out;
    uint32_t time_count = g_timer3_count;
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

    float err_x = target_x - now_x;
    float err_y = target_y - now_y;

    int32_t pid_x = Position_PID_Control(x, err_x, interval_time);
    int32_t pid_y = Position_PID_Control(y, err_y, interval_time);

    int32_t feed_x = (int32_t)(Target_Vertical_x * Kf_x);
    int32_t feed_y = (int32_t)(Target_Vertical_y * Kf_y);

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

   //Delay_us(1000);
   HAL_Delay(1);

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

// ====================== 透视变换（视觉标定）核心函数 ======================
/**
 * @brief  计算透视变换单应性矩阵H（Hartley归一化优化，适配F103）
 * @param  src[4]: 视觉采集的4个角点像素坐标
 * @param  dst[4]: 实际物理4个角点坐标
 * @param  H[3][3]: 输出3x3透视变换矩阵
 * @retval 0=成功, -1=失败
 * @note   上电仅标定一次，不占用实时算力
 */
int calcHomography(Point2D src[4], Point2D dst[4], float H[3][3])
{
    static float A[8][9];
    int i, j;
    float cx_src = 0.0f, cy_src = 0.0f, cx_dst = 0.0f, cy_dst = 0.0f;
    float avg_dist_src = 0.0f, avg_dist_dst = 0.0f;
    float s_src, s_dst;
    const float SQRT2 = 1.41421356f;

    for (i = 0; i < 8; i++)
    {
        for (j = 0; j < 9; j++)
        {
            A[i][j] = 0.0f;
        }
    }

    for (i = 0; i < 4; i++)
    {
        cx_src += src[i].x;
        cy_src += src[i].y;
        cx_dst += dst[i].x;
        cy_dst += dst[i].y;
    }
    cx_src *= 0.25f;
    cy_src *= 0.25f;
    cx_dst *= 0.25f;
    cy_dst *= 0.25f;

    for (i = 0; i < 4; i++)
    {
        float x = (src[i].x - cx_src) * s_src;
        float y = (src[i].y - cy_src) * s_src;
        float u = (dst[i].x - cx_dst) * s_dst;
        float v = (dst[i].y - cy_dst) * s_dst;

        A[2 * i][0] = x;
        A[2 * i][1] = y;
        A[2 * i][2] = 1.0f;
        A[2 * i][6] = -u * x;
        A[2 * i][7] = -u * y;
        A[2 * i][8] = -u;

        A[2 * i + 1][3] = x;
        A[2 * i + 1][4] = y;
        A[2 * i + 1][5] = 1.0f;
        A[2 * i + 1][6] = -v * x;
        A[2 * i + 1][7] = -v * y;
        A[2 * i + 1][8] = -v;
    }

    for (int col = 0; col < 8; col++)
    {
        float maxVal = fabsf(A[col][col]);
        int maxRow = col;
        for (i = col + 1; i < 8; i++)
        {
            if (fabsf(A[i][col]) > maxVal)
            {
                maxVal = fabsf(A[i][col]);
                maxRow = i;
            }
        }

        if (maxRow != col)
        {
            float temp;
            for (j = col; j < 9; j++)
            {
                temp = A[col][j];
                A[col][j] = A[maxRow][j];
                A[maxRow][j] = temp;
            }
        }

        float div = A[col][col];
        if (fabsf(div) < 1e-6f)
            return -1;

        float inv_div = 1.0f / div;
        for (j = col; j < 9; j++)
            A[col][j] *= inv_div;

        for (i = 0; i < 8; i++)
        {
            if (i != col && fabsf(A[i][col]) > 1e-6f)
            {
                float factor = A[i][col];
                for (j = col; j < 9; j++)
                    A[i][j] -= factor * A[col][j];
            }
        }
    }

    float h00 = -A[0][8], h01 = -A[1][8], h02 = -A[2][8];
    float h10 = -A[3][8], h11 = -A[4][8], h12 = -A[5][8];
    float h20 = -A[6][8], h21 = -A[7][8];

    float M00 = h00 * s_src;
    float M01 = h01 * s_src;
    float M02 = h02 - M00 * cx_src - M01 * cy_src;

    float M10 = h10 * s_src;
    float M11 = h11 * s_src;
    float M12 = h12 - M10 * cx_src - M11 * cy_src;

    float M20 = h20 * s_src;
    float M21 = h21 * s_src;
    float M22 = 1.0f - M20 * cx_src - M21 * cy_src;

    if (fabsf(M22) < 1e-6f)
        return -1;

    float inv_s_dst = 1.0f / s_dst;
    float inv_H22 = 1.0f / M22;

    H[0][0] = (inv_s_dst * M00 + cx_dst * M20) * inv_H22;
    H[0][1] = (inv_s_dst * M01 + cx_dst * M21) * inv_H22;
    H[0][2] = (inv_s_dst * M02 + cx_dst * M22) * inv_H22;

    H[1][0] = (inv_s_dst * M10 + cy_dst * M20) * inv_H22;
    H[1][1] = (inv_s_dst * M11 + cy_dst * M21) * inv_H22;
    H[1][2] = (inv_s_dst * M12 + cy_dst * M22) * inv_H22;

    H[2][0] = M20 * inv_H22;
    H[2][1] = M21 * inv_H22;
    H[2][2] = 1.0f;

    return 0;
}

/**
 * @brief  视觉像素坐标 → 实际物理坐标转换
 * @param  H[3][3]: 已标定好的透视矩阵
 * @param  pixelPt: 视觉识别的激光像素点
 * @param  screenPt: 输出转换后的实际坐标
 * @retval 无
 */
void visualToReal(float H[3][3], Point2D pixelPt, Point2D *screenPt)
{
    float x = pixelPt.x;
    float y = pixelPt.y;

    float u_prime = H[0][0] * x + H[0][1] * y + H[0][2];
    float v_prime = H[1][0] * x + H[1][1] * y + H[1][2];
    float w = H[2][0] * x + H[2][1] * y + H[2][2];

    if (fabsf(w) < 1e-5f)
    {
        screenPt->x = 0.0f;
        screenPt->y = 0.0f;
        return;
    }

    float inv_w = 1.0f / w;
    screenPt->x = u_prime * inv_w;
    screenPt->y = v_prime * inv_w;
}
