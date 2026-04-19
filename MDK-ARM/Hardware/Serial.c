#include "Serial.h" // Device header
#include "PID.h"
#include "OLED.h"
#include <stdarg.h>
#include <stdio.h>
#include <stdbool.h>
#include "stm32f4xx_hal.h"
#include <math.h>
#include "Emm_V5.h"

uint8_t rx_data; // 接收数据缓冲区

const uint8_t RESET_KEY = 0xFF; // 定义一个全局变量，用于接收串口命令，控制系统重置

// 外部变量声明
extern float Target_Vertical_x, Target_Vertical_y, target_x, target_y;
extern int8_t Questionx;
extern bool Power_on_flag, Stop_flag;

// 激光固定坐标（和PID保持一致）
#define LASER_FIX_X 160.0f
#define LASER_FIX_Y 120.0f

// 2cm对应的像素误差阈值
#define ERR_X_MAX 5.0f
#define ERR_Y_MAX 5.0f

// 共用体：用于float和4字节数组的互转，方便解析串口收到的浮点数据
typedef union UnionFloat
{
    uint8_t Array[4]; // 字节数组形式，用于逐字节接收
    float FloatNum;   // 浮点数形式，用于直接读取解析后的坐标
} UnionFloat_t;
float center_x, center_y; // 解析后的中心坐标

// ====================== 激光控制核心变量=========================
uint8_t laser_locked = 0; // 激光锁死标志：0=关 1=开（永不关闭）
#define LASER_ON() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET)
#define LASER_OFF() HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET)

// ===================== 扫描模式变量（全部放在Serial内部）=====================
extern uint8_t SCAN;             // 1=顺时针扫描  2=逆时针扫描
uint8_t Scan_Mode_Flag = 1;      // 模式标志位：0=PID跟踪模式  1=扫描寻靶模式（收到数据自动变0）
const uint16_t SCAN_SPEED = 500; // 扫描速度

// 记录最后一次收到有效数据包的时间
uint32_t Last_Valid_Rx_Time = 0;

// 函数指针类型定义：用于串口命令处理函数的跳转表
typedef void (*cmd_handler_USART_t)(void);

#define Get_square(x) ((x) * (x))

void Serial_Scan_Mode(void)
{
    if (SCAN == 1)
    {
        // X轴顺时针旋转(方向1，速度SCAN_SPEED)
        Emm_V5_Vel_Control(1, 1, SCAN_SPEED, 0, 0);
    }
    else if (SCAN == 2)
    {
        // X轴逆时针旋转(方向0，速度SCAN_SPEED)
        Emm_V5_Vel_Control(1, 0, SCAN_SPEED, 0, 0);
    }
    // Y轴电机保持停止（不晃动）
    Emm_V5_Vel_Control(2, 0, 0, 0, 0);
}

// ===================== 系统控制与模式判断轮询任务 =====================
// 独立执行：根据标志位自动选择 扫描 / PID，不依赖串口接收
void Serial_Control_Task(void)
{
    // 如果系统还没启动（没按下按键使得Stop_flag=1），或者还没有给视觉下发考题（Power_on_flag=0）
    if (Stop_flag == 0 || Power_on_flag == 0)
    {
        // 保持两个电机静止，不执行任何业务逻辑
        Emm_V5_Vel_Control(1, 0, 0, 0, 0); 
        Emm_V5_Vel_Control(2, 0, 0, 0, 0); 
        
        // 持续刷新最后接收时间
        // 防止启动瞬间系统由于经过了300ms直接判定为超时，导致一开始就疯狂进入扫描模式乱转
        Last_Valid_Rx_Time = HAL_GetTick(); 
        
        return;
    }

    // 如果超过 300ms 没有收到有效的视觉数据包，自动切换回扫描模式
    // 因为这里是在主循环 while(1) 中不断轮询，所以即使在没接线、串口中断不触发的情况下也能有效判断超时！
    if (HAL_GetTick() - Last_Valid_Rx_Time > 300)
    {
        Scan_Mode_Flag = 1;
    }

    if (Scan_Mode_Flag == 1)
    {
        // 无有效数据包 → 执行扫描
        Serial_Scan_Mode();
    }
    else
    {
        // 收到有效数据包 → 执行PID跟踪
        PID_Control((int32_t)center_x, (int32_t)center_y);
    }
}

// 处理函数1：第1题的串口数据包解析与处理
void handle_USART_BasicQuestion1(void)
{
    u8 com_data;                        // 用于读取STM32串口收到的数据，这个数据会被下一个数据掩盖，所以要将它用一个数组储存起来。
    static u8 RxCounter = 0;            // 共用体数组索引计数器（0-3循环）
    static u8 RxArrayCounter = 0;       // 全局字节计数器（0-9）
    static UnionFloat_t RxBuffer = {0}; // 定义一个6个成员的数组，可以存放6个数据，刚好放下一个数据包。
    static u8 RxState = 0;              // 接收状态，判断程序应该接收第一个帧头、第二个帧头、数据或帧尾。
    com_data = rx_data;

    if (RxState == 0 && com_data == 0xB6)
    {
        RxState = 1;
        RxCounter = 0;
        RxArrayCounter = 0;
    }
    else if (RxState == 1)
    {
        if (RxCounter < 4)
        {
            RxBuffer.Array[RxCounter++] = com_data;
        }

        RxArrayCounter++;
        if (RxArrayCounter == 4)
        {
            RxCounter = 0;
            center_x = RxBuffer.FloatNum;
        }
        else if (RxArrayCounter == 8)
        {
            RxCounter = 0;
            center_y = RxBuffer.FloatNum;
        }
        else if (RxArrayCounter == 9 && com_data == 0x6B)
        {
            // 1. 重置接收状态，准备下一次接收
            RxCounter = 0;
            RxArrayCounter = 0;
            RxState = 0;
            Scan_Mode_Flag = 0;                 // 收到完整数据包，切换到PID跟踪模式
            Last_Valid_Rx_Time = HAL_GetTick(); // 刷新有效数据包时间，打断超时

            // 2. 立刻给视觉发送应答包，通知视觉数据已收到，可以继续发送下一帧了
            uint8_t ack_data = 6; // 任意非0数据，代表ACK
            Serial_SendPacket(0xA5, 0x5A, &ack_data, 1);

            // 【移除此处的电机控制代码】由于外面已经有了 Serial_Motor_Control() 轮询，这里只需要专心接收数据即可，避免重复调用带来不稳定性。

            if (!laser_locked)
            {
                static uint8_t stable_cnt = 0;
                // 计算误差：靶心 → 激光固定点
                float err_x = fabsf(LASER_FIX_X - center_x);
                float err_y = fabsf(LASER_FIX_Y - center_y);

                // 连续10帧稳定(≈150ms) → 开光锁死
                if (err_x < ERR_X_MAX && err_y < ERR_Y_MAX)
                {
                    stable_cnt++;
                    if (stable_cnt >= 10)
                    {
                        laser_locked = 1;
                        LASER_ON(); // 激光打开，永不关闭
                    }
                }
                else
                {
                    stable_cnt = 0;
                }
            }
            return;
        }
        else if (RxArrayCounter >= 9)
        {
            // 帧尾不匹配，或数据超长溢出，强制重置状态防止内存越界！！
            RxState = 0;
            RxCounter = 0;
            RxArrayCounter = 0;
        }
        else
        {
            Target_Vertical_x = 0;
            Target_Vertical_y = 0;
            target_x = center_x;
            target_y = center_y;
        }
    }
    else
    {
        RxState = 0;
        RxCounter = 0;
        RxArrayCounter = 0;
        // 注意：千万不要在这里把 center_x 和 center_y 清零！
        // 否则如果在接收间隙收到杂波，这两项归零会导致 PID 瞬间猛拽云台去跟踪 (0,0)。
    }
}

void handle_USART_BasicQuestion2(void) {}
void handle_USART_BasicQuestion3(void) {}
void handle_USART_BasicQuestion4(void) {}

enum
{
    BasicQuestion1 = 0,
    BasicQuestion2,
    BasicQuestion3,
    BasicQuestion4
};
static cmd_handler_USART_t cmd_Questionx[6] = {
    [BasicQuestion1] = handle_USART_BasicQuestion1,
    [BasicQuestion2] = handle_USART_BasicQuestion2,
    [BasicQuestion3] = handle_USART_BasicQuestion3,
    [BasicQuestion4] = handle_USART_BasicQuestion4,
};

// ===================== 空初始化 =====================
// ===================== 串口发送函数 =====================
void Serial_SendByte(uint8_t Byte)
{
    HAL_UART_Transmit(&huart2, &Byte, 1, 10);
}

void Serial_SendArray(uint8_t *Array, uint16_t Length)
{
    HAL_UART_Transmit(&huart2, Array, Length, 100);
}

void Serial_SendString(char *String)
{
    uint16_t len = 0;
    while (String[len] != '\0')
        len++;
    HAL_UART_Transmit(&huart2, (uint8_t *)String, len, 100);
}

uint32_t Serial_Pow(uint32_t X, uint32_t Y)
{
    uint32_t Result = 1;
    while (Y--)
        Result *= X;
    return Result;
}

void Serial_SendNumber(uint32_t Number, uint8_t Length)
{
    uint8_t i;
    for (i = 0; i < Length; i++)
    {
        Serial_SendByte(Number / Serial_Pow(10, Length - i - 1) % 10 + '0');
    }
}

void Serial_SendPacket(uint8_t packet_header, uint8_t packet_tail, uint8_t *Array, uint16_t Length)
{
    Serial_SendByte(packet_header);
    Serial_SendArray(Array, Length);
    Serial_SendByte(packet_tail);
}

// 提供给外部调用的接收处理函数
void Serial_ProcessRx(uint8_t com_data)
{
    // 【关键修复】：确保全局缓冲 rx_data 与当前传入的 com_data 同步！
    // 否则 handle_USART_BasicQuestion1 内部读取的永远是旧数据！
    rx_data = com_data;

    if (Power_on_flag == 0 && com_data == 0x01)
    {
        if (Stop_flag == 1)
        {
            Power_on_flag = 1;
            Serial_SendPacket(0xA5, 0x5A, (uint8_t *)&Questionx, 1);
        }
        return;
    }

    if (((Questionx - 1) <= (-1)) || Power_on_flag == 0 || Stop_flag == 0)
    {
        return;
    }
    cmd_Questionx[Questionx - 1]();
}
