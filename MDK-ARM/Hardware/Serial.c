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
extern float H[3][3];
extern int8_t Questionx;
extern bool Power_on_flag, Stop_flag;

// 激光固定坐标（和PID保持一致）
#define LASER_FIX_X    160.0f
#define LASER_FIX_Y    120.0f

// 2cm对应的像素误差阈值
#define ERR_X_MAX  5.0f
#define ERR_Y_MAX  5.0f

// 共用体：用于float和4字节数组的互转，方便解析串口收到的浮点数据
typedef union UnionFloat {
    uint8_t Array[4]; // 字节数组形式，用于逐字节接收
    float FloatNum;   // 浮点数形式，用于直接读取解析后的坐标
} UnionFloat_t;
float center_x, center_y; // 解析后的中心坐标

// ====================== 激光控制核心变量=========================
uint8_t laser_locked = 0;   // 激光锁死标志：0=关 1=开（永不关闭）
#define LASER_ON()     HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET)  
#define LASER_OFF()    HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET)

// ===================== 扫描模式变量（全部放在Serial内部）=====================
uint8_t SCAN = 1;          // 1=顺时针扫描  2=逆时针扫描
uint8_t Scan_Mode_Flag = 0; // 模式标志位：0=PID跟踪模式  1=扫描寻靶模式（收到数据自动变0）
const uint16_t SCAN_SPEED = 500;  // 扫描速度

// 函数指针类型定义：用于串口命令处理函数的跳转表
typedef void (*cmd_handler_USART_t)(void);

#define Get_square(x) ((x) * (x))

void Serial_Scan_Mode(void)
{
    if(SCAN == 1)
    {
        // X轴顺时针旋转(方向1，速度SCAN_SPEED)
        Emm_V5_Vel_Control(1, 1, SCAN_SPEED, 0, 0);
    }
    else if(SCAN == 2)
    {
        // X轴逆时针旋转(方向0，速度SCAN_SPEED)
        Emm_V5_Vel_Control(1, 0, SCAN_SPEED, 0, 0);
    }
    // Y轴电机保持停止（不晃动）
    Emm_V5_Vel_Control(2, 0, 0, 0, 0);
}

// 处理函数1：第二题的串口数据包解析与处理
void handle_USART_BasicQuestion1(void)
{
    u8 com_data;                        // 用于读取STM32串口收到的数据，这个数据会被下一个数据掩盖，所以要将它用一个数组储存起来。
    static u8 RxCounter = 0;            // 共用体数组索引计数器（0-3循环）
    static u8 RxArrayCounter = 0;       // 全局字节计数器（0-9）
    static UnionFloat_t RxBuffer = {0}; // 定义一个6个成员的数组，可以存放6个数据，刚好放下一个数据包。
    static u8 RxState = 0;              // 接收状态，判断程序应该接收第一个帧头、第二个帧头、数据或帧尾。
    // static bool okk = 0;              // 接收状态，判断程序应该接收第一个帧头、第二个帧头、数据或帧尾。
    com_data = rx_data;

    if (RxState == 0 && com_data == 0xB6)
    {
        RxState = 1;
        RxCounter = 0;
        RxArrayCounter = 0;
    }
    else if (RxState == 1)
    {
        RxBuffer.Array[RxCounter++] = com_data;
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
            Scan_Mode_Flag = 0; // 收到数据包，切换到PID跟踪模式

             // 【屏蔽未定义的视觉/PID函数】

            //2、立刻给视觉发送应答包，通知视觉数据已收到，可以继续发送下一帧了
            uint8_t ack_data = 1;
            Serial_SendPacket(0xA5, 0x5A, &ack_data, 1);
            
            //3、标志位判断：扫描 / PID
            if(Scan_Mode_Flag == 1)
            {
                Serial_Scan_Mode();
            }
            else
            {
                PID_Control((int32_t)(center_x), (int32_t)(center_y));
            }

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
        RxState = 0; RxCounter = 0; RxArrayCounter = 0;
        center_x = 0; center_y = 0;
    }
}

void handle_USART_BasicQuestion2(void)
{
    uint8_t com_data;
    static bool data_packet_count = 0;
    static uint8_t RxCounter = 0;
    static uint8_t RxArrayCounter = 0;
    static UnionFloat_t RxBuffer = {0};
    static uint8_t RxState = 0;
    static uint8_t Position = 0;
    static Point2D src[4];
    static Point2D dst[4] = {{0.0f, 0.0f},{100.0f, 0.0f},{100.0f, 100.0f},{0.0f, 100.0f}};
    static const Point2D rect_points[4] = {{0.0f, 0.0f},{100.0f, 0.0f},{100.0f, 100.0f},{0.0f, 100.0f}};

    com_data = rx_data;

    if (RxState == 0 &&  (com_data == 0xB6 && data_packet_count == 1))
    {
        RxState = 1;
        RxCounter = 0;
        RxArrayCounter = 0;
        RxBuffer.FloatNum = 0.0f;
    }
    else if (RxState == 1)
    {
        RxBuffer.Array[RxCounter++] = com_data;
        RxArrayCounter++;
        if (data_packet_count == 0)
        {
            if (RxArrayCounter < 33)
            {
                switch (RxArrayCounter)
                {
                    case 4:  RxCounter=0; src[0].x=RxBuffer.FloatNum; break;
                    case 8:  RxCounter=0; src[0].y=RxBuffer.FloatNum; break;
                    case 12: RxCounter=0; src[1].x=RxBuffer.FloatNum; break;
                    case 16: RxCounter=0; src[1].y=RxBuffer.FloatNum; break;
                    case 20: RxCounter=0; src[2].x=RxBuffer.FloatNum; break;
                    case 24: RxCounter=0; src[2].y=RxBuffer.FloatNum; break;
                    case 28: RxCounter=0; src[3].x=RxBuffer.FloatNum; break;
                    case 32: RxCounter=0; src[3].y=RxBuffer.FloatNum; break;
                    default: break;
                }
            }
            else if (RxArrayCounter == 33)
            {
                if (com_data == 0x5A)
                {
                    RxCounter=0; RxArrayCounter=0; RxState=0;
                    // 【屏蔽未定义的视觉函数】
                    // if (!(calcHomography(src, dst, H)))
                    // {
                    //     Serial_SendByte(0x01);
                    //     data_packet_count = 1;
                    // }
                }
                else
                {
                    RxState=0; RxCounter=0; RxArrayCounter=0;
                    data_packet_count=0; center_x=0; center_y=0;
                }
            }
        }
        else
        {
            if (RxArrayCounter == 4)  { RxCounter=0; center_x=RxBuffer.FloatNum; }
            else if (RxArrayCounter == 8) { RxCounter=0; center_y=RxBuffer.FloatNum; }
            else if (RxArrayCounter == 9)
            {
                if (com_data == 0x6B)
                {
                    // 【屏蔽未定义的视觉/PID函数】
                    // Point2D pixel_pt = {center_x, center_y};
                    // Point2D real_pt;
                    // visualToReal(H, pixel_pt, &real_pt);
                    // float err_x = target_x - real_pt.x;
                    // float err_y = target_y - real_pt.y;
                    // float dist_sq = Get_square(err_x) + Get_square(err_y);
                    // if (dist_sq < 4.0f)
                    // {
                    //     Position = (Position + 1) % 4;
                    //     target_x = rect_points[Position].x;
                    //     target_y = rect_points[Position].y;
                    // }
                    // PID_Control(real_pt.x, real_pt.y);

                    RxState=0; RxCounter=0; RxArrayCounter=0; data_packet_count=0;
                }
                else { RxState=0; RxCounter=0; RxArrayCounter=0; data_packet_count=0; }
            }
        }
    }
    else
    {
        RxState=0; RxCounter=0; center_x=0; center_y=0;
        RxArrayCounter=0; RxBuffer.FloatNum=0;
    }
}

void handle_USART_BasicQuestion3(void){}
void handle_USART_BasicQuestion4(void){}

enum { BasicQuestion1=0, BasicQuestion2, BasicQuestion3, BasicQuestion4 };
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
    HAL_UART_Transmit(&huart3, &Byte, 1, 10);
}

void Serial_SendArray(uint8_t *Array, uint16_t Length)
{
    HAL_UART_Transmit(&huart3, Array, Length, 100);
}

void Serial_SendString(char *String)
{
    uint16_t len = 0;
    while(String[len] != '\0') len++;
    HAL_UART_Transmit(&huart3, (uint8_t*)String, len, 100);
}

uint32_t Serial_Pow(uint32_t X, uint32_t Y)
{
    uint32_t Result = 1;
    while(Y--) Result *= X;
    return Result;
}

void Serial_SendNumber(uint32_t Number, uint8_t Length)
{
    uint8_t i;
    for(i=0; i<Length; i++)
    {
        Serial_SendByte(Number / Serial_Pow(10, Length-i-1) % 10 + '0');
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
