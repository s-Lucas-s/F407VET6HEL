#include "Serial.h" // Device header
#include "Emm_V5.h"
#include "OLED.h"
#include "PID.h"
#include "laser.h"
#include "stm32f4xx_hal.h"
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>

const uint8_t RESET_KEY = 0xFF; // 定义一个全局变量，用于接收串口命令，控制系统重

// 外部变量声明
extern float Target_Vertical_x, Target_Vertical_y, target_x, target_y;
extern int8_t Questionx;
extern bool Power_on_flag, Start_flag;

// 共用体：用于float和4字节数组的互转，方便解析串口收到的浮点数据
typedef union UnionFloat {
    uint8_t Array[4]; // 字节数组形式，用于逐字节接收
    float FloatNum;   // 浮点数形式，用于直接读取解析后的坐标
} UnionFloat_t;
float coordinate_x, coordinate_y; // 解析后的中心坐标

// 函数指针类型定义：用于串口命令处理函数的跳转表
typedef void (*cmd_handler_USART_t)(uint8_t data);

void handle_USART_BasicQuestion1(uint8_t com_data)
{
    // 第一题不做处理
}

void handle_USART_BasicQuestion2(uint8_t com_data)
{
    static u8 RxCounter = 0;            // 共用体数组索引计数器（0-3循环）
    static u8 RxArrayCounter = 0;       // 全局字节计数器（0-9）
    static UnionFloat_t RxBuffer = {0}; // 定义一个6个成员的数组，可以存放6个数据，刚好放下一个数据包。
    static u8 RxState = 0;              // 接收状态，判断程序应该接收第一个帧头、第二个帧头、数据或帧尾。
    static bool Zeroed = false;         // 搜索开始标志，防止重复启动搜索
    if (!Zeroed)
    {
        if (RxState == 0 && com_data == 0xB6) // 0xB6帧头
        {
            RxState = 1;
        }
        else if (RxState == 1 && com_data == 0x59) // 0x59帧头
        {
            RxState = 2;
        }
        else if (RxState == 2 && com_data == 0x6B)
        {
            Zeroed = true;
            Emm_V5_Stop_Now(1, false);
            Emm_V5_Stop_Now(2, false);
            RxState = 0;
        }
    }
    else
    {
        if (RxState == 0 && com_data == 0xB6) // 0xB6帧头
        {
            RxState = 1;
            RxCounter = 0;
            RxArrayCounter = 0;
        }
        else if (RxState == 1)
        {
            RxBuffer.Array[RxCounter++] = com_data; // 数据填入共用体数组
            RxArrayCounter++;
            if (RxArrayCounter == 4) // 收满第1个float（4字节）
            {
                RxCounter = 0;
                coordinate_x = RxBuffer.FloatNum;
            }
            else if (RxArrayCounter == 8) // 收满第2个float（4字节）
            {
                RxCounter = 0;
                coordinate_y = RxBuffer.FloatNum;
            }
            else if (RxArrayCounter == 9) // 收满帧尾（第10个字节）
            {
                if (com_data == 0x6B) // 校验帧尾0x6B
                {
                    RxCounter = 0;
                    RxArrayCounter = 0;
                    RxState = 0;

                    OLED_ShowFloatNum(0, 32, coordinate_x, 3, 3, OLED_8X16);
                    OLED_ShowFloatNum(0, 48, coordinate_y, 3, 3, OLED_8X16);
                    OLED_Update();

                    // PID_Control(coordinate_x, coordinate_y);
                    return;
                }
                else
                {
                    RxState = 0;
                    RxCounter = 0;
                    RxArrayCounter = 0;
                    coordinate_x = 0;
                    coordinate_y = 0;
                }
            }
        }
        else // 接收异常
        {
            RxState = 0;
            RxCounter = 0;
            coordinate_x = 0;
            coordinate_y = 0;
            RxArrayCounter = 0;
            RxBuffer.FloatNum = 0;
        }
    }
}

void handle_USART_BasicQuestion3(uint8_t com_data)
{
    static u8 RxCounter = 0;            // 共用体数组索引计数器（0-3循环）
    static u8 RxArrayCounter = 0;       // 全局字节计数器（0-9）
    static UnionFloat_t RxBuffer = {0}; // 定义一个6个成员的数组，可以存放6个数据，刚好放下一个数据包。
    static u8 RxState = 0;              // 接收状态，判断程序应该接收第一个帧头、第二个帧头、数据或帧尾。

    if (RxState == 0 && com_data == 0xB6) // 0xB6帧头
    {
        RxState = 1;
        RxCounter = 0;
        RxArrayCounter = 0;
    }
    else if (RxState == 1)
    {
        RxBuffer.Array[RxCounter++] = com_data; // 数据填入共用体数组
        RxArrayCounter++;
        if (RxArrayCounter == 4) // 收满第1个float（4字节）
        {
            RxCounter = 0;
            coordinate_x = RxBuffer.FloatNum;
        }
        else if (RxArrayCounter == 8) // 收满第2个float（4字节）
        {
            RxCounter = 0;
            coordinate_y = RxBuffer.FloatNum;
        }
        else if (RxArrayCounter == 9) // 收满帧尾（第10个字节）
        {
            if (com_data == 0x6B) // 校验帧尾0x6B
            {
                RxCounter = 0;
                RxArrayCounter = 0;
                RxState = 0;

                // OLED_ShowFloatNum(0, 32, coordinate_x, 3, 3, OLED_8X16);
                // OLED_ShowFloatNum(0, 48, coordinate_y, 3, 3, OLED_8X16);
                // OLED_Update();

                PID_Control(coordinate_x, coordinate_y);
                return;
            }
            else
            {
                RxState = 0;
                RxCounter = 0;
                RxArrayCounter = 0;
                coordinate_x = 0;
                coordinate_y = 0;
            }
        }
    }
    else // 接收异常
    {
        RxState = 0;
        RxCounter = 0;
        coordinate_x = 0;
        coordinate_y = 0;
        RxArrayCounter = 0;
        RxBuffer.FloatNum = 0;
    }
}

void handle_USART_AdvancedQuestion1(uint8_t com_data)
{
}

void handle_USART_AdvancedQuestion2(uint8_t com_data)
{
}

enum
{
    BasicQuestion1 = 0,
    BasicQuestion2,
    BasicQuestion3,
    AdvancedQuestion1,
    AdvancedQuestion2
};
static cmd_handler_USART_t cmd_Questionx[6] = {
    [BasicQuestion1] = handle_USART_BasicQuestion1,
    [BasicQuestion2] = handle_USART_BasicQuestion2,
    [BasicQuestion3] = handle_USART_BasicQuestion3,
    [AdvancedQuestion1] = handle_USART_AdvancedQuestion1,
    [AdvancedQuestion2] = handle_USART_AdvancedQuestion2,
};

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

#define search_speed_x 5
#define search_speed_y 0
// 提供给外部调用的接收处理函数
void Serial_ProcessRx(uint8_t com_data)
{
    if (Power_on_flag == 0 && com_data == 0x6B)
    {
        if (Start_flag == 1)
        {
            Power_on_flag = 1;
            if (Questionx == 2)
            {
                Serial_SendPacket(0xA5, 0x5A, (uint8_t *)&Questionx, 1);
                Target_Vertical_x = 0;
                Target_Vertical_y = 0;
                Emm_V5_Vel_Control(2, 0, search_speed_x, 0, 0);
                // Emm_V5_Vel_Control(1, 0, search_speed_y, 0, 0);
            }
            else
            {
                Start_flag = 0;
            }
        }
        return;
    }

    if (((Questionx - 1) <= (-1)) || Power_on_flag == 0 || Start_flag == 0)
    {
        return;
    }
    cmd_Questionx[Questionx - 1](com_data);
}
