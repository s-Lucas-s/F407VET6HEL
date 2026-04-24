#include "Serial.h"        // 串口设备驱动头文件
#include "Emm_V5.h"        //  Emm-V5系列电机驱动头文件
#include "OLED.h"          // OLED显示屏驱动头文件
#include "PID.h"           // PID控制算法头文件
#include "laser.h"         // 激光相关驱动头文件
#include "stm32f4xx_hal.h" // STM32F4 HAL库核心头文件
#include <stdarg.h>        // 可变参数支持头文件
#include <stdbool.h>       // C语言布尔类型头文件
#include <stdio.h>         // 标准输入输出头文件


// 外部变量声明（在其他文件中定义，此处引用）
extern int8_t Questionx;                           // 当前题目编号（控制不同串口处理逻辑）
extern bool Power_on_flag;                         // 系统上电启动标志位
extern bool Start_flag;                            // 系统开始运行标志位

// 共用体：用于float和4字节数组的互转，方便解析串口收到的浮点数据
typedef union UnionFloat {
    uint8_t Array[4]; // 字节数组形式，用于串口逐字节接收数据
    float FloatNum;   // 浮点数形式，用于直接读取解析后的坐标数值
} UnionFloat_t;

float coordinate_x, coordinate_y; // 解析完成后的目标中心坐标（X/Y）

// 函数指针类型定义：用于串口命令处理函数的统一调用（跳转表）
typedef void (*cmd_handler_USART_t)(uint8_t data);

/**
 * @brief  第一题串口数据处理函数（无实际处理逻辑）
 * @param  com_data: 串口接收到的单字节数据
 * @retval 无
 */
void handle_USART_BasicQuestion1(uint8_t com_data)
{
    // 第一题不做处理
}

/**
 * @brief  第二题串口数据处理函数（带启动校验+双浮点数解析）
 * @param  com_data: 串口接收到的单字节数据
 * @retval 无
 * @note   协议：0xB6+0x59（启动命令：‘Y’）+0x6B → 正式数据帧：0xB6 + 4字节Xfloat + 4字节Yfloat + 0x6B帧尾
 */
void handle_USART_BasicQuestion2(uint8_t com_data)
{
    static u8 RxCounter = 0;            // 共用体数组索引计数器（0-3，对应float4字节）
    static u8 RxArrayCounter = 0;       // 全局字节计数器（0-9，一帧共10字节）
    static UnionFloat_t RxBuffer = {0}; // 浮点数接收共用体缓存
    static u8 RxState = 0;              // 接收状态机：0等待帧头，1接收数据
    static bool Zeroed = false;         // 搜索到目标标志位，未找到前为false，由软件通知

    // ============== 第一步：等待启动指令（0xB6+0x59+0x6B） ==============
    if (!Zeroed)
    {
        if (RxState == 0 && com_data == 0xB6) // 检测第一个帧头0xB6
        {
            RxState = 1;
        }
        else if (RxState == 1 && com_data == 0x59) // 检测标志位0x59
        {
            RxState = 2;
        }
        else if (RxState == 2 && com_data == 0x6B) // 检测启动命令0x6B
        {
            Zeroed = true;             // 标记启动完成
            Emm_V5_Stop_Now(1, false); // 电机1立即停止
            Emm_V5_Stop_Now(2, false); // 电机2立即停止
            RxState = 0;               // 重置状态机
        }
    }
    // ============== 第二步：启动完成，开始解析坐标数据 ==============
    else
    {
        if (RxState == 0 && com_data == 0xB6) // 检测数据帧头0xB6
        {
            RxState = 1;
            RxCounter = 0;
            RxArrayCounter = 0;
        }
        else if (RxState == 1) // 状态1：连续接收数据字节
        {
            RxBuffer.Array[RxCounter++] = com_data; // 存入共用体，自动转float
            RxArrayCounter++;

            if (RxArrayCounter == 4) // 收满4字节 → 解析出X坐标
            {
                RxCounter = 0;
                coordinate_x = RxBuffer.FloatNum;
            }
            else if (RxArrayCounter == 8) // 收满8字节 → 解析出Y坐标
            {
                RxCounter = 0;
                coordinate_y = RxBuffer.FloatNum;
            }
            else if (RxArrayCounter == 9) // 收满9字节（第10字节）→ 校验帧尾
            {
                if (com_data == 0x6B) // 帧尾正确0x6B
                {
                    // 重置接收状态
                    RxCounter = 0;
                    RxArrayCounter = 0;
                    RxState = 0;

                    //OLED为调试代码，显示解析后的坐标，正式使用时可注释掉
                    // OLED显示解析后的坐标
                    // OLED_ShowFloatNum(0, 32, coordinate_x, 3, 3, OLED_8X16);
                    // OLED_ShowFloatNum(0, 48, coordinate_y, 3, 3, OLED_8X16);
                    // OLED_Update();

                    //调整合适参数后再打开，防止过冲
                    // PID_Control(coordinate_x, coordinate_y); // PID控制（注释备用）
                    return;
                }
                else // 帧尾错误，重置接收
                {
                    RxState = 0;
                    RxCounter = 0;
                    RxArrayCounter = 0;
                    coordinate_x = 0;
                    coordinate_y = 0;
                }
            }
        }
        else // 接收异常，全部重置
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

/**
 * @brief  第三题串口数据处理函数（直接解析双浮点数坐标）
 * @param  com_data: 串口接收到的单字节数据
 * @retval 无
 * @note   协议：0xB6 + 4字节Xfloat + 4字节Yfloat + 0x6B帧尾，共10字节
 */
void handle_USART_BasicQuestion3(uint8_t com_data)
{
    //注意：还没写具体内容，以下代码仅是框架
    static u8 RxCounter = 0;            // 共用体数组索引计数器（0-3）
    static u8 RxArrayCounter = 0;       // 全局字节计数器（0-9）
    static UnionFloat_t RxBuffer = {0}; // 浮点数接收缓存
    static u8 RxState = 0;              // 接收状态机

    if (RxState == 0 && com_data == 0xB6) // 检测帧头0xB6
    {
        RxState = 1;
        RxCounter = 0;
        RxArrayCounter = 0;
    }
    else if (RxState == 1) // 接收数据
    {
        RxBuffer.Array[RxCounter++] = com_data;
        RxArrayCounter++;

        if (RxArrayCounter == 4) // 第1个float：X坐标
        {
            RxCounter = 0;
            coordinate_x = RxBuffer.FloatNum;
        }
        else if (RxArrayCounter == 8) // 第2个float：Y坐标
        {
            RxCounter = 0;
            coordinate_y = RxBuffer.FloatNum;
        }
        else if (RxArrayCounter == 9) // 帧尾校验
        {
            if (com_data == 0x6B) // 帧尾正确
            {
                // 重置接收
                RxCounter = 0;
                RxArrayCounter = 0;
                RxState = 0;

                // 执行PID闭环控制
                PID_Control(coordinate_x, coordinate_y);
                return;
            }
            else // 帧尾错误
            {
                RxState = 0;
                RxCounter = 0;
                RxArrayCounter = 0;
                coordinate_x = 0;
                coordinate_y = 0;
            }
        }
    }
    else // 异常重置
    {
        RxState = 0;
        RxCounter = 0;
        coordinate_x = 0;
        coordinate_y = 0;
        RxArrayCounter = 0;
        RxBuffer.FloatNum = 0;
    }
}

/**
 * @brief  进阶题1串口处理函数（预留）
 */
void handle_USART_AdvancedQuestion1(uint8_t com_data)
{
}

/**
 * @brief  进阶题2串口处理函数（预留）
 */
void handle_USART_AdvancedQuestion2(uint8_t com_data)
{
}

// 题目编号枚举，方便函数指针表索引
enum
{
    BasicQuestion1 = 0, // 基础题1
    BasicQuestion2,     // 基础题2
    BasicQuestion3,     // 基础题3
    AdvancedQuestion1,  // 进阶题1
    AdvancedQuestion2   // 进阶题2
};

// 串口命令处理函数跳转表：根据题目编号自动调用对应处理函数
static cmd_handler_USART_t cmd_Questionx[5] = {
    [BasicQuestion1] = handle_USART_BasicQuestion1,
    [BasicQuestion2] = handle_USART_BasicQuestion2,
    [BasicQuestion3] = handle_USART_BasicQuestion3,
    [AdvancedQuestion1] = handle_USART_AdvancedQuestion1,
    [AdvancedQuestion2] = handle_USART_AdvancedQuestion2,
};

// ===================== 串口数据发送函数库 =====================

/**
 * @brief  串口发送单字节
 * @param  Byte: 待发送字节
 * @retval 无
 */
void Serial_SendByte(uint8_t Byte)
{
    HAL_UART_Transmit(&huart2, &Byte, 1, 10);
}

/**
 * @brief  串口发送字节数组
 * @param  Array: 数据数组指针
 * @param  Length: 数据长度
 * @retval 无
 */
void Serial_SendArray(uint8_t *Array, uint16_t Length)
{
    HAL_UART_Transmit(&huart2, Array, Length, 100);
}

/**
 * @brief  串口发送字符串
 * @param  String: 字符串指针
 * @retval 无
 */
void Serial_SendString(char *String)
{
    uint16_t len = 0;
    while (String[len] != '\0') // 计算字符串长度
        len++;
    HAL_UART_Transmit(&huart2, (uint8_t *)String, len, 100);
}

/**
 * @brief  幂次计算函数（供数字发送使用）
 * @param  X: 底数 Y: 指数
 * @retval 计算结果
 */
uint32_t Serial_Pow(uint32_t X, uint32_t Y)
{
    uint32_t Result = 1;
    while (Y--)
        Result *= X;
    return Result;
}

/**
 * @brief  串口发送指定长度的数字（十进制）
 * @param  Number: 数字
 * @param  Length: 长度
 * @retval 无
 */
void Serial_SendNumber(uint32_t Number, uint8_t Length)
{
    uint8_t i;
    for (i = 0; i < Length; i++)
    {
        Serial_SendByte(Number / Serial_Pow(10, Length - i - 1) % 10 + '0');
    }
}

/**
 * @brief  串口发送完整数据包（帧头+数据+帧尾）
 * @param  packet_header: 帧头
 * @param  packet_tail: 帧尾
 * @param  Array: 数据区
 * @param  Length: 数据长度
 * @retval 无
 */
void Serial_SendPacket(uint8_t packet_header, uint8_t packet_tail, uint8_t *Array, uint16_t Length)
{
    Serial_SendByte(packet_header);
    Serial_SendArray(Array, Length);
    Serial_SendByte(packet_tail);
}

// 第二题双轴轴搜索速度参数
#define search_speed_x 5
#define search_speed_y 0

/**
 * @brief  串口接收数据总处理函数（外部调用入口）
 * @param  com_data: 串口接收到的单字节数据
 * @retval 无
 * @note   实现系统启动、题目模式切换、数据分发功能
 */
void Serial_ProcessRx(uint8_t com_data)
{
    // 系统未启动时，收到0x6B则启动系统
    if (Power_on_flag == 0 && com_data == 0x6B)
    {
        if (Start_flag == 1)
        {
            Power_on_flag = 1; // 标记系统上电运行

            // 各个题目特殊处理
            if (Questionx == 2)
            {
                Serial_SendPacket(0xA5, 0x5A, (uint8_t *)&Questionx, 1);
                Emm_V5_Vel_Control(2, 0, search_speed_x, 0, 0); // X轴电机启动搜索
                //Emm_V5_Vel_Control(2, 0, search_speed_y, 0, 0); // Y轴电机启动搜索
            }
            else// 其他题目默认不启动电机，等待串口数据解析坐标后PID控制，新增代码可能存在逻辑bug
            {
                Start_flag = 0;
            }
        }
        return;
    }

    // 系统未启动/题目无效，直接返回
    if (((Questionx - 1) <= (-1)) || Power_on_flag == 0 || Start_flag == 0)
    {
        return;
    }

    // 根据当前题目编号，调用对应串口处理函数
    cmd_Questionx[Questionx - 1](com_data);
}
