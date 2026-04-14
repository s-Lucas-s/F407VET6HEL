#ifndef __SERIAL_H
#define __SERIAL_H

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

// 类型定义
typedef uint8_t u8;
typedef uint16_t u16;
typedef uint32_t u32;

// 引用CubeMX定义的USART2句柄
extern UART_HandleTypeDef huart2;

// 外部变量
extern const uint8_t RESET_KEY;// 定义一个全局变量，用于接收串口命令，控制系统重置
extern float center_x, center_y; // 外部声明视觉解析的中心坐标


// 函数声明
void Serial_SendByte(uint8_t Byte);
void Serial_SendArray(uint8_t *Array, uint16_t Length);
void Serial_SendString(char *String);
void Serial_SendNumber(uint32_t Number, uint8_t Length);
void Serial_SendPacket(uint8_t packet_header, uint8_t packet_tail, uint8_t *Array, uint16_t Length);
void Serial_ProcessRx(uint8_t com_data);
bool Get_if_searched(void);
bool GetSet_start_search(bool if_set, bool new_value);

#endif
