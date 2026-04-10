#ifndef __SERIAL1_H
#define __SERIAL1_H

#include "main.h"
#include <stdint.h>

// 声明：huart1 定义在 main.c，这里仅引用（解决重复定义）
extern UART_HandleTypeDef huart1;

// 函数声明
void Serial1_SendByte(uint8_t Byte);
void Serial1_SendArray(uint8_t *Array, uint16_t Length);
void Serial1_SendString(char *String);
void VOFA_Send3Ch(float ch1, float ch2, float ch3);

#endif
