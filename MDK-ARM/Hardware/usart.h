#ifndef __USART_H
#define __USART_H

#include "main.h"
#include <stdbool.h>
#include "fifo.h"

// 全局变量声明（完全保留）
extern __IO bool rxFrameFlag;
extern __IO uint8_t rxCmd[FIFO_SIZE];
extern __IO uint8_t rxCount;

// 函数声明（接口完全不变）
void usart_SendCmd(uint8_t *cmd, uint8_t len);
void usart_SendByte(uint16_t data);

#endif
