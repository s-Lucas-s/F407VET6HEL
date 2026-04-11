#ifndef __FIFO_H
#define __FIFO_H

#include "main.h"
#include <stdbool.h>

#define FIFO_SIZE   128

typedef struct {
    uint16_t buffer[FIFO_SIZE];
    __IO uint8_t ptrWrite;
    __IO uint8_t ptrRead;
} FIFO_t;

extern __IO FIFO_t rx1FIFO;
extern __IO FIFO_t rx6FIFO;

// 函数名统一规范
void fifo_initQueue(void);  // 初始化队列
void fifo_enQueue(UART_HandleTypeDef *huart,uint16_t data);
uint16_t fifo_deQueue(UART_HandleTypeDef *huart);
bool fifo_isEmpty(UART_HandleTypeDef *huart);
uint16_t fifo_queueLength(UART_HandleTypeDef *huart);

#endif
