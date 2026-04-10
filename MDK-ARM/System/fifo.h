#ifndef __FIFO_H
#define __FIFO_H

#include "main.h"    // 替换 stm32f10x.h，适配 F4 HAL 库
#include <stdbool.h>

#define FIFO_SIZE   128

typedef struct {
    uint16_t buffer[FIFO_SIZE];
    __IO uint8_t ptrWrite;
    __IO uint8_t ptrRead;
} FIFO_t;

extern __IO FIFO_t rxFIFO;

// 函数名统一规范
void fifo_initQueue(void);
void fifo_enQueue(uint16_t data);
uint16_t fifo_deQueue(void);
bool fifo_isEmpty(void);
uint16_t fifo_queueLength(void);

#endif
