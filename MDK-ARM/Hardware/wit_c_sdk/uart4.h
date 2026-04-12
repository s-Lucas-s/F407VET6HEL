#ifndef __UART4_H
#define __UART4_H
#include "stm32F4xx_hal.h"
#include "stdint.h"
void Usart4Init(unsigned int uiBaud);
void Uart4Send(unsigned char *p_data, unsigned int uiSize);

#endif
