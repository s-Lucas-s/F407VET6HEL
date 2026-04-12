#include "uart4.h"
#include "wit_c_sdk.h"
#include <stdio.h>

extern UART_HandleTypeDef huart4;

void Usart4Init(unsigned int uiBaud)
{
    huart4.Instance = UART4;
    huart4.Init.BaudRate = uiBaud;
    huart4.Init.WordLength = UART_WORDLENGTH_8B;
    huart4.Init.StopBits = UART_STOPBITS_1;
    huart4.Init.Parity = UART_PARITY_NONE;
    huart4.Init.Mode = UART_MODE_TX_RX;
    huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart4.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart4) != HAL_OK)
    {
        __disable_irq();
        while (1)
        {
        }
    }
}


void Uart4Send(unsigned char *p_data, unsigned int uiSize)
{
    HAL_UART_Transmit(&huart4, p_data, uiSize, 100);
}

