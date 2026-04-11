#include "usart.h"
#include "Serial.h" // 添加Serial头文件
#include "sys.h"

// 全局变量（唯一定义）
__IO bool rx1FrameFlag = false;
__IO bool rx6FrameFlag = false;
__IO uint8_t rxCmd[FIFO_SIZE] = {0};
__IO uint8_t rxCount = 0;

static uint8_t usart1_rx_data = 0;
static uint8_t usart6_rx_data = 0;

// 引用CubeMX定义的串口句柄
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart6;

/**
 * @brief  串口发送单个字节
 */
void usart_SendByte(UART_HandleTypeDef *huart, uint16_t data)
{
    uint8_t tx_data = (uint8_t)(data & (uint16_t)0x01FF);

    HAL_UART_Transmit(huart, &tx_data, 1, 10);
}

/**
 * @brief  串口发送多个字节
 */
void usart_SendCmd(uint8_t *cmd, uint8_t len)
{
    uint8_t i = 0;

    if (cmd[0] == 1)
    {
        for (i = 0; i < len; i++)
        {
            usart_SendByte(&huart1, cmd[i]);
        }
    }
    else if (cmd[0] == 2)
    {
        for (i = 0; i < len; i++)
        {
            usart_SendByte(&huart6, cmd[i]);
        }
    }
}
