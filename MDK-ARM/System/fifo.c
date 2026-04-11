#include "fifo.h"

__IO FIFO_t rx1FIFO = {0};
__IO FIFO_t rx6FIFO = {0};

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart6;

/**
 * @brief   初始化队列
 * @param   无
 * @retval  无
 */
void fifo_initQueue(void) // 函数名与头文件统一
{
    rx1FIFO.ptrRead = 0;
    rx1FIFO.ptrWrite = 0;
    rx6FIFO.ptrRead = 0;
    rx6FIFO.ptrWrite = 0;
}

/**
 * @brief   入队
 * @param   无
 * @retval  无
 */
void fifo_enQueue(UART_HandleTypeDef *huart, uint16_t data)
{
    if (huart == &huart1)
    {
        rx1FIFO.buffer[rx1FIFO.ptrWrite] = data;
        ++rx1FIFO.ptrWrite;
    }
    else if (huart == &huart6)
    {
        rx6FIFO.buffer[rx6FIFO.ptrWrite] = data;
        ++rx6FIFO.ptrWrite;
    }
}

/**
 * @brief   出队
 * @param   无
 * @retval  无
 */
uint16_t fifo_deQueue(UART_HandleTypeDef *huart)
{
    uint16_t element = 0;

    if (huart == &huart1)
    {
        element = rx1FIFO.buffer[rx1FIFO.ptrRead];
        ++rx1FIFO.ptrRead;
        if (rx1FIFO.ptrRead >= FIFO_SIZE)
        {
            rx1FIFO.ptrRead = 0;
        }
    }
    else if (huart == &huart6)
    {
        element = rx6FIFO.buffer[rx6FIFO.ptrRead];
        ++rx6FIFO.ptrRead;
        if (rx6FIFO.ptrRead >= FIFO_SIZE)
        {
            rx6FIFO.ptrRead = 0;
        }
    }

    return element;
}

/**
 * @brief   判断空队列
 * @param   无
 * @retval  无
 */
bool fifo_isEmpty(UART_HandleTypeDef *huart)
{
    if (huart == &huart1)
    {
        if (rx1FIFO.ptrRead == rx1FIFO.ptrWrite)
        {
            return true;
        }
    }
    else if (huart == &huart6)
    {
        if (rx6FIFO.ptrRead == rx6FIFO.ptrWrite)
        {
            return true;
        }
    }
    return false;
}

/**
 * @brief   计算队列长度
 * @param   无
 * @retval  无
 */
uint16_t fifo_queueLength(UART_HandleTypeDef *huart)
{
    if (huart == &huart1)
    {
        if (rx1FIFO.ptrRead <= rx1FIFO.ptrWrite)
        {
            return (rx1FIFO.ptrWrite - rx1FIFO.ptrRead);
        }
        else
        {
            return (FIFO_SIZE - rx1FIFO.ptrRead + rx1FIFO.ptrWrite);
        }
    }
    else if (huart == &huart6)
    {
        if (rx6FIFO.ptrRead <= rx6FIFO.ptrWrite)
        {
            return (rx6FIFO.ptrWrite - rx6FIFO.ptrRead);
        }
        else
        {
            return (FIFO_SIZE - rx6FIFO.ptrRead + rx6FIFO.ptrWrite);
        }
    }
    return 0;

}
