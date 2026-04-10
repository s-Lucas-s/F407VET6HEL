#include "usart.h"
#include "Serial.h"  // 添加Serial头文件
#include "sys.h"

// 全局变量（唯一定义）
__IO bool rxFrameFlag = false;
__IO uint8_t rxCmd[FIFO_SIZE] = {0};
__IO uint8_t rxCount = 0;

static uint8_t usart1_rx_data = 0;
static uint8_t usart2_rx_data = 0;
static uint8_t usart3_rx_data = 0;

// 引用CubeMX定义的串口句柄
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern UART_HandleTypeDef huart1;

/**
  * @brief  统一UART接收回调（处理USART2 + USART3）
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance == USART2)
  {
    // 电机串口FIFO接收
    fifo_enQueue(usart2_rx_data);
    HAL_UART_Receive_IT(&huart2, &usart2_rx_data, 1);
  }
  else if(huart->Instance == USART3)
  {
    // 视觉串口业务处理
    Serial_ProcessRx(usart3_rx_data);
    HAL_UART_Receive_IT(&huart3, &usart3_rx_data, 1);
  }
  else if(huart->Instance == USART1)
  {
    // USART1 bootloader处理
    check_Bootloader(usart1_rx_data);
    HAL_UART_Receive_IT(&huart1, &usart1_rx_data, 1);
  }
}

/**
  * @brief  空闲中断回调
  */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
{
  uint8_t i = 0;
  uint8_t rx_data;
  
  if(huart->Instance == USART2)
  {
    rxCount = fifo_queueLength();
    for(i=0; i<rxCount; i++)
    {
      rxCmd[i] = fifo_deQueue();
    }
    rxFrameFlag = true;
    
    __HAL_UART_CLEAR_IDLEFLAG(huart);
    HAL_UARTEx_ReceiveToIdle_IT(&huart2, (uint8_t*)rxCmd, FIFO_SIZE);
    HAL_UART_Receive_IT(&huart2, &rx_data, 1);
  }
}

/**
  * @brief  串口发送单个字节
  */
void usart_SendByte(uint16_t data)
{
  uint16_t t0 = 0;
  uint8_t tx_data = (uint8_t)(data & (uint16_t)0x01FF);
  
  HAL_UART_Transmit(&huart2, &tx_data, 1, 10);
  
  // 模拟原超时逻辑（HAL已内置超时，此处保留兼容）
  while(HAL_UART_GetState(&huart2) == HAL_UART_STATE_BUSY_TX)
  {
    ++t0; 
    if(t0 > 8000) { return; }
  }
}

/**
  * @brief  串口发送多个字节
  */
void usart_SendCmd(uint8_t *cmd, uint8_t len)
{
  uint8_t i = 0;
  
  for(i = 0; i < len; i++) 
  { 
    usart_SendByte(cmd[i]); 
  }
}

// 初始化函数
void usart_Init(void)
{
  // 开启USART2接收
  HAL_UARTEx_ReceiveToIdle_IT(&huart2, (uint8_t*)rxCmd, FIFO_SIZE);
  HAL_UART_Receive_IT(&huart2, &usart2_rx_data, 1);

  // 开启USART3接收（关键！）
  HAL_UART_Receive_IT(&huart3, &usart3_rx_data, 1);

  // 开启USART1 bootloader接收
  HAL_UART_Receive_IT(&huart1, &usart1_rx_data, 1);
}
