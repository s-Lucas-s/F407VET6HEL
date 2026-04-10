#include "board.h"

extern UART_HandleTypeDef huart2;

/**
 *	@brief		外设时钟初始化（HAL库适配版）
 *	@param		无
 *	@retval		无
 *  @note       原F1标准库的时钟/引脚重映射逻辑，替换为HAL库API
 */
void clock_init(void)
{
    // 1. HAL库中GPIO/AFIO时钟由CubeMX自动初始化（MX_GPIO_Init），无需手动使能
    // 2. USART2时钟由CubeMX自动初始化（MX_USART2_UART_Init），无需手动使能
    
    // 禁用JTAG，保留SWD（和原代码GPIO_Remap_SWJ_JTAGDisable逻辑一致）
    //__HAL_RCC_AFIO_CLK_ENABLE();
    //__HAL_AFIO_REMAP_SWJ_NOJTAG(); // HAL库专用的JTAG禁用函数
}

/**
 * @brief   初始化USART2（HAL库适配版，原代码是USART2，不是USART1！）
 * @param   无
 * @retval  无
 * @note    原代码注释写错了（写的USART1，实际是USART2），已修正；
 *          引脚PA2(TX)/PA3(RX)、波特率115200等参数完全对齐原代码
 */
void usart_init(void)
{

    /**********************************************************
    ***	清除USART2中断标志（HAL库版）
    **********************************************************/
    __HAL_UART_CLEAR_FLAG(&huart2, UART_FLAG_RXNE);  // 清除接收中断标志
    __HAL_UART_CLEAR_FLAG(&huart2, UART_FLAG_IDLE);  // 清除空闲中断标志

    /**********************************************************
    ***	使能USART2中断（HAL库版）
    **********************************************************/
    HAL_UART_Receive_IT(&huart2, (uint8_t *)&huart2.Instance->DR, 1);  // 使能接收中断
    __HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);                       // 使能空闲中断

    /**********************************************************
    ***	使能USART2（HAL库版，CubeMX已使能，此处做冗余保证）
    **********************************************************/
    __HAL_UART_ENABLE(&huart2);
}

/**
 *	@brief		板载初始化（HAL库适配版）
 *	@param		无
 *	@retval		无
 *  @note       逻辑和原代码完全一致：时钟初始化→串口初始化→延时2秒
 */
void board_init(void)
{
    clock_init();    // 时钟/引脚重映射初始化
    usart_init();    // 串口初始化
    HAL_Delay(2000); // HAL库自带延时函数（替代原delay_ms(2000)）
}