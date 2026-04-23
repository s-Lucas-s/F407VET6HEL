#include "Laser.h"

/**
 * @brief 激光模块初始化
 * @note  GPIO 已由 MX_GPIO_Init 完成配置，这里仅做安全关断
 */
void Laser_Init(void)
{
    Laser_On();
}

/**
 * @brief 打开激光（PB14 输出高电平）
 */
void Laser_On(void)
{
    HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_SET);
}

/**
 * @brief 关闭激光（PB14 输出低电平）
 */
void Laser_Off(void)
{
    HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_RESET);
}

/**
 * @brief 按状态设置激光输出
 * @param state 非0为开，0为关
 */
void Laser_SetState(uint8_t state)
{
    if (state)
    {
        Laser_On();
    }
    else
    {
        Laser_Off();
    }
}
