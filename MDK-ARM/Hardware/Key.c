#include "Key.h"

static unsigned char Key_Code = 0;

/**
 * @brief 外部调用函数：获取非连续键值（松手后返回一次）
 * @return unsigned char 按键键值（0=无按键，1=KEY1，2=KEY2）
 */
unsigned char Key_GetCode(void)
{
    unsigned char TempCode = 0;
    TempCode = Key_Code;
    Key_Code = 0;  // 读取后清零，确保只返回一次
    return TempCode;
}

/**
  * 函数名：Key_Get
  * 功能：读取当前按键的电平状态，返回实时键码
  * 参数：无
  * 返回值：unsigned char - 实时键码（0=无按键，1=KEY1，2=KEY2）
  * 说明：适配F4 HAL库电平读取API
  */
unsigned char Key_Get(void)
{
    unsigned char CurrentKey = 0;  // 默认无按键
    
    // 读取引脚电平：低电平代表按键按下（上拉输入模式）
    // F4用HAL_GPIO_ReadPin替代GPIO_ReadInputDataBit，返回GPIO_PIN_RESET/GPIO_PIN_SET
    if (HAL_GPIO_ReadPin(KEY1_GPIO_Port, KEY1_Pin) == GPIO_PIN_RESET)
    {
        CurrentKey = 1;  // KEY1按下
    }
    if (HAL_GPIO_ReadPin(KEY2_GPIO_Port, KEY2_Pin) == GPIO_PIN_RESET)
    {
        CurrentKey = 2;  // KEY2按下
    }
    if (HAL_GPIO_ReadPin(K1_GPIO_Port, K1_Pin) == GPIO_PIN_RESET)
    {
        CurrentKey = 3;  // K1按下
    }
    
    return CurrentKey;
}

/**
  * 函数名：Key_LoopDetect
  * 功能：循环检测按键状态，实现消抖与松手检测，更新全局键码
  * 参数：无
  * 返回值：无
  * 说明：需在定时器中断中调用（推荐10ms调用一次），保留原逻辑无修改
  */
void Key_LoopDetect(void)
{
    static unsigned char LastState = 0;  // 静态变量：存储上一次按键状态
    unsigned char NowState = 0;          // 当前按键状态
    
    NowState = Key_Get();
    
    // 松手检测：上一状态为按下，当前状态为松开 → 触发键码更新
    if (LastState == 1 && NowState == 0)
    {
        Key_Code = 1;  // KEY1松手
    }
    else if (LastState == 2 && NowState == 0)
    {
        Key_Code = 2;  // KEY2松手
    }
    else if (LastState == 3 && NowState == 0)
    {
        Key_Code = 3;  // K1松手
    }
    
    // 更新上一状态
    LastState = NowState;
}
