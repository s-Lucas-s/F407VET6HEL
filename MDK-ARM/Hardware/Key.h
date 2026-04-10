#ifndef __KEY_H
#define __KEY_H

// 1. 替换F1头文件为F4 HAL库主头文件（自动包含所有GPIO相关定义）
#include "main.h"

// 函数声明：完全保留原代码，无修改（保证外部调用兼容）
unsigned char Key_GetCode(void);
unsigned char Key_Get(void);
void Key_LoopDetect(void);

#endif
