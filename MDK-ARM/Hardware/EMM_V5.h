#ifndef __EMM_V5_H
#define __EMM_V5_H

// 1. 替换标准库头文件为HAL库头文件
#include <stdint.h>   // 替代stdbool.h，HAL库标准数据类型
#include <stdbool.h>  // 保留bool类型，兼容原代码

/**********************************************************
***	Emm_V5.0步进闭环控制例程
***	适配说明：兼容STM32F407 HAL库，替换USART为HAL_UART API
**********************************************************/

// 系统参数枚举（完全保留原代码，无修改）
typedef enum {
    S_VER   = 0,  /* 读取固件版本和对应的硬件版本 */
    S_RL    = 1,  /* 读取读取相电阻和相电感 */
    S_PID   = 2,  /* 读取PID参数 */
    S_VBUS  = 3,  /* 读取总线电压 */
    S_CPHA  = 5,  /* 读取相电流 */
    S_ENCL  = 7,  /* 读取经过线性化校准后的编码器值 */
    S_TPOS  = 8,  /* 读取电机目标位置角度 */
    S_VEL   = 9,  /* 读取电机实时转速 */
    S_CPOS  = 10, /* 读取电机实时位置角度 */
    S_PERR  = 11, /* 读取电机位置误差角度 */
    S_FLAG  = 13, /* 读取使能/到位/堵转状态标志位 */
    S_Conf  = 14, /* 读取驱动参数 */
    S_State = 15, /* 读取系统状态参数 */
    S_ORG   = 16, /* 读取正在回零/回零失败状态标志位 */
} SysParams_t;

// 新增：串口发送函数声明（替代原usart_SendCmd，HAL库版）
// void usart_SendCmd(uint8_t *cmd, uint8_t len);  // 移除，统一在usart.h中声明

/**********************************************************
*** 函数声明：完全保留原代码，无修改
**********************************************************/
void Emm_V5_Reset_CurPos_To_Zero(uint8_t addr);                                                                                                                                     
void Emm_V5_Reset_Clog_Pro(uint8_t addr);                                                                                                                                           
void Emm_V5_Read_Sys_Params(uint8_t addr, SysParams_t s);                                                                                                                          
void Emm_V5_Modify_Ctrl_Mode(uint8_t addr, bool svF, uint8_t ctrl_mode);                                                                                                           
void Emm_V5_En_Control(uint8_t addr, bool state, bool snF);                                                                                                                        
void Emm_V5_Vel_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF);                                                                                           
void Emm_V5_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, bool raF, bool snF);                                                                   
void Emm_V5_Stop_Now(uint8_t addr, bool snF);                                                                                                                                      
void Emm_V5_Synchronous_motion(uint8_t addr);                                                                                                                                      
void Emm_V5_Origin_Set_O(uint8_t addr, bool svF);                                                                                                                                  
void Emm_V5_Origin_Modify_Params(uint8_t addr, bool svF, uint8_t o_mode, uint8_t o_dir, uint16_t o_vel, uint32_t o_tm, uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, bool potF); 
void Emm_V5_Origin_Trigger_Return(uint8_t addr, uint8_t o_mode, bool snF);                                                                                                         
void Emm_V5_Origin_Interrupt(uint8_t addr);                                                                                                                                         

#endif
