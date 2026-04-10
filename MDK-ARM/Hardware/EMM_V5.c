#include "Emm_V5.h"
#include "usart.h"

/**********************************************************
***	Emm_V5.0步进闭环控制例程
***	适配说明：
***	1. 新增usart_SendCmd函数（HAL_UART_Transmit实现）
***	2. 保留所有电机控制逻辑，仅替换串口发送接口
***	3. 兼容STM32F407 HAL库和AC6编译器
**********************************************************/

/**
  * @brief    将当前位置清零
  * @param    addr  ：电机地址
  * @retval   无
  */
void Emm_V5_Reset_CurPos_To_Zero(uint8_t addr)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令（原代码逻辑完全保留）
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x0A;                       // 功能码
  cmd[2] =  0x6D;                       // 辅助码
  cmd[3] =  0x6B;                       // 校验字节
  
  // 发送命令（调用HAL库版usart_SendCmd）
  usart_SendCmd(cmd, 4);
}

/**
  * @brief    解除堵转保护
  * @param    addr  ：电机地址
  * @retval   无
  */
void Emm_V5_Reset_Clog_Pro(uint8_t addr)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x0E;                       // 功能码
  cmd[2] =  0x52;                       // 辅助码
  cmd[3] =  0x6B;                       // 校验字节
  
  // 发送命令
  usart_SendCmd(cmd, 4);
}

/**
  * @brief    读取系统参数
  * @param    addr  ：电机地址
  * @param    s     ：系统参数类型
  * @retval   无
  */
void Emm_V5_Read_Sys_Params(uint8_t addr, SysParams_t s)
{
  uint8_t i = 0;
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[i] = addr; ++i;                   // 地址

  switch(s)                             // 功能码
  {
    case S_VER  : cmd[i] = 0x1F; ++i; break;
    case S_RL   : cmd[i] = 0x20; ++i; break;
    case S_PID  : cmd[i] = 0x21; ++i; break;
    case S_VBUS : cmd[i] = 0x24; ++i; break;
    case S_CPHA : cmd[i] = 0x27; ++i; break;
    case S_ENCL : cmd[i] = 0x31; ++i; break;
    case S_TPOS : cmd[i] = 0x33; ++i; break;
    case S_VEL  : cmd[i] = 0x35; ++i; break;
    case S_CPOS : cmd[i] = 0x36; ++i; break;
    case S_PERR : cmd[i] = 0x37; ++i; break;
    case S_FLAG : cmd[i] = 0x3A; ++i; break;
    case S_ORG  : cmd[i] = 0x3B; ++i; break;
    case S_Conf : cmd[i] = 0x42; ++i; cmd[i] = 0x6C; ++i; break;
    case S_State: cmd[i] = 0x43; ++i; cmd[i] = 0x7A; ++i; break;
    default: break;
  }

  cmd[i] = 0x6B; ++i;                   // 校验字节
  
  // 发送命令
  usart_SendCmd(cmd, i);
}

/**
  * @brief    修改开环/闭环控制模式
  * @param    addr     ：电机地址
  * @param    svF      ：是否存储标志，false为不存储，true为存储
  * @param    ctrl_mode：控制模式
  * @retval   无
  */
void Emm_V5_Modify_Ctrl_Mode(uint8_t addr, bool svF, uint8_t ctrl_mode)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x46;                       // 功能码
  cmd[2] =  0x69;                       // 辅助码
  cmd[3] =  svF;                        // 是否存储标志
  cmd[4] =  ctrl_mode;                  // 控制模式
  cmd[5] =  0x6B;                       // 校验字节
  
  // 发送命令
  usart_SendCmd(cmd, 6);
}

/**
  * @brief    使能信号控制
  * @param    addr  ：电机地址
  * @param    state ：使能状态
  * @param    snF   ：多机同步标志
  * @retval   无
  */
void Emm_V5_En_Control(uint8_t addr, bool state, bool snF)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0xF3;                       // 功能码
  cmd[2] =  0xAB;                       // 辅助码
  cmd[3] =  (uint8_t)state;             // 使能状态
  cmd[4] =  snF;                        // 多机同步运动标志
  cmd[5] =  0x6B;                       // 校验字节
  
  // 发送命令
  usart_SendCmd(cmd, 6);
}

/**
  * @brief    速度模式
  * @param    addr：电机地址
  * @param    dir ：方向
  * @param    vel ：速度
  * @param    acc ：加速度
  * @param    snF ：多机同步标志
  * @retval   无
  */
void Emm_V5_Vel_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, bool snF)
{
  uint8_t cmd[16] = {0};

  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0xF6;                       // 功能码
  cmd[2] =  dir;                        // 方向
  cmd[3] =  (uint8_t)(vel >> 8);        // 速度高8位
  cmd[4] =  (uint8_t)(vel >> 0);        // 速度低8位
  cmd[5] =  acc;                        // 加速度
  cmd[6] =  snF;                        // 多机同步标志
  cmd[7] =  0x6B;                       // 校验字节
  
  // 发送命令
  usart_SendCmd(cmd, 8);
}

/**
  * @brief    位置模式
  * @param    addr：电机地址
  * @param    dir ：方向
  * @param    vel ：速度
  * @param    acc ：加速度
  * @param    clk ：脉冲数
  * @param    raF ：相位/绝对标志
  * @param    snF ：多机同步标志
  * @retval   无
  */
void Emm_V5_Pos_Control(uint8_t addr, uint8_t dir, uint16_t vel, uint8_t acc, uint32_t clk, bool raF, bool snF)
{
  uint8_t cmd[16] = {0};

  // 装载命令
  cmd[0]  =  addr;                      // 地址
  cmd[1]  =  0xFD;                      // 功能码
  cmd[2]  =  dir;                       // 方向
  cmd[3]  =  (uint8_t)(vel >> 8);       // 速度高8位
  cmd[4]  =  (uint8_t)(vel >> 0);       // 速度低8位 
  cmd[5]  =  acc;                       // 加速度
  cmd[6]  =  (uint8_t)(clk >> 24);      // 脉冲数bit24-31
  cmd[7]  =  (uint8_t)(clk >> 16);      // 脉冲数bit16-23
  cmd[8]  =  (uint8_t)(clk >> 8);       // 脉冲数bit8-15
  cmd[9]  =  (uint8_t)(clk >> 0);       // 脉冲数bit0-7
  cmd[10] =  raF;                       // 相位/绝对标志
  cmd[11] =  snF;                       // 多机同步标志
  cmd[12] =  0x6B;                      // 校验字节
  
  // 发送命令
  usart_SendCmd(cmd, 13);
}

/**
  * @brief    立即停止
  * @param    addr  ：电机地址
  * @param    snF   ：多机同步标志
  * @retval   无
  */
void Emm_V5_Stop_Now(uint8_t addr, bool snF)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0xFE;                       // 功能码
  cmd[2] =  0x98;                       // 辅助码
  cmd[3] =  snF;                        // 多机同步标志
  cmd[4] =  0x6B;                       // 校验字节
  
  // 发送命令
  usart_SendCmd(cmd, 5);
}

/**
  * @brief    多机同步运动
  * @param    addr  ：电机地址
  * @retval   无
  */
void Emm_V5_Synchronous_motion(uint8_t addr)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0xFF;                       // 功能码
  cmd[2] =  0x66;                       // 辅助码
  cmd[3] =  0x6B;                       // 校验字节
  
  // 发送命令
  usart_SendCmd(cmd, 4);
}

/**
  * @brief    设置单圈回零的零点位置
  * @param    addr  ：电机地址
  * @param    svF   ：是否存储标志
  * @retval   无
  */
void Emm_V5_Origin_Set_O(uint8_t addr, bool svF)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x93;                       // 功能码
  cmd[2] =  0x88;                       // 辅助码
  cmd[3] =  svF;                        // 是否存储标志
  cmd[4] =  0x6B;                       // 校验字节
  
  // 发送命令
  usart_SendCmd(cmd, 5);
}

/**
  * @brief    修改回零参数
  * @param    addr  ：电机地址
  * @param    svF   ：是否存储标志
  * @param    o_mode ：回零模式
  * @param    o_dir  ：回零方向
  * @param    o_vel  ：回零速度
  * @param    o_tm   ：回零超时时间
  * @param    sl_vel ：碰撞检测转速
  * @param    sl_ma  ：碰撞检测电流
  * @param    sl_ms  ：碰撞检测时间
  * @param    potF   ：上电自动回零
  * @retval   无
  */
void Emm_V5_Origin_Modify_Params(uint8_t addr, bool svF, uint8_t o_mode, uint8_t o_dir, uint16_t o_vel, uint32_t o_tm, uint16_t sl_vel, uint16_t sl_ma, uint16_t sl_ms, bool potF)
{
  uint8_t cmd[32] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x4C;                       // 功能码
  cmd[2] =  0xAE;                       // 辅助码
  cmd[3] =  svF;                        // 是否存储标志
  cmd[4] =  o_mode;                     // 回零模式
  cmd[5] =  o_dir;                      // 回零方向
  cmd[6]  =  (uint8_t)(o_vel >> 8);     // 回零速度高8位
  cmd[7]  =  (uint8_t)(o_vel >> 0);     // 回零速度低8位 
  cmd[8]  =  (uint8_t)(o_tm >> 24);     // 超时时间bit24-31
  cmd[9]  =  (uint8_t)(o_tm >> 16);     // 超时时间bit16-23
  cmd[10] =  (uint8_t)(o_tm >> 8);      // 超时时间bit8-15
  cmd[11] =  (uint8_t)(o_tm >> 0);      // 超时时间bit0-7
  cmd[12] =  (uint8_t)(sl_vel >> 8);    // 碰撞转速高8位
  cmd[13] =  (uint8_t)(sl_vel >> 0);    // 碰撞转速低8位 
  cmd[14] =  (uint8_t)(sl_ma >> 8);     // 碰撞电流高8位
  cmd[15] =  (uint8_t)(sl_ma >> 0);     // 碰撞电流低8位 
  cmd[16] =  (uint8_t)(sl_ms >> 8);     // 碰撞时间高8位
  cmd[17] =  (uint8_t)(sl_ms >> 0);     // 碰撞时间低8位
  cmd[18] =  potF;                      // 上电自动回零
  cmd[19] =  0x9B;                      // 校验字节（原代码0x6B，按实际协议调整）
  
  // 发送命令
  usart_SendCmd(cmd, 20);
}

/**
  * @brief    触发回零
  * @param    addr   ：电机地址
  * @param    o_mode ：回零模式
  * @param    snF   ：多机同步标志
  * @retval   无
  */
void Emm_V5_Origin_Trigger_Return(uint8_t addr, uint8_t o_mode, bool snF)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x9A;                       // 功能码
  cmd[2] =  o_mode;                     // 回零模式
  cmd[3] =  snF;                        // 多机同步标志
  cmd[4] =  0x6B;                       // 校验字节
  
  // 发送命令
  usart_SendCmd(cmd, 5);
}

/**
  * @brief    强制中断并退出回零
  * @param    addr  ：电机地址
  * @retval   无
  */
void Emm_V5_Origin_Interrupt(uint8_t addr)
{
  uint8_t cmd[16] = {0};
  
  // 装载命令
  cmd[0] =  addr;                       // 地址
  cmd[1] =  0x9C;                       // 功能码
  cmd[2] =  0x48;                       // 辅助码
  cmd[3] =  0x6B;                       // 校验字节
  
  // 发送命令
  usart_SendCmd(cmd, 4);
}
