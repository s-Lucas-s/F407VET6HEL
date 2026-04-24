# F407VET6 工程框架说明（详细版）

> 目标：用于维护和二次开发，重点覆盖 main.c 主流程与 MDK-ARM/Hardware 模块职责、数据流、调用关系。

## 1. 工程定位

- MCU：STM32F407VET6
- 工程形态：CubeMX 生成外设初始化 + HAL 驱动 + 自定义 Hardware 业务层
- 控制模式：中断驱动接收 + 主循环状态机 + PID 闭环控制
- 电机协议：Emm V5 串口指令协议

## 2. 目录分层（维护视角）

```text
F407VET6HEL/
├─ Core/
│  ├─ Src/main.c                # 启动入口、初始化顺序、主循环状态切换
│  ├─ Src/stm32f4xx_it.c        # TIM/USART中断分发（按键、姿态、串口）
│  ├─ Src/stm32f4xx_hal_msp.c   # 串口/NVIC优先级、GPIO复用配置
│  └─ Src/system_stm32f4xx.c    # SystemInit，时钟树底层配置
├─ MDK-ARM/
│  ├─ Hardware/                 # 业务硬件层（本文件重点）
│  └─ System/sys.h              # 全局模块聚合头，主工程 include 枢纽
├─ Drivers/                     # HAL/CMSIS
├─ build/                       # 构建输出
└─ F407VET6HEL.ioc              # CubeMX 配置源
```

## 3. main.c 详细执行链

### 3.1 上电初始化阶段

1. HAL_Init()
2. 设置向量表偏移：SCB->VTOR = FLASH_BASE | NVIC_VECTTAB_OFFSET
3. SystemClock_Config()：HSI + PLL，SYSCLK 168MHz，APB1 分频 4，APB2 分频 2
4. 外设初始化顺序：
	 - MX_GPIO_Init
	 - MX_USART1_UART_Init
	 - MX_USART2_UART_Init
	 - MX_TIM2_Init
	 - MX_USART3_UART_Init
	 - MX_I2C1_Init
	 - MX_USART6_UART_Init
	 - MX_UART4_Init
5. 业务模块初始化：
	 - Laser_Init
	 - OLED_Init -> OLED_ShowString("Hello!") -> OLED_Update
	 - HAL_TIM_Base_Start_IT(&htim2)
	 - 开启 huart1/2/3/4/6 的 RXNE 中断
	 - gyroscope_Init
	 - PID_Init
	 - Serial_SendPacket(0xA5, 0x5A, RESET_KEY)

### 3.2 主循环阶段

- 周期行为（10ms）
	- 读取 Key_GetCode（松手触发型键值）
	- 按键状态机
		- key=1：Questionx 题号切换（1~6循环）
		- key=2：Start_flag 翻转（开始/停止）
		- key=3：强制停止并调用 Emm_V5_Stop_Now(0, true)
	- OLED 显示 Questionx 与 Start_flag

### 3.3 与中断协同关系

- main 只做状态管理与显示
- 高实时采样/接收在中断：
	- TIM2_IRQHandler：按键扫描 + 姿态更新
	- USART2_IRQHandler：Serial_ProcessRx（视觉串口流）
	- USART3_IRQHandler：check_Bootloader（连续 0x7E 触发升级）

### 3.4 PID 实际触发顺序（从中断到电机）

1. 视觉串口每到 1 字节进入 USART2_IRQHandler。
2. IRQ 内立即调用 Serial_ProcessRx(byte)。
3. Serial_ProcessRx 先做系统门控：
	- 未握手：仅在收到 0x6B 时尝试置 Power_on_flag。
	- 未启动或题号无效：直接 return，不进入 PID。
4. 根据 Questionx 调用对应串口解析函数。
5. 在基础题3处理函数中，当整帧解析完成且帧尾正确时，调用 PID_Control(coordinate_x, coordinate_y)。
6. PID_Control 再次做题号范围检查，然后分发到 handle_PID_BasicQuestion2/3/进阶题。
7. 当前工程真正实现闭环的是 handle_PID_BasicQuestion2：
	- 计算 dt
	- Start_flag 判定与状态清零
	- I 分离、D 低通
	- PID + 前馈合成
	- 限幅
	- Emm_V5_Vel_Control 下发双轴速度

## 4. Hardware 目录文件级框架（重点）

## 4.1 运动控制与协议层

### EMM_V5.h / EMM_V5.c

- 作用：封装步进闭环驱动 Emm V5 全部控制指令
- 关键接口：
	- Emm_V5_Vel_Control：速度模式
	- Emm_V5_Pos_Control：位置模式
	- Emm_V5_Stop_Now：立即停机
	- Emm_V5_En_Control：使能控制
	- Emm_V5_Read_Sys_Params：读取状态参数
- 下层依赖：usart_SendCmd（由 usart.c 发送到 huart1/huart6）
- 注意：命令打包字节末尾常见 0x6B，回零参数里有 0x9B 特例（代码已按协议实现）

### PID.h / PID.c

- 作用：视觉误差 -> 电机速度指令 的控制器
- 结构：按 Questionx 分发到不同题目的处理函数
- 已实现重点：handle_PID_BasicQuestion2（当前唯一完整闭环实现）
	- 入口参数：err_x / err_y（来自视觉坐标解析结果）
	- 周期估计：dt = HAL_GetTick() - last_tick，约束到 1~100ms
	- 状态量：integral_x/y、xerr_last/yerr_last、last_d_x/y
	- 停机策略：Start_flag=0 时清零积分和历史误差，直接返回
	- 积分分离：|err| < 100 才积分，且积分限幅 ±500000
	- 微分环节：raw_d = (err-now_last)/dt，再做一阶低通
	- 前馈：feed_x/y = Target_Vertical_x/y * Kf（Kf=10）
	- 合成：out = P + I + D + feed
	- 输出保护：最终速度限幅 ±16000
	- 指令下发：按正负号映射方向位 + 速度绝对值
		- X 轴 addr=1：out>=0 -> dir=1，out<0 -> dir=0
		- Y 轴 addr=2：out>=0 -> dir=0，out<0 -> dir=1
	- 结束：更新 xerr_last/yerr_last 供下一拍微分计算

### PID 运行门控与状态机（必须满足）

- 门控1：Questionx 有效（1~5）
- 门控2：Power_on_flag = 1（串口握手完成）
- 门控3：Start_flag = 1（用户已启动）
- 门控4：协议帧完整且帧尾合法（0x6B）
- 仅当 1~4 同时满足，才会进入有效 PID 计算并下发电机指令。

### PID 每拍运算顺序（可对照单步调试）

```text
输入: err_x, err_y
	-> 计算 dt 并做容错 [1,100]
	-> 若 Start_flag=0: 清零积分/历史误差并 return
	-> 积分分离累积 I
	-> 计算原始微分 d_raw
	-> 一阶低通得到 d_lpf
	-> PID_x/PID_y 计算
	-> 加前馈 feed_x/feed_y
	-> 输出限幅 ±16000
	-> 符号转方向位，绝对值转速度值
	-> Emm_V5_Vel_Control(轴1/轴2)
	-> 保存 last_err/last_d
```

### 当前 PID 逻辑的实际结论（按现代码）

1. PID_Init 将 kp/ki/kd 全设为 0，若外部未再次赋值，则闭环项输出恒为 0。
2. 在上述条件下，电机输出主要来自前馈 Target_Vertical_x/y。
3. Questionx=2 的串口处理当前主要做显示/停机握手，未调用 PID_Control；Questionx=3 才触发 PID_Control。
4. 因此“题号、握手、启动位、参数赋值”四者任一不满足，都可能表现为“PID没动作”。

## 4.2 通信链路层

### Serial.h / Serial.c（主业务串口，huart2）

- 作用：
	- 通用发送接口（字节/数组/字符串/封包）
	- 视觉协议解析
	- 基于 Questionx 的业务分发
- 数据帧特征：以 0xB6 起始，含两个 float 坐标，尾字节 0x6B
- 关键状态变量：
	- Start_flag：是否启动任务
	- Power_on_flag：是否已收到启动握手
	- Questionx：题号，决定分发函数
- 关键入口：Serial_ProcessRx（在 USART2 IRQ 中逐字节调用）

### usart.h / usart.c（底层多串口发送聚合）

- 作用：将命令按地址路由到 huart1/huart6（电机驱动通道）
- 策略：
	- cmd[0]==0：同时发送 huart1 + huart6
	- cmd[0]==1：仅 huart6
	- cmd[0]==2：仅 huart1
- 共享变量：rxCmd、rxCount、rx1FrameFlag、rx6FrameFlag

### Serial1.h / Serial1.c（调试串口，huart1）

- 作用：向上位机发送调试数据
- 特点：VOFA_Send3Ch 发送 3 路 float + VOFA 帧尾

## 4.3 人机交互与显示

### Key.h / Key.c

- 作用：按键电平读取 + 松手触发键值输出
- 机制：
	- Key_Get：读取 KEY1/KEY2/KEY3 当前状态（低电平按下）
	- Key_LoopDetect：边沿判断，松手时写入 Key_Code
	- Key_GetCode：主循环读取后清零（一次性事件）
- 调用位置：TIM2_IRQHandler 周期调用 Key_LoopDetect

### OLED.h / OLED.c / OLED_Data.h / OLED_Data.c

- 作用：0.96 寸 I2C OLED 显示与绘图
- I2C 句柄：hi2c1（来自 main.c）
- 写命令/写数据：HAL_I2C_Mem_Write
- 设计要点：
	- 先写显存 OLED_DisplayBuf，再 OLED_Update 刷屏
	- 支持字符串、数字、浮点、图元绘制

### Laser.h / Laser.c

- 作用：激光开关控制
- 引脚：PB14（工程约定：高电平开启）
- 接口：Laser_On / Laser_Off / Laser_SetState

## 4.4 升级与引导

### IAP.h / IAP.c

- 作用：APP 侧升级触发与标志管理
- Flash 地址：
	- IAP_FLAG_ADDR = 0x08008000
	- APPLICATION_ADDRESS = 0x0800C000
	- NVIC_VECTTAB_OFFSET = 0x0000C000
- 触发逻辑：串口连续收到 5 个 0x7E -> 写升级标志 -> 系统复位
- 注意点：写 Flash 前后做中断开关、解锁/上锁

## 5. 中断驱动框架图

```text
TIM2 IRQ (约1ms节拍)
	├─ Key_LoopDetect() 每10次 -> 100Hz
	└─ GetAttitudeData() 每20次 -> 50Hz

USART2 IRQ (视觉/业务串口)
	└─ Serial_ProcessRx(byte)
			 ├─ 启动握手判断（Power_on_flag）
			 ├─ 按 Questionx 分发解析
			 └─ 仅在对应题号且帧校验通过时触发 PID_Control

USART3 IRQ (蓝牙/IAP)
	└─ check_Bootloader(byte)
			 └─ 5次0x7E -> IAP_WriteFlag -> NVIC_SystemReset
```

## 6. main.c 与 Hardware 的核心调用关系

```text
main
 ├─ 初始化：Laser_Init / OLED_Init / gyroscope_Init / PID_Init
 ├─ 开中断：TIM2 + UART1/2/3/4/6 RXNE
 └─ while(1)
		├─ Key_GetCode -> Questionx / Start_flag 状态切换
		├─ 异常停止：Emm_V5_Stop_Now
		└─ OLED 状态显示

中断侧并行
 ├─ TIM2_IRQHandler -> Key_LoopDetect / GetAttitudeData
 ├─ USART2_IRQHandler -> Serial_ProcessRx -> PID_Control -> Emm_V5_Vel_Control
 └─ USART3_IRQHandler -> check_Bootloader
```

## 6.1 PID 调试最小观察链（建议按顺序打点）

1. USART2_IRQHandler 是否持续进中断（确认视觉数据在进来）。
2. Serial_ProcessRx 是否通过 Power_on_flag 与 Start_flag 门控。
3. 题号是否进入会触发 PID 的分支（当前代码主要是基础题3）。
4. PID_Control 是否被调用，err_x/err_y 是否变化。
5. Position_PID_x/y 的 kp/ki/kd 是否非零。
6. Emm_V5_Vel_Control 调用参数是否合理（方向位和速度值）。

## 7. 维护注意事项（结合当前工程）

1. 新增 C/H 文件后，需要同步加入 .eide/eide.yml 的 virtualFolder.files，否则 EIDE 不会编译。
2. Serial.c 里引用头文件为 laser.h，而文件名是 Laser.h；Windows 下不敏感，迁移到区分大小写环境要统一。
3. main.c 里 Laser_Init 调用了两次，若后续出现上电瞬态问题可先排查该重复初始化是否必要。
4. USART2 IRQ 里直接读取 DR 并调用 Serial_ProcessRx，若后续协议变长建议评估 FIFO 或 DMA 以降低中断占用。

## 8. 快速定位索引

- 主流程入口：Core/Src/main.c
- 中断分发：Core/Src/stm32f4xx_it.c
- 电机协议：MDK-ARM/Hardware/EMM_V5.c
- 视觉解析与状态机：MDK-ARM/Hardware/Serial.c
- 闭环控制：MDK-ARM/Hardware/PID.c
- 升级触发：MDK-ARM/Hardware/IAP.c

