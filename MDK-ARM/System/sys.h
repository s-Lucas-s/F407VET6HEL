#ifndef __SYS_H
#define __SYS_H

/* ——————————include—————————— */
#include "stm32F4xx_hal.h"
#include "main.h" 
#include "Delay.h"
#include "Emm_V5.h"
#include "OLED.h"
#include "PID.h"
#include "usart.h"
#include "uart4.h"
#include "Serial.h"
#include "Serial1.h"
#include "Key.h"
#include "IAP.h"
#include "wit_c_sdk.h" 
#include "gyroscope.h"

/* ————————————变量———————————— */
extern bool Stop_flag;
extern int8_t Questionx;
extern bool Power_on_flag;

// float fAcc[3], fGyro[3], fAngle[3];
// int i;

/* —————————————宏————————————— */
#define ABS(x)      ((x) >= 0 ? (x) : -(x))
#define Max_x_angle 180

/*陀螺仪*/
#define ACC_UPDATE   0x01
#define GYRO_UPDATE  0x02
#define ANGLE_UPDATE 0x04
#define MAG_UPDATE   0x08
#define READ_UPDATE  0x80

/* ————————————函数———————————— */
float Check_angle(uint8_t addr);
void check_Bootloader(uint8_t data);

#endif
