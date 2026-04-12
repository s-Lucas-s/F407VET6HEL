#ifndef __GYROSCOPE_H
#define __GYROSCOPE_H

#include "stm32F4xx_hal.h"
#include "sys.h"


typedef struct 
{
    float fAcc[3];
    float fGyro[3];
    float fAngle[3];
}GyroData_t;
void gyroscope_Init(GyroData_t *pGyroData);
void GetAttitudeData(void);

#endif
