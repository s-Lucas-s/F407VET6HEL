#ifndef __JY901_H
#define __JY901_H
#include <stdint.h>
#include "MyI2C.h"
typedef struct JY901VAR JY901VAR;
typedef struct JY901DATA JY901DATA;
typedef struct JY901FUN JY901FUN;
typedef struct JY901PERE JY901PERE;
typedef struct JY901_Driver JY901_Driver;
typedef struct JY901_PID JY901_PID;

struct JY901_PID
{
    float p;
    float i;
    float d;
    float out;
    float err;
    float err_last;
    float target;
};

/*变量*/
struct JY901VAR
{
    float yaw;
    float roll;
    float pitch;
    float gx;
    float gy;
    float gz;
    JY901_PID pid;
};
/*内部函数和参数*/
struct JY901PERE
{
    uint8_t Address;
    int16_t (*Read)(JY901_Driver *self,uint8_t RegAddress);
    void (*Write)(JY901_Driver *self,uint8_t RegAddress, uint8_t WriteData);
    void (*Write2)(JY901_Driver *self,uint8_t RegAddress, uint8_t* WriteData);
    MyI2C_Driver_t *i2c_driver;
};
/*寄存器地址*/
struct JY901DATA
{
    uint8_t Roll;
    uint8_t Pitch;
    uint8_t Yaw;
    uint8_t AX;
    uint8_t AY;
    uint8_t AZ;
    uint8_t GX;
    uint8_t GY;
    uint8_t GZ;
    uint8_t HX;
    uint8_t HY;
    uint8_t HZ;
    uint8_t LEDOFF;

};
/*对外函数*/
struct JY901FUN
{
    void (*Init)(JY901_Driver *self);
    void (*ROLL_GET)(JY901_Driver *self);
    void (*PITCH_GET)(JY901_Driver *self);
    void (*YAW_GET)(JY901_Driver *self);
    void (*GX_GET)(JY901_Driver *self);
    void (*GY_GET)(JY901_Driver *self);
    void (*GZ_GET)(JY901_Driver *self);
    void (*YAW_ZERO)(JY901_Driver *self);
    void (*YAW_PID_OUT)(JY901_Driver *self);
    void (*YAW_PID_SET)(JY901_Driver *self, float target, float kp,float ki,float kd);
};
struct JY901_Driver
{
    JY901VAR var;
    JY901DATA data;
    JY901FUN *fun;
    JY901PERE *pere;
};
JY901_Driver* JY901_Create(uint8_t JY901_Address, GPIO_TypeDef* SCL_port, uint16_t SCL_pin, GPIO_TypeDef* SDA_port, uint16_t SDA_pin);
#endif /* __JY901_H */
