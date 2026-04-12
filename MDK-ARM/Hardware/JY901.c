#include "jy901.h"
#include <stdlib.h>
#include "stm32f4xx_hal.h"
#include <stdint.h>
void Init(JY901_Driver *self)
{
    // Initialize I2C driver
    self->pere->i2c_driver->fun->MyI2C_Init(self->pere->i2c_driver);

    self->var.yaw = 0.0f;
    self->var.roll = 0.0f;
    self->var.pitch = 0.0f;
    self->var.gx = 0.0f;
    self->var.gy = 0.0f;
    self->var.gz = 0.0f;
    self->var.pid.p = 0.0f;
    self->var.pid.i = 0.0f;
    self->var.pid.d = 0.0f;
    self->var.pid.err = 0.0f;
    self->var.pid.err_last = 0.0f;
    self->var.pid.out = 0.0f;
    self->var.pid.target = 0.0f;

    self->data.Roll = 0x3d;
    self->data.Pitch = 0x3e;
    self->data.Yaw = 0x3f;
    self->data.AX = 0x34;
    self->data.AY = 0x35;
    self->data.AZ = 0x36;
    self->data.GX = 0x37;
    self->data.GY = 0x38;
    self->data.GZ = 0x39;
    self->data.HX = 0x3a;
    self->data.HY = 0x3b;
    self->data.HZ = 0x3c;
    self->data.LEDOFF = 0x1b;
}

void JY901_Write(JY901_Driver *self, uint8_t RegAddress, uint8_t WriteData)
{
    uint8_t ack;
    // 起始
    self->pere->i2c_driver->fun->MyI2C_Start(self->pere->i2c_driver);

    // 发送地址(写)
    self->pere->i2c_driver->fun->MyI2C_Send_Byte(self->pere->i2c_driver, self->pere->Address << 1);
    self->pere->i2c_driver->fun->MyI2C_ReceiveAck(self->pere->i2c_driver, &ack);

    // 发送寄存器地址
    self->pere->i2c_driver->fun->MyI2C_Send_Byte(self->pere->i2c_driver, RegAddress);
    self->pere->i2c_driver->fun->MyI2C_ReceiveAck(self->pere->i2c_driver, &ack);

    // 发送数据
    self->pere->i2c_driver->fun->MyI2C_Send_Byte(self->pere->i2c_driver, WriteData);
    self->pere->i2c_driver->fun->MyI2C_ReceiveAck(self->pere->i2c_driver, &ack);

    // 停止
    self->pere->i2c_driver->fun->MyI2C_Stop(self->pere->i2c_driver);
}

void JY901_Write2(JY901_Driver *self, uint8_t RegAddress, uint8_t *unlock_reg1)
{
    uint8_t ack;
    // 起始
    self->pere->i2c_driver->fun->MyI2C_Start(self->pere->i2c_driver);

    // 发送地址(写)
    self->pere->i2c_driver->fun->MyI2C_Send_Byte(self->pere->i2c_driver, self->pere->Address << 1);
    self->pere->i2c_driver->fun->MyI2C_ReceiveAck(self->pere->i2c_driver, &ack);

    // 按模块要求的格式发送：0xFF, 0xAA, RegAddr, DataL, DataH
    self->pere->i2c_driver->fun->MyI2C_Send_Byte(self->pere->i2c_driver, 0xFF);
    self->pere->i2c_driver->fun->MyI2C_ReceiveAck(self->pere->i2c_driver, &ack);

    self->pere->i2c_driver->fun->MyI2C_Send_Byte(self->pere->i2c_driver, 0xAA);
    self->pere->i2c_driver->fun->MyI2C_ReceiveAck(self->pere->i2c_driver, &ack);
    // 寄存器地址
    self->pere->i2c_driver->fun->MyI2C_Send_Byte(self->pere->i2c_driver, RegAddress);
    self->pere->i2c_driver->fun->MyI2C_ReceiveAck(self->pere->i2c_driver, &ack);

    // 发送数据低字节、数据高字节
    self->pere->i2c_driver->fun->MyI2C_Send_Byte(self->pere->i2c_driver, unlock_reg1[0]);
    self->pere->i2c_driver->fun->MyI2C_ReceiveAck(self->pere->i2c_driver, &ack);

    self->pere->i2c_driver->fun->MyI2C_Send_Byte(self->pere->i2c_driver, unlock_reg1[1]);
    self->pere->i2c_driver->fun->MyI2C_ReceiveAck(self->pere->i2c_driver, &ack);

    // 停止
    self->pere->i2c_driver->fun->MyI2C_Stop(self->pere->i2c_driver);
    HAL_Delay(5);
}

int16_t JY901_Read(JY901_Driver *self, uint8_t RegAddress)
{
    uint8_t ack;
    uint8_t DataL, DataH;
    int16_t ReadData;

    // 起始
    self->pere->i2c_driver->fun->MyI2C_Start(self->pere->i2c_driver);

    // 发送地址(写)
    self->pere->i2c_driver->fun->MyI2C_Send_Byte(self->pere->i2c_driver, self->pere->Address << 1);
    self->pere->i2c_driver->fun->MyI2C_ReceiveAck(self->pere->i2c_driver, &ack);

    // 发送寄存器
    self->pere->i2c_driver->fun->MyI2C_Send_Byte(self->pere->i2c_driver, RegAddress);
    self->pere->i2c_driver->fun->MyI2C_ReceiveAck(self->pere->i2c_driver, &ack);

    // 重复起始
    self->pere->i2c_driver->fun->MyI2C_Start(self->pere->i2c_driver);

    // 发送地址(读)
    self->pere->i2c_driver->fun->MyI2C_Send_Byte(self->pere->i2c_driver, (self->pere->Address << 1) | 1);
    self->pere->i2c_driver->fun->MyI2C_ReceiveAck(self->pere->i2c_driver, &ack);

    // 读低8位 → ACK
    self->pere->i2c_driver->fun->MyI2C_Receive_Byte(self->pere->i2c_driver, &DataL);
    self->pere->i2c_driver->fun->MyI2C_SendAck(self->pere->i2c_driver, 0);

    // 读高8位 → NACK
    self->pere->i2c_driver->fun->MyI2C_Receive_Byte(self->pere->i2c_driver, &DataH);
    self->pere->i2c_driver->fun->MyI2C_SendAck(self->pere->i2c_driver, 1);

    // 停止
    self->pere->i2c_driver->fun->MyI2C_Stop(self->pere->i2c_driver);

    // 组合16位数据
    ReadData = (DataH << 8) | DataL;
    return ReadData;
}

void ROLL_GET(JY901_Driver *self)
{
   float roll;
   roll= self->pere->Read(self, self->data.Roll);
   self->var.roll=roll/32768.0f * 180.0f;;
}
void PITCH_GET(JY901_Driver *self)
{
    float pitch;
    pitch = self->pere->Read(self, self->data.Pitch);
    self->var.pitch = pitch/32768.0f * 180.0f;
}
void YAW_GET(JY901_Driver *self)
{
    float yaw;
    yaw = self->pere->Read(self, self->data.Yaw);
    self->var.yaw = yaw/32768.0f * 180.0f;
}

void GX_GET(JY901_Driver *self)
{
    float gx;
    gx = self->pere->Read(self, self->data.GX);
    self->var.gx = gx/32768.0f * 2000.0f;
}
void GY_GET(JY901_Driver *self)
{
    float gy;
    gy = self->pere->Read(self, self->data.GY);
    self->var.gy = gy/32768.0f * 2000.0f;
}
void GZ_GET(JY901_Driver *self)
{
    float gz;
    gz = self->pere->Read(self, self->data.GZ);
    self->var.gz = gz/32768.0f * 2000.0f;
}

void YAW_ZERO(JY901_Driver *self)
{
    uint8_t unlock_reg1[2] = {0x88, 0xB5};
    self->pere->Write2(self,0x69, unlock_reg1);
    HAL_Delay(200);
    //uint8_t unlock_reg2[2] = {0x04, 0x00};
    self->pere->Write(self,0x01, 0x04);
    HAL_Delay(200);
    //uint8_t unlock_reg3[2] = {0x00, 0x00};
    self->pere->Write2(self,0x00, 0x00);
    HAL_Delay(200);
}

void YAW_PID_OUT(JY901_Driver *self)
{
    self->fun->YAW_GET(self); // 获取当前yaw角度
    self->var.pid.err = self->var.pid.target - self->var.yaw;
    self->var.pid.out=self->var.pid.p*self->var.pid.err;;

}

void YAW_PID_SET(JY901_Driver *self, float target, float kp, float ki, float kd)
{
    self->var.pid.target = target;
    self->var.pid.err =  self->var.pid.target- self->var.yaw;
    self->var.pid.p = kp;
    self->var.pid.i = ki;
    self->var.pid.d = kd;
}

JY901_Driver* JY901_Create(uint8_t JY901_Address, GPIO_TypeDef* SCL_port, uint16_t SCL_pin, GPIO_TypeDef* SDA_port, uint16_t SDA_pin)
{
    JY901_Driver* self = (JY901_Driver*)malloc(sizeof(JY901_Driver));
    if (self != NULL)
    {
        self->pere = (JY901PERE*)malloc(sizeof(JY901PERE));
        self->pere->i2c_driver = MyI2C_Create(SCL_port, SCL_pin, SDA_port, SDA_pin);
        self->pere->Read = JY901_Read;
        self->pere->Write = JY901_Write;
        self->pere->Write2 = JY901_Write2;
        self->pere->Address = JY901_Address;

        self->fun = (JY901FUN*)malloc(sizeof(JY901FUN));
        self->fun->Init = Init;
        self->fun->ROLL_GET = ROLL_GET;
        self->fun->PITCH_GET = PITCH_GET;
        self->fun->YAW_GET = YAW_GET;
        self->fun->GX_GET = GX_GET;
        self->fun->GY_GET = GY_GET;
        self->fun->GZ_GET = GZ_GET;
        self->fun->YAW_ZERO = YAW_ZERO;
        self->fun->YAW_PID_OUT = YAW_PID_OUT;
        self->fun->YAW_PID_SET = YAW_PID_SET;
    }
    return self;
}
