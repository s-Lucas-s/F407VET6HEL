#include "Serial1.h"

//芷乐写的上位机调参
//使用vofa，通过蓝牙串口

// ===================== 基础发送函数（不变） =====================
void Serial1_SendByte(uint8_t Byte)
{
    HAL_UART_Transmit(&huart1, &Byte, 1, 10);
}

void Serial1_SendArray(uint8_t *Array, uint16_t Length)
{
    HAL_UART_Transmit(&huart1, Array, Length, 100);
}

void Serial1_SendString(char *String)
{
    uint16_t len = 0;
    while (String[len] != '\0')
    {
        len++;
    }
    Serial1_SendArray((uint8_t *)String, len);
}

// ===================== VOFA+ 3通道波形发送函数（不变） =====================
void VOFA_Send3Ch(float ch1, float ch2, float ch3)
{
    typedef struct
    {
        float data[3];
        uint8_t tail[4];
    } VOFA_Frame_t;

    VOFA_Frame_t frame = {0};
    frame.data[0] = ch1;
    frame.data[1] = ch2;
    frame.data[2] = ch3;
    frame.tail[0] = 0x00;
    frame.tail[1] = 0x00;
    frame.tail[2] = 0x80;
    frame.tail[3] = 0x7F;

    Serial1_SendArray((uint8_t *)&frame, sizeof(frame));
}
