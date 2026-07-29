#ifndef __MY_I2C_H
#define __MY_I2C_H

#include "main.h"

// 1. 定义一个结构体，包含 SDA 和 SCL 的端口与引脚号
typedef struct {
    GPIO_TypeDef* SCL_Port;
    uint16_t      SCL_Pin;
    GPIO_TypeDef* SDA_Port;
    uint16_t      SDA_Pin;
} SoftI2C_Bus;

// 2. 函数声明（所有函数现在都要接收 SoftI2C_Bus *bus 参数）
void Software_I2C_Init(SoftI2C_Bus *bus);
void Software_I2C_Start(SoftI2C_Bus *bus);
void Software_I2C_Stop(SoftI2C_Bus *bus);
HAL_StatusTypeDef Software_I2C_WaitACK(SoftI2C_Bus *bus);
void Software_I2C_SendACK(SoftI2C_Bus *bus);
void Software_I2C_SendNACK(SoftI2C_Bus *bus);
void Software_I2C_WriteByte(SoftI2C_Bus *bus, uint8_t byte);
uint8_t Software_I2C_ReadByte(SoftI2C_Bus *bus);

// 扫描工具也要传参
void Software_I2C_Scan(SoftI2C_Bus *bus, UART_HandleTypeDef *huart);

#endif


