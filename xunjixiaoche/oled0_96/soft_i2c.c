#include "soft_i2c.h"
#include <stdio.h>
#include <string.h>

// 延时函数保持不变
static void I2C_Delay(void)
{
    volatile uint32_t i = 30; 
    while(i--);
}

/**
 * @brief  初始化指定的 I2C 总线
 */
void Software_I2C_Init(SoftI2C_Bus *bus)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    // 1. 自动判断并开启时钟 (支持 GPIOA - GPIOC，更多请自行添加)
    if(bus->SCL_Port == GPIOA || bus->SDA_Port == GPIOA) __HAL_RCC_GPIOA_CLK_ENABLE();
    if(bus->SCL_Port == GPIOB || bus->SDA_Port == GPIOB) __HAL_RCC_GPIOB_CLK_ENABLE();
    if(bus->SCL_Port == GPIOC || bus->SDA_Port == GPIOC) __HAL_RCC_GPIOC_CLK_ENABLE();

    // 2. 配置 SCL
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;
    GPIO_InitStruct.Pull = GPIO_PULLUP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    
    GPIO_InitStruct.Pin = bus->SCL_Pin;
    HAL_GPIO_Init(bus->SCL_Port, &GPIO_InitStruct);

    // 3. 配置 SDA
    GPIO_InitStruct.Pin = bus->SDA_Pin;
    HAL_GPIO_Init(bus->SDA_Port, &GPIO_InitStruct);

    // 4. 默认拉高
    HAL_GPIO_WritePin(bus->SCL_Port, bus->SCL_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(bus->SDA_Port, bus->SDA_Pin, GPIO_PIN_SET);
}

// --- 下面所有函数，把具体的引脚操作换成 bus->xxx ---

void Software_I2C_Start(SoftI2C_Bus *bus) 
{
    HAL_GPIO_WritePin(bus->SDA_Port, bus->SDA_Pin, GPIO_PIN_SET);
    HAL_GPIO_WritePin(bus->SCL_Port, bus->SCL_Pin, GPIO_PIN_SET);
    I2C_Delay();
    HAL_GPIO_WritePin(bus->SDA_Port, bus->SDA_Pin, GPIO_PIN_RESET);
    I2C_Delay();
    HAL_GPIO_WritePin(bus->SCL_Port, bus->SCL_Pin, GPIO_PIN_RESET);
}

void Software_I2C_Stop(SoftI2C_Bus *bus) 
{
    HAL_GPIO_WritePin(bus->SCL_Port, bus->SCL_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(bus->SDA_Port, bus->SDA_Pin, GPIO_PIN_RESET);
    I2C_Delay();
    HAL_GPIO_WritePin(bus->SCL_Port, bus->SCL_Pin, GPIO_PIN_SET);
    I2C_Delay();
    HAL_GPIO_WritePin(bus->SDA_Port, bus->SDA_Pin, GPIO_PIN_SET);
    I2C_Delay();
}

HAL_StatusTypeDef Software_I2C_WaitACK(SoftI2C_Bus *bus) 
{
    uint8_t timeout = 0;
    
    HAL_GPIO_WritePin(bus->SDA_Port, bus->SDA_Pin, GPIO_PIN_SET); // 释放 SDA
    I2C_Delay();
    HAL_GPIO_WritePin(bus->SCL_Port, bus->SCL_Pin, GPIO_PIN_SET);
    I2C_Delay();
    
    while (HAL_GPIO_ReadPin(bus->SDA_Port, bus->SDA_Pin)) 
    {
        timeout++;
        if (timeout > 200) {
            Software_I2C_Stop(bus);
            return HAL_ERROR;
        }
    }
    HAL_GPIO_WritePin(bus->SCL_Port, bus->SCL_Pin, GPIO_PIN_RESET);
    return HAL_OK;
}

void Software_I2C_SendACK(SoftI2C_Bus *bus)
{
    HAL_GPIO_WritePin(bus->SCL_Port, bus->SCL_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(bus->SDA_Port, bus->SDA_Pin, GPIO_PIN_RESET);
    I2C_Delay();
    HAL_GPIO_WritePin(bus->SCL_Port, bus->SCL_Pin, GPIO_PIN_SET);
    I2C_Delay();
    HAL_GPIO_WritePin(bus->SCL_Port, bus->SCL_Pin, GPIO_PIN_RESET);
}

void Software_I2C_SendNACK(SoftI2C_Bus *bus)
{
    HAL_GPIO_WritePin(bus->SCL_Port, bus->SCL_Pin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(bus->SDA_Port, bus->SDA_Pin, GPIO_PIN_SET);
    I2C_Delay();
    HAL_GPIO_WritePin(bus->SCL_Port, bus->SCL_Pin, GPIO_PIN_SET);
    I2C_Delay();
    HAL_GPIO_WritePin(bus->SCL_Port, bus->SCL_Pin, GPIO_PIN_RESET);
}

void Software_I2C_WriteByte(SoftI2C_Bus *bus, uint8_t byte) 
{
    for (uint8_t i = 0; i < 8; i++) 
    {
        HAL_GPIO_WritePin(bus->SCL_Port, bus->SCL_Pin, GPIO_PIN_RESET);
        if (byte & 0x80) 
            HAL_GPIO_WritePin(bus->SDA_Port, bus->SDA_Pin, GPIO_PIN_SET);
        else             
            HAL_GPIO_WritePin(bus->SDA_Port, bus->SDA_Pin, GPIO_PIN_RESET);
        
        byte <<= 1;
        I2C_Delay();
        HAL_GPIO_WritePin(bus->SCL_Port, bus->SCL_Pin, GPIO_PIN_SET);
        I2C_Delay();
        HAL_GPIO_WritePin(bus->SCL_Port, bus->SCL_Pin, GPIO_PIN_RESET);
        I2C_Delay();
    }
}

uint8_t Software_I2C_ReadByte(SoftI2C_Bus *bus)
{
    uint8_t i, val = 0;
    HAL_GPIO_WritePin(bus->SDA_Port, bus->SDA_Pin, GPIO_PIN_SET); // 释放
    for(i = 0; i < 8; i++)
    {
        val <<= 1;
        HAL_GPIO_WritePin(bus->SCL_Port, bus->SCL_Pin, GPIO_PIN_RESET);
        I2C_Delay();
        HAL_GPIO_WritePin(bus->SCL_Port, bus->SCL_Pin, GPIO_PIN_SET);
        I2C_Delay();
        if(HAL_GPIO_ReadPin(bus->SDA_Port, bus->SDA_Pin)) val++;
    }
    HAL_GPIO_WritePin(bus->SCL_Port, bus->SCL_Pin, GPIO_PIN_RESET);
    return val;
}

// 扫描工具
void Software_I2C_Scan(SoftI2C_Bus *bus, UART_HandleTypeDef *huart)
{
    char msg[64];
    HAL_UART_Transmit(huart, (uint8_t*)"\r\n[I2C Scan] Start...\r\n", 23, 100);

    for(uint8_t addr = 1; addr < 127; addr++)
    {
        Software_I2C_Start(bus);
        Software_I2C_WriteByte(bus, addr << 1);
        if(Software_I2C_WaitACK(bus) == HAL_OK)
        {
            sprintf(msg, "-> Found: 0x%02X\r\n", addr << 1);
            HAL_UART_Transmit(huart, (uint8_t*)msg, strlen(msg), 100);
        }
        Software_I2C_Stop(bus);
        for(volatile int k=0; k<500; k++);
    }
    HAL_UART_Transmit(huart, (uint8_t*)"[I2C Scan] End.\r\n", 17, 100);
}

