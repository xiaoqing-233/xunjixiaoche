#ifndef OLED_I2C_H
#define OLED_I2C_H

#include <stdint.h>

#include "stm32f4xx_hal.h"

#define OLED_I2C_WIDTH 128U
#define OLED_I2C_HEIGHT 64U
#define OLED_I2C_ADDRESS 0x3CU
#define OLED_I2C_ALTERNATE_ADDRESS 0x3DU

HAL_StatusTypeDef oled_i2c_init(void);
HAL_StatusTypeDef oled_i2c_clear(void);
HAL_StatusTypeDef oled_i2c_refresh(void);
HAL_StatusTypeDef oled_i2c_display_on(void);
HAL_StatusTypeDef oled_i2c_display_off(void);
HAL_StatusTypeDef oled_i2c_set_inverse(uint8_t enabled);
HAL_StatusTypeDef oled_i2c_set_rotation(uint8_t rotated_180);

void oled_i2c_draw_point(uint8_t x, uint8_t y, uint8_t color);
void oled_i2c_clear_buffer(void);
void oled_i2c_draw_char(uint8_t x, uint8_t y, char character, uint8_t size, uint8_t color);
void oled_i2c_draw_string(uint8_t x, uint8_t y, const char *text, uint8_t size, uint8_t color);
void oled_i2c_draw_uint(uint8_t x, uint8_t y, uint32_t value, uint8_t digits, uint8_t size, uint8_t color);
void oled_i2c_draw_int(uint8_t x, uint8_t y, int32_t value, uint8_t digits, uint8_t size, uint8_t color);

#endif /* OLED_I2C_H */
