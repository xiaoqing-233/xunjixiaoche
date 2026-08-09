#ifndef NO_MCU_GANV_GRAYSCALE_SENSOR_CONFIG_H_
#define NO_MCU_GANV_GRAYSCALE_SENSOR_CONFIG_H_

#include "stm32f4xx_hal.h"
#include "delay.h"

#define GW_SENSOR_REVERSE_ORDER 1U

#define GW_ADDR0_Pin GPIO_PIN_1
#define GW_ADDR0_GPIO_Port GPIOE
#define GW_ADDR1_Pin GPIO_PIN_2
#define GW_ADDR1_GPIO_Port GPIOE
#define GW_ADDR2_Pin GPIO_PIN_3
#define GW_ADDR2_GPIO_Port GPIOE
#define GW_CAL_LED_Pin GPIO_PIN_2
#define GW_CAL_LED_GPIO_Port GPIOB
#define GW_CAL_KEY_Pin GPIO_PIN_1
#define GW_CAL_KEY_GPIO_Port GPIOC

extern unsigned char Digtal;

void gw_get_value(void);

#endif
