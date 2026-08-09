/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define GW_ADC_OUT_Pin GPIO_PIN_0
#define GW_ADC_OUT_GPIO_Port GPIOA
#define MOTOR_RIGHT_PH_Pin GPIO_PIN_5
#define MOTOR_RIGHT_PH_GPIO_Port GPIOB
#define GW_CAL_LED_Pin GPIO_PIN_2
#define GW_CAL_LED_GPIO_Port GPIOB
#define OLED_SOFT_SDA_Pin GPIO_PIN_8
#define OLED_SOFT_SDA_GPIO_Port GPIOB
#define OLED_SOFT_SCL_Pin GPIO_PIN_9
#define OLED_SOFT_SCL_GPIO_Port GPIOB
#define MOTOR_LEFT_PH_Pin GPIO_PIN_0
#define MOTOR_LEFT_PH_GPIO_Port GPIOE
#define GW_CAL_KEY_Pin GPIO_PIN_1
#define GW_CAL_KEY_GPIO_Port GPIOC
#define USER_KEY1_Pin GPIO_PIN_1
#define USER_KEY1_GPIO_Port GPIOC
#define USER_KEY2_Pin GPIO_PIN_2
#define USER_KEY2_GPIO_Port GPIOC
#define USER_KEY3_Pin GPIO_PIN_3
#define USER_KEY3_GPIO_Port GPIOC
#define GW_ADDR0_Pin GPIO_PIN_1
#define GW_ADDR0_GPIO_Port GPIOE
#define GW_ADDR1_Pin GPIO_PIN_2
#define GW_ADDR1_GPIO_Port GPIOE
#define GW_ADDR2_Pin GPIO_PIN_3
#define GW_ADDR2_GPIO_Port GPIOE

/* USER CODE BEGIN Private defines */

#define CAR_STATE_TOGGLE_Pin GPIO_PIN_1
#define CAR_STATE_TOGGLE_GPIO_Port GPIOB

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
