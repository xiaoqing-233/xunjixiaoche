#ifndef JUSTFLOAT_H
#define JUSTFLOAT_H

#include "stm32f4xx_hal.h"

#define JUSTFLOAT_MAX_VALUES (16U)

HAL_StatusTypeDef justfloat_send_dma(const float *data, uint16_t count);
void justfloat_on_tx_complete(UART_HandleTypeDef *huart);
void justfloat_on_uart_error(UART_HandleTypeDef *huart);

#endif
