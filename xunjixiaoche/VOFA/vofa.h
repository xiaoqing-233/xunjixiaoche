#ifndef VOFA_H
#define VOFA_H

#include <stdint.h>

#include "stm32f4xx_hal.h"

#define VOFA_RX_FRAME_SIZE (64U)
#define VOFA_COMMAND_SIZE  (12U)

void vofa_init(void);
const uint8_t *vofa_rx_buffer(void);
void vofa_receive_byte(uint8_t data);
void vofa_on_uart_error(UART_HandleTypeDef *huart);
void vofa_task(void);

#endif
