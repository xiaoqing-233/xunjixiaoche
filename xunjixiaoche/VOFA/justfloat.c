#include "justfloat.h"

#include <string.h>

#include "usart.h"

#define JUSTFLOAT_TAIL_SIZE (4U)
#define JUSTFLOAT_BUFFER_SIZE ((JUSTFLOAT_MAX_VALUES * sizeof(float)) + JUSTFLOAT_TAIL_SIZE)

static uint8_t justfloat_buffer[JUSTFLOAT_BUFFER_SIZE];
static volatile uint8_t justfloat_tx_busy;

HAL_StatusTypeDef justfloat_send_dma(const float *data, uint16_t count)
{
    uint16_t index;
    uint16_t offset = 0U;
    uint32_t primask;
    HAL_StatusTypeDef status;

    if ((data == NULL) || (count == 0U) || (count > JUSTFLOAT_MAX_VALUES))
    {
        return HAL_ERROR;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    if (justfloat_tx_busy != 0U)
    {
        if (primask == 0U)
        {
            __enable_irq();
        }
        return HAL_BUSY;
    }

    justfloat_tx_busy = 1U;
    for (index = 0U; index < count; ++index)
    {
        uint32_t encoded;

        memcpy(&encoded, &data[index], sizeof(encoded));
        justfloat_buffer[offset++] = (uint8_t)(encoded >> 0U);
        justfloat_buffer[offset++] = (uint8_t)(encoded >> 8U);
        justfloat_buffer[offset++] = (uint8_t)(encoded >> 16U);
        justfloat_buffer[offset++] = (uint8_t)(encoded >> 24U);
    }

    justfloat_buffer[offset++] = 0x00U;
    justfloat_buffer[offset++] = 0x00U;
    justfloat_buffer[offset++] = 0x80U;
    justfloat_buffer[offset++] = 0x7FU;

    status = HAL_UART_Transmit_DMA(&huart1, justfloat_buffer, offset);
    if (status != HAL_OK)
    {
        justfloat_tx_busy = 0U;
    }
    if (primask == 0U)
    {
        __enable_irq();
    }

    return status;
}

void justfloat_on_tx_complete(UART_HandleTypeDef *huart)
{
    if (huart == &huart1)
    {
        justfloat_tx_busy = 0U;
    }
}

void justfloat_on_uart_error(UART_HandleTypeDef *huart)
{
    if ((huart == &huart1)
        && ((HAL_UART_GetState(huart) & HAL_UART_STATE_BUSY_TX) == 0U))
    {
        justfloat_tx_busy = 0U;
    }
}
