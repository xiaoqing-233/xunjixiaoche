#include "vofa.h"

#include <string.h>
#include <stdint.h>

#include "pid.h"
#include "usart.h"

static uint8_t vofa_rx_byte;
static uint8_t vofa_rx_frame[VOFA_RX_FRAME_SIZE];
static uint8_t vofa_pending_frame[VOFA_RX_FRAME_SIZE];
static volatile uint8_t vofa_rx_length;
static volatile uint8_t vofa_pending_length;
static volatile uint8_t vofa_frame_pending;

static uint8_t vofa_parse_value(const char *text, float *value);
static void vofa_process_frame(const uint8_t *frame);
static uint8_t vofa_apply_command(const char *command, float value);

void vofa_init(void)
{
    memset(vofa_rx_frame, 0, sizeof(vofa_rx_frame));
    memset(vofa_pending_frame, 0, sizeof(vofa_pending_frame));
    vofa_rx_length = 0U;
    vofa_pending_length = 0U;
    vofa_frame_pending = 0U;
    (void)HAL_UART_Receive_IT(&huart1, &vofa_rx_byte, 1U);
}

const uint8_t *vofa_rx_buffer(void)
{
    return &vofa_rx_byte;
}

void vofa_receive_byte(uint8_t data)
{
    uint8_t index;

    if ((data == '!') || (data == '\r') || (data == '\n'))
    {
        if ((vofa_rx_length != 0U) && (vofa_frame_pending == 0U))
        {
            for (index = 0U; index < vofa_rx_length; ++index)
            {
                vofa_pending_frame[index] = vofa_rx_frame[index];
            }
            vofa_pending_frame[vofa_rx_length] = '\0';
            vofa_pending_length = vofa_rx_length;
            vofa_frame_pending = 1U;
        }
        vofa_rx_length = 0U;
    }
    else if (vofa_rx_length < (VOFA_RX_FRAME_SIZE - 1U))
    {
        vofa_rx_frame[vofa_rx_length++] = data;
    }
    else
    {
        vofa_rx_length = 0U;
    }

    (void)HAL_UART_Receive_IT(&huart1, &vofa_rx_byte, 1U);
}

void vofa_on_uart_error(UART_HandleTypeDef *huart)
{
    if (huart == &huart1)
    {
        (void)HAL_UART_Receive_IT(&huart1, &vofa_rx_byte, 1U);
    }
}

void vofa_task(void)
{
    uint8_t frame[VOFA_RX_FRAME_SIZE];
    uint8_t length;
    uint8_t index;
    uint32_t primask;

    if (vofa_frame_pending == 0U)
    {
        return;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    if (vofa_frame_pending == 0U)
    {
        if (primask == 0U)
        {
            __enable_irq();
        }
        return;
    }

    length = vofa_pending_length;
    for (index = 0U; index <= length; ++index)
    {
        frame[index] = vofa_pending_frame[index];
    }
    vofa_frame_pending = 0U;
    if (primask == 0U)
    {
        __enable_irq();
    }

    vofa_process_frame(frame);
}

static void vofa_process_frame(const uint8_t *frame)
{
    char command[VOFA_COMMAND_SIZE] = {0};
    const char *cursor = (const char *)frame;
    uint8_t command_length = 0U;
    float value;

    while ((*cursor == ' ') || (*cursor == '\t'))
    {
        ++cursor;
    }
    while ((*cursor != '=') && (*cursor != '\0'))
    {
        if ((*cursor != ' ') && (*cursor != '\t'))
        {
            if (command_length >= (VOFA_COMMAND_SIZE - 1U))
            {
                return;
            }
            command[command_length++] = *cursor;
        }
        ++cursor;
    }

    if ((*cursor != '=') || (command_length == 0U) || !vofa_parse_value(cursor + 1, &value))
    {
        return;
    }

    (void)vofa_apply_command(command, value);
}

static uint8_t vofa_parse_value(const char *text, float *value)
{
    uint32_t whole = 0U;
    uint32_t fraction = 0U;
    uint32_t scale = 1U;
    uint8_t negative = 0U;
    uint8_t decimal = 0U;
    uint8_t digits = 0U;

    while ((*text == ' ') || (*text == '\t'))
    {
        ++text;
    }
    if ((*text == '-') || (*text == '+'))
    {
        negative = (*text == '-') ? 1U : 0U;
        ++text;
    }
    while (*text != '\0')
    {
        if (*text == '.')
        {
            if (decimal != 0U)
            {
                return 0U;
            }
            decimal = 1U;
        }
        else if ((*text >= '0') && (*text <= '9'))
        {
            uint32_t digit = (uint32_t)(*text - '0');

            digits = 1U;
            if (decimal != 0U)
            {
                if ((fraction > ((UINT32_MAX - digit) / 10U)) ||
                    (scale > (UINT32_MAX / 10U)))
                {
                    return 0U;
                }
                fraction = (fraction * 10U) + digit;
                scale *= 10U;
            }
            else
            {
                if (whole > ((UINT32_MAX - digit) / 10U))
                {
                    return 0U;
                }
                whole = (whole * 10U) + digit;
            }
        }
        else if ((*text == ' ') || (*text == '\t'))
        {
            break;
        }
        else
        {
            return 0U;
        }
        ++text;
    }

    if (digits == 0U)
    {
        return 0U;
    }
    *value = (float)whole + ((float)fraction / (float)scale);
    if (negative != 0U)
    {
        *value = -*value;
    }
    return 1U;
}

static uint8_t vofa_apply_command(const char *command, float value)
{
    uint8_t reset_required = 0U;
    uint32_t primask = __get_PRIMASK();

    __disable_irq();
    if (strcmp(command, "V") == 0)
    {
        if ((value < 0.0f) || (value > PWM_MAX))
        {
            if (primask == 0U) { __enable_irq(); }
            return 0U;
        }
        base_speed = (uint16_t)value;
        reset_required = 1U;
    }
    else if (strcmp(command, "SW") == 0)
    {
        star_car = (value != 0.0f) ? 1U : 0U;
        reset_required = 1U;
    }
    else if ((strcmp(command, "P1") == 0) || (strcmp(command, "P2") == 0) ||
             (strcmp(command, "P3") == 0))
    {
        if (command[1] == '1') { Kp_l = value; Kp_r = value; }
        else if (command[1] == '2') { Kp_gyro = value; }
        else { Kp_pos = value; }
        reset_required = 1U;
    }
    else if ((strcmp(command, "I1") == 0) || (strcmp(command, "I2") == 0) ||
             (strcmp(command, "I3") == 0))
    {
        if (command[1] == '1') { Ki_l = value; Ki_r = value; }
        else if (command[1] == '2') { Ki_gyro = value; }
        else { Ki_pos = value; }
        reset_required = 1U;
    }
    else if ((strcmp(command, "D1") == 0) || (strcmp(command, "D2") == 0) ||
             (strcmp(command, "D3") == 0))
    {
        if (command[1] == '1') { Kd_l = value; Kd_r = value; }
        else if (command[1] == '2') { Kd_gyro = value; }
        else { Kd_pos = value; }
        reset_required = 1U;
    }
    else
    {
        if (primask == 0U) { __enable_irq(); }
        return 0U;
    }

    if (reset_required != 0U)
    {
        PID_ResetAll();
    }
    if (primask == 0U)
    {
        __enable_irq();
    }
    return 1U;
}
