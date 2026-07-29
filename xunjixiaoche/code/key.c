#include "key.h"

#include "main.h"
#include "motor.h"
#include "pid.h"

#define KEY_DEBOUNCE_TIME_MS 20U

typedef struct
{
    GPIO_TypeDef *port;
    uint16_t pin;
    GPIO_PinState stable_state;
    GPIO_PinState candidate_state;
    uint32_t candidate_since;
} KeyDebounce;

static KeyDebounce keys[] = {
    { USER_KEY1_GPIO_Port, USER_KEY1_Pin, GPIO_PIN_SET, GPIO_PIN_SET, 0U },
    { USER_KEY2_GPIO_Port, USER_KEY2_Pin, GPIO_PIN_SET, GPIO_PIN_SET, 0U },
    { USER_KEY3_GPIO_Port, USER_KEY3_Pin, GPIO_PIN_SET, GPIO_PIN_SET, 0U }
};

static KeyEvent last_event;
static uint8_t keys_initialized;

static void key_handle_pressed(KeyEvent event)
{
    last_event = event;

    if (event == KEY_EVENT_1_PRESSED)
    {
        star_car = (star_car == 0U) ? 1U : 0U;
        PID_ResetAll();
        set_speed(0.0f, 0.0f);
    }
}

KeyEvent key_get_last_event(void)
{
    return last_event;
}

void key_control(void)
{
    uint32_t now = HAL_GetTick();
    uint32_t index;

    if (keys_initialized == 0U)
    {
        for (index = 0U; index < (sizeof(keys) / sizeof(keys[0])); ++index)
        {
            keys[index].stable_state = HAL_GPIO_ReadPin(keys[index].port, keys[index].pin);
            keys[index].candidate_state = keys[index].stable_state;
            keys[index].candidate_since = now;
        }
        keys_initialized = 1U;
        return;
    }

    for (index = 0U; index < (sizeof(keys) / sizeof(keys[0])); ++index)
    {
        KeyDebounce *key = &keys[index];
        GPIO_PinState sample = HAL_GPIO_ReadPin(key->port, key->pin);

        if (sample != key->candidate_state)
        {
            key->candidate_state = sample;
            key->candidate_since = now;
        }
        else if ((sample != key->stable_state)
                 && ((uint32_t)(now - key->candidate_since) >= KEY_DEBOUNCE_TIME_MS))
        {
            key->stable_state = sample;

            if (sample == GPIO_PIN_RESET)
            {
                key_handle_pressed((KeyEvent)(KEY_EVENT_1_PRESSED + index));
            }
        }
    }
}
