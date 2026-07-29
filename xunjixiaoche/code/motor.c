#include "motor.h"
#include "pid.h"

uint8_t stop = 0;

volatile uint16_t count1;
volatile uint16_t count22;
volatile uint16_t last_count1;
volatile uint16_t last_count2;
volatile float speed_left;
volatile float speed_right;
float r_speed_left;
float r_speed_right;

static GPIO_PinState drv8701_reverse_level(GPIO_PinState forward_level)
{
    return (forward_level == GPIO_PIN_SET) ? GPIO_PIN_RESET : GPIO_PIN_SET;
}

static void drv8701_set_motor(TIM_HandleTypeDef *pwm_timer,
                              uint32_t pwm_channel,
                              GPIO_TypeDef *ph_port,
                              uint16_t ph_pin,
                              GPIO_PinState forward_level,
                              float speed)
{
    GPIO_PinState ph_level;
    uint32_t pwm;

    if (!isfinite(speed) || (speed == 0.0f))
    {
        __HAL_TIM_SET_COMPARE(pwm_timer, pwm_channel, DRV8701_PWM_OFF);
        HAL_GPIO_WritePin(ph_port, ph_pin, GPIO_PIN_RESET);
        return;
    }

    ph_level = (speed > 0.0f) ? forward_level : drv8701_reverse_level(forward_level);
    /* PWM1 is high for CCR timer counts; ARR + 1 preserves the 0..10000 scale. */
    pwm = (uint32_t)(((speed > 0.0f) ? speed : -speed) *
                     ((float)__HAL_TIM_GET_AUTORELOAD(pwm_timer) + 1.0f) / PWM_MAX);
    HAL_GPIO_WritePin(ph_port, ph_pin, ph_level);
    __HAL_TIM_SET_COMPARE(pwm_timer, pwm_channel, pwm);
}

void Limit(float *motoA, float *motoB)
{
    if (*motoA > PWM_MAX) *motoA = PWM_MAX;
    if (*motoA < PWM_MIN) *motoA = PWM_MIN;
    if (*motoB > PWM_MAX) *motoB = PWM_MAX;
    if (*motoB < PWM_MIN) *motoB = PWM_MIN;
}

void motor_init(void)
{
    HAL_GPIO_WritePin(DRV8701_LEFT_PH_GPIO_PORT,
                      DRV8701_LEFT_PH_GPIO_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(DRV8701_RIGHT_PH_GPIO_PORT,
                      DRV8701_RIGHT_PH_GPIO_PIN, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(DRV8701_LEFT_EN_PWM_TIMER,
                          DRV8701_LEFT_EN_PWM_CHANNEL, DRV8701_PWM_OFF);
    __HAL_TIM_SET_COMPARE(DRV8701_RIGHT_EN_PWM_TIMER,
                          DRV8701_RIGHT_EN_PWM_CHANNEL, DRV8701_PWM_OFF);

    HAL_TIM_PWM_Start(DRV8701_LEFT_EN_PWM_TIMER,
                      DRV8701_LEFT_EN_PWM_CHANNEL);
    HAL_TIM_PWM_Start(DRV8701_RIGHT_EN_PWM_TIMER,
                      DRV8701_RIGHT_EN_PWM_CHANNEL);
    HAL_TIM_Encoder_Start(DRV8701_LEFT_ENCODER_TIMER,
                          DRV8701_ENCODER_CHANNELS);
    HAL_TIM_Encoder_Start(DRV8701_RIGHT_ENCODER_TIMER,
                          DRV8701_ENCODER_CHANNELS);
    __HAL_TIM_SET_COUNTER(DRV8701_LEFT_ENCODER_TIMER, 0);
    __HAL_TIM_SET_COUNTER(DRV8701_RIGHT_ENCODER_TIMER, 0);
    count1 = 0;
    count22 = 0;
    last_count1 = 0;
    last_count2 = 0;
    HAL_TIM_Base_Start_IT(&htim5);
}


float GetMotorSpeed1(void)
{
    return speed_left;
}

float GetMotorSpeed2(void)
{
    return speed_right;
}

void set_speed(float speed_l, float speed_r)
{
    Limit(&speed_l, &speed_r);
    drv8701_set_motor(DRV8701_LEFT_EN_PWM_TIMER,
                      DRV8701_LEFT_EN_PWM_CHANNEL,
                      DRV8701_LEFT_PH_GPIO_PORT,
                      DRV8701_LEFT_PH_GPIO_PIN,
                      DRV8701_LEFT_FORWARD_PH_LEVEL,
                      speed_l);
    drv8701_set_motor(DRV8701_RIGHT_EN_PWM_TIMER,
                      DRV8701_RIGHT_EN_PWM_CHANNEL,
                      DRV8701_RIGHT_PH_GPIO_PORT,
                      DRV8701_RIGHT_PH_GPIO_PIN,
                      DRV8701_RIGHT_FORWARD_PH_LEVEL,
                      speed_r);
}

void Stop_protect(float *Med_Jiaodu, float *Jiaodu)
{
    if (((*Jiaodu - *Med_Jiaodu) > 40.0f) ||
        ((*Jiaodu - *Med_Jiaodu) < -40.0f))
    {
        stop = 1;
    }
    else
    {
        stop = 0;
    }
}
