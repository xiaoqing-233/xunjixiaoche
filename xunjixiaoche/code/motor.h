#ifndef _MOTOR_H
#define _MOTOR_H

#include "main.h"
#include "stdint.h"
#include "tim.h"
#include "stdio.h"
#include "stdbool.h"
#include "stdlib.h"
#include "math.h"

#define DRV8701_PWM_COMMAND_MAX              10000.0f
#define PWM_MAX                              DRV8701_PWM_COMMAND_MAX
#define PWM_MIN                              (-PWM_MAX)

/* DRV8701 uses one enable/PWM input and one phase/direction input per motor. */
#define DRV8701_PWM_OFF                     0U
#define DRV8701_ENCODER_CHANNELS            TIM_CHANNEL_ALL

#define DRV8701_LEFT_EN_PWM_TIMER           (&htim12)
#define DRV8701_LEFT_EN_PWM_CHANNEL         TIM_CHANNEL_2
#define DRV8701_LEFT_EN_PWM_PORT            GPIOB
#define DRV8701_LEFT_EN_PWM_PIN             GPIO_PIN_15
#define DRV8701_LEFT_PH_GPIO_PORT           GPIOE
#define DRV8701_LEFT_PH_GPIO_PIN            GPIO_PIN_0
#define DRV8701_LEFT_FORWARD_PH_LEVEL       GPIO_PIN_RESET
#define DRV8701_LEFT_ENCODER_TIMER          (&htim4)

#define DRV8701_RIGHT_EN_PWM_TIMER          (&htim12)
#define DRV8701_RIGHT_EN_PWM_CHANNEL        TIM_CHANNEL_1
#define DRV8701_RIGHT_EN_PWM_PORT           GPIOB
#define DRV8701_RIGHT_EN_PWM_PIN            GPIO_PIN_14
#define DRV8701_RIGHT_PH_GPIO_PORT          GPIOB
#define DRV8701_RIGHT_PH_GPIO_PIN           GPIO_PIN_5
#define DRV8701_RIGHT_FORWARD_PH_LEVEL      GPIO_PIN_RESET
#define DRV8701_RIGHT_ENCODER_TIMER         (&htim3)

void motor_init(void);
void set_speed(float speed_l,float speed_r);
float GetMotorSpeed1(void);
float GetMotorSpeed2(void);
extern volatile float speed_left, speed_right; 
extern uint8_t stop;
extern volatile float Kp_l,Ki_l,Kp_r,Ki_r;
extern float r_speed_left, r_speed_right;

extern volatile uint16_t count1;
extern volatile uint16_t count22;
extern volatile uint16_t last_count1;
extern volatile uint16_t last_count2;





void Limit(float *motoA,float *motoB);

void Stop_protect(float *Med_Jiaodu,float *Jiaodu);


#endif
