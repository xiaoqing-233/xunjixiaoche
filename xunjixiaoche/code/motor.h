#ifndef _MOTOR_H
#define _MOTOR_H

#include "main.h"
#include "stdint.h"
#include "tim.h"
#include "stdio.h"
#include "stdbool.h"
#include "stdlib.h"
#include "math.h"
#include "key.h"

void motor_init(void);
void set_speed(float speed_l,float speed_r);
float GetMotorSpeed1(void);
float GetMotorSpeed2(void);
extern volatile float speed_left, speed_right; 
extern uint8_t stop;
extern float Kp_l,Ki_l,Kp_r,Ki_r;
extern float r_speed_left, r_speed_right;
extern volatile float speed_left, speed_right;

void Limit(float *motoA,float *motoB);

void Stop_protect(float *Med_Jiaodu,float *Jiaodu);


#endif
