#ifndef _PID_H
#define _PID_H

#include "stdint.h"
#include "string.h"
#include "stdio.h"
#include "math.h"
#include "motor.h"
#include "jy901.h"
#include "pid.h"

void control_speed(void);

/*实验*/
float direction(float zhanxiangjiaodu);

extern int16_t position_get;
extern uint8_t star_car;

extern float target_position;

extern float start_track_pos;    // 记录【转弯/直行前的初始位置】（基准值）
extern uint8_t is_turn_90;          // 直角转弯标志：0=直行/微调，1=直角转弯中
extern const float TURN_ANGLE;   // 直角转弯角度（固定90度）
extern const float TURN_TOLERANCE;// 转弯完成误差（±2度判定转完）

#endif
