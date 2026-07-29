#ifndef _PID_H
#define _PID_H

#include "stdint.h"
#include "string.h"
#include "stdio.h"
#include "math.h"
#include "motor.h"
#include "jy901.h"

void control_speed(void);
void PositionPID_Reset(void);
void PID_ResetAll(void);

/*ʵ��*/
float direction(float zhanxiangjiaodu);

extern int16_t position_get;
extern volatile uint8_t star_car;
extern volatile uint16_t base_speed;
extern volatile float Kp_l, Ki_l, Kd_l;
extern volatile float Kp_r, Ki_r, Kd_r;

extern float target_position;
extern float target_gyro_z;
extern float gyro_speed_correction;
extern volatile float Kp_pos, Ki_pos, Kd_pos;
extern volatile float Kp_gyro, Ki_gyro, Kd_gyro;
extern float pos_out_max, pos_out_min;
extern float gyro_out_max, gyro_out_min;

extern float start_track_pos;    // ��¼��ת��/ֱ��ǰ�ĳ�ʼλ�á�����׼ֵ��
extern uint8_t is_turn_90;          // ֱ��ת���־��0=ֱ��/΢����1=ֱ��ת����
extern const float TURN_ANGLE;   // ֱ��ת��Ƕȣ��̶�90�ȣ�
extern const float TURN_TOLERANCE;// ת���������2���ж�ת�꣩

#endif
