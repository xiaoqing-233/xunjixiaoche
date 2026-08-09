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
void PID_RequestSlowStop(void);
void PID_ClearSlowStop(void);
uint8_t PID_IsSlowStopDone(void);
uint8_t PID_IsTargetSpeedScaleStopped(void);
void PID_SetTargetSpeedScale(float ratio);

/*ʵ��*/
float direction(float zhanxiangjiaodu);

extern int16_t position_get;
extern volatile uint8_t star_car;
extern volatile uint8_t k;
extern volatile uint16_t base_speed;
extern volatile float Kp_l, Ki_l, Kd_l;
extern volatile float Kp_r, Ki_r, Kd_r;

extern volatile float target_position;
extern volatile float target_gyro_z;
extern volatile float debug_target_gyro_z;
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
