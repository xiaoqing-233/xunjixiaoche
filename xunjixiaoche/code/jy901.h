#ifndef __JIAODU_H
#define __JIAODU_H

#include "main.h" 

void jy901_ReceiveData(uint8_t RxData);
extern uint8_t g_uart2_receivedata;
extern float Roll,Pitch,Yaw;
extern float GyrX, GyrY, GyrZ;/*角速度信息*/

void JY901S_ZeroCalibration(void);

#endif
