#ifndef __JIAODU_H
#define __JIAODU_H

#include "main.h" 

/**
  * @brief 逐字节解析 HWT101 串口输出数据。
  * @param RxData USART2 中断收到的单字节数据。
  */
void jy901_ReceiveData(uint8_t RxData);

extern uint8_t g_uart2_receivedata;
extern float Roll,Pitch,Yaw;
extern float AccX, AccY, AccZ;
extern float GyrX, GyrY, GyrZ;

/**
  * @brief 复位 HWT101 接收解析状态，旧函数名仅用于兼容现有工程调用。
  */
void JY901S_ZeroCalibration(void);

/**
  * @brief 请求开启 HWT101 陀螺仪零偏采集。
  * @return 1=已接受请求，0=已有等待发送的零偏控制命令。
  */
uint8_t HWT101_StartGyroBiasCalibration(void);

/**
  * @brief 请求停止 HWT101 陀螺仪零偏采集并锁定当前零偏参数。
  * @return 1=已接受请求，0=已有等待发送的零偏控制命令。
  */
uint8_t HWT101_StopGyroBiasCalibration(void);

/**
  * @brief 按当前零偏采集状态切换开启/停止命令，用于用户按键手动管控。
  * @return 1=已接受请求，0=已有等待发送的零偏控制命令。
  */
uint8_t HWT101_ToggleGyroBiasCalibration(void);

/**
  * @brief 发送 HWT101 手动零偏控制命令，主循环周期性调用。
  */
void HWT101_Task(void);

#endif
