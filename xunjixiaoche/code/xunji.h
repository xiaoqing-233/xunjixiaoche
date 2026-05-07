#ifndef _TRACK_H
#define _TRACK_H

#include "stdint.h"
#include "stdbool.h"
#include "ganweixunji.h"
#include "pid.h"

// 全局变量声明
extern float weighted_value;        // 黑线位置（0-7）
extern unsigned char sensor[8];     // 8路传感器数据数组
extern uint8_t line_lost;           // 丢线标志
extern uint8_t black_count;         // 检测到黑线的传感器数量
extern float last_valid_position;   // 上次有效位置

// 循迹主函数
void track_line(void);

// 获取丢线状态
uint8_t is_line_lost(void);

// 获取历史平均位置（用于调试）
float get_average_position(void);

#endif
