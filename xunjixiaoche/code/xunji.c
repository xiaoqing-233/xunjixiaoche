#include "xunji.h"
#include "motor.h"

extern unsigned char Digtal;
unsigned char sensor[8];
uint8_t black_count = 0;

// 循迹位置相关变量
float weighted_value = 0.0f;           // 加权位置值（0-7）
float last_valid_position = 3.5f;  // 上次有效位置（初始化为中心）
uint8_t line_lost = 1;            // 丢线标志：0=正常，1=丢线

// 位置历史记录（用于更可靠的方向判断）
#define POSITION_HISTORY_SIZE 5
static float position_history[POSITION_HISTORY_SIZE] = {3.5f, 3.5f, 3.5f, 3.5f, 3.5f};
static uint8_t history_index = 0;

// 传感器位置权重（0-7，对应8个传感器）
static const float sensor_positions[8] = {0.0f, 1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f};

// 目标位置（传感器阵列中心）
#define TARGET_CENTER_POSITION 3.5f

/**
 * @brief 读取8路灰度传感器数据并转换为数组
 */
static void track_read_sensors(void)
{
    for(int i = 0; i < 8; i++)
    {
        // 0 = 检测到黑线，1 = 白色背景
        sensor[i] = ((Digtal >> i) & 1) ? 1 : 0;
    }
}

/**
 * @brief 计算位置历史的平均值
 * @return 历史位置的平均值
 */
float get_average_position(void)
{
    float sum = 0.0f;
    for(int i = 0; i < POSITION_HISTORY_SIZE; i++)
    {
        sum += position_history[i];
    }
    return sum / POSITION_HISTORY_SIZE;
}

/**
 * @brief 计算黑线的加权位置
 * @return 位置值（0-7的浮点数），如果未检测到黑线则返回上次有效位置
 *
 * 算法说明：
 * 1. 遍历8个传感器，找到所有检测到黑线的传感器（sensor[i] == 0）
 * 2. 使用加权平均法计算黑线位置：
 *    position = Σ(sensor_position[i] * weight[i]) / Σ(weight[i])
 * 3. 权重使用反向值：检测到黑线的传感器权重为1，其他为0
 * 4. 使用位置历史记录，避免误判
 */
static float calculate_line_position(void)
{
    float weighted_sum = 0.0f;
    float weight_total = 0.0f;
    black_count = 0;
		line_lost=1;
    // 计算加权位置
    for(int i = 0; i < 8; i++)
    {
        if(sensor[i] == 0)  // 检测到黑线
        {
            weighted_sum += sensor_positions[i];
            weight_total += 1.0f;
            black_count++;
					  line_lost = 0;  // 只要有一个黑线就不丢线
        }
    }
		
		if(sensor[0] == 1 && sensor[1] == 1 && sensor[2] == 1 && sensor[3] == 1 && sensor[4] == 1 && sensor[5] == 1 && sensor[6] == 1 && sensor[7] == 1)
		{
				line_lost = 1;
		}
    // 如果检测到黑线，计算加权平均位置
    if(line_lost==0)
    {
        float position = weighted_sum / weight_total;
        // 【关键】只有真正检测到黑线时才更新历史记录
        position_history[history_index] = position;
        history_index = (history_index + 1) % POSITION_HISTORY_SIZE;
        // 更新最后有效位置
        last_valid_position = position;
        return position;
    }
    else
    {
				float avg_position = get_average_position();
        // 根据历史平均位置判断应该往哪边转
        if(avg_position < 3.5f) // 历史明确在左边
        {
            // 返回一个左边的值（不是极值0，而是1.0）
					  last_valid_position=0.0f;
            return 0.0f;  // 产生 1.0-3.5=-2.5 的偏差，足够左转
        }
        else if(avg_position > 3.5f)  // 历0在右边
        {
            // 返回一个右边的值（不是极值7，而是6.0）
					  last_valid_position=7.0f;
            return 7.0f;  // 产生 6.0-3.5=2.5 的偏差，足够右转
        }
        else  if(avg_position == 3.5f)
        {
					  last_valid_position=3.5f;
					  return 3.5f;
        }
    }
}

/**
 * @brief 循迹主函数
 *
 * 功能：
 * 1. 读取8路灰度传感器数据
 * 2. 计算黑线的精确位置（0-7）
 * 3. 设置目标位置为中心位置（3.5）
 * 4. PID控制器会根据 current_position 和 target_position 的偏差进行控制
 *
 * 位置定义：
 * - 传感器0在最左边（位置0.0）
 * - 传感器7在最右边（位置7.0）
 * - 中心位置为3.5
 * - 当前位置 < 3.5：黑线在左边，需要左转
 * - 当前位置 > 3.5：黑线在右边，需要右转
 * - 当前位置 = 3.5：黑线在中心，直行
 *
 * 丢线处理：
 * - 当所有传感器都检测不到黑线时（直角弯）
 * - 根据历史位置的平均值判断方向（更可靠）
 * - 返回适度偏离的值（1.0或6.0），避免突变
 */
void track_line(void)
{
    track_read_sensors();                      // 1. 读取传感器数据
    weighted_value = calculate_line_position();// 2. 计算黑线位置（0-7），如果丢线会返回1.0或6.0

    // 3. 设置目标位置为中心（这个值会被PID控制器使用）
    // 注意：target_position 是全局变量，在 pid.c 中使用
    target_position = TARGET_CENTER_POSITION;
	
    // 4. PID控制器会在 control_speed() 中使用：
    //    - current_position = weighted_value
    //    - error = current_position - target_position
    //    - 如果 weighted_value > 3.5，error > 0，需要右转（右轮减速）
    //    - 如果 weighted_value < 3.5，error < 0，需要左转（左轮减速）
    //    - 如果丢线，weighted_value = 1.0 或 6.0，产生适度偏差，持续转向
}

/**
 * @brief 获取丢线状态
 * @return 0=正常，1=丢线
 */
uint8_t is_line_lost(void)
{
    return line_lost;
}
