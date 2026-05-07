#include "pid.h"


// ==================== 串级PID参数 ====================

// 速度环PID参数
float Kp_l = 2, Ki_l = 0.15, Kp_r = 2, Ki_r = 0.15;
float Kd_l = 0, Kd_r = 0;

uint16_t base_speed = 100;

// 位置环PID参数（需要根据实际调试）
// 注意：位置范围是0-7，中心位置是3.5
// 参考值：Kp=15-25, Ki=0-0.5, Kd=5-15, gyro_kd=50-200
float Kp_pos = 20.0f;      // 位置环Kp（建议初始值：20）
float Ki_pos = 0.0f;        // 位置环Ki（建议初始值：0，调试稳定后可加小量积分）
float Kd_pos = 10.0f;       // 位置环Kd（建议初始值：10）
float gyro_kd = 0.0f;     // 陀螺仪角速度系数（建议初始值：100，范围50-200）

// 位置环限幅
float pos_out_max = 500.0f;   // 位置环输出上限（速度修正量）
float pos_out_min = -500.0f;  // 位置环输出下限

float target_position = 0.0f;

// 位置环变量
float pos_error = 0;
float pos_last_error = 0;
float pos_sum_error = 0;
float pos_output = 0;

// 速度环变量（保留原有）
extern float r_speed_left;
extern float r_speed_right;
extern float Yaw;

int16_t position_get;  // 保留的当前位置变量

float error_l = 0, error_r = 0;
float last_error_l = 0, last_error_r = 0;
float last_last_error_l = 0, last_last_error_r = 0;
float sum_error_l, sum_error_r;
float MOTOl = 0, MOTOr = 0;

// 启动变量
uint8_t star_car;
float statr_speed = 0;
static float first_set = 0;

// ==================== 位置环PID（外环） ====================

/**
 * @brief 位置环计算 - 位置式PID + 陀螺仪角速度补偿
 * @return 位置环输出（偏右为正，左轮减速、右轮加速）
 *
 * 算法说明：
 * 1. 基于位置偏差的PID控制
 * 2. 加入陀螺仪Z轴角速度作为额外的微分项
 * 3. 陀螺仪能提前预测车身旋转，提高响应速度和稳定性
 */
float PositionPID_Calculate(float current_position)
{
    // 误差 = 当前位置 - 目标位置
    // 偏右时 current_position > target_position，error为正
    pos_error = current_position - target_position;

    // 比例项
    float p_term = Kp_pos * pos_error;

    // 积分项
    pos_sum_error += pos_error;
    // 积分限幅
    if (pos_sum_error > pos_out_max * 0.5f) pos_sum_error = pos_out_max * 0.5f;
    if (pos_sum_error < pos_out_min * 0.5f) pos_sum_error = pos_out_min * 0.5f;
    float i_term = Ki_pos * pos_sum_error;

    // 微分项
    float d_term = Kd_pos * (pos_error - pos_last_error);

    // 陀螺仪角速度补偿项（相当于额外的微分）
    // GyrZ > 0: 车身正在右转（逆时针看是正方向）
    // GyrZ < 0: 车身正在左转
    // 减去陀螺仪项可以抑制过度转向
    extern float GyrZ;
    float gyro_term = gyro_kd * GyrZ;

    // 陀螺仪项限幅（防止过度补偿）
    if (gyro_term > pos_out_max * 0.5f) gyro_term = pos_out_max * 0.5f;
    if (gyro_term < pos_out_min * 0.5f) gyro_term = pos_out_min * 0.5f;

    // 输出 = P + I + D - 陀螺仪补偿
    // 减去陀螺仪项的原因：
    // - 当车身右转时（GyrZ > 0），减去正值，减小右转力度
    // - 当车身左转时（GyrZ < 0），减去负值（相当于加），减小左转力度
    pos_output = p_term + i_term + d_term - gyro_term;

    // 输出限幅
    if (pos_output > pos_out_max) pos_output = pos_out_max;
    if (pos_output < pos_out_min) pos_output = pos_out_min;

    pos_last_error = pos_error;

    return pos_output;
}

/**
 * @brief 重置位置环
 */
void PositionPID_Reset(void)
{
    pos_error = 0;
    pos_last_error = 0;
    pos_sum_error = 0;
    pos_output = 0;
}

// ==================== 速度环PID（内环，保留原有） ====================

void control_speed(void)
{

		float current_position;
    float position_correction;
    float left_speed_target, right_speed_target;
    float i_speed;

    // 1. 获取当前位置（从循迹传感器获取，而不是IMU）
    // weighted_value 是从 xunji.c 计算得到的黑线位置（0-7）
    extern float weighted_value;
    extern uint8_t is_line_lost(void);  // 丢线检测函数
    current_position = weighted_value;

    // 2. 位置环计算（外环）
    position_correction = PositionPID_Calculate(current_position);

    // 3. 计算动态目标速度（根据偏差程度降速）
    // 偏差越大，速度越慢，提高转弯稳定性
    // 如果丢线（直角弯），进一步降低速度
    if(is_line_lost())
    {
        // 丢线时降低速度，提高直角弯通过率
        i_speed = base_speed * 0.6f;  // 降低到60%速度
    }
    else
    {
        // 正常循迹，根据偏差动态调速
        i_speed = base_speed - (fmin(fabs(current_position - target_position), 3.5f) / 3.5f) * 50;
    }

    // 4. 启动缓启动逻辑
    if (star_car)
    {
        if (statr_speed < i_speed && first_set == 0)
        {
            statr_speed += 10.5f;
        }
        else
        {
            statr_speed = i_speed;
            first_set = 1;
        }
    }
    else
    {
        // 停车重置
        statr_speed = 0;
        first_set = 0;
        MOTOl = 0;
        MOTOr = 0;
        PositionPID_Reset();

        // 速度环重置
        error_l = error_r = 0;
        last_error_l = last_error_r = 0;
        last_last_error_l = last_last_error_r = 0;

        set_speed(0, 0);
        return;
    }

    // 5. 叠加位置环偏差到速度目标
    // position_correction > 0: 黑线在右边，需要右转，左轮加速，右轮减速
    // position_correction < 0: 黑线在左边，需要左转，左轮减速，右轮加速
    left_speed_target = statr_speed + position_correction;
    right_speed_target = statr_speed - position_correction;

    // 6. 速度环计算（内环，增量式PID）
    r_speed_left = left_speed_target;
    r_speed_right = right_speed_target;

    error_l = r_speed_left - speed_left;
    error_r = r_speed_right - speed_right;

    MOTOl += Kp_l * (error_l - last_error_l) + Ki_l * error_l + Kd_l * (error_l - 2 * last_error_l + last_last_error_l);
    MOTOr += Kp_r * (error_r - last_error_r) + Ki_r * error_r + Kd_r * (error_r - 2 * last_error_r + last_last_error_r);

    // 7. 限幅输出
    Limit(&MOTOl, &MOTOr);
    set_speed(MOTOl, MOTOr);

    // 8. 保存历史误差
    last_last_error_l = last_error_l;
    last_last_error_r = last_error_r;
    last_error_l = error_l;
    last_error_r = error_r;
}
