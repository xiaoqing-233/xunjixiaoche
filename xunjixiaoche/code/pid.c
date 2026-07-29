#include "pid.h"

#define LINE_CENTER_POSITION 3.5f

/* Speed-loop gains use the same 0..10000 command scale as set_speed(). */


//速度环
volatile float Kp_l = 1.0f, Ki_l = 0.8f;
volatile float Kp_r = 1.0f, Ki_r = 0.8f;

volatile float Kd_l = 0.0f, Kd_r = 0.0f;

volatile uint16_t base_speed = 0;

volatile float Kp_pos = 25.0f;
volatile float Ki_pos = 0.0f;
volatile float Kd_pos = 8.0f;
volatile float Kp_gyro = 2.0f;
volatile float Ki_gyro = 0.0f;
volatile float Kd_gyro = 0.2f;

float pos_out_max = 120.0f;
float pos_out_min = -120.0f;
float gyro_out_max = 250.0f;
float gyro_out_min = -250.0f;

float target_position = 0.0f;
float target_gyro_z = 0.0f;
float gyro_speed_correction = 0.0f;

float pos_error = 0.0f;
float pos_last_error = 0.0f;
float pos_sum_error = 0.0f;
float pos_output = 0.0f;

static float gyro_error = 0.0f;
static float gyro_last_error = 0.0f;
static float gyro_sum_error = 0.0f;

extern float r_speed_left;
extern float r_speed_right;

int16_t position_get;

float error_l = 0.0f, error_r = 0.0f;
float last_error_l = 0.0f, last_error_r = 0.0f;
float last_last_error_l = 0.0f, last_last_error_r = 0.0f;
float sum_error_l = 0.0f, sum_error_r = 0.0f;
float MOTOl = 0.0f, MOTOr = 0.0f;

volatile uint8_t star_car;
float statr_speed = 0.0f;
static float first_set = 0.0f;

static float pid_limit(float value, float maximum, float minimum)
{
    if (value > maximum)
    {
        return maximum;
    }
    if (value < minimum)
    {
        return minimum;
    }
    return value;
}

static float position_pid_calculate(float error)
{
    float output;

    pos_error = error;
    pos_sum_error = pid_limit(pos_sum_error + pos_error,
                              pos_out_max * 0.5f,
                              pos_out_min * 0.5f);
    output = (Kp_pos * pos_error) + (Ki_pos * pos_sum_error)
             + (Kd_pos * (pos_error - pos_last_error));
    pos_output = pid_limit(output, pos_out_max, pos_out_min);
    pos_last_error = pos_error;

    return pos_output;
}

static float gyro_pid_calculate(float error)
{
    float output;

    gyro_error = error;
    gyro_sum_error = pid_limit(gyro_sum_error + gyro_error,
                               gyro_out_max * 0.5f,
                               gyro_out_min * 0.5f);
    output = (Kp_gyro * gyro_error) + (Ki_gyro * gyro_sum_error)
             + (Kd_gyro * (gyro_error - gyro_last_error));
    gyro_speed_correction = pid_limit(output, gyro_out_max, gyro_out_min);
    gyro_last_error = gyro_error;

    return gyro_speed_correction;
}

void PositionPID_Reset(void)
{
    pos_error = 0.0f;
    pos_last_error = 0.0f;
    pos_sum_error = 0.0f;
    pos_output = 0.0f;
    target_gyro_z = 0.0f;

    gyro_error = 0.0f;
    gyro_last_error = 0.0f;
    gyro_sum_error = 0.0f;
    gyro_speed_correction = 0.0f;
}

static void speed_pid_reset(void)
{
    error_l = 0.0f;
    error_r = 0.0f;
    last_error_l = 0.0f;
    last_error_r = 0.0f;
    last_last_error_l = 0.0f;
    last_last_error_r = 0.0f;
    sum_error_l = 0.0f;
    sum_error_r = 0.0f;
    MOTOl = 0.0f;
    MOTOr = 0.0f;
}

void PID_ResetAll(void)
{
    PositionPID_Reset();
    speed_pid_reset();
}

void control_speed(void)
{
    float line_offset;
    float i_speed;
    float left_speed_target;
    float right_speed_target;

    extern float weighted_value;
    extern uint8_t is_line_lost(void);

    if (!star_car)
    {
        statr_speed = 0.0f;
        first_set = 0.0f;
        PositionPID_Reset();
        speed_pid_reset();
        set_speed(0.0f, 0.0f);
        return;
    }

    line_offset = weighted_value - LINE_CENTER_POSITION;
    /* Positive GyrZ is a left turn, while a positive line offset requires a right turn. */
    target_gyro_z = position_pid_calculate(target_position - line_offset);
    gyro_speed_correction = gyro_pid_calculate(target_gyro_z - GyrZ);

    if (is_line_lost())
    {
        i_speed = base_speed * 0.6f;
    }
    else
    {
        i_speed = base_speed - (fabsf(line_offset) / LINE_CENTER_POSITION) * 50.0f;
    }

    if ((statr_speed < i_speed) && (first_set == 0.0f))
    {
        statr_speed += 10.5f;
    }
    else
    {
        statr_speed = i_speed;
        first_set = 1.0f;
    }

    /* Positive correction retains the existing left-fast/right-slow turn mapping. */
    left_speed_target = statr_speed + gyro_speed_correction;
    right_speed_target = statr_speed - gyro_speed_correction;

    r_speed_left = left_speed_target;
    r_speed_right = right_speed_target;

    error_l = r_speed_left - speed_left;
    error_r = r_speed_right - speed_right;

    MOTOl += Kp_l * (error_l - last_error_l) + Ki_l * error_l
             + Kd_l * (error_l - (2.0f * last_error_l) + last_last_error_l);
    MOTOr += Kp_r * (error_r - last_error_r) + Ki_r * error_r
             + Kd_r * (error_r - (2.0f * last_error_r) + last_last_error_r);

    Limit(&MOTOl, &MOTOr);
    set_speed(MOTOl, -MOTOr);

    last_last_error_l = last_error_l;
    last_last_error_r = last_error_r;
    last_error_l = error_l;
    last_error_r = error_r;
}








//中断回调，pid计算
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM5)
    {
        uint16_t current_count1;
        uint16_t current_count2;
        int16_t delta_count1;
        int16_t delta_count2;

        /* Preserve the established one-period feedback latency. */
        control_speed(); ///pid计算

        current_count1 = (uint16_t)__HAL_TIM_GET_COUNTER(DRV8701_LEFT_ENCODER_TIMER);
        current_count2 = (uint16_t)__HAL_TIM_GET_COUNTER(DRV8701_RIGHT_ENCODER_TIMER);
        /* Unsigned subtraction intentionally wraps at the 16-bit timer period. */
        delta_count1 = (int16_t)(current_count1 - last_count1);
        delta_count2 = (int16_t)(current_count2 - last_count2);
        count1 = current_count1;
        count22 = current_count2;
        last_count1 = current_count1;
        last_count2 = current_count2;

        speed_left = -(float)delta_count1;
        speed_right = -(float)delta_count2;
    }
}
