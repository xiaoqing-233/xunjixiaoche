#include "jy901.h"

#include "usart.h"

#define HWT101_FRAME_HEAD              0x55U
#define HWT101_FRAME_LENGTH            11U
#define HWT101_FRAME_ACC               0x51U
#define HWT101_FRAME_GYRO              0x52U
#define HWT101_FRAME_ANGLE             0x53U
#define HWT101_CMD_LENGTH              5U
#define HWT101_CMD_TIMEOUT_MS          2U
#define HWT101_CMD_HEAD_1              0xFFU
#define HWT101_CMD_HEAD_2              0xAAU
#define HWT101_REG_MANUAL_CALI         0xA6U
#define HWT101_GYRO_BIAS_START_LOW     0x01U
#define HWT101_GYRO_BIAS_START_HIGH    0x00U
#define HWT101_GYRO_BIAS_STOP_LOW      0x04U
#define HWT101_GYRO_BIAS_STOP_HIGH     0x00U

typedef enum {
    HWT101_BIAS_CMD_NONE = 0,
    HWT101_BIAS_CMD_START,
    HWT101_BIAS_CMD_STOP
} Hwt101BiasCommand;

static uint8_t hwt101_rx_buffer[HWT101_FRAME_LENGTH];
static uint8_t hwt101_rx_index;
static Hwt101BiasCommand hwt101_bias_pending_cmd = HWT101_BIAS_CMD_NONE;
static uint8_t hwt101_bias_collecting;

float Roll = 0.0f;
float Pitch = 0.0f;
float Yaw = 0.0f;
float AccX = 0.0f;
float AccY = 0.0f;
float AccZ = 0.0f;
float GyrX = 0.0f;
float GyrY = 0.0f;
float GyrZ = 0.0f;
uint8_t g_uart2_receivedata = 0U;

static uint8_t hwt101_is_data_frame(uint8_t frame_type)
{
    return ((frame_type == HWT101_FRAME_ACC) ||
            (frame_type == HWT101_FRAME_GYRO) ||
            (frame_type == HWT101_FRAME_ANGLE)) ? 1U : 0U;
}

static int16_t hwt101_make_int16(uint8_t low, uint8_t high)
{
    return (int16_t)((uint16_t)low | ((uint16_t)high << 8U));
}

static uint8_t hwt101_checksum(const uint8_t *frame)
{
    uint8_t checksum = 0U;

    // HWT101 串口帧校验为前 10 个字节累加后取低 8 位。
    for (uint8_t index = 0U; index < (HWT101_FRAME_LENGTH - 1U); index++) {
        checksum = (uint8_t)(checksum + frame[index]);
    }

    return checksum;
}

static void hwt101_parse_frame(const uint8_t *frame)
{
    int16_t raw_x;
    int16_t raw_y;
    int16_t raw_z;

    if (hwt101_checksum(frame) != frame[HWT101_FRAME_LENGTH - 1U]) {
        return;
    }

    raw_x = hwt101_make_int16(frame[2], frame[3]);
    raw_y = hwt101_make_int16(frame[4], frame[5]);
    raw_z = hwt101_make_int16(frame[6], frame[7]);

    switch (frame[1]) {
    case HWT101_FRAME_ACC:
        // 0x51 是 HWT101 模块内部校准后的加速度输出帧，单位换算为 m/s^2。
        AccX = ((float)raw_x / 32768.0f) * 16.0f * 9.8f;
        AccY = ((float)raw_y / 32768.0f) * 16.0f * 9.8f;
        AccZ = ((float)raw_z / 32768.0f) * 16.0f * 9.8f;
        break;

    case HWT101_FRAME_GYRO:
        // 0x52 是 HWT101 模块内部校准后的角速度输出帧，控制环只使用 Z 轴。
        GyrX = ((float)raw_x / 32768.0f) * 2000.0f;
        GyrY = ((float)raw_y / 32768.0f) * 2000.0f;
        GyrZ = ((float)raw_z / 32768.0f) * 2000.0f;
        break;

    case HWT101_FRAME_ANGLE:
        // 0x53 是 HWT101 姿态解算后的角度输出帧，Yaw 是修正后的航向角。
        Roll = ((float)raw_x / 32768.0f) * 180.0f;
        Pitch = ((float)raw_y / 32768.0f) * 180.0f;
        Yaw = ((float)raw_z / 32768.0f) * 180.0f;
        break;

    default:
        break;
    }
}

static uint8_t hwt101_send_command(const uint8_t *command)
{
    return (HAL_UART_Transmit(&huart2, (uint8_t *)command,
                              HWT101_CMD_LENGTH,
                              HWT101_CMD_TIMEOUT_MS) == HAL_OK) ? 1U : 0U;
}

/**
  * @brief 逐字节接收并解析 HWT101 串口输出帧。
  * @param RxData USART2 中断收到的单字节数据。
  */
void jy901_ReceiveData(uint8_t RxData)
{
    if (hwt101_rx_index == 0U) {
        if (RxData == HWT101_FRAME_HEAD) {
            hwt101_rx_buffer[hwt101_rx_index++] = RxData;
        }
        return;
    }

    if (hwt101_rx_index == 1U) {
        if (hwt101_is_data_frame(RxData) != 0U) {
            hwt101_rx_buffer[hwt101_rx_index++] = RxData;
        } else if (RxData == HWT101_FRAME_HEAD) {
            hwt101_rx_buffer[0] = RxData;
            hwt101_rx_index = 1U;
        } else {
            hwt101_rx_index = 0U;
        }
        return;
    }

    hwt101_rx_buffer[hwt101_rx_index++] = RxData;
    if (hwt101_rx_index >= HWT101_FRAME_LENGTH) {
        hwt101_parse_frame(hwt101_rx_buffer);
        hwt101_rx_index = 0U;
    }
}

/**
  * @brief 复位 HWT101 接收解析状态，保留旧函数名以避免影响现有 main.c 调用。
  */
void JY901S_ZeroCalibration(void)
{
    hwt101_rx_index = 0U;
}

/**
  * @brief 请求开启 HWT101 陀螺仪零偏采集。
  * @return 1=已接受请求，0=已有等待发送的零偏控制命令。
  */
uint8_t HWT101_StartGyroBiasCalibration(void)
{
    if (hwt101_bias_pending_cmd != HWT101_BIAS_CMD_NONE) {
        return 0U;
    }

    hwt101_bias_pending_cmd = HWT101_BIAS_CMD_START;
    return 1U;
}

/**
  * @brief 请求停止 HWT101 陀螺仪零偏采集并锁定当前零偏参数。
  * @return 1=已接受请求，0=已有等待发送的零偏控制命令。
  */
uint8_t HWT101_StopGyroBiasCalibration(void)
{
    if (hwt101_bias_pending_cmd != HWT101_BIAS_CMD_NONE) {
        return 0U;
    }

    hwt101_bias_pending_cmd = HWT101_BIAS_CMD_STOP;
    return 1U;
}

/**
  * @brief 按当前零偏采集状态切换开启/停止命令，用于用户按键手动管控。
  * @return 1=已接受请求，0=已有等待发送的零偏控制命令。
  */
uint8_t HWT101_ToggleGyroBiasCalibration(void)
{
    if (hwt101_bias_collecting == 0U) {
        return HWT101_StartGyroBiasCalibration();
    }

    return HWT101_StopGyroBiasCalibration();
}

/**
  * @brief 发送 HWT101 手动零偏控制命令，主循环周期性调用，不做延时等待。
  */
void HWT101_Task(void)
{
    static const uint8_t start_bias_cmd[HWT101_CMD_LENGTH] = {
        HWT101_CMD_HEAD_1,
        HWT101_CMD_HEAD_2,
        HWT101_REG_MANUAL_CALI,
        HWT101_GYRO_BIAS_START_LOW,
        HWT101_GYRO_BIAS_START_HIGH
    };
    static const uint8_t stop_bias_cmd[HWT101_CMD_LENGTH] = {
        HWT101_CMD_HEAD_1,
        HWT101_CMD_HEAD_2,
        HWT101_REG_MANUAL_CALI,
        HWT101_GYRO_BIAS_STOP_LOW,
        HWT101_GYRO_BIAS_STOP_HIGH
    };

    switch (hwt101_bias_pending_cmd) {
    case HWT101_BIAS_CMD_START:
        // 静止 1 秒后由用户系统请求：FF AA A6 01 00，开启陀螺仪零偏采集。
        if (hwt101_send_command(start_bias_cmd) != 0U) {
            hwt101_bias_collecting = 1U;
            hwt101_bias_pending_cmd = HWT101_BIAS_CMD_NONE;
        }
        break;

    case HWT101_BIAS_CMD_STOP:
        // 运动前 1 秒由用户系统请求：FF AA A6 04 00，停止零偏采集并锁定参数。
        if (hwt101_send_command(stop_bias_cmd) != 0U) {
            hwt101_bias_collecting = 0U;
            hwt101_bias_pending_cmd = HWT101_BIAS_CMD_NONE;
        }
        break;

    default:
        break;
    }
}
