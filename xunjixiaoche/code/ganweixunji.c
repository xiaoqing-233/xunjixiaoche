#include "ganweixunji.h"

#include "jy901.h"

#include <string.h>

unsigned char Digtal = 0xFFU;

#define GW_SENSOR_COUNT              8U
#define GW_ADC_MAX_VALUE             4095U
#define GW_ADC_SETTLE_US             20U
#define GW_ADC_SAMPLE_TIMES          8U

// FIXME: Keep this storage address reserved in the linker/CubeMX memory plan.
#define GW_CAL_FLASH_ADDR            0x080E0000U
#define GW_CAL_FLASH_SECTOR          FLASH_SECTOR_11
#define GW_CAL_MAGIC                 0x47574341U
#define GW_CAL_VERSION               1U
#define GW_CAL_LONG_PRESS_MS         4000U
#define GW_CAL_DEBOUNCE_MS           20U
#define GW_CAL_SAMPLE_FRAMES         32U
#define GW_CAL_SAMPLE_DELAY_MS       30U

typedef enum {
    GW_CAL_IDLE = 0,
    GW_CAL_WAIT_BLACK,
    GW_CAL_WAIT_WHITE
} GwCalibrationState;

typedef enum {
    GW_KEY_NONE = 0,
    GW_KEY_PRESSED,
    GW_KEY_RELEASED
} GwKeyEvent;

typedef struct {
    uint32_t magic;
    uint32_t version;
    uint16_t black[GW_SENSOR_COUNT];
    uint16_t white[GW_SENSOR_COUNT];
    uint32_t checksum;
} GwCalibrationData;

static uint8_t gw_initialized;
static GwCalibrationState gw_cal_state = GW_CAL_IDLE;
static uint8_t gw_key_stable;
static uint8_t gw_key_candidate;
static uint8_t gw_long_press_handled;
static uint32_t gw_key_candidate_since;
static uint32_t gw_key_press_tick;
static uint16_t gw_threshold_low[GW_SENSOR_COUNT];
static uint16_t gw_threshold_high[GW_SENSOR_COUNT];
static uint16_t gw_pending_black[GW_SENSOR_COUNT];

static void gw_address_write(uint8_t channel)
{
    HAL_GPIO_WritePin(GW_ADDR0_GPIO_Port, GW_ADDR0_Pin,
                      (channel & 0x01U) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(GW_ADDR1_GPIO_Port, GW_ADDR1_Pin,
                      (channel & 0x02U) ? GPIO_PIN_RESET : GPIO_PIN_SET);
    HAL_GPIO_WritePin(GW_ADDR2_GPIO_Port, GW_ADDR2_Pin,
                      (channel & 0x04U) ? GPIO_PIN_RESET : GPIO_PIN_SET);
}

static void gw_adc_init(void)
{
    __HAL_RCC_ADC1_CLK_ENABLE();

    ADC->CCR = ADC_CCR_ADCPRE_0;
    ADC1->CR1 = 0U;
    ADC1->CR2 = 0U;
    ADC1->SMPR2 &= ~ADC_SMPR2_SMP0;
    ADC1->SMPR2 |= ADC_SMPR2_SMP0_2;
    ADC1->SQR1 = 0U;
    ADC1->SQR2 = 0U;
    ADC1->SQR3 = 0U;
    ADC1->CR2 = ADC_CR2_ADON;
}

static uint16_t gw_adc_read_once(void)
{
    uint32_t start_tick = HAL_GetTick();

    ADC1->SR = 0U;
    ADC1->CR2 |= ADC_CR2_SWSTART;

    while ((ADC1->SR & ADC_SR_EOC) == 0U) {
        if ((uint32_t)(HAL_GetTick() - start_tick) > 2U) {
            return 0U;
        }
    }

    return (uint16_t)(ADC1->DR & GW_ADC_MAX_VALUE);
}

static uint16_t gw_read_channel(uint8_t channel)
{
    uint32_t sum = 0U;

    gw_address_write(channel);
    Delay_us(GW_ADC_SETTLE_US);

    for (uint8_t sample = 0U; sample < GW_ADC_SAMPLE_TIMES; sample++) {
        sum += gw_adc_read_once();
    }

    return (uint16_t)(sum / GW_ADC_SAMPLE_TIMES);
}

static void gw_read_analog(uint16_t *result)
{
    for (uint8_t channel = 0U; channel < GW_SENSOR_COUNT; channel++) {
#if GW_SENSOR_REVERSE_ORDER
        result[GW_SENSOR_COUNT - 1U - channel] = gw_read_channel(channel);
#else
        result[channel] = gw_read_channel(channel);
#endif
    }
}

static uint32_t gw_cal_checksum(const GwCalibrationData *data)
{
    uint32_t checksum = data->magic ^ data->version ^ 0x5A5AA5A5U;

    for (uint8_t index = 0U; index < GW_SENSOR_COUNT; index++) {
        checksum += ((uint32_t)data->black[index] << (index & 0x07U));
        checksum ^= ((uint32_t)data->white[index] << ((index + 3U) & 0x07U));
    }

    return checksum;
}

static uint8_t gw_cal_load(uint16_t *white, uint16_t *black)
{
    const GwCalibrationData *data = (const GwCalibrationData *)GW_CAL_FLASH_ADDR;

    if ((data->magic != GW_CAL_MAGIC) ||
        (data->version != GW_CAL_VERSION) ||
        (data->checksum != gw_cal_checksum(data))) {
        return 0U;
    }

    memcpy(black, data->black, sizeof(data->black));
    memcpy(white, data->white, sizeof(data->white));
    return 1U;
}

static uint8_t gw_cal_save(const uint16_t *white, const uint16_t *black)
{
    GwCalibrationData data;
    FLASH_EraseInitTypeDef erase_init;
    uint32_t sector_error = 0U;
    uint32_t address = GW_CAL_FLASH_ADDR;
    const uint32_t *word = (const uint32_t *)&data;

    data.magic = GW_CAL_MAGIC;
    data.version = GW_CAL_VERSION;
    memcpy(data.black, black, sizeof(data.black));
    memcpy(data.white, white, sizeof(data.white));
    data.checksum = gw_cal_checksum(&data);

    HAL_FLASH_Unlock();

    erase_init.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase_init.Sector = GW_CAL_FLASH_SECTOR;
    erase_init.NbSectors = 1U;
    erase_init.VoltageRange = FLASH_VOLTAGE_RANGE_3;

    if (HAL_FLASHEx_Erase(&erase_init, &sector_error) != HAL_OK) {
        HAL_FLASH_Lock();
        return 0U;
    }

    for (uint32_t index = 0U; index < (sizeof(data) / sizeof(uint32_t)); index++) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address, word[index]) != HAL_OK) {
            HAL_FLASH_Lock();
            return 0U;
        }
        address += sizeof(uint32_t);
    }

    if (memcmp((const void *)GW_CAL_FLASH_ADDR, &data, sizeof(data)) != 0) {
        HAL_FLASH_Lock();
        return 0U;
    }

    HAL_FLASH_Lock();
    return 1U;
}

static void gw_apply_calibration(const uint16_t *white, const uint16_t *black)
{
    for (uint8_t index = 0U; index < GW_SENSOR_COUNT; index++) {
        uint16_t low = black[index];
        uint16_t high = white[index];

        if (low > high) {
            uint16_t temp = low;
            low = high;
            high = temp;
        }

        gw_threshold_low[index] = (uint16_t)((high + (low * 2U)) / 3U);
        gw_threshold_high[index] = (uint16_t)(((high * 2U) + low) / 3U);
    }
}

static void gw_load_default_calibration(void)
{
    uint16_t default_black[GW_SENSOR_COUNT];
    uint16_t default_white[GW_SENSOR_COUNT];

    for (uint8_t index = 0U; index < GW_SENSOR_COUNT; index++) {
        default_black[index] = 100U;
        default_white[index] = 3500U;
    }

    (void)gw_cal_load(default_white, default_black);

    gw_apply_calibration(default_white, default_black);
}

static void gw_read_average(uint16_t *result)
{
    uint32_t sum[GW_SENSOR_COUNT] = {0U};
    uint16_t sample[GW_SENSOR_COUNT];

    for (uint8_t frame = 0U; frame < GW_CAL_SAMPLE_FRAMES; frame++) {
        gw_read_analog(sample);
        for (uint8_t index = 0U; index < GW_SENSOR_COUNT; index++) {
            sum[index] += sample[index];
        }

        if ((frame & 0x03U) == 0U) {
            HAL_GPIO_TogglePin(GW_CAL_LED_GPIO_Port, GW_CAL_LED_Pin);
        }
        HAL_Delay(GW_CAL_SAMPLE_DELAY_MS);
    }

    for (uint8_t index = 0U; index < GW_SENSOR_COUNT; index++) {
        result[index] = (uint16_t)(sum[index] / GW_CAL_SAMPLE_FRAMES);
    }
}

static uint8_t gw_key_is_pressed(void)
{
    return (HAL_GPIO_ReadPin(GW_CAL_KEY_GPIO_Port, GW_CAL_KEY_Pin) == GPIO_PIN_RESET) ? 1U : 0U;
}

static GwKeyEvent gw_key_update(void)
{
    uint32_t now = HAL_GetTick();
    uint8_t sample = gw_key_is_pressed();

    if (sample != gw_key_candidate) {
        gw_key_candidate = sample;
        gw_key_candidate_since = now;
        return GW_KEY_NONE;
    }

    if ((sample != gw_key_stable) &&
        ((uint32_t)(now - gw_key_candidate_since) >= GW_CAL_DEBOUNCE_MS)) {
        gw_key_stable = sample;
        return sample ? GW_KEY_PRESSED : GW_KEY_RELEASED;
    }

    return GW_KEY_NONE;
}

static uint8_t gw_calibration_task(void)
{
    GwKeyEvent event = gw_key_update();
    uint32_t now = HAL_GetTick();

    if (event == GW_KEY_PRESSED) {
        gw_key_press_tick = now;
        gw_long_press_handled = 0U;
    } else if (event == GW_KEY_RELEASED) {
        if (gw_long_press_handled != 0U) {
            gw_long_press_handled = 0U;
        } else if (gw_cal_state == GW_CAL_WAIT_BLACK) {
            gw_read_average(gw_pending_black);
            HAL_GPIO_WritePin(GW_CAL_LED_GPIO_Port, GW_CAL_LED_Pin, GPIO_PIN_SET);
            gw_cal_state = GW_CAL_WAIT_WHITE;
        } else if (gw_cal_state == GW_CAL_WAIT_WHITE) {
            uint16_t white[GW_SENSOR_COUNT];

            gw_read_average(white);
            if (gw_cal_save(white, gw_pending_black) != 0U) {
                gw_apply_calibration(white, gw_pending_black);
                HAL_GPIO_WritePin(GW_CAL_LED_GPIO_Port, GW_CAL_LED_Pin, GPIO_PIN_RESET);
                gw_cal_state = GW_CAL_IDLE;
            } else {
                HAL_GPIO_WritePin(GW_CAL_LED_GPIO_Port, GW_CAL_LED_Pin, GPIO_PIN_SET);
            }
        } else {
            // 用户系统手动管控零偏：静止后短按开启，运动前短按停止。
            (void)HWT101_ToggleGyroBiasCalibration();
        }
    }

    if ((gw_key_stable != 0U) &&
        (gw_long_press_handled == 0U) &&
        (gw_cal_state == GW_CAL_IDLE) &&
        ((uint32_t)(now - gw_key_press_tick) >= GW_CAL_LONG_PRESS_MS)) {
        gw_long_press_handled = 1U;
        gw_cal_state = GW_CAL_WAIT_BLACK;
        Digtal = 0xFFU;
        HAL_GPIO_WritePin(GW_CAL_LED_GPIO_Port, GW_CAL_LED_Pin, GPIO_PIN_SET);
    }

    return (gw_cal_state != GW_CAL_IDLE) ? 1U : 0U;
}

static void gw_init_once(void)
{
    if (gw_initialized != 0U) {
        return;
    }

    gw_adc_init();
    gw_key_stable = gw_key_is_pressed();
    gw_key_candidate = gw_key_stable;
    gw_key_candidate_since = HAL_GetTick();
    gw_key_press_tick = HAL_GetTick();
    HAL_GPIO_WritePin(GW_CAL_LED_GPIO_Port, GW_CAL_LED_Pin, GPIO_PIN_RESET);
    gw_load_default_calibration();
    gw_initialized = 1U;
}

void gw_get_value(void)
{
    uint16_t analog[GW_SENSOR_COUNT];

    gw_init_once();
    HWT101_Task();
    if (gw_calibration_task() != 0U) {
        Digtal = 0xFFU;
        return;
    }

    gw_read_analog(analog);
    for (uint8_t index = 0U; index < GW_SENSOR_COUNT; index++) {
        if (analog[index] > gw_threshold_high[index]) {
            Digtal |= (uint8_t)(1U << index);
        } else if (analog[index] < gw_threshold_low[index]) {
            Digtal &= (uint8_t)~(1U << index);
        }
    }
}
