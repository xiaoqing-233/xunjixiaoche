/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "stdlib.h"
#include "math.h"
#include "ganweixunji.h"
#include "motor.h"
#include "string.h"
#include "xunji.h"
#include "vofa.h"
#include "justfloat.h"
#include "pid.h"
#include "jy901.h"
#include "key.h"
#include "oled_i2c.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum {
  RACE_TASK_AB = 0,
  RACE_TASK_LOOP,
  RACE_TASK_LOOP_A
} RaceTask;

typedef enum {
  RACE_MENU_CALIBRATION = 0,
  RACE_MENU_TASK
} RaceMenu;

typedef enum {
  RACE_STATE_READY = 0,
  RACE_STATE_RUNNING,
  RACE_STATE_PRESTOP,
  RACE_STATE_STOPPING,
  RACE_STATE_DONE
} RaceState;


/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
uint8_t send[30];
uint8_t sensor_erjinzhi[8];

static volatile HAL_StatusTypeDef oled_i2c_init_status;

uint32_t lcd_refresh_time = 0;
uint32_t gw_read_time = 0;
static uint32_t telemetry_next_time;
extern float r_speed_left, r_speed_right;

float suduzuo;
float suduyou;

extern float distance;

extern float finnal_angle_z;


/*角度*/
extern float Roll,Pitch,Yaw;/*角度信息，如果只需要整数可以改为整数类型*/
extern float AccX, AccY, AccZ;/*加速度信息*/
extern float GyrX, GyrY, GyrZ;/*角速度信息*/
/***********传感器************/
uint8_t track_mode = 1;  
/****************************/

static RaceTask race_selected_task = RACE_TASK_AB;
static RaceMenu race_menu = RACE_MENU_CALIBRATION;
static RaceState race_state = RACE_STATE_READY;
static uint32_t race_start_tick;
static uint32_t race_elapsed_ms;
static float race_last_yaw;
static float race_yaw_delta;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static void race_task_handle_keys(void);
static void race_task_start(void);
static void race_task_update(uint32_t now);
static void race_task_draw_oled(void);
static void race_task_draw_calibration_oled(void);
static float race_yaw_delta_from_start(float current_yaw, float start_yaw);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static float race_yaw_delta_from_start(float current_yaw, float start_yaw)
{
  float delta = current_yaw - start_yaw;

  if (delta > 15.0f) {
    delta -= 360.0f;
  }

  return delta;
}

static void race_task_start(void)
{
  uint32_t now = HAL_GetTick();

  PID_ClearSlowStop();
  PID_SetTargetSpeedScale(1.0f);
  PID_ResetAll();
  race_state = RACE_STATE_RUNNING;
  race_start_tick = now;
  race_elapsed_ms = 0U;
  race_last_yaw = Yaw;
  race_yaw_delta = 0.0f;
  k = (race_selected_task == RACE_TASK_LOOP_A) ? 2U : 1U;
  star_car = 1U;
  HAL_GPIO_WritePin(CAR_STATE_TOGGLE_GPIO_Port, CAR_STATE_TOGGLE_Pin, GPIO_PIN_RESET);
}

static void race_task_handle_keys(void)
{
  KeyEvent event = key_take_last_event();

  if (event == KEY_EVENT_NONE) {
    return;
  }

  if (race_menu == RACE_MENU_CALIBRATION) {
    if (event == KEY_EVENT_3_PRESSED) {
      race_menu = RACE_MENU_TASK;
      race_state = RACE_STATE_READY;
      race_elapsed_ms = 0U;
    }
    return;
  }

  if ((event == KEY_EVENT_2_PRESSED) &&
      ((race_state == RACE_STATE_READY) || (race_state == RACE_STATE_DONE))) {
    if (race_selected_task == RACE_TASK_AB) {
      race_selected_task = RACE_TASK_LOOP;
    } else if (race_selected_task == RACE_TASK_LOOP) {
      race_selected_task = RACE_TASK_LOOP_A;
    } else {
      race_selected_task = RACE_TASK_AB;
    }
    race_state = RACE_STATE_READY;
    race_elapsed_ms = 0U;
  } else if ((event == KEY_EVENT_3_PRESSED) &&
             ((race_state == RACE_STATE_READY) || (race_state == RACE_STATE_DONE))) {
    race_task_start();
  }
}

static void race_task_draw_calibration_oled(void)
{
  oled_i2c_clear_buffer();
  sprintf((char *)send, "%.0f %.0f", speed_left, speed_right);
  oled_i2c_draw_string(0, 0, (char *)send, 16, 1);
  sprintf((char *)send, "%.1f %.1f", GyrZ, Yaw);
  oled_i2c_draw_string(0, 16, (char *)send, 16, 1);
  sprintf((char *)send, "%d%d%d%d%d%d%d%d",
          sensor[0], sensor[1], sensor[2], sensor[3],
          sensor[4], sensor[5], sensor[6], sensor[7]);
  oled_i2c_draw_string(0, 32, (char *)send, 16, 1);
}

static void race_task_draw_menu_oled(void)
{
  uint32_t task_id = (uint32_t)race_selected_task + 1U;

  oled_i2c_clear_buffer();
  sprintf((char *)send, "%lu", (unsigned long)task_id);
  oled_i2c_draw_string(0, 0, (char *)send, 16, 1);
}

static void race_task_draw_done_oled(void)
{
  uint32_t seconds = race_elapsed_ms / 1000U;
  uint32_t centiseconds = (race_elapsed_ms % 1000U) / 10U;

  oled_i2c_clear_buffer();
  sprintf((char *)send, "%lu.%02lu", (unsigned long)seconds, (unsigned long)centiseconds);
  oled_i2c_draw_string(0, 0, (char *)send, 16, 1);
}

static void race_task_draw_running_oled(void)
{
  uint32_t task_id = (uint32_t)race_selected_task + 1U;
  uint32_t state_id;
  uint32_t seconds = race_elapsed_ms / 1000U;
  uint32_t centiseconds = (race_elapsed_ms % 1000U) / 10U;

  if (race_state == RACE_STATE_RUNNING) { state_id = 1U; }
  else if (race_state == RACE_STATE_PRESTOP) { state_id = 2U; }
  else if (race_state == RACE_STATE_STOPPING) { state_id = 3U; }
  else { state_id = 4U; }

  oled_i2c_clear_buffer();
  sprintf((char *)send, "%lu %lu", (unsigned long)task_id, (unsigned long)state_id);
  oled_i2c_draw_string(0, 0, (char *)send, 16, 1);
  sprintf((char *)send, "%lu.%02lu", (unsigned long)seconds, (unsigned long)centiseconds);
  oled_i2c_draw_string(0, 16, (char *)send, 16, 1);
  sprintf((char *)send, "%.0f", race_yaw_delta);
  oled_i2c_draw_string(0, 32, (char *)send, 16, 1);
  sprintf((char *)send, "%u %u", black_count, line_lost);
  oled_i2c_draw_string(0, 48, (char *)send, 16, 1);
}

static void race_task_update(uint32_t now)
{
  if ((race_state != RACE_STATE_RUNNING) &&
      (race_state != RACE_STATE_PRESTOP) &&
      (race_state != RACE_STATE_STOPPING)) {
    return;
  }

  race_elapsed_ms = now - race_start_tick;
  race_yaw_delta = race_yaw_delta_from_start(Yaw, race_last_yaw);

  if (race_state == RACE_STATE_RUNNING) {
    if ((race_selected_task == RACE_TASK_AB) && (race_yaw_delta <= -20.0f)) {
			PID_SetTargetSpeedScale(0.0f);
      HAL_GPIO_WritePin(CAR_STATE_TOGGLE_GPIO_Port, CAR_STATE_TOGGLE_Pin, GPIO_PIN_SET);
      race_state = RACE_STATE_STOPPING;
    } else if ((race_selected_task == RACE_TASK_LOOP) &&
               (race_yaw_delta <= -320.0f)) {
      PID_SetTargetSpeedScale(0.0f);
      HAL_GPIO_WritePin(CAR_STATE_TOGGLE_GPIO_Port, CAR_STATE_TOGGLE_Pin, GPIO_PIN_SET);
      race_state = RACE_STATE_STOPPING;
    } else if ((race_selected_task == RACE_TASK_LOOP_A) &&
               (race_yaw_delta <= -330.0f)) {
      PID_SetTargetSpeedScale(0.4f);
      race_state = RACE_STATE_PRESTOP;
    }
  } else if (race_state == RACE_STATE_PRESTOP) {
    if ((line_lost != 0U) || (black_count > 3U)) {
      HAL_GPIO_WritePin(CAR_STATE_TOGGLE_GPIO_Port, CAR_STATE_TOGGLE_Pin, GPIO_PIN_SET);
      star_car = 0U;
      k = 0U;
			set_speed(0, 0);
      race_state = RACE_STATE_DONE;
      race_elapsed_ms = now - race_start_tick;
    } else if (black_count == 3U) {
      HAL_GPIO_WritePin(CAR_STATE_TOGGLE_GPIO_Port, CAR_STATE_TOGGLE_Pin, GPIO_PIN_SET);
      PID_RequestSlowStop();
      race_state = RACE_STATE_STOPPING;
    }
  }

  if ((race_state == RACE_STATE_STOPPING) &&
      ((PID_IsSlowStopDone() != 0U) || (PID_IsTargetSpeedScaleStopped() != 0U))) {
    star_car = 0U;
    race_state = RACE_STATE_DONE;
    k = 0U;
    race_elapsed_ms = now - race_start_tick;
  }
}

static void race_task_draw_oled(void)
{
  if (race_menu == RACE_MENU_CALIBRATION) {
    race_task_draw_calibration_oled();
    return;
  }

  if (race_state == RACE_STATE_READY) {
    race_task_draw_menu_oled();
  } else if (race_state == RACE_STATE_DONE) {
    race_task_draw_done_oled();
  } else {
    race_task_draw_running_oled();
  }
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_USART1_UART_Init();
  MX_TIM12_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
	oled_i2c_init_status = oled_i2c_init();
	motor_init();
	vofa_init();
	HAL_UART_Receive_IT(&huart2, &g_uart2_receivedata, 1);
	JY901S_ZeroCalibration();
	telemetry_next_time = HAL_GetTick() + 10U;
  /* USER CODE END 2 */



  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
		uint32_t current_time;

		vofa_task();
		current_time = HAL_GetTick();
		if ((uint32_t)(current_time - telemetry_next_time) < 0x80000000U) //vofa 发送
		{
			float telemetry_values[2] = { Kp_pos,speed_right };

			telemetry_next_time = current_time + 10U;
			(void)justfloat_send_dma(telemetry_values, 2U);
		}

		if((oled_i2c_init_status == HAL_OK) && (HAL_GetTick() - lcd_refresh_time > 100))  //oled数据
		{
			lcd_refresh_time = HAL_GetTick();
			race_task_draw_oled();
			oled_i2c_init_status = oled_i2c_refresh();
		}
		if(HAL_GetTick() - gw_read_time > 5)  //循迹i2c读取速度限制  200hz
		{  
			gw_read_time = HAL_GetTick();
			gw_get_value();
			track_line();
			race_task_update(gw_read_time);
		}
			
			key_control();
			race_task_handle_keys();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/*dma传递*/
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart == &huart1){
		vofa_receive_byte(*vofa_rx_buffer());
	}
	else if(huart == &huart2){
		jy901_ReceiveData(g_uart2_receivedata);
		HAL_UART_Receive_IT(&huart2, &g_uart2_receivedata, 1);
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	justfloat_on_tx_complete(huart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	justfloat_on_uart_error(huart);
	vofa_on_uart_error(huart);
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
