//#include "gw_grayscale_sensor.h"



//#define GW_GRAY_ADDR GW_GRAY_ADDR_DEF
//#define GW_GRAY_SERIAL_GPIO_GROUP GPIOB
//#define GW_GRAY_SERIAL_GPIO_CLK GPIO_PIN_8 //PB8 串行CLK
//#define GW_GRAY_SERIAL_GPIO_DAT GPIO_PIN_9 //PB9 串行DAT
//#define GW_GRAY_SERIAL_DELAY_VALUE 300




//void SystemClock_Config(void);

///* 8MHz 5us大概是27, 64MHz 5us大概是270 */
// void delay(uint32_t delay_count)
//{
//	for (int i = 0; i < delay_count; ++i) {
//	}
//}

// uint8_t gw_gray_serial_read()
//{
//	uint8_t ret = 0;

//	for (int i = 0; i < 8; ++i) {
//		/* 输出时钟下降沿 */
//		HAL_GPIO_WritePin(GW_GRAY_SERIAL_GPIO_GROUP, GW_GRAY_SERIAL_GPIO_CLK, 0);
//		delay(GW_GRAY_SERIAL_DELAY_VALUE); // 外部有上拉源(大约10k电阻) 可不加此行

//		ret |= HAL_GPIO_ReadPin(GW_GRAY_SERIAL_GPIO_GROUP, GW_GRAY_SERIAL_GPIO_DAT) << i;

//		/* 输出时钟上升沿,让传感器更新数据*/
//		HAL_GPIO_WritePin(GW_GRAY_SERIAL_GPIO_GROUP, GW_GRAY_SERIAL_GPIO_CLK, 1);

//		/* 主控频率高的需要给一点点延迟,延迟需要在5us左右 */
//		delay(GW_GRAY_SERIAL_DELAY_VALUE);
//	}

//	return ret;
//}

////int main(void)
////{
////	HAL_Init();
////	SystemClock_Config();
////	MX_GPIO_Init();

////	// 此处"volatile"仅用于观察数据（volatile会阻止编译器对目标的优化），移植的时候请去掉"volatile"修饰词


////	while (1) {
////		sensor_data = gw_gray_serial_read();
////		// sensor_data 有8个探头的数据 最低位是第一个探头数据

////		// 把八个探头的数据分散到八个变量里
////		SEP_ALL_BIT8(sensor_data,
////			sensor[0], sensor[1], sensor[2], sensor[3], sensor[4], sensor[5], sensor[6], sensor[7]);
////		HAL_Delay(1);
////	}
////}



//void SENSOR_Init(void)
//{
//	GPIO_InitTypeDef GPIO_InitStruct = {0};
//	__HAL_RCC_GPIOB_CLK_ENABLE();

//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_8, GPIO_PIN_RESET);

//	GPIO_InitStruct.Pin = GPIO_PIN_8;
//	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
//	GPIO_InitStruct.Pull = GPIO_NOPULL;
//	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
//	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

//	GPIO_InitStruct.Pin = GPIO_PIN_9;
//	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
//	GPIO_InitStruct.Pull = GPIO_PULLUP;
//	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
//}


