#include "jy901.h"
#include "usart.h"
#include "dma.h"
#include "stm32f4xx_hal.h"

static uint8_t RxBuffer[11];/*接收数据数组*/
static volatile uint8_t RxState = 0;/*接收状态标志位*/
static uint8_t RxIndex = 0;/*接受数组索引*/
float Roll,Pitch,Yaw;/*角度信息，如果只需要整数可以改为整数类型*/
float AccX, AccY, AccZ;/*加速度信息*/
float GyrX, GyrY, GyrZ;/*角速度信息*/
uint8_t g_uart2_receivedata = 0;
/*实验*/
uint8_t Txdata_start=0x55;
uint8_t Txdata_cmd=0x51;


/**
 * @brief       数据包处理函数
 * @param       串口接收的数据RxData
 * @retval      无
 */
void jy901_ReceiveData(uint8_t RxData)
{
	uint8_t i,sum=0;
	if (RxState == 0)	//等待包头
	{
		if (RxData == 0x55)	//收到包头//0x55
		{
			RxBuffer[RxIndex] = RxData;
			RxState = 1;
			RxIndex = 1;	//进入下一状态
		}
	}
	else if (RxState == 1)
	{
		if (RxData == 0x53)	/*判断数据内容，修改这里可以改变要读的数据内容，0x53为角度输出*/
		{
			RxBuffer[RxIndex] = RxData;
			RxState = 2;
			RxIndex = 2;	//进入下一状态
		}
		else if (RxData == 0x51)	/*加速度数据包*/
    {
      RxBuffer[RxIndex] = RxData;
      RxState = 3;  // 使用不同状态接收加速度数据
      RxIndex = 2;
    }
	 else if (RxData == 0x52)	/*角速度数据包*/
	 {
			RxBuffer[RxIndex] = RxData;
			RxState = 4;  // 使用不同状态接收角速度数据
			RxIndex = 2;
	 }
	}
	else if (RxState == 2)	//接收角度数据
	{
		RxBuffer[RxIndex++] = RxData;
		if(RxIndex == 11)	//接收完成
		{
			for(i=0;i<10;i++)
			{
				sum = sum + RxBuffer[i];	//计算校验和
			}
			if(sum == RxBuffer[10])		//校验成功
			{
				/*计算数据，根据数据内容选择对应的计算公式*/
				Roll = ((int16_t) ((int16_t) RxBuffer[3] << 8 | (int16_t) RxBuffer[2])) / 32768.0f * 180.0f;
				Pitch = ((int16_t) ((int16_t) RxBuffer[5] << 8 | (int16_t) RxBuffer[4])) / 32768.0f * 180.0f;
				Yaw = ((int16_t) ((int16_t) RxBuffer[7] << 8 | (int16_t) RxBuffer[6])) / 32768.0f * 180.0f;
			}
			RxState = 0;
			RxIndex = 0;	//读取完成，回到最初状态，等待包头
		}
	}
	else if (RxState == 3)	//接收加速度数据
	{
		RxBuffer[RxIndex++] = RxData;
		if(RxIndex == 11)	//接收完成
		{
				for(i=0;i<10;i++)
				{
						sum = sum + RxBuffer[i];	//计算校验和
				}
				if(sum == RxBuffer[10])		//校验成功
				{
						/*计算加速度数据 (单位: g)*/
						AccX = ((int16_t) ((int16_t) RxBuffer[3] << 8 | (int16_t) RxBuffer[2])) / 32768.0f * 16.0f * 9.8f;
						AccY = ((int16_t) ((int16_t) RxBuffer[5] << 8 | (int16_t) RxBuffer[4])) / 32768.0f * 16.0f * 9.8f;
						AccZ = ((int16_t) ((int16_t) RxBuffer[7] << 8 | (int16_t) RxBuffer[6])) / 32768.0f * 16.0f * 9.8f;
				}
				RxState = 0;
				RxIndex = 0;	//读取完成，回到最初状态
		}
	}
	else if (RxState == 4)	//接收角速度数据
	{
		RxBuffer[RxIndex++] = RxData;
		if(RxIndex == 11)	//接收完成
		{
			for(i=0;i<10;i++)
			{
					sum = sum + RxBuffer[i];	//计算校验和
			}
			if(sum == RxBuffer[10])		//校验成功
			{
					/*计算角速度数据 (单位: °/s)*/
					GyrX = ((int16_t) ((int16_t) RxBuffer[3] << 8 | (int16_t) RxBuffer[2])) / 32768.0f * 2000.0f;
					GyrY = ((int16_t) ((int16_t) RxBuffer[5] << 8 | (int16_t) RxBuffer[4])) / 32768.0f * 2000.0f;
					GyrZ = ((int16_t) ((int16_t) RxBuffer[7] << 8 | (int16_t) RxBuffer[6])) / 32768.0f * 2000.0f;
			}
			RxState = 0;
			RxIndex = 0;	//读取完成，回到最初状态
		}
	}
}



// JY901S置零命令
void JY901S_ZeroCalibration(void)
{
	
	  uint8_t zero_start[5]= {0xFF, 0xAA, 0x69, 0x88, 0xB5};
    uint8_t zero_cmd[5] = {0xFF, 0xAA, 0x01, 0x08, 0x00};
    uint8_t zero_save[5] = {0xFF, 0xAA, 0x00, 0x00, 0x00};
		
    // 发送解锁命令
    HAL_UART_Transmit(&huart1, zero_start, 5, 1000);
		HAL_Delay(200);
		//校准命令
		HAL_UART_Transmit(&huart1, zero_cmd, 5, 1000);
		HAL_Delay(3000);
		//校准保存
		HAL_UART_Transmit(&huart1, zero_save, 5, 1000);
}




//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
//	if(huart == &huart1){
//		jy61p_ReceiveData(g_uart2_receivedata);
//		HAL_UART_Receive_IT(&huart1,&g_uart2_receivedata,1);
//	}
//}

//_Bool rx_Dma_state = 0;

//void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
//{
//	if(huart->Instance == USART1)
//	{
//		//rx_Dma_state = 1;
//		
//		jy901_ReceiveData(g_uart2_receivedata);
//	//	
//	//	recive_proc(Rx_data);
//	//	
//	//	HAL_UART_Receive_IT(&huart1,&Rx_data,1);
//		
//		HAL_UART_Receive_DMA(&huart1,&g_uart2_receivedata,11);
//	}
//}






