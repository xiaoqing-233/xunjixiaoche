#include "jy901.h"
#include "usart.h"
#include "dma.h"
#include "stm32f4xx_hal.h"

static uint8_t RxBuffer[11];/*������������*/
static volatile uint8_t RxState = 0;/*����״̬��־λ*/
static uint8_t RxIndex = 0;/*������������*/
float Roll,Pitch,Yaw;/*�Ƕ���Ϣ�����ֻ��Ҫ�������Ը�Ϊ��������*/
float AccX, AccY, AccZ;/*���ٶ���Ϣ*/
float GyrX, GyrY, GyrZ;/*���ٶ���Ϣ*/
uint8_t g_uart2_receivedata = 0;
/*ʵ��*/
uint8_t Txdata_start=0x55;
uint8_t Txdata_cmd=0x51;


/**
 * @brief       ���ݰ���������
 * @param       ���ڽ��յ�����RxData
 * @retval      ��
 */
void jy901_ReceiveData(uint8_t RxData)
{
	uint8_t i,sum=0;
	if (RxState == 0)	//�ȴ���ͷ
	{
		if (RxData == 0x55)	//�յ���ͷ//0x55
		{
			RxBuffer[RxIndex] = RxData;
			RxState = 1;
			RxIndex = 1;	//������һ״̬
		}
	}
	else if (RxState == 1)
	{
		if (RxData == 0x53)	/*�ж��������ݣ��޸�������Ըı�Ҫ�����������ݣ�0x53Ϊ�Ƕ����*/
		{
			RxBuffer[RxIndex] = RxData;
			RxState = 2;
			RxIndex = 2;	//������һ״̬
		}
		else if (RxData == 0x51)	/*���ٶ����ݰ�*/
    {
      RxBuffer[RxIndex] = RxData;
      RxState = 3;  // ʹ�ò�ͬ״̬���ռ��ٶ�����
      RxIndex = 2;
    }
	 else if (RxData == 0x52)	/*���ٶ����ݰ�*/
	 {
			RxBuffer[RxIndex] = RxData;
			RxState = 4;  // ʹ�ò�ͬ״̬���ս��ٶ�����
			RxIndex = 2;
	 }
	}
	else if (RxState == 2)	//���սǶ�����
	{
		RxBuffer[RxIndex++] = RxData;
		if(RxIndex == 11)	//�������
		{
			for(i=0;i<10;i++)
			{
				sum = sum + RxBuffer[i];	//����У���
			}
			if(sum == RxBuffer[10])		//У��ɹ�
			{
				/*�������ݣ�������������ѡ���Ӧ�ļ��㹫ʽ*/
				Roll = ((int16_t) ((int16_t) RxBuffer[3] << 8 | (int16_t) RxBuffer[2])) / 32768.0f * 180.0f;
				Pitch = ((int16_t) ((int16_t) RxBuffer[5] << 8 | (int16_t) RxBuffer[4])) / 32768.0f * 180.0f;
				Yaw = ((int16_t) ((int16_t) RxBuffer[7] << 8 | (int16_t) RxBuffer[6])) / 32768.0f * 180.0f;
			}
			RxState = 0;
			RxIndex = 0;	//��ȡ��ɣ��ص����״̬���ȴ���ͷ
		}
	}
	else if (RxState == 3)	//���ռ��ٶ�����
	{
		RxBuffer[RxIndex++] = RxData;
		if(RxIndex == 11)	//�������
		{
				for(i=0;i<10;i++)
				{
						sum = sum + RxBuffer[i];	//����У���
				}
				if(sum == RxBuffer[10])		//У��ɹ�
				{
						/*������ٶ����� (��λ: g)*/
						AccX = ((int16_t) ((int16_t) RxBuffer[3] << 8 | (int16_t) RxBuffer[2])) / 32768.0f * 16.0f * 9.8f;
						AccY = ((int16_t) ((int16_t) RxBuffer[5] << 8 | (int16_t) RxBuffer[4])) / 32768.0f * 16.0f * 9.8f;
						AccZ = ((int16_t) ((int16_t) RxBuffer[7] << 8 | (int16_t) RxBuffer[6])) / 32768.0f * 16.0f * 9.8f;
				}
				RxState = 0;
				RxIndex = 0;	//��ȡ��ɣ��ص����״̬
		}
	}
	else if (RxState == 4)	//���ս��ٶ�����
	{
		RxBuffer[RxIndex++] = RxData;
		if(RxIndex == 11)	//�������
		{
			for(i=0;i<10;i++)
			{
					sum = sum + RxBuffer[i];	//����У���
			}
			if(sum == RxBuffer[10])		//У��ɹ�
			{
					/*������ٶ����� (��λ: ��/s)*/
					GyrX = ((int16_t) ((int16_t) RxBuffer[3] << 8 | (int16_t) RxBuffer[2])) / 32768.0f * 2000.0f;
					GyrY = ((int16_t) ((int16_t) RxBuffer[5] << 8 | (int16_t) RxBuffer[4])) / 32768.0f * 2000.0f;
					GyrZ = ((int16_t) ((int16_t) RxBuffer[7] << 8 | (int16_t) RxBuffer[6])) / 32768.0f * 2000.0f;
			}
			RxState = 0;
			RxIndex = 0;	//��ȡ��ɣ��ص����״̬
		}
	}
}



// JY901S��������
void JY901S_ZeroCalibration(void)
{
	
	  uint8_t zero_start[5]= {0xFF, 0xAA, 0x69, 0x88, 0xB5};
    uint8_t zero_cmd[5] = {0xFF, 0xAA, 0x01, 0x08, 0x00};
    uint8_t zero_save[5] = {0xFF, 0xAA, 0x00, 0x00, 0x00};
		
    // ���ͽ�������
    HAL_UART_Transmit(&huart2, zero_start, 5, 1000);
		HAL_Delay(200);
		//У׼����
    HAL_UART_Transmit(&huart2, zero_cmd, 5, 1000);
		HAL_Delay(1000);
		//У׼����
    HAL_UART_Transmit(&huart2, zero_save, 5, 1000);
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





