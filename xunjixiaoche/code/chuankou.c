#include "chuankou.h"
#include "usart.h"


uint8_t Rx_buffer[10];
uint8_t counter = 0;
uint8_t Rx_state = 0;
extern float Kp_l,Ki_l,Kp_r,Ki_r;
extern float r_speed_left, r_speed_right;
extern uint8_t Rx_data;


void recive_proc(uint8_t data)
{
	if(Rx_state == 0)
	{
		if(data == '=')
		{
			Rx_buffer[counter] = data;
			counter++;
			Rx_state = 1;
		}
	}
	else if(Rx_state == 1)
	{
		Rx_buffer[counter++] = data;
		if(counter >= 6 && Rx_buffer[5] == '!')
		{
			//获取数值
			uint8_t num1 = Rx_buffer[3] - '0';
      uint8_t num2 = Rx_buffer[4] - '0';
      float value = (num1 * 10 + num2) / 10.0f;
			
			if(Rx_buffer[1] == 'P')
			{
				if(Rx_buffer[2] == 'L')
				{
					Kp_l = (num1 * 10 + num2) / 10.0f;
				}
				else if(Rx_buffer[2] == 'R')
				{
					Kp_r = (num1 * 10 + num2) / 10.0f;
				}
				counter = 0;
				Rx_state = 0;
			}
			else if(Rx_buffer[1] == 'I')
			{
				if(Rx_buffer[2] == 'L')
				{
					Ki_l = (num1 * 10 + num2) / 10.0f;
				}
				else if(Rx_buffer[2] == 'R')
				{
					Ki_r = (num1 * 10 + num2) / 10.0f;
				}
				
		   counter = 0;
			 Rx_state = 0;
			}
			else if(counter >= 10)
			{ 
				counter = 0;
				Rx_state = 0;
			
			}
		}
	}
		
	
}




