#include "motor.h"
#include "pid.h"

#define PWM_MAX 1000
#define PWM_MIN -1000
uint8_t stop = 0;

volatile int32_t count1;        // 编码器计数值（来自编码器接口）
volatile int16_t count22;        // 编码器计数值（来自编码器接口）
volatile int32_t last_count1;  // 上次采样的计数值
volatile int32_t last_count2;  // 上次采样的计数值
volatile float speed_left, speed_right;  // 电机速度（RPM）
float r_speed_left, r_speed_right;			//yuqisudu
float speed_1,speed_2;
uint8_t time1;

/*电机限幅函数*/
void Limit(float *motoA,float *motoB)
{
	if(*motoA>PWM_MAX)*motoA=PWM_MAX;
	if(*motoA<PWM_MIN)*motoA=PWM_MIN;
	if(*motoB>PWM_MAX)*motoB=PWM_MAX;
	if(*motoB<PWM_MIN)*motoB=PWM_MIN;
}



//电机两个pwm生成和编码器初始化,电机驱动使能
void motor_init(void){
	//HAL_GPIO_WritePin(SYBT_GPIO_Port,SYBT_Pin,GPIO_PIN_SET);
	
	HAL_TIM_Base_Start_IT(&htim5);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);
	HAL_TIM_Encoder_Start(&htim1,TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim2,TIM_CHANNEL_ALL);
	__HAL_TIM_SET_COUNTER(&htim1,0);//32700
	__HAL_TIM_SET_COUNTER(&htim2,0);
	
}

//void TIM5_IRQHandler(void) {
//   HAL_TIM_IRQHandler(&htim5);
//}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {

	if (htim->Instance == TIM5)
		{
				
				control_speed();
			  count1 = __HAL_TIM_GET_COUNTER(&htim2);
			  count22 = __HAL_TIM_GET_COUNTER(&htim1);
			  int32_t count2= (int32_t)count22;
        static uint32_t prev_time = 0;
        uint32_t current_time = HAL_GetTick();
        float dt = (current_time - prev_time) *0.001f;  // 采样时间（秒）
        prev_time = current_time;
        
        /* 读取编码器计数值（假设已在编码器中断中更新） */
        int32_t current_count1 = count1;
        int32_t current_count2 = count2;
        
        /* 计算计数值变化（考虑溢出情况） */
        int32_t delta_count1 = current_count1 - last_count1;
        int32_t delta_count2 = current_count2 - last_count2;

        /* 更新上次计数值 */
        last_count1 = current_count1;
        last_count2 = current_count2;
        
        /* 计算速度（RPM） */
        speed_left = -(float)delta_count1 * dt * 4.12f*60;
        speed_right = (float)delta_count2 * dt * 4.12f*60;
				
	
    }
	if(htim->Instance == TIM12)
	{

	}
}
		




/* 获取当前速度（供主程序调用） */
float GetMotorSpeed1(void) {
    return speed_left;
}

float GetMotorSpeed2(void) {
    return speed_right;
}

//0前进，1后退
void set_speed(float speed_l,float speed_r){
		if(speed_l > 1000)
			speed_1 = 1000;
		if(speed_l < -1000)
			speed_1 = -1000;
		if(speed_r > 1000)
			speed_2 = 1000;
		if(speed_r < -1000)
			speed_2 = -1000;
	
		if(speed_l > 0){
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_RESET);
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4,speed_l);
		}
		else if(speed_l < 0){
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_SET);
			__HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_4,-speed_l);
		}
		else{
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_8,GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_7,GPIO_PIN_RESET);
		}
		
		if(speed_r > 0){
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,GPIO_PIN_SET);
			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,speed_r);
		}
		else if(speed_r < 0){
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6, GPIO_PIN_SET);
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,GPIO_PIN_RESET);
			__HAL_TIM_SET_COMPARE(&htim4,TIM_CHANNEL_3,-speed_r);
		}
		else{
			HAL_GPIO_WritePin(GPIOC,GPIO_PIN_6, GPIO_PIN_RESET);
			HAL_GPIO_WritePin(GPIOD,GPIO_PIN_15,GPIO_PIN_RESET);
		}
}

/*电机停止函数*/
void Stop_protect(float *Med_Jiaodu,float *Jiaodu)
{
	if((*Jiaodu-*Med_Jiaodu)>40)
	{
//		set_speed(0,0);
		stop=1;
	}
	else if((*Jiaodu-*Med_Jiaodu)<-40)
	{
//		set_speed(0,0);
		stop=1;
	}
	else if((*Jiaodu-*Med_Jiaodu)>-40 && (*Jiaodu-*Med_Jiaodu)<40)
	{
		stop=0;
	}
}







