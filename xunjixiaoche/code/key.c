#include "key.h"

bool key_occur;

uint8_t key_select(){
	bool B1_state = HAL_GPIO_ReadPin(GPIOD,GPIO_PIN_8);
	if(key_occur == 0 && B1_state == 0){
		key_occur = 1;
		HAL_Delay(5);
		if(B1_state == 0){
			return key_B1;
		}
	}
	else if(B1_state)
		key_occur = 0;
	return keyno;
}

void key_control(){
	uint8_t key_value = key_select();
	switch(key_value){
		case key_B1 : star_car = !star_car;break;
	}
}
