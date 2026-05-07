#ifndef _KEY_H
#define _KEY_H

#include "stm32f4xx_hal.h"
#include "stdint.h"
#include "stdbool.h"
#include "pid.h"

typedef enum{
	key_B1,
	keyno
}key;
void key_control();

extern bool car_state;

#endif
