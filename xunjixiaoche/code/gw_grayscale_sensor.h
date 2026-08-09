#ifndef INC_GW_GRAYSCALE_SENSOR_H_
#define INC_GW_GRAYSCALE_SENSOR_H_

#include <stdint.h>

/* 八路红外巡线模块地址：硬件 IIC 例程使用 (IR_ADDRESS << 1)。 */
#define IR_ADDRESS 0x12U

/* 八路红外巡线模块寄存器：0x30 读数据，0x01 进入/退出校准。 */
#define IR_DATA_REG 0x30U
#define IR_ADJUST_REG 0x01U

#endif /* INC_GW_GRAYSCALE_SENSOR_H_ */
