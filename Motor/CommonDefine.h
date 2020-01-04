#ifndef COMMON_H
#define COMMON_H
#include "stm32f4xx_hal.h"

typedef enum {
	LEFT_MOTOR = 0,
	RIGHT_MOTOR ,
	ANGLE,
} Motor_Type;

typedef enum {
	UNKNOWN=0,
	GO_FORWARD ,
	GO_BACKWARD,
} MotorDir_Type;


#endif /* COMMON_H */

