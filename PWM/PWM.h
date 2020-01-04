#ifndef PWM_H
#define PWM_H
#include "stm32f4xx_hal.h"
#include "Motor.h"


extern TIM_HandleTypeDef TIM_MOTORLEFT_PWM_Handle;
extern TIM_HandleTypeDef TIM_MOTORRIGHT_PWM_Handle;

void TIM_PWM_SetPulseWidth_SetDir(double db_Pulse_Width, MotorDir_Type dir, Motor_HandleTypedef* Motor_Handle);
void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim);


void MX_TIM4_PWM_Init(void);
void MX_TIM9_PWM_Init(void);

#endif
