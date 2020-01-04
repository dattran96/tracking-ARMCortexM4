#ifndef EncoderPosition_H
#define EncoderPosition_H

#include "stm32f4xx_hal.h"
#include "Motor.h"

extern TIM_HandleTypeDef TIM_MOTORLEFT_POSITION_Handle;
extern TIM_HandleTypeDef TIM_MOTORRIGHT_POSITION_Handle;
extern Motor_Pulse_Count_TypeDef MOTORLEFT_Pulse_Count;
extern Motor_Pulse_Count_TypeDef MOTORRIGHT_Pulse_Count;


void MX_TIM3_Encoder_Init(void);
void MX_TIM8_Encoder_Init(void);
void MOTOR_PULSE_COUNT_Init(void);
void Set_Reference_Point_Encoder(Motor_HandleTypedef* Motor_Handle);
void Get_Position(Motor_HandleTypedef* Motor_Handle);
void Get_DIR(Motor_HandleTypedef* Motor_Handle);
void PreProcess_Get_Velocity_IRQHandler(Motor_HandleTypedef* Motor_Handle);
void Get_Velocity_Basedon_PulseCount(Motor_HandleTypedef* Motor_Handle);
double Get_Velocity_Average(void);

#endif
