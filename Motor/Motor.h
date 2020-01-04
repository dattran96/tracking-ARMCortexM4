#ifndef MOTOR_H
#define MOTOR_H
#include "stm32f4xx_hal.h"
#include "CommonDefine.h"
#include "PID.h"


//contain the information for which channel is actived and number of cycle repetition of timer base
typedef struct{
	TIM_TypeDef* IC_State_Motor_TimBase;
	Motor_Type IC_State_Motor;
	uint32_t IC_State_Active_Channel;
	uint16_t IC_State_Period_Repetition;
}IC_Motor_State_TypeDef;

// for counting pulse of each motor
typedef struct{
	double Pulse_Count_t1;
	double Pulse_Count_t2;
	uint16_t Pulse_Count_Repetition;
	uint16_t Pulse_Count_Repetition_Temp;
	uint8_t PulseCount_Sampling_Time; //ms
}Motor_Pulse_Count_TypeDef;


typedef struct{
	Motor_Type Motor_Name;
	uint32_t ENC_PulseperRound_AB_BothEdge;
	
	
	TIM_HandleTypeDef* PWM_Motor_Handle;
	TIM_TypeDef * PWM_Motor_Tim_Base;
	uint16_t PWM_Motor_Pinsource;
	uint8_t PWM_Motor_AF;
	GPIO_TypeDef * PWM_Motor_Port;
	uint32_t PWM_Motor_Channel;
	
	MotorDir_Type Motor_Dir;
	uint16_t DIR_Motor_Pinsource;
	GPIO_TypeDef * DIR_Motor_Port;
	
	double Motor_Position;
	double Position_Pulse_Count;			//cong don so xung dem duoc
	TIM_HandleTypeDef* Position_Motor_Handle;
	TIM_TypeDef * Position_Motor_Tim_Base;
	Motor_Pulse_Count_TypeDef* Motor_Pulse_Count;
	
	double Motor_Veclocity;
	TIM_HandleTypeDef* Veclocity_Motor_Handle;
	TIM_TypeDef *  Veclocity_Motor_Tim_Base;
	IC_Motor_State_TypeDef * IC_Motor_State;
	
	PID_TypeDef* PID_Motor_Angle_Handle;
	PID_TypeDef* PID_Motor_VELOCITY_Handle;
	
}Motor_HandleTypedef;

	

#endif /* MOTOR_H_ */

