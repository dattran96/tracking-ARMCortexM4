#ifndef EncoderVelocity_H
#define EncoderVelocity_H
#include "stm32f4xx_hal.h"
#include "Motor.h"

extern IC_Motor_State_TypeDef IC_Motor_Left_State;
extern IC_Motor_State_TypeDef IC_Motor_Right_State;
extern TIM_HandleTypeDef TIM_MOTORRIGHT_VELOCITY_Handle;
extern TIM_HandleTypeDef TIM_MOTORLEFT_VELOCITY_Handle;
void MX_MOTORRIGHT_IC_Init(void);
void MX_MOTORLEFT_IC_Init(void);
void IC_Motor_State_Init(void);
void Set_Para_IC_Motor_State(IC_Motor_State_TypeDef* IC_Motor_State,TIM_TypeDef*IC_State_Motor_TimBase,Motor_Type IC_State_Motor);
void Get_Motor_Veclocity_BasedOn_IC(Motor_HandleTypedef* Motor_Handle );


typedef enum {
	Channel_A = 0,
	Channel_B,
} IC_Motor_Active_Channel_TypeDef;




#endif
