/**
  ******************************************************************************
  * File Name          : Encoder_Velocity.c
  * Description        : Read data from Encoder mounted on Motor, then process and get Velocity
  ******************************************************************************
  */

/**
 * author Nguyen Tien Dat Tran, Ho Chi Minh University of Technology, Vietnam
 */

#include "stm32f4xx_hal.h"
#include "PWM.h"
#include <stdbool.h>
#include "define.h"
#include "Encoder_Velocity.h"
#include "Control.h"

TIM_HandleTypeDef TIM_MOTORRIGHT_VELOCITY_Handle;
TIM_HandleTypeDef TIM_MOTORLEFT_VELOCITY_Handle;
IC_Motor_State_TypeDef IC_Motor_Left_State;
IC_Motor_State_TypeDef IC_Motor_Right_State;

int test_remove = 1;

// 1 la 8, 2 la 3
//remember to config 2 IC_MOtor_State_Typedef


void IC_Motor_State_Init(void)
{
	IC_Motor_Left_State.IC_State_Motor= LEFT_MOTOR;
	IC_Motor_Left_State.IC_State_Motor_TimBase= IC_MOTORLEFT_TIMERBASE;
	
	IC_Motor_Right_State.IC_State_Motor = RIGHT_MOTOR;
	IC_Motor_Right_State.IC_State_Motor_TimBase= IC_MOTORRIGHT_TIMERBASE;
}
void Set_Para_IC_Motor_State(IC_Motor_State_TypeDef* IC_Motor_State,TIM_TypeDef*IC_State_Motor_TimBase,Motor_Type IC_State_Motor)
{
	IC_Motor_State->IC_State_Motor_TimBase= IC_State_Motor_TimBase;
	IC_Motor_State->IC_State_Motor= IC_State_Motor;
}
void Get_Motor_Veclocity_BasedOn_IC(Motor_HandleTypedef* Motor_Handle )
{
		double CCR1 =0;
		double CCR2 =0;
		double temp=0;
		CCR1 = Motor_Handle->Veclocity_Motor_Tim_Base->CCR1;
		CCR2 = Motor_Handle->Veclocity_Motor_Tim_Base->CCR2;
		if(Motor_Handle->IC_Motor_State->IC_State_Active_Channel == HAL_TIM_ACTIVE_CHANNEL_1)
		{
			temp= CCR1-CCR2;
			if (temp < 0)
			{
				if (Motor_Handle->IC_Motor_State->IC_State_Period_Repetition ==0)
					temp = temp*-1;
			}	
			Motor_Handle->Motor_Veclocity = ((double)1.0/(double)Motor_Handle->ENC_PulseperRound_AB_BothEdge)/((double)(temp+Motor_Handle->IC_Motor_State->IC_State_Period_Repetition*Motor_Handle->Veclocity_Motor_Handle->Init.Period)*((double)1.0/((double)60.0*IC_TICK_PERIOD_MOTOR_LEFT)));
		}
		if(Motor_Handle->IC_Motor_State->IC_State_Active_Channel == HAL_TIM_ACTIVE_CHANNEL_2)
		{
			temp= CCR2-CCR1;
			if (temp < 0)
			{
				if (Motor_Handle->IC_Motor_State->IC_State_Period_Repetition ==0)
					temp = temp*-1;
			}
			Motor_Handle->Motor_Veclocity = ((double)1.0/(double)Motor_Handle->ENC_PulseperRound_AB_BothEdge)/((double)(temp+Motor_Handle->IC_Motor_State->IC_State_Period_Repetition*Motor_Handle->Veclocity_Motor_Handle->Init.Period)*((double)1.0/((double)60.0*IC_TICK_PERIOD_MOTOR_LEFT)));
		}
		
			
}
/* TIM1 init function */
void MX_MOTORRIGHT_IC_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_IC_InitTypeDef sConfigIC;

  TIM_MOTORRIGHT_VELOCITY_Handle.Instance = IC_MOTORRIGHT_TIMERBASE;
  TIM_MOTORRIGHT_VELOCITY_Handle.Init.Prescaler = (uint16_t) ((SystemCoreClock / 2) / IC_TICK_PERIOD_MOTOR_RIGHT) - 1;
  TIM_MOTORRIGHT_VELOCITY_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
  TIM_MOTORRIGHT_VELOCITY_Handle.Init.Period = IC_RELOAD_PERIOD_MOTOR_RIGHT;
  TIM_MOTORRIGHT_VELOCITY_Handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  TIM_MOTORRIGHT_VELOCITY_Handle.Init.RepetitionCounter = 0;
  if (HAL_TIM_Base_Init(&TIM_MOTORRIGHT_VELOCITY_Handle) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&TIM_MOTORRIGHT_VELOCITY_Handle, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_IC_Init(&TIM_MOTORRIGHT_VELOCITY_Handle) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&TIM_MOTORRIGHT_VELOCITY_Handle, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&TIM_MOTORRIGHT_VELOCITY_Handle, &sConfigIC, IC_MOTORRIGHT_CHANNEL1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_IC_ConfigChannel(&TIM_MOTORRIGHT_VELOCITY_Handle, &sConfigIC, IC_MOTORRIGHT_CHANNEL2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM2 init function */
void MX_MOTORLEFT_IC_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_IC_InitTypeDef sConfigIC;

  TIM_MOTORLEFT_VELOCITY_Handle.Instance = IC_MOTORLEFT_TIMERBASE;
  TIM_MOTORLEFT_VELOCITY_Handle.Init.Prescaler = (uint16_t) ((SystemCoreClock / 4) / IC_TICK_PERIOD_MOTOR_LEFT) - 1;
  TIM_MOTORLEFT_VELOCITY_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
  TIM_MOTORLEFT_VELOCITY_Handle.Init.Period = IC_RELOAD_PERIOD_MOTOR_LEFT;
  TIM_MOTORLEFT_VELOCITY_Handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&TIM_MOTORLEFT_VELOCITY_Handle) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&TIM_MOTORLEFT_VELOCITY_Handle, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_IC_Init(&TIM_MOTORLEFT_VELOCITY_Handle) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&TIM_MOTORLEFT_VELOCITY_Handle, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&TIM_MOTORLEFT_VELOCITY_Handle, &sConfigIC, IC_MOTORLEFT_CHANNEL1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_IC_ConfigChannel(&TIM_MOTORLEFT_VELOCITY_Handle, &sConfigIC, IC_MOTORLEFT_CHANNEL2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
}
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == IC_Motor_Left_State.IC_State_Motor_TimBase)
	{
		IC_Motor_Left_State.IC_State_Active_Channel = htim->Channel;
		IC_Motor_Left_State.IC_State_Period_Repetition=0;
		test_remove=10;
	}
	if(htim->Instance == IC_Motor_Right_State.IC_State_Motor_TimBase)
	{
		IC_Motor_Right_State.IC_State_Active_Channel=htim->Channel;
		IC_Motor_Right_State.IC_State_Period_Repetition=0;
		test_remove=15;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	//Input Capture Overflow
	if(htim->Instance== IC_Motor_Left_State.IC_State_Motor_TimBase )
		IC_Motor_Left_State.IC_State_Period_Repetition++;
	if(htim->Instance == IC_Motor_Right_State.IC_State_Motor_TimBase)
		IC_Motor_Right_State.IC_State_Period_Repetition++;
	
	//Encoder Overflow
	if(htim->Instance == MOTOR_LEFT_HANDLE.Position_Motor_Tim_Base)
	{
		MOTOR_LEFT_HANDLE.Motor_Pulse_Count->Pulse_Count_Repetition_Temp++;
		test_remove = 20;
	}
	if(htim->Instance == MOTOR_RIGHT_HANDLE.Position_Motor_Tim_Base)
	{
		MOTOR_RIGHT_HANDLE.Motor_Pulse_Count->Pulse_Count_Repetition_Temp++;
		test_remove = 25;
	}

}

