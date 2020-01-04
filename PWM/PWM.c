/**
  ******************************************************************************
  * File Name          : PWM.c
  * Description        : PWM for motor
  ******************************************************************************
  */

/**
 * author Thanh Ta Minh, Ho Chi Minh University of Technology, Vietnam
	  Nguyen Tien Dat Tran, Ho Chi Minh University of Technology, Vietnam
 */
#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include "define.h"
#include "PWM.h"
TIM_HandleTypeDef TIM_MOTORLEFT_PWM_Handle;
TIM_HandleTypeDef TIM_MOTORRIGHT_PWM_Handle;

/* TIM4 init function */
void MX_TIM4_PWM_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  TIM_MOTORLEFT_PWM_Handle.Instance = PWM_MOTORLEFT_TIMERBASE;
  TIM_MOTORLEFT_PWM_Handle.Init.Prescaler = (uint16_t) ((SystemCoreClock / 4) / TICK_PERIOD_MotorLeft) - 1;
  TIM_MOTORLEFT_PWM_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
  TIM_MOTORLEFT_PWM_Handle.Init.Period = RELOAD_PERIOD_MotorLeft;
  TIM_MOTORLEFT_PWM_Handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&TIM_MOTORLEFT_PWM_Handle) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&TIM_MOTORLEFT_PWM_Handle, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&TIM_MOTORLEFT_PWM_Handle) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&TIM_MOTORLEFT_PWM_Handle, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&TIM_MOTORLEFT_PWM_Handle, &sConfigOC, PWM_MOTORLEFT_CHANNEL) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&TIM_MOTORLEFT_PWM_Handle);
}
/* TIM9 init function */
void MX_TIM9_PWM_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_OC_InitTypeDef sConfigOC;

  TIM_MOTORRIGHT_PWM_Handle.Instance = PWM_MOTORRIGHT_TIMERBASE;
  TIM_MOTORRIGHT_PWM_Handle.Init.Prescaler = (uint16_t) ((SystemCoreClock / 2) / TICK_PERIOD_MotorRight) - 1;
  TIM_MOTORRIGHT_PWM_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
  TIM_MOTORRIGHT_PWM_Handle.Init.Period = RELOAD_PERIOD_MotorRight;
  TIM_MOTORRIGHT_PWM_Handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&TIM_MOTORRIGHT_PWM_Handle) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&TIM_MOTORRIGHT_PWM_Handle, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&TIM_MOTORRIGHT_PWM_Handle) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&TIM_MOTORRIGHT_PWM_Handle, &sConfigOC, PWM_MOTORRIGHT_CHANNEL) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&TIM_MOTORRIGHT_PWM_Handle);

}
//pulse_width : (0,1)
void TIM_PWM_SetPulseWidth_SetDir(double db_Pulse_Width, MotorDir_Type dir, Motor_HandleTypedef* Motor_Handle)
{

			//HAL_TIM_PWM_Stop(Motor_Handle->PWM_Motor_Handle,Motor_Handle->PWM_Motor_Channel);			
			__HAL_TIM_SetCompare(Motor_Handle->PWM_Motor_Handle,Motor_Handle->PWM_Motor_Channel,Motor_Handle->PWM_Motor_Handle->Init.Period*db_Pulse_Width);
			switch(dir)
			{
				//not yet tested
				case GO_FORWARD:
					HAL_GPIO_WritePin(Motor_Handle->DIR_Motor_Port,Motor_Handle->DIR_Motor_Pinsource,GPIO_PIN_SET);
					break;	
				case GO_BACKWARD:
					HAL_GPIO_WritePin(Motor_Handle->DIR_Motor_Port,Motor_Handle->DIR_Motor_Pinsource,GPIO_PIN_RESET);
					break;
				default:
					break;		
			}
			HAL_TIM_PWM_Start(Motor_Handle->PWM_Motor_Handle,Motor_Handle->PWM_Motor_Channel);
}
