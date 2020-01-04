/**
  ******************************************************************************
  * File Name          : Encoder_Position.c
  * Description        : Read data from Encoder mounted on Motor, then process and get Motor Position
  ******************************************************************************
  */

/**
 * author Nguyen Tien Dat Tran, Ho Chi Minh University of Technology, Vietnam
 */
#include "stm32f4xx_hal.h"
#include <stdbool.h>
#include "define.h"
#include "Motor.h"
#include "Encoder_Position.h"
#include "Control.h"
TIM_HandleTypeDef TIM_MOTORLEFT_POSITION_Handle;
TIM_HandleTypeDef TIM_MOTORRIGHT_POSITION_Handle;
Motor_Pulse_Count_TypeDef MOTORLEFT_Pulse_Count;
Motor_Pulse_Count_TypeDef MOTORRIGHT_Pulse_Count;


/* TIM3 init function */

void MOTOR_PULSE_COUNT_Init(void)
{
	MOTORLEFT_Pulse_Count.Pulse_Count_t2=0;
	MOTORLEFT_Pulse_Count.Pulse_Count_t1=0;
	MOTORLEFT_Pulse_Count.Pulse_Count_Repetition=0;
	MOTORLEFT_Pulse_Count.Pulse_Count_Repetition_Temp=0;
	MOTORLEFT_Pulse_Count.PulseCount_Sampling_Time= ENC_MOTORLEFT_PulseCount_Sampling_Time;
	
	MOTORRIGHT_Pulse_Count.Pulse_Count_t2=0;
	MOTORRIGHT_Pulse_Count.Pulse_Count_t1=0;
	MOTORRIGHT_Pulse_Count.Pulse_Count_Repetition=0;
	MOTORRIGHT_Pulse_Count.Pulse_Count_Repetition_Temp=0;
	MOTORRIGHT_Pulse_Count.PulseCount_Sampling_Time= ENC_MOTORRIGHT_PulseCount_Sampling_Time;

}
void PreProcess_Get_Velocity_IRQHandler(Motor_HandleTypedef* Motor_Handle)
{	
	Motor_Handle->Motor_Pulse_Count->PulseCount_Sampling_Time--;
	if(Motor_Handle->Motor_Pulse_Count->PulseCount_Sampling_Time==0)
	{
	Motor_Handle->Motor_Pulse_Count->Pulse_Count_t1 = Motor_Handle->Motor_Pulse_Count->Pulse_Count_t2;
	Motor_Handle->Motor_Pulse_Count->Pulse_Count_t2 = Motor_Handle->Position_Motor_Tim_Base->CNT;
	Motor_Handle->Motor_Pulse_Count->Pulse_Count_Repetition = Motor_Handle->Motor_Pulse_Count->Pulse_Count_Repetition_Temp;
	Motor_Handle->Motor_Pulse_Count->Pulse_Count_Repetition_Temp =0 ;
	Motor_Handle->Motor_Pulse_Count->PulseCount_Sampling_Time = ENC_MOTORLEFT_PulseCount_Sampling_Time;
	}
}
//chua lay dc Pulse_Count_Repetition nen viet chua chay, chi dung khi toc do < 1 vong/ms
void Get_Velocity_Basedon_PulseCount(Motor_HandleTypedef* Motor_Handle)
{
	double temp =0;
	temp = Motor_Handle->Motor_Pulse_Count->Pulse_Count_t2-Motor_Handle->Motor_Pulse_Count->Pulse_Count_t1;
	if( temp >= 0)
	{
		if( temp > Motor_Handle->ENC_PulseperRound_AB_BothEdge/2) //overflow
			Motor_Handle->Motor_Veclocity = ((-temp+1.0*(double)Motor_Handle->Position_Motor_Handle->Init.Period)/(double)(Motor_Handle->ENC_PulseperRound_AB_BothEdge))/((double)Motor_Handle->Motor_Pulse_Count->PulseCount_Sampling_Time/(60.0*1000.0));	// RPM
		else
			Motor_Handle->Motor_Veclocity = ((temp)/(double)(Motor_Handle->ENC_PulseperRound_AB_BothEdge))/((double)Motor_Handle->Motor_Pulse_Count->PulseCount_Sampling_Time/(60.0*1000.0));	// RPM
	}
	else // 2 truong hop, quay nguoc hoac la overflow
	{
		if( temp*-1 > Motor_Handle->ENC_PulseperRound_AB_BothEdge/2) //overflow
			Motor_Handle->Motor_Veclocity = ((temp+1.0*(double)Motor_Handle->Position_Motor_Handle->Init.Period)/(double)(Motor_Handle->ENC_PulseperRound_AB_BothEdge))/((double)Motor_Handle->Motor_Pulse_Count->PulseCount_Sampling_Time/(60.0*1000.0));	// RPM
		else
			Motor_Handle->Motor_Veclocity = ((temp*-1)/(double)(Motor_Handle->ENC_PulseperRound_AB_BothEdge))/((double)Motor_Handle->Motor_Pulse_Count->PulseCount_Sampling_Time/(60.0*1000.0));	// RPM
		//Motor_Handle->Motor_Veclocity = Motor_Handle->Motor_Veclocity*-1;
	}
}
/*Unit :  m/s */
/*Wheel Radius : 8.5 cm*/
double Get_Velocity_Average(void)
{
	double temp = 0;
	Get_Velocity_Basedon_PulseCount(&MOTOR_LEFT_HANDLE);
	Get_Velocity_Basedon_PulseCount(&MOTOR_RIGHT_HANDLE);
	temp = ((double)MOTOR_LEFT_HANDLE.Motor_Veclocity + (double)MOTOR_RIGHT_HANDLE.Motor_Veclocity)/2.0*(1.0/60.0)*(2.0*3.1415926535*8.5*0.01);
	return temp;
}

void MX_TIM3_Encoder_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  TIM_MOTORLEFT_POSITION_Handle.Instance = ENC_MOTORLEFT_TIMERBASE;
  TIM_MOTORLEFT_POSITION_Handle.Init.Prescaler = 0;
  TIM_MOTORLEFT_POSITION_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
  TIM_MOTORLEFT_POSITION_Handle.Init.Period = ENC_MOTORLEFT_PulseperRound_AB_BothEdge;
  TIM_MOTORLEFT_POSITION_Handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&TIM_MOTORLEFT_POSITION_Handle, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&TIM_MOTORLEFT_POSITION_Handle, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}
/* TIM8 init function */
void MX_TIM8_Encoder_Init(void)
{

  TIM_Encoder_InitTypeDef sConfig;
  TIM_MasterConfigTypeDef sMasterConfig;

  TIM_MOTORRIGHT_POSITION_Handle.Instance = ENC_MOTORRIGHT_TIMERBASE;
  TIM_MOTORRIGHT_POSITION_Handle.Init.Prescaler = 0;
  TIM_MOTORRIGHT_POSITION_Handle.Init.CounterMode = TIM_COUNTERMODE_UP;
  TIM_MOTORRIGHT_POSITION_Handle.Init.Period = ENC_MOTORRIGHT_PulseperRound_AB_BothEdge;
  TIM_MOTORRIGHT_POSITION_Handle.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  TIM_MOTORRIGHT_POSITION_Handle.Init.RepetitionCounter = 0;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 0;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 0;
  if (HAL_TIM_Encoder_Init(&TIM_MOTORRIGHT_POSITION_Handle, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&TIM_MOTORRIGHT_POSITION_Handle, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }
}

//void Set_Reference_Point_Encoder(Motor_HandleTypedef* Motor_Handle)
//{
//	__HAL_TIM_SetCounter(Motor_Handle->Position_Motor_Handle,0);
//}

// Prepare for counting Pulse
void Set_Reference_Point_Encoder(Motor_HandleTypedef* Motor_Handle)
{
	Motor_Handle->Position_Pulse_Count = 0;
}
void Get_Position(Motor_HandleTypedef* Motor_Handle)
{
	Motor_Handle->Motor_Position = (double)(Motor_Handle->Position_Motor_Tim_Base->CNT)/(double)(Motor_Handle->ENC_PulseperRound_AB_BothEdge)*(double)360;
}	
// NOT YET TESTED
void Get_DIR(Motor_HandleTypedef* Motor_Handle)
{
	if ( (Motor_Handle->Position_Motor_Tim_Base->CR1 & 0x0010) != 0)
		Motor_Handle->Motor_Dir = GO_FORWARD;
	else
		Motor_Handle->Motor_Dir = GO_BACKWARD;
}
