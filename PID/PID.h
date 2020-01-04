#ifndef PID_H
#define PID_H

/* Includes ------------------------------------------------------------------*/
#include "define.h"
#include <stdint.h>
#include <stdbool.h>
#include "CommonDefine.h"
typedef struct{
	Motor_Type Name;
	double Kp;
	double Ki;
	double Kd;
	double Ts;
	double Error;
	double Error_1;
	double Error_2;
	double Omega;
	double Setpoint;
	double Result;
	double Result_1;
	double PID_Duty_Cycle;
	double Stable_Angle;
	double Error_Tolerance;	//sai so cho phep
	double Result_Limit;
//add filter here 		
}PID_TypeDef;

double PID_Calculate(PID_TypeDef *PID_Type, double Angle_Feeback, double Rate_Feedback);

void PID_Reset_Param(PID_TypeDef *PID_Type);
void PID_Set_Param(PID_TypeDef *PID_Type, uint8_t Name, double Kp, double Ki, double Kd, double Setpoint, double Result_1, double Omega, double Ts);
void PID_Calib_Stable_Angle(PID_TypeDef *PID_Type, double Stable_Angle);

#endif /* PID_H_ */

