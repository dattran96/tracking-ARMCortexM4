#include "PID.h"
#include <stdbool.h>
#include "define.h"
#include "Control.h"
 double a_pid,b_pid,c_pid = 0;
//PID_Control low level
double PID_Calculate(PID_TypeDef *PID_Type, double Angle_Feeback, double Rate_Feedback)
{
	//double a, b, c; 
	double Angle_error;
#ifdef PID_METHOD1
	/* Control angle by applying PID controller to its "Rate":
	 *	Desired_Angle ----->(+-)------------------->(+-)------>| PID_Controller |------->(Steering_Angle)
	 *											 /|\                     /|\
	 *	Measured_Angle----------------------------|						  |
	 *																	  |
	 *	Angle_Rate------------| Omega >-----------------------------------|
	 */
	
	/* Note: Calculate below parameters based on the concept not holding desired error */
	/* PID ANGLE OR PID SPEED*/
	if (PID_Type->Name == ANGLE)
	{
		Angle_error = PID_Type->Setpoint - Angle_Feeback;
		if ( Angle_error > 180.0 )
			Angle_error = Angle_error-360.0;
		else if (Angle_error <-180 )
			Angle_error = Angle_error + 360.0;
		
		if(Angle_error < 0)
			Turn_Right = 0;
		else
			Turn_Right =1 ;
	}
		
	else
		Angle_error = PID_Type->Setpoint - Angle_Feeback;
	
	
	
	PID_Type->Error = Angle_error - Rate_Feedback*PID_Type->Omega;
	if ((PID_Type->Error < PID_Type->Error_Tolerance) && (PID_Type->Error > -PID_Type->Error_Tolerance))
{
			PID_Type->Error = 0.0;
//		//PID_Type->Result = 0.0;
//		//PID_Type->Result_1 = 0.0;
	}
//	if ((PID_Type->Error > LOWER_PITCH_ERR_THRES) || (PID_Type->Error < -LOWER_PITCH_ERR_THRES))
//		PID_Set_Param(PID_Type, NO_CHANGE, 1.5*KP_PITCH, 1.5*KI_PITCH, NO_CHANGE, NO_CHANGE, NO_CHANGE, NO_CHANGE, NO_CHANGE);
//	else
//		PID_Set_Param(PID_Type, NO_CHANGE, KP_PITCH, KI_PITCH, NO_CHANGE, NO_CHANGE, NO_CHANGE, NO_CHANGE, NO_CHANGE);
#endif

	a_pid = (PID_Type->Kp + PID_Type->Ki*PID_Type->Ts/2 + PID_Type->Kd/PID_Type->Ts);
	b_pid = (-PID_Type->Kp + PID_Type->Ki*PID_Type->Ts/2 - 2*PID_Type->Kd/PID_Type->Ts);
	c_pid = PID_Type->Kd/PID_Type->Ts;
	
	PID_Type->Result = PID_Type->Result_1 + a_pid*PID_Type->Error + b_pid*PID_Type->Error_1 + c_pid*PID_Type->Error_2;
	//PID_Type->Result = PID_Type->Kp*PID_Type->Error;

	if (PID_Type->Result >= PID_Type->Result_Limit)
		PID_Type->Result = PID_Type->Result_Limit;
	else if (PID_Type->Result <= -PID_Type->Result_Limit)
		PID_Type->Result = -PID_Type->Result_Limit;
	
	PID_Type->Result_1 = PID_Type->Result;
	PID_Type->Error_1 = PID_Type->Error;
	PID_Type->Error_2 = PID_Type->Error_1;
	/* Calculate PWM value and assign to PID_Type->PWM*/	
	//PID_Type->WM = (uint16_t) ((PID_Type->Result + 1.0)/2.0*(800.0 - 400.0) + 400.0);
	//PID_Type->PID_Duty_Cycle = PID_Type->Result/PID_Type->Result_Limit;
	PID_Type->PID_Duty_Cycle = PID_Type->Result;
	return PID_Type->PID_Duty_Cycle;
}

void PID_Set_Param(PID_TypeDef *PID_Type, uint8_t Name, double Kp, double Ki, double Kd, double Setpoint, double Result_1, double Omega, double Ts) 
{
	//if (Name != NO_CHANGE) PID_Type->Name = Name;
	if (Kp != NO_CHANGE) PID_Type->Kp = Kp;
	if (Ki != NO_CHANGE) PID_Type->Ki = Ki;
	if (Kd != NO_CHANGE) PID_Type->Kd = Kd;
	if (Setpoint != NO_CHANGE) PID_Type->Setpoint = Setpoint;
	if (Result_1 != NO_CHANGE) PID_Type->Result_1 = Result_1;
	if (Omega != NO_CHANGE) PID_Type->Omega = Omega;
	if (Ts != NO_CHANGE) PID_Type->Ts = Ts;
}
void PID_Calib_Stable_Angle(PID_TypeDef *PID_Type, double Stable_Angle)
{
	PID_Type->Stable_Angle = Stable_Angle;
}
