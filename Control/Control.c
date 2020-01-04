/**
  ******************************************************************************
  * File Name          : control.c
  * Description        : Control the motor's velocity and position.
			 Steeling control of vehicle using Stanley path planning to track the predetermined trajectory.
  ******************************************************************************
  */

/**
 * author Thanh Ta Minh, Ho Chi Minh University of Technology, Vietnam
	  Nguyen Tien Dat Tran, Ho Chi Minh University of Technology, Vietnam
 */
#include "PID.h"
#include <stdbool.h>
#include "define.h"
#include "Control.h"
#include "Motor.h"
#include "PWM.h"
#include "Encoder_Position.h"
#include "Encoder_Velocity.h"
#include "UTM.h"
#include <stdbool.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>
#include "gps.h"
#include "Stanley.h"
Motor_HandleTypedef MOTOR_LEFT_HANDLE;
Motor_HandleTypedef MOTOR_RIGHT_HANDLE;

PID_TypeDef MOTOR_LEFT_Angle_PID_HANDLE; //angle
PID_TypeDef MOTOR_RIGHT_Angle_PID_HANDLE; //angle

PID_TypeDef MOTOR_LEFT_Speed_PID_HANDLE; //speed
PID_TypeDef MOTOR_RIGHT_Speed_PID_HANDLE; //speed

SowingMachine_Control_Mode MODE = AUTO_MODE;
SowingMachine_Control_State STATE = DECISION_STATE ;
SowingMachine_Control_State PREVIOUS_STATE = UNKNOWN_STATE;
SowingMachine_Control_State COMMAND_STATE=UNKNOWN_STATE;

double pwm0 = 0;
double pwm1 = 0; 
PID_TypeDef PID_PWM_R  = {RIGHT_MOTOR,PID_MOTORRIGHT_Speed_KP,PID_MOTORRIGHT_Speed_KI,PID_MOTORRIGHT_Speed_KD,PID_MOTORRIGHT_Speed_Ts,0,0,0,0,0,0,0,0,0,PID_MOTORRIGHT_Speed_Error_Tolerance,PID_MOTORRIGHT_Speed_Limit};

uint32_t tick_start_Speed = 0;
uint32_t tick_start_Angle=0;
PID_TypeDef PID_Angle  = {ANGLE,PID_MOTOR_ANGLE_KP,PID_MOTOR_ANGLE_KI,PID_MOTOR_ANGLE_KD,PID_MOTOR_ANGLE_Ts,0,0,0,0,0,0,0,0,0,PID_MOTOR_ANGLE_Error_Tolerance,PID_MOTOR_ANGLE_Limit };
PID_TypeDef PID_PWM_L  = {LEFT_MOTOR,PID_MOTORLEFT_Speed_KP,PID_MOTORLEFT_Speed_KI,PID_MOTORLEFT_Speed_KD,PID_MOTORLEFT_Speed_Ts,0,0,0,0,0,0,0,0,0,PID_MOTORLEFT_Speed_Error_Tolerance,PID_MOTORLEFT_Speed_Limit};
double Temp1=0;
double Temp2=0;
double Temp3=0;
uint32_t Test_1 = 100;
bool Temp3_Test = false;
bool FLAG_START ;
bool FLAG_TESTING = false;
uint32_t tick_measure_1 = 0;
uint32_t tick_measure_2 = 0;

//ghe da
//double dTrajectory [Trajectory_Size][2] = {{10.772842050, 106.659266800},{10.772843240, 106.659269390},{10.772845690,106.659273500}};

// dang sau ghe da, duong di ra circle K
//double dTrajectory [Trajectory_Size][2]= {{ 10.77285833333,106.6591433333,},{10.77285283333,106.6591423333},{10.77285066667,106.6591668333}};

//double dTrajectory [Trajectory_Size][2]= {{ 10.77284985016667,106.6591531667},{10.77281866667,106.6591971667},{10.77285066667,106.6591668333}};

// song song ly thuong kiet
//double dTrajectory [Trajectory_Size][2]= {{ 106.6591089,10.7729926},{106.6591265,10.7728796},{106.659273500,10.772845690}};
//double dTrajectory [Trajectory_Size][2]= {{ 10.7729926,106.6591089},{10.7728796,106.6591265},{10.772845690,106.659273500}};
//vuong goc ly thuong kiet
//double dTrajectory [Trajectory_Size][2]= {{ 10.7726657,106.6593005},{10.7726482,106.6591817},{10.772845690,106.659273500}};

//test stanley
//double dTrajectory [ Trajectory_Size][2] ={{10.772870487,106.659145173},{10.772847441,106.659181501},{10.772826316,106.659178228},{10.772821325,106.659177253},{10.772820152,106.659158705}};
//duong san banh

//double dTrajectory [ Trajectory_Size][2] ={{10.772710717,106.659270821},{10.772676123,106.659299882},{10.772638314,106.659323221}};
// 10.772664091, 106.659312513 , diem test ben trai goc 195 quy dao
//330
double dTrajectory [ Trajectory_Size][2] ={{10.772853252,106.659865095},{10.772868829,106.659854961},{10.772884802,106.659842500},{10.772901727,106.659831876}};

//150
//double dTrajectory [ Trajectory_Size][2] ={{10.772901727,106.659831876},{10.772884802,106.659842500},{10.772868829,106.659854961},{10.772853252,106.659865095}};


//ben phai quy dao, thung rac
//106.6592094,10.7727699
double dTrajectory_UTM [Trajectory_Size][2];
uint8_t ui8_Utmzone[Trajectory_Size][2];
uint32_t ui32_Trajectory_Size = Trajectory_Size;
uint32_t ui32_Trajectory_Index = 0;
int WayPoint[2][2] = {{0,1},{0,2}};

//test
double vleft =0,vright=0 , v_offset = 0 , v_init =20;
double Cur_Angle = 0;
double SetPoint_Angle =	0;
uint8_t Turn_Right = 0;

double Vehicle_Coordinate[1][2];
double Vehicle_Coordinate_UTM[1][2];
uint8_t Vehicle_Utmzone[1][2];

//create 2 object for 2 motor left and right
void Motor_Handle_Init(void)
{
	MOTOR_LEFT_HANDLE.Motor_Name = LEFT_MOTOR;
	MOTOR_LEFT_HANDLE.ENC_PulseperRound_AB_BothEdge = ENC_MOTORLEFT_PulseperRound_AB_BothEdge;
	MOTOR_LEFT_HANDLE.PWM_Motor_Handle = &TIM_MOTORLEFT_PWM_Handle;
	MOTOR_LEFT_HANDLE.PWM_Motor_Tim_Base= PWM_MOTORLEFT_TIMERBASE;
	MOTOR_LEFT_HANDLE.PWM_Motor_Pinsource=PWM_MOTORLEFT_PINSOURCE;
	MOTOR_LEFT_HANDLE.PWM_Motor_AF=PWM_MOTORLEFT_AF;
	MOTOR_LEFT_HANDLE.PWM_Motor_Port= PWM_MOTORLEFT_PORT;
	MOTOR_LEFT_HANDLE.PWM_Motor_Channel=PWM_MOTORLEFT_CHANNEL;
	MOTOR_LEFT_HANDLE.DIR_Motor_Pinsource= DIR_MOTORLEFT_PINSOURCE;
	MOTOR_LEFT_HANDLE.DIR_Motor_Port=DIR_MOTORLEFT_PORT;
	MOTOR_LEFT_HANDLE.Position_Motor_Handle= &TIM_MOTORLEFT_POSITION_Handle;
	MOTOR_LEFT_HANDLE.Position_Motor_Tim_Base= ENC_MOTORLEFT_TIMERBASE;
	MOTOR_LEFT_HANDLE.Motor_Pulse_Count = &MOTORLEFT_Pulse_Count;
	MOTOR_LEFT_HANDLE.Veclocity_Motor_Handle=&TIM_MOTORLEFT_VELOCITY_Handle;
	MOTOR_LEFT_HANDLE.Veclocity_Motor_Tim_Base= IC_MOTORLEFT_TIMERBASE;
	MOTOR_LEFT_HANDLE.IC_Motor_State= &IC_Motor_Left_State;
	MOTOR_LEFT_HANDLE.PID_Motor_Angle_Handle = &MOTOR_LEFT_Angle_PID_HANDLE;
	MOTOR_LEFT_HANDLE.PID_Motor_VELOCITY_Handle = &MOTOR_LEFT_Speed_PID_HANDLE;
	
	
	MOTOR_RIGHT_HANDLE.Motor_Name = RIGHT_MOTOR;
	MOTOR_RIGHT_HANDLE.ENC_PulseperRound_AB_BothEdge = ENC_MOTORRIGHT_PulseperRound_AB_BothEdge;
	MOTOR_RIGHT_HANDLE.PWM_Motor_Handle = &TIM_MOTORRIGHT_PWM_Handle;
	MOTOR_RIGHT_HANDLE.PWM_Motor_Tim_Base= PWM_MOTORRIGHT_TIMERBASE;
	MOTOR_RIGHT_HANDLE.PWM_Motor_Pinsource=PWM_MOTORRIGHT_PINSOURCE;
	MOTOR_RIGHT_HANDLE.PWM_Motor_AF=PWM_MOTORRIGHT_AF;
	MOTOR_RIGHT_HANDLE.PWM_Motor_Port= PWM_MOTORRIGHT_PORT;
	MOTOR_RIGHT_HANDLE.PWM_Motor_Channel=PWM_MOTORRIGHT_CHANNEL;
	MOTOR_RIGHT_HANDLE.DIR_Motor_Pinsource= DIR_MOTORRIGHT_PINSOURCE;
	MOTOR_RIGHT_HANDLE.DIR_Motor_Port=DIR_MOTORRIGHT_PORT;
	MOTOR_RIGHT_HANDLE.Position_Motor_Handle= &TIM_MOTORRIGHT_POSITION_Handle;
	MOTOR_RIGHT_HANDLE.Position_Motor_Tim_Base= ENC_MOTORRIGHT_TIMERBASE;
	MOTOR_RIGHT_HANDLE.Motor_Pulse_Count = &MOTORRIGHT_Pulse_Count;
	MOTOR_RIGHT_HANDLE.Veclocity_Motor_Handle=&TIM_MOTORRIGHT_VELOCITY_Handle;
	MOTOR_RIGHT_HANDLE.Veclocity_Motor_Tim_Base= IC_MOTORRIGHT_TIMERBASE;
	MOTOR_RIGHT_HANDLE.IC_Motor_State= &IC_Motor_Right_State;
	MOTOR_RIGHT_HANDLE.PID_Motor_Angle_Handle = &MOTOR_RIGHT_Angle_PID_HANDLE;
	MOTOR_RIGHT_HANDLE.PID_Motor_VELOCITY_Handle = &MOTOR_RIGHT_Speed_PID_HANDLE;
}

void PID_Angle_Handle_Init(void)
{
	MOTOR_LEFT_Angle_PID_HANDLE.Name = LEFT_MOTOR;
	MOTOR_LEFT_Angle_PID_HANDLE.Kp = PID_MOTORLEFT_Angle_KP;
	MOTOR_LEFT_Angle_PID_HANDLE.Ki = PID_MOTORLEFT_Angle_KI;
	MOTOR_LEFT_Angle_PID_HANDLE.Kd = PID_MOTORLEFT_Angle_KD;
	MOTOR_LEFT_Angle_PID_HANDLE.Ts = PID_MOTORLEFT_Angle_Ts;
	
	MOTOR_RIGHT_Angle_PID_HANDLE.Name = RIGHT_MOTOR;
	MOTOR_RIGHT_Angle_PID_HANDLE.Kp = PID_MOTORRIGHT_Angle_KP;
	MOTOR_RIGHT_Angle_PID_HANDLE.Ki = PID_MOTORRIGHT_Angle_KI;
	MOTOR_RIGHT_Angle_PID_HANDLE.Kd = PID_MOTORRIGHT_Angle_KD;
	MOTOR_RIGHT_Angle_PID_HANDLE.Ts = PID_MOTORRIGHT_Angle_Ts;
}
void PID_SPEED_Handle_Init(void)
{
	MOTOR_LEFT_Speed_PID_HANDLE.Name = LEFT_MOTOR;
	MOTOR_LEFT_Speed_PID_HANDLE.Kp = PID_MOTORLEFT_Speed_KP;
	MOTOR_LEFT_Speed_PID_HANDLE.Ki = PID_MOTORLEFT_Speed_KI;
	MOTOR_LEFT_Speed_PID_HANDLE.Kd = PID_MOTORLEFT_Speed_KD;
	MOTOR_LEFT_Speed_PID_HANDLE.Ts = PID_MOTORLEFT_Speed_Ts;
	
	MOTOR_RIGHT_Speed_PID_HANDLE.Name = RIGHT_MOTOR;
	MOTOR_RIGHT_Speed_PID_HANDLE.Kp = PID_MOTORRIGHT_Speed_KP;
	MOTOR_RIGHT_Speed_PID_HANDLE.Ki = PID_MOTORRIGHT_Speed_KI;
	MOTOR_RIGHT_Speed_PID_HANDLE.Kd = PID_MOTORRIGHT_Speed_KD;
	MOTOR_RIGHT_Speed_PID_HANDLE.Ts = PID_MOTORRIGHT_Speed_Ts;
}
//PID_Control medium level
double PID_Angle_Control(PID_TypeDef* PID_Angle, double Angle_SetPoint,double Angle_IMU)
{
	PID_Angle->Setpoint = Angle_SetPoint;	//Load SetPoint Value to PID Position Object	
	return PID_Calculate(PID_Angle,Angle_IMU,0);	
}

double PID_Speed_Control(PID_TypeDef* PID_Speed, double Speed_SetPoint,double Speed)
{
	PID_Speed->Setpoint = Speed_SetPoint;
	return PID_Calculate(PID_Speed,Speed,0);
}




//void 
//{
//	Get_AngleSetPoint(); --> Path_Planning to the next-point(for example using a index for waypointarray,increse index if it reach the target point),then calculate Angle setpoint
//	Get_AngleIMU();
//}

void SowingMachine_Control_State_Machine(void)
{
	switch (STATE)
	{
		case DECISION_STATE:
			if ( PREVIOUS_STATE == UNKNOWN_STATE)
			{
				if(FLAG_START==true)
					COMMAND_STATE = START;
				else if(FLAG_TESTING==true)
				{
					COMMAND_STATE = TESTING;
				}
				else
					COMMAND_STATE = UNKNOWN_STATE;
			}

			if (PREVIOUS_STATE == TESTING)
			{
				COMMAND_STATE = TESTING;
			}
			if ( PREVIOUS_STATE == RUN_AND_TRACK )
			{
				COMMAND_STATE = PATH_PLANNING;
			}
			if ( PREVIOUS_STATE == PATH_PLANNING )
			{
				COMMAND_STATE = RUN_AND_TRACK;
			}
			if ( PREVIOUS_STATE == START )
			{
				COMMAND_STATE = PATH_PLANNING;
				//COMMAND_STATE =FLOATING;
			}
			UPDATE_STATE_FUNCTION(COMMAND_STATE);
			break;
		case START:
			{
				/*PREPARE UTM-WAYPOINT*/
				/*Stanley_Init*/
				UTM_Deg2utm(dTrajectory,dTrajectory_UTM,ui8_Utmzone,ui32_Trajectory_Size);
				Vehicle_Coordinate[0][0] = Get_LatGPS();
				Vehicle_Coordinate[0][1] = Get_LongGPS();
				UTM_Deg2utm(Vehicle_Coordinate,Vehicle_Coordinate_UTM,Vehicle_Utmzone,1);
				Stanley_Init(dTrajectory_UTM,ui32_Trajectory_Size,Constant_CrossTrack_Error,Vehicle_Coordinate_UTM);
				PREVIOUS_STATE = START;
				UPDATE_STATE_FUNCTION(DECISION_STATE);
			}
			break;
		case STOP :
			break;
		case RUN_AND_TRACK:
			if (Systick_IsTimeOut(tick_start_Angle, (double)PID_MOTOR_ANGLE_Ts*1000.0))
			{
				
				SetPoint_Angle = Stanley_Get_Steer_Ang();
				control_angle(SetPoint_Angle);
				//PID_Speed_Control_LEFT_and_RIGHT(10,0);			
				tick_start_Angle = SysTick_GetTick();
				
			}
			if (Systick_IsTimeOut(tick_start_Speed, (double)PID_MOTORLEFT_Speed_Ts*1000.0))
			{
				tick_measure_1 = SysTick_GetTick();
				//PID_Speed_Control_LEFT_and_RIGHT(20,0);
				PID_Speed_Control_LEFT_and_RIGHT(vleft,vright);
				Test_1++;	
				tick_start_Speed = SysTick_GetTick();
				tick_measure_2 = SysTick_GetTick();
			}
			PREVIOUS_STATE = RUN_AND_TRACK;
			UPDATE_STATE_FUNCTION(DECISION_STATE);
			break;
		case PATH_PLANNING:
			//if ( khoang cach tu xe den target < threshhold )
					//index++
			
			Vehicle_Coordinate[0][0] = Get_LatGPS();
			Vehicle_Coordinate[0][1] = Get_LongGPS();
			UTM_Deg2utm(Vehicle_Coordinate,Vehicle_Coordinate_UTM,Vehicle_Utmzone,1);
			Stanley_Algorithm(Vehicle_Coordinate_UTM,Get_IMUHeading(),Get_Velocity_Average());
			PREVIOUS_STATE = PATH_PLANNING;
			UPDATE_STATE_FUNCTION(DECISION_STATE);
			
			break;
		case SOWING:
			break;
		case UNKNOWN_STATE:
			UPDATE_STATE_FUNCTION(DECISION_STATE);
			break;
		case TESTING:
//			if (Systick_IsTimeOut(tick_start_Angle, (double)PID_MOTOR_ANGLE_Ts*1000.0))
//			{
//				control_angle(Test_1);
//				//PID_Speed_Control_LEFT_and_RIGHT(10,0);			
//				tick_start_Angle = SysTick_GetTick();
//			}
//			if (Systick_IsTimeOut(tick_start_Speed, (double)PID_MOTORLEFT_Speed_Ts*1000.0))
//			{
//				//PID_Speed_Control_LEFT_and_RIGHT(20,0);
//				PID_Speed_Control_LEFT_and_RIGHT(vleft,vright);
//				//Test_1++;	
//				tick_start_Speed = SysTick_GetTick();
//			}
			PREVIOUS_STATE = TESTING;
			break;
		case FLOATING :
			break;
	}
}
void SowingMachine_Control (void)
{
	switch (MODE)
	{
		case AUTO_MODE:
			SowingMachine_Control_State_Machine();
			break;
		case MANUAL_MODE:
			break;
	}
}

void UPDATE_STATE_FUNCTION (SowingMachine_Control_State STATE_Temp)
{
	STATE = STATE_Temp;
	//return UPDATESTATE;
}


void control_angle(double angle)
{
//	double vleft =0,vright=0 , v_offset = 0 , v_init =100;
//	double Cur_Angle = 0;
//	bool Turn_Right = false;
	//	double dx = 0 ; 
//	double dy = 0 ;
//	double gps_x = 0;
//	double gps_y = 0;
//	double goal_x=0;
//	double goal_y=0;
	
//	
//	GPS_Reading_Data();// toa do qua UTM
//	dx = goal_x - gps_x ;
//	dy = goal_y - gps_y ;
//	Cur_Angle = atan2(dx,dy)*180/PI ;	
//	if()
//	{
//	}
	Cur_Angle = Get_IMUHeading();
	v_offset = PID_Angle_Control(&PID_Angle,angle,Cur_Angle);
	if (v_offset < 0)
		v_offset = v_offset*-1.0;
//	if (Cur_Angle >=0 && Cur_Angle <= 180)
//	{
//		if ((angle - Cur_Angle) >=0 && (angle - Cur_Angle) <=180)
//			Turn_Right = 1;
//		else if ((angle - Cur_Angle) > 180 || (angle - Cur_Angle) <0)
//			Turn_Right = 0;
//	}
//	else
//	{
//		if ( (angle - Cur_Angle) >= -180 && (angle - Cur_Angle) <= 0)
//			Turn_Right = 0;
//		else if ((angle - Cur_Angle) >0 || (angle - Cur_Angle) < -180)
//			Turn_Right = 1;
//			
//	}
	if ( Turn_Right == 1)
	{
		vleft = v_init + v_offset*v_init ;
		vright = v_init - v_offset*v_init;
	}
	else
	{
		vleft = v_init - v_offset*v_init ;
		vright = v_init + v_offset*v_init;
	}
}
void PID_Speed_Control_LEFT_and_RIGHT(double vleft_Setpoint, double vright_Setpoint)
{

	
	 //speed control
/**PWM0**/
		Get_Velocity_Basedon_PulseCount(&MOTOR_LEFT_HANDLE);
		pwm0 = PID_Speed_Control(&PID_PWM_L,vleft_Setpoint,MOTOR_LEFT_HANDLE.Motor_Veclocity);
		if ( pwm0 < 0)
			pwm0=0;
		if (pwm0 >0.95)
			pwm0 = 0.95;
		TIM_PWM_SetPulseWidth_SetDir(pwm0,GO_FORWARD,&MOTOR_LEFT_HANDLE);
		//TIM_PWM_SetPulseWidth_SetDir(0.2,GO_FORWARD,&MOTOR_LEFT_HANDLE);


	 
/**PWM1**/		 
		 
			Get_Velocity_Basedon_PulseCount(&MOTOR_RIGHT_HANDLE);
		 pwm1 = PID_Speed_Control(&PID_PWM_R,vright_Setpoint,MOTOR_RIGHT_HANDLE.Motor_Veclocity);
		 if ( pwm1 < 0)
			 pwm1 = 0;
		 if ( pwm1 > 0.95 )
			 pwm1 = 0.95;
		 TIM_PWM_SetPulseWidth_SetDir(pwm1,GO_FORWARD,&MOTOR_RIGHT_HANDLE);
		 //TIM_PWM_SetPulseWidth_SetDir(0.2,GO_FORWARD,&MOTOR_RIGHT_HANDLE);
}
/*Button Interrupt Routine Service*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{			
	if(GPIO_Pin == Button_Start)
	{
//		FLAG_TESTING =true;
//		//if( Temp3_Test == false)
//		//{
//		//	Temp3_Test = true;
//			
//			Temp1 = PID_PWM_L.Ki + 0.0001;
//			if (Temp1 > 1)
//				Temp1 = 1;
//			//vleft = Temp1;
//			//vright = Temp1;
			//PID_PWM_L.Ki=Temp1;
			//PID_PWM_R.Kp=Temp1;
		//}
		//TIM_PWM_SetPulseWidth_SetDir(0.99,GO_FORWARD,&MOTOR_RIGHT_HANDLE);
		//TIM_PWM_SetPulseWidth_SetDir(0.99,GO_FORWARD,&MOTOR_LEFT_HANDLE);
		//HAL_Delay(10);
	}
	else if(GPIO_Pin == Button_Stop)
	{
		//if (Temp3_Test == true )
		//{
//			//Temp3_Test = false;
//			Temp2 = PID_PWM_L.Kd + 0.00001;
//			if ( Temp2 > 1 )
//				Temp2 = 1;
//			PID_PWM_L.Kd=Temp2;
//			PID_PWM_R.Kp=Temp2;
//			//vleft = Temp2;
//			//vright = Temp2;
		//}
		//TIM_PWM_SetPulseWidth_SetDir(0.6,GO_FORWARD,&MOTOR_RIGHT_HANDLE);
		//TIM_PWM_SetPulseWidth_SetDir(0.6,GO_FORWARD,&MOTOR_LEFT_HANDLE);
			//HAL_Delay(10);
	}
	else if ( GPIO_Pin == Button_Reset)
	{
//			Temp3 = PID_PWM_L.Kp + 0.00001;
//			if ( Temp3 > 1)
//				Temp3 =1;
//			//PID_PWM_L.Kp = Temp3;
//			//vleft = Temp3;
//			//vright = Temp3;
//		//TIM_PWM_SetPulseWidth_SetDir(0,GO_FORWARD,&MOTOR_RIGHT_HANDLE);
//		//TIM_PWM_SetPulseWidth_SetDir(Temp3,GO_FORWARD,&MOTOR_LEFT_HANDLE);
//			//HAL_Delay(10);
		FLAG_START =true;
		FLAG_TESTING = false;
	}
}
