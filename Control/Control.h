#ifndef CONTROL_H
#define CONTROL_H

/* Includes ------------------------------------------------------------------*/
#include "define.h"
#include <stdint.h>
#include <stdbool.h>
#include "Motor.h"
#include "PID.h"



typedef enum{
	MANUAL_MODE = 0,
	AUTO_MODE
} SowingMachine_Control_Mode;

typedef enum{
	DECISION_STATE=0,
	START,
	STOP,
	RUN_AND_TRACK,
	PATH_PLANNING,
	SOWING,
	UNKNOWN_STATE,
	TESTING,
	FLOATING
} SowingMachine_Control_State;


extern Motor_HandleTypedef MOTOR_LEFT_HANDLE;
extern Motor_HandleTypedef MOTOR_RIGHT_HANDLE;
extern PID_TypeDef MOTOR_LEFT_Angle_PID_HANDLE;
extern PID_TypeDef MOTOR_RIGHT_Angle_PID_HANDLE;
extern uint32_t uwTick;
extern SowingMachine_Control_State PREVIOUS_STATE;
//extern SowingMachine_Control_State STATE;
extern double vleft,vright,v_offset,v_init;
extern PID_TypeDef PID_PWM_L;
extern PID_TypeDef PID_PWM_R;
extern double Cur_Angle ;
extern	uint8_t Turn_Right ;


void Motor_Handle_Init(void);
void PID_Angle_Handle_Init(void);
void PID_SPEED_Handle_Init(void);
uint32_t SysTick_GetTick(void);
bool Systick_IsTimeOut(uint32_t start_time, uint32_t timeout_ms);
void SowingMachine_Control(void);
void UPDATE_STATE_FUNCTION (SowingMachine_Control_State UPDATESTATE);
void PID_Speed_Control_LEFT_and_RIGHT(double vleft_Setpoint, double vright_Setpoint);
void control_angle(double angle);

extern double dTrajectory [Trajectory_Size][2];
extern double dTrajectory_UTM [Trajectory_Size][2];
extern uint8_t ui8_Utmzone[Trajectory_Size][2];
extern uint32_t ui32_Trajectory_Size;
extern uint32_t ui32_Trajectory_Index ;
extern bool FLAG_START;


#define T          1000


#endif /* CONTROL_H_ */

