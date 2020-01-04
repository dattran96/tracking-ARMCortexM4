#ifndef DEFINE_H
#define DEFINE_H

/* PID files */
#define PID_METHOD1
#define NO_CHANGE 			((float) 0xFFFFFFFF)
#define ERROR_THRES			((float) 0.1)			// Variation within ERROR_THRES will not effect.
#define RESULT_THRES 		((float)0.6)

/* Stanley files */
#define PI 					((float) 180.0)
#define MAX_TRA_POINT 		100
#define RAD2DEG				((double) 180.0/3.1415926535)
#define UPDATE_DISTANCE		((float) 0.1)
#define Constant_CrossTrack_Error 0.2
#define Constant_Speed	1

/*IMU calib*/
//0 	45	90	135	180	225	270	315	360
//a1	a2	a3	a4	a5	a6	a7	a8	a9
#define IMU_calib_a1 	31.1
#define IMU_calib_a2	83.3
#define IMU_calib_a3	109.5
#define IMU_calib_a4	154.5
#define IMU_calib_a5	211.7
#define IMU_calib_a6	241.05
#define IMU_calib_a7	298.5
#define IMU_calib_a8	343.3



/* PWM and DIR of LEFT motor files */
#define TICK_PERIOD_MotorLeft 		20000000					// unit: Hz, period for single count.
#define RELOAD_PERIOD_MotorLeft 		1000					// period for pulse, Reload Cycle = RELOAD_PERIOD / TICK_PERIOD.
#define PWM_MOTORLEFT_PINSOURCE 				GPIO_PIN_12
#define PWM_MOTORLEFT_AF								GPIO_AF2_TIM4
#define PWM_MOTORLEFT_PORT							GPIOD
#define PWM_MOTORLEFT_TIMERBASE 		TIM4
#define PWM_MOTORLEFT_CHANNEL 			TIM_CHANNEL_1
#define DIR_MOTORLEFT_PINSOURCE			GPIO_PIN_8
#define DIR_MOTORLEFT_PORT					GPIOA
/* PWM and DIR of RIGHT motor files */
#define TICK_PERIOD_MotorRight 		20000000					// unit: Hz, period for single count.
#define RELOAD_PERIOD_MotorRight 		1000					// period for pulse, Reload Cycle = RELOAD_PERIOD / TICK_PERIOD.
#define PWM_MOTORRIGHT_PINSOURCE 				GPIO_PIN_6
#define PWM_MOTORRIGHT_AF								GPIO_AF3_TIM9
#define PWM_MOTORRIGHT_PORT							GPIOE
#define PWM_MOTORRIGHT_TIMERBASE		TIM9
#define PWM_MOTORRIGHT_CHANNEL 			TIM_CHANNEL_2
#define DIR_MOTORRIGHT_PINSOURCE		GPIO_PIN_15
#define DIR_MOTORRIGHT_PORT					GPIOA
/*ENCODER FOR POSITON OF LEFT MOTOR*/
#define ENC_MOTORLEFT_TIMERBASE		TIM3
#define ENC_MOTORLEFT_PulseperRound_AB_BothEdge 39400
#define ENC_MOTORLEFT_PulseCount_Sampling_Time	1				//1 ms
/*ENCODER FOR POSITON OF RIGHT MOTOR*/
#define ENC_MOTORRIGHT_TIMERBASE		TIM8
#define ENC_MOTORRIGHT_PulseperRound_AB_BothEdge 39400
#define ENC_MOTORRIGHT_PulseCount_Sampling_Time	1	
/*ENCODER FOR INPUT CAPTURE RIGHT MOTOR*/
#define IC_TICK_PERIOD_MOTOR_RIGHT 10000000
#define IC_RELOAD_PERIOD_MOTOR_RIGHT 50000
#define IC_MOTORRIGHT_TIMERBASE 		TIM1
#define IC_MOTORRIGHT_CHANNEL1 		TIM_CHANNEL_1
#define IC_MOTORRIGHT_CHANNEL2			TIM_CHANNEL_2
#define IC_MOTORRIGHT_PINSOURCE		GPIO_PIN_9|GPIO_PIN_11
#define IC_MOTORRIGHT_AF						GPIO_AF1_TIM1
#define IC_MOTORRIGHT_PORT					GPIOE
/*ENCODER FOR INPUT CAPTURE LEFT MOTOR*/
#define IC_TICK_PERIOD_MOTOR_LEFT 10000000
#define IC_RELOAD_PERIOD_MOTOR_LEFT 50000
#define IC_MOTORLEFT_TIMERBASE 		TIM2
#define IC_MOTORLEFT_CHANNEL1 			TIM_CHANNEL_1
#define IC_MOTORLEFT_CHANNEL2 			TIM_CHANNEL_2
#define IC_MOTORLEFT_PINSOURCE		 	GPIO_PIN_0|GPIO_PIN_1
#define IC_MOTORLEFT_AF						GPIO_AF1_TIM2
#define IC_MOTORLEFT_PORT					GPIOA
/*PID LEFT MOTOR POSITION*/
#define PID_MOTORLEFT_Angle_KP	0
#define PID_MOTORLEFT_Angle_KI	0
#define PID_MOTORLEFT_Angle_KD	0
#define PID_MOTORLEFT_Angle_Ts	0
#define PID_MOTORRIGHT_Angle_KP	0
#define PID_MOTORRIGHT_Angle_KI	0
#define PID_MOTORRIGHT_Angle_KD	0
#define PID_MOTORRIGHT_Angle_Ts	0
/*PID LEFT MOTOR SPEED*/
#define PID_MOTORLEFT_Speed_KP	0.0000001
#define PID_MOTORLEFT_Speed_KI	0.02
#define PID_MOTORLEFT_Speed_KD	0.0000001
#define PID_MOTORLEFT_Speed_Ts	0.003
#define PID_MOTORLEFT_Speed_Error_Tolerance 0.1
#define PID_MOTORLEFT_Speed_Limit 0.6
/*PID RIGHT MOTOR SPEED*/
#define PID_MOTORRIGHT_Speed_KP	 0.0000001
#define PID_MOTORRIGHT_Speed_KI	0.02
#define PID_MOTORRIGHT_Speed_KD	0.0000001
#define PID_MOTORRIGHT_Speed_Ts	0.003
#define PID_MOTORRIGHT_Speed_Error_Tolerance 0.1
#define PID_MOTORRIGHT_Speed_Limit 0.6
/*PID ANGLE*/
#define PID_MOTOR_ANGLE_KP	0.0000001
#define PID_MOTOR_ANGLE_KI	0.02
#define PID_MOTOR_ANGLE_KD	0.0000001
#define PID_MOTOR_ANGLE_Ts	0.005
#define PID_MOTOR_ANGLE_Error_Tolerance 0.1
#define PID_MOTOR_ANGLE_Limit 0.6
/*BUTTON DEFINE*/
#define Button_Start GPIO_PIN_13
#define Button_Stop	 GPIO_PIN_14
#define Button_Reset GPIO_PIN_15
#define Button_Port	 GPIOD
/*TRAJECTORY*/
#define Trajectory_Size 4



/*PID RIGHT MOTOR*/


/*ENCODER FOR INPUT CAPTURE RIGHT MOTOR*/

/* IMU_GPS files */
#define KNOT2M				((float) 1852.0)		// 1 knot = 1852 m
#define KPH2MPS				((float) 5.0/18.0)		// 1 k/m = 5/18 m/s
#define LATDEG2M			((float) 110567)		// 1 degree Latitude = 110.567 km at the Equator
#define LONDEG2M			((float) 111321)		// 1 degree Longtitude = 111.321 km at the Equator
/* RF files*/
#define BUFFTX_SIZE 		220
#define TRANS_PEDRIOD		100						// milisecond
#define H_RIGHT_SETPOINT	10.0 // Official setpoint of turning right
#define H_LEFT_SETPOINT		-10.0
#define L_RIGHT_SETPOINT	5.0
#define L_LEFT_SETPOINT		-5.0
#define STRAIGHT_SETPOINT	-4.0
/* UTM files*/
#define PI_RAD				((double) 3.1415926535898)
/* General definition*/
#define TRAJECT_SIZE 		6
#define MIN_SPEED			7.5
#define PITCH_SETPOINT		((float) 4.0)
#define LOWER_PW			((float) 50.0)
#define UPPER_PW			((float) 250.0)
#define KP_ROLL				((double) 0.005)									//((double) 0.01)				//((double) 0.035)					//((double) 0.035)
#define KI_ROLL				((double) 0.0025)									//((double) 0.0005)					//((double) 0.01)					//((double) 0.018)
#define KD_ROLL				((double) 0.0005)									//((double) 0.001)
#define KP_PITCH			((double) 0.02)					//((double) 0.01)					//((double) 0.03)
#define KI_PITCH			((double) 0.001)				//((double) 0.008)					//((double) 0.01)
#define KD_PITCH			((double) 0.005)
#define OMEGA_PITCH		((double) 0.0)					//((double) 0.1)
#define OMEGA_ROLL		((double) 0.0)					//((double) 0.1)
#define YAW_SETPOINT	((double) 0.0)
#define RIGHT_YAW_SETPOINT ((double) 5.0)
#define TURNING_SETPOINT	((double) 20.0) // Test goc queo
#define ROLL_ERROR_THRESHOLD ((double) 10.0) // need to consider again, currently this para ist dismissed
#define STRAIGHT_TURNING_THRES ((double) 90.0)
#define ACCEPTABLE_THRESHOLD	0.5
#define STABLE_DEADBAND		5.0 // have to modify during experience
#define TRACK_DEADBAND		5.0 // have to modify during experience
#define TURN_DEADBAND		5.0 // have to modify during experience
#define SEGMENT_SETPOINT_DEABAND 5.0 
#define MAX_NUMBER_SEGMENT_SETPOINT 5 
#define YAW_ERROR_STABLE_RANGE 5
#define YAW_ERROR_TRACK_RANGE 5
#define TURN_ANGLE 90
#endif
