#ifndef STANLEY_H
#define STANLEY_H

#include "define.h"
#include "math.h"
#include "stdint.h"
#include "stdlib.h"
#include "stdbool.h"

typedef struct{
	double Trajectory[MAX_TRA_POINT][2];
	uint16_t Size;
	double Slope[MAX_TRA_POINT];
	double Velocity;
	double K;
	double Coordinate[1][2];
	uint8_t Index;
	double Steering_Angle;
	double Vehicle_Heading;
} Stanley_Param;
extern int8_t i8_TURN_RIGHT;
extern Stanley_Param Parameter ;
void Stanley_Init(double Trajectory[][2], uint16_t Size, double K, double Start_Coordinate[][2]);
void Stanley_Algorithm(double fCoordinate[][2], double fSteer_Ang, double fVelocity);
void Stanley_Test_Sample_Trajectory(float CoordinateX, float CoordinateY);
double Stanley_Get_Steer_Ang(void);
void Stanley_Set_Size(uint16_t Size);

#endif
