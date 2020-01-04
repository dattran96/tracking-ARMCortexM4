/**
  ******************************************************************************
  * File Name          : Stanley.c
  * Description        : Stanley path planning

			 
  ******************************************************************************
  */

/**
 * author Duy Tan Nguyen, Ho Chi Minh University of Technology, Vietnam
	  Minh Quang Tran, Ho Chi Minh University of Technology, Vietnam
 */
#include "Stanley.h"
#include "gps.h"

double dDis_test;
Stanley_Param Parameter;
int8_t i8_TURN_RIGHT;
double paral_vct[2], perpen_vct[2], f_sign_val, point2line_vct[2], f_x, f_phi ;

void Stanley_Set_Trajectory_Size(double fTrajectory[][2], uint8_t ui8_size)
{
	uint8_t i, j;
	for (i = 0; i < ui8_size; i++)
		for (j = 0; j <= 1; j++)
			Parameter.Trajectory[i][j] = fTrajectory[i][j];
	Parameter.Size = ui8_size;
}
void Stanley_Add_Coordinate(double fCoordinate[][2])
{
	Parameter.Coordinate[0][0] = fCoordinate[0][0];
	Parameter.Coordinate[0][1] = fCoordinate[0][1];
}
void Stanley_Add_Steer_Ang(double fSteer_Ang)
{
	Parameter.Vehicle_Heading= fSteer_Ang;
}
void Stanley_Add_K(double fK)
{
	Parameter.K = fK;
}
void Stanley_Add_Velocity(double fVelocity)
{
	Parameter.Velocity = fVelocity;
}
uint8_t Min_Index(double *array, uint8_t size)
{
	int i;
	double min = array[0];
	uint8_t index = 0;
	for (i = 0; i < size; i++)
		if (array[i] < min) 
		{
			min = array[i];
			index = i;
		}
	return index;
}


/* This function returns a array containing slopes of all segments in desired trajectory
 The range of slopes is from 0 to 2*pi, actually slopes are angle between vector of directory and vector i*/
void Stanley_Slope(void)
{
	uint8_t i;
	/* Calculate the slopes of segments of desired trajectory */
	/*0 < = Slope < 360*/
	for (i = 0; i < (Parameter.Size - 1); i++)
	{
		//Parameter.Slope[i] = atan((Parameter.Trajectory[i+1][1] - Parameter.Trajectory[i][1]) / (Parameter.Trajectory[i+1][0] - Parameter.Trajectory[i][0]))*RAD2DEG;
		Parameter.Slope[i] = atan2(Parameter.Trajectory[i+1][1] - Parameter.Trajectory[i][1],Parameter.Trajectory[i+1][0] - Parameter.Trajectory[i][0])*RAD2DEG;
		//if ((Parameter.Trajectory[i+1][0] - Parameter.Trajectory[i][0]) < 0)
			//Parameter.Slope[i] = PI + Parameter.Slope[i];
		//Parameter.Slope[i] = - (Parameter.Slope[i] - PI/2.0); /* Convert angles relative to East into angles relative to True North */
//		if (Parameter.Slope[i] > PI)
//			Parameter.Slope[i] = Parameter.Slope[i] - 2*PI;
		if ((Parameter.Slope[i] < 0))
			Parameter.Slope[i] = Parameter.Slope[i] + 2*180;
	}
	//Parameter.Slope[Parameter.Size - 1] = atan((Parameter.Trajectory[0][1] - Parameter.Trajectory[Parameter.Size - 1][1]) / (Parameter.Trajectory[0][0] - Parameter.Trajectory[Parameter.Size - 1][0]))*RAD2DEG;
	Parameter.Slope[Parameter.Size - 1] = atan2((Parameter.Trajectory[0][1] - Parameter.Trajectory[Parameter.Size - 1][1]),(Parameter.Trajectory[0][0] - Parameter.Trajectory[Parameter.Size - 1][0]))*RAD2DEG;
//	if ((Parameter.Trajectory[0][0] - Parameter.Trajectory[Parameter.Size - 1][0]) < 0)
//		Parameter.Slope[Parameter.Size - 1] = PI + Parameter.Slope[Parameter.Size - 1];
//	Parameter.Slope[Parameter.Size - 1] = - (Parameter.Slope[Parameter.Size - 1] - PI/2.0); /* Convert angles relative to East into angles relative to True North */
//	if ((Parameter.Slope[Parameter.Size - 1] > 2.0*PI))
//		Parameter.Slope[Parameter.Size - 1] = Parameter.Slope[Parameter.Size - 1] - 2*PI;
	if ((Parameter.Slope[Parameter.Size - 1] < 0))
		Parameter.Slope[Parameter.Size - 1] = Parameter.Slope[Parameter.Size - 1] + 2*180;
	for (i =0; i < Parameter.Size; i++)
	{
		if ( Parameter.Slope[i] >=0 && Parameter.Slope[i] < 90)
			Parameter.Slope[i] =  90 -Parameter.Slope[i];
		else if (  Parameter.Slope[i] >= 90 && Parameter.Slope[i] < 180 )
			Parameter.Slope[i] = 270+ (180-Parameter.Slope[i] );
		else if (   Parameter.Slope[i] >= 180 &&  Parameter.Slope[i] < 270 )
			 Parameter.Slope[i] = 180 + (  270 - Parameter.Slope[i] );
		else if (  Parameter.Slope[i] >= 270 &&  Parameter.Slope[i] < 360 )
			 Parameter.Slope[i] = 90 + ( 360 -  Parameter.Slope[i] );
	}
}

/* Choose the nearest point of desired trajectory to follow
	This function returns the index of the nearest point in array */
void Stanley_Scan_Distance(void)
{
	double f_distance[MAX_TRA_POINT - 1];
	uint8_t i;
	for (i = 0; i < Parameter.Size; i++)
		f_distance[i] = sqrt((Parameter.Coordinate[0][0] - Parameter.Trajectory[i][0])*(Parameter.Coordinate[0][0] - Parameter.Trajectory[i][0]) + (Parameter.Coordinate[0][1] - Parameter.Trajectory[i][1])*(Parameter.Coordinate[0][1] - Parameter.Trajectory[i][1]));
	Parameter.Index = Min_Index(f_distance, Parameter.Size);
}

void Stanley_Algorithm(double fCoordinate[][2], double fSteer_Ang, double fVelocity)
{
	
	/* Get new values: coordinate, steering angle and velocity*/
	Stanley_Add_Coordinate(fCoordinate);
	Stanley_Add_Steer_Ang(fSteer_Ang);
	Stanley_Add_Velocity(fVelocity);
	/* Calculate parallel vector (then perpendicular vector) to determine the line equation of segments in desired trajectory
	 if parallel vector is (a, b), then perpendicular vector is (b, -a) */
	if (Parameter.Index < (Parameter.Size - 1))
	{
		paral_vct[0] = Parameter.Trajectory[Parameter.Index + 1][0] - Parameter.Trajectory[Parameter.Index][0];
		paral_vct[1] = Parameter.Trajectory[Parameter.Index + 1][1] - Parameter.Trajectory[Parameter.Index][1];
	}
	else
	{
		paral_vct[0] = Parameter.Trajectory[0][0] - Parameter.Trajectory[Parameter.Size - 1][0];
		paral_vct[1] = Parameter.Trajectory[0][1] - Parameter.Trajectory[Parameter.Size - 1][1];
	}
	perpen_vct[0] = -paral_vct[1];
	perpen_vct[1] = paral_vct[0];
	/* The line equation is Ax + By - (Ax0 + By0) = 0, with (A, B) is perpendicular vector, (x0, y0) is a particular point belonging to line
	If B != 0, B*(Ax0 + By0 + C) >= 0 claims that the point (x0, y0) is above the line Ax + By + C =0 and vice versa. 
	If B = 0, the line is parallel to vector j, this function determines if the point (x0, y0) is on right or left side of the line*/
	/*f_sign_val > 0 : TURN RIGHT*/
	/*f_sign_val < 0 : TURN LEFT*/
	if (Parameter.Index < (Parameter.Size - 1))
	{
		if (perpen_vct[1] != 0)
			f_sign_val = (paral_vct[0])*(perpen_vct[0]*Parameter.Coordinate[0][0] + perpen_vct[1]*Parameter.Coordinate[0][1] 
					- (perpen_vct[0]*Parameter.Trajectory[Parameter.Index + 1][0] + perpen_vct[1]*Parameter.Trajectory[Parameter.Index + 1][1]));
		else
			f_sign_val = -paral_vct[1]*(perpen_vct[0]*Parameter.Coordinate[0][0] 
					- (perpen_vct[0]*Parameter.Trajectory[Parameter.Index+1][0] + perpen_vct[1]*Parameter.Trajectory[Parameter.Index + 1][1]));
	}
	else
	{
		if (perpen_vct[1] != 0)
			f_sign_val = (paral_vct[0])*(perpen_vct[0]*Parameter.Coordinate[0][0] + perpen_vct[1]*Parameter.Coordinate[0][1] 
					- (perpen_vct[0]*Parameter.Trajectory[0][0] + perpen_vct[1]*Parameter.Trajectory[0][1]));
		else
			f_sign_val = -paral_vct[1]*(perpen_vct[0]*Parameter.Coordinate[0][0] 
					- (perpen_vct[0]*Parameter.Trajectory[0][0] + perpen_vct[1]*Parameter.Trajectory[0][1]));
	}
	if (f_sign_val >= 0)
		i8_TURN_RIGHT = 1;
	else i8_TURN_RIGHT = -1;
	/* Calculate cross-track error X:
	Cross-track error is the distance from current position to tracking segments 
	To determine distance from a point to a line with known perpendicular vector, apply x = abs(vct n . vct AB)/|vct n|
	where, vct n is a perpendicular vector, vct AB is a vector from the point to a point belonging to the line.*/
	point2line_vct[0] = Parameter.Trajectory[Parameter.Index][0] - Parameter.Coordinate[0][0];
	point2line_vct[1] = Parameter.Trajectory[Parameter.Index][1] - Parameter.Coordinate[0][1];
	f_x = fabs(perpen_vct[0]*point2line_vct[0] + perpen_vct[1]*point2line_vct[1])/sqrt(perpen_vct[0]*perpen_vct[0] + perpen_vct[1]*perpen_vct[1]);
	/* Calculate PHI*/
	/*Parameter.Steering_Angle  : Degree*/
	/*Parameter.Slope : Degree*/
	f_phi = Parameter.Slope[Parameter.Index];
	/*Rotate the angle which is less than 180*/
//	if ( f_phi > 180)
//		f_phi = f_phi - 360 ;
//	if (f_phi < -180 )
//		f_phi = f_phi +360;
	
	/* Calculate steering angle by using Stanley algorithm: DELTA = PHI + SIGN*ATAN(K*X/U)
	Then, New_Steering_Angle = DELTA + Old_Steering_Angle 
	Sign of the coordinate atan(kx/u) depends on the range of angles*/
	if ( Parameter.Velocity < 1.0)
		Parameter.Velocity = Constant_Speed;
	if (i8_TURN_RIGHT ==1 )
		Parameter.Steering_Angle = f_phi + atan(Parameter.K*f_x/Parameter.Velocity)*RAD2DEG;
	else
		Parameter.Steering_Angle =  f_phi - atan(Parameter.K*f_x/Parameter.Velocity)*RAD2DEG;
	if ( Parameter.Steering_Angle >= 360 )
		Parameter.Steering_Angle = 		Parameter.Steering_Angle -360;
	if ( Parameter.Steering_Angle < 0 )
		Parameter.Steering_Angle  = Parameter.Steering_Angle  + 360;
	/* Compare the distance from current point to current tracking point with the distance from current point to next one,
	consider to change to next tracking point.*/
	if (Parameter.Index < (Parameter.Size - 1))
		dDis_test = sqrt((Parameter.Coordinate[0][0] - Parameter.Trajectory[Parameter.Index + 1][0])*(Parameter.Coordinate[0][0] - Parameter.Trajectory[Parameter.Index + 1][0]) 
					+ (Parameter.Coordinate[0][1] - Parameter.Trajectory[Parameter.Index + 1][1])*(Parameter.Coordinate[0][1] - Parameter.Trajectory[Parameter.Index + 1][1]));
	else
		dDis_test = sqrt((Parameter.Coordinate[0][0] - Parameter.Trajectory[0][0])*(Parameter.Coordinate[0][0] - Parameter.Trajectory[0][0]) 
					+ (Parameter.Coordinate[0][1] - Parameter.Trajectory[0][1])*(Parameter.Coordinate[0][1] - Parameter.Trajectory[0][1]));
		if (dDis_test <= UPDATE_DISTANCE)
	{
		if (Parameter.Index < (Parameter.Size - 1))
			Parameter.Index += 1;
		else
			Parameter.Index = 0;
	}
}
void Stanley_Init(double Trajectory[][2], uint16_t Size, double K, double Start_Coordinate[][2])
{
	Stanley_Add_K(K);
	Stanley_Add_Coordinate(Start_Coordinate);
	Stanley_Set_Trajectory_Size(Trajectory, Size);
	Stanley_Slope();
	Stanley_Scan_Distance();
	//Stanley_Add_Steer_Ang(Start_Angle);
	//Stanley_Add_Velocity(Velocity);
}
double Stanley_Get_Steer_Ang(void)
{
	return Parameter.Steering_Angle;
	//Double2Str(Parameter.Steering_Angle,6,steering_send);
}
void Stanley_Set_Size(uint16_t Size)
{
	Parameter.Size = Size;
}
void Stanley_Test_Sample_Trajectory(float CoordinateX, float CoordinateY)
{
	Parameter.Trajectory[Parameter.Size][0] = CoordinateX;
	Parameter.Trajectory[Parameter.Size][1] = CoordinateY;
	Parameter.Size += 1;
}
