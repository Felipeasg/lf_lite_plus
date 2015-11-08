/**
  *************************************************************************************
  * \file    profile.h 
  * \author  felipeasg
  * \version Vx.y.z
  * \date    Aug 15, 2015
  * \brief   Small description
  *************************************************************************************
  * description
  * 
  * @attention
  *
  * 
  *
  * <h2><center>&copy; COPYRIGHT 2011 ENTERPRISE</center></h2>
  *************************************************************************************
  */

#ifndef INCLUDE_PROFILE_H_
#define INCLUDE_PROFILE_H_

#ifdef __cplusplus
extern "C"
{
#endif //__cplusplus


/*************************************** INCLUDES *************************************/

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "dcmotorcontrol.h"

/******************************* DEFINITIONS AND MACROS *******************************/

#define ABS(n)	((n)>=0?(n):(-n))		// Get absolute value

// ---------------------------------------------------------------------------------
//	Robot data (AlphaCentre Training robot)
//  Robot speed and distance control are done in real unit (mm and mm/sec)
//	The following macros allo us to easily convert real unit in mm & mm/sec into encoder counts
//  Change these constants to match that of your robot
#define ENCODER		(120.0F*4.0F)		//encoder resolution per revolution
#define W_DIAMETER	32.0F			// Wheel diameter
#define W_DISTANCE	83.0F			// Wheel to wheel distance
#define GEAR_RATIO	(30.0F/1.0F)	// Wheel to motor gear ratio
#define _PI_		M_PI//3.142
#define CNT_PER_CM	((ENCODER*10)*(GEAR_RATIO)/(W_DIAMETER*_PI_))

// Speed and acceleration variables used fixed point representation with lower n bits
// for decimal point. This is necessary as slower finer speed/acceleration is required
// Macro convert speed to virtual encoder count per PID_period
// PID loop interval is 1 msec

#define CNT_PER_CM_PER_MS	((CNT_PER_CM)/1000)

// This assumes a wheel chair configuration for the robot
// For the robot to turn 360 degrees, the wheel has to travel PI*W_DISTANCE (assumindo o robo girando em torno do proprio eixo)
#define CNT_PER_360DEG			((ENCODER*W_DISTANCE)*((GEAR_RATIO)/(W_DIAMETER)))
#define CNT_PER_360DEG_PER_MS	((CNT_PER_360DEG)/1000.0F)

// ---------------------------------------------------------------------------------
// These macros are used to convert real unit to optical count
// This is necessary as ultimately, the motor feedback is in term of encoder counts
// ---------------------------------------------------------------------------------
#define FIXED_PT_SCALING	64 //128

#define SPEED_mm_ec(n)	((CNT_PER_CM_PER_MS*(n))/10.0F)
											// Convert speed (mm/s) to (oc/ms)
											// oc refers to optical count
											// Note max speed is about 4m/s, before overflow
#define ACC_mm_ec(n)	((CNT_PER_CM_PER_MS*(n))/10000.0F)
											// Convert acceleration (mm/s/s) to
											// (oc/ms/ms)
#define SPEED_deg_ec(n)	((CNT_PER_360DEG_PER_MS*(n))/360.0F)

#define ACC_deg_ec(n)	((CNT_PER_360DEG_PER_MS*(n))/360000.0F)


#define DIST_mm_ec(dist)	(((CNT_PER_CM)*(dist))/10.0F)	// Convert distance to optical count
//#define ANGLE_ADJUST		950.0F							// adjust this value to make 360 accurate if necessary with encoders without gyro
#define ANGLE_ADJUST		0								// adjust this value to make 360 accurate if necessary
#define DIST_deg_ec(dist)	(((CNT_PER_360DEG+ANGLE_ADJUST)*(dist))/360.0F)
#define DIST_deg_ecf(dist)	(((CNT_PER_360DEG+ANGLE_ADJUST)*(dist))/360.0F)


////////////////////////////////////////////////////////////////////////////////////
// There are 2 wheels. It is convenient to store wheels' related data into array
////////////////////////////////////////////////////////////////////////////////////
#define NUM_OF_WHEEL	2		// 0 - left
								// 1 - right
#define LEFT_WHEEL		0
#define RIGHT_WHEEL		1

// There are 2 kind of speeds and distance, linear x-speed and rotational w-speed.
// It is also convenient to theis 2 kinds of related data into array
#define NUM_OF_SPEED	2		// 0 - xSpeed
								// 1 - wSpeed (angular)
#define 	XSPEED		0
#define 	WSPEED		1

#define 	RIGHT	0
#define 	LEFT	1

#define USE_IDEAL_VELOCITY_TO_UPDATE_CURRENT_DISTANCE true

/************************* TYPEDEFS, CLASSES AND STRUCTURES ***************************/

/************************* TYPEDEFS, CLASSES AND STRUCTURES ***************************/

typedef struct {
	float x;		// unit in mm
	float y;
	float dir;	// unit in 0.1 degree
} sPos;


/********************************** GLOBAL VARIABLES **********************************/

/********************************** GLOBAL FUNCTIONS **********************************/

// ---------------------------------------------------------------------------------
// Reset the drive motor speed profile internal data
// This should be called once only after the motor drive wheels are just enabled
// User should never have to call this function directly.
// ---------------------------------------------------------------------------------
void ResetSpeedProfileData();

// ---------------------------------------------------------------------------------
// The following functions are used to set the current target speed of the robot.
// Not to be used with the MoveRobot() or SetMoveCommand()
// ---------------------------------------------------------------------------------
void SetRobotSpeedX( float xSpeed);
void SetRobotSpeedW( float wSpeed);
void SetRobotAccX( float acc);
void SetRobotAccW( float acc);

void UpdateWheelPos();
void UpdateCurSpeed();

float GetDecRequired(float dist, float afterBrakeDist, float curSpeed, float endSpeed);
int EndOfMove(int moveCmd);
void DoMoveCommand();

void StopRobot(void);
void MoveRobot(int16_t speedType, float dist, float brakeDist, float topSpeed, float endSpeed, float acc);
void SetMoveCommand(int16_t speedType, float dist, int32_t brakeDist, float topSpeed, float endSpeed, float acc);
void UpdateRobotPos();
void DoSpeedProfile();

void WaitDist(int16_t speedType, int32_t dist);

#ifdef __cplusplus
}
#endif //__cplusplus

#endif /* INCLUDE_PROFILE_H_ */

/*************************************** EOF ******************************************/
