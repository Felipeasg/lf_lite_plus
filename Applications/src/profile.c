/**
  *************************************************************************************
  * \file    profile.c 
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



/*************************************** INCLUDES *************************************/

#include "profile.h"
#include "dcmotorcontrol.h"
#include "trigfunctions.h"


/******************************* DEFINITIONS AND MACROS *******************************/

/************************* TYPEDEFS, CLASSES AND STRUCTURES ***************************/

/************************************** PROTOTYPES ************************************/

/********************************** GLOBAL VARIABLES **********************************/

extern volatile float posErr[NUM_OF_SPEED];
extern volatile float posErrOld[NUM_OF_SPEED];

extern volatile float PIDInput[NUM_OF_SPEED];

volatile int32_t encoderSpeed[NUM_OF_WHEEL];

extern volatile float gyroW;

bool bWheelMotorEnable;

/********************************** LOCAL  VARIABLES **********************************/

bool bAlignLineFlag;
volatile int16_t alignSpeed;
volatile sPos robotPos;
volatile float robotPosInc[NUM_OF_SPEED];

volatile float curPos[NUM_OF_SPEED];
volatile float finalPos[NUM_OF_SPEED];

float curAcc[NUM_OF_SPEED] = {ACC_mm_ec(5000),		// curAcc[0] is for X-component
								ACC_deg_ec(2000)};	// curAcc[1] is for W-component

volatile float targetSpeed[NUM_OF_SPEED];
volatile float targetEndSpeed[NUM_OF_SPEED];
volatile float curSpeed[NUM_OF_SPEED];
volatile int32_t moveState[2];

bool bDistDirFlag[NUM_OF_SPEED];
float afterBrakeDist[NUM_OF_SPEED];
volatile int uncertainty;

bool gyroFeedbakEnabled = false;

bool lineSensorsFeedbackEnabled = false;


/******************************* FUNCTION  IMPLEMENTATION *****************************/

void ResetSpeedProfileData() {
	int i;
	//Fixme: critical
//	DI;
	for (i=0; i<NUM_OF_SPEED; i++) {
		curSpeed[i]=0;
		targetSpeed[i]=0;
		curPos[i]=0;
		moveState[i]=-1;
		posErr[i]=0;
		posErrOld[i]=0;
	}
	//Fixme: critical
//	EI;
}

// Robot speed is specified in term of translational speed and rotational speed
// The translational speed is effectively the sum of the left and right wheel speed.
// The rotaional speed is effectively the difference of the left and right wheel speed.
// xSpeed is in cm/sec
// wSpeed is in 10xDeg/sec
//
// ---------------------------------------------------------------------------------
// Set the robot targetSpeed
// The UpdateCurSpeed() function will update the curSpeed[] to the new targetSpeed[]
// according to the acceleration value in curAcc[]
// ---------------------------------------------------------------------------------

// @brief : To move in x-direction
// @param : xSpeed in mm/s
void SetRobotSpeedX( float xSpeed)
{
	targetSpeed[0] = SPEED_mm_ec(xSpeed);	// mm to optical count conversion
}

// @brief : To rotate robot
// @param : wSpeed in deg/s
void SetRobotSpeedW( float wSpeed)
{
	targetSpeed[1] = SPEED_deg_ec(wSpeed);
}

// @brief : To set acceleration for x-movement
// @param : acc in mm/s/s
void SetRobotAccX( float acc)
{
	curAcc[0] = ACC_mm_ec(acc);
}

// @brief : To set acceleration for rotational-movement
// @param : acc in deg/s/s
void SetRobotAccW( float acc)
{
	curAcc[1] = ACC_deg_ec(acc);
}

// ---------------------------------------------------------------------------------
// This function updates the curPos[] and PIDInput[] based on the curSpeed[].
// It is similar to UpdateRobotPos().
// This function is mainly used for the speed profile generator, whereas UpdateRobotPos()
// is used for keeping track of the robotX,Y,dir in real unit.
//
// Should combine the two functions in future.
// ---------------------------------------------------------------------------------
void UpdateWheelPos() {
	int i;

	robotPosInc[WSPEED]+=alignSpeed;	// alignment is also rotational

	for (i=0; i<NUM_OF_SPEED; i++) {

#if (USE_IDEAL_VELOCITY_TO_UPDATE_CURRENT_DISTANCE == true)


		if(i == XSPEED)
		{
			curPos[i] += (((float)((encoderSpeed[LEFT] + encoderSpeed[RIGHT])))/2);
		}
		if(i == WSPEED)
		{
			if(gyroFeedbakEnabled == true)
			{
				curPos[i] += SPEED_deg_ec(gyroW);
			}
			else
			{
				curPos[i] += (((float)(encoderSpeed[LEFT] - encoderSpeed[RIGHT]))/2);
			}
		}

		//curPos[i] += (int32_t)robotPosInc[i];	// add the whole number

		PIDInput[i] = curSpeed[i];
		PIDInput[i] *=2;								// There are 2 wheels, so
														// the encoder feedback are doubled

		// clear the whole number and keep the fraction
		//robotPosInc[i] = robotPosInc[i] - (long)(robotPosInc[i]);

#else


		robotPosInc[i] += curSpeed[i];

		curPos[i] += (int32_t)robotPosInc[i];	// add the whole number

		PIDInput[i] = curSpeed[i];
		PIDInput[i] *=2;								// There are 2 wheels, so
														// the encoder feedback are doubled

		// clear the whole number and keep the fraction
		robotPosInc[i] = robotPosInc[i] - (long)(robotPosInc[i]);
#endif
	}

}

// ---------------------------------------------------------------------------------
// This function will keep adjusting the curSpeed[] of the robot until it reaches the
// targetSpeed. The main function would have to set the targetSpeed, and the curAcc[] in
// order to control the speed profile.
// ---------------------------------------------------------------------------------
void UpdateCurSpeed() {
	register int i;

	for (i=0; i<NUM_OF_SPEED; i++) {

		if (curSpeed[i] < targetSpeed[i]) {
			curSpeed[i] += curAcc[i];
			if (curSpeed[i] > targetSpeed[i])
				curSpeed[i] = targetSpeed[i];
		}
		else if (curSpeed[i] > targetSpeed[i]) {
			curSpeed[i] -= curAcc[i];
			if (curSpeed[i] < targetSpeed[i])
				curSpeed[i] = targetSpeed[i];
		}
	}

}

// ---------------------------------------------------------------------------------
// Get the deceleration required
// Deceleration = (curSpeed*curSpeed - endSpeed*endSpeed)/(2*distToDec);
// ---------------------------------------------------------------------------------
float GetDecRequired(float dist, float afterBrakeDist, float curSpeed, float endSpeed) {

	if (dist<0 && curSpeed>=0) return 0;
	if (dist>0 && curSpeed<=0) return 0;
	if (curSpeed<0) curSpeed = -curSpeed;
	if (endSpeed<0) endSpeed = -endSpeed;
	if (dist<0) dist = -dist;
	if (endSpeed>=curSpeed) return 0;
	if (dist<afterBrakeDist) return ACC_mm_ec(9800); //FIXME: As vezes ele dá uma freiada brusca pode ser devido a esta desaceleração
	if (dist == 0) dist = 1;  //prevent divide by 0

	//(curXSpeed*curXSpeed - endXSpeed*endXSpeed)/(2*(finalPosX-curPosX-brakeAllowanceX))


	dist-=afterBrakeDist;

	return ((curSpeed*curSpeed - endSpeed*endSpeed)/(dist*2));
}

// ---------------------------------------------------------------------------------
// This function checks for end of current move
// A move ends when the distance set for the speed profile is reached.
// ---------------------------------------------------------------------------------
int EndOfMove(int moveCmd) {
	return moveState[moveCmd] == -1;
}

// ---------------------------------------------------------------------------------
// It set the targetSpeed[] of the robot according to the speed profile parameters.
// This is called from a regular interrupt.
// Both the Xspeed and Wspeed profile run in parallel
//
// The profile generator keeps checking the distance left to travel with the
// curAcc[]. When it's time to decelerate, it will set the targetSpeed[]to the
// targetEndSpeed[]
// ---------------------------------------------------------------------------------
void DoMoveCommand( ) {
	int i;

	float decelerationRequired;

	for (i=0; i<NUM_OF_SPEED; i++) {

		if (moveState[i] == -1) continue;	// No active profile

		decelerationRequired = GetDecRequired(finalPos[i]-curPos[i], afterBrakeDist[i], curSpeed[i], targetEndSpeed[i] );

		if (decelerationRequired>=ABS(curAcc[i]) ) {
			// Time to decelerate
#ifndef xxx
			if (moveState[i]==0) {
				moveState[i] = 1;
				targetSpeed[i] = targetEndSpeed[i];
				if (targetSpeed[i] == 0) {
					// when targetSpeed[i] == 0, the robot might not
					// be able to reach the final position due to
					// truncation error here and there.
					// solution is to temporarily set the speed slightly
					// more than zero.
					targetSpeed[i]=bDistDirFlag[i]?curAcc[i]:-curAcc[i];
				}
			}
#else
			moveState[i] = 1;
			targetSpeed[i] = targetEndSpeed[i];
#endif
			// set curAcc[i] to decelerationRequired else the robot
			// might not be able to reach targetSpeed[i] when the
			// target distance is reached. The result is that the robot
			// might need to stop at a high speed which caused it to
			// jerk.
			// Changing the curAcc[i] here means that we do not know what's
			// the curAcc[i] after a speed profile is completed and
			// we need to set curAcc[i] again. If you do not want
			// curAcc[i] to change, comment off the following line.

			// O bizarro é que se eu deixo esta linha após acabar o speed profile com um target speed de zero ele
			// mantém uma velocidade bem baixa.
//			curAcc[i] = decelerationRequired;
		}

		if (bDistDirFlag[i]) {
			// distance is towards positive direction
			if (finalPos[i]<=curPos[i]) {
				moveState[i] = -1;
				curSpeed[i] = targetSpeed[i] = targetEndSpeed[i];
				finalPos[i] = 0;
				curPos[i] = 0;

				gyroFeedbakEnabled = false;
			}
		}
		else {
			if (finalPos[i]>=curPos[i]) {
				moveState[i] = -1;
				curSpeed[i] = targetSpeed[i] = targetEndSpeed[i];
				finalPos[i] = 0;
				curPos[i] = 0;

				gyroFeedbakEnabled = false;
			}
		}


	}

}

void StopRobot(void)
{
	ResetSpeedProfileData();
}

// ---------------------------------------------------------------------------------
// @brief : To move Robot a certain distance. Distance can be translational (in mm) or
// 			rotational(in degree)
// @param  see SetMoveCommand()
void MoveRobot(int16_t speedType, float dist, float brakeDist, float topSpeed, float endSpeed, float acc) {

	SetMoveCommand(speedType, dist, brakeDist,  topSpeed, endSpeed, acc);

	while(!EndOfMove(speedType)) {
		// Do other stuff here!!!
		// like checking for sensors to detect object etc

		//TODO: Break The robot
//		if (bSWFlag) {	// user switch break!
//			break;
//		}
	}
}

// ---------------------------------------------------------------------------------
// Wait for the robot to travel certain distance. Must make sure the distance is less than
// final target distance
// ---------------------------------------------------------------------------------
void WaitDist(int16_t speedType, int32_t dist) {
	int32_t dist32;
	if (speedType == XSPEED) {
		dist32 = DIST_mm_ec(dist)+curPos[XSPEED];
	}
	else {
		dist32 = DIST_deg_ec(dist)+curPos[WSPEED];
	}
	if (bDistDirFlag[speedType]) {
		// positive direction
		while (dist32>curPos[speedType]);
	}
	else {
		while (dist32<curPos[speedType]);

	}

}

// ---------------------------------------------------------------------------------
// @brief : To set up the speed profile for movement.
// @param : speedType == 0 : XSPEED (forward/backward) movement
// 			speedType == 1 : WSPEED (rotational) movement
// @param : dist(mm or deg) is the distance(translational or angular) the robot will move
// @param :	topSpeed is the maxSpeed of the speed profile
// @param :	endSpeed is the ending speed of the profile (best not set to zero!!!!)
//			(else the robot may not reach the final distance due to truncation error)
// @param :	brakeDist is a parameter to get the robot to decelerate(brake) earlier so that
// 			there is a certain safety margin
//
// Note that both XSPEED and WSPEED profiles can be active at the same time.
// In that case, the robot will be moving in a curve.
//
// The profile generator runs regularly in the background through a ISR.
// It ends when the distance to move is reached or exceeded.

//Com velocidades angulares menores do que 300 e linear menor que duzentos (para fazer smooth path xspeed = 200 e wspeed = 300)
//fica quase perfeito, acima disto é interessante utilizar o giro para diminuir o erro de posição angular. A aceleracao angular
//tem que estar alta, por volta dos 10000
// ---------------------------------------------------------------------------------
void SetMoveCommand(int16_t speedType, float dist, int32_t brakeDist, float topSpeed, float endSpeed, float acc) {

	float dist32;
	if (speedType == XSPEED) {
		dist32 = DIST_mm_ec(dist);
		brakeDist = DIST_mm_ec(brakeDist);
		topSpeed = SPEED_mm_ec(topSpeed);
		endSpeed = SPEED_mm_ec(endSpeed);
		acc 	 = ACC_mm_ec(acc);
	}
	else {
		dist32 = DIST_deg_ec(dist);
		brakeDist = DIST_deg_ec(brakeDist);
		topSpeed = SPEED_deg_ec(topSpeed);
		endSpeed = SPEED_deg_ec(endSpeed);
		acc 	 = ACC_deg_ec(acc);
	}
	// Init speed profile parameters
	if (dist==0) {
		moveState[speedType]=-1;
		targetSpeed[speedType] = endSpeed;
		return;
	}
//FIXME: Critical
//	DI;
	finalPos[speedType] = dist32;
	curPos[speedType] = 0;
	afterBrakeDist[speedType]=brakeDist;

	if (dist<0) {
		topSpeed = -topSpeed;
		endSpeed = -endSpeed;
		bDistDirFlag[speedType]=false;		// reverse direction
	}
	else {
		bDistDirFlag[speedType]=true;		// forward (+ve) distance direction
	}

	targetSpeed[speedType] = topSpeed;
	targetEndSpeed[speedType] = endSpeed;
	curAcc[speedType] = acc;
	moveState[speedType] = 0;

#if USE_GYRO_FEEDBACK
	if(speedType == WSPEED )
	{
		gyroFeedbakEnabled = true;
	}
#endif

//FIXME: Critical
//	EI;
}

// ---------------------------------------------------------------------------------
// Use this routine only if you need to keep track of robot position.
// To update the robot position based on the curSpeed[].
// This allows the robot to keep track of it's position based on odometry (motor encoder).
// Output in robotPos.
// roboPos.x & robotPos.y contain the x&y coordinates in mm.
// robotPos.dir contains the direction in 0.1 deg.
//
// Following universal mathematical convention,
// positive angle is anticlockwise & negative is clockwise
// Similarly, Y-axis is positive in North or 90 degree direction &
// X-axis is positive in East or 0-degree direction
//
// It is important to follow this convention as we are using the standard
// trigo functions like sine() & cosine() to update the robot position
//
// Use compass or curSpeed[WSPEED]? Combined???
// ---------------------------------------------------------------------------------
float robotPosIncX, robotPosIncY, robotPosIncT;
float robotPosIncW;
#define CD_1MM		((int)(CNT_PER_CM)/10.0F)
#define CD_1DEG		((int)(CNT_PER_360DEG)/360.0F)
#define ALIGN_TOL	300
void UpdateRobotPos() {
	float inc;

	// If we are using the compass for robot direction, we simply assign the
	// compass value to the robotPos.dir
	// How to combine odometry and compass???????
//	if (bUseCompassForDir)
//		robotPos.dir = curCompassValue;

	robotPosIncT  += curSpeed[XSPEED];
	robotPosIncY += curSpeed[XSPEED]*Sine((int16_t)robotPos.dir)/10000.0F;
	robotPosIncX += curSpeed[XSPEED]*Cosine((int16_t)robotPos.dir)/10000.0F;
	robotPosIncW += curSpeed[WSPEED] + alignSpeed;


	inc = robotPosIncT/CD_1MM;
	// uncertainty is a cheap way of telling us how accurate the robot's
	// position is. The more the robot moves, the less accurate the position.
	// Robot position is most accurate when it first started of when
	// it has a chance to realign with external features, like detecting
	// a know line.
	if (inc) {
		robotPosIncT -= CD_1MM*inc;		// assume no negative XSPEED
		uncertainty += inc;
	}

	inc = robotPosIncX/CD_1MM;
	if (inc) {
		robotPosIncX -= CD_1MM*inc;
		robotPos.x += inc;
	}
	inc = robotPosIncY/CD_1MM;
	if (inc) {
		robotPosIncY -= CD_1MM*inc;
		robotPos.y += inc;
	}

	// If compass is not used, the robot W-speed is used to estimate the change
	// in robot direction.
	// If the robot is well calibrated, the angle derived from odometry is more accurate
	// in the short term. After a long series of moves (maybe more than a minute), it
	// might be better to trust the compass reading more.
	// It is important that there is no abrupt movement of robots in order for the
	// robot position to be accurate. That is, there shouldn't be any fast acceleration
	// or sudden top. If the robot is jerking, chances is that the robot position will
	// not be accurate.
	//if (!bUseCompassForDir)
	{
		inc = robotPosIncW/CD_1DEG;
		//DEBUGDATA0(robotPosIncW);
		//DEBUGDATA1(inc);
		//DEBUGDATA2(robotPos.dir);
		if (inc) {
			robotPosIncW -= CD_1DEG*inc;
			robotPos.dir += inc*10;
		}
		// Limit the direction to +/- 180 degrees (i.e. -1800 to 1800)
		// Unit is in 0.1 degree
		Limit180Deg(robotPos.dir);
	}
}

void DoSpeedProfile() {

	if (bWheelMotorEnable)
	{
		DoMoveCommand();		// speed profile

		UpdateCurSpeed();		// update curSpeed[] according to targetSpeed[]

		UpdateWheelPos();

		UpdateRobotPos();		// To update the robot's current position based on the robot

	}
	else
		ResetSpeedProfileData();
}


/*************************************** EOF ******************************************/

