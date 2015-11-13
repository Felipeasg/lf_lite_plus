/**
  *************************************************************************************
  * \file    dcmotorcontrol.c 
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

#include "dcmotorcontrol.h"

//FreeRTOS
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

//libraries
#include "encodersbsp.h"
#include "motorsbsp.h"

#include "profile.h"

#include <arm_math.h>


/******************************* DEFINITIONS AND MACROS *******************************/

#define LEFTMOTORPWM		TIM1->CCR4
#define RIGHTMOTORPWM		TIM1->CCR1

#define LEFTMOTORENCODER	TIM2->CNT
#define RIGHTMOTORPENCODER  TIM5->CNT
#define MAXPWMVALUE			1000


/************************* TYPEDEFS, CLASSES AND STRUCTURES ***************************/

/************************************** PROTOTYPES ************************************/

static void readLineSensorFeedback(void);

/********************************** GLOBAL VARIABLES **********************************/

extern volatile float gyroW;

extern bool lineSensorsFeedbackEnabled;
extern volatile float lineSensorFeedback;
extern volatile float lineSensorActuation;

/********************************** LOCAL  VARIABLES **********************************/

/* Data for motor functions */
// PID gain constant
//float kp[NUM_OF_SPEED] = {2,		// kp[0] is for X-component
//							1};		// kp[1] is for W-component
//
//float kd[NUM_OF_SPEED] = {10,		// kd[0] is for X-component
//							1};		// kd[1] is for W-component

bool bWheelMotorEnable;
volatile bool bMotorISRFlag;
// These variables relate to individual wheel control
volatile int32_t wheelPWM[NUM_OF_WHEEL];		// wheelPWM[0] is for left wheel
												// wheelPWM[1] is for right wheel

volatile int32_t newEncoderData[NUM_OF_WHEEL];
volatile int32_t encoderSpeed[NUM_OF_WHEEL];
volatile int32_t oldEncoderData[NUM_OF_WHEEL];

// PID control variables
volatile float posPWM[NUM_OF_SPEED];
volatile float posErr[NUM_OF_SPEED];
volatile float posErrOld[NUM_OF_SPEED];
volatile float PIDFeedback[NUM_OF_SPEED];
volatile float PIDInput[NUM_OF_SPEED];


int32_t encoderChangeCnt;

bool bUseEncoderClickFlag;

//typedef struct
//{
//  float32_t A0;          /**< The derived gain, A0 = Kp + Ki + Kd . */
//  float32_t A1;          /**< The derived gain, A1 = -Kp - 2Kd. */
//  float32_t A2;          /**< The derived gain, A2 = Kd . */
//  float32_t state[3];    /**< The state array of length 3. */
//  float32_t Kp;               /**< The proportional gain. */
//  float32_t Ki;               /**< The integral gain. */
//  float32_t Kd;               /**< The derivative gain. */
//} arm_pid_instance_f32;

arm_pid_instance_f32 SpeedPID[2];

#if USE_GYRO_FEEDBACK
arm_pid_instance_f32 GyroPID;
#endif

extern bool gyroFeedbakEnabled;


/******************************* FUNCTION  IMPLEMENTATION *****************************/

void InitMotor(void)
{
	motorsbsp_init();
	encodersbsp_init();

	LEFTMOTORPWM = 0;
	RIGHTMOTORPWM = 0;

	DisWheelMotor();

	SpeedPID[XSPEED].Kp = 1.0;
	SpeedPID[XSPEED].Kd = 12;
	SpeedPID[XSPEED].Ki = 0.0;

	arm_pid_init_f32(&SpeedPID[XSPEED], true);


	SpeedPID[WSPEED].Kp = 2.9;
	SpeedPID[WSPEED].Kd = 14;
	SpeedPID[WSPEED].Ki = 0.0;

	arm_pid_init_f32(&SpeedPID[WSPEED], true);

#if USE_GYRO_FEEDBACK

	GyroPID.Kp = 0.210796;
	GyroPID.Kd = 0.02585;
	GyroPID.Ki = 0;

	arm_pid_init_f32(&GyroPID, true);
#endif


}

void EnWheelMotor() {
	bWheelMotorEnable = 1;
}

void DisWheelMotor() {
	bWheelMotorEnable = 0;
}

// ---------------------------------------------------------------------------------
// Motor PID for a wheel chair configuration robot
// Robot movement comprised 2 components, translational (forward & backward)
// and rotational movements.
// The PID is implemented according to the 2 movement components. This is because
// the Kp & Kd parameters might be different for both. Hence the performance for
// such a system should be better.
// ---------------------------------------------------------------------------------
void MotorPID(void)
{
	int i;

	// Read in encoder values.
	// The encoder values, encoderSpeed[] should be positive when robot is moving forward
	newEncoderData[0] = LEFTMOTORENCODER;//-TIM_GetCounter(TIM2);		// left wheel motor encoder
	newEncoderData[1] = RIGHTMOTORPENCODER;//TIM_GetCounter(TIM4);		// right wheel motor

	// wheel0 - left wheel
	// wheel1 - right wheel
	// speed0 - x speed
	// speed1 - w speed

	// Calculate encoder difference and accumulate it into the wheel error
	// Calculate encoder difference and accumulate it into the wheel error
	for (i=0; i<NUM_OF_WHEEL; i++) {
		encoderSpeed[i] = (newEncoderData[i] - oldEncoderData[i]);
		oldEncoderData[i] = newEncoderData[i];
	}

	// XSPEED is the sum of left and right wheels speed
	PIDFeedback[XSPEED] = (float)(encoderSpeed[0] + encoderSpeed[1]);

#if (USE_GYRO_FEEDBACK == true)

	if(gyroFeedbakEnabled == true)
	{
		PIDFeedback[WSPEED] = (SPEED_deg_ec(gyroW))*2.0;
	}
	else
	{
		PIDFeedback[WSPEED] = ((float)(encoderSpeed[1] - encoderSpeed[0]));
	}


#elif(USE_LINE_SENSOR_FEEDBACK == true)

	// WSPEED is the difference between left and right wheel
	// SInce antiClockwise is positive, we take right-left wheel speed
	PIDFeedback[WSPEED] = (float)(encoderSpeed[1] - encoderSpeed[0]);

	if(lineSensorsFeedbackEnabled == true)
	{
		PIDFeedback[WSPEED] += lineSensorActuation;
	}

#else
	// WSPEED is the difference between left and right wheel
	// SInce antiClockwise is positive, we take right-left wheel speed
	PIDFeedback[WSPEED] = (float)(encoderSpeed[1] - encoderSpeed[0]);
#endif

	// ---------------------------------------------------------------------------------
	// PID position control
	// ---------------------------------------------------------------------------------
#if (USE_GYRO_FEEDBACK == true)
	for (i=0; i<NUM_OF_SPEED; i++)
	{

		// Accumulate the speed error to get position error
		posErr[i] += PIDInput[i]-PIDFeedback[i];

		// Simple PD control
		//posPWM[i] = (kp[i]*posErr[i] + kd[i]*(posErr[i]-posErrOld[i]));

		if(gyroFeedbakEnabled == true)
		{
			if(i == XSPEED)
			{
				posPWM[i] = arm_pid_f32(&SpeedPID[i], posErr[i]);
			}
			else if((i == WSPEED))
			{
				posPWM[i] = arm_pid_f32(&GyroPID, posErr[i]);
			}
		}
		else
		{
			posPWM[i] = arm_pid_f32(&SpeedPID[i], posErr[i]);
		}

		posErrOld[i] = posErr[i];
	}
#else
	for (i=0; i<NUM_OF_SPEED; i++)
	{

		// Accumulate the speed error to get position error
		posErr[i] += PIDInput[i]-PIDFeedback[i];

		// Simple PD control
		//posPWM[i] = (kp[i]*posErr[i] + kd[i]*(posErr[i]-posErrOld[i]));

		posPWM[i] = arm_pid_f32(&SpeedPID[i], posErr[i]);
//		posErrOld[i] = posErr[i];
	}
#endif


	/////////////////////////////////////////////////////////
	// Calculate individual wheels PWM from X & W components
	/////////////////////////////////////////////////////////
	wheelPWM[0] = (int32_t)(posPWM[0] - posPWM[1]);
	wheelPWM[1] = (int32_t)(posPWM[0] + posPWM[1]);

	// Limit maximum PWM value
	for (i=0; i<NUM_OF_WHEEL; i++) {
		CLIP(wheelPWM[i], -(MAXPWMVALUE), (MAXPWMVALUE) );
	}

//FIXME: Critical
//	DI;		// Disable interrupt so that motor pwm and dir pin are set without interruption


	// In the following, the dir and pwm sign depends on the motor connection
	// To check SetPWM0() & SetPWM1() functions, set the values below.
	//wheelPWM[0] = wheelPWM[1] = 0;

	if (bWheelMotorEnable){
//		SetPWM0(wheelPWM[0]);
//		SetPWM1(wheelPWM[1]);
		motorsbsp_setPWM(wheelPWM[0], wheelPWM[1]);
	}
	else {
		if (!bUseEncoderClickFlag) {
//			SetPWM0(0);
//			SetPWM1(0);
			motorsbsp_setPWM(0,0);
		}
	}

//FIXME: Critical
//	EI;
	bMotorISRFlag = true;
}

/*************************************** EOF ******************************************/

