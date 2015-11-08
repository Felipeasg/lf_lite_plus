/**
  *************************************************************************************
  * \file    dcmotorcontrol.h 
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

#ifndef INCLUDE_DCMOTORCONTROL_H_
#define INCLUDE_DCMOTORCONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif //__cplusplus


/*************************************** INCLUDES *************************************/

#include <stdint.h>
#include <stdbool.h>

#include "stm32f4xx_hal.h"



/******************************* DEFINITIONS AND MACROS *******************************/

#define CLIP(n, min, max)	((n)=(n)>(max)?(max):(n)<(min)?(min):(n))

#define LEFTMOTORENCODER	TIM2->CNT
#define RIGHTMOTORPENCODER  TIM5->CNT

#define USE_GYRO_FEEDBACK	true
/********************************** GLOBAL VARIABLES **********************************/

/********************************** GLOBAL FUNCTIONS **********************************/
                                      
void InitMotor(void);

void DisWheelMotor();	// Disable the 2 drive wheels
void EnWheelMotor() ;	// Enable the 2 drive wheels

void MotorPID(void);		// Does the motor PID and robot move speed profile
							// Called from a regular interrupt routine

#ifdef __cplusplus
}
#endif //__cplusplus

#endif /* INCLUDE_DCMOTORCONTROL_H_ */

/*************************************** EOF ******************************************/
