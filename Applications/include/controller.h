/**
  *************************************************************************************
  * \file    controller.h 
  * \author  fadriano
  * \version Vx.y.z
  * \date    Jul 29, 2015
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

#ifndef INCLUDE_CONTROLLER_H_
#define INCLUDE_CONTROLLER_H_

#ifdef __cplusplus
extern "C"
{
#endif //__cplusplus


/*************************************** INCLUDES *************************************/

#include <stdint.h>
#include "stm32f4xx_hal.h"

/******************************* DEFINITIONS AND MACROS *******************************/

//#define CONTROLER_DATALOG
#define SEND_SPEED

/************************* TYPEDEFS, CLASSES AND STRUCTURES ***************************/

/********************************** GLOBAL VARIABLES **********************************/

/********************************** GLOBAL FUNCTIONS **********************************/
                                      
void controller_Init(void);
int32_t controller_getSpeedX(void);
int32_t controller_getSpeedW(void);

#ifdef __cplusplus
}
#endif //__cplusplus

#endif /* INCLUDE_CONTROLLER_H_ */

/*************************************** EOF ******************************************/
