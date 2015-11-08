/**
  *************************************************************************************
  * \file    trigfunctions.h 
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

#ifndef INCLUDE_TRIGFUNCTIONS_H_
#define INCLUDE_TRIGFUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif //__cplusplus


/*************************************** INCLUDES *************************************/

/******************************* DEFINITIONS AND MACROS *******************************/

#define Degree180			1800
#define Degree90			900
#define SineTabSize			900
#define SineValueScale		10000
#define TangentTabSize		450
#define TangentValueScale	10000
#define Limit180Deg(angle)	(angle=(angle>1800)?(-3600+angle):((angle<-1800)?(3600+angle):angle))

/************************* TYPEDEFS, CLASSES AND STRUCTURES ***************************/

/********************************** GLOBAL VARIABLES **********************************/

/********************************** GLOBAL FUNCTIONS **********************************/

// return the value of sine function.
// Input deg are in terms of 0.1 degrees. Range from -3600 to 3600
// Output is from -10000 to 10000, i.e. values are scaled up 10000!!!
int16_t Sine(int16_t deg);
int16_t Cosine(int16_t deg);
int16_t ArcSine(int16_t sine);
int16_t ArcTangent(int16_t y, int16_t x);


#ifdef __cplusplus
}
#endif //__cplusplus

#endif /* INCLUDE_TRIGFUNCTIONS_H_ */

/*************************************** EOF ******************************************/
