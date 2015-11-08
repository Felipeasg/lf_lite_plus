/**
  *************************************************************************************
  * \file    trigfunctions.c 
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

#include <stdint.h>
#include <stdbool.h>

#include "trigfunctions.h"

/******************************* DEFINITIONS AND MACROS *******************************/

/************************* TYPEDEFS, CLASSES AND STRUCTURES ***************************/

/************************************** PROTOTYPES ************************************/

/********************************** GLOBAL VARIABLES **********************************/

extern  const int16_t sineTab[];
extern const int16_t tangentTab[];


/********************************** LOCAL  VARIABLES **********************************/

/******************************* FUNCTION  IMPLEMENTATION *****************************/

// Taylor series
// To implement in future
// sin(x) = x - (x^3)/3! + (x^5)/5! - ...
// cos(x) = 1 - (x^2)/2! + (x^4)/4! - ...
// atan(x) = x - (x^3)/3! + (x^5)/5! - .... (same as sin(x)

// return the value of sine function.
// Input deg are in terms of 0.1 degrees. Range from -3600 to 3600
// Output is from -10000 to 10000, i.e. values are scaled up 10000!!!
int16_t Sine(int16_t deg) {

	int sign=1;
	while(deg>1800) deg-=3600;
	while(deg<-1800) deg+=3600;
	if (deg<0) {
		sign = -1;
		deg=-deg;
	}
	if (deg>900)
		deg = 1800-deg;
	if (sign==1)
		return sineTab[deg];
	else
		return -sineTab[deg];
}

int16_t Cosine(int16_t deg) {
	return Sine(deg+900);
}

// input values must be from 0-10000
int16_t ArcSine(int16_t sine) {
	int index = SineTabSize/2;
	int min = 0;
	int max = SineTabSize;
	while(1) {
		if (sine>sineTab[index]) {
			min = index+1;
		}
		else if (sine<sineTab[index]){
			max = index-1;
		}
		else break;
		index = (min+max)/2;
		if (min>=max) break;
	}
	return index;
}

int16_t ArcTangent(int16_t y, int16_t x) {
	int index = TangentTabSize/2;
	int min = 0;
	int max = TangentTabSize;
	int xsign=x, ysign=y;
	int tanValue;
	bool lowFlag;
	int quadrant;

	if (x<0) x = -x;
	if (y<0) y = -y;

	if (x>=y) {
		// from 0 to 45 degrees
		lowFlag = true;
		tanValue = y*TangentValueScale/x;
	}
	else {
		// from 45 to 90 degrees
		lowFlag = false;
		tanValue = x*TangentValueScale/y;
	}

	while(1) {
		if (tanValue>tangentTab[index]) {
			min = index+1;
		}
		else if (tanValue<tangentTab[index]){
			max = index-1;
		}
		else break;
		index = (min+max)/2;
		if (min>=max) break;
	}
	if (!lowFlag) {
		index = TangentTabSize*2 - index;
	}
	if (ysign>=0) {
		if (xsign>=0) quadrant = 0;
		else quadrant = 1;
	}
	else {
		if (xsign>=0) quadrant = 3;
		else quadrant = 2;
	}
	switch(quadrant) {
	case 1:
		index = Degree180 - index; break;
	case 2:
		index = -(Degree180 - index); break;
	case 3:
		index = - index; break;
	}
	return index;
}


/*************************************** EOF ******************************************/

