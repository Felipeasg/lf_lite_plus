/**
  *************************************************************************************
  * \file    sensorsbsp.h 
  * \author  Felipe Adriano <felipeadrianosg@gmail.com>
  * \version V1.0.0
  * \date    07/05/2015
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

#ifndef BSP_SENSORSBSP_H_
#define BSP_SENSORSBSP_H_

#ifdef __cplusplus
extern "C"
{
#endif //__cplusplus


/*************************************** INCLUDES *************************************/

#include "stm32f4xx.h"
//#include "main.h"


/******************************* DEFINITIONS AND MACROS *******************************/

#define INFINITO	88888		// Valor que indica a não identificação de linha

// Constantes para calibrar o valor da tensão medida
#define VBAT_RAW	3434
#define VBAT_V		8270

#define GYRO_TIME_FREQ			2734.375F

#define GYRO_VRATIO				3300.0F	//Gyro is running at 3300mV
#define GYRO_SENSITIVITY		0.67F   	//Our example gyro is 6mV/deg/sec @5V
#define ADC_RESOLUTION			16383.0  //Oversamples 2 bits on 12 bits dac

#define GYRO_COEFF				(GYRO_VRATIO/(ADC_RESOLUTION*GYRO_SENSITIVITY)) // Convert gyro to dps

#define LF_INDEX	0
#define L_INDEX		2
#define R_INDEX		1
#define RF_INDEX	3

//Mean move average filter window
#define MEAN_WINDOW 10

#define NUM_SENSOR		4


#define HAS_RIGTH_WALL_TRESHOLD				350
#define NO_HAS_RIGTH_WALL_TRESHOLD			200

#define HAS_LEFT_WALL_TRESHOLD				350
#define NO_HAS_LEFT_WALL_TRESHOLD			200

#define HAS_RIGTHFRONT_WALL_TRESHOLD		300
#define NO_HAS_RIGTHFRONT_WALL_TRESHOLD		200

#define HAS_LEFTFRONT_WALL_TRESHOLD			300
#define NO_HAS_LEFTFRONT_WALL_TRESHOLD		200

#define PAREDE_ESQUERDA 0b100
#define PAREDE_FRONTAL  0b010
#define PAREDE_DIREITA  0b001


#define LINE_SENSOR_RESOLUTION_HIGH

/************************* TYPEDEFS, CLASSES AND STRUCTURES ***************************/

/********************************** GLOBAL VARIABLES **********************************/

/********************************** GLOBAL FUNCTIONS **********************************/

/**
 * @brief
 *
 * @return void
 */
void sensorsbsp_init(void);


/**
 * @brief
 *
 * @param
 * @param
 * @param
 * @param
 * @return
 */
void sensorsbsp_getWallSensors(int32_t* lf, int32_t* l, int32_t* r, int32_t* rf);

/**
 * @brief
 *
 * @return
 */

int32_t sensorsbsp_getWallSensorsInMilimeter(float* lf_mm, float* l_mm, float* r_mm, float* rf_mm);


int32_t sensorsbsp_getLineSensors(void);

/**
 * @brief
 *
 * @return
 */
int32_t sensorsbsp_getGyro();

/**
 * @brief
 *
 * @return
 */
int32_t sensorsbsp_getGyroVoltage(void);

int32_t sensorsbsp_getGyroVref(void);

void sensorsbsp_gyroCalibrate(uint32_t nbrCicles);



#ifdef __cplusplus
}
#endif //__cplusplus

#endif /* BSP_SENSORSBSP_H_ */

/*************************************** EOF ******************************************/
