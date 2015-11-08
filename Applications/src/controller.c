/**
  *************************************************************************************
  * \file    controller.c 
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



/*************************************** INCLUDES *************************************/

#include "controller.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"

#include "encodersbsp.h"
#include "motorsbsp.h"
#include "sensorsbsp.h"
#include "uart1bsp.h"
#include "ledsbsp.h"
#include "dcmotorcontrol.h"
#include "profile.h"

#include <stdbool.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

/******************************* DEFINITIONS AND MACROS *******************************/

#define LINE_SENSOR_GAIN	0.08F

/************************* TYPEDEFS, CLASSES AND STRUCTURES ***************************/

/************************************** PROTOTYPES ************************************/

static void ControlTask(void* pvParameters);
static void readLineSensor(void);

/********************************** GLOBAL VARIABLES **********************************/

extern volatile float PIDFeedback[NUM_OF_SPEED];
extern volatile float PIDInput[NUM_OF_SPEED];
extern volatile float curSpeed[NUM_OF_SPEED];

extern volatile int32_t curPos[NUM_OF_SPEED];

extern volatile int32_t finalPos[NUM_OF_SPEED];

extern volatile float gyroW;
extern volatile float gyroOffset;

/********************************** LOCAL  VARIABLES **********************************/

uint32_t idx = 0;
char header[4] = {0xaa,0xaa,0xaa,0xaa};
float samples[200];
uint8_t data[804];

volatile float lineSensorFeedback = 0;
volatile float oldLineSensorFeedback = 0;

/******************************* FUNCTION  IMPLEMENTATION *****************************/

void controller_Init(void)
{
	//FIXME: se o encoder inicializa antes do motor o encoder da esquerda para de funcionar (pq?)
//	motorsbsp_init(); //motor driver to hardware layer
//	encodersbsp_init();

	//sensorsbsp_init();


	//Motor control init will init encoder and motorbsp drivers to hardware



	InitMotor();

	ResetSpeedProfileData();


	xTaskCreate(ControlTask, "V Control Task", (configMINIMAL_STACK_SIZE*3), NULL, (tskIDLE_PRIORITY+1), NULL);
}



static void ControlTask(void* pvParameters)
{
	(void)pvParameters;


	TickType_t xLastWakeTime = xTaskGetTickCount();

	//vTaskDelay(1000);

	for(;;)
	{

		readLineSensor();

		DoSpeedProfile();

		MotorPID();


		samples[idx] = lineSensorFeedback;
		samples[idx+1] = 0;

		idx += 2;
		if(idx == 200)
		{
			memcpy(data, header, 4);
			memcpy(&data[4], samples, 800);
			uart1bsp_sendData(data, 804);

			idx = 0;
		}


		vTaskDelayUntil(&xLastWakeTime, 1);

	}
}

static void readLineSensor(void)
{
	float lineError = sensorsbsp_getLineSensors();

	if(lineError < INFINITO)
	{
		lineSensorFeedback = lineError * LINE_SENSOR_GAIN;

		oldLineSensorFeedback = lineSensorFeedback;
	}
	else
	{
		lineSensorFeedback = oldLineSensorFeedback;
	}

}


/*************************************** EOF ******************************************/

