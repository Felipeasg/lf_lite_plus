#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"

#include "uart1bsp.h"
#include "buttonsbsp.h"
#include "encodersbsp.h"
#include "sensorsbsp.h"
#include "motorsbsp.h"
#include "controller.h"
#include "ledsbsp.h"
#include "buzzerbsp.h"

#include "dcmotorcontrol.h"
#include "profile.h"

#include "delay.h"


void vApplicationTickHook( void )
{
	HAL_IncTick();
}

int32_t dados[10] = {0xff};
static void mainTask(void* pvParameters)
{
	(void)pvParameters;

	ledsbsp_toogleOutputLed(LED1);
	vTaskDelay(500);
	ledsbsp_toogleOutputLed(LED1);
	vTaskDelay(500);
	ledsbsp_toogleOutputLed(LED1);
	vTaskDelay(500);
	ledsbsp_toogleOutputLed(LED1);
	vTaskDelay(500);
	ledsbsp_toogleOutputLed(LED1);
	vTaskDelay(500);
	ledsbsp_toogleOutputLed(LED1);
	vTaskDelay(500);

	controller_Init();

	EnWheelMotor();

	float maxLinearSpeed = 800;
	float endLinearSpeed = 0;
	float maxAngularSpeed = 500;
	float accX = 2200;
	float accW = 8000;

	MoveRobot(XSPEED, 5000, 0, maxLinearSpeed, 0, accX);

#if 0
	for(int i = 0; i < 8; i++)
	{
		SetMoveCommand(XSPEED, 180, 0, maxLinearSpeed, maxLinearSpeed, accX);
		WaitDist(XSPEED, 150);
		buzzerbsp_beep(ON);
		vTaskDelay(10);
		buzzerbsp_beep(OFF);
		while(!EndOfMove(XSPEED));

		MoveRobot(WSPEED, -90, 0, maxAngularSpeed, 0, accW);
	}

	SetMoveCommand(XSPEED, 180, 0, maxLinearSpeed, endLinearSpeed, accX);
	WaitDist(XSPEED, 150);
	buzzerbsp_beep(ON);
	vTaskDelay(10);
	buzzerbsp_beep(OFF);
	while(!EndOfMove(XSPEED));
#endif

#if 0
	for(int i = 0; i < 4; i++)
	{
		SetMoveCommand(XSPEED, 180, 0, maxLinearSpeed, maxLinearSpeed, accX);
		WaitDist(XSPEED, 150);
		buzzerbsp_beep(ON);
		vTaskDelay(10);
		buzzerbsp_beep(OFF);
		while(!EndOfMove(XSPEED));

		MoveRobot(WSPEED, -90, 0, maxAngularSpeed, 0, accW);


		SetMoveCommand(XSPEED, 180, 0, maxLinearSpeed, maxLinearSpeed, accX);
		WaitDist(XSPEED, 150);
		buzzerbsp_beep(ON);
		vTaskDelay(10);
		buzzerbsp_beep(OFF);
		while(!EndOfMove(XSPEED));

		MoveRobot(WSPEED, 90, 0, maxAngularSpeed, 0, accW);

		SetMoveCommand(XSPEED, 180, 0, maxLinearSpeed, maxLinearSpeed, accX);
		WaitDist(XSPEED, 150);
		buzzerbsp_beep(ON);
		vTaskDelay(10);
		buzzerbsp_beep(OFF);
		while(!EndOfMove(XSPEED));

		MoveRobot(WSPEED, -90, 0, maxAngularSpeed, 0, accW);

		SetMoveCommand(XSPEED, 180, 0, maxLinearSpeed, maxLinearSpeed, accX);
		WaitDist(XSPEED, 150);
		buzzerbsp_beep(ON);
		vTaskDelay(10);
		buzzerbsp_beep(OFF);
		while(!EndOfMove(XSPEED));

		MoveRobot(WSPEED, -90, 0, maxAngularSpeed, 0, accW);

		SetMoveCommand(XSPEED, 180, 0, maxLinearSpeed, maxLinearSpeed, accX);
		WaitDist(XSPEED, 150);
		buzzerbsp_beep(ON);
		vTaskDelay(10);
		buzzerbsp_beep(OFF);
		while(!EndOfMove(XSPEED));

		SetMoveCommand(XSPEED, 180, 0, maxLinearSpeed, maxLinearSpeed, accX);
		WaitDist(XSPEED, 150);
		buzzerbsp_beep(ON);
		vTaskDelay(10);
		buzzerbsp_beep(OFF);
		while(!EndOfMove(XSPEED));

		MoveRobot(WSPEED, -90, 0, maxAngularSpeed, 0, accW);

		SetMoveCommand(XSPEED, 180, 0, maxLinearSpeed, maxLinearSpeed, accX);
		WaitDist(XSPEED, 150);
		buzzerbsp_beep(ON);
		vTaskDelay(10);
		buzzerbsp_beep(OFF);
		while(!EndOfMove(XSPEED));

		SetMoveCommand(XSPEED, 180, 0, maxLinearSpeed, maxLinearSpeed, accX);
		WaitDist(XSPEED, 150);
		buzzerbsp_beep(ON);
		vTaskDelay(10);
		buzzerbsp_beep(OFF);
		while(!EndOfMove(XSPEED));

		MoveRobot(WSPEED, -90, 0, maxAngularSpeed, 0, accW);
	}

	SetMoveCommand(XSPEED, 180, 0, maxLinearSpeed, endLinearSpeed, accX);
	WaitDist(XSPEED, 150);
	buzzerbsp_beep(ON);
	vTaskDelay(10);
	buzzerbsp_beep(OFF);
	while(!EndOfMove(XSPEED));
#endif




	for(;;)
	{
		ledsbsp_toogleOutputLed(LED1);

		vTaskDelay(300);
	}
}
// ----- main() ---------------------------------------------------------------



int main(void)
{
	// At this stage the system clock should have already been configured
	// at high speed.


	uart1bsp_init();
	sensorsbsp_init();
	ledsbsp_init();
	buzzerbsp_init();

//	motorsbsp_init();
//	encodersbsp_init();


	xTaskCreate(mainTask, "mt", (configMINIMAL_STACK_SIZE*3), NULL, tskIDLE_PRIORITY, NULL);

	vTaskStartScheduler();

	// Infinite loop
	for(;;);

	return 0;
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
