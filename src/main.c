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

#include "cmdline_parser.h"

#include "delay.h"

void vApplicationTickHook( void )
{
	HAL_IncTick();
}


extern bool lineSensorsFeedbackEnabled;

static void mainTask(void* pvParameters)
{
	//CMDLINE_CONTEXT cmd_context;

	//cmd_context.out = printf;
	//cmdline_init(&cmd_context);

	static bool flag = false;
	char c=0;
	float vel = 0;

	bool bluetooth = false;

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

	if(bluetooth == false)
	{
		EnWheelMotor();
		vel = 900;
	}


	lineSensorsFeedbackEnabled = true;



	SetRobotAccX(800);
	SetRobotSpeedX(vel);

	//	DisWheelMotor();

	for(;;)
	{
		if(bluetooth == true)
		{
			if(usart1bsp_GetNBytes((uint8_t*)&c,1) > 0)
			{
				if(c == 'm')
				{
					if(flag == false)
					{
						EnWheelMotor();
						SetRobotSpeedX(600);
						flag = true;

					}
					else
					{
						SetRobotSpeedX(0);
						DisWheelMotor();
						flag = false;

					}
				}
				if(c == 'p')
				{
					vel += 50;
					SetRobotSpeedX(vel);
				}
				if(c == 'n')
				{
					vel -= 50;
					SetRobotSpeedX(vel);
				}

				ledsbsp_toogleOutputLed(LED1);
			}
			vTaskDelay(100);
		}
		else
		{

			vTaskDelay(100);
			ledsbsp_toogleOutputLed(LED1);
		}
		vTaskDelay(100);
					ledsbsp_toogleOutputLed(LED1);
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

	xTaskCreate(mainTask, "mt", (configMINIMAL_STACK_SIZE*5), NULL, tskIDLE_PRIORITY, NULL);

	vTaskStartScheduler();

	// Infinite loop
	for(;;);

	return 0;
}

#pragma GCC diagnostic pop

// ----------------------------------------------------------------------------
