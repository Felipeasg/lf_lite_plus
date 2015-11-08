/**
  *************************************************************************************
  * \file    ledsbsp.c 
  * \author  Felipe Adriano
  * \version V1.0
  * \date    01/05/2015
  * \brief   This is the implementation to ledsbsp
  *************************************************************************************
  * 
  * This is the implementation of the ledsbps.
  * Important details about implementation should go in comments.
  *
  * @attention
  *
  * 
  *
  * <h2><center>&copy; COPYRIGHT 2011 ENTERPRISE</center></h2>
  *************************************************************************************
  */



/*************************************** INCLUDES *************************************/

#include "ledsbsp.h"

/******************************* DEFINITIONS AND MACROS *******************************/

/************************* TYPEDEFS, CLASSES AND STRUCTURES ***************************/

/************************************** PROTOTYPES ************************************/

/********************************** GLOBAL VARIABLES **********************************/

/********************************** LOCAL  VARIABLES **********************************/

/******************************* FUNCTION  IMPLEMENTATION *****************************/

void ledsbsp_init(void)
{
	  GPIO_InitTypeDef GPIO_InitStruct;

	  /* GPIO Ports Clock Enable */
	  __GPIOB_CLK_ENABLE();
	  __GPIOC_CLK_ENABLE();
	  __GPIOA_CLK_ENABLE();

	  /*Configure GPIO pins : PC0  */
	  GPIO_InitStruct.Pin = GPIO_PIN_0;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
	  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	  /*Configure GPIO pins : PA5 */
	  GPIO_InitStruct.Pin = GPIO_PIN_5;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
	  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	  /*Configure GPIO pins : PB10 PB11  PB5*/
	  GPIO_InitStruct.Pin = GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_5;
	  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	  GPIO_InitStruct.Pull = GPIO_NOPULL;
	  GPIO_InitStruct.Speed = GPIO_SPEED_FAST;
	  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

void ledsbsp_outputLed(EN_Leds en_led, GPIO_PinState en_pinstate)
{
	switch(en_led)
	{
	case LED1:
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_5, en_pinstate);
		break;
	case LED2:
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_0, en_pinstate);
		break;
	case LED3:
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, en_pinstate);
		break;
	case LED4:
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_10, en_pinstate);
		break;
	case LED5:
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11, en_pinstate);
		break;

	default:
		break;
	}
}

void ledsbsp_setAllLeds(void)
{
	ledsbsp_outputLed(LED1, HIGH);
	ledsbsp_outputLed(LED2, HIGH);
	ledsbsp_outputLed(LED3, HIGH);
	ledsbsp_outputLed(LED4, HIGH);
	ledsbsp_outputLed(LED5, HIGH);
}

void ledsbsp_resetAllLeds(void)
{
	ledsbsp_outputLed(LED1, LOW);
	ledsbsp_outputLed(LED2, LOW);
	ledsbsp_outputLed(LED3, LOW);
	ledsbsp_outputLed(LED4, LOW);
	ledsbsp_outputLed(LED5, LOW);
}

void ledsbsp_toogleOutputLed(EN_Leds en_led)
{
	switch(en_led)
		{
		case LED1:
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_5);
			break;
		case LED2:
			HAL_GPIO_TogglePin(GPIOC, GPIO_PIN_0);
			break;
		case LED3:
			HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
			break;
		case LED4:
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_10);
			break;
		case LED5:
			HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_11);
			break;

		default:
			break;
		}
}

/*************************************** EOF ******************************************/

