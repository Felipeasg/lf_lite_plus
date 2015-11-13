/**
 ******************************************************************************
 * @file    stm32f4xx_it.c
 * @date    19/03/2015 23:56:50
 * @brief   Interrupt Service Routines.
 ******************************************************************************
 *
 * COPYRIGHT(c) 2015 STMicroelectronics
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"

#include "basetypes.h"
#include "cmdline_parser.h"
//#include "main.h"
//#include "buzzer.h"
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

extern UART_HandleTypeDef huart1;

char  serial_buffer[100];

/******************************************************************************/
/*            Cortex-M4 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
 * @brief This function handles System tick timer.
 */
//void SysTick_Handler(void)
//{
//	/* USER CODE BEGIN SysTick_IRQn 0 */
////	if(buzzerTime < 0)
////	{
////		BEEP_OFF;
////	}
////	buzzerTime--;
//
////	systick();
////	/* USER CODE END SysTick_IRQn 0 */
////
////	HAL_IncTick();
////	HAL_SYSTICK_IRQHandler();
//
//	/* USER CODE BEGIN SysTick_IRQn 1 */
//
//	/* USER CODE END SysTick_IRQn 1 */
//}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
 * @brief  This function handles TIM interrupt request.
 * @param  None
 * @retval None
 */
void TIM3_IRQHandler(void)
{
	//  HAL_TIM_IRQHandler(&htim3);
}


/**
 * @brief  This function handles UART interrupt request.
 * @param  None
 * @retval None
 */
void USART1_IRQHandler(void)
{
	HAL_UART_IRQHandler(&huart1);

	/* USER CODE BEGIN USART3_IRQn 0 */
	uint32_t      uart_status_flag;
	uint32_t      uart_it_flag;
	static char   *pBuffer = serial_buffer;
	unsigned char c;
	/* USER CODE END USART3_IRQn 0 */
	HAL_UART_IRQHandler(&huart1);
	/* USER CODE BEGIN USART3_IRQn 1 */

	// Get UART flags
	uart_status_flag = __HAL_UART_GET_FLAG(&huart1, UART_FLAG_RXNE);
	uart_it_flag = __HAL_UART_GET_IT_SOURCE(&huart1, UART_IT_RXNE);

	if ((uart_status_flag != RESET) && (uart_it_flag != RESET))
	{
		// Clear interrupt flag
		__HAL_UART_CLEAR_FLAG(&huart1, UART_FLAG_RXNE);

		// Get the character received
		c = (uint16_t)(huart1.Instance->DR & (uint16_t)0x01FF);

		switch (c)
		{
		case CMDLINE_CR:
			// Carriage Return
			cmdline_ctxt.cmd_len = (pBuffer - serial_buffer) + 1;
			cmdline_ctxt.cmd_received = TRUE;
			*pBuffer = c;
			pBuffer = serial_buffer;
			return;

		case CMDLINE_LF:
			// Line Feed
			pBuffer = serial_buffer;
			break;

		case CMDLINE_BS:
			// Backspace
			if (pBuffer == serial_buffer)
			{
				HAL_UART_Transmit(&huart1, (unsigned char *)"\x07", 1, 100);
			}
			else
			{
				HAL_UART_Transmit(&huart1, (unsigned char *)"\x08\x7F", 2, 100);
				pBuffer--;
			}
			break;

		default:
			/* Echo received character */
			HAL_UART_Transmit(&huart1, &c, 1, 100);
			*pBuffer = c;
			pBuffer++;
			break;
		}

	}
}

	/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
