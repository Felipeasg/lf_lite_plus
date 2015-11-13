/**
 *************************************************************************************
 * \file    uart1bsp.c
 * \author  Felipe Adriano
 * \version V1.0
 * \date    03/05/2015
 * \brief   Small description
 *************************************************************************************
 * description
 *
 * @attention
 *
 *
 *
 * <h2><center>&copy; COPYRIGHT </center></h2>
 *************************************************************************************
 */



/*************************************** INCLUDES *************************************/

#include "uart1bsp.h"

#include "ledsbsp.h"

#include "buffer.h"

/******************************* DEFINITIONS AND MACROS *******************************/

#define USARTx_IRQn			USART1_IRQn
#define USARTx_IRQHandler	USART1_IRQHandler

#define USARTx_TX_DMA_CHANNEL            DMA_CHANNEL_4
#define USARTx_TX_DMA_STREAM             DMA2_Stream7

#define RX_CIRC_BUFF_SIZE	512

//char RxBuffer[BUFFER_SIZE];
char RxByte;
uint32_t rx_available = 0;


/************************* TYPEDEFS, CLASSES AND STRUCTURES ***************************/

/************************************** PROTOTYPES ************************************/

/********************************** GLOBAL VARIABLES **********************************/

/********************************** LOCAL  VARIABLES **********************************/

UART_HandleTypeDef huart1;
DMA_HandleTypeDef hdma_tx;

static uint8_t u8_rxarrayU1[RX_CIRC_BUFF_SIZE];

static TBuffer T_rxcircBuffU1;


/******************************* FUNCTION  IMPLEMENTATION *****************************/


void uart1bsp_init(void)
{
	__GPIOA_CLK_ENABLE();	// Habilita o barramento de clock do GPIOA
	__USART1_CLK_ENABLE();	// Habilita o barramento de clock da USART1
	__DMA2_CLK_ENABLE();

	initBuffer(&T_rxcircBuffU1, u8_rxarrayU1, RX_CIRC_BUFF_SIZE);

	// Configura os GPIOs da USART1 como Alternate Function
	GPIO_InitTypeDef GPIO_InitStruct;
	GPIO_InitStruct.Pin = GPIO_PIN_9|GPIO_PIN_10;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
	GPIO_InitStruct.Alternate = GPIO_AF7_USART1;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

	// Configura��o do perif�rico USART
	huart1.Instance = USART1;
	huart1.Init.BaudRate = BAUD_RATE;
	huart1.Init.WordLength = UART_WORDLENGTH_8B;
	huart1.Init.StopBits = UART_STOPBITS_1;
	huart1.Init.Parity = UART_PARITY_NONE;
	huart1.Init.Mode = UART_MODE_TX_RX;
	huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart1.Init.OverSampling = UART_OVERSAMPLING_16;
	HAL_UART_Init(&huart1);

	HAL_NVIC_SetPriority(USARTx_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(USARTx_IRQn);


	/*##-3- Configure the DMA streams ##########################################*/
	/* Configure the DMA handler for Transmission process */
	hdma_tx.Instance                 = USARTx_TX_DMA_STREAM;
	hdma_tx.Init.Channel             = USARTx_TX_DMA_CHANNEL;
	hdma_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
	hdma_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
	hdma_tx.Init.MemInc              = DMA_MINC_ENABLE;
	hdma_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	hdma_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
	hdma_tx.Init.Mode                = DMA_NORMAL;
	hdma_tx.Init.Priority            = DMA_PRIORITY_LOW;
	hdma_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
	hdma_tx.Init.FIFOThreshold       = DMA_FIFO_THRESHOLD_FULL;
	hdma_tx.Init.MemBurst            = DMA_MBURST_INC4;
	hdma_tx.Init.PeriphBurst         = DMA_PBURST_INC4;

	HAL_DMA_Init(&hdma_tx);

	/* Associate the initialized DMA handle to the the UART handle */
	__HAL_LINKDMA(&huart1, hdmatx, hdma_tx);

	/*##-4- Configure the NVIC for DMA #########################################*/
	/* NVIC configuration for DMA transfer complete interrupt (USARTx_TX) */
	HAL_NVIC_SetPriority(DMA2_Stream7_IRQn, 1, 1);
	HAL_NVIC_EnableIRQ(DMA2_Stream7_IRQn);

	HAL_UART_Receive_IT(&huart1, &RxByte, 1);
}

/**
  * @brief  Rx Transfer completed callback
  * @param  UartHandle: UART handle
  * @note   This example shows a simple way to report end of IT Rx transfer, and
  *         you can add your own implementation.
  * @retval None
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle)
{
//    static uint32_t pos = 0;

//	HAL_UART_Receive_IT(UartHandle, (uint8_t*)&RxByte, 1);
//
//	bufferPutByte(&T_rxcircBuffU1, RxByte);


	ledsbsp_toogleOutputLed(LED4);
}


void DMA2_Stream7_IRQHandler(void)
{
	HAL_DMA_IRQHandler(huart1.hdmatx);
}

/**
 * This function checks if the circular buffer for data receive is empty and put the data
 * in a refer buff.
 * @param[in] buff pointer to the array where the data will be stored
 * @param[in] u16_size size of buff used to store data
 * @return the number of data placed in the circular buffer.
 */
uint32_t usart1bsp_GetNBytes(uint8_t *buff, uint32_t u32_size) {
    uint32_t u32_datareceived = 0;

    uint32_t u32_bufflength = bufferGetLength(&T_rxcircBuffU1);

    if ((u32_size <= T_rxcircBuffU1.array_size) && (u32_bufflength != 0)) {
        if (u32_size <= u32_bufflength) {
            bufferGetN(&T_rxcircBuffU1, buff, u32_size);
            u32_datareceived = u32_size;
        } else {
            bufferGetN(&T_rxcircBuffU1, buff, u32_bufflength);
            u32_datareceived = u32_bufflength;
        }
    }

    return u32_datareceived;
}

uint8_t uart1bsp_getByte(void)
{
	uint8_t u8_data;

	HAL_UART_Receive(&huart1, &u8_data, 1, 100);

	return u8_data;
}

void uart1bsp_sendData(void* data, uint32_t size)
{
	HAL_UART_DMAResume(&huart1);
	HAL_UART_Transmit_DMA(&huart1, (uint8_t*)data, size);
}

#ifdef STDIO_UART

/**
 * @brief _write redefinition to printf uart dump - stdio.h
 */
int _write(int file, char *ptr, int len)
{
	HAL_UART_Transmit(&huart1, ptr, len, USART_TIMEOUT);

	return len;
}


/**
 * @brief _read redefinition to scanf uart - stdio.h
 */
int _read (int file, char *ptr, int len)
{
	HAL_UART_Receive(&huart1, ptr, len, USART_TIMEOUT);
	return len;
}

#endif

/*************************************** EOF ******************************************/

