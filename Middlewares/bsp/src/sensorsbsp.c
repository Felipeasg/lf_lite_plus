/**
 *************************************************************************************
 * \file    sensorsbsp.c
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



/*************************************** INCLUDES *************************************/

#include <stdint.h>
#include <stdbool.h>

#include "sensorsbsp.h"
#include "ledsbsp.h"

#include "delay.h"

//#include "FreeRTOS.h"
//#include "task.h"

/******************************* DEFINITIONS AND MACROS *******************************/

#define LOW		GPIO_PIN_RESET
#define HIGH	GPIO_PIN_SET
#define LINHA	GPIO_PIN_RESET	// RESET (linha branca); SET (linha preta)

#define SENSORES_CLK	__GPIOA_CLK_ENABLE(); __GPIOB_CLK_ENABLE(); __GPIOC_CLK_ENABLE(); __GPIOD_CLK_ENABLE()

#define N_EMISSORES	6
#define LF_E_PORT	GPIOD
#define LF_E_PIN	GPIO_PIN_2
#define POWER1_PORT	GPIOC
#define POWER1_PIN	GPIO_PIN_12
#define POWER2_PORT	GPIOC
#define POWER2_PIN	GPIO_PIN_11
#define RF_E_PORT	GPIOC
#define RF_E_PIN	GPIO_PIN_10
#define L_LINE_PORT	GPIOA
#define L_LINE_PIN	GPIO_PIN_14
#define R_LINE_PORT	GPIOA
#define R_LINE_PIN	GPIO_PIN_13

#define N_RECEPTORES	8
#define LINE1_PORT	GPIOC
#define LINE1_PIN	GPIO_PIN_9
#define LINE2_PORT	GPIOC
#define LINE2_PIN	GPIO_PIN_8
#define LINE3_PORT	GPIOC
#define LINE3_PIN	GPIO_PIN_7
#define LINE4_PORT	GPIOC
#define LINE4_PIN	GPIO_PIN_6
#define LINE5_PORT	GPIOB
#define LINE5_PIN	GPIO_PIN_15
#define LINE6_PORT	GPIOB
#define LINE6_PIN	GPIO_PIN_14
#define LINE7_PORT	GPIOB
#define LINE7_PIN	GPIO_PIN_13
#define LINE8_PORT	GPIOB
#define LINE8_PIN	GPIO_PIN_12

#define N_ANALOGICAS	7
#define LF_R_PORT	GPIOC
#define LF_R_PIN	GPIO_PIN_5
#define LF_R_CH		ADC_CHANNEL_15
#define L_R_PORT	GPIOC
#define L_R_PIN		GPIO_PIN_4
#define L_R_CH		ADC_CHANNEL_14
#define R_R_PORT	GPIOA
#define R_R_PIN		GPIO_PIN_7
#define R_R_CH		ADC_CHANNEL_7
#define RF_R_PORT	GPIOA
#define RF_R_PIN	GPIO_PIN_6
#define RF_R_CH		ADC_CHANNEL_6
#define G_OUTZ_PORT	GPIOB
#define G_OUTZ_PIN	GPIO_PIN_1
#define G_OUTZ_CH	ADC_CHANNEL_9
#define G_VREF_PORT	GPIOB
#define G_VREF_PIN	GPIO_PIN_0
#define G_VREF_CH	ADC_CHANNEL_8
#define VBAT_PORT	GPIOA
#define VBAT_PIN	GPIO_PIN_2
#define VBAT_CH		ADC_CHANNEL_2
#define VBAT_ALERTA 7000

#define A_LF -0.0006952756
#define B_LF 166.4632263002
#define A_RF -0.0006477681
#define B_RF 159.0638322756
#define A_L -0.0007288824
#define B_L 129.8690380511
#define A_R -0.0006497809
#define B_R 139.0614362168

#define THRESHOLD_FRONTAL_L WALL_HAS_FRONT_MM
#define THRESHOLD_FRONTAL_R WALL_HAS_FRONT_MM
#define THRESHOLD_DIAGONAL_L 110
#define THRESHOLD_DIAGONAL_R 110

#define HISTERESE_SENSOR 7




#define ON	1
#define OFF 0


#define NUM_ADC2_INPUT	4


//#define GYRO_INDEX	3
//#define BAT_INDEX	0

/************************* TYPEDEFS, CLASSES AND STRUCTURES ***************************/

/************************************** PROTOTYPES ************************************/

/**
 * @brief
 *
 * @param canal
 * @return
 */
uint32_t getRawADC(uint32_t canal);

/********************************** GLOBAL VARIABLES **********************************/

/********************************** LOCAL  VARIABLES **********************************/

GPIO_TypeDef* EMISSORES_PORT[N_EMISSORES] =
{LF_E_PORT, POWER1_PORT, POWER2_PORT, RF_E_PORT,
		L_LINE_PORT, R_LINE_PORT};
const uint16_t EMISSORES_PIN[N_EMISSORES] =
{LF_E_PIN, POWER1_PIN, POWER2_PIN, RF_E_PIN,
		L_LINE_PIN, R_LINE_PIN};

GPIO_TypeDef* RECEPTORES_PORT[N_RECEPTORES] =
{LINE1_PORT, LINE2_PORT, LINE3_PORT, LINE4_PORT,
		LINE5_PORT, LINE6_PORT, LINE7_PORT, LINE8_PORT};
const uint16_t RECEPTORES_PIN[N_RECEPTORES] =
{LINE1_PIN, LINE2_PIN, LINE3_PIN, LINE4_PIN,
		LINE5_PIN, LINE6_PIN, LINE7_PIN, LINE8_PIN};

GPIO_TypeDef* ANALOGICAS_PORT[N_ANALOGICAS] =
{LF_R_PORT, L_R_PORT, R_R_PORT, RF_R_PORT,
		G_OUTZ_PORT, G_VREF_PORT, VBAT_PORT};
const uint16_t ANALOGICAS_PIN[N_ANALOGICAS] =
{LF_R_PIN, L_R_PIN, R_R_PIN, RF_R_PIN,
		G_OUTZ_PIN, G_VREF_PIN, VBAT_PIN};

ADC_HandleTypeDef hadc1;


ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc2;
TIM_HandleTypeDef htim3;

volatile uint32_t adc2DMABuffer[4];
volatile uint32_t adc1DMABuffer[16];
bool bEndSensorISRFlag = false;

static int32_t numberOfCalibrationCicles = 0;

static uint8_t state = 0;
static uint8_t subState = 0;
static uint8_t adcCnt = 0;

volatile uint32_t mLf = 0;
volatile uint32_t mL = 0;
volatile uint32_t mR = 0;
volatile uint32_t mRf = 0;

static volatile uint32_t lHigh = 0;
static volatile uint32_t rHigh = 0;
static volatile uint32_t rfHigh = 0;
static volatile uint32_t lfHigh = 0;

static volatile uint32_t lLow = 0;
static volatile uint32_t rLow = 0;
static volatile uint32_t rfLow = 0;
static volatile uint32_t lfLow = 0;

volatile float gyroW = 0;

volatile uint8_t calibrationState = 0;
volatile int32_t calibrationCicle = 0;
volatile float gyroOffset = 0;

ADC_HandleTypeDef hadc2;
DMA_HandleTypeDef hdma_adc2;
TIM_HandleTypeDef htim3;

ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

uint32_t adcsamples[4];
volatile uint32_t count = 0;
volatile uint32_t count2 = 0;

//On = 1 index off = 0 index
//Num adc2 input is number of sensors lf rf l and r
volatile uint16_t adc[2][NUM_ADC2_INPUT];
volatile uint16_t sensor[NUM_SENSOR];
volatile uint16_t sensorOld[NUM_SENSOR];
//volatile uint16_t sensorMin[NUM_SENSOR];
//volatile uint16_t sensorMax[NUM_SENSOR];

extern int32_t l;
extern int32_t r;
extern int32_t rf;
extern int32_t lf;
//extern int32_t paredes;

static volatile float m_in[MEAN_WINDOW + 1] = {0};
static volatile float m_out[2] = {0};
/******************************* FUNCTION  IMPLEMENTATION *****************************/


/* ADC2 init function */
void MX_ADC2_Init(void)
{

	ADC_ChannelConfTypeDef sConfig;

	/**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc2.Instance = ADC2;
	hadc2.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV4;
	hadc2.Init.Resolution = ADC_RESOLUTION12b;
	hadc2.Init.ScanConvMode = ENABLE;
	hadc2.Init.ContinuousConvMode = DISABLE;
	hadc2.Init.DiscontinuousConvMode = DISABLE;
	hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
	hadc2.Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T3_TRGO;
	hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc2.Init.NbrOfConversion = 4;
	hadc2.Init.DMAContinuousRequests = ENABLE;
	hadc2.Init.EOCSelection = EOC_SEQ_CONV;
	HAL_ADC_Init(&hadc2);

	/**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_6;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	HAL_ADC_ConfigChannel(&hadc2, &sConfig);

	/**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_7;
	sConfig.Rank = 2;
	HAL_ADC_ConfigChannel(&hadc2, &sConfig);

	/**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_14;
	sConfig.Rank = 3;
	HAL_ADC_ConfigChannel(&hadc2, &sConfig);

	/**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_15;
	sConfig.Rank = 4;
	HAL_ADC_ConfigChannel(&hadc2, &sConfig);
}

/**
 * Enable DMA controller clock
 */
void MX_DMA_Init(void)
{
	/* DMA controller clock enable */
	__DMA2_CLK_ENABLE();

#if DMA_SENSOR
	/* DMA interrupt init */
	HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 3, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);
#endif

	/* DMA interrupt init */ //GYro
	HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);


}

/** Configure pins as
 * Analog
 * Input
 * Output
 * EVENT_OUT
 * EXTI
 */
void MX_GPIO_Init(void)
{

	/* GPIO Ports Clock Enable */
	__GPIOH_CLK_ENABLE();
	__GPIOA_CLK_ENABLE();
	__GPIOC_CLK_ENABLE();

}



/* TIM3 init function */
void MX_TIM3_Init(void)
{

	TIM_ClockConfigTypeDef sClockSourceConfig;
	TIM_MasterConfigTypeDef sMasterConfig;

	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 4-1;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 2100-1;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV4;
	HAL_TIM_Base_Init(&htim3);

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig);

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
	HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig);

}

void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc)
{

	GPIO_InitTypeDef GPIO_InitStruct;
	if(hadc->Instance==ADC2)
	{
		/* USER CODE BEGIN ADC2_MspInit 0 */

		/* USER CODE END ADC2_MspInit 0 */
		/* Peripheral clock enable */
		__ADC2_CLK_ENABLE();

		/**ADC2 GPIO Configuration
    PA6     ------> ADC2_IN6
    PA7     ------> ADC2_IN7
    PC4     ------> ADC2_IN14
    PC5     ------> ADC2_IN15
		 */
		GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
		GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

		GPIO_InitStruct.Pin = GPIO_PIN_4|GPIO_PIN_5;
		GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

		/* Peripheral DMA init*/

		hdma_adc2.Instance = DMA2_Stream2;
		hdma_adc2.Init.Channel = DMA_CHANNEL_1;
		hdma_adc2.Init.Direction = DMA_PERIPH_TO_MEMORY;
		hdma_adc2.Init.PeriphInc = DMA_PINC_DISABLE;
		hdma_adc2.Init.MemInc = DMA_MINC_ENABLE;
		hdma_adc2.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
		hdma_adc2.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
		hdma_adc2.Init.Mode = DMA_CIRCULAR;
		hdma_adc2.Init.Priority = DMA_PRIORITY_HIGH;
		hdma_adc2.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
		hdma_adc2.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
		hdma_adc2.Init.MemBurst = DMA_MBURST_INC4;
		hdma_adc2.Init.PeriphBurst = DMA_MBURST_INC4;
		HAL_DMA_Init(&hdma_adc2);

		__HAL_LINKDMA(hadc,DMA_Handle,hdma_adc2);

		/* USER CODE BEGIN ADC2_MspInit 1 */

		/* USER CODE END ADC2_MspInit 1 */
	}

	if(hadc->Instance==ADC1)
	{
		/* USER CODE BEGIN ADC1_MspInit 0 */

		/* USER CODE END ADC1_MspInit 0 */
		/* Peripheral clock enable */
		__ADC1_CLK_ENABLE();

		/**ADC1 GPIO Configuration
  PB1     ------> ADC1_IN9
		 */
		GPIO_InitStruct.Pin = GPIO_PIN_1;
		GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
		GPIO_InitStruct.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

		/* Peripheral DMA init*/

		hdma_adc1.Instance = DMA2_Stream0;
		hdma_adc1.Init.Channel = DMA_CHANNEL_0;
		hdma_adc1.Init.Direction = DMA_PERIPH_TO_MEMORY;
		hdma_adc1.Init.PeriphInc = DMA_PINC_DISABLE;
		hdma_adc1.Init.MemInc = DMA_MINC_ENABLE;
		hdma_adc1.Init.PeriphDataAlignment = DMA_PDATAALIGN_WORD;
		hdma_adc1.Init.MemDataAlignment = DMA_MDATAALIGN_WORD;
		hdma_adc1.Init.Mode = DMA_CIRCULAR;
		hdma_adc1.Init.Priority = DMA_PRIORITY_HIGH;
		hdma_adc1.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
		hdma_adc1.Init.FIFOThreshold = DMA_FIFO_THRESHOLD_FULL;
		hdma_adc1.Init.MemBurst = DMA_MBURST_INC4;
		hdma_adc1.Init.PeriphBurst = DMA_MBURST_INC4;

		HAL_DMA_Init(&hdma_adc1);

		__HAL_LINKDMA(hadc,DMA_Handle,hdma_adc1);

		/* USER CODE BEGIN ADC1_MspInit 1 */

		/* USER CODE END ADC1_MspInit 1 */
	}

}

/* ADC1 init function */
void MX_ADC1_Init(void)
{

	//ADC Clock vem do APB2 = 82MHz
	//Clock prescaler = 4
	//ADC Clock = APB2/4
	//Tempo para ler 1 amostra de 12bits - 15 ADC Clock

	//As conversoes regulares foram setadas com um tempo de amostragem de 480 Ciclos
	//O ad possui apenas um rank, ou seja apenas uma conversão

	//então como o SampleTIME é bem maior queo o tempo para ler 1 amostra tudo funcionara ok
	//e o SAMPLE_RATE do ad é 84000000/4/480 = 43750 Hz

	//Como eu peço para o DMA 10 amostra de cada vez a frequncia da interrrupcao do dma sera 43750/10 = 4375Hz
	ADC_ChannelConfTypeDef sConfig;

	/**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV6;
	hadc1.Init.Resolution = ADC_RESOLUTION12b;
	hadc1.Init.ScanConvMode = ENABLE;
	hadc1.Init.ContinuousConvMode = ENABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = ENABLE;
	hadc1.Init.EOCSelection = EOC_SINGLE_CONV;
	HAL_ADC_Init(&hadc1);

	/**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_9;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_480CYCLES;
	HAL_ADC_ConfigChannel(&hadc1, &sConfig);

}

void sensorsbsp_init(void)
{
#if DMA_SENSOR
	__TIM3_CLK_ENABLE();
	SENSORES_CLK;

	GPIO_InitTypeDef GPIO_InitStructure;

	// Configura os GPIOs dos emissores como saída push/pull
	for (int i = 0; i < N_EMISSORES; i++)
	{
		GPIO_InitStructure.Pin = EMISSORES_PIN[i];;
		GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStructure.Pull = GPIO_PULLUP;
		GPIO_InitStructure.Speed = GPIO_SPEED_LOW;
		HAL_GPIO_Init(EMISSORES_PORT[i], &GPIO_InitStructure);
		HAL_GPIO_WritePin(EMISSORES_PORT[i], EMISSORES_PIN[i], LOW);
	}

	HAL_GPIO_WritePin(LF_E_PORT, LF_E_PIN, HIGH);
	HAL_GPIO_WritePin(RF_E_PORT, RF_E_PIN, HIGH);
	HAL_GPIO_WritePin(POWER1_PORT, POWER1_PIN, HIGH);

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_ADC1_Init();
	MX_ADC2_Init();
	MX_TIM3_Init();

	HAL_ADC_MspInit(&hadc1);
	HAL_ADC_MspInit(&hadc2);

	HAL_ADC_Start(&hadc1);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc1DMABuffer, 16);

	HAL_ADC_Start(&hadc2);
	HAL_TIM_Base_Start(&htim3);
	HAL_ADC_Start_DMA(&hadc2, (uint32_t *)adc2DMABuffer, 4);
	//	MX_DMA_Init();
	//	HAL_ADC_MspInit(&hadc1);

#else
	GPIO_InitTypeDef GPIO_InitStructure;

	__GPIOA_CLK_ENABLE(); __GPIOB_CLK_ENABLE(); __GPIOC_CLK_ENABLE(); __GPIOD_CLK_ENABLE();

	// Configura os GPIOs dos emissores como saída push/pull
	for (int i = 0; i < N_EMISSORES; i++)
	{
		GPIO_InitStructure.Pin = EMISSORES_PIN[i];;
		GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStructure.Pull = GPIO_PULLUP;
		GPIO_InitStructure.Speed = GPIO_SPEED_LOW;
		HAL_GPIO_Init(EMISSORES_PORT[i], &GPIO_InitStructure);
		HAL_GPIO_WritePin(EMISSORES_PORT[i], EMISSORES_PIN[i], LOW);
	}


	// Configura os GPIOs dos receptores como entrada sem resistor interno
	for (int i = 0; i < N_RECEPTORES; i++)
	{
		GPIO_InitStructure.Pin = RECEPTORES_PIN[i];
		GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
		GPIO_InitStructure.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(RECEPTORES_PORT[i], &GPIO_InitStructure);
	}

#if 1
	sensorsbsp_gyroCalibrate((2734*2)+1000); //1 segundo de calibragem
	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_DMA_Init();
	MX_ADC1_Init();
	//	MX_ADC2_Init();
	//	MX_TIM3_Init();

	HAL_ADC_MspInit(&hadc1);
	//	HAL_ADC_MspInit(&hadc2);

	HAL_ADC_Start(&hadc1);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc1DMABuffer, 16);

	//	HAL_ADC_Start(&hadc2);
	//	HAL_TIM_Base_Start(&htim3);
	//	HAL_ADC_Start_DMA(&hadc2, (uint32_t *)adc2DMABuffer, 4);


	// Configura os pinos analógicos
	for (int i = 0; i < 4; i++)
	{
		GPIO_InitStructure.Pin = ANALOGICAS_PIN[i];
		GPIO_InitStructure.Mode = GPIO_MODE_ANALOG;
		GPIO_InitStructure.Pull = GPIO_NOPULL;
		HAL_GPIO_Init(ANALOGICAS_PORT[i], &GPIO_InitStructure);
	}

	// Configuração do ADC
	__ADC2_CLK_ENABLE();
	hadc2.Instance = ADC2;
	hadc2.Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV4;
	hadc2.Init.Resolution = ADC_RESOLUTION12b;
	hadc2.Init.ScanConvMode = DISABLE;
	hadc2.Init.ContinuousConvMode = DISABLE;
	hadc2.Init.DiscontinuousConvMode = DISABLE;
	hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc2.Init.NbrOfConversion = 1;
	hadc2.Init.DMAContinuousRequests = DISABLE;
	hadc2.Init.EOCSelection = EOC_SINGLE_CONV;
	HAL_ADC_Init(&hadc2);

	HAL_GPIO_WritePin(LF_E_PORT, LF_E_PIN, LOW);
	HAL_GPIO_WritePin(RF_E_PORT, RF_E_PIN, LOW);
	HAL_GPIO_WritePin(POWER1_PORT, POWER1_PIN, LOW);

	//	while(calibrationState != 1); //wait callibration
#endif

#endif

}

/**
 * @brief Realiza a leitura dos sensores de parede
 * 		(atualiza os sensores frontais e laterais)
 * @param lf Valor proporcional a distância do sensor frontal esquerdo
 * @param l Valor proporcional a distância do sensor diagonal esquerdo
 * @param r Valor proporcional a distância do sensor diagonal direito
 * @param rf Valor proporcional a distância do sensor frontal direito
 */
void sensorsbsp_getWallSensors(int32_t* lf, int32_t* l, int32_t* r, int32_t* rf)
{
	int32_t paredes = 0;

#if DMA_SENSOR
	(*lf) = sensor[LF_INDEX];
	(*l) = sensor[L_INDEX];
	(*r) = sensor[R_INDEX];
	(*rf) = sensor[L_INDEX];


	// Realiza a máscara de bits
	if ((*lf) > THRESHOLD || (*rf) > THRESHOLD)
	{
		paredes |= PAREDE_FRONTAL;
	}

	if ((*l) > THRESHOLD)
	{
		paredes |= PAREDE_ESQUERDA;
	}


	if ((*r) > THRESHOLD)
	{
		paredes |= PAREDE_DIREITA;
	}
#else

	(*lf) = getRawADC(LF_R_CH);
	(*l)  = getRawADC(L_R_CH);
	(*r)  = getRawADC(R_R_CH);
	(*rf) = getRawADC(RF_R_CH);


	// Registra o tempo atual
	uint32_t t0 = micros();

	// Sensor frontal esquerdo
	HAL_GPIO_WritePin(LF_E_PORT, LF_E_PIN, HIGH);
	elapse_us(60, t0);
	(*lf) = getRawADC(LF_R_CH) - (*lf);
	HAL_GPIO_WritePin(LF_E_PORT, LF_E_PIN, LOW);
	if ((*lf) < 0)
	{
		(*lf) = 0;
	}
	elapse_us(140, t0);

	// Sensor frontal direito
	HAL_GPIO_WritePin(RF_E_PORT, RF_E_PIN, HIGH);
	elapse_us(200, t0);
	(*rf) = getRawADC(RF_R_CH) - (*rf);
	HAL_GPIO_WritePin(RF_E_PORT, RF_E_PIN, LOW);
	if ((*rf) < 0)
	{
		(*rf) = 0;
	}
	elapse_us(280, t0);

	// Sensores laterais
	HAL_GPIO_WritePin(POWER1_PORT, POWER1_PIN, HIGH);
	elapse_us(340, t0);
	(*l) = getRawADC(L_R_CH) - (*l);
	(*r) = getRawADC(R_R_CH) - (*r);
	HAL_GPIO_WritePin(POWER1_PORT, POWER1_PIN, LOW);
	if ((*l) < 0)
	{
		(*l) = 0;
	}
	if ((*r) < 0)
	{
		(*r) = 0;
	}

	sensor[LF_INDEX]     = (*lf);
	sensor[L_INDEX]      = (*l);
	sensor[R_INDEX]      = (*r);
	sensor[RF_INDEX]     = (*rf);

}

/**
 * @brief Utiliza a função que Realiza a leitura dos sensores de parede
 * 		(atualiza os sensores frontais e laterais) e os converte para milímetros
 * @param lf_mm distância do sensor frontal esquerdo
 * @param l_mm distância do sensor diagonal esquerdo
 * @param r_mm distância do sensor diagonal direito
 * @param rf_mm distância do sensor frontal direito
 * @return paredes: máscara de bits indicando presença (1) ou não (0)
 * 	de paredes. O bit mais significativo representa a parede da esquerda.
 * 	Ex.: 011 = presença de parede frontal e direita.
 *
 */

void regressionFunct(int32_t *sensorIR, double A, double B, float *sensor_in_mm){
	(*sensor_in_mm) = (float)B*expf(((double)(*sensorIR)*A));
}

int32_t sensorsbsp_getWallSensorsInMilimeter(float* lf_mm, float* l_mm, float* r_mm, float* rf_mm){

	int32_t paredes = 0;

	sensorsbsp_getWallSensors(&lf, &l, &r, &rf);

	regressionFunct(&lf, A_LF, B_LF, lf_mm);
	regressionFunct(&rf, A_RF, B_RF, rf_mm);
	regressionFunct(&l, A_L, B_L, l_mm);
	regressionFunct(&r, A_R, B_R, r_mm);

#if 1
	if(sensor[R_INDEX] > HAS_RIGTH_WALL_TRESHOLD)
	{
		paredes |= PAREDE_DIREITA;
		//		ledsbsp_outputLed(LED3, HIGH);
	}
	else if(sensor[R_INDEX] < NO_HAS_RIGTH_WALL_TRESHOLD)
	{
		paredes &= ~PAREDE_DIREITA;
		//		ledsbsp_outputLed(LED3, LOW);
	}

	if(sensor[L_INDEX] > HAS_LEFT_WALL_TRESHOLD)
	{
		//		ledsbsp_outputLed(LED1, HIGH);
		paredes |= PAREDE_ESQUERDA;
	}
	else if(sensor[L_INDEX] < NO_HAS_LEFT_WALL_TRESHOLD)
	{
		//		ledsbsp_outputLed(LED1, LOW);
		paredes &= ~PAREDE_ESQUERDA;
	}

	if(sensor[LF_INDEX] > HAS_LEFTFRONT_WALL_TRESHOLD && sensor[RF_INDEX] > HAS_RIGTHFRONT_WALL_TRESHOLD)
	{
		//		ledsbsp_outputLed(LED2, HIGH);
		paredes |= PAREDE_FRONTAL;
	}
	else if( sensor[LF_INDEX] < NO_HAS_LEFTFRONT_WALL_TRESHOLD && sensor[RF_INDEX] > NO_HAS_RIGTHFRONT_WALL_TRESHOLD)
	{
		//		ledsbsp_outputLed(LED2, LOW);
		paredes &= ~PAREDE_FRONTAL;
	}

	//	paredes & PAREDE_ESQUERDA ? ledsbsp_outputLed(LED1, HIGH) : ledsbsp_outputLed(LED1, LOW);
	//	paredes & PAREDE_FRONTAL ? ledsbsp_outputLed(LED2, HIGH) : ledsbsp_outputLed(LED2, LOW);
	//	paredes & PAREDE_ESQUERDA ? ledsbsp_outputLed(LED3, HIGH) : ledsbsp_outputLed(LED3, LOW);

	// Realiza a máscara de bits com histerese
#else
	if(paredes & PAREDE_FRONTAL){
		if ((*lf_mm) > THRESHOLD_FRONTAL_L+HISTERESE_SENSOR && (*rf_mm) > THRESHOLD_FRONTAL_R+HISTERESE_SENSOR)
		{
			paredes &= ~PAREDE_FRONTAL;
		}
	}
	else{
		if ((*lf_mm) < THRESHOLD_FRONTAL_L-HISTERESE_SENSOR || (*rf_mm) < THRESHOLD_FRONTAL_R-HISTERESE_SENSOR)
		{
			paredes |= PAREDE_FRONTAL;
		}
	}

	if(paredes & PAREDE_ESQUERDA){
		if ((*l_mm) > THRESHOLD_DIAGONAL_L+HISTERESE_SENSOR)
		{
			paredes &= ~PAREDE_ESQUERDA;
		}
	}
	else{
		if ((*l_mm) < THRESHOLD_DIAGONAL_L-HISTERESE_SENSOR)
		{
			paredes |= PAREDE_ESQUERDA;
		}
	}

	if(paredes & PAREDE_DIREITA){
		if ((*r_mm) > THRESHOLD_DIAGONAL_R+HISTERESE_SENSOR)
		{
			paredes &= ~PAREDE_DIREITA;
		}
	}
	else{
		if ((*r_mm) < THRESHOLD_DIAGONAL_R-HISTERESE_SENSOR){
			paredes |= PAREDE_DIREITA;
		}
	}
#endif

#endif
	return paredes;
}


int32_t sensorsbsp_getLineSensors(void)
{
#ifdef LINE_SENSOR_RESOLUTION_HIGH

	int32_t erro = 0, soma = 0, n = 0;

	for(int i = 50; i <= 100; i += 10)
	{
		uint32_t t0 = micros();
		// Habilita os emissores
		HAL_GPIO_WritePin(L_LINE_PORT, L_LINE_PIN, HIGH);
		HAL_GPIO_WritePin(R_LINE_PORT, R_LINE_PIN, HIGH);

		elapse_us(i, t0);


		// Realiza a leitura de todos os sensores de linha, os sensores das
		// extremidades pussuem peso maior, no final é realizada a média ponderada
		if(HAL_GPIO_ReadPin(LINE1_PORT, LINE1_PIN) == LINHA)
		{
			soma += -2000;
			n++;
		}
		if(HAL_GPIO_ReadPin(LINE2_PORT, LINE2_PIN) == LINHA)
		{
			soma += -1333;
			n++;
		}
		if(HAL_GPIO_ReadPin(LINE3_PORT, LINE3_PIN) == LINHA)
		{
			soma += -667;
			n++;
		}
		if(HAL_GPIO_ReadPin(LINE4_PORT, LINE4_PIN) == LINHA)
		{
			soma += -167;
			n++;
		}
		if(HAL_GPIO_ReadPin(LINE5_PORT, LINE5_PIN) == LINHA)
		{
			soma += 167;
			n++;
		}
		if(HAL_GPIO_ReadPin(LINE6_PORT, LINE6_PIN) == LINHA)
		{
			soma += 667;
			n++;
		}
		if(HAL_GPIO_ReadPin(LINE7_PORT, LINE7_PIN) == LINHA)
		{
			soma += 1333;
			n++;
		}
		if(HAL_GPIO_ReadPin(LINE8_PORT, LINE8_PIN) == LINHA)
		{
			soma += 2000;
			n++;
		}

		// Desabilita os emissores
		HAL_GPIO_WritePin(L_LINE_PORT, L_LINE_PIN, LOW);
		HAL_GPIO_WritePin(R_LINE_PORT, R_LINE_PIN, LOW);

		elapse_us(i * 2, t0);


	}


	if(n != 0)
	{
		erro = soma / n;
	}
	else
	{
		erro = INFINITO;
	}
	return erro;

#endif


#ifdef LINE_SENSOR_RESOLUTION_LOW

	int32_t erro = 0, soma = 0, n = 0;
	uint32_t t0 = micros();

	// Habilita os emissores
	HAL_GPIO_WritePin(L_LINE_PORT, L_LINE_PIN, HIGH);
	HAL_GPIO_WritePin(R_LINE_PORT, R_LINE_PIN, HIGH);

	elapse_us(100, t0);

	// Realiza a leitura de todos os sensores de linha, os sensores das
	// extremidades pussuem peso maior, no final é realizada a média ponderada
	if (HAL_GPIO_ReadPin(LINE1_PORT, LINE1_PIN) == LINHA)
	{
		soma += -40;
		n++;
	}
	if (HAL_GPIO_ReadPin(LINE2_PORT, LINE2_PIN) == LINHA)
	{
		soma += -30;
		n++;
	}
	if (HAL_GPIO_ReadPin(LINE3_PORT, LINE3_PIN) == LINHA)
	{
		soma += -20;
		n++;
	}
	if (HAL_GPIO_ReadPin(LINE4_PORT, LINE4_PIN) == LINHA)
	{
		soma += -10;
		n++;
	}
	if (HAL_GPIO_ReadPin(LINE5_PORT, LINE5_PIN) == LINHA)
	{
		soma += 10;
		n++;
	}
	if (HAL_GPIO_ReadPin(LINE6_PORT, LINE6_PIN) == LINHA)
	{
		soma += 20;
		n++;
	}
	if (HAL_GPIO_ReadPin(LINE7_PORT, LINE7_PIN) == LINHA)
	{
		soma += 30;
		n++;
	}
	if (HAL_GPIO_ReadPin(LINE8_PORT, LINE8_PIN) == LINHA)
	{
		soma += 40;
		n++;
	}

	// Desabilita os emissores
	HAL_GPIO_WritePin(L_LINE_PORT, L_LINE_PIN, LOW);
	HAL_GPIO_WritePin(R_LINE_PORT, R_LINE_PIN, LOW);


	// Retorna a média ou retorna a constante INFINITO indicando
	// que nenhum sensor leu linha
	if (n != 0)
	{
		erro = soma / n;
	}
	else
	{
		erro = INFINITO;
	}

	return erro;

#endif

}

void sensorsbsp_gyroCalibrate(uint32_t nbrCicles)
{
	numberOfCalibrationCicles = nbrCicles;
	calibrationState = 0;
}
/**
 * @brief Realiza a conversão de um canal analógico
 * @param canal: Canal analógico a ser realizado a conversão
 * @return rawADC: Resultado da conversão (valor de 12 bits)
 */
uint32_t getRawADC(uint32_t canal)
{
	uint32_t rawADC;
	ADC_ChannelConfTypeDef sConfig;

	sConfig.Channel = canal;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_84CYCLES;
	HAL_ADC_ConfigChannel(&hadc2, &sConfig);
	HAL_ADC_Start(&hadc2);
	HAL_ADC_PollForConversion(&hadc2, 10);
	rawADC = HAL_ADC_GetValue(&hadc2);

	return rawADC;
}

int32_t i = 0;
int32_t adc_temp = 0;
void gyro_IT(void)
{
	//	int32_t adc_temp = 0;

	adc_temp =  adc1DMABuffer[0];
	adc_temp += adc1DMABuffer[1];
	adc_temp += adc1DMABuffer[2];
	adc_temp += adc1DMABuffer[3];
	adc_temp += adc1DMABuffer[4];
	adc_temp += adc1DMABuffer[5];
	adc_temp += adc1DMABuffer[6];
	adc_temp += adc1DMABuffer[7];
	adc_temp += adc1DMABuffer[8];
	adc_temp += adc1DMABuffer[9];
	adc_temp += adc1DMABuffer[10];
	adc_temp += adc1DMABuffer[11];
	adc_temp += adc1DMABuffer[12];
	adc_temp += adc1DMABuffer[13];
	adc_temp += adc1DMABuffer[14];
	adc_temp += adc1DMABuffer[15];


	adc_temp = adc_temp >> 2; //Oversample to increase two bits

	if(calibrationState == 0)
	{
		if(calibrationCicle < numberOfCalibrationCicles)
		{
			calibrationCicle++;
			gyroOffset += (float)adc_temp;
		}
		else
		{
			gyroOffset = gyroOffset/calibrationCicle;
			calibrationState = 1;
		}
	}
	else if(calibrationState == 1)
	{
		//		gyroW = (((float)adc_temp) - gyroOffset)*GYRO_COEFF;

		//  next output value = lp_yv[1];

		for(i = 0; i < MEAN_WINDOW; i++)
		{
			m_in[i] = m_in[i+1];
		}

		m_in[MEAN_WINDOW] = (((float)adc_temp) - gyroOffset)*GYRO_COEFF;

		m_out[0] = m_out[1];

		m_out[1] = m_out[0] + ((m_in[MEAN_WINDOW] - m_in[0])/MEAN_WINDOW);

		gyroW = m_out[1];
	}

	//	adc_temp = adc_temp*0.1235 - 225.4;

	//	for(int i = 0; i < MEAN_WINDOW; i++)
	//	{
	//		m_in[i] = m_in[i+1];
	//	}
	//
	//	m_in[MEAN_WINDOW] = adc_temp;
	//
	//	m_out[0] = m_out[1];
	//
	//	m_out[1] = m_out[0] + ((m_in[MEAN_WINDOW] - m_in[0])/MEAN_WINDOW);
	//
	//	gyroW = m_out[1];

}

void sensors_IT(void)
{
#if DMA_SENSOR
	if(subState == 0)
	{
		subState++;

		switch( state ) {

		case 0:

			// store dc values of IR sensors
			adc[LOW][LF_INDEX] = adc2DMABuffer[LF_INDEX];
			adc[LOW][L_INDEX] = adc2DMABuffer[L_INDEX];
			adc[LOW][R_INDEX] = adc2DMABuffer[R_INDEX];
			adc[LOW][RF_INDEX] = adc2DMABuffer[RF_INDEX];


			HAL_GPIO_WritePin(LF_E_PORT, LF_E_PIN, HIGH);	// turn transmitter ON

			break;

		case 3:

			HAL_GPIO_WritePin(POWER1_PORT, POWER1_PIN, HIGH);

			break;

		case 6:

			HAL_GPIO_WritePin(RF_E_PORT, RF_E_PIN, HIGH);

			break;
		}
	}
	else
	{
		//turn off transmitter
		subState = 0;

		switch( state )
		{

		case 0:

			// store adc values with tx ON
			adc[HIGH][LF_INDEX] = adc2DMABuffer[LF_INDEX];
			HAL_GPIO_WritePin(LF_E_PORT, LF_E_PIN, LOW);		// turn transmitter OFF

			break;

		case 3:

			adc[HIGH][L_INDEX] = adc2DMABuffer[L_INDEX];
			adc[HIGH][R_INDEX] = adc2DMABuffer[R_INDEX];
			HAL_GPIO_WritePin(POWER1_PORT, POWER1_PIN, LOW);
			//			IO_Write(SEN_TX12_H, LOW);

			break;

		case 6:

			adc[HIGH][RF_INDEX] = adc2DMABuffer[RF_INDEX];
			HAL_GPIO_WritePin(RF_E_PORT, RF_E_PIN, LOW);

			bEndSensorISRFlag = true;

			break;
		}

		state++;
	}

	if(bEndSensorISRFlag)
	{
		int i;

		//		__HAL_TIM_DISABLE(htim3);
		//		HAL_TIM_Base_Stop(&htim3);
		bEndSensorISRFlag = false;
		state = 0;

		for (i = 0; i < NUM_ADC2_INPUT; i++) {
			sensorOld[i] = sensor[i];
			// Subtract DC values
			sensor[i] = adc[HIGH][i] > adc[LOW][i] ? (adc[HIGH][i] - adc[LOW][i]) : 0;
		}

	}
#endif

}

/**
 * @brief This function handles DMA2 Stream0 global interrupt. //gyro
 */
void DMA2_Stream0_IRQHandler(void)
{

	/* USER CODE BEGIN DMA2_Stream0_IRQn 0 */

	/* USER CODE END DMA2_Stream0_IRQn 0 */
	HAL_DMA_IRQHandler(&hdma_adc1);
	/* USER CODE BEGIN DMA2_Stream0_IRQn 1 */

	/* USER CODE END DMA2_Stream0_IRQn 1 */

	gyro_IT();

	if(calibrationState == 0)
	{
		if(count2++ >= 250)
		{
			ledsbsp_toogleOutputLed(LED2);
			count2 = 0;
		}
	}
}


void DMA2_Stream2_IRQHandler(void) //IR
{
	/* USER CODE BEGIN DMA2_Stream2_IRQn 0 */

	/* USER CODE END DMA2_Stream2_IRQn 0 */
	HAL_DMA_IRQHandler(&hdma_adc2);
	/* USER CODE BEGIN DMA2_Stream2_IRQn 1 */

	/* USER CODE END DMA2_Stream2_IRQn 1 */

	sensors_IT();

	count++;
	if(count > 10000)
	{
		count = 0;
		ledsbsp_toogleOutputLed(LED3);
	}
}


/*************************************** EOF ******************************************/

