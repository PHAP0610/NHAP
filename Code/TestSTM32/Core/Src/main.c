/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "Buzzer.h"
#include <string.h>
#include <stdio.h>
#include "7seg.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum _Flag {
	USART3_INT,
	I2C1_INT,
} _Flag;

typedef enum _ecdProcState {
	ECD_WAIT1 = 1,
	ECD_WAIT3,
	ECD_State1,
	ECD_State2,
	ECD_State3,
} _ecdProcState;

typedef enum _btnProcState {
	BTN_WAIT1 = 1,
	BTN_WAIT3,
	BTN_State1,
	BTN_State2,
	BTN_State3,
} _btnProcState;

typedef enum _LedState {
	LED_Start,
	LED_Delay,
} _LedState;

typedef enum _BuzzerState{
	Buzzer_On,
	Buzzer_Off,
}_BuzzerState;

typedef enum _DelayProcess {
	LED_Count_Delay = 1,
	BTN_Count_Delay,
	ECD_Count_Delay,
	BUZZER_Count_Delay,
	LED_Count_Done,
	BTN_Count_Done,
	ECD_Count_Done,
	BUZZER_Count_Done
} _DelayProcess;

typedef struct _DS3231{
	uint8_t sec;
	uint8_t min;
	uint8_t hour;
	uint8_t day;
	uint8_t date;
	uint8_t month;
	uint8_t year;
}_DS3231;

_BuzzerState buzzerState;
_DS3231 ds3231;
_DelayProcess delayProcess;
_LedState ledstate;
_ecdProcState ecdState;
_btnProcState btnState;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define DS3231_ADDRESS (0x68<<1)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
uint32_t flag = 0;

uint32_t delay_counter[2][4] = { 0 };

uint8_t upperdigit = 0;
uint8_t lowerdigit = 0;
int8_t upperNumber[3] = { 0 };
int8_t lowerNumber[4] = { 0 };

uint16_t ecdcount = 0;
uint8_t ecdcheck = 0;

uint8_t push = 0;
uint8_t btncheck = 0;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void FlagSet(uint32_t *f, _Flag bitToSet){
	*f |= (1<<bitToSet);
}

void FlagClear(uint32_t *f, _Flag bitToClear){
	*f &= ~(1<<bitToClear);
}

uint8_t FlagCheck(uint32_t f, _Flag bitToCheck){
	if(f&(1<<bitToCheck))
		return 1;
	else
		return 0;
}

uint8_t B2D (uint8_t num){
	return ((num/10 * 16) + (num%10));
}

uint8_t D2B (uint8_t num){
	return ((num/16 * 10) + (num%16));
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == EnA_Pin) {
		ecdState = ECD_WAIT1;
		HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
	}
	if (GPIO_Pin == EnBtn_Pin) {
		btnState = BTN_WAIT1;
		HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
	}
}

void Delay_Tim(uint32_t delay, _DelayProcess _delayProc) {
	switch (_delayProc) {
	case BUZZER_Count_Delay:
		delay_counter[1][3] = delay;
		if(delay_counter[0][3 < delay])
			delayProcess = BUZZER_Count_Delay;
		break;
	case LED_Count_Delay:
		delay_counter[1][2] = delay;
		if (delay_counter[0][2] < delay)
			delayProcess = LED_Count_Delay;
		break;
	case BTN_Count_Delay:
		delay_counter[1][1] = delay;
		if (delay_counter[0][1] < delay)
			delayProcess = BTN_Count_Delay;
		break;
	case ECD_Count_Delay:
		delay_counter[1][0] = delay;
		if (delay_counter[0][0] < delay)
			delayProcess = ECD_Count_Delay;
		break;
	case LED_Count_Done:
		ledstate = LED_Start;
		break;
	case BTN_Count_Done:
		switch (btnState)
		{
		case BTN_WAIT1:
			btnState = BTN_State1;
			break;
		default:
			btnState = BTN_State3;
			break;
		}
		break;
	case ECD_Count_Done:
		switch (ecdState)
		{
		case ECD_WAIT1:
			ecdState = ECD_State1;
			break;
		default:
			ecdState = ECD_State3;
			break;
		}
		break;
	case BUZZER_Count_Done:
		switch(buzzerState)
		{
		case Buzzer_On:
			buzzerState = Buzzer_Off;
			break;
		default:
			buzzerState = Buzzer_On;
			break;
		}
		break;
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM2) {
		switch (delayProcess) {
		case BUZZER_Count_Delay:
			if (delay_counter[0][3] < delay_counter[1][3])
				delay_counter[0][3]++;
			else{
				delay_counter[0][3]=0;
				delayProcess = BUZZER_Count_Done;
			}
			delayProcess = 0;
			break;
		case LED_Count_Delay:
			if (delay_counter[0][2] < delay_counter[1][2])
				delay_counter[0][2]++;
			else{
				delay_counter[0][2]=0;
				delayProcess = LED_Count_Done;
			}
			delayProcess = 0;
			break;
		case BTN_Count_Delay:
			if (delay_counter[0][1] < delay_counter[1][1])
				delay_counter[0][1]++;
			else{
				delay_counter[0][1]=0;
				delayProcess = BTN_Count_Done;
			}
			delayProcess = 0;
			break;
		case ECD_Count_Delay:
			if (delay_counter[0][0] < delay_counter[1][0])
				delay_counter[0][0]++;
			else{
				delay_counter[0][0]=0;
				delayProcess = ECD_Count_Done;
			}
			delayProcess = 0;
			break;
		case LED_Count_Done:
			break;
		case BUZZER_Count_Done:
			break;
		case ECD_Count_Done:
			break;
		case BTN_Count_Done:
			break;
		}
	}
	if (htim->Instance == TIM3) {
		upperdigit++;
		lowerdigit++;
		if (upperdigit == 3)
			upperdigit = 0;
		if (lowerdigit == 4)
			lowerdigit = 0;
		switch (upperdigit) {
		case 0:
			SegLed_Show(LE3_BIT, upperNumber[0]);
			break;
		case 1:
			SegLed_Show(LE2_BIT, upperNumber[1]);
			break;
		case 2:
			SegLed_Show(LE1_BIT, upperNumber[2]);
			break;
		}
		switch (lowerdigit) {
			case 0:
				SegLed_Show(LE7_BIT,lowerNumber[0]);
				break;
			case 1:
				SegLed_Show(LE6_BIT,lowerNumber[1]);
				break;
			case 2:
				SegLed_Show(LE5_BIT,lowerNumber[2]);
				break;
			case 3:
				SegLed_Show(LE4_BIT,lowerNumber[3]);
		}
	}
}

void DS3232_GetTime(){
	uint8_t rtcdata[7] = {0};

	HAL_I2C_Master_Transmit(&hi2c1, DS3231_ADDRESS, 0x00, 1, HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(&hi2c1, DS3231_ADDRESS, rtcdata, 7, HAL_MAX_DELAY);

	ds3231.sec   = B2D(rtcdata[0]);
	ds3231.min   = B2D(rtcdata[0]);
	ds3231.hour  = B2D(rtcdata[0]);
	ds3231.day   = B2D(rtcdata[0]);
	ds3231.date  = B2D(rtcdata[0]);
	ds3231.month = B2D(rtcdata[0]);
	ds3231.year  = B2D(rtcdata[0]);
}

void DS3231_SetTime(uint8_t sec, uint8_t min, uint8_t hour, uint8_t day, uint8_t date, uint8_t month, uint8_t year){
	uint8_t rtcdata[8] = {0};
	rtcdata[0] = 0x00;
	rtcdata[1] = D2B(sec);
	rtcdata[2] = D2B(min);
	rtcdata[3] = D2B(hour);
	rtcdata[4] = D2B(day);
	rtcdata[5] = D2B(date);
	rtcdata[6] = D2B(month);
	rtcdata[7] = D2B(year);
	HAL_I2C_Master_Transmit(&hi2c1, DS3231_ADDRESS, rtcdata, 8, HAL_MAX_DELAY);
}

void Read_Encoder() {
	switch (ecdState) {
	case ECD_WAIT1:
		Delay_Tim(2, ECD_Count_Delay);
		break;
	case ECD_WAIT3:
		Delay_Tim(2, ECD_Count_Delay);
		break;
	case ECD_State1:
		if (HAL_GPIO_ReadPin(EnB_GPIO_Port, EnB_Pin))
			ecdcount++;
		else
			ecdcount--;
		ecdState = ECD_State2;
		break;
	case ECD_State2:
		while (!HAL_GPIO_ReadPin(EnA_GPIO_Port, EnA_Pin));
		ecdState = ECD_State3;
		break;
	case ECD_State3:
		if (!HAL_GPIO_ReadPin(EnA_GPIO_Port, EnA_Pin))
			ecdcheck = 0;
		else
			ecdcheck++;
		if (ecdcheck == 3) {
			ecdState = 0;
			HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
		}else
			ecdState = ECD_WAIT3;
		break;
	}
}

void Read_Button() {
	switch (btnState) {
	case BTN_WAIT1:
		Delay_Tim(2, BTN_Count_Delay);
		break;
	case BTN_WAIT3:
		Delay_Tim(2,BTN_Count_Delay);
		break;
	case BTN_State1:
		push ^= 1;
		btnState = BTN_State2;
		break;
	case BTN_State2:
		while (!HAL_GPIO_ReadPin(EnBtn_GPIO_Port, EnBtn_Pin));
		btnState = BTN_State3;
		break;
	case BTN_State3:
		if (!HAL_GPIO_ReadPin(EnBtn_GPIO_Port, EnBtn_Pin))
			btncheck = 0;
		else
			btncheck++;
		if (btncheck >= 3){
			HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
			btnState = 0;
		}
		else
			btnState = BTN_WAIT3;
		break;
	}
}

void UpperLed() {
	switch (ledstate) {
	case LED_Start:
		if (!push) {
			upperNumber[0]++;
			for (uint8_t i = 0; i < sizeof(upperNumber) - 1; i++) {
				if (upperNumber[i] == 10) {
					upperNumber[i] = 0;
					upperNumber[i + 1]++;
				}
				if (upperNumber[3] == 10)
					memset(upperNumber, 0, sizeof(upperNumber));
			}
		} else {
			upperNumber[0]--;
			for (uint8_t j = 0; j < sizeof(upperNumber) - 1; j++) {
				if (upperNumber[j] == -1) {
					upperNumber[j] = 9;
					upperNumber[j + 1]--;
				}
				if (upperNumber[3] == -1)
					memset(upperNumber, 9, sizeof(upperNumber));
			}
		}
		ledstate = LED_Delay;
		break;
	case LED_Delay:
		Delay_Tim(10000, LED_Count_Delay);
		break;
	}
}

void LowerLed() {
	lowerNumber[0]++;
	for (uint8_t i = 0; i < sizeof(lowerNumber) - 1; i++) {
		if (lowerNumber[i] == 10) {
			lowerNumber[i] = 0;
			ecdcount = 0;
			lowerNumber[i + 1]++;
		}
		if (lowerNumber[3] == 10)
			memset(lowerNumber, 0, sizeof(lowerNumber));
	}

}

void Buzzer(){
	switch(buzzerState){
	case Buzzer_On:
		HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, 1);
		Delay_Tim(10000, BUZZER_Count_Delay);
		break;
	case Buzzer_Off:
		HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, 0);
		Delay_Tim(10000, BUZZER_Count_Delay);
		break;
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start_IT(&htim2);
  HAL_TIM_Base_Start_IT(&htim3);
//  DS3231_SetTime(0, 14, 20, 3, 25, 7, 23);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		Read_Encoder();
		Read_Button();
		UpperLed();
	    LowerLed();
//		Buzzer();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 100000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_LSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 64-1;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 100-1;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 64-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000-1;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LE4_Pin|LE5_Pin|LE3_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, BUZZER_Pin|HC595_DS_Pin|HC595_OE_Pin|HC595_LATCH_Pin
                          |HC595_SCLK_Pin|LORA_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, TRIAC_Pin|SD_CS_Pin|SD_DET_Pin|LORA_INT_Pin
                          |LORA_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LE4_Pin LE5_Pin LE3_Pin */
  GPIO_InitStruct.Pin = LE4_Pin|LE5_Pin|LE3_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : BUZZER_Pin HC595_DS_Pin HC595_OE_Pin HC595_LATCH_Pin
                           HC595_SCLK_Pin */
  GPIO_InitStruct.Pin = BUZZER_Pin|HC595_DS_Pin|HC595_OE_Pin|HC595_LATCH_Pin
                          |HC595_SCLK_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : ACDET_Pin */
  GPIO_InitStruct.Pin = ACDET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(ACDET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TRIAC_Pin */
  GPIO_InitStruct.Pin = TRIAC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TRIAC_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : SD_CS_Pin SD_DET_Pin LORA_INT_Pin LORA_RST_Pin */
  GPIO_InitStruct.Pin = SD_CS_Pin|SD_DET_Pin|LORA_INT_Pin|LORA_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : EnA_Pin */
  GPIO_InitStruct.Pin = EnA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(EnA_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : EnB_Pin */
  GPIO_InitStruct.Pin = EnB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(EnB_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : EnBtn_Pin */
  GPIO_InitStruct.Pin = EnBtn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(EnBtn_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LORA_CS_Pin */
  GPIO_InitStruct.Pin = LORA_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LORA_CS_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI9_5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);

  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {

	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
