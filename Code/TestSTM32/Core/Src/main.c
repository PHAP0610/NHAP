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

typedef enum Flag{
	FLAG_TIMER2_INT,
	FLAG_TIMER3_INT,
	FLAG_UART2_INT,
}Flag;

typedef enum ecdProcState{
	ECD_WAIT1 = 1,
	ECD_WAIT3,
	ECD_State1,
	ECD_State2,
	ECD_State3,
}ecdProcState;

typedef enum btnProcState{
	BTN_WAIT1 = 1,
	BTN_WAIT3,
	BTN_State1,
	BTN_State2,
	BTN_State3,
}btnProcState;

ecdProcState ecdState;
btnProcState btnState;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */
uint8_t delay_counter = 0;

uint8_t digitCount = 0;
int8_t digitNumber[4] = {0};
uint32_t flag = 0;

int8_t ecdcount = 0;
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
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void FlagSet(uint32_t *f, Flag bitToSet){
	*f |= (1<<bitToSet);
}

void FlagClear(uint32_t *f, Flag bitToClear){
	*f &= ~(1<<bitToClear);
}

uint8_t FlagCheck(uint32_t f, Flag bitToCheck){
	if(f&(1<<bitToCheck))
		return 1;
	else
		return 0;
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin){
	if(GPIO_Pin == EnA_Pin){
		ecdState = ECD_WAIT1;
		HAL_NVIC_DisableIRQ(EXTI9_5_IRQn);
	}
	if(GPIO_Pin == EnBtn_Pin){
		btnState = BTN_WAIT1;
		HAL_NVIC_DisableIRQ(EXTI15_10_IRQn);
	}
}

void EcdState1(){
	if(HAL_GPIO_ReadPin(EnB_GPIO_Port, EnB_Pin))
		ecdcount++;
	else
		ecdcount--;
	ecdState = ECD_State2;
}

void EcdState2(){
	while(!HAL_GPIO_ReadPin(EnA_GPIO_Port, EnA_Pin)){};
	ecdState = ECD_State3;
}

void EcdState3(){
	if(!HAL_GPIO_ReadPin(EnA_GPIO_Port, EnA_Pin))
		ecdcheck = 0;
	else
		ecdcheck++;
	if(ecdcheck == 3)
		ecdState = 0;
	else
		ecdState = ECD_WAIT3;
	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
}

void BtnState1(){
	push = !push;
	btnState = BTN_State2;
}

void BtnState2(){
	while(!HAL_GPIO_ReadPin(EnBtn_GPIO_Port, EnBtn_Pin)){};
	btnState = BTN_State3;
}

void BtnState3(){
	if(!HAL_GPIO_ReadPin(EnBtn_GPIO_Port, EnBtn_Pin))
		btncheck = 0;
	else
		btncheck++;
	if(ecdcheck == 3)
		btnState = 0;
	else
		btnState = BTN_WAIT3;
	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim){
	if(htim->Instance == TIM2){
		delay_counter++;
	}
	if(htim->Instance == TIM3){
		FlagSet(&flag, FLAG_TIMER3_INT);
		digitCount++;
		if(digitCount == 4) digitCount = 0;
	}
}


int DelayTim(uint8_t delay){
	HAL_TIM_Base_Start_IT(&htim2);
	if(delay_counter >= delay){
		delay_counter = 0;
		HAL_TIM_Base_Stop_IT(&htim2);
		return 1;
	}else return 0;
}

void Read_Encoder(){
	switch(ecdState){
	  	case ECD_WAIT1:
	  		if(DelayTim(20))
	  			ecdState = ECD_State1;
	  		break;
	  	case ECD_WAIT3:
	  		if(DelayTim(10))
	  			ecdState = ECD_State3;
	  		break;
	  	case ECD_State1:
	  		EcdState1();
	  		break;
	  	case ECD_State2:
	  		EcdState2();
	  		break;
	  	case ECD_State3:
	  		EcdState3();
	  		break;
  	}
}

void Read_Button(){
	switch(btnState){
		case BTN_WAIT1:
			if(DelayTim(20))
				btnState = BTN_State1;
			break;
		case BTN_WAIT3:
			if(DelayTim(10))
				btnState = BTN_State3;
			break;
		case BTN_State1:
			BtnState1();
			break;
		case BTN_State2:
			BtnState2();
			break;
		case BTN_State3:
			BtnState3();
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
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start_IT(&htim3);

  __HAL_DBGMCU_FREEZE_TIM2();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(FlagCheck(flag, FLAG_TIMER3_INT)){
		  switch (digitCount) {
			case 0:
				SegLed_Show(LE7_BIT,digitNumber[0]);
			break;
			case 1:
				SegLed_Show(LE6_BIT,digitNumber[1]);

			break;
			case 2:
				SegLed_Show(LE5_BIT,digitNumber[2]);

			break;
			case 3:
				SegLed_Show(LE4_BIT,digitNumber[3]);
			break;
		}
		  digitNumber[0]++;
		  for(uint8_t i=0;i<sizeof(digitNumber)-1;i++){
			  if(digitNumber[i]==10){
				  digitNumber[i]=0;
				  digitNumber[i+1]++;
			  }
		  }
		  if(digitNumber[3]==10){
			  memset(digitNumber,0,sizeof(digitNumber));
		  }
		  FlagClear(&flag, FLAG_TIMER3_INT);
	  }

	  Read_Encoder();
	  Read_Button();

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
  htim2.Init.Period = 10-1;
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
  htim3.Init.Prescaler = 8000-1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 10;
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
                          |HC595_SCLK_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TRIAC_GPIO_Port, TRIAC_Pin, GPIO_PIN_RESET);

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

  /*Configure GPIO pins : EnA_Pin EnBtn_Pin */
  GPIO_InitStruct.Pin = EnA_Pin|EnBtn_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : EnB_Pin */
  GPIO_InitStruct.Pin = EnB_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(EnB_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
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
  while (1)
  {

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
