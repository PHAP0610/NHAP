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

typedef enum _DelayProcess {
	LED_Count_Delay = 1,
	BTN_Count_Delay,
	ECD_Count_Delay,
} _DelayProcess;

_DelayProcess delayProcess;
_LedState ledstate;
_ecdProcState ecdState;
_btnProcState btnState;

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
uint16_t delay_counter[2][3] = { 0 };

uint8_t upperdigit = 0;
uint8_t lowerdigit = 0;
int8_t upperNumber[3] = { 0 };
int8_t lowerNumber[4] = { 0 };
uint32_t flag = 0;

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
/* USER CODE BEGIN PFP */
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//void FlagSet(uint32_t *f, Flag bitToSet){
//	*f |= (1<<bitToSet);
//}
//
//void FlagClear(uint32_t *f, Flag bitToClear){
//	*f &= ~(1<<bitToClear);
//}
//
//uint8_t FlagCheck(uint32_t f, Flag bitToCheck){
//	if(f&(1<<bitToCheck))
//		return 1;
//	else
//		return 0;
//}
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

// void EcdState1() {
// 	if (HAL_GPIO_ReadPin(EnB_GPIO_Port, EnB_Pin))
// 		ecdcount++;
// 	else
// 		ecdcount--;
// 	ecdState = ECD_State2;
// }

// void EcdState2() {
// 	while (!HAL_GPIO_ReadPin(EnA_GPIO_Port, EnA_Pin)) {
// 	};
// 	ecdState = ECD_State3;
// }

// void EcdState3() {
// 	if (!HAL_GPIO_ReadPin(EnA_GPIO_Port, EnA_Pin))
// 		ecdcheck = 0;
// 	else
// 		ecdcheck++;
// 	if (ecdcheck == 3)
// 		ecdState = 0;
// 	else
// 		ecdState = ECD_WAIT3;
// 	HAL_NVIC_EnableIRQ(EXTI9_5_IRQn);
// }

// void BtnState1() {
// 	push = !push;
// 	btnState = BTN_State2;
// }

// void BtnState2() {
// 	while (!HAL_GPIO_ReadPin(EnBtn_GPIO_Port, EnBtn_Pin)) {
// 	};
// 	btnState = BTN_State3;
// }

// void BtnState3() {
// 	if (!HAL_GPIO_ReadPin(EnBtn_GPIO_Port, EnBtn_Pin))
// 		btncheck = 0;
// 	else
// 		btncheck++;
// 	if (ecdcheck == 3)
// 		btnState = 0;
// 	else
// 		btnState = BTN_WAIT3;
// 	HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
// }

void Delay_Tim(uint16_t delay, _DelayProcess _delayProc) {
	switch (_delayProc) {
	case LED_Count_Delay:
		delay_counter[1][0] = delay;
		if (delay_counter[0][0] < delay)
			delayProcess = LED_Count_Delay;
		else
			ledstate = LED_Start;
		break;
	case BTN_Count_Delay:
		delay_counter[1][1] = delay;
		if (delay_counter[0][1] < delay)
			delayProcess = BTN_Count_Delay;
		else
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
	case ECD_Count_Delay:
		delay_counter[1][2] = delay;
		if (delay_counter[0][2] < delay)
			delayProcess = ECD_Count_Delay;
		else
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
	}
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM2) {
		switch (delayProcess) {
		case LED_Count_Delay:
			if (delay_counter[0][0] < delay_counter[1][0]) 
				delay_counter[0][0]++;
			else
				delay_counter[0][0] = 0;
			delayProcess = 0;
			break;
		case BTN_Count_Delay:
			if (delay_counter[0][1] < delay_counter[1][1]) 
				delay_counter[0][1]++;
			else
				delay_counter[0][1] = 0;
			delayProcess = 0;
			break;
		case ECD_Count_Delay:
			if (delay_counter[0][2] < delay_counter[1][2])
				delay_counter[0][2]++;
			else
				delay_counter[0][2] = 0;
			delayProcess = 0;
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
//		switch (lowerdigit) {
//			case 0:
//				SegLed_Show(LE7_BIT,lowerNumber[0]);
//				break;
//			case 1:
//				SegLed_Show(LE6_BIT,lowerNumber[1]);
//				break;
//			case 2:
//				SegLed_Show(LE5_BIT,lowerNumber[2]);
//				break;
//			case 3:
//				SegLed_Show(LE4_BIT,lowerNumber[3]);
//		}
	}
}

void Read_Encoder() {
	switch (ecdState) {
	case ECD_WAIT1:
		Delay_Tim(20, ECD_Count_Delay);
		break;
	case ECD_WAIT3:
		Delay_Tim(20, ECD_Count_Delay);
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
		Delay_Tim(20, BTN_Count_Delay);
		break;
	case BTN_WAIT3:
		Delay_Tim(20,BTN_Count_Delay);
		break;
	case BTN_State1:
		push = !push;
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
		if (ecdcheck == 3){
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
			lowerNumber[i + 1]++;
		}
		if (lowerNumber[3] == 10)
			memset(lowerNumber, 0, sizeof(lowerNumber));
	}

}

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {
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

	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start_IT(&htim3);

	/* USER CODE END 2 */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1) {
		Read_Encoder();
		Read_Button();
		UpperLed();
//	  LowerLed();
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */
	}
	/* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
	RCC_OscInitTypeDef RCC_OscInitStruct = { 0 };
	RCC_ClkInitTypeDef RCC_ClkInitStruct = { 0 };

	/** Initializes the RCC Oscillators according to the specified parameters
	 * in the RCC_OscInitTypeDef structure.
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI_DIV2;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL16;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		Error_Handler();
	}

	/** Initializes the CPU, AHB and APB buses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
		Error_Handler();
	}
}

/**
 * @brief TIM2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_TIM2_Init(void) {

	/* USER CODE BEGIN TIM2_Init 0 */

	/* USER CODE END TIM2_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM2_Init 1 */

	/* USER CODE END TIM2_Init 1 */
	htim2.Instance = TIM2;
	htim2.Init.Prescaler = 64 - 1;
	htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim2.Init.Period = 10 - 1;
	htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim2) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig)
			!= HAL_OK) {
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
static void MX_TIM3_Init(void) {

	/* USER CODE BEGIN TIM3_Init 0 */

	/* USER CODE END TIM3_Init 0 */

	TIM_ClockConfigTypeDef sClockSourceConfig = { 0 };
	TIM_MasterConfigTypeDef sMasterConfig = { 0 };

	/* USER CODE BEGIN TIM3_Init 1 */

	/* USER CODE END TIM3_Init 1 */
	htim3.Instance = TIM3;
	htim3.Init.Prescaler = 8000 - 1;
	htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim3.Init.Period = 10;
	htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim3) != HAL_OK) {
		Error_Handler();
	}
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK) {
		Error_Handler();
	}
	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig)
			!= HAL_OK) {
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
static void MX_USART3_UART_Init(void) {

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
	if (HAL_UART_Init(&huart3) != HAL_OK) {
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
static void MX_GPIO_Init(void) {
	GPIO_InitTypeDef GPIO_InitStruct = { 0 };
	/* USER CODE BEGIN MX_GPIO_Init_1 */
	/* USER CODE END MX_GPIO_Init_1 */

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOC, LE4_Pin | LE5_Pin | LE3_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOA,
	BUZZER_Pin | HC595_DS_Pin | HC595_OE_Pin | HC595_LATCH_Pin | HC595_SCLK_Pin,
			GPIO_PIN_RESET);

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(TRIAC_GPIO_Port, TRIAC_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pins : LE4_Pin LE5_Pin LE3_Pin */
	GPIO_InitStruct.Pin = LE4_Pin | LE5_Pin | LE3_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

	/*Configure GPIO pins : BUZZER_Pin HC595_DS_Pin HC595_OE_Pin HC595_LATCH_Pin
	 HC595_SCLK_Pin */
	GPIO_InitStruct.Pin = BUZZER_Pin | HC595_DS_Pin | HC595_OE_Pin
			| HC595_LATCH_Pin | HC595_SCLK_Pin;
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
	GPIO_InitStruct.Pin = EnA_Pin | EnBtn_Pin;
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
void Error_Handler(void) {
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
