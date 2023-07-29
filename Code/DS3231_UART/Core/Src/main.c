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

#include "string.h"
#include "stdio.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef struct{
	uint8_t sec;
	uint8_t min;
	uint8_t hour;
	uint8_t day;
	uint8_t date;
	uint8_t month;
	uint8_t year;
}_DS3231;

_DS3231 ds3231;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define DS3231_ADDRESS (0x68<<1)

#define CHECKFLAG(FLAG, variable)  (variable & FLAG ? 1 : 0)
#define SETFLAG(FLAG, variable)    (variable |= (FLAG))
#define CLEARFLAG(FLAG, variable)  (variable &= ~(FLAG))

#define FLAG_COMPLETE_UART (1<<0)
#define FLAG_SET_TIME 	   (1<<1)
#define FLAG_GET_TIME	   (1<<2)

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
I2C_HandleTypeDef hi2c1;

UART_HandleTypeDef huart3;

/* USER CODE BEGIN PV */

uint8_t flag = 0;

uint8_t sec = 0, min = 0, hour = 0, day = 0, date = 0, month = 0, year = 0;

char sRx[1];
char logRxData[100] = {0};
char sTx[100] = {0};

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_USART3_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t B2D(uint8_t num){
	return ((num/10)*16+(num%10));
}

uint8_t D2B(uint8_t num){
	return ((num/16)*10+(num%16));
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if(huart->Instance == USART3){
	  HAL_UART_Receive_IT(&huart3, (uint8_t*)sRx, 1);
	  if(!strcmp((char*)sRx,"\n")){
		  SETFLAG(FLAG_COMPLETE_UART,flag);
		  sRx[0] = 0;
	  }else
		  strcat((char *)logRxData,(char *)sRx);
  }
}

void DS3231_SetTime(uint8_t sec, uint8_t min, uint8_t hour, uint8_t day, uint8_t date, uint8_t month, uint8_t year){
	uint8_t rtcData[8] = {0};

	rtcData[0] = 0x00;
	rtcData[1] = D2B(sec);
	rtcData[2] = D2B(min);
	rtcData[3] = D2B(hour);
	rtcData[4] = D2B(day);
	rtcData[5] = D2B(date);
	rtcData[6] = D2B(month);
	rtcData[7] = D2B(year);

	HAL_I2C_Master_Transmit(&hi2c1, DS3231_ADDRESS, rtcData, 8, HAL_MAX_DELAY);
}

void DS3231_GetTime(){
	uint8_t rtcData[7];
	HAL_I2C_Master_Transmit(&hi2c1, DS3231_ADDRESS, 0x00, 1, HAL_MAX_DELAY);
	HAL_I2C_Master_Receive(&hi2c1, DS3231_ADDRESS, rtcData, 7, HAL_MAX_DELAY);

	ds3231.sec   = B2D(rtcData[0]);
	ds3231.min   = B2D(rtcData[1]);
	ds3231.hour  = B2D(rtcData[2]);
	ds3231.day   = B2D(rtcData[3]);
	ds3231.date  = B2D(rtcData[4]);
	ds3231.month = B2D(rtcData[5]);
	ds3231.year  = B2D(rtcData[6]);
}

void XuLyDuLieuTuUart(){
	char dayofweek[10] = {0};

	if(strstr(logRxData, "SetTime")){
		sscanf(logRxData, "SetTime %hhu:%hhu:%hhu %hhu %hhu/%hhu/%hhu", &hour, &min, &sec, &day, &date, &month, &year);

		DS3231_SetTime(sec, min, hour, day, date, month, year);
		memset(logRxData,0,strlen(logRxData));
	}
	if(strstr(logRxData, "GetTime")){
		DS3231_GetTime();
		switch(ds3231.day){
		case 1:
			strcpy(dayofweek, "Sun");
			break;
		case 2:
			strcpy(dayofweek, "Mon");
			break;
		case 3:
			strcpy(dayofweek, "Tue");
			break;
		case 4:
			strcpy(dayofweek, "Wed");
			break;
		case 5:
			strcpy(dayofweek, "Thu");
			break;
		case 6:
			strcpy(dayofweek, "Fri");
			break;
		case 7:
			strcpy(dayofweek, "Sat");
			break;
		}
		sprintf(sTx,"%u:%u:%u %s %u/%u/20%u\n",ds3231.hour, ds3231.min, ds3231.sec, dayofweek, ds3231.date, ds3231.month, ds3231.year);
		HAL_UART_Transmit(&huart3, (uint8_t *)sTx, strlen(sTx), HAL_MAX_DELAY);
		memset(sTx,0,strlen(sTx));
		memset(logRxData,0,strlen(logRxData));
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
  MX_I2C1_Init();
  MX_USART3_UART_Init();
  /* USER CODE BEGIN 2 */

  HAL_UART_Receive_IT(&huart3, (uint8_t*)sRx, 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  if(CHECKFLAG(FLAG_COMPLETE_UART,flag)){

		  XuLyDuLieuTuUart();
		  CLEARFLAG(FLAG_COMPLETE_UART,flag);
	  }
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
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

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
