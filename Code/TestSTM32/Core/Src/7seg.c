/*
 * 7seg.c
 *
 *  Created on: Jul 5, 2023
 *      Author: KHOA
 */


#include "7seg.h"
const uint16_t code7seg[] = {0x00D7, 0x0014, 0x00CD, 0x005D, 0x001E, 0x005B, 0x00DB, 0x0015, 0x00DF, 0x005F};
void SegLed_TestAllOutput()
{
	HAL_GPIO_WritePin(HC595_DS_GPIO_Port, HC595_DS_Pin, 1);
	for(uint8_t i = 0; i < 16;i++){
		HAL_GPIO_WritePin(HC595_SCLK_GPIO_Port, HC595_SCLK_Pin, 1);
		HAL_GPIO_WritePin(HC595_SCLK_GPIO_Port, HC595_SCLK_Pin, 0);
	}
	HAL_GPIO_WritePin(HC595_LATCH_GPIO_Port, HC595_LATCH_Pin, 1);
	HAL_GPIO_WritePin(HC595_LATCH_GPIO_Port, HC595_LATCH_Pin, 0);
}

void SegLed_OutputDigit(uint16_t testDigit)
{
	for(uint8_t i = 0; i < 16;i++){
		if((testDigit&0x01) == 1) HAL_GPIO_WritePin(HC595_DS_GPIO_Port, HC595_DS_Pin, 1);
		else HAL_GPIO_WritePin(HC595_DS_GPIO_Port, HC595_DS_Pin, 0);
		HAL_GPIO_WritePin(HC595_SCLK_GPIO_Port, HC595_SCLK_Pin, 1);
		HAL_GPIO_WritePin(HC595_SCLK_GPIO_Port, HC595_SCLK_Pin, 0);
		testDigit = (testDigit >> 1);
	}
	HAL_GPIO_WritePin(HC595_LATCH_GPIO_Port, HC595_LATCH_Pin, 1);
	HAL_GPIO_WritePin(HC595_LATCH_GPIO_Port, HC595_LATCH_Pin, 0);
}

void SegLed_Show(uint16_t digit, uint8_t number)
{
	SegLed_OutputDigit(code7seg[number]|digit);
}


