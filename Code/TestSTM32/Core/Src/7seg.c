/*
 * 7seg.c
 *
 *  Created on: Jul 5, 2023
 *      Author: KHOA
 */


#include "7seg.h"
const char code7seg[] = {0xD7, 0x14, 0xCD, 0x5D, 0x1E, 0x5B, 0xDB, 0x15, 0xDF, 0x5F};

void SegLed_TestAllOutput()
{
	HAL_GPIO_WritePin(HC595_LATCH_GPIO_Port, HC595_LATCH_Pin, 0);
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

void SegLed_0_9(uint16_t x, uint8_t delay)
{
	// const uint16_t code7seg[] = {0x01F7, 0x0134, 0x01ED, 0x017D, 0x013E, 0x017B, 0x01FB, 0x0135, 0x01FF, 0x017F};
	SegLed_OutputDigit(0x00F7|x);
	HAL_Delay(delay*1000);
	SegLed_OutputDigit(0x0034|x);
	HAL_Delay(delay*1000);
	SegLed_OutputDigit(0x00ED|x);
	HAL_Delay(delay*1000);
	SegLed_OutputDigit(0x007D|x);
	HAL_Delay(delay*1000);
	SegLed_OutputDigit(0x003E|x);
	HAL_Delay(delay*1000);
	SegLed_OutputDigit(0x007B|x);
	HAL_Delay(delay*1000);
	SegLed_OutputDigit(0x00FB|x);
	HAL_Delay(delay*1000);
	SegLed_OutputDigit(0x0035|x);
	HAL_Delay(delay*1000);
	SegLed_OutputDigit(0x00FF|x);
	HAL_Delay(delay*1000);
	SegLed_OutputDigit(0x007F|x);
	HAL_Delay(delay*1000);

}


