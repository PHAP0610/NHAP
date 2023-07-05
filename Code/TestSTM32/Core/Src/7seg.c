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
