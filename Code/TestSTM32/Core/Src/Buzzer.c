/*
 * Buzzer.c
 *
 *  Created on: Jul 5, 2023
 *      Author: VTP
 */

#include "Buzzer.h"
#include "main.h"

void BuzzerToggle()
{
	HAL_GPIO_TogglePin(BUZZER_GPIO_Port, BUZZER_Pin);
    HAL_Delay(1000);
}

void BuzzerOn(int On){
	HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, 1);
	HAL_Delay(On);
}

void BuzzerOff(int Off){
	HAL_GPIO_WritePin(BUZZER_GPIO_Port, BUZZER_Pin, 0);
	HAL_Delay(Off);
}

void BuzzerOnOff(int delayOn, int delayOff){
	BuzzerOn(delayOn);
	BuzzerOff(delayOff);
}
