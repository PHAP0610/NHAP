/*
 * 7seg.h
 *
 *  Created on: Jul 5, 2023
 *      Author: KHOA
 */

#ifndef INC_7SEG_H_
#define INC_7SEG_H_

#include "main.h"

#define LE3_BIT 8
#define LE2_BIT 9
#define LE1_BIT 10

void SegLed_TestAllOutput();
void SegLed_OutputDigit(uint16_t testDigit);
void SegLed_0_9(uint16_t x, uint8_t delay);
#endif /* INC_7SEG_H_ */
