/*
 * 7seg.h
 *
 *  Created on: Jul 5, 2023
 *      Author: KHOA
 */

#ifndef INC_7SEG_H_
#define INC_7SEG_H_

#include "main.h"

#define LE7_BIT (1<<14)
#define LE6_BIT (1<<13)
#define LE5_BIT (1<<12)
#define LE4_BIT (1<<11)
#define LE3_BIT (1<<10)
#define LE2_BIT (1<<9)
#define LE1_BIT (1<<8)

void SegLed_TestAllOutput();
void SegLed_OutputDigit(uint16_t testDigit);
void SegLed_Show(uint16_t digit, uint8_t number);
#endif /* INC_7SEG_H_ */
