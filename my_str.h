/*
 * my_str.h
 *
 *  Created on: 10 февр. 2022 г.
 *      Author: Tr
 */

#ifndef MY_STR_H_
#define MY_STR_H_

#include <stm32f0xx.h>

uint16_t sizeStr(const uint8_t* str);
uint8_t strCmp(uint8_t* str1, uint8_t* str2);
void strnCpy(uint8_t* source, uint8_t* dst, uint8_t startPos, uint8_t size);
uint8_t Str1inStr2(uint8_t* str1, uint8_t* str2);
uint8_t intToStr(uint16_t num, uint8_t* numStr);
int16_t strToInt(const uint8_t* str, uint8_t size);

void IntToStr(char* convertnum, int num);


#endif /* MY_STR_H_ */
