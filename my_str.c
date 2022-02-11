/*
 * my_str.c
 *
 *  Created on: 10 февр. 2022 г.
 *      Author: Tr
 */

#include "my_str.h"

uint16_t sizeStr(const uint8_t* str) {
	uint16_t size = 0;
	while(str[size++] != '\0');
	return size-1;
}

uint8_t strCmp(uint8_t* str1, uint8_t* str2) {
    int i = 0;
    while(str1[i] == str2[i]) {
        if (str1[i] == 0x00 && str2[i] == 0x00) {
            return 0;
        }
        ++i;
    }

    if (str1[i] == 0x00)
        return 2;
    return 1;
}

void strnCpy(uint8_t* source, uint8_t* dst, uint8_t startPos, uint8_t size) {
	for (int i = 0; i < size; i++) {
		dst[i] = source[startPos + i];
	}
}

uint8_t Str1inStr2(uint8_t* str1, uint8_t* str2) {
    int i = 0;
    while(str1[i] == str2[i]) {
        if (str1[i] == 0x00) {
            return 0;
        }
        if (str2[i] == 0) {
        	return 2;
        }
        ++i;
    }

    if (str1[i] == 0x00)
        return 0;
    return 1;
}


uint8_t intToStr(uint16_t num, uint8_t* numStr) {
	numStr[0] = num / 10000 + 0x30;
	num -= (num/10000) * 10000;

	numStr[1] = num/1000 + 0x30;
	num -= (num/1000) * 1000;

	numStr[2] = num/100 + 0x30;
	num -= (num/100) * 100;

	numStr[3] = num/10 + 0x30;
	num -= (num/10) * 10;

	numStr[4] = num + 0x30;
	return 0;
}

uint16_t strToInt(const uint8_t* str, uint8_t size) {
	uint16_t num = 0;

	for (int i = size-1; i >= 0; i--) {
		num += (str[i] - 0x30) * pow(10, size-1 - i);
	}

	return num;
}



