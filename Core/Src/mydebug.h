/*
 * mydebug.h
 *
 *  Created on: Jul 14, 2023
 *      Author: Administrator
 */

#ifndef SRC_MYDEBUG_H_
#define SRC_MYDEBUG_H_
#include "stm32f4xx_hal.h"
#include <string.h>
#include <stdarg.h>


#define MAX_INPUT_LENGTH 100 // adjust this as necessary
extern UART_HandleTypeDef huart2;


void printDebugSimple(char* inputString);


void printDebug(const char* format, ...);

void printDebugHexString(const char* format, uint8_t *message, uint8_t numBytes);

void printDebugCharArray(const char* format, char *message, uint8_t numChars);

#endif /* SRC_MYDEBUG_H_ */
