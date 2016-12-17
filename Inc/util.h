/*
 * util.h
 *
 *  Created on: 2016/12/16
 *      Author: thinkpot
 */

#ifndef UTIL_H_
#define UTIL_H_

#include "stm32f4xx_hal.h"
#include "fatfs.h"
#include "spi.h"
#include "usart.h"
#include <stdarg.h>			//for printv
#include <stdio.h>

void utilIncTick();
uint16_t utilGetTick();
void util_Delay(uint16_t Delay);
void printv(const char* format, ...);
void error_mess(char *mess);
void sd_Init();
int8_t sd_Save(char *data);
void sd_Close();
void sd_Sync();
uint8_t sd_open(uint16_t file_num);

#endif /* UTIL_H_ */
