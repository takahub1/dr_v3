#ifndef USART_H_
#define USART_H_

#include "stm32f4xx_hal.h"
#include <string.h>			//for strcpy
#include <stdlib.h>
#include "util.h"
#include "fatfs.h"

void updateRecvGPS(uint8_t recv);
char *getHhmmss();
char *getDdmmyy();
char *getNorth();
char *getEast();
uint8_t getNRFMode();
uint8_t getNRFThrottle();
int8_t getNRFYaw();
int8_t getNRFPitch();
int8_t getNRFRoll();
char *getNRFArray();
void setPrintInt16(int16_t data,uint8_t ope);
void setPrintFloat(float data,uint8_t ope);
void setPrintChar(char *data,uint8_t ope);
void trancePrintIt(uint8_t sd);

#endif /* USART_H_ */
