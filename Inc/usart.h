#ifndef USART_H_
#define USART_H_

#include "stm32f4xx_hal.h"
#include <string.h>			//for strcpy
#include <stdlib.h>

void updateRecvGPS(uint8_t recv);
char *getHhmmss();
uint8_t getNRFMode();
uint8_t getNRFThrottle();
int8_t getNRFYaw();
int8_t getNRFPitch();
int8_t getNRFRoll();
char *getNRFArray();
void setPrintInt16(int16_t data,uint8_t ope);
void setPrintFloat(float data,uint8_t ope);
void setPrintChar(char *data,uint8_t ope);
void trancePrintIt();

#endif /* USART_H_ */
