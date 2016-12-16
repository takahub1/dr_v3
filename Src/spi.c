/*
 * mpu9250.c
 *
 *  Created on: 2016/12/04
 *      Author: thinkpot
 */

#include "spi.h"
#include <math.h>

SPI_HandleTypeDef hspi2;

uint8_t spi_receive8(uint8_t sendData,uint8_t ss){	//ss=0:mpu,ss=1:bmp
	uint8_t recv=0;
	sendData |= READ_FLAG;
	if(ss==0) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	else HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);

	HAL_SPI_Transmit(&hspi2,&sendData,1,1000);
	while(HAL_SPI_GetState(&hspi2) == HAL_SPI_STATE_BUSY);
	HAL_SPI_Receive(&hspi2,&recv,1,1000);
	while(HAL_SPI_GetState(&hspi2) == HAL_SPI_STATE_BUSY);

	if(ss==0) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
	else HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);

	HAL_Delay(1);
	return recv;
}
uint16_t spi_receive16(uint8_t sendData1,uint8_t sendData2,uint8_t ss){	//ss=0:mpu,ss=1:bmp
	sendData1 |= READ_FLAG;
	sendData2 |= READ_FLAG;
	uint8_t recvData[2] = {0};

	if(ss==0) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	else HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);

	HAL_SPI_Transmit(&hspi2,&sendData1,1,1000);
	while(HAL_SPI_GetState(&hspi2) == HAL_SPI_STATE_BUSY);
	HAL_SPI_Receive(&hspi2,&recvData[0],1,1000);
	while(HAL_SPI_GetState(&hspi2) == HAL_SPI_STATE_BUSY);
	if(ss==0) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
	else HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);

	HAL_Delay(1);

	if(ss==0) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	else HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi2,&sendData2,1,1000);
	while(HAL_SPI_GetState(&hspi2) == HAL_SPI_STATE_BUSY);
	HAL_SPI_Receive(&hspi2,&recvData[1],1,1000);
	while(HAL_SPI_GetState(&hspi2) == HAL_SPI_STATE_BUSY);

	if(ss==0) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
	else HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);

	HAL_Delay(1);

	return recvData[0]<<8 | recvData[1];
}

void spi_send(uint8_t address,uint8_t data,uint8_t ss){
	if(ss==0) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
	else HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_RESET);

	HAL_SPI_Transmit(&hspi2,&address,1,1000);
	HAL_SPI_Transmit(&hspi2,&data,1,1000);

	if(ss==0) HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
	else HAL_GPIO_WritePin(GPIOB, GPIO_PIN_3, GPIO_PIN_SET);
	HAL_Delay(1);
}
void mpuSetup(){
	spi_send(MPUREG_PWR_MGMT_1, 0x80,0);
	HAL_Delay(100);
	spi_send(MPUREG_PWR_MGMT_1, 0x00,0);

	spi_send(MPUREG_GYRO_CONFIG,BITS_FS_500DPS,0);
	spi_send(MPUREG_ACCEL_CONFIG,BITS_FS_2G,0);// +-2G

	spi_send(MPUREG_USER_CTRL, 0x34,0);
	spi_send(MPUREG_SMPLRT_DIV, 0x5,0);
	spi_send(MPUREG_CONFIG, (1 << 6) | (1 << 0),0);// FIFO_mode = 1, Use LPF, Bandwidth_gyro = 184 Hz
}
uint8_t mpu_test(){	//who am i 0x75
	if(spi_receive8(MPUREG_WHOAMI,0) == 113) return 0;
	else return 1;
}

int16_t readAccX(){
	//	return spi_receive8(MPUREG_ACCEL_XOUT_H,0)<<8 | spi_receive8(MPUREG_ACCEL_XOUT_L,0);
	return spi_receive16(MPUREG_ACCEL_XOUT_H,MPUREG_ACCEL_XOUT_L,0);
}
int16_t readAccY(){
	return spi_receive16(MPUREG_ACCEL_YOUT_H,MPUREG_ACCEL_YOUT_L,0);
	//	return spi_receive8(MPUREG_ACCEL_YOUT_H,0)<<8 | spi_receive8(MPUREG_ACCEL_YOUT_L,0);
}
int16_t readAccZ(){
	return spi_receive16(MPUREG_ACCEL_ZOUT_H,MPUREG_ACCEL_ZOUT_L,0);
	//	return spi_receive8(MPUREG_ACCEL_ZOUT_H,0)<<8 | spi_receive8(MPUREG_ACCEL_ZOUT_L,0);
}
int16_t readTemp(){
	return spi_receive16(MPUREG_TEMP_OUT_H,MPUREG_TEMP_OUT_L,0)*10/333.8+210;
	//	return (spi_receive(MPUREG_TEMP_OUT_H,0)<<8 | spi_receive(MPUREG_TEMP_OUT_L,0))*10/3338+210;
}
int16_t readGyrX(){
	return spi_receive16(MPUREG_GYRO_XOUT_H,MPUREG_GYRO_XOUT_L,0);
	//	return spi_receive(MPUREG_GYRO_XOUT_H,0)<<8 | spi_receive(MPUREG_GYRO_XOUT_L,0);
}
int16_t readGyrY(){
	return spi_receive16(MPUREG_GYRO_YOUT_H,MPUREG_GYRO_YOUT_L,0);
	//	return spi_receive(MPUREG_GYRO_YOUT_H,0)<<8 | spi_receive(MPUREG_GYRO_YOUT_L,0);
}
int16_t readGyrZ(){
	return spi_receive16(MPUREG_GYRO_ZOUT_H,MPUREG_GYRO_ZOUT_L,0);
	//	return spi_receive8(MPUREG_GYRO_ZOUT_H,0)<<8 | spi_receive8(MPUREG_GYRO_ZOUT_L,0);
}

uint16_t dig_t1; int16_t dig_t2; int16_t dig_t3;
uint16_t dig_p1; int16_t dig_p2; int16_t dig_p3; int16_t dig_p4; int16_t dig_p5;
int16_t dig_p6; int16_t dig_p7; int16_t dig_p8; int16_t dig_p9;
int64_t var1,var2,t_fine;
void bmpSetup(){
	//	spi_send(0x75, 0x00,1);
	spi_send(0x74, 0x3F,1);
	//	spi_send(0x72, 0x01,1);

	dig_t1=spi_receive16(0x89,0x88,1);
	dig_t2=spi_receive16(0x8B,0x8A,1);
	dig_t3=spi_receive16(0x8D,0x8C,1);

	dig_p1=spi_receive16(0x8F,0x8E,1);
	dig_p2=spi_receive16(0x91,0x90,1);
	dig_p3=spi_receive16(0x93,0x92,1);
	dig_p4=spi_receive16(0x95,0x94,1);
	dig_p5=spi_receive16(0x97,0x96,1);
	dig_p6=spi_receive16(0x99,0x98,1);
	dig_p7=spi_receive16(0x9B,0x9A,1);
	dig_p8=spi_receive16(0x9D,0x9C,1);
	dig_p9=spi_receive16(0x9F,0x9E,1);
//	printv("%d\t",dig_p1);

}
int16_t bmpReadTemp(){
	int32_t raw_temp = spi_receive16(0xFA,0xFB,1)<<4 | spi_receive8(0xFC,1)>>4;
	var1 = ((raw_temp>>3) - (dig_t1<<1)) * (dig_t2>>11);
	var2 = ((((raw_temp>>4) - dig_t1) * ((raw_temp>>4) - dig_t1)) >> 12) * dig_t3 >> 14;
	t_fine = var1 + var2;
	int16_t T = ((t_fine * 5 + 128)>>8)/10;
	return T;
}
int32_t p;
float bmpReadPressure(){

	int32_t raw_pressure = spi_receive16(0xF7,0xF8,1)<<4 | spi_receive8(0xF9,1)>>4;
	var1 = t_fine - 128000;
	var2 = var1 * var1 * dig_p6;
	var2 = var2 + ((var1*dig_p5)<<17);
	var2 = var2 + ((dig_p4)<<35);
	var1 = ((var1 * var1 * dig_p3)>>8) + ((var1 * dig_p2)<<12);
	var1 = ((1<<47)+var1)*(dig_p1)>>33;

	if (var1 == 0) {
		return 0;  // avoid exception caused by division by zero
	}
	p = 1048576 - raw_pressure;
	p = (((p<<31) - var2)*3125) / var1;
	var1 = (dig_p9 * (p>>13) * (p>>13)) >> 25;
	var2 = (dig_p8 * p) >> 19;

	p = ((p + var1 + var2) >> 8) + (dig_p7<<4);
	float ret=(float)p/100;
	return ret;
}
float ati_array[70]={25.01,108.82,193.31,278.50,364.39,451.00,538.35,
		626.44,715.30,804.93,895.35,986.59,1078.64,1171.54,1265.30,
		1359.93,1455.46,1551.91,1649.29,1747.62,1846.93,1947.23,
		2048.56,2150.93,2254.37,2358.90,2464.55,2571.35,2679.33,
		2788.51,2898.92,3010.60,3123.58,3237.89,3353.57,3470.66,
		3589.20,3709.22,3830.76,3953.89,4078.62,4205.03,4333.15,
		4463.05,4594.77,4728.37,4863.93,5001.49,5141.13,5282.92,
		5426.93,5573.25,5721.96,5873.15,6026.91,6183.34,6342.55,
		6504.65,6669.77,6838.02,7009.54,7184.49,7363.02,7545.29,
		7731.49,7921.81,8116.46,8315.66,8519.67,8728.75
};
uint16_t bmpReadAltitude(){
	p/=100;
	uint16_t pp=p/1000;
	uint16_t ati = ati_array[101-pp-1] + (ati_array[101-pp] - ati_array[101-pp-1]) * ((pp+1)*1000 - p) / 1000;
	return ati;
}

