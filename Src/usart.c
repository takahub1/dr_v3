#include "usart.h"

#define pLen 150
#define pDev 50

UART_HandleTypeDef huart1;
char hhmmss[10];
char ddmmyy[10];
char north[14];
char east[14];

typedef struct {
	uint8_t mode;
	char tmparray[6];
	char testarray[6];
	uint8_t th;
	int8_t yaw;
	int8_t pitch;
	int8_t roll;
	uint8_t cnt;
	uint8_t tmpcnt;
	uint8_t reset;
} NRF_t;

NRF_t nrf;

uint8_t NRF_mode=0;

char RecvGPS=0;
char RecvNRF=0;

char printData[pLen]={0};


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart){
	uint8_t a=0;
	for(a=0;a<pLen;a++) printData[a]=0;
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	/* Set transmission flag: transfer complete */
	static uint8_t i=0;
	static uint8_t string_num_tmp=0;
	static uint8_t GPRMC_flag=0;
	static uint8_t dmy_flag=0;
	static char hms_tmp[11],dmy_tmp[10],north_tmp[14],east_tmp[14];
	static uint8_t hour=0;
	static uint8_t end_flag=0;

	if(huart->Instance == USART2){//Not bound progress :
		if(RecvNRF == 'N') nrf.mode=0;
		else if(RecvNRF == 'B') nrf.mode=1;
		else if(RecvNRF == ':'){
			nrf.mode=2;
			nrf.cnt=0;
			nrf.reset=0;
		}
		else if(nrf.mode==2) nrf.mode=3;
		else if(nrf.mode == 3){
			if(nrf.reset==0){
				for(nrf.tmpcnt=0;nrf.tmpcnt<4;nrf.tmpcnt++) nrf.tmparray[nrf.tmpcnt]=0;
				nrf.cnt=0;
				nrf.reset=4;
			}
			if(RecvNRF == '\t') nrf.mode=4;
			if(nrf.cnt<4) nrf.tmparray[nrf.cnt++]=RecvNRF;
		}
		else if(nrf.mode == 4){
			if(nrf.reset==4){
				nrf.th=atoi(nrf.tmparray);
				for(nrf.tmpcnt=0;nrf.tmpcnt<4;nrf.tmpcnt++) nrf.tmparray[nrf.tmpcnt]=0;
				nrf.cnt=0;
				nrf.reset=5;
			}
			if(RecvNRF == '\t') nrf.mode=5;
			if(nrf.cnt<4 ) nrf.tmparray[nrf.cnt++]=RecvNRF;
		}
		else if(nrf.mode == 5){
			if(nrf.reset==5){
				nrf.yaw=atoi(nrf.tmparray);
				for(nrf.tmpcnt=0;nrf.tmpcnt<4;nrf.tmpcnt++) nrf.tmparray[nrf.tmpcnt]=0;
				nrf.cnt=0;
				nrf.reset=6;
			}
			if(RecvNRF == '\t') nrf.mode=6;
			if(nrf.cnt<4 ) nrf.tmparray[nrf.cnt++]=RecvNRF;
		}
		else if(nrf.mode == 6){
			if(nrf.reset==6){
				nrf.pitch=atoi(nrf.tmparray);
				for(nrf.tmpcnt=0;nrf.tmpcnt<4;nrf.tmpcnt++) nrf.tmparray[nrf.tmpcnt]=0;
				nrf.cnt=0;
				nrf.reset=7;
			}
			if(RecvNRF == '\t') nrf.mode=7;
			if(nrf.cnt<4 ) nrf.tmparray[nrf.cnt++]=RecvNRF;
		}
		else if(nrf.mode == 7){
			if(nrf.reset==7){
				nrf.roll=atoi(nrf.tmparray);
				//				for(nrf.tmpcnt=0;nrf.tmpcnt<4;nrf.tmpcnt++) nrf.tmparray[nrf.tmpcnt]=0;
				//				nrf.cnt=0;
				//				nrf.reset=8;
			}
			if(RecvNRF == '\t') nrf.mode=8;
			//			if(nrf.cnt<4 ) nrf.tmparray[nrf.cnt++]=RecvNRF;
		}
	}

	if(huart->Instance == USART3){
		i=RecvGPS;
		//		static uint8_t test[1]={0};
		//		test[0]=i;
		//		HAL_UART_Transmit_IT(&huart1,(uint8_t *)test,1);
		if(i == 'G') GPRMC_flag=1;
		else if((i == 'N' || i == 'P') && GPRMC_flag == 1) GPRMC_flag = 2;
		else if(i == 'R' && GPRMC_flag == 2) GPRMC_flag = 3;
		else if(i == ',' && GPRMC_flag == 3) GPRMC_flag = 4;
		else if(GPRMC_flag == 4){
			if(i == ',') GPRMC_flag = 5;
			else hms_tmp[string_num_tmp++] = i;
		}
		else if(GPRMC_flag == 5){
			hour = (hms_tmp[0]-'0')*10 + hms_tmp[1]-'0'; //standard time + 09:00
			hour+=9;
			if(hour >= 24)  hour-=24;
			hms_tmp[0] = hour/10 + '0';
			hms_tmp[1] = hour%10 + '0';

			hms_tmp[10] = '\0';
			strcpy(hhmmss,hms_tmp);
			GPRMC_flag=6;
			string_num_tmp=0;
		}
		else if(GPRMC_flag == 6 && dmy_flag < 7){
			if(i == ',') dmy_flag++;
			if(dmy_flag == 7) GPRMC_flag = 7;

			if(dmy_flag == 1) north_tmp[string_num_tmp++] = i;
			else if(dmy_flag == 2) end_flag=2;
			else if(dmy_flag == 3) east_tmp[string_num_tmp++] = i;
			else if(dmy_flag == 4) end_flag=3;

			if(end_flag == 2){
				north_tmp[11]='\0';
				uint8_t hoge=0;
				for(;north_tmp[hoge]!='\0';hoge++)
					north[hoge] = north_tmp[hoge+1];
				string_num_tmp = 0;
				end_flag=0;
			}
			else if(end_flag == 3){
				east_tmp[11]='\0';
				uint8_t hoge=0;
				for(;east_tmp[hoge]!='\0';hoge++)
					east[hoge] = east_tmp[hoge+1];
				string_num_tmp = 0;
				end_flag=0;
			}
		}
		else if(GPRMC_flag == 7){
			if(i == ',') GPRMC_flag = 8;
			else dmy_tmp[string_num_tmp++] = i;
			dmy_flag=0;
		}
		else if(GPRMC_flag == 8){
			if(hour < 9){		//to inrernational date line
				uint8_t day = (dmy_tmp[0]-'0')*10 + dmy_tmp[1]-'0';
				day++;
				dmy_tmp[0] = day/10 + '0';
				dmy_tmp[1] = day%10 + '0';
			}
			static uint8_t to_ymd[2];	//ddmmyy to yymmdd
			to_ymd[0] = dmy_tmp[0];
			to_ymd[1] = dmy_tmp[1];
			dmy_tmp[0] = dmy_tmp[4];
			dmy_tmp[1] = dmy_tmp[5];
			dmy_tmp[4] = to_ymd[0];
			dmy_tmp[5] = to_ymd[1];

			strcpy(ddmmyy,dmy_tmp);
			GPRMC_flag=9;
			string_num_tmp=0;
		}
	}

}

void updateRecvNRF(uint8_t recv){
	RecvNRF = recv;
}

void updateRecvGPS(uint8_t recv){
	RecvGPS = recv;
}

uint8_t getNRFMode(){
	return nrf.mode;
}
uint8_t getNRFThrottle(){
	return nrf.th;
}
int8_t getNRFYaw(){
	return nrf.yaw;
}
int8_t getNRFPitch(){
	return nrf.pitch;
}
int8_t getNRFRoll(){
	return nrf.roll;
}
char *getNRFArray(){
	return nrf.testarray;
}
char *getHhmmss(){
	return hhmmss;
}
char *getDdmmyy(){
	return ddmmyy;
}
char *getNorth(){
	return north;
}
char *getEast(){
	return east;
}

void setPrintInt16(int16_t data,uint8_t ope){	//ope=0:\t,ope=1:\r\n
	char tempBuff[16];
	if(ope==0) sprintf(tempBuff,"%5d\t",data);
	else sprintf(tempBuff,"%d\r\n",data);
	strcat(printData,tempBuff);
}
void setPrintFloat(float data,uint8_t ope){	//ope=0:\t,ope=1:\r\n
	char tempBuff[16];
	if(data < INT16_MIN || INT16_MAX < data){
		sprintf(tempBuff,"setPrintFloatError");
		strcat(printData,tempBuff);
		return;
	}
	int32_t re=(int)data;
	int32_t f=data*10;
	f%=10;
	if(f<-1) f*=-1;
	if(ope==0) sprintf(tempBuff,"%d.%d\t",(int16_t)re,(int16_t)f);
	else sprintf(tempBuff,"%d.%d\r\n",(int16_t)re,(int16_t)f);
	strcat(printData,tempBuff);
}
void setPrintChar(char *data,uint8_t ope){
	char tempBuff[16];
	if(ope==0) sprintf(tempBuff,"%s\t",data);
	else sprintf(tempBuff,"%s\r\n",data);
	strcat(printData,tempBuff);
}

void trancePrintIt(uint8_t sd){
//	static uint16_t file_num=0;
	static uint8_t lcount=0;

	static char hmsnl[24]={0};
	int8_t res=sd_Save(printData);
	HAL_Delay(5);
	if(lcount++>50){
		lcount=0;
		sd_Sync();
		//			HAL_Delay(5);
		//			while(sd_open(++file_num)){
		//				error_mess("f_open_error\r\n");
		//				sd_Close();
		//			}
		//			sd_open(++file_num);// sd_Close();	//error
	}
	if(res == 1){	//error
		sd_Close();
	}

	sprintf(hmsnl,"%s\t%d\r\n",getHhmmss(),res);
	HAL_UART_Transmit_IT(&huart1,(uint8_t *)printData,strlen(printData));


}


