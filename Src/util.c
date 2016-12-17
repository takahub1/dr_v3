#include "util.h"

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
uint16_t myTick=0;

void utilIncTick(){
	myTick++;
}
uint16_t utilGetTick(){
	return myTick;
}
void util_Delay(uint16_t Delay){
  static uint16_t tickstart = 0;
  tickstart = utilGetTick();
  while((utilGetTick() - tickstart) < Delay);
}
void printv(const char* format, ...){
	va_list arg;
	char mess[32] = {0};
	va_start(arg, format);
	vsprintf(mess,format, arg);
	va_end(arg);
	HAL_UART_Transmit(&huart1,(uint8_t *)mess, 20, 5000);
}
void error_mess(char *mess){
	printv(mess);
	HAL_Delay(100);
}
FATFS FatFs;
FIL File;
char save_directory[32]={0};
char file_name[32]={0};
extern char SD_Path[4];
uint32_t byteswritten=0;
void sd_Init(){
	while(f_mount(&FatFs,(TCHAR const*)SD_Path,0)) error_mess("f_mount\r\n");
	uint8_t get_gps=0;
	for(;get_gps<0x0f;get_gps++){
		if(strlen(getDdmmyy())>6 && strlen(getHhmmss())>6) break;
		HAL_Delay(100);
	}
	sprintf(save_directory,"%s,%s",getDdmmyy(),getHhmmss());
	//		f_mkdir("aaa");
	//	DIR Dir;
	//	f_opendir(&Dir,"");

	while(sd_open(0)) error_mess("f_open_error\r\n");
}
uint8_t sd_open(uint16_t file_num){
	sprintf(file_name,"%s_%d.txt",save_directory,file_num);
	if(strlen(file_name)<20 || 26<strlen(file_name)) return 1;
	return f_open(&File,file_name,FA_CREATE_ALWAYS|FA_WRITE);
}
int8_t sd_Save(char *data){
	__HAL_UART_DISABLE_IT(&huart3, UART_IT_RXNE);
	int8_t res=f_write(&File,data,strlen(data),(void*)&byteswritten);
	__HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);
//	return f_puts(data,&File);
	if((res != FR_OK) || (byteswritten==0)) return 1;
	else return 0;

}
void sd_Close(){
	__HAL_UART_DISABLE_IT(&huart3, UART_IT_RXNE);
	f_close(&File);
//	f_sync(&File);
	__HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);
	HAL_GPIO_TogglePin(led_GPIO_Port, led_Pin);
	HAL_Delay(1);
}
void sd_Sync(){
	__HAL_UART_DISABLE_IT(&huart3, UART_IT_RXNE);
//	f_close(&File);
	f_sync(&File);
	__HAL_UART_ENABLE_IT(&huart3, UART_IT_RXNE);
	HAL_GPIO_TogglePin(led_GPIO_Port, led_Pin);
	HAL_Delay(1);
}
