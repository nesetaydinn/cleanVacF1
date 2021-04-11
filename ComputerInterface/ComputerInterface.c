/*
 * ComputerInterface.c
 *
 *  Created on: Mar 9, 2021
 *      Author: neset
 */
#include "ComputerInterface.h"
#include "InputOutputInterface.h"

#if COMPUTERINTERFACE_ACTIVE
Com_interface getVals;
/**
 * @brief Write small data to Computer Values
 * @param uartChannel -> get uart channel
 * @param data -> data to write
 * @return none
 */
void CI_writeSmallDataWithRegister(UART_HandleTypeDef *uartChannel, uint8_t data){
	while((uartChannel->Instance->SR & USART_SR_TXE)!=USART_SR_TXE);
	uartChannel->Instance->DR=data;
	while((uartChannel->Instance->SR & USART_SR_TC)!=USART_SR_TC);
}
/**
 * @brief send to Computer Values
 * @param lenght -> bytes lenght
 * @param steer_poss -> steer motor position
 * @param driver_poss -> driver motor position
 * @param driver_speed-> driver motor speed
 * @param batt_per-> battery percent value
 * @return none
 */
uint32_t time1,time2,deltaTime;
void CI_sendDataChannel(uint16_t lenght, int32_t steer_poss,
		int32_t driver_poss, int32_t driver_speed,uint8_t inputStatus,uint8_t batt_per){
	time1=HAL_GetTick();
	uint8_t sendBuff[] = { 0x43, 0X64,
			lenght & 0xFF,lenght >> 8,
			steer_poss & 0xFF,steer_poss >> 8,steer_poss >> 16,steer_poss >> 24,
			driver_poss & 0xFF,driver_poss >> 8,driver_poss >> 16,driver_poss >> 24,
			driver_speed & 0xFF,driver_speed >> 8,driver_speed >> 16,driver_speed >> 24,
			inputStatus,batt_per };
	for (uint8_t counter = 0; counter < 18; counter++) {
		CI_writeSmallDataWithRegister(&ComputerChannel, sendBuff[counter]);
	#if SEND_VAL_CHECK==0
		vTaskDelay(50);
	#endif
	}
	time2=HAL_GetTick();
	deltaTime=time2-time1;
	if(deltaTime<10) vTaskDelay((10-deltaTime));
}

static double test=0;
static int32_t ctSpeed=0;
void testCIsend(void){

	test+=0.01;
		ctSpeed=(int32_t)test*10000;
		uint8_t tmp=IO_inputsBitsPackageToByte(IO_getInputOutputsVal());
		CI_sendDataChannel(0x11,getVals.steer_pos,ctSpeed,getVals.drive_speed,tmp,90);


}


uint8_t  getTmpCH = 0;
/**
 * @brief set enable for MDI channel 1 receive
 * @return none
 */
void CI_enableGetDataChannel(void){
	HAL_UART_Receive_IT(&ComputerChannel,&getTmpCH,1);
}

static uint8_t tmpArr[13];
/**
 * @brief get to Motor Driver 1 values
 * @return none
 */
void CI_getDataChannel_IT(UART_HandleTypeDef *callBackHandle) {
	volatile UART_HandleTypeDef *tmpHandle;
	tmpHandle = &ComputerChannel;
	if(callBackHandle->Instance == tmpHandle->Instance){
		static uint8_t counter = 0, getTmpBeff = 0;
			if (0x64 == getTmpCH && 0x43 == getTmpBeff) {
				tmpArr[0] = 0x43;
				tmpArr[1] = 0x64;
					counter = 1;
				}
			tmpArr[counter] = getTmpCH;
				getTmpBeff = getTmpCH;
				counter++;
				if (counter > 12) {
					counter = 0;
					if (0x43 == tmpArr[0] && 0x64 == tmpArr[1]) {
						getVals.size=(tmpArr[2] & 0xFF )| (tmpArr[3] << 8);
						getVals.steer_pos=(tmpArr[4] & 0xFF )| (tmpArr[5] << 8) | (tmpArr[6] << 16) | (tmpArr[7] << 24);
						getVals.drive_speed=(tmpArr[8] & 0xFF) | (tmpArr[9] << 8) | (tmpArr[10] << 16) | (tmpArr[11] << 24);
						IO_outputByteToBitsPackage(tmpArr[12]);
					}
				}
				HAL_UART_Receive_IT(callBackHandle, &getTmpCH, 1);
	}
}

Com_interface getComputerVals(void) {return getVals;}


#endif
