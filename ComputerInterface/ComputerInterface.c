/*
 * ComputerInterface.c
 *
 *  Created on: Mar 9, 2021
 *      Author: neset
 */
#include "ComputerInterface.h"
#include "InputOutputInterface.h"
#include "taskManagerInterface.h"

#include "usb_device.h"

#include "usbd_cdc_if.h"

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
	//uint8_t * tmp =;
	if(0x0D==getVals.size)CDC_Transmit_FS(sendBuff, 18);
	getVals.size=0;
//	HAL_UART_Transmit_DMA(&ComputerChannel, sendBuff, sizeof(sendBuff));
	/*for (uint8_t counter = 0; counter < 18; counter++) {
		CI_writeSmallDataWithRegister(&ComputerChannel, sendBuff[counter]);
	#if SEND_VAL_CHECK==1
		vTaskDelay(50);
	#endif

	}*/
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



/**
 * @brief get to Computer Variables
 * @return none
 */
void CI_getDataChannel_USB(uint8_t *bytes) {

		//static uint8_t counter = 0, getTmpBeff = 0;
	//for(uint8_t c;c<13;c++)tmpArrUSB[c]=bytes[c];
	if (0x64 == bytes[0] && 0x43 == bytes[1]) {
				bytes[0] = 0x43;
				bytes[1] = 0x64;
				//	counter = 1;
				}
			/*bytes[counter] = byte;
				getTmpBeff = byte;*/
				//counter++;
				/*if (counter > 12) {
					counter = 0;*/
					if (0x43 == bytes[0] && 0x64 == bytes[1]) {
						getVals.size=(bytes[2] & 0xFF )| (bytes[3] << 8);
						getVals.steer_pos=(bytes[4] & 0xFF )| (bytes[5] << 8) | (bytes[6] << 16) | (bytes[7] << 24);
						getVals.drive_speed=(bytes[8] & 0xFF) | (bytes[9] << 8) | (bytes[10] << 16) | (bytes[11] << 24);
						IO_outputByteToBitsPackage(bytes[12]);

					}

			//	}

}

Com_interface getComputerVals(void) {return getVals;}
#endif
