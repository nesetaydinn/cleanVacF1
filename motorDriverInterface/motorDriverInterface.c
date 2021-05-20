/*
 * motorDriverInterface.c
 *
 *  Created on: Feb 15, 2021
 *      Author: neset
 */

#include "motorDriverInterface.h"
#if DRIVERCHANNEL == 1
mD_interface driver1;
uint8_t rec1Buff[9];
#elif DRIVERCHANNEL == 2
mD_interface driver1;
mD_interface driver2;
tMD_interface travelMotor;

uint8_t rec1Buff[20];
uint8_t rec2Buff[20];
#endif


/**
 * @brief Write command to Motor Driver
 * @param uartChannel -> get uart channel
 * @param cmd -> command to write
 * @return none
 */
static void MDI_writeCommand(UART_HandleTypeDef *uartChannel, uint8_t cmd) {
	HAL_UART_Transmit(uartChannel, (uint8_t*) &cmd, sizeof(cmd), TIMEOUTVAL);
}
/**
 * @brief Write small data to Motor Driver
 * @param uartChannel -> get uart channel
 * @param data -> data to write
 * @return none
 */
static void MDI_writeSmallData(UART_HandleTypeDef *uartChannel, uint8_t data) {
	HAL_UART_Transmit(uartChannel, (uint8_t*) &data, sizeof(data), TIMEOUTVAL);
}
/**
 * @brief Write big data to Motor Driver
 * @param uartChannel -> get uart channel
 * @param buff -> get data array
 * @param buff_size -> get data array size
 * @return none
 */
static void MDI_writeBigData(UART_HandleTypeDef *uartChannel, uint8_t *buff,
		size_t buff_size) {
	while (buff_size > 0) {
		uint16_t chunk_size = buff_size > 65535 ? 65535 : buff_size;
		HAL_UART_Transmit(uartChannel, buff, chunk_size, TIMEOUTVAL);
		buff += chunk_size;
		buff_size -= chunk_size;
	}
}
/**
 * @brief Write  2 byte data to Motor Driver
 * @param uartChannel -> get uart channel
 * @param data -> get 2 byte data
 * @return none
 */
void MDI_2byteWriteData(UART_HandleTypeDef *uartChannel, uint16_t data) {
	uint8_t arrTmp[] = { data >> 8, data & 0xFF };
	MDI_writeBigData(uartChannel, arrTmp, sizeof(arrTmp));

}
/**
 * @brief Write small data to Motor Driver
 * @param uartChannel -> get uart channel
 * @param data -> data to write
 * @return none
 */
void MDI_writeSmallDataWithRegister(UART_HandleTypeDef *uartChannel, uint8_t data){
	while((uartChannel->Instance->SR & USART_SR_TXE)!=USART_SR_TXE);
	uartChannel->Instance->DR=data;
	while((uartChannel->Instance->SR & USART_SR_TC)!=USART_SR_TC);
}
/**
 * @brief Write big data to Motor Driver
 * @param uartChannel -> get uart channel
 * @param buff -> get data array
 * @param buff_size -> get data array size
 * @return none
 */
void MDI_writeBigDataWithRegister(UART_HandleTypeDef *uartChannel, uint8_t *buff,
		size_t buff_size){
	while(buff_size>0){
		buff_size--;
		while((uartChannel->Instance->SR & USART_SR_TXE)!=USART_SR_TXE);
		uartChannel->Instance->DR=*buff++;
		while((uartChannel->Instance->SR & USART_SR_TC)!=USART_SR_TC);
	}
}
/**
 * @brief read small data to Motor Driver
 * @param uartChannel -> get uart channel
 * @return data -> 1 byte
 */
void MDI_readSmallDataWithRegister(UART_HandleTypeDef *uartChannel){
	uint8_t data;
	if((uartChannel->Instance->SR & USART_SR_RXNE)==USART_SR_RXNE)
	data=uartChannel->Instance->DR;
	while((uartChannel->Instance->SR & USART_SR_TC)!=USART_SR_TC);
}
#if DRIVERCHANNEL == 1
/**
 * @brief drive to Motor Driver 1
 * @param angleVal -> get motor angle value
 * @param kp -> get Pid kp value
 * @param ki -> get Pid ki value
 * @param kd -> get Pid kd value
 * @param factor -> get factor of Pid elements
 * @return none
 */
void MDI_sendDataChannel1(uint16_t angleVal, uint8_t kd, uint8_t ki, uint8_t kp,
		uint8_t factor) {
	uint16_t checksumTmp = 0;
	MDI_writeCommand(&MDI_channel1, 0xFF);
	MDI_writeCommand(&MDI_channel1, 0xFF); //Data transmission started
	MDI_2byteWriteData(&MDI_channel1, angleVal);
	uint8_t tmpArr[] = { angleVal >> 8, angleVal & 0xFF };
	checksumTmp += tmpArr[0];
	checksumTmp += tmpArr[1]; //2 byte angle val sended
	MDI_writeSmallData(&MDI_channel1, kd);
	checksumTmp += kd; //writed kd
	MDI_writeSmallData(&MDI_channel1, ki);
	checksumTmp += ki; //writed ki
	MDI_writeSmallData(&MDI_channel1, kp);
	checksumTmp += kp; //writed kp
	MDI_writeSmallData(&MDI_channel1, factor);
	checksumTmp += factor; //writed factor
	uint8_t tmp = checksumTmp % 256;
	MDI_writeSmallData(&MDI_channel1, tmp); //checksum first byte
	uint8_t tmpComp = ~tmp;
	MDI_writeSmallData(&MDI_channel1, tmpComp); //checksum second byte

}
/**
 * @brief drive to Motor Driver 1 version 2 (will use for pic drive)
 * @param angleVal -> get motor angle value
 * @param kp -> get Pid kp value
 * @param ki -> get Pid ki value
 * @param kd -> get Pid kd value
 * @param factor -> get factor of Pid elements
 * @return none
 */
void MDI_getDataChannel1Ver2(void) {
	HAL_UART_Receive(&MDI_channel1, (uint8_t*) rec1Buff, 10, TIMEOUTVAL * 10);
	if (0xFF == rec1Buff[0] && 0xFF == rec1Buff[1]) {
		uint16_t checksumTmp = 0;
		for (uint8_t c = 2; c < 8; c++)
			checksumTmp += rec1Buff[c];
		uint8_t tmp = checksumTmp % 256;
		uint8_t tmpComp = ~tmp;
		if (tmp == rec1Buff[8] && tmpComp == rec1Buff[9]) {
			driver1.angle = ((uint16_t) rec1Buff[2] << 8) | rec1Buff[3];
			driver1.pid_kp = rec1Buff[4];
			driver1.pid_ki = rec1Buff[5];
			driver1.pid_kd = rec1Buff[6];
			driver1.factor = rec1Buff[7];
		}

	}
}
/**
 * @brief get to Motor Driver 1 values
 * @return none
 */
void MDI_getDataChannel1(void){
	HAL_UART_Receive(&MDI_channel1,(uint8_t*)rec1Buff,10,50);
	if(0xFF==rec1Buff[0] && 0xFF==rec1Buff[1]){
		uint16_t checksumTmp=0;
		for(uint8_t c=2;c<8;c++)checksumTmp+=rec1Buff[c];
		uint8_t tmp =checksumTmp%256;
		uint8_t tmpComp =~tmp;
		if(tmp == rec1Buff[8] && tmpComp == rec1Buff[9]){
			driver1.angle=((uint16_t)rec1Buff[2] << 8) | rec1Buff[3];
			driver1.pid_kd=rec1Buff[4];
			driver1.pid_ki=rec1Buff[5];
			driver1.pid_kp=rec1Buff[6];
			driver1.factor=rec1Buff[7];
		}
	}
	for(uint8_t c=2;c<8;c++)rec1Buff[c]=0;
	//"angle: %d - kd: %d - ki: %d - kp: %d - factor: %d\n",driver1.angle,driver1.pid_kd,driver1.pid_ki,driver1.pid_kp,driver1.factor
}
#elif DRIVERCHANNEL == 2
/**
 * @brief drive to Motor Driver 1
 * @param angleVal -> get motor angle value
 * @param kp -> get Pid kp value
 * @param ki -> get Pid ki value
 * @param kd -> get Pid kd value
 * @param factor -> get factor of Pid elements
 * @return none
 */
void MDI_sendDataChannel1(uint16_t angleVal, uint8_t kd, uint8_t ki, uint8_t kp,
		uint8_t factor) {
	uint16_t checksumTmp = 0;
	MDI_writeCommand(&MDI_channel1TX, 0xFF);
	MDI_writeCommand(&MDI_channel1TX, 0xFF); //Data transmission started
	MDI_2byteWriteData(&MDI_channel1TX, angleVal);
	uint8_t tmpArr[] = { angleVal >> 8, angleVal & 0xFF };
	checksumTmp += tmpArr[0];
	checksumTmp += tmpArr[1]; //2 byte angle val sended
	MDI_writeSmallData(&MDI_channel1TX, kd);
	checksumTmp += kd; //writed kd
	MDI_writeSmallData(&MDI_channel1TX, ki);
	checksumTmp += ki; //writed ki
	MDI_writeSmallData(&MDI_channel1TX, kp);
	checksumTmp += kp; //writed kp
	MDI_writeSmallData(&MDI_channel1TX, factor);
	checksumTmp += factor; //writed factor
	uint8_t tmp = checksumTmp % 256;
	MDI_writeSmallData(&MDI_channel1TX, tmp); //checksum first byte
	uint8_t tmpComp = ~tmp;
	MDI_writeSmallData(&MDI_channel1TX, tmpComp); //checksum second byte

}
/**
 * @brief drive to Motor Driver 1 version 2 (will use for pic drive)
 * @param angleVal -> get motor angle value
 * @param kp -> get Pid kp value
 * @param ki -> get Pid ki value
 * @param kd -> get Pid kd value
 * @param factor -> get factor of Pid elements
 * @return none
 */
void MDI_sendDataChannel1Ver2(uint16_t angleVal, uint8_t kp, uint8_t ki,
		uint8_t kd, uint8_t factor,uint8_t last_Bits) {
	uint16_t checksumTmp = 0;
	uint8_t tmpArr[] = { angleVal >> 8, angleVal & 0xFF };
	checksumTmp += tmpArr[0];
	checksumTmp += tmpArr[1];
	checksumTmp += kp;
	checksumTmp += ki;
	checksumTmp += kd;
	checksumTmp += factor;
	checksumTmp += last_Bits;
	uint8_t tmp = checksumTmp % 256;
	uint8_t tmpComp = ~tmp;
	uint8_t sendBuff[11] = { 0XFF, 0XFF, tmpArr[0], tmpArr[1], kp, ki, kd,
			factor, last_Bits,tmp, tmpComp };
	for (uint8_t counter = 0; counter < 11; counter++) {
		MDI_writeSmallDataWithRegister(&MDI_channel1TX, sendBuff[counter]);
#if SEND_VAL_CHECK==0
		vTaskDelay(1);
#elif SEND_VAL_CHECK==1
		vTaskDelay(50);
#endif
	}
}

/**
 * @brief get to Motor Driver 1 values
 * @return none
 */
uint8_t getFirstData1;
void MDI_getDataChannel1(void) {
	HAL_UART_Receive(&MDI_channel1RX, (uint8_t*) &getFirstData1, 1, TIMEOUTVAL);
	if (0xFF == getFirstData1) {
		HAL_UART_Receive(&MDI_channel1RX, (uint8_t*) rec1Buff, 9, TIMEOUTVAL * 9);
		if (0xFF == rec1Buff[0]) {
			uint16_t checksumTmp = 0;
			for (uint8_t c = 1; c < 7; c++)
				checksumTmp += rec1Buff[c];
			uint8_t tmp = checksumTmp % 256;
			uint8_t tmpComp = ~tmp;
			if (tmp == rec1Buff[7] && tmpComp == rec1Buff[8]) {
				driver1.angle = ((uint16_t) rec1Buff[1] << 8) | rec1Buff[2];
				driver1.pid_kp = rec1Buff[3];
				driver1.pid_ki = rec1Buff[4];
				driver1.pid_kd = rec1Buff[5];
				driver1.factor = rec1Buff[6];
			}
		}
		for (uint8_t c = 0; c < 9; c++)
			rec1Buff[c] = 0;
	}
	getFirstData1 = 0;
}
/**
 * @brief get to Motor Driver 1 values
 * @return none
 */
void MDI_getDataChannel1Ver2(void) {
	HAL_UART_Receive(&MDI_channel1RX, (uint8_t*) rec1Buff, 10, TIMEOUTVAL * 10);
	if (0xFF == rec1Buff[0] && 0xFF == rec1Buff[1]) {
		uint16_t checksumTmp = 0;
		for (uint8_t c = 2; c < 8; c++)
			checksumTmp += rec1Buff[c];
		uint8_t tmp = checksumTmp % 256;
		uint8_t tmpComp = ~tmp;
		if (tmp == rec1Buff[8] && tmpComp == rec1Buff[9]) {
			driver1.angle = ((uint16_t) rec1Buff[2] << 8) | rec1Buff[3];
			driver1.pid_kp = rec1Buff[4];
			driver1.pid_ki = rec1Buff[5];
			driver1.pid_kd = rec1Buff[6];
			driver1.factor = rec1Buff[7];
		}

	}
}
/**
 * @brief get to Motor Driver 1 values
 * @return none
 */

void MDI_getDataChannel1Ver3(void) {
	static uint8_t tmpArr1[10];
	volatile UART_HandleTypeDef *tmpHandle;
	tmpHandle = &MDI_channel1RX;
	static uint8_t counter = 0, getTmp = 0, getTmpBeff = 0;
	if ((tmpHandle->Instance->SR & USART_SR_RXNE) == USART_SR_RXNE) {
		getTmp = tmpHandle->Instance->DR;
		if (0xFF == getTmp && 0xFF == getTmpBeff) {
			tmpArr1[0] = 0xFF;
			tmpArr1[1] = 0xFF;
			counter = 1;
		}
		tmpArr1[counter] = getTmp;
		getTmpBeff = getTmp;
		counter++;
		if (counter > 9) {
			counter = 0;
			uint16_t checksumTmp = 0;
			for (uint8_t c = 2; c < 8; c++)
				checksumTmp += tmpArr1[c];
			uint8_t tmp = checksumTmp % 256;
			uint8_t tmpComp = ~tmp;
			if (tmp == tmpArr1[8] && tmpComp == tmpArr1[9]) {
				driver1.angle = ((uint16_t) tmpArr1[2] << 8) | tmpArr1[3];
				driver1.pid_kp = tmpArr1[4];
				driver1.pid_ki = tmpArr1[5];
				driver1.pid_kd = tmpArr1[6];
				driver1.factor = tmpArr1[7];
			}
		}
		//	__HAL_UART_CLEAR_FLAG(tmpHandle, UART_FLAG_TC); //been UART receiver timeout clear flag
			tmpHandle->RxState = HAL_UART_STATE_READY; //been set receive state ready
			tmpHandle->ErrorCode = HAL_UART_ERROR_NONE; //if was being error than be clean the error
			SET_BIT(tmpHandle->Instance->DR, 0); //if rxstate not catch for cleanig then be cleaning RDR register
		}

		if ((tmpHandle->Instance->SR & USART_SR_ORE) != 0)
			tmpHandle->Instance->DR;
		//tmpHandle->Instance->CR |= USART_CR_ORECF; //if occur a Overrun error then cleaning ORE(Overrun) bit

}
/**
 * @brief get to Motor Driver 1 values
 * @return none
 */
void MDI_getDataChannel1Ver4(void) {
	HAL_UART_Receive(&MDI_channel1RX, (uint8_t*) rec1Buff, 20, TIMEOUTVAL * 150);
	for (uint8_t counter = 0; counter < sizeof(rec1Buff); counter++) {
		if (rec1Buff[counter] == 0xFF && rec1Buff[counter + 1] == 0xFF) {
			uint16_t checksumTmp = 0;
			uint8_t tmpArr[10];
			for (uint8_t counter2 = 0; counter2 < sizeof(tmpArr); counter2++) {
				tmpArr[counter2] = rec1Buff[counter + counter2];
			}
			for (uint8_t c = 2; c < 8; c++)
				checksumTmp += tmpArr[c];
			uint8_t tmp = checksumTmp % 256;
			uint8_t tmpComp = ~tmp;
			if (tmp == tmpArr[8] && tmpComp == tmpArr[9]) {
				driver1.angle = ((uint16_t) tmpArr[2] << 8) | tmpArr[3];
				driver1.pid_kp = tmpArr[4];
				driver1.pid_ki = tmpArr[5];
				driver1.pid_kd = tmpArr[6];
				driver1.factor = tmpArr[7];
			}
			return;
		}
	}
}
uint8_t  getTmpCH1 = 0;
/**
 * @brief set enable for MDI channel 1 receive
 * @return none
 */
void MDI_enableGetDataChannel1(void){
	HAL_UART_Receive_IT(&MDI_channel1RX,&getTmpCH1,1);
}

static uint8_t tmpArr1[11];
/**
 * @brief get to Motor Driver 1 values
 * @return none
 */
void MDI_getDataChannel1_IT(UART_HandleTypeDef *callBackHandle) {
	volatile UART_HandleTypeDef *tmpHandle;
	tmpHandle = &MDI_channel1RX;
	if(callBackHandle->Instance == tmpHandle->Instance){
		static uint8_t counter = 0, getTmpBeff = 0;
			if (0xFF == getTmpCH1 && 0xFF == getTmpBeff) {
					tmpArr1[0] = 0xFF;
					tmpArr1[1] = 0xFF;
					counter = 1;
				}
				tmpArr1[counter] = getTmpCH1;
				getTmpBeff = getTmpCH1;
				counter++;
				if (counter > 10) {
					counter = 0;
					uint16_t checksumTmp = 0;
					for (uint8_t c = 2; c < 9; c++)
						checksumTmp += tmpArr1[c];
					uint8_t tmp = checksumTmp % 256;
					uint8_t tmpComp = ~tmp;
					if (tmp == tmpArr1[9] && tmpComp == tmpArr1[10]) {
						driver1.angle = ((uint16_t) tmpArr1[2] << 8) | tmpArr1[3];
						driver1.pid_kp = tmpArr1[4];
						driver1.pid_ki = tmpArr1[5];
						driver1.pid_kd = tmpArr1[6];
						driver1.factor = tmpArr1[7];
						driver1.bit_1=(MDI_BIT_0 &  tmpArr1[8]);
						driver1.bit_2=(MDI_BIT_1 &  tmpArr1[8])>>1;
					}
				}
				HAL_UART_Receive_IT(callBackHandle, &getTmpCH1, 1);
	}
}
/**
 * @brief drive to Motor Driver 2
 * @param angleVal -> get motor angle value
 * @param kp -> get Pid kp value
 * @param ki -> get Pid ki value
 * @param kd -> get Pid kd value
 * @param factor -> get factor of Pid elements
 * @return none
 */
void MDI_sendDataChannel2(uint16_t angleVal, uint8_t kp, uint8_t ki, uint8_t kd,
		uint8_t factor) {
	uint16_t checksumTmp = 0;
	MDI_writeCommand(&MDI_channel2TX, 0xFF);
	MDI_writeCommand(&MDI_channel2TX, 0xFF); //Data transmission started
	MDI_2byteWriteData(&MDI_channel2TX, angleVal);
	uint8_t tmpArr[] = { angleVal >> 8, angleVal & 0xFF };
	checksumTmp += tmpArr[0];
	checksumTmp += tmpArr[1]; //2 byte angle val sended
	MDI_writeSmallData(&MDI_channel2TX, kp);
	checksumTmp += kp; //writed kp
	MDI_writeSmallData(&MDI_channel2TX, ki);
	checksumTmp += ki; //writed ki
	MDI_writeSmallData(&MDI_channel2TX, kd);
	checksumTmp += kd; //writed kd
	MDI_writeSmallData(&MDI_channel2TX, factor);
	checksumTmp += factor; //writed factor
	uint8_t tmp = checksumTmp % 256;
	MDI_writeSmallData(&MDI_channel2TX, tmp); //checksum first byte
	uint8_t tmpComp = ~tmp;
	MDI_writeSmallData(&MDI_channel2TX, tmpComp); //checksum second byte
}
/**
 * @brief drive to Motor Driver 2 version 2 (will use for pic drive)
 * @param angleVal -> get motor angle value
 * @param kp -> get Pid kp value
 * @param ki -> get Pid ki value
 * @param kd -> get Pid kd value
 * @param factor -> get factor of Pid elements
 * @return none
 */
void MDI_sendDataChannel2Ver2(int16_t speed, uint8_t kp, uint8_t ki,uint8_t kd,
		uint8_t soft_k, uint8_t soft_f,uint8_t last_Bits) {
	uint16_t checksumTmp = 0;
	uint8_t tmpArr[] = { speed >> 8, speed & 0xFF };
	checksumTmp += tmpArr[0];
	checksumTmp += tmpArr[1];
	checksumTmp += kp;
	checksumTmp += ki;
	checksumTmp += kd;
	checksumTmp += soft_k;
	checksumTmp += soft_f;
	checksumTmp += last_Bits;
	uint8_t tmp = checksumTmp % 256;
	uint8_t tmpComp = ~tmp;
	uint8_t sendBuff[12] = { 0XFF, 0XFF,kp, tmpArr[0],ki, tmpArr[1], kd,
			soft_k,soft_f,last_Bits, tmp, tmpComp };
	for (uint8_t counter = 0; counter < 12; counter++) {
		MDI_writeSmallDataWithRegister(&MDI_channel2TX, sendBuff[counter]);
	#if SEND_VAL_CHECK==0
		vTaskDelay(1);
	#elif SEND_VAL_CHECK==1
		vTaskDelay(50);
	#endif
	}
}
/**
 * @brief get to Motor Driver 2 values
 * @return none
 */
uint8_t getFirstData2;
void MDI_getDataChannel2(void) {
	HAL_UART_Receive(&MDI_channel2RX, (uint8_t*) &getFirstData2, 1, TIMEOUTVAL);
	if (0xFF == getFirstData2) {
		HAL_UART_Receive(&MDI_channel2RX, (uint8_t*) rec2Buff, 9, TIMEOUTVAL * 9);
		if (0xFF == rec2Buff[0]) {
			uint16_t checksumTmp = 0;
			for (uint8_t c = 1; c < 7; c++)
				checksumTmp += rec2Buff[c];
			uint8_t tmp = checksumTmp % 256;
			uint8_t tmpComp = ~tmp;
			if (tmp == rec2Buff[7] && tmpComp == rec2Buff[8]) {
				driver2.angle = ((uint16_t) rec2Buff[1] << 8) | rec2Buff[2];
				driver2.pid_kp = rec2Buff[3];
				driver2.pid_ki = rec2Buff[4];
				driver2.pid_kd = rec2Buff[5];
				driver2.factor = rec2Buff[6];
			}
		}
		for (uint8_t c = 0; c < 9; c++)
			rec2Buff[c] = 0;
	}
	getFirstData2 = 0;
}
/**
 * @brief get to Motor Driver 2 values
 * @return none
 */
void MDI_getDataChannel2Ver2(void) {

	HAL_UART_Receive(&MDI_channel2RX, (uint8_t*) rec2Buff, 10, TIMEOUTVAL * 10);
	if (0xFF == rec2Buff[0] && 0xFF == rec2Buff[1]) {
		uint16_t checksumTmp = 0;
		for (uint8_t c = 2; c < 8; c++)
			checksumTmp += rec2Buff[c];
		uint8_t tmp = checksumTmp % 256;
		uint8_t tmpComp = ~tmp;
		if (tmp == rec2Buff[8] && tmpComp == rec2Buff[9]) {
			driver2.angle = ((uint16_t) rec2Buff[2] << 8) | rec2Buff[3];
			driver2.pid_kp = rec2Buff[4];
			driver2.pid_ki = rec2Buff[5];
			driver2.pid_kd = rec2Buff[6];
			driver2.factor = rec2Buff[7];
		}
	}
}
/**
 * @brief get to Motor Driver 2 values
 * @return none
 */
void MDI_getDataChannel2Ver3(void) {

	static uint8_t tmpArr2[10];
	volatile UART_HandleTypeDef *tmpHandle;
	tmpHandle = &MDI_channel2RX;
	static uint8_t counter = 0, getTmp = 0, getTmpBeff = 0;
	if ((tmpHandle->Instance->SR & USART_SR_RXNE) != 0) {
		getTmp = tmpHandle->Instance->DR;
		if (0xFF == getTmp && 0xFF == getTmpBeff) {
			tmpArr2[0] = 0xFF;
			tmpArr2[1] = 0xFF;
			counter = 1;
		}
		tmpArr2[counter] = getTmp;
		getTmpBeff = getTmp;
		counter++;
		if (counter > 9) {
			counter = 0;
			uint16_t checksumTmp = 0;
			for (uint8_t c = 2; c < 8; c++)
				checksumTmp += tmpArr2[c];
			uint8_t tmp = checksumTmp % 256;
			uint8_t tmpComp = ~tmp;
			if (tmp == tmpArr2[8] && tmpComp == tmpArr2[9]) {
				driver2.angle = ((uint16_t) tmpArr2[2] << 8) | tmpArr2[3];
				driver2.pid_kp = tmpArr2[4];
				driver2.pid_ki = tmpArr2[5];
				driver2.pid_kd = tmpArr2[6];
				driver2.factor = tmpArr2[7];
			}
		}

	//	__HAL_UART_CLEAR_FLAG(tmpHandle, UART_FLAG_TC); //been UART receiver timeout clear flag
		tmpHandle->RxState = HAL_UART_STATE_READY; //been set receive state ready
		tmpHandle->ErrorCode = HAL_UART_ERROR_NONE; //if was being error than be clean the error
		SET_BIT(tmpHandle->Instance->DR, 0); //if rxstate not catch for cleanig then be cleaning RDR register
	}
	__HAL_UART_ENABLE_IT(tmpHandle, UART_IT_ERR);
    __HAL_UART_ENABLE_IT(tmpHandle, UART_IT_RXNE);
}
/**
 * @brief get to Motor Driver 2 values
 * @return none
 */
void MDI_getDataChannel2Ver4(void) {
	HAL_UART_Receive(&MDI_channel2RX, (uint8_t*) &rec2Buff, 20, TIMEOUTVAL * 150);
	for (uint8_t counter = 0; counter < sizeof(rec2Buff); counter++) {
		if (rec2Buff[counter] == 0xFF && rec2Buff[counter + 1] == 0xFF) {
			uint16_t checksumTmp = 0;
			uint8_t tmpArr[10];
			for (uint8_t counter2 = 0; counter2 < sizeof(tmpArr); counter2++) {
				tmpArr[counter2] = rec2Buff[counter + counter2];
			}
			for (uint8_t c = 2; c < 8; c++)
				checksumTmp += tmpArr[c];
			uint8_t tmp = checksumTmp % 256;
			uint8_t tmpComp = ~tmp;
			if (tmp == tmpArr[8] && tmpComp == tmpArr[9]) {
				driver2.angle = ((uint16_t) tmpArr[2] << 8) | tmpArr[3];
				driver2.pid_kp = tmpArr[4];
				driver2.pid_ki = tmpArr[5];
				driver2.pid_kd = tmpArr[6];
				driver2.factor = tmpArr[7];
			}

		}
	}

}
/**
 * @brief set enable for MDI channel 2 receive
 * @return none
 */
uint8_t  getTmpCH2 = 0;
void MDI_enableGetDataChannel2(void){
	HAL_UART_Receive_IT(&MDI_channel2RX,&getTmpCH2,1);
}
/**
 * @brief get to Motor Driver 2 values
 * @return callBackHandle-> get u(s)art handle
 */
static uint8_t tmpArr2[17];
void MDI_getDataChannel2_IT(UART_HandleTypeDef *callBackHandle) {
	volatile UART_HandleTypeDef *tmpHandle;
	tmpHandle = &MDI_channel2RX;
	if(callBackHandle->Instance == tmpHandle->Instance){
		static uint8_t counter = 0, getTmpBeff = 0;
			if (0xFF == getTmpCH2 && 0xFF == getTmpBeff) {
				tmpArr2[0] = 0xFF;
				tmpArr2[1] = 0xFF;
				counter = 1;
			}
			tmpArr2[counter] = getTmpCH2;
			getTmpBeff = getTmpCH2;
			counter++;
			if (counter > 16) {
				counter = 0;
				uint16_t checksumTmp = 0;
				for (uint8_t c = 2; c < 15; c++)
					checksumTmp += tmpArr2[c];
				uint8_t tmp = checksumTmp % 256;
				uint8_t tmpComp = ~tmp;
				if (tmp == tmpArr2[15] && tmpComp == tmpArr2[16]) {
					travelMotor.encoder =(((tmpArr2[7] << 24)) | ((tmpArr2[9]<< 16)) | ((tmpArr2[11]<< 8)) | (tmpArr2[13] & 0xFF));
					travelMotor.speed = ( (tmpArr2[3] << 8) |(tmpArr2[5] & 0xFF));
					travelMotor.pid_kp = tmpArr2[2];
					travelMotor.pid_ki = tmpArr2[4];
					travelMotor.pid_kd = tmpArr2[6];
					travelMotor.soft_k = tmpArr2[8];
					travelMotor.soft_f = tmpArr2[10];
					travelMotor.bit_1=(MDI_BIT_0 &  tmpArr2[14]);
					travelMotor.bit_2=(MDI_BIT_1 &  tmpArr2[14])>>1;
				}
			}
			HAL_UART_Receive_IT(callBackHandle, &getTmpCH2, 1);
	}
}

#endif

#if DRIVERCHANNEL == 1
uint8_t getDriver1kd(void){ return driver1.angle;}
uint8_t getDriver1ki(void){ return driver1.pid_kd;}
uint8_t getDriver1kp(void){ return driver1.pid_ki;}
uint8_t getDriver1factor(void){ return driver1.pid_kp;}
uint16_t getDriver1angle(void){return  driver1.angle;}
mD_interface getDriver1ReceiveVal(void){return driver1;}
#elif DRIVERCHANNEL == 2
uint8_t getDriver1kd(void) {
	return driver1.angle;
}
uint8_t getDriver1ki(void) {
	return driver1.pid_kd;
}
uint8_t getDriver1kp(void) {
	return driver1.pid_ki;
}
uint8_t getDriver1factor(void) {
	return driver1.pid_kp;
}
uint16_t getDriver1angle(void) {
	return driver1.angle;
}

uint8_t getDriver2kd(void) {
	return driver2.angle;
}
uint8_t getDriver2ki(void) {
	return driver2.pid_kd;
}
uint8_t getDriver2kp(void) {
	return driver2.pid_ki;
}
uint8_t getDriver2factor(void) {
	return driver2.pid_kp;
}
uint16_t getDriver2angle(void) {
	return driver2.factor;
}

mD_interface getDriver1ReceiveVal(void) {
	return driver1;
}
tMD_interface getDriver2ReceiveVal(void) {
	return travelMotor;
}
#endif

