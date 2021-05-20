/*
 * motorDriverInterface.h
 *
 *  Created on: Feb 15, 2021
 *      Author: neset
 */

#ifndef MOTORDRIVERINTERFACE_H_
#define MOTORDRIVERINTERFACE_H_

#include "main.h"
#include "FreeRTOS.h"
#include "task.h"

#define DRIVERCHANNEL 2 //how many are there driver
//Note: uart timeout value when be long then data transmission is safe
#define TIMEOUTVAL 10 //uart transmission timeout preiod

// SEND_VAL_CHECK->0 realtime
// SEND_VAL_CHECK->1 pc debug
#define SEND_VAL_CHECK 1

typedef struct{
    uint8_t pid_kd;
    uint8_t pid_ki;
    uint8_t pid_kp;
    uint8_t factor;
    uint16_t angle;
    uint8_t bit_1:1;
    uint8_t bit_2:1;
}mD_interface;

typedef enum
{
	MDI_BIT_0 = 1u,
	MDI_BIT_1 = 2u,
	MDI_BIT_2 = 4u,
	MDI_BIT_3 = 8u,
	MDI_BIT_4 = 16u,
	MDI_BIT_5 = 32u,
	MDI_BIT_6 = 64u,
	MDI_BIT_7 = 128u
} MDI_bitValue;

typedef struct{
	uint8_t pid_kp;
    uint8_t pid_ki;
    uint8_t pid_kd;
    int16_t speed;
    uint8_t soft_f;
    uint8_t soft_k;
    int32_t encoder;
    uint8_t bit_1:1;
    uint8_t bit_2:1;
}tMD_interface;


#if DRIVERCHANNEL == 1
#define MDI_channel1 huart1
extern UART_HandleTypeDef MDI_channel1;

uint8_t getDriver1kd(void);
uint8_t getDriver1ki(void);
uint8_t getDriver1kp(void);
uint8_t getDriver1factor(void);
uint16_t getDriver1angle(void);

void MDI_sendDataChannel1(uint16_t angleVal,uint8_t kp,uint8_t ki,uint8_t kd,uint8_t rate );
void MDI_sendDataChannel1Ver2(uint16_t angleVal,uint8_t kp,uint8_t ki,uint8_t kd,uint8_t rate );
void MDI_getDataChannel1(void);
void MDI_getDataChannel1Ver2(void);
void MDI_getDataChannel1Ver3(void);
mD_interface getDriver1ReceiveVal(void);
#elif DRIVERCHANNEL == 2
#define MDI_channel1TX huart2
extern UART_HandleTypeDef MDI_channel1TX;

#define MDI_channel2TX huart3 //uart channel
extern UART_HandleTypeDef MDI_channel2TX;

#define MDI_channel1RX MDI_channel1TX//huart1
extern UART_HandleTypeDef MDI_channel1RX;

#define MDI_channel2RX MDI_channel2TX// huart2 //uart channel
extern UART_HandleTypeDef MDI_channel2RX;


uint8_t getDriver1kd(void);
uint8_t getDriver1ki(void);
uint8_t getDriver1kp(void);
uint8_t getDriver1factor(void);
uint16_t getDriver1angle(void);

uint8_t getDriver2kd(void);
uint8_t getDriver2ki(void);
uint8_t getDriver2kp(void);
uint8_t getDriver2factor(void);
uint16_t getDriver2angle(void);

void MDI_sendDataChannel1(uint16_t angleVal,uint8_t kp,uint8_t ki,uint8_t kd,uint8_t rate );
void MDI_sendDataChannel1Ver2(uint16_t angleVal,uint8_t kp,uint8_t ki,uint8_t kd,uint8_t rate ,uint8_t last_Bits);
void MDI_getDataChannel1(void);
void MDI_getDataChannel1Ver2(void);
void MDI_getDataChannel1Ver3(void);
void MDI_getDataChannel1Ver4(void);
void MDI_enableGetDataChannel1(void);
void MDI_getDataChannel1_IT(UART_HandleTypeDef *callBackHandle);
void MDI_sendDataChannel2(uint16_t angleVal,uint8_t kp,uint8_t ki,uint8_t kd,uint8_t rate );
void MDI_sendDataChannel2Ver2(int16_t speed, uint8_t kp, uint8_t ki,uint8_t kd,
		uint8_t soft_k, uint8_t soft_f,uint8_t last_Bits) ;
void MDI_getDataChannel2(void);
void MDI_getDataChannel2Ver2(void);
void MDI_getDataChannel2Ver3(void);
void MDI_getDataChannel2Ver4(void);
void MDI_enableGetDataChannel2(void);
void MDI_getDataChannel2_IT(UART_HandleTypeDef *callBackHandle);

mD_interface getDriver1ReceiveVal(void);
tMD_interface getDriver2ReceiveVal(void);

#endif

//"%d - %d - %d - %d - %d - %d - %d - %d - %d\n",rec1Buff[0],rec1Buff[1],rec1Buff[2],rec1Buff[3],rec1Buff[4],rec1Buff[5],rec1Buff[6],rec1Buff[7],rec1Buff[8]
//"DRV 1: angle: %d - kd: %d - ki: %d - kp: %d - factor: %d\n",driver1.angle,driver1.pid_kd,driver1.pid_ki,driver1.pid_kp,driver1.factor
//"%d - %d - %d - %d - %d - %d - %d - %d - %d\n",rec1Buff[0],rec1Buff[1],rec1Buff[2],rec1Buff[3],rec1Buff[4],rec1Buff[5],rec1Buff[6],rec1Buff[7],rec1Buff[8]
//"DRV 1: angle: %d - kd: %d - ki: %d - kp: %d - factor: %d\n",driver1.angle,driver1.pid_kd,driver1.pid_ki,driver1.pid_kp,driver1.factor
//HAL_UART_Transmit(&MDI_channel2,(uint8_t*)&sendBuff,sizeof(sendBuff),TIMEOUTVAL);
//"%d - %d - %d - %d - %d - %d - %d - %d - %d - %d\n",sendBuff[0],sendBuff[1],sendBuff[2],sendBuff[3],sendBuff[4],sendBuff[5],sendBuff[6],sendBuff[7],sendBuff[8],sendBuff[9]
//HAL_UART_Transmit(&MDI_channel2,(uint8_t*)&sendBuff,sizeof(sendBuff),TIMEOUTVAL);
	//"%d - %d - %d - %d - %d - %d - %d - %d - %d - %d\n",sendBuff[0],sendBuff[1],sendBuff[2],sendBuff[3],sendBuff[4],sendBuff[5],sendBuff[6],sendBuff[7],sendBuff[8],sendBuff[9]
//"DRV 2: angle: %d - kd: %d - ki: %d - kp: %d - factor: %d\n",driver2.angle,driver2.pid_kd,driver2.pid_ki,driver2.pid_kp,driver2.factor
//"DRV 2: angle: %d - kd: %d - ki: %d - kp: %d - factor: %d\n",driver2.angle,driver2.pid_kd,driver2.pid_ki,driver2.pid_kp,driver2.factor
//"DRV 2: angle: %d - kd: %d - ki: %d - kp: %d - factor: %d\n",driver2.angle,driver2.pid_kd,driver2.pid_ki,driver2.pid_kp,driver2.factor



#endif /* MOTORDRIVERINTERFACE_H_ */
