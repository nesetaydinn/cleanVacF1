/*
 * taskManagerInterface.c
 *
 *  Created on: Feb 25, 2021
 *      Author: neset
 */

#include "taskManagerInterface.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "motorDriverInterface.h"
#include "ComputerInterface.h"
//#include "InputOutputInterface.h"

#include "usb_device.h"

#include "usbd_cdc_if.h"

void driverInit(void);


void sendDataUart1Task(void *params);
void sendDataUart2Task(void *params);
void getDataUart1Task(void *params);
void getDataUart2Task(void *params);
void adcReadTask(void *params);
void adcReadTask2(void *params);
void lcdTask(void *params);
void buttonControlTask(void *params);

void computerGetTask(void *params);
void computerSendTask(void *params);
void computerValTask(void *params);

void readSwitchsTask(void *params);
void writeRelaysTask(void *params);
void ledTestTask(void *params);


Com_SenderInterface llsendComputerVals;

mD_interface driveDriver;
tMD_interface travelDriver;

uint8_t isStartConnection=0;
void setStartConnection(uint8_t tmp){isStartConnection=tmp;}

uint8_t getStartConnection(void){return isStartConnection; }
xSemaphoreHandle uart1SemphrHandle=NULL;
xSemaphoreHandle uart2SemphrHandle=NULL;
xSemaphoreHandle uart3SemphrHandle=NULL;


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	MDI_getDataChannel1_IT(huart);
	MDI_getDataChannel2_IT(huart);

}

void tasks_init(void){

	 MX_USB_DEVICE_Init();
	/* xTaskCreate(pxTaskCode, pcName, usStackDepth, pvParameters, uxPriority, pxCreatedTask)
	 * @param pxTaskCode-> function name
	 * @param pcName-> task name (string)
	 * @param usStackDepth-> stack size
	 * @param pvParameters-> function parameters
	 * @param uxPriority -> Priority
	 * @param pxCreatedTask -> handle so id
	 * */
	vSemaphoreCreateBinary(uart1SemphrHandle);
	vSemaphoreCreateBinary(uart2SemphrHandle);
	vSemaphoreCreateBinary(uart3SemphrHandle);


	if(uart1SemphrHandle!=NULL && uart2SemphrHandle!=NULL && uart3SemphrHandle!=NULL){


		xTaskCreate(computerValTask, "calc Uart 3val", configMINIMAL_STACK_SIZE, NULL,  55, NULL);
		xTaskCreate(computerSendTask, "send Uart 3", configMINIMAL_STACK_SIZE, NULL,  55, NULL);
		//xTaskCreate(computerGetTask, "get Uart 3", configMINIMAL_STACK_SIZE, NULL,  55 , NULL);

		xTaskCreate(sendDataUart1Task, "send Uart 1", configMINIMAL_STACK_SIZE, NULL,  55 , NULL);
		xTaskCreate(sendDataUart2Task, "send Uart 2", configMINIMAL_STACK_SIZE, NULL,  55 , NULL);

		xTaskCreate(getDataUart1Task, "get Uart 1", configMINIMAL_STACK_SIZE, NULL,  55, NULL);
		xTaskCreate(getDataUart2Task, "get Uart 2", configMINIMAL_STACK_SIZE, NULL,  55 , NULL);

		xTaskCreate(readSwitchsTask, "read switchs", configMINIMAL_STACK_SIZE, NULL,  55 , NULL);
		xTaskCreate(writeRelaysTask, "write relays", configMINIMAL_STACK_SIZE, NULL,  55 , NULL);


	//	xTaskCreate(ledTestTask, "led task", configMINIMAL_STACK_SIZE, NULL,  55 , NULL);


		driverInit();
		IO_init();
		vTaskStartScheduler();
	}
}

void readSwitchsTask(void *params){
	while(1){
		IO_readInputSwitchs();
	}
}
void writeRelaysTask(void *params){
	while(1){
		IO_writeOutputRelay();
	}
}
void ledTestTask(void *params){
	while(1){
		HAL_GPIO_TogglePin(led_GPIO_Port, led_Pin);

		vTaskDelay(500);
	}
}



void computerGetTask(void *params){
	while(1){
		xSemaphoreTake(uart3SemphrHandle, portMAX_DELAY);
		CI_enableGetDataChannel();
		xSemaphoreGive(uart3SemphrHandle);
	}
}

uint16_t mySpeed;
void computerValTask(void *params){
	while(1){
		llsendComputerVals.drive_speed=(int32_t)(getDriver2ReceiveVal().speed)*10000;
			llsendComputerVals.steer_pos =(int32_t)(getDriver1ReceiveVal().angle)*10000;
			if(getDriver2ReceiveVal().encoder<0){
				int intToUint =getDriver2ReceiveVal().encoder;
				intToUint=(~intToUint+1);
				uint16_t tmp =(intToUint/800);
				uint16_t tmp2=(intToUint%800);
				llsendComputerVals.drive_pos= -(tmp+valuesMapforFloat(tmp2,0,800,0.0f,1.0f))*10000;
			}
			else {
				uint16_t tmp =((getDriver2ReceiveVal().encoder)/800);
				uint16_t tmp2=((getDriver2ReceiveVal().encoder)%800);
				llsendComputerVals.drive_pos= (tmp+valuesMapforFloat(tmp2,0,800,0.0f,1.0f))*10000;
			}
			llsendComputerVals.switch_vals=IO_inputsBitsPackageToByte(IO_getInputOutputsVal());
			if(mySpeed<5000)mySpeed+=10;
			else mySpeed=0;
			travelDriver.speed=mySpeed;
			vTaskDelay(250);

	}

}

void computerSendTask(void *params){
	while(1){
	CI_sendDataChannel(0x11,llsendComputerVals.steer_pos ,llsendComputerVals.drive_pos,
				llsendComputerVals.drive_speed,llsendComputerVals.switch_vals,50);

	}
}

void sendDataUart1Task(void *params){

	while(1){
		if(getComputerVals().size)	driveDriver.angle=(uint16_t)((getComputerVals().steer_pos)/10000);
			MDI_sendDataChannel1Ver2(driveDriver.angle,driveDriver.pid_kp,driveDriver.pid_ki,driveDriver.pid_kd,driveDriver.factor,0);

	}
}
uint8_t messageWait;
void sendDataUart2Task(void *params){
	while(1){
		/*if(getComputerVals().size){	travelDriver.speed=(int16_t)((getComputerVals().drive_speed)/10000); messageWait=0;}
		else messageWait++;
		if(messageWait>50)travelDriver.speed=0;*/
		messageWait=IO_getInputOutputsVal() >> 6;
				MDI_sendDataChannel2Ver2(travelDriver.speed,travelDriver.pid_kp,travelDriver.pid_ki,travelDriver.pid_kd,travelDriver.soft_k,travelDriver.soft_f,messageWait);
	}
}
void getDataUart1Task(void *params){
	while(1){
		xSemaphoreTake(uart1SemphrHandle, portMAX_DELAY);
		MDI_enableGetDataChannel1();
		xSemaphoreGive(uart1SemphrHandle);

	}
}
void getDataUart2Task(void *params){
	while(1){
		xSemaphoreTake(uart2SemphrHandle, portMAX_DELAY);
		MDI_enableGetDataChannel2();
		xSemaphoreGive(uart2SemphrHandle);
	}
}

void driverInit(void){
	driveDriver.angle=500;
	driveDriver.pid_kp=100;
	driveDriver.pid_ki=1;
	driveDriver.pid_kd=200;
	driveDriver.factor=100;

	travelDriver.speed =0;
	travelDriver.pid_kp=70;
	travelDriver.pid_ki=1;
	travelDriver.pid_kd=1;
	travelDriver.soft_k =1;
	travelDriver.soft_f =1;

}

/**
 * @brief Creating mapping values for input values
 * @param inValue -> our input value
 * @param inMin -> input interval minumum value
 * @param inMax -> input interval maximum value
 * @param outMin -> output interval minumum value
 * @param outMax -> output interval maximum value
 * @return output value
 */
uint16_t valuesMap(uint16_t inValue, uint16_t inMin, uint16_t inMax, uint16_t outMin, uint16_t outMax) {
	return (inValue - inMin)*(outMax - outMin) / (inMax - inMin) + outMin;
}

int16_t valuesMapInt(int16_t inValue, int16_t inMin, int16_t inMax, int16_t outMin, int16_t outMax) {
	return (inValue - inMin)*(outMax - outMin) / (inMax - inMin) + outMin;
}
uint16_t valuesMapWFloat(float inValue, float inMin, float inMax, uint16_t outMin, uint16_t outMax) {
	return (inValue - inMin)*(outMax - outMin) / (inMax - inMin) + outMin;
}
int16_t valuesMapFloat(float inValue, float inMin, float inMax, int16_t outMin, int16_t outMax) {
	return (inValue - inMin)*(outMax - outMin) / (inMax - inMin) + outMin;
}
float valuesMapforFloat(uint16_t inValue, uint16_t inMin, uint16_t inMax, float outMin, float outMax) {
	return (inValue - inMin)*(outMax - outMin) / (inMax - inMin) + outMin;
}

