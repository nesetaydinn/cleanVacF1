/*
 * InputOutputInterface.h
 *
 *  Created on: Apr 2, 2021
 *      Author: neset
 */

#ifndef INPUTOUTPUTINTERFACE_H_
#define INPUTOUTPUTINTERFACE_H_

#include "main.h"
#include "stdbool.h"
typedef struct{
	bool switch_1:1;
	bool switch_2:1;
	bool switch_3:1;
	bool switch_4:1;
	bool switch_5:1;
	bool switch_6:1;
	bool switch_7:1;
	bool switch_8:1;

	bool relay_1:1;
	bool relay_2:1;
	bool relay_3:1;
	bool relay_4:1;
	bool relay_5:1;
	bool relay_6:1;
	bool relay_7:1;
	bool relay_8:1;

}IO_interface;


typedef enum
{
  IO_BIT_0 = 1u,
  IO_BIT_1 = 2u,
  IO_BIT_2 = 4u,
  IO_BIT_3 = 8u,
  IO_BIT_4 = 16u,
  IO_BIT_5 = 32u,
  IO_BIT_6 = 64u,
  IO_BIT_7 = 128u
} IO_bitValue;

#define SW_I_1 HAL_GPIO_ReadPin(switch_1_GPIO_Port,switch_1_Pin)
#define SW_I_2 HAL_GPIO_ReadPin(switch_2_GPIO_Port,switch_2_Pin)
#define SW_I_3 HAL_GPIO_ReadPin(switch_3_GPIO_Port,switch_3_Pin)
#define SW_I_4 HAL_GPIO_ReadPin(switch_4_GPIO_Port,switch_4_Pin)
#define SW_I_5 HAL_GPIO_ReadPin(switch_5_GPIO_Port,switch_5_Pin)
#define SW_I_6 HAL_GPIO_ReadPin(switch_6_GPIO_Port,switch_6_Pin)
#define SW_I_7 HAL_GPIO_ReadPin(switch_7_GPIO_Port,switch_7_Pin)
#define SW_I_8 HAL_GPIO_ReadPin(switch_8_GPIO_Port,switch_8_Pin)


#define R_1_O_HIGH HAL_GPIO_WritePin(relay_1_GPIO_Port,relay_1_Pin,GPIO_PIN_RESET)
#define R_2_O_HIGH HAL_GPIO_WritePin(relay_2_GPIO_Port,relay_2_Pin,GPIO_PIN_RESET)
#define R_3_O_HIGH HAL_GPIO_WritePin(relay_3_GPIO_Port,relay_3_Pin,GPIO_PIN_RESET)
#define R_4_O_HIGH HAL_GPIO_WritePin(relay_4_GPIO_Port,relay_4_Pin,GPIO_PIN_RESET)
#define R_5_O_HIGH HAL_GPIO_WritePin(relay_5_GPIO_Port,relay_5_Pin,GPIO_PIN_RESET)
#define R_6_O_HIGH HAL_GPIO_WritePin(relay_6_GPIO_Port,relay_6_Pin,GPIO_PIN_RESET)
#define R_7_O_HIGH HAL_GPIO_WritePin(relay_7_GPIO_Port,relay_7_Pin,GPIO_PIN_RESET)
#define R_8_O_HIGH HAL_GPIO_WritePin(relay_8_GPIO_Port,relay_8_Pin,GPIO_PIN_RESET)


#define R_1_O_LOW HAL_GPIO_WritePin(relay_1_GPIO_Port,relay_1_Pin,GPIO_PIN_SET)
#define R_2_O_LOW HAL_GPIO_WritePin(relay_2_GPIO_Port,relay_2_Pin,GPIO_PIN_SET)
#define R_3_O_LOW HAL_GPIO_WritePin(relay_3_GPIO_Port,relay_3_Pin,GPIO_PIN_SET)
#define R_4_O_LOW HAL_GPIO_WritePin(relay_4_GPIO_Port,relay_4_Pin,GPIO_PIN_SET)
#define R_5_O_LOW HAL_GPIO_WritePin(relay_5_GPIO_Port,relay_5_Pin,GPIO_PIN_SET)
#define R_6_O_LOW HAL_GPIO_WritePin(relay_6_GPIO_Port,relay_6_Pin,GPIO_PIN_SET)
#define R_7_O_LOW HAL_GPIO_WritePin(relay_7_GPIO_Port,relay_7_Pin,GPIO_PIN_SET)
#define R_8_O_LOW HAL_GPIO_WritePin(relay_8_GPIO_Port,relay_8_Pin,GPIO_PIN_SET)

void IO_init(void);
void IO_readInputSwitchs(void);
void IO_writeOutputRelay(void);
void IO_testInputOutput(void);
IO_interface IO_getInputOutputsVal(void);
uint8_t IO_inputsBitsPackageToByte(IO_interface interface);
void IO_outputByteToBitsPackage(uint8_t byte);

#endif /* INPUTOUTPUTINTERFACE_H_ */
