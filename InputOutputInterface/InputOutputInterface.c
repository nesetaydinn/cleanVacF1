/*
 * InputOutputInterface.c
 *
 *  Created on: Apr 2, 2021
 *      Author: neset
 */
#include "InputOutputInterface.h"
IO_interface inputOutputs;


void IO_init(void){
	inputOutputs.switch_1=false;
	inputOutputs.switch_2=false;
	inputOutputs.switch_3=false;
	inputOutputs.switch_4=false;
	inputOutputs.switch_5=false;
	inputOutputs.switch_6=false;
	inputOutputs.switch_7=false;
	inputOutputs.switch_8=false;

	inputOutputs.relay_1=false;
	inputOutputs.relay_2=false;
	inputOutputs.relay_3=false;
	inputOutputs.relay_4=false;
	inputOutputs.relay_5=false;
	inputOutputs.relay_6=false;
	inputOutputs.relay_7=false;
	inputOutputs.relay_8=false;
}

void IO_readInputSwitchs(void){
	inputOutputs.switch_1=!SW_I_1;
	inputOutputs.switch_2=!SW_I_2;
	inputOutputs.switch_3=!SW_I_3;
	inputOutputs.switch_4=!SW_I_4;
	inputOutputs.switch_5=!SW_I_5;
	inputOutputs.switch_6=!SW_I_6;
	inputOutputs.switch_7=!SW_I_7;
	inputOutputs.switch_8=!SW_I_8;
}
void IO_writeOutputRelay(void){
	if(inputOutputs.relay_1)R_1_O_HIGH; else R_1_O_LOW;
	if(inputOutputs.relay_2)R_2_O_HIGH; else R_2_O_LOW;
	if(inputOutputs.relay_3)R_3_O_HIGH; else R_3_O_LOW;
	if(inputOutputs.relay_4)R_4_O_HIGH; else R_4_O_LOW;
	if(inputOutputs.relay_5)R_5_O_HIGH; else R_5_O_LOW;
	if(inputOutputs.relay_6)R_6_O_HIGH; else R_6_O_LOW;
	if(inputOutputs.relay_7)R_7_O_HIGH; else R_7_O_LOW;
	if(inputOutputs.relay_8)R_8_O_HIGH; else R_8_O_LOW;
}
uint8_t testBtye[1];
void IO_testInputOutput(void){
/*	if(inputOutputs.switch_1)R_1_O_HIGH; else R_1_O_LOW;
	if(inputOutputs.switch_2)R_2_O_HIGH; else R_2_O_LOW;
	if(inputOutputs.switch_3)R_3_O_HIGH; else R_3_O_LOW;
	if(inputOutputs.switch_4)R_4_O_HIGH; else R_4_O_LOW;
	if(inputOutputs.switch_5)R_5_O_HIGH; else R_5_O_LOW;
	if(inputOutputs.switch_6)R_6_O_HIGH; else R_6_O_LOW;
	if(inputOutputs.switch_7)R_7_O_HIGH; else R_7_O_LOW;
	if(inputOutputs.switch_8)R_8_O_HIGH; else R_8_O_LOW;*/

	testBtye[0]=IO_inputsBitsPackageToByte(inputOutputs);
	IO_outputByteToBitsPackage(testBtye[0]);

}

uint8_t IO_inputsBitsPackageToByte(IO_interface inputBits){
	uint8_t bitPackage;
	bitPackage = (inputBits.switch_8 << 7) | (inputBits.switch_7 << 6) |
	(inputBits.switch_6 << 5) | (inputBits.switch_5 << 4) |
	(inputBits.switch_4 << 3) | (inputBits.switch_3 << 2) |
    (inputBits.switch_2 << 1) | (inputBits.switch_1);
	return bitPackage;
}

void IO_outputByteToBitsPackage(uint8_t byte){
	inputOutputs.relay_1=(bool)(byte & IO_BIT_0);
	inputOutputs.relay_2=(bool)(byte & IO_BIT_1);
	inputOutputs.relay_3=(bool)(byte & IO_BIT_2);
	inputOutputs.relay_4=(bool)(byte & IO_BIT_3);
	inputOutputs.relay_5=(bool)(byte & IO_BIT_4);
	inputOutputs.relay_6=(bool)(byte & IO_BIT_5);
	inputOutputs.relay_7=(bool)(byte & IO_BIT_6);
	inputOutputs.relay_8=(bool)(byte & IO_BIT_7);
}

IO_interface IO_getInputOutputsVal(void){return inputOutputs;}
