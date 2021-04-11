/*
 * taskManagerInterface.h
 *
 *  Created on: Feb 25, 2021
 *      Author: neset
 */

#ifndef TASKMANAGERINTERFACE_H_
#define TASKMANAGERINTERFACE_H_

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "main.h"


void tasks_init(void);


uint16_t valuesMap(uint16_t inValue, uint16_t inMin, uint16_t inMax, uint16_t outMin, uint16_t outMax);
int16_t valuesMapInt(int16_t inValue, int16_t inMin, int16_t inMax, int16_t outMin, int16_t outMax);
uint16_t valuesMapWFloat(float inValue, float inMin, float inMax, uint16_t outMin, uint16_t outMax);
int16_t valuesMapFloat(float inValue, float inMin, float inMax, int16_t outMin, int16_t outMax);
float valuesMapforFloat(uint16_t inValue, uint16_t inMin, uint16_t inMax, float outMin, float outMax);

#endif /* TASKMANAGERINTERFACE_H_ */
