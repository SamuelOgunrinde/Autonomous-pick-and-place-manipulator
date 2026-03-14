// This file contains all the function prototypes for the ir_sensor.c file

#ifndef IR_SENSOR_H
#define IR_SENSOR_H

#include "main.h"
#include "tim.h"
#include "stm32f1xx_hal.h"

void IR_Init(void);
void IR_Stop(void);
uint8_t IR_Detected(void);

#endif
