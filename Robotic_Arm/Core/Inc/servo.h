// This file contains all the function prototypes for the servo.c file

#ifndef SERVO_H
#define SERVO_H

#include "main.h"
#include "tim.h"

void Servo_Init(void);
void Servo_SetAngle(uint8_t servo_id, uint8_t angle);
void Servo_Update(void);

#endif

