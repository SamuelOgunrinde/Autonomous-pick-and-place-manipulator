// This file contains all the function prototypes for the bsp.c file
#ifndef BSP_H
#define BSP_H

#include "main.h"
#include "tim.h"
#include "gpio.h"
#include "servo.h"
#include "ir_sensor.h"
#include "qpc.h"

#define BSP_TICKS_PER_SEC 1000U

void BSP_Init(void);
void BSP_ledGreenOn();
void BSP_ledGreenOff();
void BSP_toggleLed(void);
void SysTick_Handler(void);
void QV_onIdle(void);

enum EventSignals {
	OBJECT_DETECTED_SIG = Q_USER_SIG,
    MOVE_SERVO_SIG,
    SERVO_DONE_SIG,
	POLL_SIG
};

/* Active objects... */
extern QActive *AO_RobotController;
extern QActive *AO_ServoController;
extern QActive *AO_IRSensor;

#endif
