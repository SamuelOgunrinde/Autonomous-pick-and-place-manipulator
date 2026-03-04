#include "servo.h"

#define SERVO_COUNT 4

static const uint32_t servo_channels[SERVO_COUNT] = {
		TIM_CHANNEL_1,
		TIM_CHANNEL_2,
		TIM_CHANNEL_3,
		TIM_CHANNEL_4
};

// Array to store desired angles for all servos
uint8_t desired_angles[SERVO_COUNT] = {0, 0, 0, 0};

// Start all PWM channels
void Servo_Init(void) {
	for (uint8_t i = 0; i < SERVO_COUNT; i++) {
		HAL_TIM_PWM_Start(&htim2, servo_channels[i]);
	}
}

//
void Servo_SetAngle(uint8_t servo_id, uint8_t angle) {

	if (servo_id >= SERVO_COUNT) return;

	if (angle < 0) angle = 0;

	if (angle > 180) angle = 180;

	desired_angles[servo_id] = angle;
}

void Servo_Update(void) {
	// Loop through all servos and set their angles
	for (uint8_t i = 0; i < SERVO_COUNT; i++)
	{
		// angle to pulse plus 0.5ms offset
		uint32_t pulse_us = 500 + ((uint32_t)desired_angles[i] * 2000) / 180;

		__HAL_TIM_SET_COMPARE(&htim2, servo_channels[i], pulse_us);
	}
}

