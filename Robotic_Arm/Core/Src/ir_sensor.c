#include "ir_sensor.h"

// Start 38kHz IR transmission
void IR_Init(void){
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
}

// Stop 38kHz IR transmission
void IR_Stop(void){
	HAL_TIM_PWM_Stop(&htim3, TIM_CHANNEL_3);
}

// If IR is detected
uint8_t IR_Detected(void) {
	if(HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_1) == GPIO_PIN_RESET) {
		return 1;
	}
	return 0;
}

void IR_Update(void){
	if(IR_Detected()) {
		//LED ON
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
	}
	else {
		//LED OFF
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
	}
	HAL_Delay(10);
}
