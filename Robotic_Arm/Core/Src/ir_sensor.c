#include "ir_sensor.h"
#include "bsp.h"
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
