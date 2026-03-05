#include "bsp.h"


void BSP_Init(void) {
	MX_GPIO_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
}
