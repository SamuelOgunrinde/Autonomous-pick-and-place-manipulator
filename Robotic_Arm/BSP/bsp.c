#include "bsp.h"
#include "qpc.h"

void SysTick_Handler(void) {
	QF_TICK_X(0U, (void *)0); /* process all QP/C time events */
}

void BSP_Init(void) {
	MX_GPIO_Init();
	MX_TIM2_Init();
	MX_TIM3_Init();
}

void BSP_ledGreenOn(void) {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET);
}

void BSP_ledGreenOff(void) {
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET);
}

void BSP_toggleLed(void) {
	HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_5);
}


void QF_onStartup(void) {
    SysTick_Config(SystemCoreClock / BSP_TICKS_PER_SEC);
}

void QF_onCleanup(void) {
}

void QV_onIdle(void) {
#ifdef NDEBUG
    /* Put the CPU and peripherals to the low-power mode.
    * you might need to customize the clock management for your application,
    * see the datasheet for your particular Cortex-M MCU.
    */
    QV_CPU_SLEEP();  /* atomically go to sleep and enable interrupts */
#else
    QF_INT_ENABLE(); /* just enable interrupts */
#endif
}

void Q_onError(char const *module, int_t id){
	(void)module;
	(void)id;

	while(1){

	}
}
