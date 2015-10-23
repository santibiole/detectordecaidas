#include <stdint.h>
#include "bsp.h"
#include "stm32l1xx_rcc.h"		// Para configurar el (Reset and clock controller)
#include "stm32l1xx_gpio.h"		// Perifericos de E/S

void SIM908_On (void) {
	GPIO_SetBits(GPIOB, GPIO_Pin_12);
	bsp_delay_ms(1500);
	GPIO_ResetBits(GPIOB, GPIO_Pin_12);
}

void SIM908_Off (void) {
	GPIO_SetBits(GPIOB, GPIO_Pin_12);
	bsp_delay_ms(1500);
	GPIO_ResetBits(GPIOB, GPIO_Pin_12);
}
