#include <stdint.h>
#include "bsp/bsp.h"
#include "driver_mma8451/mma8451.h"
#include "driver_mma8451/mma8451_i2c.h"

int main(void) {

	uint8_t RegValue=0;

	bsp_init();

	while (1) {
//		if (!sw_getState(0)) {
//			MMA8451_Init();
//		}
//		if (!sw_getState(1)) {
//			RegValue = I2C_RegRead(MMA8451ADD, CTRL_REG1);
//			RegValue = I2C_RegRead(MMA8451ADD, XYZ_DATA_CFG_REG);
//		}
	}
}


void APP_ISR_1ms (void) {
	static uint16_t count_1s = 1000; // static: es una variable que se declara una sola vez, se fija en 0, y luego cada vez que entramos en la función, count conserva el valor anterior.
	count_1s--;
	if (!count_1s) {
//		led_toggle(0);
//		led_toggle(1);
		count_1s = 1000;
	}
}

void APP_ISR_AF (void) {
	led_on(0);
	led_on(1);
	while(1){

	}
}

void APP_ISR_EP (void) {
	led_toggle(0);
	led_toggle(1);
}
