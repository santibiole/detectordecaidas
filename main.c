#include <stdint.h>
#include "bsp.h"
#include "mma8451.h"
#include "mma8451_i2c.h"
#include "sim908.h"

int main(void) {

	uint8_t RegValue=0;
	bsp_init();
	MMA8451_Init();
	SIM908_On();

	// Configuring Linear Freefall Detection
//	MMA8451_StandBy();
//	I2C_RegWrite(MMA8451ADD, CTRL_REG1, (I2C_RegRead(MMA8451ADD, CTRL_REG1) & ~FREAD_MASK));
//	I2C_RegWrite(MMA8451ADD, CTRL_REG3, PP_OD_MASK);
//	I2C_RegWrite(MMA8451ADD, CTRL_REG4, INT_EN_DRDY_MASK);
//	I2C_RegWrite(MMA8451ADD, CTRL_REG5, INT_CFG_DRDY_MASK);
//	MMA8451_Active();

	while (1) {

	}
}


void APP_ISR_MS (void) {
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

void APP_ISR_ACC(void) {
//	uint8_t XL = 0, XH = 0, YL = 0, YH = 0, ZL = 0, ZH = 0;
//
//	led_toggle(0);
//	led_toggle(1);
//
//	XL = I2C_RegRead(MMA8451ADD, OUT_X_LSB_REG);
//	XH = I2C_RegRead(MMA8451ADD, OUT_X_MSB_REG);
//	YL = I2C_RegRead(MMA8451ADD, OUT_Y_LSB_REG);
//	YH = I2C_RegRead(MMA8451ADD, OUT_Y_MSB_REG);
//	ZL = I2C_RegRead(MMA8451ADD, OUT_Z_LSB_REG);
//	ZH = I2C_RegRead(MMA8451ADD, OUT_Z_MSB_REG);
}

void APP_ISR_EM (void) {
	led_on(0);
	led_on(1);
	bsp_delay_ms(5000);
	led_off(0);
	led_off(1);
}

void APP_ISR_EP (void) {
	led_on(0);
	led_on(1);
	bsp_delay_ms(5000);
	led_off(0);
	led_off(1);
}
void APP_ISR_uartrx (void) {
	led_toggle(0);
	led_toggle(1);
}
