#include <stdint.h>
#include "mma8451.h"
#include "mma8451_i2c.h"
#include "stm32l1xx_i2c.h"

void MMA8451_Init (void) {
	  MMA8451_StandBy();
	  /*
	  **  Configure sensor for:
	  **    - Sleep Mode Poll Rate of 50Hz (20ms)
	  **    - System Output Data Rate of 200Hz (5ms)
	  **    - Full Scale of +/-2g
	  */
	  I2C_RegWrite(MMA8451ADD, CTRL_REG1, ASLP_RATE_160MS+DATA_RATE_5MS);
	  I2C_RegWrite(MMA8451ADD, XYZ_DATA_CFG_REG, FULL_SCALE_2G);
	  MMA8451_Active();
}

void MMA8451_Active (void) {
	I2C_RegWrite(MMA8451ADD, CTRL_REG1, (I2C_RegRead(MMA8451ADD, CTRL_REG1) | ACTIVE_MASK));
}

void MMA8451_StandBy (void) {
	  uint8_t n;
	  /*
	  **  Read current value of System Control 1 Register.
	  **  Put sensor into Standby Mode.
	  **  Return with previous value of System Control 1 Register.
	  */
	  n = I2C_RegRead(MMA8451ADD, CTRL_REG1);
	  I2C_RegWrite(MMA8451ADD, CTRL_REG1, n & ~ACTIVE_MASK);
}
