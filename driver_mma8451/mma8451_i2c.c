#include <stdint.h>
#include "stm32l1xx_i2c.h"

/*
 * @brief Operación de lectura de un registro del acelerómetro
 * @param Address dirección del esclavo
 * @param RegAddress dirección del registro a leer
 */
uint8_t I2C_RegRead(uint8_t Address, uint8_t RegAddress) {
	uint8_t value;
	while (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));								// Check BUSY Flag

	I2C_GenerateSTART(I2C1, ENABLE);											// START
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));				// Check EV5

	I2C_Send7bitAddress(I2C1, Address, I2C_Direction_Transmitter); 				// Send ADDRESS and R/W bit
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));	// Check EV6 (Master Transmitter)

	I2C_SendData(I2C1, RegAddress); 											// Send REGADDRESS
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));			// Check EV8_2

	I2C_GenerateSTART(I2C1, ENABLE);											// RESTART
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));				// Check EV5

	I2C_Send7bitAddress(I2C1, Address, I2C_Direction_Receiver); 				// Send ADDRESS and R/W bit
	I2C_NACKPositionConfig(I2C1, I2C_NACKPosition_Current);						// Config NACK condition
	I2C_AcknowledgeConfig(I2C1, DISABLE);										// Disable ACK
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED));		// Check EV6
	I2C_GenerateSTOP(I2C1, ENABLE); 											// STOP

	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_RECEIVED));				// Check EV7
	value = I2C_ReceiveData(I2C1); 												// Return DATA from DR

	return value;
}

/*
 * @brief Operación de escritura de un registro del acelerómetro
 * @param Address dirección del esclavo
 * @param RegAddress dirección del registro a leer
 * @param RegValue valor a escribir en el registro
 */
void I2C_RegWrite(uint8_t Address, uint8_t RegAddress, uint8_t RegValue) {
	while (I2C_GetFlagStatus(I2C1, I2C_FLAG_BUSY));

	I2C_GenerateSTART(I2C1, ENABLE); // Send Start
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_MODE_SELECT));

	I2C_Send7bitAddress(I2C1, Address, I2C_Direction_Transmitter); // Send Address
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED));

	I2C_SendData(I2C1, RegAddress); // Send Register Address
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));

	I2C_SendData(I2C1, RegValue); // Send Register Value
	I2C_GenerateSTOP(I2C1, ENABLE); // Send Stop
	while (!I2C_CheckEvent(I2C1, I2C_EVENT_MASTER_BYTE_TRANSMITTED));
}
