#ifndef _MMA8451_I2C_H
#define _MMA8451_I2C_H

/*
 * @brief Operación de lectura de un registro del acelerómetro
 * @param Address dirección del esclavo
 * @param RegAddress dirección del registro a leer
 */
uint8_t I2C_RegRead(uint8_t Address, uint8_t RegAddress);

/*
 * @brief Operación de escritura de un registro del acelerómetro
 * @param Address dirección del esclavo
 * @param RegAddress dirección del registro a leer
 * @param RegValue valor a escribir en el registro
 */
void I2C_RegWrite(uint8_t Address, uint8_t RegAddress, uint8_t RegValue);

#endif /* _MMA8451_I2C_H */

