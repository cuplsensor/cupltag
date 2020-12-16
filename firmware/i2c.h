/*
 * i2c.h
 *
 *  Created on: 5 Aug 2017
 *      Author: mmackay
 */

#ifndef I2C_H_
#define I2C_H_

#include "driverlib.h"

void i2c_init();
void i2c_off();


int i2c_readreg(uint8_t slaveAddr, uint8_t mema, uint8_t rega);
int i2c_read_block(uint8_t slaveAddr, uint8_t regOffset, uint8_t bytesToRead, uint8_t * rxData, uint8_t rega);
int i2c_write_block(uint8_t slaveAddr, uint8_t regOffset, uint8_t bytesToWrite, uint8_t * txData);

uint8_t i2c_read8(uint8_t slaveAddr, uint8_t regOffset);
uint16_t i2c_read16(uint8_t slaveAddr, uint8_t regOffset);
uint16_t i2c_read32(uint8_t slaveAddr, uint8_t regOffset, uint16_t * temp, uint16_t * hum);
uint8_t i2c_write8(uint8_t slaveAddr, uint8_t regOffset, uint8_t writeData);

#endif /* I2C_H_ */
