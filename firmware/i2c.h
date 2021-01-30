/*  This MSP430FR2155 program collects data from an HDC2010 sensor and
 *  passes it to the encoder part of cuplcodec.
 *
 *  https://github.com/cuplsensor/cupltag
 *
 *  Original Author: Malcolm Mackay
 *  Email: malcolm@plotsensor.com
 *
 *  Copyright (C) 2021. Plotsensor Ltd.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
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
