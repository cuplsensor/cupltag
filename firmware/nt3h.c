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

/**
 * @file nt3h.c
 * @author Malcolm Mackay
 *
 * @brief A driver for the NXP NT3H2111 NFC EEPROM
 *
 * Reads and writes registers and memory on the NT3H2111 EEPROM using I2C.
 *
 * Datasheet: https://www.nxp.com/docs/en/data-sheet/NT3H2111_2211.pdf
 *
 */

#include "nt3h.h"
#include <string.h>
#include "i2c.h"
#include <stdbool.h>
#include "defs.h"

unsigned char rxData[16] = {0};
//unsigned char txData[16] = {0};
//unsigned char chunk1Data[50] = {0};
//unsigned char chunk2Data[50] = {0};

#define DEVADDR     0x55 // Careful. Was 0x55. Got overwritten.

#define PAGE0       0

#define CC_REGADDR  0x0C

#define BLOCKSIZE  16


/*! \brief Recreate a problem where the NT3H2211 I2C EEPROM
 *  is assigned the wrong I2C_Slave address. Unfortunately,
 *  this address is both mutable and located in the very
 *  first byte of memory. When power is running low, an
 *  unwanted transaction can occur that sets this to 0.
 */
void nt3h_init_wrongaddress(void)
{
    volatile int ccval = 0;
    i2c_read_block(DEVADDR, PAGE0, BLOCKSIZE, rxData, 0xFF);
    // Set the wrong device address.
    rxData[0] = 90 << 1;
    // Write capability container.
    rxData[12] = 0xE1;
    rxData[13] = 0x10;
    rxData[14] = 0x6D;
    i2c_write_block(DEVADDR, PAGE0, BLOCKSIZE, rxData);
}

/*! \brief Check that the NT3H2211 is at DEVADDR, where
 *  it is expected to be. If it is not, write to the address
 *  field to change its I2C address to DEVADDR. This means all
 *  subsequent transactions will work.
 *  Returns 1 if the Capability Container is not as expected.
 *  This can happen from the factory of if a text record has been written to the tag for configuration.
 */
int nt3h_check_address(void)
{
    int success = 0;
    int slaveaddr;
    int updatecc = 0;
    i2c_init();
    success = i2c_read_block(DEVADDR, 0, BLOCKSIZE, rxData, 0xFF);
    if (success < 0) {
        // Scan all I2C addresses for the NT3H2111 tag.
        for (slaveaddr=0; slaveaddr<=127; slaveaddr++) {
            success = i2c_read_block(slaveaddr, PAGE0, BLOCKSIZE, rxData, 0xFF);
            if ((success>=0) && (slaveaddr != 0x40))
            {
                // If a response has been received and it is not from 0x40 (HDC2010)
                // correct the device address and break out of the loop.
                rxData[0] = DEVADDR << 1;
                // Write capability container.
                rxData[12] = 0xE1;
                rxData[13] = 0x10;
                rxData[14] = 0x6D;
                i2c_write_block(slaveaddr, PAGE0, BLOCKSIZE, rxData);
                break;
            }
        }
    } else {
        if ((rxData[12] != 0xE1) || (rxData[13] != 0x10) || (rxData[14] != 0x6D)) {
            updatecc = 1;
        }
    }

    return updatecc;
}

void nt3h_update_cc(void) {
    i2c_read_block(DEVADDR, 0, BLOCKSIZE, rxData, 0xFF);
    rxData[0] = DEVADDR << 1;
    rxData[12] = 0xE1;
    rxData[13] = 0x10;
    rxData[14] = 0x6D;
    i2c_write_block(DEVADDR, PAGE0, BLOCKSIZE, rxData);
    while(nt3h_eepromwritedone() != 0);
}

/*! \brief Write a 16-byte block of the NT3H2211 I2C EEPROM.
 *  \param eepromBlock index of the block to write
 *  \param blkdata 16-byte array containing data to write
 */
int nt3h_writetag(int eepromBlock, char * blkdata)
{
    // Do not overwrite the serial number and slave address.
    return i2c_write_block(DEVADDR, eepromBlock, BLOCKSIZE, blkdata);
}

/*! \brief Read a 16-byte block from the NT3H2211 I2C EEPROM.
 *  \param eepromBlock index of the block to read.
 *  \param blkdata 16-byte array into which the EEPROM block contents will be stored.
 */
int nt3h_readtag(int eepromBlock, char * blkdata)
{
    i2c_read_block(DEVADDR, eepromBlock, BLOCKSIZE, blkdata, 0xFF);

    return 0;
}

volatile int nsreg2, nsreg3;

void nt3h_clearlock(void)
{
    nsreg2 = i2c_readreg(DEVADDR, 0xFE, 6);

    // Mask bit 6. Clear data
    unsigned char txData[3] = {6, 0x40, 0};

    i2c_write_block(DEVADDR, 0xFE, 3, txData);

    nsreg3 = i2c_readreg(DEVADDR, 0xFE, 6);
}

int nt3h_eepromwritedone(void)
{
    int nsreg = i2c_readreg(DEVADDR, 0xFE, 6);

    return (nsreg & 0x02);
}
