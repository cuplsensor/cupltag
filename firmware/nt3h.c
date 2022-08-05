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
 * @brief A driver for the <a href="https://www.nxp.com/docs/en/data-sheet/NT3H2111_2211.pdf">NXP NT3H2111</a> NFC EEPROM.
 *
 * Reads and writes memory on the NT3H2111 EEPROM using I2C. The memory
 * is organised into 16-byte blocks from the I2C perspective.
 *
 * BLOCK0 contains configuration data such as the Capability Container.
 *
 */

#include "nt3h.h"
#include <string.h>
#include "i2c.h"
#include <stdbool.h>
#include "defs.h"

#define ADDR_7b_MIN             0           /*!< Minimum value of a 7-bit device address. */
#define ADDR_7b_MAX             127         /*!< Maximum value of a 7-bit device address. */
#define WRONG_DEVADDR           90          /*!< A wrong NFC EEPROM device address. */
#define BLOCK0                  0           /*!< Address of the first memory block. */
#define BLOCK_SESSION           0xFE        /*!< Address of the memory block used for session registers. */
#define DEVADDR_OFFSET          0x00        /*!< Byte-offset of the Device Address within the first memory block. */
#define CC_OFFSET               0x0C        /*!< Byte-offset of the Capability Container within the first memory block. */
#define NSREG_OFFSET            0x06        /*!< Byte-offset of the NS_REG session register. */
#define NSREG_EEPROM_WR_BUSY    0x02        /*!< Select the EEPROM Write Busy bit from NS_REG. Set to 1'b1 when a write is in progress. */
#define BLOCKSIZE               0x10        /*!< Block size in bytes. */
#define CC0_MAGIC               0xE1        /*!< Byte 0 of the Capability Container. Magic number. */
#define CC1_VER                 0x10        /*!< Byte 1 of the Capability Container. Version. */
#define CC2_NBYTESBY8           0x6D        /*!< Byte 2 of the Capability Container. The number of bytes in memory divided by 8. */

unsigned char rxData[BLOCKSIZE] = {0}; /*!< Holds the content of one block. */
volatile int nsreg2, nsreg3;


/*!
 *  @brief Deliberately assign the wrong device address to the NFC EEPROM.
 *
 *  This can be called to re-create a problem where the NFC EEPROM ends up with the wrong device address.
 *
 *  Unfortunately, the device address is mutable and located in byte 0 of memory.
 *
 *  When power is running low, an unwanted transaction can occur (a series of zeroes) that clears the device address.
 */
void nt3h_init_wrongaddress(void)
{
    // Read the first block into rxData.
    i2c_read_block(NT3H_DEVADDR, BLOCK0, BLOCKSIZE, rxData, 0xFF);
    // Modify the device address with the block.
    rxData[DEVADDR_OFFSET] = WRONG_DEVADDR << 1;
    // Write capability container.
    rxData[CC_OFFSET]   = CC0_MAGIC;
    rxData[CC_OFFSET+1] = CC1_VER;
    rxData[CC_OFFSET+2] = CC2_NBYTESBY8;
    i2c_write_block(NT3H_DEVADDR, BLOCK0, BLOCKSIZE, rxData);
}

/*!
 *  @brief Ensure that the NFC EEPROM is at the expected device address.
 *
 *  Check that the NFC EEPROM responds. If it does not, find it on the bus
 *  by scanning all available addresses. Once found, correct the address so there is no
 *  need to scan next time.
 *
 *  Check that the capability container is correct. If an external device has
 *  written to the tag, it may have been altered. It is not safe to write the capability
 *  container just-in-case. This will wear out the EEPROM block.
 *
 *  @return 0 if OK or 1 if the Capability Container is not correct.
 */
int nt3h_check_address(void)
{
    int readok = 0;     /*!< 0 or positive if the I2C read is successful. Otherwise non-zero. */
    int slaveaddr;      /*!< I2C device address. */
    int updatecc = 0;   /*!< Set to 1 if the Capability Container has changed. */

    // Initialise the I2C peripheral.
    i2c_init();

    // Attempt to read the first block into rxData.
    readok = i2c_read_block(NT3H_DEVADDR, BLOCK0, BLOCKSIZE, rxData, 0xFF);

    if (readok < 0) {
        /*  The I2C read has failed.
         *  Assume the NFC EEPROM is configured to use the wrong address.
         *  Scan for it at all possible device addresses. */
        for (slaveaddr=ADDR_7b_MIN; slaveaddr<=ADDR_7b_MAX; slaveaddr++) {
            // Attempt to read the first block into rxData.
            readok = i2c_read_block(slaveaddr, BLOCK0, BLOCKSIZE, rxData, 0xFF);

            if ((readok>=0) && (slaveaddr != HDC_DEVADDR))
            {
                /* A response has been received. It is not from any other device on the bus.
                 * The NFC EEPROM has been found!
                 * Correct its device address and break out of the loop. */
                rxData[DEVADDR_OFFSET] = NT3H_DEVADDR << 1;
                // Write the capability container in case it is incorrect.
                rxData[CC_OFFSET]   = CC0_MAGIC;
                rxData[CC_OFFSET+1] = CC1_VER;
                rxData[CC_OFFSET+2] = CC2_NBYTESBY8;
                // Write to the first block.
                i2c_write_block(slaveaddr, BLOCK0, BLOCKSIZE, rxData);
                break;
            }
        }
    } else {
        /* The I2C read has succeeded. Check that the capability container is correct. */
        if ((rxData[CC_OFFSET] != CC0_MAGIC) || (rxData[CC_OFFSET+1] != CC1_VER) || (rxData[CC_OFFSET+2] != CC2_NBYTESBY8)) {
            updatecc = 1;
        }
    }

    return updatecc;
}

/*!
 *  @brief Write the Capability Container.
 *
 *  Sets the CC to the values required for long NDEF messages.
 *
 *  The CC is 0 from the factory. It can also be altered by a phone
 *  that writes to the tag.
 *
 *  This is a blocking operation. It will not return until the write is done.
 */
void nt3h_update_cc(void) {
    i2c_read_block(NT3H_DEVADDR, 0, BLOCKSIZE, rxData, 0xFF);
    rxData[DEVADDR_OFFSET] = NT3H_DEVADDR << 1;
    rxData[CC_OFFSET]      = CC0_MAGIC;
    rxData[CC_OFFSET+1]    = CC1_VER;
    rxData[CC_OFFSET+2]    = CC2_NBYTESBY8;
    i2c_write_block(NT3H_DEVADDR, BLOCK0, BLOCKSIZE, rxData);

    // Wait for the block write to complete.
    while(nt3h_eepromwritedone() != 0);
}

/*!
 *  @brief Write one block to the NFC EEPROM.
 *  @param[in] eepromBlock index of the block to write
 *  @param[in] blkdata 16-byte array containing data to write
 *  @return A negative value if the write has failed.
 */
int nt3h_writetag(int eepromBlock, char * blkdata)
{
    return i2c_write_block(NT3H_DEVADDR, eepromBlock, BLOCKSIZE, blkdata);
}

/*!
 *  @brief Read one 16-byte block from the NFC EEPROM.
 *  @param[in] eepromBlock index of the block to read.
 *  @param[out] blkdata 16-byte array into which the EEPROM block contents will be read.
 */
int nt3h_readtag(int eepromBlock, char * blkdata)
{
    i2c_read_block(NT3H_DEVADDR, eepromBlock, BLOCKSIZE, blkdata, 0xFF);

    return 0;
}


/*!
 *  @brief Intended to clear the I2C_LOCKED bit in the NS_REG session register.
 *
 *  BUG. Does not affect NS_REG. Instead writes 0x06 to NC_REG, 0x40 to LAST_NDEF_BLOCK and 0x00 to SRAM_MIRROR_BLOCK.
 *
 *  Does not do any harm, because the device is powered down immediately afterwards anyway. Remove in future.
 */
void nt3h_clearlock(void)
{
    nsreg2 = i2c_readreg(NT3H_DEVADDR, BLOCK_SESSION, NSREG_OFFSET);
    unsigned char txData[3] = {0x06, 0x40, 0};
    i2c_write_block(NT3H_DEVADDR, BLOCK_SESSION, 3, txData);
    nsreg3 = i2c_readreg(NT3H_DEVADDR, BLOCK_SESSION, NSREG_OFFSET);
}

/*!
 *  @brief Check if an EEPROM write is in-progress.
 *  @return 2 when a write is in progress. Otherwise 0 when EEPROM access is possible.
 */
int nt3h_eepromwritedone(void)
{
    int nsreg = i2c_readreg(NT3H_DEVADDR, BLOCK_SESSION, NSREG_OFFSET);

    return (nsreg & NSREG_EEPROM_WR_BUSY);
}
