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
 * @file i2c.c
 * @author Malcolm Mackay
 *
 * @brief Communicates with devices on an I2C bus.
 *
 * Configures the EUSCI peripheral as an I2C master. Up to 16 bytes can be written to or read from a memory address on the I2C slave.
 *
 * Some devices embed up to 16 registers within each memory address. There is a function for reading one register only.
 *
 */

#include "i2c.h"


volatile uint8_t buffer[16] = {0};       /*!< Read or write buffer. This is declared volatile because it is accessed from an ISR. */
volatile uint8_t bytesLength = 0;        /*!< Transaction length. This is declared volatile because it is read from an ISR. */
volatile uint8_t gbl_regOffset = 0;      /*!< Memory address. Volatile because it is read from an ISR. */
volatile bool restartTx = false;
volatile bool nackFlag = false;
volatile bool stopFlag = false;
volatile bool restartRx = false;

#define EUSCI_BASE  EUSCI_B0_BASE         /*!< Base address of the EUSCI peripheral. */

/*!
 * @brief Initialise the EUSCI peripheral and I/O pins for I2C.
 *
 * Weak pull-up resistors must be fitted to the I/O pins.
 *
 */
void i2c_init()
{
    // Configure P1.2 and P1.3 to connect to the EUSCI peripheral.
    GPIO_setAsPeripheralModuleFunctionInputPin(
            GPIO_PORT_P1,
            GPIO_PIN2 + GPIO_PIN3,
            GPIO_PRIMARY_MODULE_FUNCTION
    );

    // Initialize the EUSCI peripheral.
    EUSCI_B_I2C_initMasterParam param = {0};
    // Clock the peripheral from SMCLK (1 MHz)
    param.selectClockSource = EUSCI_B_I2C_CLOCKSOURCE_SMCLK;
    param.i2cClk = CS_getSMCLK();
    // Set data rate to 100 kHz.
    param.dataRate = 100000;
    // Disable automatic stop.
    param.byteCounterThreshold = 16;
    param.autoSTOPGeneration = EUSCI_B_I2C_NO_AUTO_STOP;
    // Write settings to the EUSCI module.
    EUSCI_B_I2C_initMaster(EUSCI_BASE, &param);
}

/*!
 * @brief Put the EUSCI module into reset. Enable pull-downs on the I/O pins.
 *
 * Floating pins waste power.
 */
void i2c_off() {
    EUSCI_B_I2C_disable(EUSCI_BASE);

    GPIO_setAsInputPinWithPullDownResistor(GPIO_PORT_P1, GPIO_PIN2 + GPIO_PIN3);
}

/*!
 * @brief Write one byte to the I2C device.
 *
 * Blocks until the write has completed.
 *
 * @param[in] sa slave address
 * @param[in] mema memory address
 * @param[in] txbyte byte to write
 */
uint8_t i2c_write8(uint8_t sa, uint8_t mema, uint8_t txbyte)
{
    while(i2c_write_block(sa, mema, 1, &txbyte)==0);

    return 0;
}

/*!
 * @brief Read one register on an I2C device.
 *
 * @param[in] sa slave address
 * @param[in] mema memory address
 * @param[in] rega register address
 *
 * @return one byte of register data.
 */
int i2c_readreg(uint8_t sa, uint8_t mema, uint8_t rega)
{
    uint8_t rxData = 0;

    i2c_read_block(sa, mema, 1, &rxData, rega);

    return rxData;
}

/*!
 * @brief Write N bytes to the I2C device.
 *
 * When NBYTES == 0: START | WRITE SA | MEMA | STOP
 * When NBYTES >= 1: START | WRITE SA | MEMA | TXDATA[0] | TXDATA[...] | TXDATA[NBYTES-1] | STOP
 *
 * @param[in] sa slave address.
 * @param[in] mema memory address.
 * @param[in] nbytes the number of bytes to write.
 * @param[in] txdata a pointer to the array of bytes to write.
 */
int i2c_write_block(uint8_t sa, uint8_t mema, uint8_t nbytes, uint8_t * txdata)
{
    int i;
    int success = 0;
    int retransmitCounter = 0;

    retransmitCounter = 0;

    bytesLength = nbytes;

    // Copy from the transmit array into the volatile buffer.
    for(i=0; i<bytesLength; i++)
    {
        buffer[i] = *(txdata + i);
    }

    //Disable the USCI module and clears the other bits of control register
    EUSCI_B_I2C_disable(EUSCI_BASE);

    //Configure Automatic STOP condition generation
    HWREG16(EUSCI_BASE + OFS_UCBxCTLW1) &= ~UCASTP_3;
    HWREG16(EUSCI_BASE + OFS_UCBxCTLW1) |= EUSCI_B_I2C_NO_AUTO_STOP;

    //Specify slave address
    EUSCI_B_I2C_setSlaveAddress(EUSCI_BASE, sa);

    //Set in transmit mode
    EUSCI_B_I2C_setMode(EUSCI_BASE, EUSCI_B_I2C_TRANSMIT_MODE);

    //Enable I2C Module to start operations
    EUSCI_B_I2C_enable(EUSCI_BASE);

    // Clear master transmit interrupt.
    EUSCI_B_I2C_clearInterrupt(EUSCI_BASE,
                               EUSCI_B_I2C_TRANSMIT_INTERRUPT0 + EUSCI_B_I2C_RECEIVE_INTERRUPT0 +
                               EUSCI_B_I2C_STOP_INTERRUPT +
                               EUSCI_B_I2C_NAK_INTERRUPT
                               );

    //Enable master Transmit interrupt
    EUSCI_B_I2C_enableInterrupt(EUSCI_BASE,
                                EUSCI_B_I2C_TRANSMIT_INTERRUPT0 +
                                EUSCI_B_I2C_STOP_INTERRUPT +
                                EUSCI_B_I2C_NAK_INTERRUPT
                                );


    restartTx = true;
    stopFlag = false;
    gbl_regOffset = mema;


    success = EUSCI_B_I2C_masterSendMultiByteStartWithTimeout(EUSCI_BASE, gbl_regOffset, 1000);
    __bis_SR_register(CPUOFF + GIE);    // Enter LPM0 w/ interrupt

    while((nackFlag == true) && (retransmitCounter++ < 100))
    {
        nackFlag = false;
        EUSCI_B_I2C_masterSendMultiByteStart(EUSCI_BASE, gbl_regOffset);
        __bis_SR_register(CPUOFF + GIE);    // Enter LPM0 w/ interrupt
    }

    while((stopFlag == false) && (retransmitCounter++ < 100))
    {
        __bis_SR_register(CPUOFF + GIE);    // Enter LPM0 w/ interrupt
    }

    restartTx = false;


    //Disable master transmit interrupt
    EUSCI_B_I2C_disableInterrupt(EUSCI_BASE,
                                 EUSCI_B_I2C_TRANSMIT_INTERRUPT0 +
                                 EUSCI_B_I2C_STOP_INTERRUPT +
                                 EUSCI_B_I2C_NAK_INTERRUPT
    );

    if (retransmitCounter++ > 100)
    {
        success = -1;
    }

    return success;

}

/*!
 * @brief Read N bytes from the I2C device.
 *
 * When a register address is specified:   START | WRITE SA | MEMA | REGA | STOP  | START   | READ SA | BYTE0    | BYTE ... | BYTE n-1 | STOP.
 * When no register address is specified:  START | WRITE SA | MEMA | STOP | START | READ SA | BYTE0   | BYTE ... | BYTE n-1 | STOP.
 *
 * @param[in] sa slave address.
 * @param[in] mema memory address.
 * @param[in] nbytes the number of bytes to read.
 * @param[out] rxdata a pointer to an array used to store read data. Must be at least nbytes long.
 * @param[in] rega register address. Set to 0xFF when a register read is not required.
 *
 * @returns -1 when the slave fails to respond, otherwise zero.
 */
int i2c_read_block(uint8_t sa, uint8_t mema, uint8_t nbytes, uint8_t * rxdata, uint8_t rega)
{
    int i;
    int retransmitCounter = 0;
    int bytesToWrite;
    int success;
    uint8_t txData = 0;

    // If a register address has been supplied, write this first.
    if (rega == 0xFF)
    {
        // Not reading from a register, so WRITE SA | MEMA | STOP
        bytesToWrite = 0;
    }
    else
    {
        // Reading from a register, so WRITE SA | MEMA | REGA | STOP
        bytesToWrite = 1;
        txData = rega;
    }

    // Start the write transaction and wait for it to finish.
    while(1) {
        success = i2c_write_block(sa, mema, bytesToWrite, &txData);
        if (success != 0) {
            break;
        }
    }

    if (success == -1) {
        return -1;
    }

    //Disable the USCI module and clears the other bits of control register
    EUSCI_B_I2C_disable(EUSCI_BASE);

    //Configure Automatic STOP condition generation
    HWREG16(EUSCI_BASE + OFS_UCBxCTLW1) &= ~UCASTP_3;
    HWREG16(EUSCI_BASE + OFS_UCBxCTLW1) |= EUSCI_B_I2C_SEND_STOP_AUTOMATICALLY_ON_BYTECOUNT_THRESHOLD;

    //Byte Count Threshold
    HWREG16(EUSCI_BASE + OFS_UCBxTBCNT) = nbytes;

    bytesLength = nbytes;

    // Set to receive mode
    EUSCI_B_I2C_setMode(EUSCI_BASE, EUSCI_B_I2C_RECEIVE_MODE);

    // Enable I2C Module to start operations
    EUSCI_B_I2C_enable(EUSCI_BASE);


    EUSCI_B_I2C_clearInterrupt(EUSCI_BASE,
                               EUSCI_B_I2C_RECEIVE_INTERRUPT0 + EUSCI_B_I2C_TRANSMIT_INTERRUPT0 +
                               EUSCI_B_I2C_NAK_INTERRUPT
    );

    //Enable the I2C Receive interrupts
    EUSCI_B_I2C_enableInterrupt(EUSCI_BASE,
                                EUSCI_B_I2C_RECEIVE_INTERRUPT0 +
                                EUSCI_B_I2C_NAK_INTERRUPT
    );

    __delay_cycles(100);

    // Send an I2C start condition
    EUSCI_B_I2C_masterReceiveStart(EUSCI_BASE);

    // Wait for the read transaction to complete
    __bis_SR_register(CPUOFF + GIE);    // Enter LPM0 w/ interrupt

    // Retry the read if it has failed with a 'no acknowledge'.
    while(nackFlag == true && (retransmitCounter++ < 200))
    {
        nackFlag = false;
        EUSCI_B_I2C_masterReceiveStart(EUSCI_BASE);
        __bis_SR_register(CPUOFF + GIE);    // Enter LPM0 w/ interrupt
    }

    // Disable the I2C Receive interrupts
    EUSCI_B_I2C_disableInterrupt(EUSCI_BASE,
                                 EUSCI_B_I2C_RECEIVE_INTERRUPT0 +
                                 EUSCI_B_I2C_NAK_INTERRUPT
    );

    // Copy from the volatile buffer into the read data array.
    for(i=0; i<bytesLength; i++)
    {
        *(rxdata + i) = buffer[i];
    }

    return success;

}

/*!
 * @brief Read one byte from memory on the I2C slave.
 *
 * START | WRITE SA | MEMA | STOP | START | READ SA | BYTE0 | STOP.
 *
 * @param[in] sa slave address.
 * @param[in] mema memory address.
 *
 * @returns one byte read from the I2C slave.
 */
uint8_t i2c_read8(uint8_t sa, uint8_t mema)
{
    uint8_t rxdata;

    i2c_read_block(sa, mema, 1, &rxdata, 0xFF);

    return rxdata;
}

/*!
 * @brief Read two bytes from memory on the I2C slave.
 *
 * START | WRITE SA | MEMA | STOP | START | READ SA | BYTE0 | BYTE1 | STOP.
 *
 * @param[in] sa slave address.
 * @param[in] mema memory address.
 *
 * @returns one little-endian 16-bit integer read from the I2C slave.
 */
uint16_t i2c_read16(uint8_t sa, uint8_t mema)
{
    uint8_t rxdata[2];
    uint16_t rxDataLsb;
    uint16_t rxDataMsb;


    i2c_read_block(sa, mema, 2, rxdata, 0xFF);

    rxDataLsb = rxdata[0];
    rxDataMsb = rxdata[1];

    return ((rxDataMsb << 8)&0xFF00) | (rxDataLsb & 0x00FF);
}

/*!
 * @brief Read two consecutive unsigned integers from the I2C slave.
 *
 * START | WRITE SA | MEMA | STOP | START | READ SA | BYTE0 | BYTE1 | BYTE2 | BYTE3 | STOP.
 *
 * @param[in] sa slave address.
 * @param[in] mema memory address.
 * @param[out] uint0 pointer to an address for storing the first little endian unsigned integer.
 * @param[out] uint1 pointer to an address for strong the second little endian unsigned integer.
 */
uint16_t i2c_read16x2(uint8_t sa, uint8_t mema, uint16_t * uint0, uint16_t * uint1)
{
    uint8_t rxdata[4];
    uint16_t u0Lsb, u1Lsb;
    uint16_t u0Msb, u1Msb;

    i2c_read_block(sa, mema, 4, rxdata, 0xFF);

    u0Lsb = rxdata[0];
    u0Msb = rxdata[1];
    u1Lsb = rxdata[2];
    u1Msb = rxdata[3];

    *uint0 = ((u0Msb << 8)&0xFF00) | (u0Lsb & 0x00FF);
    *uint1 = ((u1Msb << 8)&0xFF00) | (u1Lsb & 0x00FF);

    return 0;
}


#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=USCI_B0_VECTOR
__interrupt
#elif defined(__GNUC__)
__attribute__((interrupt(USCI_B0_VECTOR)))
#endif
void USCIB0_ISR(void)
{
    static uint8_t rxCount = 0;
    static uint8_t txCount = 0;
    switch(__even_in_range(UCB0IV,0x1E))
    {
    case 0x00: break;       // Vector 0: No interrupts break;
    case 0x02: break;       // Vector 2: ALIFG break;
    case 0x04:
        txCount = 0;
        rxCount = 0;
        nackFlag = true;
        __bic_SR_register_on_exit(CPUOFF);    // Exit LPM0
        break;     // Vector 4: NACKIFG break;
    case 0x06: break;       // Vector 6: STT IFG break;
    case 0x08:
        txCount = 0;
        stopFlag = true;
        __bic_SR_register_on_exit(CPUOFF);    // Exit LPM0
        break;       // Vector 8: STPIFG break;
    case 0x0a: break;       // Vector 10: RXIFG3 break;
    case 0x0c: break;       // Vector 14: TXIFG3 break;
    case 0x0e: break;       // Vector 16: RXIFG2 break;
    case 0x10: break;       // Vector 18: TXIFG2 break;
    case 0x12: break;       // Vector 20: RXIFG1 break;
    case 0x14: break;       // Vector 22: TXIFG1 break;
    case 0x16:                        // Get RX data
        buffer[rxCount++] = HWREG16(EUSCI_BASE + OFS_UCBxRXBUF);
        if(rxCount >= bytesLength)
        {
            rxCount = 0;
            __bic_SR_register_on_exit(CPUOFF);    // Exit LPM0
        }
        break;     // Vector 24: RXIFG0 break;
    case 0x18:
        if(txCount >= bytesLength)
        {
            HWREG16(EUSCI_BASE + OFS_UCBxCTLW0) |= UCTXSTP;
        }
        else
        {
            HWREG16(EUSCI_BASE + OFS_UCBxTXBUF) = buffer[txCount];
        }
        txCount++;
        break;       // Vector 26: TXIFG0 break;
    case 0x1a:
        break;           // Vector 28: BCNTIFG break;
    case 0x1c:
        break;       // Vector 30: clock low timeout break;
    case 0x1e: break;       // Vector 32: 9th bit break;
    default: break;
    }
}
