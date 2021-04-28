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

#include "i2c.h"


volatile uint8_t buffer[16] = {0};
volatile uint8_t bytesLength = 0;
volatile uint8_t gbl_regOffset = 0;
volatile bool restartTx = false;
volatile bool nackFlag = false;
volatile bool stopFlag = false;
volatile bool restartRx = false;

#define EUSCI_BASE  EUSCI_B0_BASE


void i2c_init()
{
    /* Select Port 1
    * Set Pin 6, 7 to input Secondary Module Function, (UCB2SIMO/UCB2SDA, UCB2SOMI/UCB2SCL).
    */
    GPIO_setAsPeripheralModuleFunctionInputPin(
            GPIO_PORT_P1,
            GPIO_PIN2 + GPIO_PIN3,
            GPIO_PRIMARY_MODULE_FUNCTION
    );

    //Initialize Master
    EUSCI_B_I2C_initMasterParam param = {0};
    param.selectClockSource = EUSCI_B_I2C_CLOCKSOURCE_SMCLK;
    param.i2cClk = CS_getSMCLK();
    param.dataRate = 100000;
    param.byteCounterThreshold = 16;
    param.autoSTOPGeneration = EUSCI_B_I2C_NO_AUTO_STOP;
    EUSCI_B_I2C_initMaster(EUSCI_BASE, &param);

}

void i2c_off() {
    EUSCI_B_I2C_disable(EUSCI_BASE);

    GPIO_setAsInputPinWithPullDownResistor(GPIO_PORT_P1, GPIO_PIN2 + GPIO_PIN3);
}

uint8_t i2c_write8(uint8_t slaveAddr, uint8_t regOffset, uint8_t writeData)
{
    while(i2c_write_block(slaveAddr, regOffset, 1, &writeData)==0);

    return 0;
}

int i2c_readreg(uint8_t slaveAddr, uint8_t mema, uint8_t rega)
{
    /* If I2C bus is currently being used
         * do not attempt to manipulate any I2C vars or registers */
    uint8_t rxData = 0;

    i2c_read_block(slaveAddr, mema, 1, &rxData, rega);

    return rxData;
}

void i2c_send_start(uint8_t slaveAddr)
{
    EUSCI_B_I2C_masterSendStart(EUSCI_BASE);
    EUSCI_B_I2C_masterSendStart(EUSCI_BASE);
    EUSCI_B_I2C_masterSendStart(EUSCI_BASE);
}

int i2c_write_block(uint8_t slaveAddr, uint8_t regOffset, uint8_t bytesToWrite, uint8_t * txData)
{
    int i;
    int success = 0;
    int retransmitCounter = 0;

    retransmitCounter = 0;

    bytesLength = bytesToWrite;

    for(i=0; i<bytesLength; i++)
    {
        buffer[i] = *(txData + i);
    }

    //Disable the USCI module and clears the other bits of control register
    EUSCI_B_I2C_disable(EUSCI_BASE);

    //Configure Automatic STOP condition generation
    HWREG16(EUSCI_BASE + OFS_UCBxCTLW1) &= ~UCASTP_3;
    HWREG16(EUSCI_BASE + OFS_UCBxCTLW1) |= EUSCI_B_I2C_NO_AUTO_STOP;

    //Specify slave address
    EUSCI_B_I2C_setSlaveAddress(EUSCI_BASE, slaveAddr);

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
    gbl_regOffset = regOffset;


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

int i2c_read_block(uint8_t slaveAddr, uint8_t regOffset, uint8_t bytesToRead, uint8_t * rxData, uint8_t rega)
{
    int i;
    int retransmitCounter = 0;
    int bytesToWrite;
    int success;
    uint8_t txData = 0;

    if (rega == 0xFF)
    {
        bytesToWrite = 0;
    }
    else
    {
        bytesToWrite = 1;
        txData = rega;
    }

    while(1) {
        success = i2c_write_block(slaveAddr, regOffset, bytesToWrite, &txData);
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
    HWREG16(EUSCI_BASE + OFS_UCBxTBCNT) = bytesToRead;

    bytesLength = bytesToRead;

   // Set to receive mode
   EUSCI_B_I2C_setMode(EUSCI_BASE, EUSCI_B_I2C_RECEIVE_MODE);

   // Enable I2C Module to start operations
   EUSCI_B_I2C_enable(EUSCI_BASE);


    EUSCI_B_I2C_clearInterrupt(EUSCI_BASE,
                               EUSCI_B_I2C_RECEIVE_INTERRUPT0 + EUSCI_B_I2C_TRANSMIT_INTERRUPT0 +
                               EUSCI_B_I2C_NAK_INTERRUPT
    );

    //Enable master Receive interrupt
    EUSCI_B_I2C_enableInterrupt(EUSCI_BASE,
                                EUSCI_B_I2C_RECEIVE_INTERRUPT0 +
                                EUSCI_B_I2C_NAK_INTERRUPT
    );

    __delay_cycles(100);

    // I2C start condition
    EUSCI_B_I2C_masterReceiveStart(EUSCI_BASE);

    __bis_SR_register(CPUOFF + GIE);    // Enter LPM0 w/ interrupt

    while(nackFlag == true && (retransmitCounter++ < 200))
    {
        nackFlag = false;
        EUSCI_B_I2C_masterReceiveStart(EUSCI_BASE);
        __bis_SR_register(CPUOFF + GIE);    // Enter LPM0 w/ interrupt
    }

    //Enable master Receive interrupt
    EUSCI_B_I2C_disableInterrupt(EUSCI_BASE,
                                 EUSCI_B_I2C_RECEIVE_INTERRUPT0 +
                                 EUSCI_B_I2C_NAK_INTERRUPT
    );

    for(i=0; i<bytesLength; i++)
    {
        *(rxData + i) = buffer[i];
    }

    return success;

}

uint8_t i2c_read8(uint8_t slaveAddr, uint8_t regOffset)
{
    uint8_t receiveData;

    i2c_read_block(slaveAddr, regOffset, 1, &receiveData, 0xFF);

    return receiveData;
}

uint16_t i2c_read16(uint8_t slaveAddr, uint8_t regOffset)
{
    uint8_t receiveData[2];
    uint16_t rxDataLsb;
    uint16_t rxDataMsb;


    i2c_read_block(slaveAddr, regOffset, 2, receiveData, 0xFF);

    rxDataLsb = receiveData[0];
    rxDataMsb = receiveData[1];

    return ((rxDataMsb << 8)&0xFF00) | (rxDataLsb & 0x00FF);
}

uint16_t i2c_read32(uint8_t slaveAddr, uint8_t regOffset, uint16_t * temp, uint16_t * hum)
{
    uint8_t receiveData[4];
    uint16_t tempLsb , humLsb;
    uint16_t tempMsb, humMsb;

    i2c_read_block(slaveAddr, regOffset, 4, receiveData, 0xFF);

    tempLsb = receiveData[0];
    tempMsb = receiveData[1];
    humLsb = receiveData[2];
    humMsb = receiveData[3];

    *temp = ((tempMsb << 8)&0xFF00) | (tempLsb & 0x00FF);
    *hum = ((humMsb << 8)&0xFF00) | (humLsb & 0x00FF);

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
