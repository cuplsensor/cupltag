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

#include "comms_uart.h"
#include "defs.h"
#include <msp430.h>
#include <stdbool.h>
#include "stdint.h"
#include <string.h>

#define HW_VERSION    HT07R2
#define FW_VERSION    1.5

#define    XSTR(V)        #V
#define    STR(V)        XSTR(V)

#define    VERSION        STR(HW_VERSION) "_F" STR(FW_VERSION) "_C" STR(CODEC_VERSION)

static char __version__[] = "<x:" VERSION ">";

typedef enum uart_ret_codes {
    rc_ok,
    rc_fail,
    rc_wait
} t_uretcode;

typedef enum uart_event_codes {
    evt_none,
    evt_rxdone,
    evt_txdone,
} t_uevent;

typedef enum uart_state_codes {
    uartsc_init,
    uartsc_txboot,
    uartsc_prepRx,
    uartsc_waitforRx,
    uartsc_pcktrxed,
    uartsc_prepTx,
    uartsc_waitforTx,
    uartsc_error
} t_ustate;

struct utransition {
    t_ustate      src_state;
    t_uretcode    ret_code;
    t_ustate      dst_state;
};

t_uretcode uart_init(t_uevent evt);
t_uretcode uart_txboot(t_uevent evt);
t_uretcode uart_prepRx(t_uevent evt);
t_uretcode uart_waitforRx(t_uevent evt);
t_uretcode uart_pcktrxed(t_uevent evt);
t_uretcode uart_prepTx(t_uevent evt);
t_uretcode uart_waitforTx(t_uevent evt);
t_uretcode uart_error(t_uevent evt);

#define EXIT_STATE uartsc_error
#define ENTRY_STATE uartsc_init

/* array and enum below must be in sync! */
static t_uretcode (* ustate_fcns[])(t_uevent) = {
                          uart_init,
                          uart_txboot,
                          uart_prepRx,
                          uart_waitforRx,
                          uart_pcktrxed,
                          uart_prepTx,
                          uart_waitforTx,
                          uart_error
};

struct utransition ustate_transitions[] = {
                                          {uartsc_init,  rc_ok,  uartsc_txboot},

                                          {uartsc_txboot,  rc_ok,  uartsc_prepTx},

                                          {uartsc_prepRx,  rc_ok,  uartsc_waitforRx},

                                          {uartsc_waitforRx,  rc_ok,  uartsc_pcktrxed},
                                          {uartsc_waitforRx,  rc_wait,  uartsc_waitforRx},

                                          {uartsc_pcktrxed,  rc_ok,  uartsc_prepTx},
                                          {uartsc_pcktrxed,  rc_wait,  uartsc_pcktrxed},

                                          {uartsc_prepTx,  rc_ok,  uartsc_waitforTx},

                                          {uartsc_waitforTx,  rc_ok,  uartsc_prepRx},
                                          {uartsc_waitforTx,  rc_wait,  uartsc_waitforTx}
};



/* Global variables. */
volatile uint8_t uartBuffer[72]; // Must be bigger than the largest packet that can be recieved.
volatile unsigned int bufIndex = 0;
volatile int drdyFlag = 0;
volatile int txDoneFlag = 0;
static t_ustate cur_state = ENTRY_STATE;

/* Look up transitions from the table for a current state and given return code. */
static t_ustate lookup_transitions(t_ustate curstate, t_uretcode rc)
{
    t_ustate nextstate = uartsc_error;
    int i=0;

    for (i=0; i<sizeof(ustate_transitions)/sizeof(ustate_transitions[0]); i++)
    {
        if ((ustate_transitions[i].src_state == curstate) && (ustate_transitions[i].ret_code == rc))
        {
            nextstate = ustate_transitions[i].dst_state;
        }
    }

    return nextstate;
}



/******************************************************************************
* @brief  uartSetup function
******************************************************************************/
t_uretcode uart_init(t_uevent evt)
{
    // Set the USCI state machine to reset.
    UCA0CTL1 = UCSWRST;

    // Initialise the USCI registers.
    /* http://mspgcc.sourceforge.net/cgi-bin/msp-uart.pl?clock=1500000&baud=9600&submit=calculate */
    UCA0CTL1 |= UCSSEL__SMCLK;      // CLK = SMCLK
    UCA0BR0 = 0x68;                 // Baud rate register. 1MHz / 9600baud.
    UCA0BR1 = 0x00;
    UCA0MCTLW = UCBRS0;

    // Clear UCSWRST flag. Leave reset.
    UCA0CTL1 &= ~UCSWRST;


    // Configure ports for UART.
    P1SEL0 |= BIT6 + BIT7;


    return rc_ok;
}

t_uretcode uart_txboot(t_uevent evt)
{
    t_uretcode rc = rc_ok;

    strncpy(uartBuffer, "<boot>", sizeof(uartBuffer));

    return rc;
}

t_uretcode uart_prepRx(t_uevent evt)
{
    t_uretcode rc = rc_ok;

    // Enable receive interrupts.
    bufIndex = 0;
    UCA0IE |= UCRXIE;

    return rc;
}

t_uretcode uart_waitforRx(t_uevent evt)
{
    t_uretcode rc = rc_wait;

    if (evt == evt_rxdone)
    {
        rc = rc_ok;
    }

    return rc;
}

#define INDEX_ID    1
#define INDEX_VAL   3

t_uretcode uart_pcktrxed(t_uevent evt)
{
    t_uretcode rc = rc_ok;
    bool validpacket;
    char id = uartBuffer[INDEX_ID];
    char len = bufIndex - INDEX_VAL;

    // Disable receive interrupts.
    UCA0IE &= ~UCRXIE;

    // Process packet here
    if (id == 'z') {
        PMMCTL0 = PMMPW | PMMSWPOR; // Soft Power on Reset.
    }
    else if (id == 'y') {
        __disable_interrupt();      // Disable interrupts
        UCA0CTL1 = UCSWRST;          // Reset the USCI state machine.
        RTCCTL   = 0;                // Stop the RTC
        TB0CTL   = MC_0;             // Stop Timer B
        TB1CTL   = MC_0;
        WDTCTL   = WDTPW | WDTHOLD;  // Hold the watchdog.
        ((void (*)())0x1000)();      // Jump to BSL.
    }
    else if (id == 'x')
    {
        validpacket = true;
        strncpy(uartBuffer, __version__, sizeof(__version__));
    }
    else {
        validpacket = nvparams_write(id, &uartBuffer[INDEX_VAL], len);
    }


    // Respond with an error.
    if (validpacket == false)
    {
        strncpy(uartBuffer, "<e>", sizeof(uartBuffer));
    }

    return rc;
}

t_uretcode uart_prepTx(t_uevent evt)
{
    t_uretcode rc = rc_ok;

    // Enable transmit interrupt.
    bufIndex = 0;
    UCA0IE |= UCTXIE;

    return rc;
}

t_uretcode uart_waitforTx(t_uevent evt)
{
    t_uretcode rc = rc_wait;

    if (evt == evt_txdone)
    {
        rc = rc_ok;
    }

    return rc;
}

t_uretcode uart_error(t_uevent evt)
{
    t_uretcode rc = rc_ok;

    return rc;
}

t_ustat uart_run()
{
    t_ustat status = ustat_running;
    t_uevent evt = evt_none;
    t_uretcode (* state_fun)(t_uevent);
    t_uretcode rc;

    if (drdyFlag)
    {
        drdyFlag = 0;
        evt = evt_rxdone;
    }
    else if (txDoneFlag)
    {
        txDoneFlag = 0;
        evt = evt_txdone;
    }

    state_fun = ustate_fcns[cur_state];
    rc = state_fun(evt);
    cur_state = lookup_transitions(cur_state, rc);

    if (rc == rc_wait)
    {
        status = ustat_waiting;
    }

    if (cur_state == EXIT_STATE)
    {
        status = ustat_finished;
    }

    return status;
}

// Echo back RXed character, confirm TX buffer is ready first
#pragma vector=USCI_A0_VECTOR
__interrupt void USCI_A0_ISR(void)
{
  uint8_t txrxByte;

  switch(__even_in_range(UCA0IV,0x08))
  {
  case 0:break;                             // Vector 0 - no interrupt
  case 2:                                  	// Vector 2 - RXIFG
	  /* Process the byte. */
      txrxByte = UCA0RXBUF;
	  if (txrxByte == '<')
	  {
	      bufIndex = 0;
	  }
	  else if (bufIndex < sizeof(uartBuffer)-1)
	  {
	      bufIndex++;
	  }

	  uartBuffer[bufIndex] = txrxByte;

	  if (txrxByte == '>' || (bufIndex >= sizeof(uartBuffer)-1))
	  {
	      UCA0IE &= ~UCRXIE;
	      drdyFlag = 1;
	      __bic_SR_register_on_exit(LPM3_bits); // Clear LPM bits upon ISR Exit
	  }
  	  break;
  case 4:
      /* Transmit the byte. */
      txrxByte = uartBuffer[bufIndex++];
      UCA0TXBUF = txrxByte;

      if (txrxByte == '>' || (bufIndex >= sizeof(uartBuffer)-1))
      {
          UCA0IE &= ~UCTXIE;
          txDoneFlag = 1;
          __bic_SR_register_on_exit(LPM3_bits);     // Clear LPM bits upon ISR Exit
      }

      break;
  default: break;
  }
}
