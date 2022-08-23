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
 * @file main.c
 * @author Malcolm Mackay
 *
 * @brief Top-level Finite State Machine for controlling the MSP430 and cuplTag as a whole.
 *
 * A Finite State Machine is defined in this file and run from main(). This has several features.
 *
 * It makes calls to drivers for communicating with the HDC2022 humidity sensor and the NT3H2111 NFC EEPROM.
 *
 * It controls entry into the 'programming mode' sub state machine, where configuration strings can be written
 * using a serial port.
 *
 * It reads configuration strings from the NFC EEPROM if any are present.
 *
 * It collects samples from an HDC2022 at a fixed time interval and passes these to the cuplcodec encoder.
 *
 */

#include "driverlib.h"
#include "hdc2021.h"
#include "sample.h"
#include "comms_uart.h"
#include "confignfc.h"
#include "nt3h.h"
#include "stat.h"
#include "defs.h"
#include "eep.h"
#include "batv.h"
#include <stdlib.h>


#define CS_SMCLK_DESIRED_FREQUENCY_IN_KHZ   1000       /*!< Target frequency for SMCLK in kHz. */

#define CS_XT1_CRYSTAL_FREQUENCY            32768       /*!< Resonant frequency of the XT1 crystal in kHz. */
#define CS_XT1_TIMEOUT                      65000       /*!< Timeout for XT1 to stabilise at the resonant frequency in SMCLK cycles. */

#define CP10MS                                  41      /*!< ACLK Cycles Per 10 MilliSeconds. Assumes ACLK = 32768 kHz and a divide-by-8. */


#define EXIT_STATE sc_end                               /*!< State machine exit state. */
#define ENTRY_STATE sc_init                             /*!< State machine entry state. */

/*!
 * @brief Enable writes to program FRAM.
 *
 * Some variables are stored in program FRAM. RAM cannot be used because state is lost in deep sleep mode (LPM3.5).
 * The Program FRAM Write Protect bit must be cleared (and interrupts disabled) before a write.
 */
void fram_write_enable() {
    __disable_interrupt();      // TI advise against having interrupts enabled during a write of Program FRAM.
    SYSCFG0 = FRWPPW | DFWP;    // Clear the PFWP bit (program FRAM write protect). Leave the DFWP bit set (user FRAM).
}

/*!
 * @brief Disable writes to program FRAM.
 *
 * Sets the Program FRAM Write Protect bit and re-enables interrupts.
 */
void fram_write_disable() {
    SYSCFG0 = FRWPPW | DFWP | PFWP;  // Set both program FRAM and user FRAM write protect bits.
    __enable_interrupt();            // Re-enable global interrupts after the FRAM write has completed.
}

volatile int timerFlag = 0;    /*!< Flag set by the Timer Interrupt Service Routine. */
volatile int hdcFlag = 0;      /*!< Flag set by the HDC2021 humidity sensor data-ready Interrupt Service Routine. */


#pragma PERSISTENT(minutecounter)
int minutecounter = 0;         /*!< Incremented each time the sampling loop is run. */

/** Hard-coded NDEF message containing one text record
 *  "Programming Mode. Connect to serial port at 9600 baud." */
const char ndefmsg_progmode[] = {0x03, 0x3D, 0xD1, 0x01,
                                 0x39, 0x54, 0x02, 0x65,
                                 0x6E, 0x50, 0x72, 0x6F,
                                 0x67, 0x72, 0x61, 0x6D,
                                 0x6D, 0x69, 0x6E, 0x67,
                                 0x20, 0x4D, 0x6F, 0x64,
                                 0x65, 0x2E, 0x20, 0x43,
                                 0x6F, 0x6E, 0x6E, 0x65,
                                 0x63, 0x74, 0x20, 0x74,
                                 0x6F, 0x20, 0x73, 0x65,
                                 0x72, 0x69, 0x61, 0x6C,
                                 0x20, 0x70, 0x6F, 0x72,
                                 0x74, 0x20, 0x61, 0x74,
                                 0x20, 0x39, 0x36, 0x30,
                                 0x30, 0x20, 0x62, 0x61,
                                 0x75, 0x64, 0x2E, 0xFE};

/** Hard-coded NDEF message containing one text record
 *  "Config check failed. See cuplTag docs." */
const char ndefmsg_noconfig[] = {0x03, 0x2D, 0xD1, 0x01,
                                 0x29, 0x54, 0x02, 0x65,
                                 0x6E, 0x43, 0x6F, 0x6E,
                                 0x66, 0x69, 0x67, 0x20,
                                 0x63, 0x68, 0x65, 0x63,
                                 0x6B, 0x20, 0x66, 0x61,
                                 0x69, 0x6C, 0x65, 0x64,
                                 0x2E, 0x20, 0x53, 0x65,
                                 0x65, 0x20, 0x63, 0x75,
                                 0x70, 0x6C, 0x54, 0x61,
                                 0x67, 0x20, 0x64, 0x6F,
                                 0x63, 0x73, 0x2E, 0xFE};

/** Hard-coded NDEF message containing one text record
 *  "Error: Invalid state transition." */
const char ndefmsg_badtrns[] =  {0x03, 0x27, 0xD1, 0x01,
                                 0x23, 0x54, 0x02, 0x65,
                                 0x6E, 0x45, 0x72, 0x72,
                                 0x6F, 0x72, 0x3A, 0x20,
                                 0x49, 0x6E, 0x76, 0x61,
                                 0x6C, 0x69, 0x64, 0x20,
                                 0x73, 0x74, 0x61, 0x74,
                                 0x65, 0x20, 0x74, 0x72,
                                 0x61, 0x6E, 0x73, 0x69,
                                 0x74, 0x69, 0x6F, 0x6E,
                                 0x2E, 0xFE};

/** States in the Finite State Machine are represented by a code.
 *  This is used each time the state machine is run, to determine:
 *
 *  1. Which state function to call in main().
 *  2. The next state in lookup_transitions().  */
typedef enum state_codes {
    sc_init,                    /*!< State code for init_state() */
    sc_init_reqmemon,           /*!< State code for init_reqmemon() */
    sc_init_waitmemon,          /*!< State code for init_waitmemon() */
    sc_init_ntag,               /*!< State code for init_ntag() */
    sc_init_progmode,           /*!< State code for init_progmode() */
    sc_init_configcheck,        /*!< State code for init_configcheck() */
    sc_init_errorcheck,         /*!< State code for init_errorcheck() */
    sc_init_wakeupcheck,        /*!< State code for init_wakeupcheck() */
    sc_init_batvwait,           /*!< State code for init_batvwait() */
    sc_init_rtc_slow,           /*!< State code for init_rtc_slow() */
    sc_init_rtc_1min,           /*!< State code for init_rtc_1min() */
    sc_smpl_checkcounter,       /*!< State code for smpl_checkcounter() */
    sc_smpl_hdcreq,             /*!< State code for smpl_hdcreq() */
    sc_smpl_hdcwait,            /*!< State code for smpl_hdcwait() */
    sc_smpl_hdcread,            /*!< State code for smpl_hdcread() */
    sc_smpl_wait,               /*!< State code for smpl_wait() */
    sc_err_msg,                 /*!< State code for err_msg() */
    sc_end                      /*!< State code for end_state() */
} tstate;

/** Each state function returns at least one code from the list below.
 *  This is used by lookup_transitions() to determine the next state. */
typedef enum ret_codes { 
    tr_ok,
    tr_prog,
    tr_newconfig,
    tr_hdcreq,         /*!< Request a sample from the HDC2021 sensor. */
    tr_updatemin,
    tr_deepsleep,      /*!< cuplTag should enter a deep sleep state LPM3.5 */
    tr_lowbat,
    tr_fail,
    tr_samplingloop,   /*!< cuplTag is in the sampling loop. The reset was caused by an exit from LPM3.5 */
    tr_por,            /*!< cuplTag is NOT in the sampling loop. A Power-On-Reset has occurred. */
    tr_wait            /*!< cuplTag should enter a sleep state, such as LPM0, to wait for an event. */
} tretcode;

/** Events occur asynchronously to execution of the FSM. The MSP430 will typically wait for an event in sleep mode to save power.
 *  An example is an edge on the INT (interrupt) output from a temperature sensor. This is connected to an input on the MSP430,
 *  which is configured to call an Interrupt Service Routine (ISR). The ISR sets a flag and 'wakes up' the MSP430 from sleep mode.
 *
 *  The main() function is entered, flags are checked then cleared and the event variable is set according to the list of codes below.
 *  Finally, the state function is called with the event passed as an argument.
 *
 *  Multiple events can occur simultaneously, but only one at-a-time is passed to the FSM.
 *  */
typedef enum event_codes {
    evt_none,               /*!< No event has occurred. */
    evt_timerfinished,      /*!< The timer peripheral has counted down to 0. */
    evt_hdcint              /*!< Pin change interrupt received from the HDC2021 temperature and humidity sensor. */
} tevent;

// https://stackoverflow.com/questions/1371460/state-machines-tutorials
tretcode init_state(tevent);
tretcode init_reqmemon(tevent);
tretcode init_waitmemon(tevent);
tretcode init_ntag(tevent);
tretcode init_progmode(tevent);
tretcode init_configcheck(tevent);
tretcode init_errorcheck(tevent);
tretcode init_wakeupcheck(tevent);
tretcode init_batvwait(tevent);
tretcode init_rtc_slow(tevent);
tretcode init_rtc_1min(tevent);
tretcode smpl_checkcounter(tevent);
tretcode smpl_hdcreq(tevent);
tretcode smpl_hdcwait(tevent);
tretcode smpl_hdcread(tevent);
tretcode smpl_wait(tevent);
tretcode err_msg(tevent);
tretcode end_state(tevent);

/* This array must be in sync with the ::state_codes enumerator above.  */
tretcode (* state_fcns[])(tevent) = {
                          init_state,
                          init_reqmemon,
                          init_waitmemon,
                          init_ntag,
                          init_progmode,
                          init_configcheck,
                          init_errorcheck,
                          init_wakeupcheck,
                          init_batvwait,
                          init_rtc_slow,
                          init_rtc_1min,
                          smpl_checkcounter,
                          smpl_hdcreq,
                          smpl_hdcwait,
                          smpl_hdcread,
                          smpl_wait,
                          err_msg,
                          end_state
};


struct transition {
    tstate      src_state; /*!< Source state. */
    tretcode    ret_code;  /*!< Code returned after executing a state function. */
    tstate      dst_state; /*!< Destination state. */
};

/** The state transition table. */
struct transition state_transitions[] = {
                                         {sc_init,          tr_ok,      sc_init_reqmemon},

                                         {sc_init_reqmemon, tr_ok,      sc_init_waitmemon},

                                         {sc_init_waitmemon, tr_ok,      sc_init_ntag},
                                         {sc_init_waitmemon, tr_wait,    sc_init_waitmemon},

                                         {sc_init_ntag,    tr_ok,        sc_init_wakeupcheck},
                                         {sc_init_ntag,    tr_newconfig, sc_init_rtc_slow},
                                         {sc_init_ntag,    tr_prog,      sc_init_progmode},

                                         {sc_init_progmode,  tr_ok,      sc_init_progmode},
                                         {sc_init_progmode,  tr_wait,    sc_init_progmode},
                                         {sc_init_progmode,  tr_fail,    sc_err_msg},

                                         {sc_init_wakeupcheck,  tr_por,           sc_init_rtc_slow},
                                         {sc_init_wakeupcheck,  tr_samplingloop,  sc_smpl_checkcounter},

                                         {sc_init_rtc_slow,  tr_ok,     sc_init_batvwait},

                                         {sc_init_batvwait,  tr_ok,     sc_init_configcheck},
                                         {sc_init_batvwait,  tr_wait,   sc_init_batvwait},

                                         {sc_init_configcheck,  tr_ok,          sc_init_errorcheck},
                                         {sc_init_configcheck,  tr_deepsleep,   sc_end},

                                         {sc_init_errorcheck,  tr_ok,           sc_init_rtc_1min},
                                         {sc_init_errorcheck,  tr_deepsleep,    sc_end},

                                         {sc_init_rtc_1min,    tr_ok,      sc_smpl_checkcounter},

                                         {sc_smpl_checkcounter, tr_hdcreq,      sc_smpl_hdcreq},
                                         {sc_smpl_checkcounter, tr_updatemin,   sc_smpl_wait},

                                         {sc_smpl_hdcreq,   tr_ok,      sc_smpl_hdcwait},

                                         {sc_smpl_hdcwait,  tr_ok,      sc_smpl_hdcread},
                                         {sc_smpl_hdcwait,  tr_wait,    sc_smpl_hdcwait},

                                         {sc_smpl_hdcread,  tr_ok,      sc_smpl_wait},
                                         {sc_smpl_hdcread,  tr_lowbat,  sc_end},

                                         {sc_smpl_wait,     tr_deepsleep,   sc_end},

                                         {sc_err_msg,  tr_deepsleep,     sc_end}
                                         
};


/*!
 *   @brief Look up the next state in the Finite State Machine.
 *
 *   The look-up is performed by iterating through an array of transitions.
 *   An error state is returned if no match is found.
 *
 *   @param[in] curstate Current state.
 *   @param[in] rc Code returned from the current state.
 *   @return Next state.
 */
tstate lookup_transitions(tstate curstate, tretcode rc)
{
    tstate nextstate = sc_err_msg; /* This should never be reached. */
    int i=0;

    /* Read each transition in the state_transitions array. */
    for (i=0; i<sizeof(state_transitions)/sizeof(state_transitions[0]); i++)
    {
        /* If a transition that matches the current state and return code is found... */
        if ((state_transitions[i].src_state == curstate) && (state_transitions[i].ret_code == rc))
        {
            /* ...use it to set the next state and break out of the loop. */
            nextstate = state_transitions[i].dst_state;
            break;
        }
    }

    return nextstate;
}

/*!
 *   @brief Write an NDEF message to the NFC EEPROM.
 *
 *   The NDEF message normally contains one text record. It can be created with an
 *   external program and stored as a constant array. The function is used to display simple
 *   error messages to the end-user.
 *
 *   @param[in] msgptr Pointer an NDEF message array.
 *   @param[in] len Length of the NDEF message.
 */
static void writetxt(const char * msgptr, int len) {
    int eepindex = 0;
    int blk;

    eep_cp(&eepindex, msgptr, len);
    for (blk=0; blk<4; blk++) {
        eep_write(blk, blk);
    }
    eep_waitwritedone();
}

/*!
 *   @brief Kick the watchdog, to prevent it from timing out.
 *
 *   This is done by writing to the watchdog control register.
 */
static void wdog_kick()
{
    /*
     * WDTPW = Watchdog password.
     * WDTSSEL__ACLK = Use the 32.768 kHZ auxiliary clock (ACLK) as a source for the watchdog timer.
     * WDTCNTCL = Clear the watchdog timer.
     * WDTIS = Watchdog timer interval select. 8192K corresponds to 4 minutes, 16 seconds at 32 kHz.
     */
    WDTCTL = WDTPW | WDTSSEL__ACLK | WDTCNTCL | WDTIS__8192K;
}

/*!
 *  @brief Start a single-shot timer.
 *
 *  An interrupt fires when the timer has finished counting. The MSP430 
 *  can sleep in LPM3 whilst waiting for it. This saves power over delay loops.
 *
 *  The function is best suited to pausing execution for a short time (milliseconds).
 *
 *  @param[in] intervalCycles Number of 4.096 kHz clock cycles to count.
 *
 */
static void start_timer(unsigned int intervalCycles)
{
    /* Initialise a continuous mode parameters struct. */
    Timer_B_initContinuousModeParam initContParam = {0};
    
    /* Set the clock source to ACLK / 8 = 32.768kHz / 8. */
    initContParam.clockSource = TIMER_B_CLOCKSOURCE_ACLK;
    initContParam.clockSourceDivider = TIMER_B_CLOCKSOURCE_DIVIDER_8;

    /* Disable TBIFG by de-asserting TBIE.
     * Timer_B has 2 dedicated interrupt request outputs. Only the latter is used.:
     * TBIFG: Multiplexed interrupt flag.
     * CCIFG: Dedicated interrupt flag for Capture Compare Register 0.  */
    initContParam.timerInterruptEnable_TBIE = TIMER_B_TBIE_INTERRUPT_DISABLE;

    /* Clear the count value and disable the timer. */
    initContParam.timerClear = TIMER_B_DO_CLEAR;
    initContParam.startTimer = false;

    /* Write all settings from the parameters struct to Timer_B1. */
    Timer_B_initContinuousMode(TB1_BASE, &initContParam);

    /* Clear any existing Capture Compare Register interrupts. */
    Timer_B_clearCaptureCompareInterrupt(TB1_BASE, TIMER_B_CAPTURECOMPARE_REGISTER_0);

    /* Initialise a compare mode parameters struct. */
    Timer_B_initCompareModeParam initCompParam = {0};

    /* Set the compare register to CCR0. */
    initCompParam.compareRegister = TIMER_B_CAPTURECOMPARE_REGISTER_0;

    /* Enable interrupt CCIFG. This fires when the counter equals the compare value in CCR0. */
    initCompParam.compareInterruptEnable = TIMER_B_CAPTURECOMPARE_INTERRUPT_ENABLE;
    
    /* Set the timer output to the interrupt flag value. 
     * This can be routed to an output pin, but it is not used. */
    initCompParam.compareOutputMode = TIMER_B_OUTPUTMODE_OUTBITVALUE;

    /* Set the compare value to intervalCycles. */
    initCompParam.compareValue = intervalCycles;

    /* Write all settings from the compare mode structure to Timer_B1. */
    Timer_B_initCompareMode(TB1_BASE, &initCompParam);

    /* Start the counter from 0. */
    Timer_B_startCounter(TB1_BASE, TIMER_B_CONTINUOUS_MODE);
}

/*!
 *  @brief Power down the VMEM domain.
 *
 *   The load switch enable pin is set low, breaking the circuit between VDD and VMEM.
 *   This is done to save power in sleep mode. The NT3H2111 EEPROM will otherwise draw ~10uA.
 */
static void memoff()
{
    // Set the load switch enable pin P3.2 LOW.
    GPIO_setOutputLowOnPin(
            GPIO_PORT_P3,
            GPIO_PIN2
    );

    // Stop the timer if it is running. Use the RTC for delays when VMEM=0V.
    Timer_B_stop(TB1_BASE);

    // All I2C slaves are powered down so disable the MSP430 peripheral to save power.
    i2c_off();
}

/*!
 *  @brief Initialise clocks and IOs on the MSP430.
 *
 *  All IOs are configured into an initial state. The number of IOs left as
 *  inputs (default) must be minimised to reduce power consumption.
 *
 *  The slow Auxiliary Clock (ACLK) is sourced form the external 32.768 kHz crystal.
 *  An internal 10 kHz source is used by default. This is power hungry and drifts with temperature.
 *
 *  Next, the Phased-Locked Loop (DCO) generates an output frequency of 1 MHz, by multiplying
 *  the external 32.768 kHz crystal frequency up by 31.
 *
 *  Internal clocks MCLK and SMCLK are connected to the DCO output.
 *
 *  1 MHz was selected to minimise current draw from the high impedance coin cell battery.
 *  This results in a lower voltage drop after exiting the sleep state. Battery life is limited by this voltage drop. 
 *  This is not the case if the source impedance is lower. Then it is best to operate at a higher frequency: up to 24 MHz.
 *
 *  Finally, the cause of the reset is read. The program needs to know whether this is just a routine wake-up
 *  from sleep (LPM3.5) or the result of a fault.
 */
tretcode init_state(tevent evt)
{
    int error;

    // Prevent watchdog interrupts during the first part of initialisation.
    WDTCTL = WDTPW | WDTHOLD;

    // Disable the Supply Voltage Supervisor.
    PMM_disableSVSH();

    // Initialise IO to reduce power.
    // P1.1 HDC_INT as input
    // P1.6 RX_BY_MCU as input
    // P3.5 nPRG as input
    // All other pins output low.
    P1DIR = 0xBD; P2DIR = 0xFF; P3DIR = 0xDF; P4DIR = 0xFF;
    P5DIR = 0xFF; P6DIR = 0xFF; P7DIR = 0xFF; P8DIR = 0xFF;
    P1OUT = 0x00; P2OUT = 0x00; P3OUT = 0x20; P4OUT = 0x00;
    P5OUT = 0x00; P6OUT = 0x00; P7OUT = 0x00; P8OUT = 0x00;
    P1REN = 0x00; P2REN = 0x00; P3REN = 0x24; P4REN = 0x00;
    P5REN = 0x00; P6REN = 0x00; P7REN = 0x00; P8REN = 0x00;

    // Configure P2.6 and P2.7 as an external crystal oscillator output and input respectively.
    P2SEL1 |= BIT6 | BIT7;

    // Disable the GPIO power-on default high-impedance mode
    // to activate previously configured port settings
    PM5CTL0 &= ~LOCKLPM5;

    // Read the reset cause.
    stat_rdrstcause();

    // Specify the external crystal frequency for the XT1 oscillator.
    CS_setExternalClockSource(CS_XT1_CRYSTAL_FREQUENCY);

    // Enable the XT1 oscillator.
    error = CS_turnOnXT1LFWithTimeout(CS_XT1_DRIVE_0, CS_XT1_TIMEOUT);

    if (error == STATUS_FAIL)
    {
        stat_setclockfailure();
    }

    // Set Auxiliary clock ACLK = XT1 oscillator output.
    CS_initClockSignal(
            CS_ACLK,
            CS_XT1CLK_SELECT,
            CS_CLOCK_DIVIDER_1
    );

    // Set DCO FLL reference = XT1 oscillator output.
    CS_initClockSignal(
            CS_FLLREF,
            CS_XT1CLK_SELECT,
            CS_CLOCK_DIVIDER_1
    );

    // Initialise the DCO by setting a Ratio (31) and Desired MCLK frequency (1 MHz)
    // 1MHz / 32.768 kHz = ~31
    error = CS_initFLLSettle(
                CS_SMCLK_DESIRED_FREQUENCY_IN_KHZ,
                31
    );

    if (error == STATUS_FAIL)
    {
        stat_setclockfailure();
    }

    // Set Sub Main Clock SMCLK = DCO output.
    CS_initClockSignal(
            CS_SMCLK,
            CS_DCOCLKDIV_SELECT,
            CS_CLOCK_DIVIDER_1
    );

    // Set Main Clock MCLK = DCO output.
    CS_initClockSignal(
            CS_MCLK,
            CS_DCOCLKDIV_SELECT,
            CS_CLOCK_DIVIDER_1
    );

    // Enable the watchdog timer.
    wdog_kick();

    // P3.5 nPRG as input
    GPIO_setAsInputPinWithPullUpResistor(
            GPIO_PORT_P3,
            GPIO_PIN5
    );

    // P1.6 UART RX as input
    GPIO_setAsInputPinWithPullDownResistor(
            GPIO_PORT_P1,
            GPIO_PIN6
    );

    return tr_ok;
}

/*!
 *  @brief Enable power to the VMEM domain.
 *
 *  Configure a pin to receive interrupts from the humidity sensor.
 *  Set the load switch enable pin HIGH to power up the VMEM domain from VDD.
 *
 *  After this function has been called, the MSP430 must sleep whilst waiting for
 *  a Timer interrupt. When this fires, the VMEM voltage should be stable.
 *
 */
static tretcode reqmemon(tevent evt)
{
    // P1.1 HDC_INT as input
    GPIO_setAsInputPinWithPullDownResistor(
            GPIO_PORT_P1,
            GPIO_PIN1
    );

    // P3.2 EN as output high
    GPIO_setAsOutputPin(
            GPIO_PORT_P3,
            GPIO_PIN2
    );

    GPIO_setOutputHighOnPin(
            GPIO_PORT_P3,
            GPIO_PIN2
    );

    // Use Timer_B to generate an interrupt after 20ms.
    // This allows enough time for capacitors in the VMEM domain to charge from VDD.
    start_timer(2*CP10MS);

    return tr_ok;
}

/*!
 *  @brief Wait for the VMEM voltage to stabilise after power on.
 *
 *  Timer_B1 must be started with start_timer() prior to calling this function.
 *
 *  @param[in] evt Event. When set to evt_timerfinished, I2C is enabled and the state machine progresses.
 *
 */
static tretcode waitmemon(tevent evt)
{
    tretcode rc = tr_wait;

    if (evt == evt_timerfinished)
    {
        /* SYS is up. Initialise I2C. */
        i2c_init();

        rc = tr_ok;
    }

    return rc;
}

/*!
 *  @brief This state calls reqmemon().
 */
tretcode init_reqmemon(tevent evt)
{
    return reqmemon(evt);
}

/*!
 *  @brief This state calls waitmemon().
 */
tretcode init_waitmemon(tevent evt)
{
    return waitmemon(evt);
}

/*!
 *  @brief Initialise the dual-interface I2C+NFC EEPROM.
 * 
 *  A call is made to 'nt3h_check_address()' to make sure the EEPROM is 
 *  at device address 0x55.
 *  
 *  The first EEPROM block is read to check for an NFC text record. If found, 
 *  configuration strings are extracted and saved into non-volatile memory.
 *
 *  The capability container is written if it needs to be. These 4 bytes 
 *  indicate that the tag contains an NDEF message.
 * 
 *  The programming mode select pin (nPRG) is checked. If it is 
 *  LOW, the return code is updated. 
 */ 
tretcode init_ntag(tevent evt)
{
    tretcode rc = tr_ok;
    int nPRG;
    int updatecc;

    // Enable Supply voltage supervisor
    // This should be running in active mode,
    // so a low battery voltage can cause a reset and
    // disable further operation.
    PMM_enableSVSH();

    //nt3h_init_wrongaddress();
    updatecc = nt3h_check_address();

    // Checks for an NFC text record.
    if (confignfc_check())
    {
        confignfc_readtext(); // Configure from text records.
        rc = tr_newconfig;
    }

    // Write the capability container if needed.
    if (updatecc) {
        nt3h_update_cc();
    }

    /* Check nPRG. */
    nPRG = GPIO_getInputPinValue(GPIO_PORT_P3, GPIO_PIN5);

    if (!nPRG) {
        // Write programming mode NDEF text record.
        writetxt(ndefmsg_progmode, sizeof(ndefmsg_progmode));
        rc = tr_prog;
    }

    return rc;
}

/*!
 *  @brief Run the programming mode sub-state machine.
 *
 *  Enables the serial port (UART) and responds to text commands.
 *
 *  It is only intended that this state be entered in a production environment,
 *  not by the end user.
 *
 *  The only way to exit is to reset the microcontroller. This can be done by
 *  sending a soft reset command '<z>'.
 */
tretcode init_progmode(tevent evt)
{
    tretcode rc;
    t_ustat uartstatus;

    // Kick the watchdog.
    wdog_kick();

    // Run the UART child state machine.
    uartstatus = uart_run();

    if (uartstatus == ustat_waiting)
    {
        rc = tr_wait; // An opportunity to sleep whilst waiting for an interrupt.
    }
    else if (uartstatus == ustat_finished)
    {
        /* The UART state machine should never finish. It should run until reset.
           If this is reached then an invalid state transition has been requested.
           In response the tag will output an error message and go into LPM4 (deep sleep). */
        rc = tr_fail;
    }
    else
    {
        rc = tr_ok; // Re-enter this state without sleeping.
    }

    return rc;
}

/*!
 *  @brief Verifies that all configuration strings have been written.
 *
 *  The state machine cannot continue unless the tag is fully configured. There are no default settings.
 *
 *  When configuration is incomplete, a text-based error message is written to the tag and
 *  deep-sleep mode is entered.
 */
tretcode init_configcheck(tevent evt)
{
    tretcode rc;

    if (nvparams_allwritten()) {
        rc = tr_ok;
    }
    else {
        // Write configuration failed NDEF text.
        writetxt(ndefmsg_noconfig, sizeof(ndefmsg_noconfig));

        // Enter LPM3.5 to save power.
        rc = tr_deepsleep;
    }

    return rc;
}

/*!
 *  @brief Check for an error condition before continuing startup.
 *
 *  An error occurs if the reset was caused by a reason other than a new battery insertion.
 *
 *  When the battery voltage is below a threshold (set in NVM). The
 *  state machine should not get stuck in a loop, where an attempt is made to write
 *  to the NFC EEPROM before the micro-controller resets. Reset loops can wear out the EEPROM.
 *
 *  In the event of 10 consecutive resets that result in an error, or a single low battery reading:
 *    -# Report the most recent error in the cupl URL status word.
 *    -# Do not include any samples in the cupl URL.
 *    -# Go to a deep sleep state that prevents another reset cycle from occurring for some time.
 *
 *  Otherwise:
 *    -# Report the most recent error in the cupl URL but treat it as spurious.
 *    -# Allow the state machine to continue.
 *
 *  @return ::tr_deepsleep when 10 consecutive errors have occurred or the battery voltage is low.
 *  Otherwise indicate no errors with ::tr_ok.
 *
 */
tretcode init_errorcheck(tevent evt)
{
    tretcode rc;
    bool syserr = false;    /* System error flag. When asserted, the system cannot continue. */
    bool rsterr = false;    /* Reset error flag. */
    bool rstborsvs = false; /* Reset caused by Brownout or Supply Voltage Supervisor flag. This is not always an error. It depends on battery voltage. */
    bool lowbatv = false;   /* Low battery voltage flag. */

    unsigned int status;
    int resetsalltime, resetsperloop;
    unsigned int batv, batv_mv;

    wdog_kick();                                    // Kick the watchdog.
    nvparams_incrcounters();                        // Increment both reset counters.
    resetsalltime = nvparams_getresetsalltime();    // Get the number of resets since first use from NVM.
    resetsperloop = nvparams_getresetsperloop();    // Get the number of resets before the first circular buffer wraparound.

    // Check for low battery.
    batv = batv_measure();                          // Use an ADC to measure the battery voltage.
    batv_mv = batv_to_mv(batv);                     // Convert from ADC counts to millivolts.
    if (batv_mv < nvparams_getminvoltagemv())       // Set a flag if the battery voltage is less than a threshold.
    {
        lowbatv = true;
    }

    // Check what has caused the reset and whether or not this is an error.
    // Also get the status word for inclusion in the URL.
    status = stat_get(&rsterr, &rstborsvs, resetsalltime);

    // Cancel the reset error if a new battery has been inserted.
    // New battery insertion corresponds to a BOR or an SVS_H reset flag
    // and a battery voltage higher than the minimum threshold. Sometimes
    // BOR and SVS_H occur when a new battery is inserted, but these are not indicative
    // of a power supply issue.
    if (rstborsvs && !lowbatv)
    {
        rsterr = false;
    }

    // Clear the resets counter if the reset has been caused intentionally (e.g. by the user replacing the battery).
    // The resets counter should only count unintended resets, not caused by user intervention.
    // Without this, the battery will go flat and cuplTag will be stuck in an error state,
    // even after a new battery is inserted.
    if (rsterr == false)
    {
        nvparams_cresetsperloop();
        resetsperloop = 0;
    }

    // Check for too many resets. Reset errors are permitted, so long as we do not count more than 10 consecutively.
    if ((resetsperloop > 10) || lowbatv)
    {
        // Go to LPM3.5 This prevents wearing out the EEPROM after repeated resets.
        syserr = true; // More than 10 consecutive unintended resets before reaching the end of the circular buffer. Stop.
    }

    /* Initialise minute counter to 0. */
    fram_write_enable();
    minutecounter = 0;
    fram_write_disable();

    /* Initialise the encoder. */
    enc_init(status, syserr, batv);
    rc = (syserr) ? tr_deepsleep : tr_ok;

    return rc;
}

/*!
 *  @brief Has the reset has been caused by a routine RTC wake-up?
 *
 *  In the sampling loop, Wake-ups from LPM3.5 occur every minute. These are
 *  invoked by an interrupt from the Real Time Clock peripheral.
 *
 *  This function first makes a call to stat_rstcause_is_lpm5wu().
 *  Then it checks if the first integer in Backup Memory is 1. If it is, then the cuplTag is
 *  in the sampling loop.
 *
 *  @return ::tr_samplingloop if the cuplTag is in the sampling loop.
 *  Otherwise, indicate a power-on-reset with ::tr_por.
 */
tretcode init_wakeupcheck(tevent evt) {
    tretcode rc = tr_por;
    unsigned int samplingloop; // Indicates that the sampling loop has been entered.

    // The next state will depends on whether this is
    // a wakeup from LPM3.5 or a Power On Reset.
    if (stat_rstcause_is_lpm5wu()) {
        // Read samplingloop from BKMEM. This will be set if the
        // MSP430 entered LPM3.5 as part of the sampling loop.
        samplingloop = *(unsigned int *)BKMEM_BASE;

        if (samplingloop == 1) {
            // Return to the sampling loop
            rc = tr_samplingloop;
        }
    }

    // Now that BKMEM has been read, reset it.
    // IF an unexpected error occurs the cuplTag will not be stuck in the sampling loop.
    *(unsigned int *)BKMEM_BASE = 0;

    return rc;
}

/*!
 *  @brief Configure the Real Time Clock to peripheral to generate one interrupt every 30 minutes.
 *
 *  This is done to prevent the cuplTag from being stuck in the end_state when an
 *  error occurs during startup. This state must be entered before any possible transitions
 *  to the end_state.
 *
 *  Whatever the error, the cuplTag must wake up and try to start up again.
 *  A long time interval has been chosen so that the battery is not depleted.
 */
tretcode init_rtc_slow(tevent evt)
{
    // An external 32.768 kHz crystal is the source of the Real Time Clock.
    // There are 32768 / 1024 = 32 RTC counts per second, due to the clock PreScaler.
    // 32 RTC counts per second * 60 seconds per minute * 30 minutes = 57600 RTC counts between interrupts.
    RTCMOD = 57600-1;
    RTCCTL = RTCSS__XT1CLK | RTCSR |RTCPS__1024;
    RTCCTL |= RTCIE; // Enable RTC interrupts

    // Pause for the battery voltage to stabilise.
    // If this is not done, the battery can appear to have a lower voltage than it should
    // and the state machine does not progress beyond init_errorcheck.
    start_timer(300*CP10MS);

    return tr_ok;
}

/*!
 *   @brief Wait for the battery voltage to stabilise.
 *
 *   It takes some time for capacitors to charge after a battery is
 *   inserted. A timer is started in the previous state. The MSP430
 *   waits here in low power mode.
 *
 *   @param[in] evt Event. When set to ::evt_timerfinished the state machine progresses.
 *   @return ::tr_ok when the timer has finished. Otherwise ::tr_wait.
 */
tretcode init_batvwait(tevent evt)
{
    // Wait for the timer flag to be raised.
    tretcode rc = tr_wait;

    if (evt == evt_timerfinished)
    {
        rc = tr_ok;
    }

    return rc;
}


/*!
 *  @brief Configure the Real Time Clock peripheral to generate one interrupt every minute.
 *
 *  The cuplTag spends most of the time in a deep sleep mode LPM3.5 to save power.
 *  Each minute, it wakes up for a few milliseconds to collect a sample or to increment ::minutecounter.
 *
 *  TURBO MODE is a special feature that is useful for test purposes.
 *  When enabled, the interrupt occurs every 3 seconds. To enable, set the time interval parameter to 0.
 */
tretcode init_rtc_1min(tevent evt)
{
    // An external 32.768 kHz crystal is the source of the Real Time Clock.
    // There are 32768 / 1024 = 32 RTC counts per second, due to the clock PreScaler.
    if (nvparams_getsmplintmins()==0)
    {
        // TURBO MODE is enabled.
        // 32 RTC counts per second * 3 seconds = ~101 RTC counts between interrupts.
        RTCMOD = 101-1;
    } else {
        // TURBO MODE is disabled.
        // 32 RTC counts per second * 60 seconds = 1920 RTC counts between interrupts.
        RTCMOD = 1920-1;
    }

    // Enable the RTC peripheral, using the external 32.768kHz crystal divided by 1024 as a clock source.
    RTCCTL = RTCSS__XT1CLK | RTCSR |RTCPS__1024;
    RTCCTL |= RTCIE; // Enable RTC interrupts

    return tr_ok;
}

/*!
 *  @brief Write the NDEF message ::ndefmsg_badtrns to the NFC EEPROM.
 *
 *  Notify the developer that an invalid state machine transition has been requested.
 *  The user should never see this. This will occur if a transition has been
 *  requested that does not exist in the ::state_transitions table.
 */
tretcode err_msg(tevent evt)
{
    /* An invalid state machine transition has been requested
     * i.e. one that is not in the table. You should never reach here.
     * If you do, a text message is written to the NFC EEPROM.  */
    // Write configuration failed NDEF text.
    writetxt(ndefmsg_badtrns, sizeof(ndefmsg_badtrns));
    return tr_deepsleep;
}

/*!
 *  @brief Request a sample from the HDC2021 sensor.
 *
 *  A sample is requested by the MSP430 with hdc2020_startconv(). When data is ready, the HDC2021 makes a HIGH to LOW transition on its INT pin.
 *  To save power, the MSP430 sleeps whilst the measurement is taken.
 *
 *  The MSP430 pin connected to INT is made to raise an interrupt when a falling edge is detected.
 */
tretcode smpl_hdcreq(tevent evt)
{
    /* Enable HDCint P1.1 falling edge interrupt. */
    P1IFG &= ~BIT1; // Clear flag.
    P1IES |= BIT1;  // Falling edge detect.

    hdc2021_startconv();

    /* Enable the interrupt on P1.1 */
    GPIO_enableInterrupt(GPIO_PORT_P1, GPIO_PIN1);

    return tr_ok;
}

/*!
 *  @brief Wait in LPM3 whilst waiting for a data ready interrupt from the HDC2021 sensor.
 *
 *  @return ::tr_ok when a DRDY interrupt has been detected. Otherwise return ::tr_wait to indicate that the MSP430 should sleep.
 */
tretcode smpl_hdcwait(tevent evt)
{
    tretcode rc = tr_wait;

    if (evt == evt_hdcint)
    {
        rc = tr_ok;
    }

    return rc;
}

tretcode smpl_hdcread(tevent evt)
{
    tretcode rc = tr_ok;
    int temp, rh;
    int batv, batv_mv;

    /* Disable HDCint P1.1 rising edge interrupt. */
    GPIO_disableInterrupt(GPIO_PORT_P1, GPIO_PIN1);

    // Read temperature and humidity from the sensor.
    hdc2021_read_temprh(&temp, &rh);

    if (enc_pushsample(temp, rh) > 0)
    {
        // Looping around
        nvparams_cresetsperloop();
        // Check battery voltage
        batv = enc_getbatv();
        batv_mv = batv_to_mv(batv);
        if (batv_mv < nvparams_getminvoltagemv()) {
            enc_init(0, true, batv);    // Write the low power error message

            // Clear samplingloop. This prevents the sampling loop from being re-entered before
            // the error-check routine is run again.
            *(unsigned int *)BKMEM_BASE = 0;

            rc = tr_lowbat;               // Drop into LPM3.5
        }
    }
    nt3h_clearlock();

    return rc;
}

tretcode smpl_wait(tevent evt)
{
    // Set samplingloop=1 to indicate that LPM3.5 is about to be entered from the sampling loop.
    *(unsigned int *)BKMEM_BASE = 1;

    // Enter LPM3.5. This will trigger an LPM5 reset after 1 minute.
    return tr_deepsleep;
}

tretcode smpl_checkcounter(tevent evt)
{
    tretcode rc;
    unsigned int smplintmins = nvparams_getsmplintmins();

    if ((minutecounter == 0) || (smplintmins == 0))
    {
        // Request reading when minute counter is 0 or when in turbo mode.
        rc = tr_hdcreq;
    }
    else
    {
        rc = tr_updatemin;
        enc_setelapsed(minutecounter);
    }


    fram_write_enable();
    if (minutecounter < smplintmins-1)
    {
        minutecounter++;
    }
    else
    {
        minutecounter = 0;
    }
    fram_write_disable();


    return rc;
}

/*!
 *  @brief Put the MSP430 into deep sleep LPM3.5
 *
 *  Minimise power consumption by powering down as much of the MSP430 (and cuplTag)
 *  as possible. Only the Real Time Clock and Backup Memory peripherals remain powered on.
 *
 *  The RTC can generate an interrupt to wake the MSP430 up. When this occurs, the program starts
 *  from a reset condition. Only the backup memory can be used to persist state.
 *
 *  This function disables GPIO interrupt sources. It stops any timers, in case these prevent
 *  LPM3.5 from being entered. It disables the watchdog, because this is power hungry and the
 *  RTC can be used instead (see init_rtc_slow()).
 *
 *  Most importantly it calls memoff() to make sure that the VMEM domain is powered down.
 *
 *  The Supply Voltage Supervisor is disabled to save power. It is not very useful in deep sleep
 *  mode. The battery voltage will decline very little between wake-ups from the RTC.
 *
 *  @return ::tr_ok but this is to keep the compiler happy. Deep sleep is entered before the function returns.
 */
tretcode end_state(tevent evt)
{
    /* Disable HDCint P1.1 rising edge interrupt. */
    GPIO_disableInterrupt(GPIO_PORT_P1, GPIO_PIN1);
    Timer_B_disableInterrupt(TB1_BASE);
    /* Power down the VMEM domain. */
    memoff();
    /* Stop timers. */
    Timer_B_stop(TB0_BASE);
    Timer_B_stop(TB1_BASE);
    /* Hold the watchdog */
    WDTCTL = WDTPW | WDTHOLD;
    // Enter LPM3.5 mode with interrupts enabled. Note that this operation does
    // not return. The LPM3.5 will exit through a RESET event, resulting in a
    // re-start of the code.
    PMMCTL0_H = PMMPW_H;                    // Open PMM Registers for write
    // Disable high-side SVS in deep sleep, which consumes ~240nA.
    // If the battery voltage is too low this will be apparent each minute when the system is active.
    PMMCTL0_L &= ~(SVSHE);
    // and set PMMREGOFF
    PMMCTL0_L |= PMMREGOFF;
    __bis_SR_register(LPM3_bits | GIE);
    __no_operation();

    return tr_ok; // Never reached.
}

void main(void)
{
    volatile int error;
    tstate cur_state = ENTRY_STATE;
    tretcode rc;
    tretcode (* state_fun)(tevent);
    tevent evt;


    while (1) {
        // Process events
        if (timerFlag)
        {
            timerFlag = 0;
            evt = evt_timerfinished;
        }
        else if (hdcFlag)
        {
            hdcFlag = 0;
            evt = evt_hdcint;
        }
        else
        {
            evt = evt_none;
        }

        state_fun = state_fcns[cur_state];
        rc = state_fun(evt);
        if (rc == tr_wait)
        {
            // Sleep in LPM3 whilst waiting for the timer or the HDC2010 to assert it's interrupt output.
            __bis_SR_register(LPM3_bits + GIE);
        }
        cur_state = lookup_transitions(cur_state, rc);
    }
}

//******************************************************************************
//
//This is the TIMER1_A3 interrupt vector service routine.
//
//******************************************************************************
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=TIMER1_B0_VECTOR
__interrupt
#elif defined(__GNUC__)
__attribute__((interrupt(TIMER1_A0_VECTOR)))
#endif
void TIMER1_B0_ISR(void)
{
    Timer_B_stop(TB1_BASE);
    Timer_B_disableInterrupt(TB1_BASE);
    timerFlag = 1;
    __bic_SR_register_on_exit(LPM3_bits); // Clear LPM bits upon ISR Exit
}


//******************************************************************************
//
//This is the PORT_2 interrupt vector service routine.
//
//******************************************************************************
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=PORT1_VECTOR
__interrupt void Port_1(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(PORT2_VECTOR))) Port_2 (void)
#else
#error Compiler not supported!
#endif
{
  P1IFG &= ~BIT1;                           // P4.3 IFG cleared
  if ((P1IN & BIT1) == 0)
  {
      // P1.1 is low.
      hdcFlag = 1;
      __bic_SR_register_on_exit(LPM3_bits);     // Clear LPM bits upon ISR Exit
  }
}

#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector = RTC_VECTOR
__interrupt void RTC_ISR(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(RTC_VECTOR))) RTC_ISR (void)
#else
#error Compiler not supported!
#endif
{
    switch(__even_in_range(RTCIV, RTCIV_RTCIF))
    {
        case RTCIV_NONE : break;            // No interrupt pending
        case RTCIV_RTCIF:                   // RTC Overflow
            break;
        default:          break;
    }
}
