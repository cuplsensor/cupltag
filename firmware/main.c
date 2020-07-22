#include "driverlib.h"
#include "hdc2010.h"
#include "sample.h"
#include "comms_uart.h"
#include "confignfc.h"
#include "nt3h.h"
#include "stat.h"
#include "defs.h"
#include "eep.h"
#include <stdlib.h>


#define CS_SMCLK_DESIRED_FREQUENCY_IN_KHZ   16000       /*!< Target frequency for SMCLK in kHz. */

#define CS_XT1_CRYSTAL_FREQUENCY            32768       /*!< Resonant frequency of the XT1 crystal in kHz. */
#define CS_XT1_TIMEOUT                      65000       /*!< Timeout for XT1 to stabilise at the resonant frequency in SMCLK cycles. */

#define CP10MS                                  41      /*!< ACLK Cycles Per 10 MilliSeconds. Assumes ACLK = 32768 kHz and a divide-by-8. */

#define EXIT_STATE sc_end                               /*!< State machine exit state. */
#define ENTRY_STATE sc_init                             /*!< State machine entry state. */

volatile int rtcFlag = 0;                               /*!< Flag set by the Real-Time-Clock Interrupt Service Route. */
volatile int timerFlag = 0;                             /*!< Flag set by the Timer Interrupt Service Routine. */
volatile int hdcFlag = 0;                               /*!< Flag set by the HDC2021 humidity sensor data-ready Interrupt Service Routine. */

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


typedef enum state_codes {
    sc_init,
    sc_init_reqmemon,
    sc_init_waitmemon,
    sc_init_ntag,
    sc_init_progmode,
    sc_init_configcheck,
    sc_init_errorcheck,
    sc_init_rtc,
    sc_smpl_checkcounter,
    sc_smpl_hdcreq,
    sc_smpl_hdcwait,
    sc_smpl_hdcread,
    sc_smpl_wait,
    sc_rtc_reqmemon,
    sc_rtc_waitmemon,
    sc_err_reqmemon,
    sc_err_waitmemon,
    sc_err_msg,
    sc_end
} tstate;

typedef enum ret_codes { 
    tr_ok,
    tr_prog,
    tr_hdcreq,
    tr_updatemin,
    tr_fail,
    tr_timeout,
    tr_wait
} tretcode;

typedef enum event_codes {
    evt_none,
    evt_rtc,
    evt_timerfinished,
    evt_hdcint
} tevent;

// https://stackoverflow.com/questions/1371460/state-machines-tutorials
tretcode init_state(tevent);
tretcode init_reqmemon(tevent);
tretcode init_waitmemon(tevent);
tretcode init_ntag(tevent);
tretcode init_progmode(tevent);
tretcode init_configcheck(tevent);
tretcode init_errorcheck(tevent);
tretcode init_rtc(tevent);
tretcode smpl_checkcounter(tevent);
tretcode smpl_hdcreq(tevent);
tretcode smpl_hdcwait(tevent);
tretcode smpl_hdcread(tevent);
tretcode smpl_wait(tevent);
tretcode rtc_reqmemon(tevent);
tretcode rtc_waitmemon(tevent);
tretcode err_reqmemon(tevent);
tretcode err_waitmemon(tevent);
tretcode err_msg(tevent);
tretcode end_state(tevent);

/* array and enum below must be in sync! */
tretcode (* state_fcns[])(tevent) = {
                          init_state,
                          init_reqmemon,
                          init_waitmemon,
                          init_ntag,
                          init_progmode,
                          init_configcheck,
                          init_errorcheck,
                          init_rtc,
                          smpl_checkcounter,
                          smpl_hdcreq,
                          smpl_hdcwait,
                          smpl_hdcread,
                          smpl_wait,
                          rtc_reqmemon,
                          rtc_waitmemon,
                          err_reqmemon,
                          err_waitmemon,
                          err_msg,
                          end_state
};


struct transition {
    tstate      src_state;
    tretcode    ret_code;
    tstate      dst_state;
};

struct transition state_transitions[] = {
                                         {sc_init,          tr_ok,      sc_init_reqmemon},

                                         {sc_init_reqmemon, tr_ok,      sc_init_waitmemon},

                                         {sc_init_waitmemon, tr_ok,      sc_init_ntag},
                                         {sc_init_waitmemon, tr_wait,    sc_init_waitmemon},

                                         {sc_init_ntag,      tr_ok,      sc_init_configcheck},
                                         {sc_init_ntag,      tr_prog,    sc_init_progmode},

                                         {sc_init_progmode,  tr_ok,      sc_init_progmode},
                                         {sc_init_progmode,  tr_wait,    sc_init_progmode},
                                         {sc_init_progmode,  tr_fail,    sc_err_reqmemon},

                                         {sc_init_configcheck,  tr_ok,   sc_init_errorcheck},
                                         {sc_init_configcheck,         tr_fail, sc_end},

                                         {sc_init_errorcheck,  tr_ok,      sc_init_rtc},
                                         {sc_init_errorcheck,  tr_fail,    sc_end},

                                         {sc_init_rtc,      tr_ok,      sc_smpl_checkcounter},

                                         {sc_smpl_checkcounter, tr_hdcreq,      sc_smpl_hdcreq},
                                         {sc_smpl_checkcounter, tr_updatemin,   sc_smpl_wait},

                                         {sc_smpl_hdcreq,   tr_ok,      sc_smpl_hdcwait},

                                         {sc_smpl_hdcwait,  tr_ok,      sc_smpl_hdcread},
                                         {sc_smpl_hdcwait,  tr_wait,    sc_smpl_hdcwait},

                                         {sc_smpl_hdcread,  tr_ok,      sc_smpl_wait},

                                         {sc_smpl_wait,     tr_timeout,     sc_rtc_reqmemon},
                                         {sc_smpl_wait,     tr_wait,        sc_smpl_wait},

                                         {sc_rtc_reqmemon,  tr_ok,      sc_rtc_waitmemon},



                                         {sc_rtc_waitmemon, tr_ok, sc_smpl_checkcounter},
                                         {sc_rtc_waitmemon, tr_wait,   sc_rtc_waitmemon},

                                         {sc_err_reqmemon,  tr_ok,     sc_err_waitmemon},

                                         {sc_err_waitmemon, tr_ok,      sc_err_msg},
                                         {sc_err_waitmemon, tr_wait,    sc_err_waitmemon},

                                         {sc_err_msg,  tr_ok,     sc_end},

                                         
};

/* Look up transitions from the table for a current state and given return code. */
tstate lookup_transitions(tstate curstate, tretcode rc)
{
    tstate nextstate = sc_err_reqmemon; /* This should never be reached. */
    int i=0;

    for (i=0; i<sizeof(state_transitions)/sizeof(state_transitions[0]); i++)
    {
        if ((state_transitions[i].src_state == curstate) && (state_transitions[i].ret_code == rc))
        {
            nextstate = state_transitions[i].dst_state;
            break;
        }
    }

    return nextstate;
}

static void writetxt(char * msgptr, char len) {
    int eepindex = 0;
    int blk;

    eep_cp(&eepindex, msgptr, len);
    for (blk=0; blk<4; blk++) {
        eep_write(blk, blk);
    }
    eep_waitwritedone();
}

static void wdog_kick()
{
    WDTCTL = WDTPW | WDTSSEL__ACLK | WDTCNTCL | WDTIS__8192K;
}

static void start_timer(unsigned int intervalCycles)
{
    // Start timer in continuous mode sourced by SMCLK
        Timer_B_initContinuousModeParam initContParam = {0};
        initContParam.clockSource = TIMER_B_CLOCKSOURCE_ACLK;
        initContParam.clockSourceDivider = TIMER_B_CLOCKSOURCE_DIVIDER_8;
        initContParam.timerInterruptEnable_TBIE = TIMER_B_TBIE_INTERRUPT_DISABLE;
        initContParam.timerClear = TIMER_B_DO_CLEAR;
        initContParam.startTimer = false;
        Timer_B_initContinuousMode(TB1_BASE, &initContParam);

        //Initialise compare mode
        Timer_B_clearCaptureCompareInterrupt(TB1_BASE, TIMER_B_CAPTURECOMPARE_REGISTER_0);

        Timer_B_initCompareModeParam initCompParam = {0};
        initCompParam.compareRegister = TIMER_B_CAPTURECOMPARE_REGISTER_0;
        initCompParam.compareInterruptEnable = TIMER_B_CAPTURECOMPARE_INTERRUPT_ENABLE;
        initCompParam.compareOutputMode = TIMER_B_OUTPUTMODE_OUTBITVALUE;
        initCompParam.compareValue = intervalCycles;
        Timer_B_initCompareMode(TB1_BASE, &initCompParam);

        Timer_B_startCounter(TB1_BASE, TIMER_B_CONTINUOUS_MODE);
}


static void memoff()
{
    // P4.2 EN as output low.
    GPIO_setOutputLowOnPin(
            GPIO_PORT_P4,
            GPIO_PIN2
    );
}

// sensorinit -> memon. FDint OFF. RTCint OFF.
// waitforserial -> memoff. FDint ON. RTCint OFF.
// reqsensor -> memon. FDint ON. HDCint ON. RTCint OFF.
// updateurl -> memon. FDint ON. RTCint OFF.
// updatesc -> memon. FDint OFF. RTCint ON. wait a few seconds before leaving this state as hysteresis.
// waitrtc -> memoff. FDint ON. RTCint ON.

tretcode init_state(tevent evt)
{
    int error;

    WDTCTL = WDTPW | WDTHOLD;               // Stop WDT
    PMM_disableSVSH(); // Disable Supply Voltage Supervisor.

    //Set wait state to 1
    FRAMCtl_configureWaitStateControl(FRAMCTL_ACCESS_TIME_CYCLES_1);

    // Initialise IO to reduce power.
    // P1.2 FD as input
    // P1.1 UART RX as input
    // P4.2 HDC_INT as input
    // P2.7 EN as output low
    P1DIR = 0x3F; P2DIR = 0x3F; P3DIR = 0xDF; P4DIR = 0x00;
    P5DIR = 0xFF; P6DIR = 0xFF; P7DIR = 0xFF; P8DIR = 0xFF;
    P1OUT = 0x00; P2OUT = 0x00; P3OUT = 0x00; P4OUT = 0x00;
    P5OUT = 0x00; P6OUT = 0x00; P7OUT = 0x00; P8OUT = 0x00;

    P2SEL1 |= BIT6 | BIT7;                  // P2.6~P2.7: crystal pins

    // Disable the GPIO power-on default high-impedance mode
    // to activate previously configured port settings
    PM5CTL0 &= ~LOCKLPM5;

    // Read the reset cause.
    stat_rdrstcause();

    //Initializes the XT1 and XT2 crystal frequencies being used
    CS_setExternalClockSource(CS_XT1_CRYSTAL_FREQUENCY);

    //Initialize XT1. Returns STATUS_SUCCESS if initializes successfully
    error = CS_turnOnXT1LFWithTimeout(CS_XT1_DRIVE_0, CS_XT1_TIMEOUT);

    if (error == STATUS_FAIL)
    {
        stat_setclockfailure();
    }

    CS_initClockSignal(
            CS_ACLK,
            CS_XT1CLK_SELECT,
            CS_CLOCK_DIVIDER_1
    );

    //Set DCO FLL reference = REFO
    CS_initClockSignal(
            CS_FLLREF,
            CS_XT1CLK_SELECT,
            CS_CLOCK_DIVIDER_1
    );


    //Set Ratio and Desired MCLK Frequency  and initialize DCO
    error = CS_initFLLSettle(
                CS_SMCLK_DESIRED_FREQUENCY_IN_KHZ,
                487
    );


    if (error == STATUS_FAIL)
    {
        stat_setclockfailure();
    }

    //
    //Set SMCLK = DCO with frequency divider of 1
    CS_initClockSignal(
            CS_SMCLK,
            CS_DCOCLKDIV_SELECT,
            CS_CLOCK_DIVIDER_1
    );
    //
    //Set MCLK = DCO with frequency divider of 1
    CS_initClockSignal(
            CS_MCLK,
            CS_DCOCLKDIV_SELECT,
            CS_CLOCK_DIVIDER_1
    );

    // Enable watchdog timer.
    wdog_kick();

    // P3.5 nPRG as input
    GPIO_setAsInputPin(
            GPIO_PORT_P3,
            GPIO_PIN5
    );

    // P1.6 UART RX as input
    GPIO_setAsInputPin(
            GPIO_PORT_P1,
            GPIO_PIN6
    );


    return tr_ok;
}

static tretcode reqmemon(tevent evt)
{
    /* Power up the I2C bus. */
    // P4.3 HDC_INT as input
    GPIO_setAsInputPin(
            GPIO_PORT_P4,
            GPIO_PIN3
    );

    // P4.2 EN as output high
    GPIO_setAsOutputPin(
            GPIO_PORT_P4,
            GPIO_PIN2
    );

    GPIO_setOutputHighOnPin(
            GPIO_PORT_P4,
            GPIO_PIN2
    );


    start_timer(2*CP10MS);

    return tr_ok;
}

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

tretcode init_reqmemon(tevent evt)
{
    return reqmemon(evt);
}

tretcode init_waitmemon(tevent evt)
{
    return waitmemon(evt);
}

tretcode init_ntag(tevent evt)
{
    tretcode rc;
    int nPRG;

    // Checks for an NFC text record.
    if (confignfc_check())
    {
        confignfc_readtext(); // Configure from text records.
    }


    /* Initialise the NTAG. */
    nt3h_init();

    /* Read the whoami registers of both the NT3H and the HDC2010. */

    /* Check nPRG. */
    nPRG = GPIO_getInputPinValue(GPIO_PORT_P3, GPIO_PIN5);

    if (nPRG) {
        rc = tr_ok;
    } else {
        // Write programming mode NDEF text.
        writetxt(ndefmsg_progmode, sizeof(ndefmsg_progmode));
        rc = tr_prog;
    }

    return rc;
}

tretcode init_progmode(tevent evt)
{
    tretcode rc;
    t_ustat uartstatus;

    // Kick the watchdog.
    wdog_kick();

    uartstatus = uart_run();

    if (uartstatus == ustat_waiting)
    {
        rc = tr_wait;
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
        rc = tr_ok;
    }

    return rc;
}



tretcode init_configcheck(tevent evt)
{
    tretcode rc = tr_fail;

    if (nvparams_allwritten()) {
        rc = tr_ok;
    }
    else {
        // Write configuration failed NDEF text.
        writetxt(ndefmsg_noconfig, sizeof(ndefmsg_noconfig));
    }

    return rc;
}

tretcode init_errorcheck(tevent evt)
{
    tretcode rc;
    bool err = false;
    unsigned int status;
    int resetsalltime, resetsperloop;

    nvparams_incrcounters(); // Increment reset counter.
    resetsalltime = nvparams_getresetsalltime();
    resetsperloop = nvparams_getresetsperloop();

    // Check for low battery here.
    status = stat_get(&err, resetsalltime);

    if (err == false)
    {
        // Clear the resets counter if the reset has been caused intentionally (e.g. by the user replacing the battery).
        // The resets counter should only count unintended resets (e.g. due to brown out or the watchdog).
        // Without this behaviour, the Tag could get stuck in an error state.
        nvparams_cresetsperloop();
    }

    if (resetsperloop < 10)
    {
        // Fewer than 10 consecutive unintended resets due to an error. Try to keep going.
        wdog_kick();
        enc_init(status, false);
        rc = tr_ok;
    }
    else
    {
        // More than 10 consecutive unintended resets before reaching the end of the circular buffer. Stop.
        enc_init(status, true); // Do not write the full URL.
        rc = tr_fail; // Go to LPM4. This prevents wearing out the EEPROM after repeated resets.
    }

    return rc;
}

tretcode init_rtc(tevent evt)
{
    // Enable Supply voltage supervisor
    PMM_enableSVSH();

    // Configure RTC
    // Interrupt and reset happen every 1024/32768 * 32 = 1 sec.
    RTCMOD = 1920-1;
    RTCCTL = RTCSS__XT1CLK | RTCSR |RTCPS__1024;
    RTCCTL |= RTCIE;

    return tr_ok;
}

tretcode err_reqmemon(tevent evt)
{
    /* An invalid state machine transition has been requested
     * i.e. one that is not in the table. You should never reach here.
     * If you do, a text message is written to the  */
    /* Power up the I2C bus if it is not already on. */
    return reqmemon(evt);
}

tretcode err_waitmemon(tevent evt)
{
    return waitmemon(evt);
}

tretcode err_msg(tevent evt)
{
    // Write configuration failed NDEF text.
    writetxt(ndefmsg_badtrns, sizeof(ndefmsg_badtrns));
    return tr_ok;
}

tretcode smpl_hdcreq(tevent evt)
{
    /* Enable HDCint P4.2 rising edge interrupt. */
    P4IFG &= ~BIT3; // Clear flag.
    P4IES |= BIT3; // Falling edge detect.

    hdc2010_startconv();

    P4IE |= BIT3;  // Allow interrupt.

    return tr_ok;
}

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
    int temp, rh;

    /* Disable HDCint P4.3 rising edge interrupt. */
    P4IE &= ~BIT3 ; // Disable interrupt on port 2 bit 3.

    // Read temperature and humidity from the sensor.
    hdc2010_read_temp(&temp, &rh);

    if (enc_pushsample(temp, rh) > 0)
    {
        // Looping around
        nvparams_cresetsperloop();
    }

    /* Power down the MEM domain. */
    memoff();

    return tr_ok;
}



tretcode smpl_wait(tevent evt)
{
    tretcode rc;

    // Kick the watchdog.
    wdog_kick();

    if (evt == evt_rtc)
    {
        rc = tr_timeout;
    }
    else
    {
        rc = tr_wait;
    }

    return rc;
}

tretcode smpl_checkcounter(tevent evt)
{
    tretcode rc = tr_fail;
    static int minutecounter = 0;

    if (minutecounter == 0)
    {
        rc = tr_hdcreq;
    }
    else
    {
        rc = tr_updatemin;
        enc_setelapsed(minutecounter);
        // Check for a configuration text record.
        if (confignfc_check())
        {
            PMMCTL0 = PMMPW | PMMSWPOR; // Reset if a text record has been found.
        }

        /* Power down peripherals on the I2C bus. */
        memoff();
    }

    if (minutecounter < nvparams_getsmplintmins()-1)
    {
        minutecounter++;
    }
    else
    {
        minutecounter = 0;
    }

    return rc;
}

tretcode rtc_reqmemon(tevent evt)
{
    return reqmemon(evt);
}

tretcode rtc_waitmemon(tevent evt)
{
    return waitmemon(evt);
}

tretcode end_state(tevent evt)
{
    // Turn peripheral power off.
    memoff();
    /* Disable HDCint P4.2 rising edge interrupt. */
    P4IE = 0 ; // Disable interrupt on port 2 bit 3.
    // Go to deep sleep mode.
    WDTCTL = WDTPW | WDTHOLD;        // Hold the watchdog.
    PMMCTL0_H = PMMPW_H;                // Open PMM Registers for write
    PMMCTL0_L &= ~(SVSHE);              // Disable high-side SVS
    PMMCTL0_L |= PMMREGOFF;             // and set PMMREGOFF
    PMMCTL0_H = 0;                      // Lock PMM Registers
    TB0CTL = MC_0;                   //turn off timer A
    TB1CTL = MC_0;                   //turn off timer A
    RTCCTL = 0;                      // Stop the RTC.
    __delay_cycles(10000);
    // Deep sleep.
    __bis_SR_register(LPM4_bits | GIE);
    return tr_ok; // Never reached.
}

static tevent evt;

void main(void)
{
    volatile int error;
    tstate cur_state = ENTRY_STATE;
    tretcode rc;
    tretcode (* state_fun)(tevent);




    while (1) {
        // Process events
        if (rtcFlag)
        {
            rtcFlag = 0;
            evt = evt_rtc;
        }
        else if (timerFlag)
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
            // Sleep whilst waiting for an eventconf
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
#pragma vector=PORT4_VECTOR
__interrupt void Port_4(void)
#elif defined(__GNUC__)
void __attribute__ ((interrupt(PORT2_VECTOR))) Port_2 (void)
#else
#error Compiler not supported!
#endif
{
  P4IFG &= ~BIT3;                           // P4.3 IFG cleared
  if ((P4IN & BIT3) == 0)
  {
      // P4.3 is low.
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
            rtcFlag = 1;
            __bic_SR_register_on_exit(LPM3_bits); // Clear LPM bits upon ISR Exit
            break;
        default:          break;
    }
}
