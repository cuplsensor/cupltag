/*
 * batv.c
 *
 *  Created on: 6 Jan 2018
 *      Author: malcolm
 */

#include "driverlib.h"
#include "batv.h"

volatile unsigned int adcvoltage = 0;

static void adc_enable()
{

    // Configure ADC10
    ADCCTL0 &= ~ADCENC;                     // Disable ADC
    ADCCTL0 = ADCSHT_2 | ADCON;             // ADCON, S&H=16 ADC clks
    ADCCTL1 = ADCSHP;                       // ADCCLK = MODOSC; sampling timer
    ADCCTL2 = ADCRES_0;                     // 8-bit conversion results
    ADCIE = ADCIE0;                         // Enable ADC conv complete interrupt
    ADCMCTL0 = ADCINCH_13 | ADCSREF_0;      // A13 ADC input select = 1.5V Ref Vref = DVCC
    // Configure reference module located in the PMM
    PMMCTL0_H = PMMPW_H;                    // Unlock the PMM registers
    PMMCTL2 = INTREFEN | REFVSEL_0;        // Enable internal 1.5V reference
    while(!(PMMCTL2 & REFGENRDY));          // Poll till internal reference settles
}

static void adc_disable()
{
    ADC_disable(ADC_BASE);

    //Disable Memory Buffer interrupt
    ADC_disableInterrupt(ADC_BASE,
                        ADC_COMPLETED_INTERRUPT);

    //Internal Reference OFF
    PMM_disableInternalReference();
}


unsigned int batv_measure()
{
    adc_enable();

    //Enable and Start the conversion
    //in Single-Channel, Single Conversion Mode
    ADC_startConversion(ADC_BASE, ADC_SINGLECHANNEL);

    //LPM0, ADC_ISR will force exit
    __bis_SR_register(CPUOFF + GIE);

    adc_disable();

    return adcvoltage;
}

unsigned int batv_to_mv(unsigned int batv)
{
    return ((uint32_t)255 * (uint32_t)1500)/batv;
}

//******************************************************************************
//
//This is the ADC interrupt vector service routine.
//
//******************************************************************************
#if defined(__TI_COMPILER_VERSION__) || defined(__IAR_SYSTEMS_ICC__)
#pragma vector=ADC_VECTOR
__interrupt
#elif defined(__GNUC__)
__attribute__((interrupt(ADC_VECTOR)))
#endif
void ADC_ISR (void)
{
    uint32_t adcresult;
    switch (__even_in_range(ADCIV,12)){
        case  0: break; //No interrupt
        case  2: break; //conversion result overflow
        case  4: break; //conversion time overflow
        case  6: break; //ADCHI
        case  8: break; //ADCLO
        case 10: break; //ADCIN
        case 12:        //ADCIFG0
            //Automatically clears ADCIFG0 by reading memory buffer
            //ADCMEM = A0 > 0.5V?
            adcresult = ADC_getResults(ADC_BASE);
            //adcresult = ((uint32_t)1024 * (uint32_t)1500)/adcresult;
            adcvoltage = adcresult;

            //Clear CPUOFF bit from 0(SR)
            //Breakpoint here and watch ADC_Result
            __bic_SR_register_on_exit(CPUOFF);
            break;
        default: break;
    }
}
