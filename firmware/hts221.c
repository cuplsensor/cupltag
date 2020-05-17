/*
 * hts221.c
 *
 *  Created on: 5 Aug 2017
 *      Author: mmackay
 */

#include "driverlib.h"
#include "hts221.h"

#define MULTIBYTE 0x80

#define HTS221_READ16(devAddr, regAddr) i2c_read16(devAddr, regAddr | MULTIBYTE)

#define DEVADDR             0x5F
#define WHOAMI_REGADDR      0x0F
#define AVCONF_REGADDR      0x10
#define CR1_REGADDR         0x20
#define HUMOUTL_REGADDR     0x28    // Humidity ADC value low
#define HUMOUTH_REGADDR     0x29    // Humidity ADC value high
#define TMPOUTL_REGADDR     0x2A    // Measurement ADC value low
#define TMPOUTH_REGADDR     0x2B    // Measurement ADC value high
#define H0RHX2_REGADDR      0x30    // Humidity cal point 0 relative humidity in %
#define H1RHX2_REGADDR      0x31    // Humidity cal point 1 relative humidity in %
#define T0DEGCX8_REGADDR    0x32    // Calibration point 0 temperature in degrees C
#define T1DEGCX8_REGADDR    0x33    // Calibration point 1 temperature in degrees C
#define T1T0MSB_REGADDR     0x35    // MSBs of temperature in degrees C
#define H0T0OUTL_REGADDR    0x36
#define H0T0OUTH_REGADDR    0x37
#define H1T0OUTL_REGADDR    0x3A
#define H1T0OUTH_REGADDR    0x3B
#define T0OUTL_REGADDR      0x3C    // Calibration point 0 ADC value low
#define T0OUTH_REGADDR      0x3D    // Calibration point 0 ADC value high
#define T1OUTL_REGADDR      0x3E    // Calibration point 1 ADC value low
#define T1OUTH_REGADDR      0x3F    // Calibration point 1 ADC value high

#define WHOAMI_VALUE                    0xBC
#define CR1_PDON_BDUON_ODR1HZ_VALUE     0x85
#define AVCONF_MAXAVG_VALUE             0x3F

/* Global static variables. */
static float t0degc = 0.0;
static float t0out  = 0;
static float degpercount = 0.0;
static float h0t0out = 0.0;
static float h0rh = 0.0;
static float rhpercount = 0.0;
volatile uint16_t testreg = 0;

int hts221_init()
{
    /* Declare all of the single use calibraiton variables. */
    int h0rhx2, h1rhx2;
    int t0degcx8, t1degcx8;
    int t1t0msb;
    volatile float t1degc = 0.0;
    float t1out;
    float h1rh;
    float h1t0out;
    volatile int t0out_i16;
    volatile int t1out_i16;
    volatile int t0degc_i16;
    volatile int t1degc_i16;
    volatile int whoami;

    whoami = hts221_read_whoami();

    /* Turn on the sensor with an ODR of 1Hz. */
    i2c_write8(DEVADDR, CR1_REGADDR, CR1_PDON_BDUON_ODR1HZ_VALUE);
    i2c_write8(DEVADDR, AVCONF_REGADDR, AVCONF_MAXAVG_VALUE);

    /* Read all of the calibration parameters. */
    h0rhx2 = i2c_read8(DEVADDR, H0RHX2_REGADDR);
    h1rhx2 = i2c_read8(DEVADDR, H1RHX2_REGADDR);
    testreg = HTS221_READ16(DEVADDR, H0RHX2_REGADDR);

    t0degcx8 = i2c_read8(DEVADDR, T0DEGCX8_REGADDR);
    t1degcx8 = i2c_read8(DEVADDR, T1DEGCX8_REGADDR);
    t1t0msb = i2c_read8(DEVADDR, T1T0MSB_REGADDR);

    t0out_i16 = HTS221_READ16(DEVADDR, T0OUTL_REGADDR);
    t1out_i16 = HTS221_READ16(DEVADDR, T1OUTL_REGADDR);

    t0out = (float)t0out_i16;
    t1out = (float)t1out_i16;

    h0t0out = (float)HTS221_READ16(DEVADDR, H0T0OUTL_REGADDR);
    h1t0out = (float)HTS221_READ16(DEVADDR, H1T0OUTL_REGADDR);

    /* Store the calibration paramters for later use in calculating temperature. */
    t0degc = ((float)(t0degcx8 | ((t1t0msb & 0x03)<<8)))/8.0;
    t1degc_i16 = t1degcx8|((t1t0msb & 0x0c)<<6);
    t1degc = (float)(t1degc_i16)/8.0;
    degpercount = (t1degc - t0degc) / (t1out - t0out);

    /* Store the calibration parameters for later use in calculating RH. */
    h0rh = (float)h0rhx2/2.0;
    h1rh = (float)h1rhx2/2.0;
    rhpercount = (h1rh - h0rh) / (h1t0out - h0t0out);

    return 0;
}

int hts221_read_whoami()
{
    return i2c_read8(DEVADDR, WHOAMI_REGADDR);
}


float hts221_temp(void)
{
    volatile int tmpout_i16 = 0;

    volatile float tmpout, tdegc;

    /* Read the tmpout registers. */
    tmpout_i16 = HTS221_READ16(DEVADDR, TMPOUTL_REGADDR);
    tmpout = (float)tmpout_i16;

    /* Convert to a temperature in degrees C with cal params. */
    tdegc = t0degc + (tmpout - t0out)*degpercount;

    return tdegc;
}

float hts221_rh(void)
{
    float humout, humrh;

    /* Read the humout registers. */
    humout = (float)HTS221_READ16(DEVADDR, HUMOUTL_REGADDR);

    /* Convert to an RH in % with cal params. */
    humrh = h0rh + (humout - h0t0out)*rhpercount;

    return humrh;
}



