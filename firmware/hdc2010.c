/*
 * hdc2010.c
 *
 *  Created on: 4 Nov 2017
 *      Author: mmackay
 */

#include "driverlib.h"
#include "hdc2010.h"


#define DEVADDR                     0x40

#define TEMPL_REGADDR               0x00
#define TEMPH_REGADDR               0x01
#define HUML_REGADDR                0x02
#define HUMH_REGADDR                0x03
#define INT_REGADDR                 0x04

#define TEMPMAX_REGADDR             0x05
#define HUMMAX_REGADDR              0x06
#define INTEN_REGADDR              0x07
#define TEMPOFFSETADJ_REGADDR       0x08
#define HUMOFFSETADJ_REGADDR        0x09
#define TEMPTHRL_REGADDR            0x0A
#define TEMPTHRH_REGADDR            0x0B
#define RHTHRH_REGADDR              0x0D
#define RHTHRL_REGADDR              0x0C
#define RSTDRDYINTCONF_REGADDR      0x0E
#define MEASCONF_REGADDR            0x0F

#define MANFIDL_REGADDR             0xFC
#define MANFIDH_REGADDR             0xFD
#define DEVIDL_REGADDR              0xFE
#define DEVIDH_REGADDR              0xFF

#define MEAS_TRIG_BIT               0x01
#define DRDY_STATUS_BIT             0x80

#define INTEN_DRDYEN_BIT            0x80

#define DRDY_SOFT_RES_BIT           0x80
#define DRDY_ODR_NOREPEAT_BITS      0x00
#define DRDY_ODR_5HZ_BITS           0x70
#define DRDY_HEATER_BIT             0x08
#define DRDY_INTEN_BIT              0x04
#define DRDY_INTPOL_BIT             0x02

int hdc2010_startconv()
{
    int err = 0;
    int measconf = 0;

    // Enable interrupt pin, active high.
    i2c_write8(DEVADDR, RSTDRDYINTCONF_REGADDR, DRDY_INTEN_BIT);
    // Enable DRDY interrupt
    i2c_write8(DEVADDR, INTEN_REGADDR, INTEN_DRDYEN_BIT);

    measconf |= MEAS_TRIG_BIT;

    i2c_write8(DEVADDR, MEASCONF_REGADDR, measconf);

    return err;
}

int hdc2010_init()
{
    int rstdrdy = 0;

    rstdrdy = DRDY_SOFT_RES_BIT;

    i2c_write8(DEVADDR, RSTDRDYINTCONF_REGADDR, rstdrdy);

    return 0;
}

uint32_t hdc2010_read_temp(int * tempdeg, int * rh)
{
    uint32_t temp = 0;
    uint32_t hum = 0;
    i2c_read32(DEVADDR, TEMPL_REGADDR, (uint16_t *)&temp, (uint16_t *)&hum);


    //hum = hum * 100L;
    hum = hum >> 4;
    *rh = hum;

    //temp = temp * 165L;
    temp = temp >> 4; // Was >> 16

    *tempdeg = temp;


    return 0;
}

uint32_t hdc2010_read_humidity()
{
    uint16_t humrh;
    uint32_t humidity = 0;
    humidity = i2c_read8(DEVADDR, HUMH_REGADDR);
    humidity <<= 8;
    humidity |= i2c_read8(DEVADDR, HUML_REGADDR);
    humidity = humidity * 100L;
    humidity = humidity >> 16;

    humrh = (uint32_t)humidity;
    return humrh;
}

int hdc2010_read_whoami()
{
    volatile int test = 100;
    test = i2c_read16(DEVADDR, DEVIDL_REGADDR);
    return test;
}
