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
 * @file hdc2021.c
 * @author Malcolm Mackay
 *
 * @brief A driver for the <a href="https://www.ti.com/product/HDC2021">Texas Instruments HDC2021</a> temperature and humidity sensor.
 *
 * Only the basic functionality is needed - to start the sensor in one-shot mode and
 * collect a temperature and humidity measurement.
 *
 * It is compatible with the <a href="https://www.ti.com/product/HDC2022">HDC2022</a> and <a href="https://www.ti.com/product/HDC2080">HDC2080</a> parts, which are electrically identical.
 *
 */

#include "driverlib.h"
#include "hdc2021.h"




#define TEMPL_REGADDR               0x00    /*!< Temperature Low register address. */
#define TEMPH_REGADDR               0x01    /*!< Temperature High register address. */
#define HUML_REGADDR                0x02    /*!< Humidity Low register address. */
#define HUMH_REGADDR                0x03    /*!< Humidity High register address. */
#define STAT_REGADDR                0x04    /*!< Status register address. */

#define INTEN_REGADDR               0x07    /*!< Interrupt enable register address. */
#define TEMPOFFSETADJ_REGADDR       0x08    /*!< Temperature offset adjustment register address. */
#define HUMOFFSETADJ_REGADDR        0x09    /*!< Humidity offset adjustment register address. */
#define DEVCONF_REGADDR             0x0E    /*!< Device configuration (soft-reset and interrupt reporting) register address. */
#define MEASCONF_REGADDR            0x0F    /*!< Measurement configuration register address. */

#define MANFIDL_REGADDR             0xFC    /*!< Manufacturer ID low-byte address. */
#define MANFIDH_REGADDR             0xFD    /*!< Manufacturer ID high-byte address. */
#define DEVIDL_REGADDR              0xFE    /*!< Device ID low-byte address. */
#define DEVIDH_REGADDR              0xFF    /*!< Device ID high-byte address. */

#define MEASCONF_MEAS_TRIG_BIT      0x01    /*!< Trigger a measurement when applied to ::MEASCONF_REGADDR */
#define STAT_DRDY_STATUS_BIT        0x80    /*!< Select the DRDY_STATUS bit from ::STAT_REGADDR */
#define INTEN_DRDYEN_BIT            0x80    /*!< Enable the DataReady Interrupt when applied to ::INTEN_REGADDR */
#define DEVCONF_SOFT_RES_BIT        0x80    /*!< Trigger a soft reset when applied to ::DEVCONF_REGADDR */
#define DEVCONF_DRDY_INTEN_BIT      0x04    /*!< Enable the DRDY/INTEN pin when applied to ::DEVCONF_REGADDR */

/*!
 * @brief Trigger a one-shot conversion.
 *
 * The HDC2021 is in one-shot mode. This function sends an I2C command
 * to trigger the temperature and humidity measurement.
 *
 * It also configures the interrupt pin on HDC2021 to make a
 * high-to-low transition when the conversion data are ready. This takes
 * ~500us, so the MSP430 can sleep in this time to save power.
 *
 */
int hdc2021_startconv()
{
    int err = 0;
    int measconf = 0;

    // Enable interrupt pin, active low.
    i2c_write8(HDC_DEVADDR, DEVCONF_REGADDR, DEVCONF_DRDY_INTEN_BIT);
    // Enable DRDY interrupt
    i2c_write8(HDC_DEVADDR, INTEN_REGADDR, INTEN_DRDYEN_BIT);
    // Clear any existing interrupt by reading the status register.
    i2c_read8(HDC_DEVADDR, STAT_REGADDR);

    // Trigger a measurement by setting the MEAS_TRIG bit in the MEASCONF register.
    measconf |= MEASCONF_MEAS_TRIG_BIT;
    i2c_write8(HDC_DEVADDR, MEASCONF_REGADDR, measconf);

    return err;
}

/*!
 * @brief Initialise the HDC2021 in one-shot mode.
 */
int hdc2021_init()
{
    int devconf = 0;

    // Set the Soft Reset bit. Keep the CC bits clear to enable one-shot mode.
    devconf = DEVCONF_SOFT_RES_BIT;
    // Write to the DEVCONF register.
    i2c_write8(HDC_DEVADDR, DEVCONF_REGADDR, devconf);

    return 0;
}

/*!
 * @brief Read temperature and humidity from the HDC2021.
 *
 * @param[out] temp12b Pointer to a variable for storing the raw 12-bit temperature.
 * @param[out] rh12b Pointer to a variable for storing the raw 12-bit relative humidity.
 */
uint32_t hdc2021_read_temp(int * temp12b, int * rh12b)
{
    uint32_t temp16b = 0;
    uint32_t rh16b = 0;

    // Populate temp16b and rh16b with a 32-bit I2C read starting from TEMPL_REGADDR.
    i2c_read32(HDC_DEVADDR, TEMPL_REGADDR, (uint16_t *)&temp16b, (uint16_t *)&rh16b);

    // Remove the 4 most-significant bits from the relative humidity reading.
    rh16b = rh16b >> 4;
    *rh12b = rh16b;

    // Remove the 4 most-signficant bits from the temperature reading.
    temp16b = temp16b >> 4;
    *temp12b = temp16b;

    return 0;
}

/*!
 * @brief Read the Device ID registers.
 *
 * @return A 16-bit Device ID. The expected value is 0x07D0.
 */
int hdc2021_read_whoami()
{
    volatile int test = 100;
    test = i2c_read16(HDC_DEVADDR, DEVIDL_REGADDR);
    return test;
}
