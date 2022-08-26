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
 * @file nvparams.c
 *
 * @brief Reads and writes parameters from non-volatile memory (FRAM).
 *
 * The cuplTag (and cuplcodec) is configured with a small set of parameters. These control
 * the tag serial string, the sampling interval or the URL of the web application that decodes the tag contents.
 *
 * A parameter has a single-byte identifier e.g. 'w' and a value. The length of the value field
 * depends on the parameter. For example, the serial string consists of eight bytes (e.g. 'AB43xkp4').
 *
 * A variable is used to monitor how many parameters have been written since the last power cycle.
 * A full set of parameters is needed for the program to proceed.
 *
 */

#include "nvparams.h"
#include "nvtype.h"
#include <string.h>
#include <msp430.h>
#include <stdlib.h>

#define NVPARAM_SERIAL_ID       'w'       /*!< Serial ID */
#define NVPARAM_SECKEY_ID       's'       /*!< Secret key ID */
#define NVPARAM_BASEURL_ID      'b'       /*!< Base URL of the cupl web application ID */
#define NVPARAM_FMT_ID          'f'       /*!< Sample format ID */
#define NVPARAM_SMPLINT_ID      't'       /*!< Sample interval ID */
#define NVPARAM_MINVOLT_ID      'u'       /*!< Minimum operating voltage (in mV) ID */
#define NVPARAM_HTTPSDIS_ID     'h'       /*!< Disable HTTPS ID */
#define NVPARAM_USEHMAC_ID      'i'       /*!< Use HMAC ID */

#define SERIAL_PARAM_WRITTEN    BIT0
#define SECKEY_PARAM_WRITTEN    BIT1
#define FMT_PARAM_WRITTEN       BIT2
#define SMPLINT_PARAM_WRITTEN   BIT3
#define MINVOLT_PARAM_WRITTEN   BIT4
#define BASEURL_PARAM_WRITTEN   BIT5
#define HTTPSDIS_PARAM_WRITTEN  BIT6
#define USEHMAC_PARAM_WRITTEN   BIT7

#define ALL_PARAMS_WRITTEN      0xFF      /*!< 0xFF in the 'paramswritten' RAM variable indicates that all parameters have been written. */
#define NVM_ALL_PARAMS_WRITTEN  0x00      /*!< Zero in NVM indicates that all parameters have been written. Why zero? After programming, the initial value for this NVM section is 0xFF. */

#define MINUTES_PER_DAY         1440
#define INTEGERFIELD_LENBYTES   5           /*!< Value up to 65535 (5 ASCII digits) */

#define DISABLE_FRAM_DATA_WRITEPROTECT  (SYSCFG0 = FRWPPW | PFWP)          /*!< Clear the Data FRAM Write Protect bit. */
#define ENABLE_FRAM_DATA_WRITEPROTECT   (SYSCFG0 = FRWPPW | DFWP | PFWP)   /*!< Set the Data FRAM Write Protect bit. */

#pragma DATA_SECTION(nv, ".info");
nv_t nv;

unsigned int paramswritten = 0;           /*!< Bits are set in this integer that correspond to parameters written since the last power cycle. */

/*!
 *  @brief Get the 8-character alphanumeric serial string.
 *
 *  This is used to identify a cuplTag to the server. It is included in
 *  the URL.
 *
 *  @return A pointer to the serial string.
 */
char * nvparams_getserial()
{
    return nv.serial;
}

/*!
 *  @brief Get the secret key used by cuplcodec to calculate an HMAC-MD5.
 *
 *  The secret key is unique per tag. It is known only to the web server and the tag.
 *  It is used to generate a Hash based Message Authenticity Code, which prevents an
 *  'imposter tag' from writing sample data to the web server.
 *
 *  @return A pointer to the secret key, which is SECKEY_LENBYTES long.
 */
char * nvparams_getsecretkey()
{
    return nv.seckey;
}

/*!
 *  @brief Get the minimum operating voltage parameter.
 *
 *  If the battery voltage is allowed to drop to the brown-out voltage, then the NFC EEPROM will be left with stale data.
 *  There will be insufficient power to overwrite this with a 'low power' message, because the MSP430 will be
 *  stuck in a reset loop.
 *
 *  This parameter should be slightly higher than the brown-out voltage e.g. 2200mV. When it is reached,
 *  the sampling loop will stop, sample data removed and the user will be notified.
 *
 *  @return Minimum operating voltage in millivolts.
 */
unsigned int nvparams_getminvoltagemv(void)
{
    return nv.minvoltagemv;
}

/*!
 *  @brief Get the sample interval in minutes.
 *
 *  Temperature/humidity sensor samples are written to the circular buffer at this interval.
 *
 *  @return The sample interval in minutes as a 16-bit unsigned integer.
 */
unsigned int nvparams_getsmplintmins()
{
    return ((unsigned int)nv.smplintervalmins[1] << 8) | (nv.smplintervalmins[0] & 0xFF);
}

/*!
 *  @brief Get the sleep interval in minutes (deprecated).
 *
 *  This NVM parameter is not used.
 *
 *  @return The tag sleep interval in minutes.
 */
long nvparams_getsleepintmins()
{
    return nv.sleepintervaldays * MINUTES_PER_DAY;
}

/*!
 *  @brief Check that the cuplTag is fully configured.
 *
 *  @return 'true' if all NVM parameters have been written.
 */
bool nvparams_allwritten()
{
    return (nv.allwritten == NVM_ALL_PARAMS_WRITTEN);
}

/*!
 *  @brief Get resets per loop.
 *
 *  @return The number of resets that have occurred during the present 'loop' of the circular buffer.
 */
int nvparams_getresetsperloop()
{
    return nv.resetsperloop;
}

/*!
 *  @return The number of resets that have occurred from the factory.
 */
int nvparams_getresetsalltime()
{
    return nv.resetsalltime;
}

/*!
 *  @brief Clear resets per loop.
 *
 *  Clear the number of resets that has occurred during the present 'loop' of the circular buffer.
 *  This should be done each time data wraps from the end of the circular buffer back to the start.
 */
void nvparams_cresetsperloop()
{
    DISABLE_FRAM_DATA_WRITEPROTECT;

    nv.resetsperloop = 0;

    ENABLE_FRAM_DATA_WRITEPROTECT;
}

/*!
 *  @brief Increment both reset counters in NVM.
 */
void nvparams_incrcounters()
{
    DISABLE_FRAM_DATA_WRITEPROTECT;

    nv.resetsperloop += 1;
    nv.resetsalltime += 1;

    ENABLE_FRAM_DATA_WRITEPROTECT;
}


/*!
 *  @brief Write a parameter to non-volatile memory.
 *
 *  A parameter consists of an ID (one byte) and a value (one or more bytes).
 *
 *  @param[in] id parameter ID.
 *  @param[in] valptr pointer to an array that contains the parameter value.
 *  @param[in] vlen length of the value in bytes.
 *
 *  @return true if the parameter ID is recognised and its value has the correct length in bytes. Otherwise false.
 */
bool nvparams_write(char id, char * valptr, unsigned int vlen)
{
    bool paramvalid = true;
    // Parameter value as a string. +1 and initialised to 0 because the string must be null terminated.
    char valstr[INTEGERFIELD_LENBYTES + 1] = {0};
    int smplintervalmins;

    DISABLE_FRAM_DATA_WRITEPROTECT;

    if ((id == NVPARAM_SERIAL_ID) && (vlen == SERIAL_LENBYTES))
    {
        // Copy the value string to the serial field in NVM.
        strncpy(nv.serial, valptr, vlen);
        paramswritten |= SERIAL_PARAM_WRITTEN;
    }
    else if ((id == NVPARAM_SECKEY_ID) && (vlen == SECKEY_LENBYTES))
    {
        // Copy the value string to the secret key field in NVM.
        strncpy(nv.seckey, valptr, vlen);
        paramswritten |= SECKEY_PARAM_WRITTEN;
    }
    else if ((id== NVPARAM_BASEURL_ID) && (vlen < BASEURL_LENBYTES))
    {
        // Copy the value string to the base URL field in NVM.
        strncpy(nv.baseurl, valptr, vlen);
        // Null terminate the base URL. Bug: the base URL field should be 65 characters rather than 64.
        nv.baseurl[vlen] = 0;
        paramswritten |= BASEURL_PARAM_WRITTEN;
    }
    else if ((id == NVPARAM_FMT_ID) && (vlen <= FMT_ASCII_MAXLEN))
    {
        // Copy the value string to valstr to ensure it is null terminated.
        strncpy(valstr, valptr, vlen);
        // Convert the format string to an integer.
        nv.format = atoi(valstr);
        paramswritten |= FMT_PARAM_WRITTEN;
    }
    else if ((id == NVPARAM_SMPLINT_ID) && (vlen <= SMPLINT_ASCII_MAXLEN))
    {
        // Copy the value string to valstr to ensure it is null terminated.
        strncpy(valstr, valptr, vlen);
        // Convert the sample interval from ASCII into an integer.
        smplintervalmins = atoi(valstr);
        // Write the Least Significant Byte into NVM.
        nv.smplintervalmins[0] = smplintervalmins & 0xFF;
        // Write the Most Signficant Byte into NVM
        nv.smplintervalmins[1] = smplintervalmins >> 8;
        paramswritten |= SMPLINT_PARAM_WRITTEN;
    }
    else if ((id == NVPARAM_MINVOLT_ID) && (vlen <= MINVOLT_ASCII_MAXLEN))
    {
        // Copy the value string to valstr to ensure it is null terminated.
        strncpy(valstr, valptr, vlen);
        // Convert the minimum voltage from ASCII to an integer.
        nv.minvoltagemv = atoi(valstr);
        paramswritten |= MINVOLT_PARAM_WRITTEN;
    }
    else if ((id == NVPARAM_HTTPSDIS_ID) && (vlen == HTTPSDIS_LENBYTES))
    {
        // Convert the HTTPS disable ASCII character to an integer.
        nv.httpsdisable = *valptr - '0';
        paramswritten |= HTTPSDIS_PARAM_WRITTEN;
    }
    else if ((id == NVPARAM_USEHMAC_ID) && (vlen == USEHMAC_LENBYTES))
    {
        // Convert the Use HMAC ASCII character to an integer.
        nv.usehmac = *valptr - '0';
        // Check data for validity.
        if ((nv.usehmac == 0) || (nv.usehmac == 1))
        {
            paramswritten |= USEHMAC_PARAM_WRITTEN;
        }
    }
    else
    {
        /* Parameter ID and value length are not recognised. */
        paramvalid = false;
    }

    // Check that all parameters have been written.
    if ((paramswritten ^ ALL_PARAMS_WRITTEN) == 0)
    {
        nv.allwritten = NVM_ALL_PARAMS_WRITTEN;
        // Clear both reset counters.
        nv.resetsperloop = 0;
        nv.resetsalltime = 0;
    }

    ENABLE_FRAM_DATA_WRITEPROTECT;

    return paramvalid;
}
