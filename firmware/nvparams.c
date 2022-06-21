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

#include "nvparams.h"
#include "nvtype.h"
#include <string.h>
#include <msp430.h>
#include <stdlib.h>

#define DEFAULT_SMPLINTMINS     6
#define DEFAULT_SLEEPINTDAYS    0
#define DEFAULT_RANDSTATE       0x4D4C
#define MINUTES_PER_DAY         1440
#define INTEGERFIELD_LENBYTES   5           // Value up to 65535 (5 ASCII digits)

#pragma DATA_SECTION(nv, ".info");
nv_t nv;

unsigned int writtenfields = 0;

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
    return (nv.allwritten == 0);
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
    SYSCFG0 = FRWPPW | PFWP;                   // Program FRAM write enable

    nv.resetsperloop = 0;

    SYSCFG0 = FRWPPW | DFWP | PFWP;           // Program FRAM write protected (not writable)
}

/*!
 *  @brief Increment both reset counters in NVM.
 */
void nvparams_incrcounters()
{
    SYSCFG0 = FRWPPW | PFWP;                   // Program FRAM write enable

    nv.resetsperloop += 1;
    nv.resetsalltime += 1;

    SYSCFG0 = FRWPPW | DFWP | PFWP;             // Program FRAM write protected (not writable)
}


bool nvparams_write(char id, char * valptr, unsigned int payloadlen)
{
    bool validpacket = true;
    char temp[INTEGERFIELD_LENBYTES + 1] = {0}; // Must be null terminated.
    int smplintervalmins;

    SYSCFG0 = FRWPPW | PFWP;                   // Program FRAM write enable

    if ((id == 'w') && (payloadlen == sizeof(nv.serial)))
    {
        strncpy(nv.serial, valptr, payloadlen);
        writtenfields |= BIT0;
    }
    else if ((id == 's') && (payloadlen == sizeof(nv.seckey)))
    {
        strncpy(nv.seckey, valptr, payloadlen);
        writtenfields |= BIT1;
    }
    else if ((id== 'b') && (payloadlen < BASEURL_LENBYTES))
    {
        strncpy(nv.baseurl, valptr, payloadlen);
        nv.baseurl[payloadlen] = 0;
        writtenfields |= BIT5;
    }
    else if ((id == 'f') && (payloadlen <= FORMAT_ASCII_MAXLEN))
    {
        strncpy(temp, valptr, payloadlen); // Copy to temp to ensure null termination.
        nv.format = atoi(temp);
        writtenfields |= BIT2;
    }
    else if ((id == 't') && (payloadlen <= SMPLINTERVAL_ASCII_MAXLEN))
    {
        strncpy(temp, valptr, payloadlen); // Copy to temp to ensure null termination.
        smplintervalmins = atoi(temp);
        nv.smplintervalmins[0] = smplintervalmins & 0xFF;
        nv.smplintervalmins[1] = smplintervalmins >> 8;
        writtenfields |= BIT3;
    }
    else if ((id == 'u') && (payloadlen <= MINVOLTAGEMV_ASCII_MAXLEN))
    {
        strncpy(temp, valptr, payloadlen); // Copy to temp to ensure null termination.
        nv.minvoltagemv = atoi(temp);
        writtenfields |= BIT4;
    }
    else if ((id == 'h') && (payloadlen == 1))
    {
        nv.httpsdisable = *valptr - 0x30;
        writtenfields |= BIT6;
    }
    else if ((id == 'i') && (payloadlen == 1))
    {
        nv.usehmac = *valptr - 0x30;

        if ((nv.usehmac == 0) || (nv.usehmac == 1))
        {
            writtenfields |= BIT7;
        }
    }
    else
    {
        validpacket = false;
    }

    if ((writtenfields ^ 0xFF) == 0)
    {
        nv.allwritten = 0;
        nv.resetsperloop = 0;
        nv.resetsalltime = 0;
    }

    SYSCFG0 = FRWPPW | DFWP | PFWP;                    // Program FRAM write protected (not writable)

    return validpacket;
}
