/*
 * nvparams.c
 *
 *  Created on: 5 Jul 2018
 *      Author: mmackay
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

char * nvparams_getserial()
{
    return nv.serial;
}

char * nvparams_getsecretkey()
{
    return nv.seckey;
}

unsigned int nvparams_getminvoltagemv(void)
{
    return nv.minvoltagemv;
}

unsigned int nvparams_getsmplintmins()
{
    return ((unsigned int)nv.smplintervalmins[1] << 8) | (nv.smplintervalmins[0] & 0xFF);
}

long nvparams_getsleepintmins()
{
    return nv.sleepintervaldays * MINUTES_PER_DAY;
}

bool nvparams_allwritten()
{
    return (nv.allwritten == 0);
}

int nvparams_getresetsperloop()
{
    return nv.resetsperloop;
}

int nvparams_getresetsalltime()
{
    return nv.resetsalltime;
}

void nvparams_cresetsperloop()
{
    SYSCFG0 = FRWPPW | PFWP;                   // Program FRAM write enable

    nv.resetsperloop = 0;

    SYSCFG0 = FRWPPW | DFWP | PFWP;           // Program FRAM write protected (not writable)
}

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
