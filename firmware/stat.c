/*
 * stat.c
 *
 *  Created on: 26 Jul 2018
 *      Author: malcolm
 */

#include "stat.h"
#include "stat_bits.h"

#include <msp430.h>



unsigned int rstcause = 0;

void stat_rdrstcause()
{
    unsigned int sysrstiv;

    sysrstiv = SYSRSTIV;

    switch(sysrstiv)
    {
    case SYSRSTIV_SVSHIFG:
        rstcause |= SVSH_BIT;
        break;
    case SYSRSTIV_BOR:
        rstcause |= BOR_BIT;
        break;
    case SYSRSTIV_WDTTO:
        rstcause |= WDT_BIT;
        break;
    case SYSRSTIV_LPM5WU:
        rstcause |= LPM5WU_BIT;
        break;
    case SYSRSTIV_NONE:
        break;
    case SYSRSTIV_RSTNMI:
        break;
    default:
        rstcause |= MISC_BIT;
        break;
    }
}

void stat_setclockfailure()
{
    rstcause |= CLOCKFAIL_BIT;
}

void stat_setscantimeout()
{
    rstcause |= SCANTIMEOUT_BIT;
}

unsigned int stat_get(bool * err, int resetsalltime)
{
    *err = ((rstcause & ERR_BITS) > 0);
    resetsalltime = resetsalltime >> 4;
    return (resetsalltime << 8) | (rstcause & 0xFF);
}
