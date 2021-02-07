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

int stat_rstcause_is_lpm5wu() {
    return (rstcause & LPM5WU_BIT);
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
