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



unsigned int rstcause = 0; /*!< Reset cause global variable. */

/*!
 * @brief Find what has caused the latest reset by reading the System Reset Interrupt Vector register.
 *
 * This function sets bits in the global variable rstcause, which is initialised to zero.
 * Only the highest priority interrupt is read.
 *
 */
void stat_rdrstcause()
{
    unsigned int sysrstiv;

    // Read the System Reset Interrupt Vector register into an intermediate variable.
    // Each read clears the highest priority interrupt code, until there are no more to read.
    // We are only interested in the highest priority code.
    sysrstiv = SYSRSTIV;

    switch(sysrstiv)
    {
    case SYSRSTIV_SVSHIFG:
        rstcause |= SVSH_BIT;       // Power-up, brown-out, supply voltage supervisor.
        break;
    case SYSRSTIV_BOR:
        rstcause |= BOR_BIT;        // Brown-out reset.
        break;
    case SYSRSTIV_WDTTO:
        rstcause |= WDT_BIT;        // Watch-dog timeout.
        break;
    case SYSRSTIV_LPM5WU:
        rstcause |= LPM5WU_BIT;     // Low Power Mode LPMx.5 wake-up.
        break;
    case SYSRSTIV_NONE:
        break;
    case SYSRSTIV_RSTNMI:
        break;
    default:
        rstcause |= MISC_BIT;       // Miscellaneous cause.
        break;
    }
}

/*!
 * @brief Check if the reset was caused by a routine wakeup from LPMx.5
 *
 * LPMx.5 is entered each time the state machine runs during normal operation.
 * This is to minimise power consumption. The Program Counter resets to zero and RAM is powered down.
 * The Real Time Clock (RTC) peripheral triggers a reset (and an exit from LPMx.5) after one minute has elapsed.
 *
 * The stat_rdrstcause() function must be called first.
 *
 * @returns Non-zero when the reset has been caused by a wake-up from LPMx.5
 */
int stat_rstcause_is_lpm5wu() {
    return (rstcause & LPM5WU_BIT);
}

/*!
 * @brief Set the clock failure bit in the rstcause global variable.
 */
void stat_setclockfailure()
{
    rstcause |= CLOCKFAIL_BIT;
}

/*!
 * @brief Get status information from the rstcause global variable.
 *
 * @param[out] err Pointer to an error flag. The flag is set if the latest reset has been caused by an error.
 * @param[out] borsvs Pointer to the Brownout or SVS reset flag. The flag is set if the latest reset has been caused by a voltage drop to the SVSH or BOR levels.
 * @param[in] resetsalltime Number of resets that have occurred from the factory.
 *
 * @returns A 16-bit status word for inclusion in the URL by cuplCodec. The upper byte is resetsalltime/16. The lower byte is a copy of the rstcause variable.
 */
unsigned int stat_get(bool * err, bool * borsvs, int resetsalltime)
{
    *err = ((rstcause & ERR_BITS) > 0);                 // Set the error flag.
    *borsvs = ((rstcause & (BOR_BIT | SVSH_BIT)) > 0);  // Set the Brownout or SVS reset flag.

    resetsalltime = resetsalltime >> 4;                 // Divide resetsalltime by 16, to make best use of the 8-bits available.
    return (resetsalltime << 8) | (rstcause & 0xFF);
}
