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

#include "confignfc.h"
#include "nt3h.h"
#include "nvparams.h"
#include "defs.h"

static char readbuffer[BLKSIZE];                /*!< Holds one 16-byte EEPROM block. */
extern unsigned char msgblock[64];              /*!< Re-use an array declared as part of cuplcodec for calculating the MD5 checksum. */

#define EEPROM_USERMEM_FIRST_BLOCK      1       /*!< Index of the first 16-byte block of unprotected user memory. */
#define NDEF_RECORDTYPE_TEXT            'T'     /*!< NDEF text record type. */

#define PAYLOADSTART_SHORTREC_INDEX     6       /*!< NDEF record payload starts at this index. */
#define RECORDTYPE_SHORTREC_INDEX       5       /*!< Index corresponding to NDEF record type. Only correct for short NDEF records. */
#define PAYLOADLEN_SHORTREC_INDEX       4       /*!< Index corresponding to NDEF record length. Only correct for short NDEF records. */

typedef enum {
    startcharsearch,        /*!< Search for the config string start character. */
    cmdfound,               /*!< Read the command byte. */
    datafound,              /*!< Check for the delimiter byte. */
    endcharsearch           /*!< Copy the configuration string value into msgblock. */
} parserstate_t;

/*!
 * @brief Read the first block of NFC EEPROM user memory. Check if it contains an NDEF text record.
 *
 * Configuration data are written as strings in the text record.
 *
 * @returns 1 if a short text record is present, otherwise 0.
 */
int confignfc_check()
{
    // Read the first block of user memory into a buffer.
    nt3h_readtag(EEPROM_USERMEM_FIRST_BLOCK, readbuffer);
    // Check for a text record.
    return (readbuffer[RECORDTYPE_SHORTREC_INDEX] == NDEF_RECORDTYPE_TEXT);
}

/*!
 * @brief Parse the NDEF text record into configuration strings. Writing configuration to NVM.
 *
 * Configuration strings are formatted as:
 *
 * <c:xyz>
 *
 * Where:
 *
 * '<' is the start character.
 *
 * 'c' is the command.
 *
 * ':' is the delimiter.
 *
 * 'xyz' is the value.
 *
 * '>' is the end character.
 *
 * The text record can contain one or more strings. There is no separator
 * character between them.
 *
 * Configuration strings are written to non-volatile memory with nvparams_write().
 *
 */
int confignfc_parse()
{
    int cursorblock = 0;
    int payloadlen;
    int payloadindex = 0;
    volatile char rectype = 0;
    int md5index = 0;
    int bufferindex = PAYLOADSTART_SHORTREC_INDEX;
    char payloadbyte;
    char cmd;
    parserstate_t searchstate = startcharsearch;

    nt3h_readtag(cursorblock+1, readbuffer);
    cursorblock++;

    payloadlen = readbuffer[PAYLOADLEN_SHORTREC_INDEX];

    while(payloadindex++ < payloadlen)
    {
        // Read from the buffer.
        payloadbyte = readbuffer[bufferindex];

        if (searchstate == startcharsearch)
        {
            if (payloadbyte == '<')
            {
                searchstate = cmdfound;
            }
        }
        else if (searchstate == cmdfound)
        {
            cmd = payloadbyte;
            searchstate = datafound;
        }
        else if (searchstate == datafound)
        {
            if (payloadbyte == ':')
            {
                searchstate = endcharsearch;
            }
        }
        else
        {
            if (payloadbyte == '>')
            {
                searchstate = startcharsearch;
                nvparams_write(cmd, msgblock, md5index);

                md5index = 0;
            }
            else
            {
                msgblock[md5index++] = payloadbyte;
            }
        }



        if (bufferindex >= 15)
        {
            nt3h_readtag(cursorblock+1, readbuffer);
            cursorblock++;
            bufferindex = 0;
        }
        else
        {
            bufferindex += 1;
        }
    }




    return 0;
}
