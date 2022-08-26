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

#define PAYLOADSTART_SHORTREC_INDEX     6       /*!< NDEF record payload starts at this byte within EEPROM block 1. */
#define RECORDTYPE_SHORTREC_INDEX       5       /*!< Index corresponding to NDEF record type. Only correct for short NDEF records. */
#define PAYLOADLEN_SHORTREC_INDEX       4       /*!< Index corresponding to NDEF record length. Only correct for short NDEF records. */

#define CONFIGSTR_STARTCHAR             '<'     /*!< Marks the start of a configuration string. */
#define CONFIGSTR_DELIMCHAR             ':'     /*!< Separates the ID from a value in a configuration string. */
#define CONFIGSTR_ENDCHAR               '>'     /*!< Marks the end of a configuration string. */

typedef enum {
    findstartchar,          /*!< Search for the config string start character. */
    storeid,                 /*!< Read the ID byte. */
    checkdelimiter,         /*!< Check for the delimiter byte. */
    storevalue              /*!< Copy the configuration string value into msgblock. */
} parserstate_t;            /*!< State of the configuration string parser. */

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
 * 'c' is the ID.
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
    volatile char rectype = 0;
    int cursorblock = EEPROM_USERMEM_FIRST_BLOCK;   /* Index of the EEPROM block to be read. */
    int payloadlen;                                 /* Length of the NDEF record payload. */
    int payloadindex = 0;                           /* Index within the NDEF record payload array. The first payload byte is index 0. */
    int valindex = 0;                               /* Index within the configuration string value array. The first byte of the configuration string value is index 0. */
    int bufferindex = PAYLOADSTART_SHORTREC_INDEX;  /* Index of the byte to be read within the current EEPROM block. */
    char payloadbyte;                               /* Stores one byte of the NDEF record payload. */
    char id;                                        /* Used to store the ID from a configuration string. */
    parserstate_t parserstate = findstartchar;      /* State of the configuration string parser. */

    // Read the first block of user memory into the buffer.
    nt3h_readtag(cursorblock, readbuffer);
    cursorblock++;

    // Read the NDEF record length.
    // BUG: There is nothing to prevent payloadlen from exceeding the length of the msgblock array.
    payloadlen = readbuffer[PAYLOADLEN_SHORTREC_INDEX];

    // Iterate through all bytes of the NDEF record payload.
    while(payloadindex++ < payloadlen)
    {
        // Read from the buffer
        payloadbyte = readbuffer[bufferindex];

        if (parserstate == findstartchar)
        {
            /* Look for the start of a configuration string. */
            if (payloadbyte == CONFIGSTR_STARTCHAR)
            {
                /* A configuration string has been found. */
                parserstate = storeid;
            }
        }
        else if (parserstate == storeid)
        {
            /* Store the ID (one byte) from the configuration string. */
            id = payloadbyte;
            parserstate = checkdelimiter;
        }
        else if (parserstate == checkdelimiter)
        {
            /* Check for the delimiter character. */
            if (payloadbyte == CONFIGSTR_DELIMCHAR)
            {
                /* The delimiter has been found. */
                parserstate = storevalue;
            }
        }
        else
        {
            /* Store the value (several bytes) into the msgblock array. */
            if (payloadbyte == CONFIGSTR_ENDCHAR)
            {
                /* End character has been found. This marks the end of the value field of the configuration string. */
                parserstate = findstartchar;
                /* If the ID corresponds to a configuration parameter (e.g. the serial string), write it into NVM. */
                nvparams_write(id, msgblock, valindex);
                valindex = 0;
            }
            else
            {
                /* Append current payload byte to the array. */
                msgblock[valindex++] = payloadbyte;
            }
        }


        if (bufferindex >= (BLKSIZE-1))
        {
            /* Read the next block from EEPROM into the buffer. */
            nt3h_readtag(cursorblock, readbuffer);
            cursorblock++;
            bufferindex = 0;
        }
        else
        {
            /* Read the next byte from the current EEPROM block on the next loop. */
            bufferindex += 1;
        }
    }

    return 0;
}
