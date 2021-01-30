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

static char readbuffer[BLKSIZE];
extern unsigned char msgblock[64];

#define PAYLOADSTART_SHORTREC_INDEX 6
#define RECORDTYPE_SHORTREC_INDEX  5
#define PAYLOADLEN_SHORTREC_INDEX  4

typedef enum {startcharsearch, cmdfound, datafound, endcharsearch} searchstate_t;

int readfromtag = 0;

int confignfc_check()
{
    int cursorblock = 0;
    char rectype = 0;

    nt3h_readtag(cursorblock+1, readbuffer);
    cursorblock++;

    rectype = readbuffer[RECORDTYPE_SHORTREC_INDEX];

    return (rectype == 'T');
}


int confignfc_readtext()
{
    int cursorblock = 0;
    int payloadlen;
    int payloadindex = 0;
    volatile char rectype = 0;
    int md5index = 0;
    int bufferindex = PAYLOADSTART_SHORTREC_INDEX;
    char payloadbyte;
    char cmd;
    searchstate_t searchstate = startcharsearch;

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
