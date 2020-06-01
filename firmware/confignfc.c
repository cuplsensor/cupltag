/*
 * confignfc.c
 *
 *  Created on: 22 Apr 2019
 *      Author: malcolm
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
