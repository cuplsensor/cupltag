/*
 * comms_uart.h
 *
 *  Created on: 2 Sep 2014
 *      Author: Malcolm
 */

#ifndef COMMS_UART_H_
#define COMMS_UART_H_

#include <stdbool.h>
#include <stdint.h>

#define UART_BAUDRATE 115200

#include "nvparams.h"

typedef enum ustat {
    ustat_running,
    ustat_waiting,
    ustat_finished
} t_ustat;


/* Function prototypes. */
t_ustat uart_run(int nPRG);

#endif /* COMMS_UART_H_ */
