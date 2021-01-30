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

#ifndef COMMS_NVPARAMS_H_
#define COMMS_NVPARAMS_H_

#include <stdbool.h>


char * nvparams_getserial(void);
char * nvparams_getsecretkey(void);
unsigned int nvparams_getversion(void);
unsigned int nvparams_getsmplintmins(void);
long nvparams_getsleepintmins(void);
unsigned int nvparams_getminvoltagemv(void);
// Resets
int nvparams_getresetsperloop(void);
int nvparams_getresetsalltime(void);
void nvparams_cresetsperloop(void);
void nvparams_incrcounters(void);
bool nvparams_allwritten(void);


bool nvparams_write(char id, char * valptr, unsigned int payloadLen);

#endif /* COMMS_NVPARAMS_H_ */
