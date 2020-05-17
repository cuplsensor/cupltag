/*
 * nvparams.h
 *
 *  Created on: 5 Jul 2018
 *      Author: mmackay
 */

#ifndef COMMS_NVPARAMS_H_
#define COMMS_NVPARAMS_H_

#include <stdbool.h>


char * nvparams_getserial(void);
char * nvparams_getsecretkey(void);
unsigned int nvparams_getversion(void);
unsigned int nvparams_getsmplintmins(void);
long nvparams_getsleepintmins(void);
// Resets
int nvparams_getresetsperloop(void);
int nvparams_getresetsalltime(void);
void nvparams_cresetsperloop(void);
void nvparams_incrcounters(void);
bool nvparams_allwritten(void);


bool nvparams_write(char id, char * valptr, unsigned int payloadLen);

#endif /* COMMS_NVPARAMS_H_ */
