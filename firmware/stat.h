/*
 * stat.h
 *
 *  Created on: 26 Jul 2018
 *      Author: malcolm
 */

#ifndef STAT_H_
#define STAT_H_

#include <stdbool.h>

void stat_rdrstcause();
unsigned int stat_get(bool * err, int resetsalltime);
void stat_setscantimeout();
void stat_setclockfailure();

#endif /* STAT_H_ */
