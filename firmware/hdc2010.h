/*
 * hdc2010.h
 *
 *  Created on: 4 Nov 2017
 *      Author: mmackay
 */

#ifndef HDC2010_H_
#define HDC2010_H_

#include "i2c.h"

int hdc2010_init();
uint32_t hdc2010_read_temp(int *, int *);
uint32_t hdc2010_read_humidity();
int hdc2010_read_whoami();
int hdc2010_startconv();
#endif /* HDC2010_H_ */
