/* --COPYRIGHT--,BSD
 * Copyright (c) 2017, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
//*****************************************************************************
//
// ecomp.c - Driver for the ecomp Module.
//
//*****************************************************************************

//*****************************************************************************
//
//! \addtogroup ecomp_api ecomp
//! @{
//
//*****************************************************************************

#include "inc/hw_memmap.h"

#ifdef __MSP430_HAS_ECOMPx__
#include "ecomp.h"

#include <assert.h>

void EComp_init(uint16_t baseAddress, EComp_initParam *param)
{
	HWREG16(baseAddress + OFS_CPCTL0) &= ~(CPNEN | CPNSEL_7 | CPPEN | CPPSEL_7);
	HWREG16(baseAddress + OFS_CPCTL1) &= ~(CPFLT | CPFLTDLY_3 | CPINV);

	if (param->positiveTerminalInput != ECOMP_INPUT_DISABLED) {
		HWREG16(baseAddress + OFS_CPCTL0) |= CPPEN | param->positiveTerminalInput;
	}

	if (param->negativeTerminalInput != ECOMP_INPUT_DISABLED) {
		HWREG16(baseAddress + OFS_CPCTL0) |= CPNEN | (param->negativeTerminalInput<<8);
	}

	HWREG16(baseAddress + OFS_CPCTL1) |= param->outputFilterEnableAndDelayLevel |
			param->invertedOutputPolarity;
}

void EComp_selectHysteresisMode(uint16_t baseAddress,
		uint16_t hysteresisMode)
{
	HWREG16(baseAddress + OFS_CPCTL1) &= ~CPHSEL_3;
	HWREG16(baseAddress + OFS_CPCTL1) |= hysteresisMode;
}

void EComp_selectPowerMode(uint16_t baseAddress, uint16_t powerMode)
{
	HWREG16(baseAddress + OFS_CPCTL1) &= ~CPMSEL;
	HWREG16(baseAddress + OFS_CPCTL1) |= powerMode;
}

void EComp_enable(uint16_t baseAddress)
{
	HWREG16(baseAddress + OFS_CPCTL1) |= CPEN;
}

void EComp_disable(uint16_t baseAddress)
{
	HWREG16(baseAddress + OFS_CPCTL1) &= ~CPEN;
}

void EComp_enableInterrupt(uint16_t baseAddress, uint16_t interruptMask)
{
	HWREG16(baseAddress + OFS_CPCTL1) |= interruptMask;
}

void EComp_disableInterrupt(uint16_t baseAddress, uint16_t interruptMask)
{
	HWREG16(baseAddress + OFS_CPCTL1) &= ~(interruptMask);
}

void EComp_clearInterrupt(uint16_t baseAddress, uint16_t interruptFlagMask)
{
	HWREG16(baseAddress + OFS_CPINT) |= interruptFlagMask;
}

uint8_t EComp_getInterruptStatus(uint16_t baseAddress,
		uint16_t interruptFlagMask)
{
    return (HWREG8(baseAddress + OFS_CPINT) & interruptFlagMask);
}

void EComp_setInterruptEdgeDirection(uint16_t baseAddress,
		uint16_t edgeDirection)
{
	HWREG16(baseAddress + OFS_CPCTL1) &= ~CPIES;
	HWREG16(baseAddress + OFS_CPCTL1) |= edgeDirection;
}

void EComp_toggleInterruptEdgeDirection(uint16_t baseAddress)
{
	HWREG16(baseAddress + OFS_CPCTL1) ^= CPIES;
}

uint8_t EComp_outputValue(uint16_t baseAddress)
{
	return (HWREG8(baseAddress + OFS_CPCTL1) & CPOUT);
}

void EComp_configureDAC(uint16_t baseAddress, EComp_configureDACParam *param)
{
	HWREG16(baseAddress + OFS_CPDACCTL) &= ~(CPDACREFS | CPDACBUFS | CPDACSW);
	HWREG16(baseAddress + OFS_CPDACDATA) &= 0xC0C0;

	HWREG16(baseAddress + OFS_CPDACCTL) |= param->referenceVoltage |
			param->bufferSource;

	HWREG16(baseAddress + OFS_CPDACDATA) |= param->firstBufferData;
	HWREG16(baseAddress + OFS_CPDACDATA) |= param->secondBufferData<<8;
}

void EComp_enableDAC(uint16_t baseAddress)
{
	HWREG16(baseAddress + OFS_CPDACCTL) |= CPDACEN;
}

void EComp_disableDAC(uint16_t baseAddress)
{
	HWREG16(baseAddress + OFS_CPDACCTL) &= ~CPDACEN;
}

#endif
//*****************************************************************************
//
//! Close the doxygen group for ecomp_api
//! @}
//
//*****************************************************************************
