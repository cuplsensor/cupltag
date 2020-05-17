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
// tia.h - Driver for the TIA Module.
//
//*****************************************************************************

#ifndef __MSP430WARE_TIA_H__
#define __MSP430WARE_TIA_H__

#include "inc/hw_memmap.h"

#ifdef __MSP430_HAS_TRIx__

//*****************************************************************************
//
// If building with a C++ compiler, make all of the definitions in this header
// have a C binding.
//
//*****************************************************************************
#ifdef __cplusplus
extern "C"
{
#endif

//*****************************************************************************
//
// The following are values that can be passed to the positiveInput parameter
// for functions: TIA_selectPositiveInput().
//
//*****************************************************************************
#define TIA_POSITIVE_INPUT_EXTERNAL_SOURCE                            TRIPSEL_0
#define TIA_POSITIVE_INPUT_DEVICE_SPECIFIC_1                          TRIPSEL_1
#define TIA_POSITIVE_INPUT_DEVICE_SPECIFIC_2                          TRIPSEL_2
#define TIA_POSITIVE_INPUT_DEVICE_SPECIFIC_3                          TRIPSEL_3

//*****************************************************************************
//
// The following are values that can be passed to the powerMode parameter for
// functions: TIA_selectPowerMode().
//
//*****************************************************************************
#define TIA_HIGH_SPEED_HIGH_POWER                                           0x0
#define TIA_LOW_SPEED_LOW_POWER                                           TRIPM

//*****************************************************************************
//
// Prototypes for the APIs.
//
//*****************************************************************************

//*****************************************************************************
//
//! \brief Selects TIA positive input
//!
//! This function selects TIA positive input.
//!
//! \param baseAddress is the base address of the TIA module.
//! \param positiveInput selects positive input.
//!        Valid values are:
//!        - \b TIA_POSITIVE_INPUT_EXTERNAL_SOURCE [Default]
//!        - \b TIA_POSITIVE_INPUT_DEVICE_SPECIFIC_1
//!        - \b TIA_POSITIVE_INPUT_DEVICE_SPECIFIC_2
//!        - \b TIA_POSITIVE_INPUT_DEVICE_SPECIFIC_3
//!        \n Modified bits are \b TRIPSEL of \b TRICTL register.
//!
//! \return None
//
//*****************************************************************************
extern void TIA_selectPositiveInput(uint16_t baseAddress,
                                    uint16_t positiveInput);

//*****************************************************************************
//
//! \brief Selects TIA power mode
//!
//! This function selects TIA power mode.
//!
//! \param baseAddress is the base address of the TIA module.
//! \param powerMode selects TIA power mode.
//!        Valid values are:
//!        - \b TIA_HIGH_SPEED_HIGH_POWER [Default]
//!        - \b TIA_LOW_SPEED_LOW_POWER
//!        \n Modified bits are \b TRIPM of \b TRICTL register.
//!
//! \return None
//
//*****************************************************************************
extern void TIA_selectPowerMode(uint16_t baseAddress,
                                uint16_t powerMode);

//*****************************************************************************
//
//! \brief Enables TIA module
//!
//! This function enables TIA module.
//!
//! \param baseAddress is the base address of the TIA module.
//!
//! \return None
//
//*****************************************************************************
extern void TIA_enable(uint16_t baseAddress);

//*****************************************************************************
//
//! \brief Disables TIA module
//!
//! This function disables TIA module.
//!
//! \param baseAddress is the base address of the TIA module.
//!
//! \return None
//
//*****************************************************************************
extern void TIA_disable(uint16_t baseAddress);

//*****************************************************************************
//
// The following are deprecated APIs.
//
//*****************************************************************************
#define TRI_selectPositiveInput                         TIA_selectPositiveInput
#define TRI_selectPowerMode                                 TIA_selectPowerMode
#define TRI_enable                                                   TIA_enable
#define TRI_disable                                                 TIA_disable
#define TRI_POSITIVE_INPUT_EXTERNAL_SOURCE   TIA_POSITIVE_INPUT_EXTERNAL_SOURCE
#define TRI_POSITIVE_INPUT_DEVICE_SPECIFIC_1                                  \
                                           TIA_POSITIVE_INPUT_DEVICE_SPECIFIC_1
#define TRI_POSITIVE_INPUT_DEVICE_SPECIFIC_2                                  \
                                           TIA_POSITIVE_INPUT_DEVICE_SPECIFIC_2
#define TRI_POSITIVE_INPUT_DEVICE_SPECIFIC_3                                  \
                                           TIA_POSITIVE_INPUT_DEVICE_SPECIFIC_3
#define TRI_HIGH_SPEED_HIGH_POWER                     TIA_HIGH_SPEED_HIGH_POWER
#define TRI_LOW_SPEED_LOW_POWER                         TIA_LOW_SPEED_LOW_POWER
#define TIA0_BASE                                                     TRI0_BASE

//*****************************************************************************
//
// Mark the end of the C bindings section for C++ compilers.
//
//*****************************************************************************
#ifdef __cplusplus
}
#endif

#endif
#endif // __MSP430WARE_TIA_H__
