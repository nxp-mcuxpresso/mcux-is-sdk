/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*! File: frdm_stbc_agm01_shield.h
* @brief The \b frdm_stbc_agm01_shield.h file declares mapping of the Kinetis
         Device peripherals to the frmd-k64f and frmd-stbc-agm01 shield
*/

#ifndef _FRDM_STBC_AGM02_SHIELD_H_
#define _FRDM_STBC_AGM02_SHIELD_H_

/* The shield name */
#define SHIELD_NAME "FRDM-STBC-AGM02"

// MAG3110 Sensor Information
#define MAG3110_I2C_ADDR 0x0E
#define MAG3110_INT1 D4

//  FXLS8952C Sensor Information
#define FXLS8952C_I2C_ADDR 0x18
#define FXLS8952C_INT1 D3
#define FXLS8952C_INT2 D2

// FXAS21002 Sensor Information
#define FXAS21002_I2C_ADDR 0x20
#define FXAS21002_CS D10
#define FXAS21002_INT1 D5
#define FXAS21002_INT2 D8

// FRDM-STBC-AGM02 Shield Reset
#define RESET_GPIO A3

///@name Shield Parameters
/// Use this section to define the shield board: 3 bit code 0 to 7 inclusive
/// transmitted in bits 7-5 for display purposes only.
///@{
#define SHIELD_MULTIB 0
#define SHIELD_NONE 1
#define SHIELD_AGM01 2
#define SHIELD_AGM02 3
#define THIS_SHIELD SHIELD_AGM02

// spare 5 to 7 inclusive
///@}

#endif /* _FRDM_STBC_AGM02_SHIELD_H_ */
