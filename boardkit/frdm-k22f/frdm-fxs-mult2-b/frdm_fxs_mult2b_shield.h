/*
* Copyright (c) 2016, Freescale Semiconductor, Inc.
* All rights reserved.
*
*
* SPDX-License-Identifier: BSD-3-Clause
*/

/*! File: frdm_fxs_mult2b_shield.h
* @brief The \b frdm_fxs_mult2b_shield.h file declares mapping of the Kinetis
         Device peripherals to the frmd-k64f and frdm-fxs-mult2-b shield
*/

#ifndef _FRDM_FXS_MULT2B_SHIELD_H_
#define _FRDM_FXS_MULT2B_SHIELD_H_

/* The shield name */
#define SHIELD_NAME "FRDM-FXS-MULT2-B"

// FXOS8700 Sensor Information
/*
   In order to route INT1 signal from FXOS8700 to the K64F,
   Pins 1-2 of Jumper J3 on the FRDM-FXS-MULT2-B should be connected.
   In order to route INT2 signal from FXOS8700 to the K64F,
   Pins 1-2 of Jumper J4 on the FRDM-FXS-MULT2-B should be connected.
*/
#define FXOS8700_I2C_ADDR 0x1E
#define FXOS8700_INT1 D2
#define FXOS8700_INT2 D4

// FXAS21002 Sensor Information
/*
   In order to route INT1 signal from FXAS21002 to the K64F,
   Pins 1-2 of Jumper J6 on the FRDM-FXS-MULT2-B should be connected.
*/
#define FXAS21002_I2C_ADDR 0x20
#define FXAS21002_INT1 D5

// MAG3110 Sensor Information
/*
   In order to route INT1 signal from MAG3110 to the K64F,
   Pins 2-3 of Jumper J3 on the FRDM-FXS-MULT2-B should be connected.
*/
#define MAG3110_I2C_ADDR 0x0E
#define MAG3110_INT1 D2

// MPL3115 Sensor Information
/*
   In order to route INT1 signal from MPL3115 to the K64F,
   Pins 2-3 of Jumper J5 on the FRDM-FXS-MULT2-B should be connected.
*/
#define MPL3115_I2C_ADDR 0x60
#define MPL3115_INT1 D8

// MMA8652 Sensor Information
/*
   In order to route INT1 signal from MMA8652 to the K64F,
   Pins 2-3 of Jumper J4 on the FRDM-FXS-MULT2-B should be connected.
*/
#define MMA8652_I2C_ADDR 0x1D
#define MMA8652_INT1 D4

// FXLS8471Q Sensor Information
/*
   In order to route INT1 signal from FXLS8471Q to the FRDM-K64F,
   Pins 2-3 of Jumper J6 on the FRDM-FXS-MULT2-B should be connected.
*/
#define FXLS8471_INT1 D5
#define FXLS8471_SPI_CS D10

///@name Shield Parameters
/// Sensor Fusion specific defines
/// Use this section to define the shield board: 3 bit code 0 to 7 inclusive
/// transmitted in bits 7-5 for display purposes only.
///@{
#define SHIELD_MULTIB 0
#define SHIELD_NONE 1
#define SHIELD_AGM01 2
#define SHIELD_AGM02 3
#define THIS_SHIELD SHIELD_MULTIB

// spare 5 to 7 inclusive
///@}

#endif /* _FRDM_FXS_MULT2B_SHIELD_H_ */
