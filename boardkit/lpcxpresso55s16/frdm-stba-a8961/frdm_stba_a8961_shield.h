/*
 * Copyright 2023 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*! File: frdm_stba_a8961_shield.h
* @brief The frdm_stba_a8961_shield.h file declares arduino pin mapping for frdm_stba_a8961_shield with frdm-k22f.
*/

#ifndef _FRDM_STBA_A8961_SHIELD_H_
#define _FRDM_STBA_A8961_SHIELD_H_

/* The shield name */
#define SHIELD_NAME "FRDM-STBA-A8961"

// FXLS8961 Sensor Information
#define FXLS8961_I2C_ADDR 0x18
#define FXLS8961_CS       D10
#define FXLS8961_MOSI     D11
#define FXLS8961_MISO     D12
#define FXLS8961_SCLK     D13
#define FXLS8961_INT1     D2
#define FXLS8961_INT2     A0

// FRDM-STBA-A8961 Shield Reset
#define RESET_GPIO A3

#endif /* _FRDM_STBA_A8961_SHIELD_H_ */
