/*
 * Copyright 2023 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*! File: frdm_stbi_a8971_shield.h
* @brief The frdm_stbi_a8971_shield.h file declares arduino pin mapping for frdm_stbi_a8971_shield with frdm-k22f.
*/

#ifndef _FRDM_STBI_A8971_SHIELD_H_
#define _FRDM_STBI_A8971_SHIELD_H_

/* The shield name */
#define SHIELD_NAME "FRDM-STBI-A8971"

// FXLS8971 Sensor Information
#define FXLS8971_I2C_ADDR 0x18
#define FXLS8971_CS       D10
#define FXLS8971_MOSI     D11
#define FXLS8971_MISO     D12
#define FXLS8971_SCLK     D13
#define FXLS8971_INT1     D2
#define FXLS8971_INT2     A0

// FRDM-STBI-A8971 Shield Reset
#define RESET_GPIO A3

#endif /* _FRDM_STBI_A8971_SHIELD_H_ */
