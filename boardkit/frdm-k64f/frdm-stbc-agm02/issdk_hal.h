/*
* Copyright (c) 2016, Freescale Semiconductor, Inc.
* All rights reserved.
*
*
* SPDX-License-Identifier: BSD-3-Clause
*/

/*! \file issdk_hal.h
    \brief Wrapper for Hardware Abstraction Layer (HAL)

    This file simply provides one level of indirection for the developer
    to select the particular Hardware Abstraction Layer they would like to use.
*/

#ifndef __ISSDK_HAL_H__
#define __ISSDK_HAL_H__

#include "frdm_k64f.h"              //Include appropriate MCU board header file
#include "frdm_stbc_agm02_shield.h" //Include appropriate sensor shield board header file
// Pin mapping and driver information for default I2C brought to shield
// By default, we use I2C_S1 defined in the frdm_k64f.h file.
// The other option is I2C_S2.
// S1 is on A5:4.  S2 is on D15:14.
#define I2C_S_SCL_PIN I2C_S1_SCL_PIN
#define I2C_S_SDA_PIN I2C_S1_SDA_PIN
#define I2C_S_DRIVER_NONBLOCKING I2C_S1_DRIVER_NONBLOCKING
#define I2C_S_DRIVER_BLOCKING    I2C_S1_DRIVER_BLOCKING

#endif // __ISSDK_HAL_H__
