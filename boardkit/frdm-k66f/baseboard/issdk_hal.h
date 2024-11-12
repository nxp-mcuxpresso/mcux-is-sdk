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

#include "frdm_k66f.h"              //Include appropriate MCU board header file

#define I2C_S_SCL_PIN I2C_BB_SCL_PIN
#define I2C_S_SDA_PIN I2C_BB_SDA_PIN
#define SHIELD_NAME "N/A"

#endif // __ISSDK_HAL_H__
