/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file issdk_hal.h
 * @brief Wrapper for Hardware Abstraction Layer (HAL)

    This file simply provides one level of indirection for the developer
    to select the particular Hardware Abstraction Layer they would like to use.
*/

#ifndef __ISSDK_HAL_H__
#define __ISSDK_HAL_H__

#include "frdm_kw41z.h" //Include appropriate MCU board header file

// Pin mapping and driver information for default I2C brought to shield
// By default, we use I2C_BB defined in the frdm_kw41z.h file.
#define I2C_S_SCL_PIN I2C_BB_SCL_PIN
#define I2C_S_SDA_PIN I2C_BB_SDA_PIN
#define I2C_S_DRIVER I2C_BB_DRIVER
#define I2C_S_SIGNAL_EVENT I2C_BB_SIGNAL_EVENT
#define I2C_S_DEVICE_INDEX I2C_BB_DEVICE_INDEX

#endif // __ISSDK_HAL_H__
