/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file qn9080.h
 * @brief The qn9080.h file defines GPIO pin mappings for QN908xDK board
*/

#ifndef QN9080_H_
#define QN9080_H_

#include "pin_mux.h"
#include "fsl_power.h"
#include "RTE_Device.h"
#include "gpio_driver.h"

// I2C0 Pin Handles
extern gpioHandleKSDK_t D14;
extern gpioHandleKSDK_t D15;

// I2C1/SPI0 Handles
extern gpioHandleKSDK_t D11;
extern gpioHandleKSDK_t D12;
extern gpioHandleKSDK_t D13;

// UART0 Handle
extern gpioHandleKSDK_t D0;
extern gpioHandleKSDK_t D1;

// QN9080 Arduino Connector Pin Defintion
extern gpioHandleKSDK_t A0;
extern gpioHandleKSDK_t A1;
extern gpioHandleKSDK_t A2;
extern gpioHandleKSDK_t A3;
extern gpioHandleKSDK_t A4;
extern gpioHandleKSDK_t A5;
extern gpioHandleKSDK_t D2;
extern gpioHandleKSDK_t D3;
extern gpioHandleKSDK_t D4;
extern gpioHandleKSDK_t D5;
extern gpioHandleKSDK_t D6;
extern gpioHandleKSDK_t D7;
extern gpioHandleKSDK_t D8;
extern gpioHandleKSDK_t D9;
extern gpioHandleKSDK_t D10;

// QN9080 Internal Peripheral Pin Definitions
extern gpioHandleKSDK_t RED_LED;
extern gpioHandleKSDK_t GREEN_LED;
extern gpioHandleKSDK_t BLUE_LED;

// I2C_S1: Pin mapping and driver information for default I2C brought to shield
#define I2C_S1_SCL_PIN D15
#define I2C_S1_SDA_PIN D14
#define I2C_S1_DRIVER Driver_I2C0
#define I2C_S1_DEVICE_INDEX I2C0_INDEX
#define I2C_S1_SIGNAL_EVENT I2C0_SignalEvent_t

// I2C_S2: Pin mapping and driver information for alternate I2C bus on shield
#define I2C_S2_DRIVER Driver_I2C1
#define I2C_S2_DEVICE_INDEX I2C1_INDEX
#define I2C_S2_SIGNAL_EVENT I2C1_SignalEvent_t

// I2C_BB: PPin mapping and driver information for I2C routed on KW41Z base board
#define I2C_BB_SCL_PIN D15
#define I2C_BB_SDA_PIN D14
#define I2C_BB_DRIVER Driver_I2C0
#define I2C_BB_DEVICE_INDEX I2C0_INDEX
#define I2C_BB_SIGNAL_EVENT I2C0_SignalEvent_t

// SPIS: Pin mapping and driver information default SPI brought to shield
#define SPI_S_DRIVER Driver_SPI0
#define SPI_S_BAUDRATE 500000U ///< Transfer baudrate - 500k
#define SPI_S_DEVICE_INDEX SPI0_INDEX
#define SPI_S_SIGNAL_EVENT SPI0_SignalEvent_t

// UART: Driver information for default UART to communicate with HOST PC.
#define HOST_S_DRIVER Driver_USART0
#define HOST_S_SIGNAL_EVENT HOST_SignalEvent_t
#define HOST_B_DRIVER Driver_USART0
#define HOST_B_SIGNAL_EVENT HOST_SignalEvent_t

// ADS_FLASH: The next to last page of flash.
#define ADS_NVM_ADDR \
    (FSL_FEATURE_FLASH_BASE_ADDR + FSL_FEATURE_FLASH_SIZE_BYTES - (2 * FSL_FEATURE_FLASH_PAGE_SIZE_BYTES))

// On-Board FXOS8700 Sensor Information
#define MMA8652_BB_I2C_ADDR 0x1D
#define MMA8652_BB_INT1 D8

/* @brief  Ask use input to resume after specified samples have been processed. */
#define ASK_USER_TO_RESUME(x)                                                          \
    static bool askResume = true;                                                      \
    static uint16_t samplesToProcess = x - 1;                                          \
    if (askResume && !samplesToProcess--)                                              \
    {                                                                                  \
        PRINTF("\r\n Specified samples processed, press any key to continue... \r\n"); \
        GETCHAR();                                                                     \
        askResume = false;                                                             \
    }

/* @brief dummy arguement to Power Mode Wait Wrapper. */
#define SMC NULL

/* @brief Kinetis style Wrapper API for Power Mode Wait (Wait for Interrupt). */
status_t SMC_SetPowerModeWait(void *);

/* @brief Kinetis style Wrapper API for Power Mode Wait (Vlpr for Interrupt). */
status_t SMC_SetPowerModeVlpr(void *);

#endif /* QN9080_H_ */
