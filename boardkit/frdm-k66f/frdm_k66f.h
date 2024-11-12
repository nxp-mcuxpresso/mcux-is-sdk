/*
* Copyright (c) 2016, Freescale Semiconductor, Inc.
* All rights reserved.
*
*
* SPDX-License-Identifier: BSD-3-Clause
*/

/*! \file frdm_k66f.h
    \brief The \b frdm_k66f.h file defines GPIO pins for FRDM-K66F board
*/

#ifndef FRDM_K66F_H_
#define FRDM_K66F_H_

#include "pin_mux.h"
#include "MK66F18.h"
#include "RTE_Device.h"
#include "gpio_driver.h"

#include "fsl_flash.h"
#include "fsl_uart_cmsis.h"

// I2C0 Pin Handles
extern gpioHandleKSDK_t D15;
extern gpioHandleKSDK_t D14;

// I2C1 Handle
extern gpioHandleKSDK_t A5;
extern gpioHandleKSDK_t A4;

// SPI0 Handle
extern gpioHandleKSDK_t D13;
extern gpioHandleKSDK_t D11;
extern gpioHandleKSDK_t D12;

// UART3 Handle
extern gpioHandleKSDK_t D0;
extern gpioHandleKSDK_t D1;

// FRDM-K66F Arduino Connector Pin Defintion
extern gpioHandleKSDK_t A0;
extern gpioHandleKSDK_t A1;
extern gpioHandleKSDK_t A2;
extern gpioHandleKSDK_t A3;
extern gpioHandleKSDK_t D2;
extern gpioHandleKSDK_t D3;
extern gpioHandleKSDK_t D4;
extern gpioHandleKSDK_t D5;
extern gpioHandleKSDK_t D6;
extern gpioHandleKSDK_t D6;
extern gpioHandleKSDK_t D8;
extern gpioHandleKSDK_t D9;
extern gpioHandleKSDK_t D10;

// FRDM-K66F Internal Peripheral Pin Definitions
extern gpioHandleKSDK_t RED_LED;
extern gpioHandleKSDK_t GREEN_LED;
extern gpioHandleKSDK_t BLUE_LED;

// I2C_S1: Pin mapping and driver information for default I2C brought to shield
#define I2C_S1_SCL_PIN A5
#define I2C_S1_SDA_PIN A4
#define I2C_S1_DRIVER_NONBLOCKING Driver_I2C0_KSDK2_NonBlocking
#define I2C_S1_DRIVER_BLOCKING Driver_I2C0_KSDK2_Blocking

// I2C_S2: Pin mapping and driver information for alternate I2C bus on shield
#define I2C_S2_SCL_PIN D15
#define I2C_S2_SDA_PIN D14
#define I2C_S2_DRIVER_NONBLOCKING Driver_I2C1_KSDK2_NonBlocking
#define I2C_S2_DRIVER_BLOCKING Driver_I2C1_KSDK2_Blocking

// I2C_B: Pin mapping and driver information for I2C routed on K66F base board
#define I2C_BB_SCL_PIN SCL
#define I2C_BB_SDA_PIN SCL
#define I2C_BB_DRIVER I2C_S0_DRIVER
#define I2C_BB_SIGNAL_EVENT I2C_S0_SIGNAL_EVENT
#define I2C_BB_DEVICE_INDEX I2C_S0_DEVICE_INDEX

// SPIS: Pin mapping and driver information default SPI brought to shield
#define SPI_S_SCLK D13
#define SPI_S_MOSI D11
#define SPI_S_MISO D12
#define SPI_S_DRIVER_NONBLOCKING Driver_SPI0_KSDK2_NonBlocking
#define SPI_S_DRIVER_BLOCKING Driver_SPI0_KSDK2_Blocking

/* @brief  Block for the specified milliseconds. */
#define BOARD_DELAY_CORRECTION 2.3
#define BOARD_DELAY_ms(x)                                                                                           \
    for (uint32_t delay = 0; delay < BOARD_DELAY_CORRECTION * USEC_TO_COUNT(x, CLOCK_GetCoreSysClkFreq()); delay++) \
    __NOP()

/* @brief  Ask use input to resume after specified samples have been processed. */
#define ASK_USER_TO_RESUME(x)                                                          \
    static volatile bool askResume = true;                                             \
    static uint16_t samplesToProcess = x - 1;                                          \
    if (askResume && !samplesToProcess--)                                              \
    {                                                                                  \
        PRINTF("\r\n Specified samples processed, press any key to continue... \r\n"); \
        GETCHAR();                                                                     \
        askResume = false;                                                             \
    }

// UART: Driver information for default UART to communicate with HOST PC.
#define HOST_S_DRIVER UART0_NonBlocking_Driver
#define HOST_S_DEVICE_BASE UART0
#define HOST_S_CMSIS_HANDLE UART0_Handle
#define HOST_S_SIGNAL_EVENT HOST_SignalEvent_t

// Miscellaneous Hardware Configuration Parameters
#define THIS_BOARD 20                   ///< FRDM_K66F
#define CORE_SYSTICK_HZ 120000000       ///< core and systick clock rate (Hz)
#define CALIBRATION_NVM_ADDR 0x001FF000 ///< start of final 4KB (sector size) of 1MB flash
#define FLASH_SECTOR_SIZE_PROPERTY kFLASH_PropertyPflashSectorSize
#define FLASH_ERASE_KEY kFLASH_ApiEraseKey

// offsets from start of NVM block for calibration coefficients
#define MAG_NVM_OFFSET 0     // 68 bytes used
#define GYRO_NVM_OFFSET 100  // 16 bytes used
#define ACCEL_NVM_OFFSET 140 // 88 bytes used
///@}

#define BB_FXOS8700_I2C_ADDR 0x1D
#define BB_FXAS21002_I2C_ADDR 0x21

#endif /* FRDM_K66F_H_ */
