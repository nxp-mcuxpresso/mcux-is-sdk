/*
 * Copyright 2022 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file evkmimxrt1180.h
 * @brief The evkmimxrt1180.h file defines GPIO pin mappings for evkmimxrt1180 board
 */

#ifndef EVKMIMXRT1180_H_
#define EVKMIMXRT1180_H_

#include "pin_mux.h"
#include "RTE_Device.h"
#include "fsl_clock.h"
#include "gpio_driver.h"
#include "fsl_iomuxc.h"

extern gpioHandleiMXSDK_t D2;
extern gpioHandleiMXSDK_t D10;
extern gpioHandleiMXSDK_t LED;

// I2C_S2: Pin mapping and driver information for alternate I2C bus on shield
#define I2C_S2_SCL_PIN      IOMUXC_GPIO_AON_16_LPI2C2_SCL
#define I2C_S2_SDA_PIN      IOMUXC_GPIO_AON_15_LPI2C2_SDA
#define I2C_S2_DRIVER       Driver_I2C2
#define I2C_S2_DEVICE_INDEX I2C2_INDEX
#define I2C_S2_SIGNAL_EVENT I2C2_SignalEvent_t

// SPIS: Pin mapping and driver information default SPI brought to shield
#define SPI_S_SCLK         IOMUXC_GPIO_SD_B1_01_LPSPI3_SCK
#define SPI_S_MISO         IOMUXC_GPIO_SD_B1_03_LPSPI3_SDI
#define SPI_S_MOSI         IOMUXC_GPIO_SD_B1_02_LPSPI3_SDO
//#define FXLS8974_CS        IOMUXC_GPIO_SD_B1_00_LPSPI3_PCS0
#define SPI_S_DRIVER       Driver_SPI3
#define SPI_S_BAUDRATE     500000U ///< Transfer baudrate - 500k
#define SPI_S_DEVICE_INDEX SPI3_INDEX
#define SPI_S_SIGNAL_EVENT SPI3_SignalEvent_t

// UART: Driver information for default UART to communicate with HOST PC.
#define HOST_S_DRIVER       Driver_USART1
#define HOST_S_SIGNAL_EVENT HOST_SignalEvent_t

/* @brief  Ask use input to resume after specified samples have been processed. */
#define ASK_USER_TO_RESUME(x)                                                          \
    static volatile bool askResume   = true;                                           \
    static uint16_t samplesToProcess = x - 1;                                          \
    if (askResume && !samplesToProcess--)                                              \
    {                                                                                  \
        PRINTF("\r\n Specified samples processed, press any key to continue... \r\n"); \
        GETCHAR();                                                                     \
        askResume = false;                                                             \
    }

/* Compatability definitions for evkmimxrt1040 */
#define I2C2          LPI2C2
#define I2C_Type      LPI2C_Type
#define I2C_BASE_PTRS LPI2C_BASE_PTRS
#define SPI3          LPSPI3
#define SPI_Type      LPSPI_Type
#define SPI_BASE_PTRS LPSPI_BASE_PTRS
#define UART1          LPUART1
#define UART_Type      LPUART_Type
#define UART_BASE_PTRS LPUART_BASE_PTRS

/* @brief dummy arguement to Power Mode Wait Wrapper. */
#define SMC NULL

/* @brief Kinetis style Wrapper API for Power Mode Wait (Wait for Interrupt). */
status_t SMC_SetPowerModeWait(void *arg);
/* @brief Kinetis style Wrapper API for Power Mode VLPR (Wait for Interrupt). */
status_t APP_LowPower_EnterLowPower(void *arg);

#endif /* EVKMIMXRT1180_H_ */
