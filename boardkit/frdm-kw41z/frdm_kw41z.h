/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file frdm_KW41Z.h
 * @brief The frdm_KW41Z.h file defines GPIO pin mappings for FRDM-KW41Z board
 */

#ifndef FRDM_KW41Z_H_
#define FRDM_KW41Z_H_

#include "pin_mux.h"
#include "fsl_smc.h"
#include "MKW41Z4.h"
#include "RTE_Device.h"
#include "gpio_driver.h"

// I2C1 Handle
extern gpioHandleKSDK_t D15;
extern gpioHandleKSDK_t D14;

// I2C0 Handle
extern gpioHandleKSDK_t A5;
extern gpioHandleKSDK_t A4;

// SPI0 Handle
extern gpioHandleKSDK_t D13;
extern gpioHandleKSDK_t D11;
extern gpioHandleKSDK_t D12;

// UART0 Handle
extern gpioHandleKSDK_t D0;
extern gpioHandleKSDK_t D1;

// FRDM-KW41Z Arduino Connector Pin Defintion
extern gpioHandleKSDK_t A0;
extern gpioHandleKSDK_t A1;
extern gpioHandleKSDK_t A2;
extern gpioHandleKSDK_t A3;
extern gpioHandleKSDK_t D2;
extern gpioHandleKSDK_t D4;
extern gpioHandleKSDK_t D5;
extern gpioHandleKSDK_t D6;
extern gpioHandleKSDK_t D7;
extern gpioHandleKSDK_t D8;
extern gpioHandleKSDK_t D9;
extern gpioHandleKSDK_t D10;

// FRDM-KW41Z Internal Peripheral Pin Definitions
extern gpioHandleKSDK_t RED_LED;
extern gpioHandleKSDK_t GREEN_LED;
extern gpioHandleKSDK_t BLUE_LED;

// I2C_S1: Pin mapping and driver information for default I2C brought to shield
#define I2C_S1_SCL_PIN D15
#define I2C_S1_SDA_PIN D14
#define I2C_S1_DRIVER Driver_I2C1
#define I2C_S1_DEVICE_INDEX I2C1_INDEX
#define I2C_S1_SIGNAL_EVENT I2C1_SignalEvent_t

// I2C_S2: Pin mapping and driver information for alternate I2C bus on shield
#define I2C_S2_SCL_PIN A5
#define I2C_S2_SDA_PIN A4
#define I2C_S2_DRIVER Driver_I2C0
#define I2C_S2_DEVICE_INDEX I2C0_INDEX
#define I2C_S2_SIGNAL_EVENT I2C0_SignalEvent_t

// I2C_BB: PPin mapping and driver information for I2C routed on KW41Z base board
#define I2C_BB_SCL_PIN ACCEL_SCL
#define I2C_BB_SDA_PIN ACCEL_SDA
#define I2C_BB_DRIVER Driver_I2C1
#define I2C_BB_DEVICE_INDEX I2C1_INDEX
#define I2C_BB_SIGNAL_EVENT I2C1_SignalEvent_t

// SPI_S: Pin mapping and driver information default SPI brought to shield
#define SPI_S_SCLK D13
#define SPI_S_MOSI D11
#define SPI_S_MISO D12
#define SPI_S_DRIVER Driver_SPI1
#define SPI_S_BAUDRATE 500000U ///< Transfer baudrate - 500k
#define SPI_S_DEVICE_INDEX SPI1_INDEX
#define SPI_S_SIGNAL_EVENT SPI1_SignalEvent_t

// UART: Driver information for default UART to communicate with HOST PC.
#define HOST_S_DRIVER Driver_USART0
#define HOST_S_SIGNAL_EVENT HOST_SignalEvent_t

// Bluetooth: Driver information for default UART to communicate with HOST PC.
#define HOST_B_DRIVER Driver_USART0
#define HOST_B_SIGNAL_EVENT HOST_SignalEvent_t

// On-Board FXOS8700 Sensor Information
#define FXOS8700_BB_I2C_ADDR 0x1F
#define FXOS8700_BB_INT1 D9

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

// Set GPIO pin for handling the gpio interrupt using PORTB_PORTC_PORTD_PORTE_IRQHandler
// By default, GPIO Pin set to GPIOC
#define GPIO_TYPE GPIOC
#define GPIO_PORT_NUM PORTC_NUM

#define ADS_NVM_ADDR 0x0007F000 ///< start of the next to last 2KB (sector size) of the 512K flash
#define FLASH_SECTOR_SIZE_PROPERTY kFLASH_PropertyPflashSectorSize

/* Release the onboard FXOS8700 Bus */
void BOARD_ACCEL_ReleaseBus(void);

#endif /* FRDM_KW41Z_H_ */
