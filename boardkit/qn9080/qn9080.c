/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file lpc54114.c
 * @brief The lpc54114.c file defines GPIO pins and I2C CMSIS utilities for LPCXpresso54114 board.
 */

#include "qn9080.h"

// I2C0 Handle
gpioHandleKSDK_t D15 = {.base = GPIOA,
                        .pinNumber = 6,
                        .mask = 1 << (6),
                        .irq = GPIOA_IRQn,
                        .clockName = kCLOCK_Gpio,
                        .portNumber = PORTA_NUM};
gpioHandleKSDK_t D14 = {.base = GPIOA,
                        .pinNumber = 7,
                        .mask = 1 << (7),
                        .irq = GPIOA_IRQn,
                        .clockName = kCLOCK_Gpio,
                        .portNumber = PORTA_NUM};

// I2C1/SPI0 Handle
gpioHandleKSDK_t D13 = {.base = GPIOA,
                        .pinNumber = 30,
                        .mask = 1 << (30),
                        .irq = GPIOA_IRQn,
                        .clockName = kCLOCK_Gpio,
                        .portNumber = PORTA_NUM};
gpioHandleKSDK_t D12 = {.base = GPIOA,
                        .pinNumber = 5,
                        .mask = 1 << (5),
                        .irq = GPIOA_IRQn,
                        .clockName = kCLOCK_Gpio,
                        .portNumber = PORTA_NUM};
gpioHandleKSDK_t D11 = {.base = GPIOA,
                        .pinNumber = 4,
                        .mask = 1 << (4),
                        .irq = GPIOA_IRQn,
                        .clockName = kCLOCK_Gpio,
                        .portNumber = PORTA_NUM};

// UART0 Handle
gpioHandleKSDK_t D0 = {.base = GPIOA,
                       .pinNumber = 17,
                       .mask = 1 << (17),
                       .irq = GPIOA_IRQn,
                       .clockName = kCLOCK_Gpio,
                       .portNumber = PORTA_NUM};
gpioHandleKSDK_t D1 = {.base = GPIOA,
                       .pinNumber = 16,
                       .mask = 1 << (16),
                       .irq = GPIOA_IRQn,
                       .clockName = kCLOCK_Gpio,
                       .portNumber = PORTA_NUM};

// QN9080 Arduino Connector Pin Defintion
gpioHandleKSDK_t A0 = {.base = GPIOA,
                       .pinNumber = 11,
                       .mask = 1 << (11),
                       .irq = GPIOA_IRQn,
                       .clockName = kCLOCK_Gpio,
                       .portNumber = PORTA_NUM};
gpioHandleKSDK_t A1 = {.base = GPIOA,
                       .pinNumber = 10,
                       .mask = 1 << (10),
                       .irq = GPIOA_IRQn,
                       .clockName = kCLOCK_Gpio,
                       .portNumber = PORTA_NUM};
gpioHandleKSDK_t A2 = {.base = GPIOA,
                       .pinNumber = 8,
                       .mask = 1 << (8),
                       .irq = GPIOA_IRQn,
                       .clockName = kCLOCK_Gpio,
                       .portNumber = PORTA_NUM};
gpioHandleKSDK_t A3 = {.base = GPIOA,
                       .pinNumber = 9,
                       .mask = 1 << (9),
                       .irq = GPIOA_IRQn,
                       .clockName = kCLOCK_Gpio,
                       .portNumber = PORTA_NUM};
gpioHandleKSDK_t A4 = {.base = GPIOA,
                       .pinNumber = 1,
                       .mask = 1 << (1),
                       .irq = GPIOA_IRQn,
                       .clockName = kCLOCK_Gpio,
                       .portNumber = PORTA_NUM};
gpioHandleKSDK_t A5 = {.base = GPIOA,
                       .pinNumber = 0,
                       .mask = 1 << (0),
                       .irq = GPIOA_IRQn,
                       .clockName = kCLOCK_Gpio,
                       .portNumber = PORTA_NUM};
gpioHandleKSDK_t D2 = {.base = GPIOA,
                       .pinNumber = 21,
                       .mask = 1 << (21),
                       .irq = GPIOA_IRQn,
                       .clockName = kCLOCK_Gpio,
                       .portNumber = PORTA_NUM};
gpioHandleKSDK_t D3 = {.base = GPIOA,
                       .pinNumber = 29,
                       .mask = 1 << (29),
                       .irq = GPIOA_IRQn,
                       .clockName = kCLOCK_Gpio,
                       .portNumber = PORTA_NUM};
gpioHandleKSDK_t D4 = {.base = GPIOA,
                       .pinNumber = 18,
                       .mask = 1 << (18),
                       .irq = GPIOA_IRQn,
                       .clockName = kCLOCK_Gpio,
                       .portNumber = PORTA_NUM};
gpioHandleKSDK_t D5 = {.base = GPIOA,
                       .pinNumber = 15,
                       .mask = 1 << (15),
                       .irq = GPIOA_IRQn,
                       .clockName = kCLOCK_Gpio,
                       .portNumber = PORTA_NUM};
gpioHandleKSDK_t D6 = {.base = GPIOA,
                       .pinNumber = 20,
                       .mask = 1 << (20),
                       .irq = GPIOA_IRQn,
                       .clockName = kCLOCK_Gpio,
                       .portNumber = PORTA_NUM};
gpioHandleKSDK_t D7 = {.base = GPIOA,
                       .pinNumber = 12,
                       .mask = 1 << (12),
                       .irq = GPIOA_IRQn,
                       .clockName = kCLOCK_Gpio,
                       .portNumber = PORTA_NUM};
gpioHandleKSDK_t D8 = {.base = GPIOA,
                       .pinNumber = 14,
                       .mask = 1 << (14),
                       .irq = GPIOA_IRQn,
                       .clockName = kCLOCK_Gpio,
                       .portNumber = PORTA_NUM};
gpioHandleKSDK_t D9 = {.base = GPIOA,
                       .pinNumber = 2,
                       .mask = 1 << (2),
                       .irq = GPIOA_IRQn,
                       .clockName = kCLOCK_Gpio,
                       .portNumber = PORTA_NUM};
gpioHandleKSDK_t D10 = {.base = GPIOA,
                        .pinNumber = 3,
                        .mask = 1 << (3),
                        .irq = GPIOA_IRQn,
                        .clockName = kCLOCK_Gpio,
                        .portNumber = PORTA_NUM};

// QN9080 Internal Peripheral Pin Definitions
gpioHandleKSDK_t RED_LED = {.base = GPIOA,
                            .pinNumber = 31,
                            .mask = (uint32_t)1 << (31),
                            .irq = GPIOA_IRQn,
                            .clockName = kCLOCK_Gpio,
                            .portNumber = PORTA_NUM};
gpioHandleKSDK_t GREEN_LED = {.base = GPIOA,
                              .pinNumber = 13,
                              .mask = 1 << (13),
                              .irq = GPIOA_IRQn,
                              .clockName = kCLOCK_Gpio,
                              .portNumber = PORTA_NUM};
gpioHandleKSDK_t BLUE_LED = {.base = GPIOA,
                             .pinNumber = 25,
                             .mask = 1 << (25),
                             .irq = GPIOA_IRQn,
                             .clockName = kCLOCK_Gpio,
                             .portNumber = PORTA_NUM};

/*!
 * @brief Configures the system to WAIT power mode.
 *        API name used from Kinetis family to maintain compatibility.
 *
 * @param Power peripheral base address (dummy).
 * @return Configuration error code.
 */
status_t SMC_SetPowerModeWait(void *arg)
{
    POWER_EnterSleep();

    return kStatus_Success;
}

/*!
 * @brief Configures the system to VLPR power mode.
 *        API name used from Kinetis family to maintain compatibility.
 *
 * @param Power peripheral base address (dummy).
 * @return Configuration error code.
 */
status_t SMC_SetPowerModeVlpr(void *arg)
{
    POWER_EnterSleep();

    return kStatus_Success;
}

/*! @brief       Determines the Clock Frequency feature.
 *  @details     The Clock Frequecny computation API required by fsl_uart_cmsis.c.
 *  @param[in]   void
 *  @Constraints None
 *  @Reentrant   Yes
 *  @return      uint32_t Returns the clock frequency .
 */
uint32_t USART0_GetFreq(void)
{
    return CLOCK_GetFreq(kCLOCK_BusClk);
}

/*! @brief       Determines the Clock Frequency feature.
 *  @details     The Clock Frequecny computation API required by fsl_i2c_cmsis.c.
 *  @param[in]   void
 *  @Constraints None
 *  @Reentrant   Yes
 *  @return      uint32_t Returns the clock frequency .
 */
uint32_t I2C0_GetFreq(void)
{
    return CLOCK_GetFreq(kCLOCK_BusClk);
}

/*! @brief       Determines the Clock Frequency feature.
 *  @details     The Clock Frequecny computation API required by fsl_i2c_cmsis.c.
 *  @param[in]   void
 *  @Constraints None
 *  @Reentrant   Yes
 *  @return      uint32_t Returns the clock frequency .
 */
uint32_t I2C1_GetFreq(void)
{
    return CLOCK_GetFreq(kCLOCK_BusClk);
}

/*! @brief       Determines the Clock Frequency feature.
 *  @details     The Clock Frequecny computation API required by fsl_spi_cmsis.c.
 *  @param[in]   void
 *  @Constraints None
 *  @Reentrant   Yes
 *  @return      uint32_t Returns the clock frequency .
 */
uint32_t SPI0_GetFreq(void)
{
    return CLOCK_GetFreq(kCLOCK_BusClk);
}

/*! @brief       Determines the Clock Frequency feature.
 *  @details     The Clock Frequecny computation API required by fsl_spi_cmsis.c.
 *  @param[in]   void
 *  @Constraints None
 *  @Reentrant   Yes
 *  @return      uint32_t Returns the clock frequency .
 */
uint32_t SPI1_GetFreq(void)
{
    return CLOCK_GetFreq(kCLOCK_BusClk);
}
