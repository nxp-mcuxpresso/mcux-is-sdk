/*
 * Copyright 2022 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file evkmimxrt1180.c
 * @brief The evkmimxrt1180.c file defines GPIO pins and CMSIS utilities for
 * evkmimxrt1180 board.
 */

#include "evkmimxrt1180.h"

/* Get frequency of lpi2c clock */
#define LPI2C_CLOCK_FREQUENCY (CLOCK_GetRootClockFreq(kCLOCK_Root_Lpi2c0102))
/* Get frequency of lpspi clock */
#define LPSPI_CLOCK_FREQUENCY (CLOCK_GetRootClockFreq(kCLOCK_Root_Lpspi0304))
/* Get frequency of lpspi clock */
#define LPUART_CLOCK_FREQUENCY (CLOCK_GetRootClockFreq(kCLOCK_Root_Lpuart0102))

gpioHandleiMXSDK_t D2 = {
    .base = RGPIO4, .pinNumber = 15, .mask = 1 << (15), .irq = GPIO4_0_IRQn, .portNumber = GPIO4_NUM};
gpioHandleiMXSDK_t D10 = {
    .base = RGPIO5, .pinNumber = 04, .mask = 1 << (04), .irq = GPIO5_0_IRQn, .portNumber = GPIO5_NUM};
gpioHandleiMXSDK_t LED = {
    .base = RGPIO4, .pinNumber = 27, .mask = 1 << (27), .irq = GPIO4_0_IRQn, .portNumber = GPIO4_NUM};

/*! @brief       Determines the Clock Frequency feature.
 *  @details     The Clock Frequecny computation API required by
 * fsl_uart_cmsis.c.
 *  @param[in]   void
 *  @Constraints None
 *  @Reentrant   Yes
 *  @return      uint32_t Returns the clock frequency .
 */
uint32_t LPUART1_GetFreq(void)
{
    return LPUART_CLOCK_FREQUENCY;
}

/*! @brief       Determines the Clock Frequency feature.
 *  @details     The Clock Frequecny computation API required by
 * fsl_i2c_cmsis.c.
 *  @param[in]   void
 *  @Constraints None
 *  @Reentrant   Yes
 *  @return      uint32_t Returns the clock frequency .
 */
uint32_t LPI2C2_GetFreq(void)
{
    return LPI2C_CLOCK_FREQUENCY;
}

/*! @brief       Determines the Clock Frequency feature.
 *  @details     The Clock Frequecny computation API required by
 * fsl_spi_cmsis.c.
 *  @param[in]   void
 *  @Constraints None
 *  @Reentrant   Yes
 *  @return      uint32_t Returns the clock frequency .
 */
uint32_t LPSPI3_GetFreq(void)
{
    return LPSPI_CLOCK_FREQUENCY;
}
