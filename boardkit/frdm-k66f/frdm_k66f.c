/*
* Copyright (c) 2016, Freescale Semiconductor, Inc.
* All rights reserved.
*
*
* SPDX-License-Identifier: BSD-3-Clause
*/

/*! \file frdm_k64f.h
    \brief The \b frdm_k64f.h file defines GPIO pins for FRDM-K64F board
*/

#ifndef FRDM_K64F_H_
#define FRDM_K64F_H_

#include "frdm_k66f.h"

// On Shield I2C0 Pin Handles
gpioHandleKSDK_t D15 = {.base = GPIOC,
                        .portBase = PORTC,
                        .pinNumber = 10,
                        .mask = 1 << (10),
                        .irq = PORTC_IRQn,
                        .clockName = kCLOCK_PortC,
                        .portNumber = PORTC_NUM};
gpioHandleKSDK_t D14 = {.base = GPIOC,
                        .portBase = PORTC,
                        .pinNumber = 11,
                        .mask = 1 << (11),
                        .irq = PORTC_IRQn,
                        .clockName = kCLOCK_PortC,
                        .portNumber = PORTC_NUM};

// On board I2C0 Pin Handles
gpioHandleKSDK_t SCL = {.base = GPIOD,
                        .portBase = PORTD,
                        .pinNumber = 8,
                        .mask = 1 << (8),
                        .irq = PORTD_IRQn,
                        .clockName = kCLOCK_PortD,
                        .portNumber = PORTD_NUM};
gpioHandleKSDK_t SDA = {.base = GPIOD,
                        .portBase = PORTD,
                        .pinNumber = 9,
                        .mask = 1 << (9),
                        .irq = PORTD_IRQn,
                        .clockName = kCLOCK_PortD,
                        .portNumber = PORTD_NUM};

// I2C1 Handle
gpioHandleKSDK_t A5 = {.base = GPIOB,
                       .portBase = PORTB,
                       .pinNumber = 2,
                       .mask = 1 << (2),
                       .irq = PORTB_IRQn,
                       .clockName = kCLOCK_PortB,
                       .portNumber = PORTB_NUM};
gpioHandleKSDK_t A4 = {.base = GPIOB,
                       .portBase = PORTB,
                       .pinNumber = 3,
                       .mask = 1 << (3),
                       .irq = PORTB_IRQn,
                       .clockName = kCLOCK_PortB,
                       .portNumber = PORTB_NUM};

// FRDM-K64F Arduino Connector Pin Defintion
gpioHandleKSDK_t A0 = {.base = GPIOB,
                       .portBase = PORTB,
                       .pinNumber = 7,
                       .mask = 1 << (7),
                       .irq = PORTB_IRQn,
                       .clockName = kCLOCK_PortB,
                       .portNumber = PORTB_NUM};
gpioHandleKSDK_t A1 = {.base = GPIOB,
                       .portBase = PORTB,
                       .pinNumber = 6,
                       .mask = 1 << (6),
                       .irq = PORTB_IRQn,
                       .clockName = kCLOCK_PortB,
                       .portNumber = PORTB_NUM};
gpioHandleKSDK_t A2 = {.base = GPIOB,
                       .portBase = PORTB,
                       .pinNumber = 5,
                       .mask = 1 << (5),
                       .irq = PORTB_IRQn,
                       .clockName = kCLOCK_PortB,
                       .portNumber = PORTB_NUM};
gpioHandleKSDK_t A3 = {.base = GPIOB,
                       .portBase = PORTB,
                       .pinNumber = 4,
                       .mask = 1 << (4),
                       .irq = PORTB_IRQn,
                       .clockName = kCLOCK_PortB,
                       .portNumber = PORTB_NUM};

gpioHandleKSDK_t D0 = {.base = GPIOC,
                       .portBase = PORTC,
                       .pinNumber = 3,
                       .mask = 1 << (3),
                       .irq = PORTC_IRQn,
                       .clockName = kCLOCK_PortC,
                       .portNumber = PORTC_NUM};
gpioHandleKSDK_t D1 = {.base = GPIOC,
                       .portBase = PORTC,
                       .pinNumber = 4,
                       .mask = 1 << (4),
                       .irq = PORTC_IRQn,
                       .clockName = kCLOCK_PortC,
                       .portNumber = PORTC_NUM};
gpioHandleKSDK_t D2 = {.base = GPIOC,
                       .portBase = PORTC,
                       .pinNumber = 16,
                       .mask = 1 << (16),
                       .irq = PORTC_IRQn,
                       .clockName = kCLOCK_PortC,
                       .portNumber = PORTC_NUM};
gpioHandleKSDK_t D3 = {.base = GPIOC,
                       .portBase = PORTC,
                       .pinNumber = 8,
                       .mask = 1 << (8),
                       .irq = PORTC_IRQn,
                       .clockName = kCLOCK_PortC,
                       .portNumber = PORTC_NUM};
gpioHandleKSDK_t D4 = {.base = GPIOC,
                       .portBase = PORTC,
                       .pinNumber = 12,
                       .mask = 1 << (12),
                       .irq = PORTC_IRQn,
                       .clockName = kCLOCK_PortC,
                       .portNumber = PORTC_NUM};
gpioHandleKSDK_t D5 = {.base = GPIOC,
                       .portBase = PORTC,
                       .pinNumber = 5,
                       .mask = 1 << (5),
                       .irq = PORTC_IRQn,
                       .clockName = kCLOCK_PortC,
                       .portNumber = PORTC_NUM};
gpioHandleKSDK_t D6 = {.base = GPIOC,
                       .portBase = PORTC,
                       .pinNumber = 2,
                       .mask = 1 << (2),
                       .irq = PORTC_IRQn,
                       .clockName = kCLOCK_PortC,
                       .portNumber = PORTC_NUM};
gpioHandleKSDK_t D7 = {.base = GPIOA,
                       .portBase = PORTA,
                       .pinNumber = 25,
                       .mask = 1 << (25),
                       .irq = PORTA_IRQn,
                       .clockName = kCLOCK_PortA,
                       .portNumber = PORTA_NUM};
gpioHandleKSDK_t D8 = {.base = GPIOB,
                       .portBase = PORTB,
                       .pinNumber = 18,
                       .mask = 1 << (18),
                       .irq = PORTB_IRQn,
                       .clockName = kCLOCK_PortB,
                       .portNumber = PORTB_NUM};
gpioHandleKSDK_t D9 = {.base = GPIOB,
                       .portBase = PORTB,
                       .pinNumber = 19,
                       .mask = 1 << (19),
                       .irq = PORTB_IRQn,
                       .clockName = kCLOCK_PortB,
                       .portNumber = PORTB_NUM};
gpioHandleKSDK_t D10 = {.base = GPIOD,
                        .portBase = PORTD,
                        .pinNumber = 0,
                        .mask = 1 << (0),
                        .irq = PORTD_IRQn,
                        .clockName = kCLOCK_PortD,
                        .portNumber = PORTD_NUM};
gpioHandleKSDK_t D11 = {.base = GPIOD,
                        .portBase = PORTD,
                        .pinNumber = 2,
                        .mask = 1 << (2),
                        .irq = PORTD_IRQn,
                        .clockName = kCLOCK_PortD,
                        .portNumber = PORTD_NUM};
gpioHandleKSDK_t D12 = {.base = GPIOD,
                        .portBase = PORTD,
                        .pinNumber = 3,
                        .mask = 1 << (3),
                        .irq = PORTD_IRQn,
                        .clockName = kCLOCK_PortD,
                        .portNumber = PORTD_NUM};
gpioHandleKSDK_t D13 = {.base = GPIOD,
                        .portBase = PORTD,
                        .pinNumber = 1,
                        .mask = 1 << (1),
                        .irq = PORTD_IRQn,
                        .clockName = kCLOCK_PortD,
                        .portNumber = PORTD_NUM};

gpioHandleKSDK_t RED_LED = {.base = GPIOC,
                            .portBase = PORTC,
                            .pinNumber = 9,
                            .mask = 1 << (9),
                            .irq = PORTC_IRQn,
                            .clockName = kCLOCK_PortC,
                            .portNumber = PORTC_NUM};
gpioHandleKSDK_t GREEN_LED = {.base = GPIOE,
                              .portBase = PORTE,
                              .pinNumber = 6,
                              .mask = 1 << (6),
                              .irq = PORTE_IRQn,
                              .clockName = kCLOCK_PortE,
                              .portNumber = PORTE_NUM};
gpioHandleKSDK_t BLUE_LED = {.base = GPIOA,
                             .portBase = PORTA,
                             .pinNumber = 11,
                             .mask = 1 << (11),
                             .irq = PORTA_IRQn,
                             .clockName = kCLOCK_PortA,
                             .portNumber = PORTA_NUM};

/*! @brief       Determines the Clock Frequency feature.
 *  @details     The Clock Frequecny computation API required by fsl_i2c_cmsis.c.
 *  @param[in]   void
 *  @Constraints None
 *  @Reentrant   Yes
 *  @return      uint32_t Returns the clock frequency .
 */
uint32_t I2C0_GetFreq(void)
{
    return CLOCK_GetFreq(I2C0_CLK_SRC);
}
uint32_t UART0_GetFreq(void)
{
    return CLOCK_GetFreq(UART0_CLK_SRC);
}

#endif /* FRDM_K64F_H_ */
