/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _BOARD_H_
#define _BOARD_H_

#include "clock_config.h"
#include "fsl_common.h"
#include "fsl_gpio.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*! @brief The board name */
#define BOARD_NAME "LPCXPRESSO54114"

#define BOARD_EXTCLKINRATE (0)

/*! @brief The UART to use for debug messages. */
#define BOARD_DEBUG_UART_TYPE DEBUG_CONSOLE_DEVICE_TYPE_FLEXCOMM
#define BOARD_DEBUG_UART_BASEADDR (uint32_t) USART0
#define BOARD_DEBUG_UART_CLK_FREQ CLOCK_GetFreq(kCLOCK_Flexcomm0)
#define BOARD_DEBUG_UART_CLK_ATTACH kFRO12M_to_FLEXCOMM0
#define BOARD_DEBUG_UART_RST kFC0_RST_SHIFT_RSTn

#define BOARD_DEBUG_SPI_CLK_FREQ 12000000

#ifndef BOARD_DEBUG_UART_BAUDRATE
#define BOARD_DEBUG_UART_BAUDRATE 115200
#endif /* BOARD_DEBUG_UART_BAUDRATE */

#define BOARD_LED_RED_GPIO GPIO
#define BOARD_LED_RED_GPIO_PORT 0U
#define BOARD_LED_RED_GPIO_PIN 29U
#define BOARD_LED_GREEN_GPIO GPIO
#define BOARD_LED_GREEN_GPIO_PORT 1U
#define BOARD_LED_GREEN_GPIO_PIN 10U
#define BOARD_LED_BLUE_GPIO GPIO
#define BOARD_LED_BLUE_GPIO_PORT 1U
#define BOARD_LED_BLUE_GPIO_PIN 9U

#define BOARD_SW1_GPIO GPIO
#define BOARD_SW1_GPIO_PORT 0U
#define BOARD_SW1_GPIO_PIN 24U
#define BOARD_SW1_NAME "SW1"
#define BOARD_SW3_IRQ PIN_INT0_IRQn
#define BOARD_SW3_IRQ_HANDLER PIN_INT0_IRQHandler

#define BOARD_SW2_GPIO GPIO
#define BOARD_SW2_GPIO_PORT 0U
#define BOARD_SW2_GPIO_PIN 31U
#define BOARD_SW2_NAME "SW2"
#define BOARD_SW3_IRQ PIN_INT0_IRQn
#define BOARD_SW3_IRQ_HANDLER PIN_INT0_IRQHandler

#define BOARD_SW3_GPIO GPIO
#define BOARD_SW3_GPIO_PORT 0U
#define BOARD_SW3_GPIO_PIN 4U
#define BOARD_SW3_NAME "SW3"
#define BOARD_SW3_IRQ PIN_INT0_IRQn
#define BOARD_SW3_IRQ_HANDLER PIN_INT0_IRQHandler
#define BOARD_SW3_GPIO_PININT_INDEX 0

/* Board led color mapping */
#define LOGIC_LED_ON 0U
#define LOGIC_LED_OFF 1U

#define LED_RED_INIT(output)                                                          \
    GPIO_PinInit(BOARD_LED_RED_GPIO, BOARD_LED_RED_GPIO_PORT, BOARD_LED_RED_GPIO_PIN, \
                 &(gpio_pin_config_t){kGPIO_DigitalOutput, (output)}) /*!< Enable target LED_RED */
#define LED_RED_ON()                                                  \
    GPIO_ClearPinsOutput(BOARD_LED_RED_GPIO, BOARD_LED_RED_GPIO_PORT, \
                         1U << BOARD_LED_RED_GPIO_PIN) /*!< Turn on target LED_RED */
#define LED_RED_OFF()                                               \
    GPIO_SetPinsOutput(BOARD_LED_RED_GPIO, BOARD_LED_RED_GPIO_PORT, \
                       1U << BOARD_LED_RED_GPIO_PIN) /*!< Turn off target LED_RED */
#define LED_RED_TOGGLE()                                               \
    GPIO_TogglePinsOutput(BOARD_LED_RED_GPIO, BOARD_LED_RED_GPIO_PORT, \
                          1U << BOARD_LED_RED_GPIO_PIN) /*!< Toggle on target LED_RED */

#define LED_GREEN_INIT(output)                                                              \
    GPIO_PinInit(BOARD_LED_GREEN_GPIO, BOARD_LED_GREEN_GPIO_PORT, BOARD_LED_GREEN_GPIO_PIN, \
                 &(gpio_pin_config_t){kGPIO_DigitalOutput, (output)}) /*!< Enable target LED_GREEN */
#define LED_GREEN_ON()                                                    \
    GPIO_ClearPinsOutput(BOARD_LED_GREEN_GPIO, BOARD_LED_GREEN_GPIO_PORT, \
                         1U << BOARD_LED_GREEN_GPIO_PIN) /*!< Turn on target LED_GREEN */
#define LED_GREEN_OFF()                                                 \
    GPIO_SetPinsOutput(BOARD_LED_GREEN_GPIO, BOARD_LED_GREEN_GPIO_PORT, \
                       1U << BOARD_LED_GREEN_GPIO_PIN) /*!< Turn off target LED_GREEN */
#define LED_GREEN_TOGGLE()                                                 \
    GPIO_TogglePinsOutput(BOARD_LED_GREEN_GPIO, BOARD_LED_GREEN_GPIO_PORT, \
                          1U << BOARD_LED_GREEN_GPIO_PIN) /*!< Toggle on target LED_GREEN */

#define LED_BLUE_INIT(output)                                                            \
    GPIO_PinInit(BOARD_LED_BLUE_GPIO, BOARD_LED_BLUE_GPIO_PORT, BOARD_LED_BLUE_GPIO_PIN, \
                 &(gpio_pin_config_t){kGPIO_DigitalOutput, (output)}) /*!< Enable target LED_BLUE */
#define LED_BLUE_ON()                                                   \
    GPIO_ClearPinsOutput(BOARD_LED_BLUE_GPIO, BOARD_LED_BLUE_GPIO_PORT, \
                         1U << BOARD_LED_BLUE_GPIO_PIN) /*!< Turn on target LED_BLUE */
#define LED_BLUE_OFF()                                                \
    GPIO_SetPinsOutput(BOARD_LED_BLUE_GPIO, BOARD_LED_BLUE_GPIO_PORT, \
                       1U << BOARD_LED_BLUE_GPIO_PIN) /*!< Turn off target LED_BLUE */
#define LED_BLUE_TOGGLE()                                                \
    GPIO_TogglePinsOutput(BOARD_LED_BLUE_GPIO, BOARD_LED_BLUE_GPIO_PORT, \
                          1U << BOARD_LED_BLUE_GPIO_PIN) /*!< Toggle on target LED_BLUE */

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

/*******************************************************************************
 * API
 ******************************************************************************/

status_t BOARD_InitDebugConsole(void);

#if defined(__cplusplus)
}
#endif /* __cplusplus */

#endif /* _BOARD_H_ */
