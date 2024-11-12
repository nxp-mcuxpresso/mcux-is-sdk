/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _BOARD_H_
#define _BOARD_H_

#include "clock_config.h"
#include "fsl_gpio.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/* The board name */
#define BOARD_NAME "FRDM-K64F"

/* The UART to use for debug messages. */
#define BOARD_DEBUG_UART_TYPE DEBUG_CONSOLE_DEVICE_TYPE_UART
#define BOARD_DEBUG_UART_BASEADDR (uint32_t) UART0
#define BOARD_DEBUG_UART_CLKSRC SYS_CLK
#define BOARD_DEBUG_UART_CLK_FREQ CLOCK_GetCoreSysClkFreq()
#define BOARD_UART_IRQ UART0_RX_TX_IRQn
#define BOARD_UART_IRQ_HANDLER UART0_RX_TX_IRQHandler

#ifndef BOARD_DEBUG_UART_BAUDRATE
#define BOARD_DEBUG_UART_BAUDRATE 115200
#endif /* BOARD_DEBUG_UART_BAUDRATE */

/* Define the port interrupt number for the board switches */
#define BOARD_SW2_GPIO GPIOC
#define BOARD_SW2_PORT PORTC
#define BOARD_SW2_GPIO_PIN 6U
#define BOARD_SW2_IRQ PORTC_IRQn
#define BOARD_SW2_IRQ_HANDLER PORTC_IRQHandler
#define BOARD_SW2_NAME "SW2"

#define BOARD_SW3_GPIO GPIOA
#define BOARD_SW3_PORT PORTA
#define BOARD_SW3_GPIO_PIN 4U
#define BOARD_SW3_IRQ PORTA_IRQn
#define BOARD_SW3_IRQ_HANDLER PORTA_IRQHandler
#define BOARD_SW3_NAME "SW3"

/* Board led color mapping */
#define LOGIC_LED_ON 0U
#define LOGIC_LED_OFF 1U
#define BOARD_LED_RED_GPIO GPIOB
#define BOARD_LED_RED_GPIO_PORT PORTB
#define BOARD_LED_RED_GPIO_PIN 22U
#define BOARD_LED_GREEN_GPIO GPIOE
#define BOARD_LED_GREEN_GPIO_PORT PORTE
#define BOARD_LED_GREEN_GPIO_PIN 26U
#define BOARD_LED_BLUE_GPIO GPIOB
#define BOARD_LED_BLUE_GPIO_PORT PORTB
#define BOARD_LED_BLUE_GPIO_PIN 21U

#define LED_RED_INIT(output)                                 \
    GPIO_PinInit(BOARD_LED_RED_GPIO, BOARD_LED_RED_GPIO_PIN, \
                 &(gpio_pin_config_t){kGPIO_DigitalOutput, (output)}) /*!< Enable target LED_RED */
#define LED_RED_ON() \
    GPIO_ClearPinsOutput(BOARD_LED_RED_GPIO, 1U << BOARD_LED_RED_GPIO_PIN) /*!< Turn on target LED_RED */
#define LED_RED_OFF() \
    GPIO_SetPinsOutput(BOARD_LED_RED_GPIO, 1U << BOARD_LED_RED_GPIO_PIN) /*!< Turn off target LED_RED */
#define LED_RED_TOGGLE() \
    GPIO_TogglePinsOutput(BOARD_LED_RED_GPIO, 1U << BOARD_LED_RED_GPIO_PIN) /*!< Toggle on target LED_RED */

#define LED_GREEN_INIT(output)                                   \
    GPIO_PinInit(BOARD_LED_GREEN_GPIO, BOARD_LED_GREEN_GPIO_PIN, \
                 &(gpio_pin_config_t){kGPIO_DigitalOutput, (output)}) /*!< Enable target LED_GREEN */
#define LED_GREEN_ON() \
    GPIO_ClearPinsOutput(BOARD_LED_GREEN_GPIO, 1U << BOARD_LED_GREEN_GPIO_PIN) /*!< Turn on target LED_GREEN */
#define LED_GREEN_OFF() \
    GPIO_SetPinsOutput(BOARD_LED_GREEN_GPIO, 1U << BOARD_LED_GREEN_GPIO_PIN) /*!< Turn off target LED_GREEN */
#define LED_GREEN_TOGGLE() \
    GPIO_TogglePinsOutput(BOARD_LED_GREEN_GPIO, 1U << BOARD_LED_GREEN_GPIO_PIN) /*!< Toggle on target LED_GREEN */

#define LED_BLUE_INIT(output)                                  \
    GPIO_PinInit(BOARD_LED_BLUE_GPIO, BOARD_LED_BLUE_GPIO_PIN, \
                 &(gpio_pin_config_t){kGPIO_DigitalOutput, (output)}) /*!< Enable target LED_BLUE */
#define LED_BLUE_ON() \
    GPIO_ClearPinsOutput(BOARD_LED_BLUE_GPIO, 1U << BOARD_LED_BLUE_GPIO_PIN) /*!< Turn on target LED_BLUE */
#define LED_BLUE_OFF() \
    GPIO_SetPinsOutput(BOARD_LED_BLUE_GPIO, 1U << BOARD_LED_BLUE_GPIO_PIN) /*!< Turn off target LED_BLUE */
#define LED_BLUE_TOGGLE() \
    GPIO_TogglePinsOutput(BOARD_LED_BLUE_GPIO, 1U << BOARD_LED_BLUE_GPIO_PIN) /*!< Toggle on target LED_BLUE */

/* The SDHC instance/channel used for board */
#define BOARD_SDHC_CD_GPIO_IRQ_HANDLER PORTB_IRQHandler

/* SDHC base address, clock and card detection pin */
#define BOARD_SDHC_BASEADDR SDHC
#define BOARD_SDHC_CLKSRC kCLOCK_CoreSysClk
#define BOARD_SDHC_IRQ SDHC_IRQn
#define BOARD_SDHC_CD_GPIO_BASE GPIOE
#define BOARD_SDHC_CD_GPIO_PIN 6U
#define BOARD_SDHC_CD_PORT_BASE PORTE
#define BOARD_SDHC_CD_PORT_IRQ PORTE_IRQn
#define BOARD_SDHC_CD_PORT_IRQ_HANDLER PORTE_IRQHandler
#define BOARD_SDHC_CD_LOGIC_RISING

#define BOARD_ACCEL_I2C_BASEADDR I2C0

/* @brief FreeRTOS tickless timer configuration. */
#define vPortLptmrIsr LPTMR0_IRQHandler /*!< Timer IRQ handler. */
#define TICKLESS_LPTMR_BASE_PTR LPTMR0  /*!< Tickless timer base address. */
#define TICKLESS_LPTMR_IRQn LPTMR0_IRQn /*!< Tickless timer IRQ number. */

/* @brief pit IRQ configuration for lwip demo */
#define LWIP_TIME_ISR PIT0_IRQHandler
#define LWIP_PIT_IRQ_ID PIT0_IRQn

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus */

/*******************************************************************************
 * API
 ******************************************************************************/
void BOARD_InitDebugConsole(void);

#if defined(__cplusplus)
}
#endif /* __cplusplus */

#endif /* _BOARD_H_ */
