/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file frdm_KW41Z.c
 * @brief The frdm_KW41Z.c file defines GPIO pins for FRDM-KW41Z board
 */
#include "fsl_gpio.h"
#include "fsl_port.h"
#include "frdm_KW41Z.h"

// I2C1 Handle
gpioHandleKSDK_t D15 = {.base = GPIOC,
                        .portBase = PORTC,
                        .pinNumber = 2,
                        .mask = 1 << (2),
                        .irq = PORTB_PORTC_IRQn,
                        .clockName = kCLOCK_PortC,
                        .portNumber = PORTC_NUM};
gpioHandleKSDK_t D14 = {.base = GPIOC,
                        .portBase = PORTC,
                        .pinNumber = 3,
                        .mask = 1 << (3),
                        .irq = PORTB_PORTC_IRQn,
                        .clockName = kCLOCK_PortC,
                        .portNumber = PORTC_NUM};

// I2C0 Handle
gpioHandleKSDK_t A5 = {.base = GPIOB,
                       .portBase = PORTB,
                       .pinNumber = 0,
                       .mask = 1 << (0),
                       .irq = PORTB_PORTC_IRQn,
                       .clockName = kCLOCK_PortB,
                       .portNumber = PORTB_NUM};
gpioHandleKSDK_t A4 = {.base = GPIOB,
                       .portBase = PORTB,
                       .pinNumber = 1,
                       .mask = 1 << (1),
                       .irq = PORTB_PORTC_IRQn,
                       .clockName = kCLOCK_PortB,
                       .portNumber = PORTB_NUM};

// SPI1 Handle
gpioHandleKSDK_t D13 = {.base = GPIOA,
                        .portBase = PORTA,
                        .pinNumber = 18,
                        .mask = 1 << (18),
                        .irq = PORTA_IRQn,
                        .clockName = kCLOCK_PortA,
                        .portNumber = PORTA_NUM};
gpioHandleKSDK_t D12 = {.base = GPIOA,
                        .portBase = PORTA,
                        .pinNumber = 17,
                        .mask = 1 << (17),
                        .irq = PORTA_IRQn,
                        .clockName = kCLOCK_PortA,
                        .portNumber = PORTA_NUM};
gpioHandleKSDK_t D11 = {.base = GPIOA,
                        .portBase = PORTA,
                        .pinNumber = 16,
                        .mask = 1 << (16),
                        .irq = PORTA_IRQn,
                        .clockName = kCLOCK_PortA,
                        .portNumber = PORTA_NUM};

// UART0 Handle
gpioHandleKSDK_t D0 = {.base = GPIOC,
                       .portBase = PORTC,
                       .pinNumber = 6,
                       .mask = 1 << (6),
                       .irq = PORTB_PORTC_IRQn,
                       .clockName = kCLOCK_PortC,
                       .portNumber = PORTC_NUM};
gpioHandleKSDK_t D1 = {.base = GPIOC,
                       .portBase = PORTC,
                       .pinNumber = 7,
                       .mask = 1 << (7),
                       .irq = PORTB_PORTC_IRQn,
                       .clockName = kCLOCK_PortC,
                       .portNumber = PORTC_NUM};

// FRDM-KW41Z Arduino Connector Pin Defintion
gpioHandleKSDK_t A0; // No GPIO Port associated.
gpioHandleKSDK_t A1 = {.base = GPIOB,
                       .portBase = PORTB,
                       .pinNumber = 18,
                       .mask = 1 << (18),
                       .irq = PORTB_PORTC_IRQn,
                       .clockName = kCLOCK_PortB,
                       .portNumber = PORTB_NUM};
gpioHandleKSDK_t A2 = {.base = GPIOB,
                       .portBase = PORTB,
                       .pinNumber = 2,
                       .mask = 1 << (2),
                       .irq = PORTB_PORTC_IRQn,
                       .clockName = kCLOCK_PortB,
                       .portNumber = PORTB_NUM};
gpioHandleKSDK_t A3 = {.base = GPIOB,
                       .portBase = PORTB,
                       .pinNumber = 3,
                       .mask = 1 << (3),
                       .irq = PORTB_PORTC_IRQn,
                       .clockName = kCLOCK_PortB,
                       .portNumber = PORTB_NUM};
gpioHandleKSDK_t D2 = {.base = GPIOC,
                       .portBase = PORTC,
                       .pinNumber = 19,
                       .mask = 1 << (19),
                       .irq = PORTB_PORTC_IRQn,
                       .clockName = kCLOCK_PortC,
                       .portNumber = PORTC_NUM};
gpioHandleKSDK_t D3 = {.base = GPIOC,
                       .portBase = PORTC,
                       .pinNumber = 16,
                       .mask = 1 << (16),
                       .irq = PORTB_PORTC_IRQn,
                       .clockName = kCLOCK_PortC,
                       .portNumber = PORTC_NUM};
gpioHandleKSDK_t D4 = {.base = GPIOC,
                       .portBase = PORTC,
                       .pinNumber = 4,
                       .mask = 1 << (4),
                       .irq = PORTB_PORTC_IRQn,
                       .clockName = kCLOCK_PortC,
                       .portNumber = PORTC_NUM};
gpioHandleKSDK_t D5 = {.base = GPIOC,
                       .portBase = PORTC,
                       .pinNumber = 17,
                       .mask = 1 << (17),
                       .irq = PORTB_PORTC_IRQn,
                       .clockName = kCLOCK_PortC,
                       .portNumber = PORTC_NUM};
gpioHandleKSDK_t D6 = {.base = GPIOC,
                       .portBase = PORTC,
                       .pinNumber = 18,
                       .mask = 1 << (18),
                       .irq = PORTB_PORTC_IRQn,
                       .clockName = kCLOCK_PortC,
                       .portNumber = PORTC_NUM};
gpioHandleKSDK_t D7 = {.base = GPIOA,
                       .portBase = PORTA,
                       .pinNumber = 1,
                       .mask = 1 << (1),
                       .irq = PORTA_IRQn,
                       .clockName = kCLOCK_PortA,
                       .portNumber = PORTA_NUM};
gpioHandleKSDK_t D8 = {.base = GPIOA,
                       .portBase = PORTA,
                       .pinNumber = 0,
                       .mask = 1 << (0),
                       .irq = PORTA_IRQn,
                       .clockName = kCLOCK_PortA,
                       .portNumber = PORTA_NUM};
gpioHandleKSDK_t D9 = {.base = GPIOC,
                       .portBase = PORTC,
                       .pinNumber = 1,
                       .mask = 1 << (1),
                       .irq = PORTB_PORTC_IRQn,
                       .clockName = kCLOCK_PortC,
                       .portNumber = PORTC_NUM};
gpioHandleKSDK_t D10 = {.base = GPIOA,
                        .portBase = PORTA,
                        .pinNumber = 19,
                        .mask = 1 << (19),
                        .irq = PORTA_IRQn,
                        .clockName = kCLOCK_PortA,
                        .portNumber = PORTA_NUM};

// FRDM-KW41Z Internal Peripheral Pin Definitions
gpioHandleKSDK_t RED_LED = {.base = GPIOC,
                            .portBase = PORTC,
                            .pinNumber = 1,
                            .mask = 1 << (1),
                            .irq = PORTB_PORTC_IRQn,
                            .clockName = kCLOCK_PortC,
                            .portNumber = PORTC_NUM};
gpioHandleKSDK_t GREEN_LED = {.base = GPIOA,
                              .portBase = PORTA,
                              .pinNumber = 19,
                              .mask = 1 << (19),
                              .irq = PORTA_IRQn,
                              .clockName = kCLOCK_PortA,
                              .portNumber = PORTA_NUM};
gpioHandleKSDK_t BLUE_LED = {.base = GPIOA,
                             .portBase = PORTA,
                             .pinNumber = 18,
                             .mask = 1 << (18),
                             .irq = PORTA_IRQn,
                             .clockName = kCLOCK_PortA,
                             .portNumber = PORTA_NUM};

/*! @brief       Determines the Clock Frequency feature.
 *  @details     The Clock Frequecny computation API required by fsl_uart_cmsis.c.
 *  @param[in]   void
 *  @Constraints None
 *  @Reentrant   Yes
 *  @return      uint32_t Returns the clock frequency .
 */
uint32_t LPUART0_GetFreq(void)
{
    CLOCK_SetLpuartClock(1U);
    return CLOCK_GetFreq(kCLOCK_CoreSysClk);
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
    return CLOCK_GetFreq(I2C0_CLK_SRC);
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
    return CLOCK_GetFreq(I2C1_CLK_SRC);
}

/*! @brief       Determines the Clock Frequency feature.
 *  @details     The Clock Frequecny computation API required by fsl_spi_cmsis.c.
 *  @param[in]   void
 *  @Constraints None
 *  @Reentrant   Yes
 *  @return      uint32_t Returns the clock frequency .
 */
uint32_t DSPI1_GetFreq(void)
{
    return CLOCK_GetBusClkFreq();
}

static void i2c_release_bus_delay(void)
{
    uint32_t i = 0;
    for (i = 0; i < 100U; i++)
    {
        __NOP();
    }
}

/*! @brief       Release the onboard FXOS8700 Bus.
 *  @details     The API to Release the onboard FXOS8700 Bus to enable I2C communication.
 *  @param[in]   void
 *  @Constraints None
 *  @Reentrant   Yes
 *  @return      void
 */
void BOARD_ACCEL_ReleaseBus(void)
{
    uint8_t i = 0;
    gpio_pin_config_t pin_config;
    port_pin_config_t i2c_pin_config = {0};

    /* Config pin mux as gpio */
    i2c_pin_config.pullSelect = kPORT_PullUp;
    i2c_pin_config.mux = kPORT_MuxAsGpio;

    pin_config.pinDirection = kGPIO_DigitalOutput;
    pin_config.outputLogic = 1U;
    CLOCK_EnableClock(kCLOCK_PortC);
    PORT_SetPinConfig(PORTC, 2U, &i2c_pin_config);
    PORT_SetPinConfig(PORTC, 3U, &i2c_pin_config);

    GPIO_PinInit(GPIOC, 2U, &pin_config);
    GPIO_PinInit(GPIOC, 3U, &pin_config);

    /* Drive SDA low first to simulate a start */
    GPIO_WritePinOutput(GPIOC, 3U, 0U);
    i2c_release_bus_delay();

    /* Send 9 pulses on SCL and keep SDA low */
    for (i = 0; i < 9; i++)
    {
        GPIO_WritePinOutput(GPIOC, 2U, 0U);
        i2c_release_bus_delay();

        GPIO_WritePinOutput(GPIOC, 3U, 1U);
        i2c_release_bus_delay();

        GPIO_WritePinOutput(GPIOC, 2U, 1U);
        i2c_release_bus_delay();
        i2c_release_bus_delay();
    }

    /* Send stop */
    GPIO_WritePinOutput(GPIOC, 2U, 0U);
    i2c_release_bus_delay();

    GPIO_WritePinOutput(GPIOC, 3U, 0U);
    i2c_release_bus_delay();

    GPIO_WritePinOutput(GPIOC, 2U, 1U);
    i2c_release_bus_delay();

    GPIO_WritePinOutput(GPIOC, 3U, 1U);
    i2c_release_bus_delay();
}