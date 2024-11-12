/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_port.h"
#include "pin_mux.h"

/*******************************************************************************
 * Code
 ******************************************************************************/
void BOARD_InitPins(void)
{
    /* Declare and initialise for pull up configuration */
    port_pin_config_t pinConfig = {0};
    pinConfig.pullSelect = kPORT_PullUp;
#if defined(FSL_FEATURE_PORT_HAS_OPEN_DRAIN) && FSL_FEATURE_PORT_HAS_OPEN_DRAIN
    pinConfig.openDrainEnable = kPORT_OpenDrainEnable;
#endif /* FSL_FEATURE_PORT_HAS_OPEN_DRAIN */

    /* Initialize UART1 pins below */
    /* Ungate the port clock */
    CLOCK_EnableClock(kCLOCK_PortB);
    /* Affects PORTB_PCR16 register */
    PORT_SetPinMux(PORTB, 16U, kPORT_MuxAlt3);
    /* Affects PORTB_PCR17 register */
    PORT_SetPinMux(PORTB, 17U, kPORT_MuxAlt3);

    /* Ungate the port clock */
    CLOCK_EnableClock(kCLOCK_PortE);
    /* I2C0 pull up resistor setting */
    PORT_SetPinConfig(PORTE, 24U, &pinConfig);
    PORT_SetPinConfig(PORTE, 25U, &pinConfig);
    /* I2C0 PIN_MUX Configuration */
    PORT_SetPinMux(PORTE, 24U, kPORT_MuxAlt5);
    PORT_SetPinMux(PORTE, 25U, kPORT_MuxAlt5);

    /* Ungate the port clock */
    CLOCK_EnableClock(kCLOCK_PortC);
    /* I2C1 pull up resistor setting */
    PORT_SetPinConfig(PORTC, 10U, &pinConfig);
    PORT_SetPinConfig(PORTC, 11U, &pinConfig);
    /* I2C1 PIN_MUX Configuration */
    PORT_SetPinMux(PORTC, 10U, kPORT_MuxAlt2);
    PORT_SetPinMux(PORTC, 11U, kPORT_MuxAlt2);

    /* Led pin mux Configuration */
    PORT_SetPinMux(PORTB, 21U, kPORT_MuxAsGpio); // BLUE
    PORT_SetPinMux(PORTB, 22U, kPORT_MuxAsGpio); // RED
    PORT_SetPinMux(PORTE, 26U, kPORT_MuxAsGpio); // GREEN
}
