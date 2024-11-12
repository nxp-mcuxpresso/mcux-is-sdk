/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "board.h"
#include "fsl_debug_console.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/
/* Initialize debug console. */
status_t BOARD_InitDebugConsole(void)
{
    status_t result;

    /* update Flexcomm fractional divider to correct the baud rate */
    CLOCK_SetFRGClock(BOARD_DEBUG_UART_BASEADDR == (uint32_t)USART0 ? kCLOCK_DivFrg0 : kCLOCK_DivFrg1,
                      FLEXCOMM_CLK(BOARD_DEBUG_UART_CLK_FREQ, BOARD_DEBUG_UART_BAUDRATE));

    result = DbgConsole_Init(BOARD_DEBUG_UART_BASEADDR, BOARD_DEBUG_UART_BAUDRATE, DEBUG_CONSOLE_DEVICE_TYPE_FLEXCOMM,
                             BOARD_DEBUG_UART_CLK_FREQ);
    assert(kStatus_Success == result);
    return result;
}
