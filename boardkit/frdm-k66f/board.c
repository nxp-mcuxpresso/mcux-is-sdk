/*
* Copyright (c) 2015, Freescale Semiconductor, Inc.
* All rights reserved.
*
*
* SPDX-License-Identifier: BSD-3-Clause
*/

#include <stdint.h>
#include "fsl_device_registers.h"
#include "board.h"
#include "fsl_debug_console.h"

/*******************************************************************************
 * Variables
 ******************************************************************************/

/*******************************************************************************
 * Code
 ******************************************************************************/

/* Initialize debug console. */
void BOARD_InitDebugConsole(void)
{
    uint32_t uartClkSrcFreq = BOARD_DEBUG_UART_CLK_FREQ;
    DbgConsole_Init(BOARD_DEBUG_UART_BASEADDR, BOARD_DEBUG_UART_BAUDRATE, BOARD_DEBUG_UART_TYPE, uartClkSrcFreq);
}
