/*
 * Copyright (c) 2015 - 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*! \file driver_ctimer.c
    \brief Provides a simple abstraction for a periodic interval timer.

   Bare metal implementations of the sensor fusion library require at least
   one periodic interrupt for use as a timebase for sensor fusion functions.
   The CTIMER module on the LPC is one such module.  The timer functions are
   only referenced at the main() level.  There is no interaction within the
   fusion routines themselves.
*/
#include "issdk_hal.h"
#include "board.h"
#include "fsl_ctimer.h"
#include "pin_mux.h"
#include "clock_config.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define CTIMER CTIMER0                 /* Timer 0 */
#define CTIMER_MAT0_OUT kCTIMER_Match_0 /* Match output 0 */
#define BUS_CLK_FREQ CLOCK_GetFreq(kCLOCK_BusClk)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
void ctimer_callback(uint32_t flags);

/* Array of function pointers for callback for each channel */
ctimer_callback_t ctimer_callback_table[] =
{
    ctimer_callback,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL,
    NULL
};

volatile bool   pitIsrFlag = false;

void ctimer_callback(uint32_t flags)
{
    CTIMER_ClearStatusFlags(CTIMER, kCTIMER_Match0Flag);
    pitIsrFlag = true;
    GPIO_TogglePinsOutput(GPIO, 0U, 1U << 30);  // Arduino Pin A0 for LPC54114, for freq debug purpose only
}

/*******************************************************************************
* Variables
******************************************************************************/
/* Match Configuration for Channel 0 */
static ctimer_match_config_t matchConfig0;

void pit_init(uint32_t microseconds)
{
    long busFreq = BUS_CLK_FREQ;
    ctimer_config_t config;

    long fusionHz = 1000000/microseconds;
    // Program Arduino A0 pin as output
    GPIO_PinInit(GPIO, 0U, 30, &(gpio_pin_config_t){kGPIO_DigitalOutput, 1U}); // Arduino Pin A0 for LPC54114

    CTIMER_GetDefaultConfig(&config);

    CTIMER_Init(CTIMER, &config);

    /* Configuration 0 */
    matchConfig0.enableCounterReset = true;
    matchConfig0.enableCounterStop = false;
    matchConfig0.matchValue = busFreq / fusionHz;
    matchConfig0.outControl = kCTIMER_Output_Toggle;
    matchConfig0.outPinInitState = false;
    matchConfig0.enableInterrupt = true;

    /* initialize CTIMER */
    CTIMER_RegisterCallBack(CTIMER, &ctimer_callback_table[0], kCTIMER_MultipleCallback);
    CTIMER_SetupMatch(CTIMER, CTIMER_MAT0_OUT, &matchConfig0);
    CTIMER_StartTimer(CTIMER);
}
