/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/* Local Includes */
#include "issdk_sim.h"

/*******************************************************************************
 * Code
 ******************************************************************************/
int main(void)
{
    ISSDK_Sim_HW_Init();

    for (;;)
    { /* Do user space processing and then enter to Wait Mode */
        ISSDK_Sim_ProcessAndWait();
    }
}
