/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef _CLOCK_CONFIG_H_
#define _CLOCK_CONFIG_H_

/*******************************************************************************
 * DEFINITION
 ******************************************************************************/
#define BOARD_XTAL0_CLK_HZ 50000000U
#define BOARD_XTAL32K_CLK_HZ 32768U

/*******************************************************************************
 * API
 ******************************************************************************/
void BOARD_BootClockVLPR(void);
void BOARD_BootClockRUN(void);

#endif /* _CLOCK_CONFIG_H_ */
