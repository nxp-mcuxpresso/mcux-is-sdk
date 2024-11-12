/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef _CLOCK_CONFIG_H_
#define _CLOCK_CONFIG_H_

/*******************************************************************************
 * DEFINITION
 ******************************************************************************/
#define BOARD_XTAL0_CLK_HZ 12000000U
#define BOARD_XTAL32K_CLK_HZ 32768U

/*******************************************************************************
 * API
 ******************************************************************************/
#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus*/

void BOARD_BootClockVLPR(void);
void BOARD_BootClockRUN(void);
void BOARD_BootClockHSRUN(void);

#if defined(__cplusplus)
}
#endif /* __cplusplus*/

#endif /* _CLOCK_CONFIG_H_ */
