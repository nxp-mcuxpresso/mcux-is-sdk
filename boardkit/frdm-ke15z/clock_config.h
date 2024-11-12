/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _CLOCK_CONFIG_H_
#define _CLOCK_CONFIG_H_

#include "fsl_common.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define BOARD_XTAL0_CLK_HZ                          8000000U  /*!< Board xtal0 frequency in Hz */

/*******************************************************************************
 ********************** Configuration BOARD_BootClockRUN ***********************
 ******************************************************************************/
/*******************************************************************************
 * Definitions for BOARD_BootClockRUN configuration
 ******************************************************************************/
#define BOARD_BOOTCLOCKRUN_CORE_CLOCK              72000000U  /*!< Core clock frequency: 72000000Hz */

/*! @brief SCG set for BOARD_BootClockRUN configuration.
 */
extern const scg_sys_clk_config_t g_sysClkConfig_BOARD_BootClockRUN;
/*! @brief System OSC set for BOARD_BootClockRUN configuration.
 */
extern const scg_sosc_config_t g_scgSysOscConfig_BOARD_BootClockRUN;
/*! @brief SIRC set for BOARD_BootClockRUN configuration.
 */
extern const scg_sirc_config_t g_scgSircConfig_BOARD_BootClockRUN;
/*! @brief FIRC set for BOARD_BootClockRUN configuration.
 */
extern const scg_firc_config_t g_scgFircConfigBOARD_BootClockRUN;
/*! @brief Low Power FLL set for BOARD_BootClockRUN configuration.
 */
extern const scg_lpfll_config_t g_scgLpFllConfigBOARD_BootClockRUN;

/*******************************************************************************
 * API for BOARD_BootClockRUN configuration
 ******************************************************************************/
#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus*/

/*!
 * @brief This function executes configuration of clocks.
 *
 */
void BOARD_BootClockRUN(void);

#if defined(__cplusplus)
}
#endif /* __cplusplus*/

/*******************************************************************************
 ********************* Configuration BOARD_BootClockVLPR ***********************
 ******************************************************************************/
/*******************************************************************************
 * Definitions for BOARD_BootClockVLPR configuration
 ******************************************************************************/
#define BOARD_BOOTCLOCKVLPR_CORE_CLOCK              4000000U  /*!< Core clock frequency: 4000000Hz */

/*! @brief SCG set for BOARD_BootClockVLPR configuration.
 */
extern const scg_sys_clk_config_t g_sysClkConfig_BOARD_BootClockVLPR;
/*! @brief System OSC set for BOARD_BootClockVLPR configuration.
 */
extern const scg_sosc_config_t g_scgSysOscConfig_BOARD_BootClockVLPR;
/*! @brief SIRC set for BOARD_BootClockVLPR configuration.
 */
extern const scg_sirc_config_t g_scgSircConfig_BOARD_BootClockVLPR;
/*! @brief FIRC set for BOARD_BootClockVLPR configuration.
 */
extern const scg_firc_config_t g_scgFircConfigBOARD_BootClockVLPR;
/*! @brief Low Power FLL set for BOARD_BootClockVLPR configuration.
 */
extern const scg_lpfll_config_t g_scgLpFllConfigBOARD_BootClockVLPR;

/*******************************************************************************
 * API for BOARD_BootClockVLPR configuration
 ******************************************************************************/
#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus*/

/*!
 * @brief This function executes configuration of clocks.
 *
 */
void BOARD_BootClockVLPR(void);

#if defined(__cplusplus)
}
#endif /* __cplusplus*/

#endif /* _CLOCK_CONFIG_H_ */

