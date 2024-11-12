/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "fsl_common.h"
#include "fsl_smc.h"
#include "clock_config.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*! @brief Clock configuration structure. */
typedef struct _clock_config
{
    mcg_config_t mcgConfig;       /*!< MCG configuration.      */
    sim_clock_config_t simConfig; /*!< SIM configuration.      */
    osc_config_t oscConfig;       /*!< OSC configuration.      */
    uint32_t coreClock;           /*!< core clock frequency.   */
} clock_config_t;

/*******************************************************************************
 * Variables
 ******************************************************************************/
/* System clock frequency. */
extern uint32_t SystemCoreClock;

/* Configuration for enter VLPR mode. Core clock = 4MHz. */
const clock_config_t g_defaultClockConfigVlpr = {
    .mcgConfig =
        {
            .mcgMode = kMCG_ModeBLPI,            /* Work in BLPI mode. */
            .irclkEnableMode = kMCG_IrclkEnable, /* MCGIRCLK enable. */
            .ircs = kMCG_IrcFast,                /* Select IRC4M. */
            .fcrdiv = 0U,                        /* FCRDIV is 0. */

            .frdiv = 0U,
            .drs = kMCG_DrsLow,         /* Low frequency range. */
            .dmx32 = kMCG_Dmx32Default, /* DCO has a default range of 25%. */
            .oscsel = kMCG_OscselOsc,   /* Select OSC. */

            .pll0Config =
                {
                    .enableMode = 0U, /* Don't eanble PLL. */
                    .prdiv = 0U,
                    .vdiv = 0U,
                },
        },
    .simConfig =
        {
            .pllFllSel = 3U,        /* PLLFLLSEL select IRC48MCLK. */
            .er32kSrc = 2U,         /* ERCLK32K selection, use RTC. */
            .clkdiv1 = 0x00040000U, /* SIM_CLKDIV1. */
        },
    .oscConfig = {.freq = BOARD_XTAL0_CLK_HZ,
                  .capLoad = 0,
                  .workMode = kOSC_ModeExt,
                  .oscerConfig =
                      {
                          .enableMode = kOSC_ErClkEnable,
#if (defined(FSL_FEATURE_OSC_HAS_EXT_REF_CLOCK_DIVIDER) && FSL_FEATURE_OSC_HAS_EXT_REF_CLOCK_DIVIDER)
                          .erclkDiv = 0U,
#endif
                      }},
    .coreClock = 4000000U, /* Core clock frequency */
};

/* Configuration for enter RUN mode. Core clock = 120MHz. */
const clock_config_t g_defaultClockConfigRun = {
    .mcgConfig =
        {
            .mcgMode = kMCG_ModePEE,             /* Work in PEE mode. */
            .irclkEnableMode = kMCG_IrclkEnable, /* MCGIRCLK enable. */
            .ircs = kMCG_IrcSlow,                /* Select IRC32k. */
            .fcrdiv = 0U,                        /* FCRDIV is 0. */

            .frdiv = 7U,
            .drs = kMCG_DrsLow,         /* Low frequency range. */
            .dmx32 = kMCG_Dmx32Default, /* DCO has a default range of 25%. */
            .oscsel = kMCG_OscselOsc,   /* Select OSC. */

            .pll0Config =
                {
                    .enableMode = 0U, .prdiv = 0x13U, .vdiv = 0x18U,
                },
        },
    .simConfig =
        {
            .pllFllSel = 1U,        /* PLLFLLSEL select PLL. */
            .er32kSrc = 2U,         /* ERCLK32K selection, use RTC. */
            .clkdiv1 = 0x01140000U, /* SIM_CLKDIV1. */
        },
    .oscConfig = {.freq = BOARD_XTAL0_CLK_HZ,
                  .capLoad = 0,
                  .workMode = kOSC_ModeExt,
                  .oscerConfig =
                      {
                          .enableMode = kOSC_ErClkEnable,
#if (defined(FSL_FEATURE_OSC_HAS_EXT_REF_CLOCK_DIVIDER) && FSL_FEATURE_OSC_HAS_EXT_REF_CLOCK_DIVIDER)
                          .erclkDiv = 0U,
#endif
                      }},
    .coreClock = 120000000U, /* Core clock frequency */
};

/*******************************************************************************
 * Code
 ******************************************************************************/
/*
 * How to setup clock using clock driver functions:
 *
 * 1. CLOCK_SetSimSafeDivs, to make sure core clock, bus clock, flexbus clock
 *    and flash clock are in allowed range during clock mode switch.
 *
 * 2. Call CLOCK_Osc0Init to setup OSC clock, if it is used in target mode.
 *
 * 3. Set MCG configuration, MCG includes three parts: FLL clock, PLL clock and
 *    internal reference clock(MCGIRCLK). Follow the steps to setup:
 *
 *    1). Call CLOCK_BootToXxxMode to set MCG to target mode.
 *
 *    2). If target mode is FBI/BLPI/PBI mode, the MCGIRCLK has been configured
 *        correctly. For other modes, need to call CLOCK_SetInternalRefClkConfig
 *        explicitly to setup MCGIRCLK.
 *
 *    3). Don't need to configure FLL explicitly, because if target mode is FLL
 *        mode, then FLL has been configured by the function CLOCK_BootToXxxMode,
 *        if the target mode is not FLL mode, the FLL is disabled.
 *
 *    4). If target mode is PEE/PBE/PEI/PBI mode, then the related PLL has been
 *        setup by CLOCK_BootToXxxMode. In FBE/FBI/FEE/FBE mode, the PLL could
 *        be enabled independently, call CLOCK_EnablePll0 explicitly in this case.
 *
 * 4. Call CLOCK_SetSimConfig to set the clock configuration in SIM.
 */

void BOARD_BootClockVLPR(void)
{
    CLOCK_SetSimSafeDivs();

    CLOCK_BootToBlpiMode(g_defaultClockConfigVlpr.mcgConfig.fcrdiv, g_defaultClockConfigVlpr.mcgConfig.ircs,
                         g_defaultClockConfigVlpr.mcgConfig.irclkEnableMode);

    CLOCK_SetSimConfig(&g_defaultClockConfigVlpr.simConfig);

    SystemCoreClock = g_defaultClockConfigVlpr.coreClock;

    SMC_SetPowerModeProtection(SMC, kSMC_AllowPowerModeAll);
    SMC_SetPowerModeVlpr(SMC, false);
    while (SMC_GetPowerModeState(SMC) != kSMC_PowerStateVlpr)
    {
    }
}

void BOARD_BootClockRUN(void)
{
    CLOCK_SetSimSafeDivs();

    CLOCK_InitOsc0(&g_defaultClockConfigRun.oscConfig);
    CLOCK_SetXtal0Freq(BOARD_XTAL0_CLK_HZ);

    CLOCK_BootToPeeMode(g_defaultClockConfigRun.mcgConfig.oscsel, kMCG_PllClkSelPll0,
                        &g_defaultClockConfigRun.mcgConfig.pll0Config);

    CLOCK_SetInternalRefClkConfig(g_defaultClockConfigRun.mcgConfig.irclkEnableMode,
                                  g_defaultClockConfigRun.mcgConfig.ircs, g_defaultClockConfigRun.mcgConfig.fcrdiv);

    CLOCK_SetSimConfig(&g_defaultClockConfigRun.simConfig);

    SystemCoreClock = g_defaultClockConfigRun.coreClock;
}
