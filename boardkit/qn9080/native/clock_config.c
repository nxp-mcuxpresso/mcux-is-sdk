/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*
 * How to set up clock using clock driver functions:
 *
 * 1. Setup clock sources.
 *
 * 2. Set up all selectors to provide selected clocks.
 *
 * 3. Set up all dividers.
 *
 * 4. Set up clock out source and pins.
 *
 * 5. Set up the specified peripheral clock.
 */

/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!GlobalInfo
product: Clocks v3.0
processor: QN908XC
package_id: QN9080C
mcu_data: ksdk2_0
processor_version: 0.0.0
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/

#include "fsl_power.h"
#include "fsl_common.h"
#include "clock_config.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
/* System clock frequency. */
extern uint32_t SystemCoreClock;

/*******************************************************************************
 ********************* Configuration BOARD_BootClockLSRUN **********************
 ******************************************************************************/
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!Configuration
name: BOARD_BootClockLSRUN
outputs:
- {id: AHB_clock.outFreq, value: 8 MHz}
- {id: APB_clock.outFreq, value: 8 MHz}
- {id: RTC_SLETimer_clock.outFreq, value: 32.768 kHz}
- {id: RTC_clock.outFreq, value: 8 MHz}
- {id: WDT_clock.outFreq, value: 8 MHz}
settings:
- {id: SYSCON.AHB_DIV.scale, value: '2', locked: true}
- {id: SYSCON.AUTO_M_MULT.scale, value: '6'}
- {id: SYSCON.BLE_DIV.scale, value: '1'}
- {id: SYSCON.CLK_32K_SEL.sel, value: SYSCON.XTAL32K}
- {id: SYSCON.CLK_SYS_SEL.sel, value: SYSCON.XTAL_DIV}
- {id: SYSCON.XTAL_DIV.scale, value: '2', locked: true}
sources:
- {id: SYSCON.XTAL.outFreq, value: 32 MHz, enabled: true}
- {id: SYSCON.XTAL32K.outFreq, value: 32.768 kHz, enabled: true}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/

/*******************************************************************************
 * Variables for BOARD_BootClockLSRUN configuration
 ******************************************************************************/
/*******************************************************************************
 * Code for BOARD_BootClockLSRUN configuration
 ******************************************************************************/
void BOARD_BootClockLSRUN(void)
{
    /* Power up/Power down the module. */

    /* Set up clock selectors - Attach clocks to the peripheries */
    CLOCK_AttachClk(k32M_to_XTAL_CLK);                  /* Switch XTAL_CLK to 32M */
    CLOCK_AttachClk(kXTAL32K_to_32K_CLK);                  /* Switch 32K_CLK to XTAL32K */
    CLOCK_AttachClk(kXTAL_to_SYS_CLK);                  /* Switch SYS_CLK to XTAL */
    CLOCK_AttachClk(kAPB_to_WDT_CLK);                  /* Switch WDT_CLK to APB */

    /* Set up dividers */
    CLOCK_SetClkDiv(kCLOCK_DivOsc32mClk, 1U);                  /* Set OSC32M_DIV divider to value 2 */
    CLOCK_SetClkDiv(kCLOCK_DivXtalClk, 1U);                  /* Set XTAL_DIV divider to value 2 */
    CLOCK_SetClkDiv(kCLOCK_DivAhbClk, 1U);                  /* Set AHB_DIV divider to value 2 */
    CLOCK_SetClkDiv(kCLOCK_DivFrg1, 0U);                  /* Set FRG_MULT1 to value 0, Set FRG_DIV1 to value 255 */
    CLOCK_SetClkDiv(kCLOCK_DivFrg0, 0U);                  /* Set FRG_MULT0 to value 0, Set FRG_DIV0 to value 255 */
    CLOCK_SetClkDiv(kCLOCK_DivApbClk, 0U);                  /* Set APB_DIV divider to value 1 */

    /* Enable/Disable clock out source and pins.*/

    /* Enable/Disable the specified peripheral clock.*/

}

/*******************************************************************************
 ********************** Configuration BOARD_BootClockRUN ***********************
 ******************************************************************************/
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!Configuration
name: BOARD_BootClockRUN
called_from_default_init: true
outputs:
- {id: AHB_clock.outFreq, value: 16 MHz}
- {id: APB_clock.outFreq, value: 16 MHz}
- {id: RTC_SLETimer_clock.outFreq, value: 32.768 kHz}
- {id: RTC_clock.outFreq, value: 16 MHz}
- {id: WDT_clock.outFreq, value: 16 MHz}
settings:
- {id: SYSCON.AUTO_M_MULT.scale, value: '6'}
- {id: SYSCON.CLK_32K_SEL.sel, value: SYSCON.XTAL32K}
- {id: SYSCON.CLK_SYS_SEL.sel, value: SYSCON.XTAL_DIV}
- {id: SYSCON.XTAL_DIV.scale, value: '2', locked: true}
sources:
- {id: SYSCON.XTAL.outFreq, value: 32 MHz, enabled: true}
- {id: SYSCON.XTAL32K.outFreq, value: 32.768 kHz, enabled: true}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/

/*******************************************************************************
 * Variables for BOARD_BootClockRUN configuration
 ******************************************************************************/
/*******************************************************************************
 * Code for BOARD_BootClockRUN configuration
 ******************************************************************************/
void BOARD_BootClockRUN(void)
{
    /* Power up/Power down the module. */

    /* Set up clock selectors - Attach clocks to the peripheries */
    CLOCK_AttachClk(k32M_to_XTAL_CLK);                  /* Switch XTAL_CLK to 32M */
    CLOCK_AttachClk(kXTAL32K_to_32K_CLK);                  /* Switch 32K_CLK to XTAL32K */
    CLOCK_AttachClk(kXTAL_to_SYS_CLK);                  /* Switch SYS_CLK to XTAL */
    CLOCK_AttachClk(kAPB_to_WDT_CLK);                  /* Switch WDT_CLK to APB */

    /* Set up dividers */
    CLOCK_SetClkDiv(kCLOCK_DivOsc32mClk, 1U);                  /* Set OSC32M_DIV divider to value 2 */
    CLOCK_SetClkDiv(kCLOCK_DivXtalClk, 1U);                  /* Set XTAL_DIV divider to value 2 */
    CLOCK_SetClkDiv(kCLOCK_DivAhbClk, 0U);                  /* Set AHB_DIV divider to value 1 */
    CLOCK_SetClkDiv(kCLOCK_DivFrg1, 0U);                  /* Set FRG_MULT1 to value 0, Set FRG_DIV1 to value 255 */
    CLOCK_SetClkDiv(kCLOCK_DivFrg0, 0U);                  /* Set FRG_MULT0 to value 0, Set FRG_DIV0 to value 255 */
    CLOCK_SetClkDiv(kCLOCK_DivApbClk, 0U);                  /* Set APB_DIV divider to value 1 */

    /* Enable/Disable clock out source and pins.*/

    /* Enable/Disable the specified peripheral clock.*/

}

/*******************************************************************************
 ********************* Configuration BOARD_BootClockHSRUN **********************
 ******************************************************************************/
/* TEXT BELOW IS USED AS SETTING FOR TOOLS *************************************
!!Configuration
name: BOARD_BootClockHSRUN
outputs:
- {id: AHB_clock.outFreq, value: 32 MHz}
- {id: APB_clock.outFreq, value: 32 MHz}
- {id: RTC_SLETimer_clock.outFreq, value: 32.768 kHz}
- {id: RTC_clock.outFreq, value: 32 MHz}
- {id: WDT_clock.outFreq, value: 32 MHz}
settings:
- {id: SYSCON.BLE_DIV.scale, value: '4'}
- {id: SYSCON.CLK_32K_SEL.sel, value: SYSCON.XTAL32K}
- {id: SYSCON.CLK_SYS_SEL.sel, value: SYSCON.XTAL_DIV}
sources:
- {id: SYSCON.XTAL.outFreq, value: 32 MHz, enabled: true}
- {id: SYSCON.XTAL32K.outFreq, value: 32.768 kHz, enabled: true}
 * BE CAREFUL MODIFYING THIS COMMENT - IT IS YAML SETTINGS FOR TOOLS **********/

/*******************************************************************************
 * Variables for BOARD_BootClockHSRUN configuration
 ******************************************************************************/
/*******************************************************************************
 * Code for BOARD_BootClockHSRUN configuration
 ******************************************************************************/
void BOARD_BootClockHSRUN(void)
{
    /* Power up/Power down the module. */

    /* Set up clock selectors - Attach clocks to the peripheries */
    CLOCK_AttachClk(k32M_to_XTAL_CLK);                  /* Switch XTAL_CLK to 32M */
    CLOCK_AttachClk(kXTAL32K_to_32K_CLK);                  /* Switch 32K_CLK to XTAL32K */
    CLOCK_AttachClk(kXTAL_to_SYS_CLK);                  /* Switch SYS_CLK to XTAL */
    CLOCK_AttachClk(kAPB_to_WDT_CLK);                  /* Switch WDT_CLK to APB */

    /* Set up dividers */
    CLOCK_SetClkDiv(kCLOCK_DivOsc32mClk, 1U);                  /* Set OSC32M_DIV divider to value 2 */
    CLOCK_SetClkDiv(kCLOCK_DivXtalClk, 0U);                  /* Set XTAL_DIV divider to value 1 */
    CLOCK_SetClkDiv(kCLOCK_DivAhbClk, 0U);                  /* Set AHB_DIV divider to value 1 */
    CLOCK_SetClkDiv(kCLOCK_DivFrg1, 0U);                  /* Set FRG_MULT1 to value 0, Set FRG_DIV1 to value 255 */
    CLOCK_SetClkDiv(kCLOCK_DivFrg0, 0U);                  /* Set FRG_MULT0 to value 0, Set FRG_DIV0 to value 255 */
    CLOCK_SetClkDiv(kCLOCK_DivApbClk, 0U);                  /* Set APB_DIV divider to value 1 */

    /* Enable/Disable clock out source and pins.*/

    /* Enable/Disable the specified peripheral clock.*/

}

