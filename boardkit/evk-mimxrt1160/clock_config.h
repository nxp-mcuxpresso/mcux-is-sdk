/*
 * Copyright 2021 NXP
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

#define BOARD_XTAL0_CLK_HZ 24000000U /*!< Board xtal0 frequency in Hz */

#define BOARD_XTAL32K_CLK_HZ 32768U  /*!< Board xtal32k frequency in Hz */

/*******************************************************************************
 ************************ BOARD_InitBootClocks function ************************
 ******************************************************************************/

#if defined(__cplusplus)
extern "C" {
#endif /* __cplusplus*/

/*!
 * @brief This function executes default configuration of clocks.
 *
 */
void BOARD_InitBootClocks(void);

#if defined(__cplusplus)
}
#endif /* __cplusplus*/

/*******************************************************************************
 ********************** Configuration BOARD_BootClockRUN ***********************
 ******************************************************************************/
/*******************************************************************************
 * Definitions for BOARD_BootClockRUN configuration
 ******************************************************************************/
#if __CORTEX_M == 7
    #define BOARD_BOOTCLOCKRUN_CORE_CLOCK 996000000UL /*!< CM7 Core clock frequency: 996000000Hz */
#else
    #define BOARD_BOOTCLOCKRUN_CORE_CLOCK 392727272UL /*!< CM4 Core clock frequency: 392727272Hz */
#endif

/* Clock outputs (values are in Hz): */
#define BOARD_BOOTCLOCKRUN_ACMP_CLK_ROOT              24000000UL
#define BOARD_BOOTCLOCKRUN_ADC1_CLK_ROOT              24000000UL
#define BOARD_BOOTCLOCKRUN_ADC2_CLK_ROOT              24000000UL
#define BOARD_BOOTCLOCKRUN_ARM_PLL_CLK                996000000UL
#define BOARD_BOOTCLOCKRUN_ASRC_CLK_ROOT              24000000UL
#define BOARD_BOOTCLOCKRUN_AXI_CLK_ROOT               996000000UL
#define BOARD_BOOTCLOCKRUN_BUS_CLK_ROOT               240000000UL
#define BOARD_BOOTCLOCKRUN_BUS_LPSR_CLK_ROOT          160000000UL
#define BOARD_BOOTCLOCKRUN_CAN1_CLK_ROOT              24000000UL
#define BOARD_BOOTCLOCKRUN_CAN2_CLK_ROOT              24000000UL
#define BOARD_BOOTCLOCKRUN_CAN3_CLK_ROOT              24000000UL
#define BOARD_BOOTCLOCKRUN_CCM_CLKO1_CLK_ROOT         24000000UL
#define BOARD_BOOTCLOCKRUN_CCM_CLKO2_CLK_ROOT         24000000UL
#define BOARD_BOOTCLOCKRUN_CLK_1M                     1000000UL
#define BOARD_BOOTCLOCKRUN_CSI2_CLK_ROOT              24000000UL
#define BOARD_BOOTCLOCKRUN_CSI2_ESC_CLK_ROOT          24000000UL
#define BOARD_BOOTCLOCKRUN_CSI2_UI_CLK_ROOT           24000000UL
#define BOARD_BOOTCLOCKRUN_CSI_CLK_ROOT               24000000UL
#define BOARD_BOOTCLOCKRUN_CSSYS_CLK_ROOT             24000000UL
#define BOARD_BOOTCLOCKRUN_CSTRACE_CLK_ROOT           24000000UL
#define BOARD_BOOTCLOCKRUN_ELCDIF_CLK_ROOT            24000000UL
#define BOARD_BOOTCLOCKRUN_EMV1_CLK_ROOT              24000000UL
#define BOARD_BOOTCLOCKRUN_EMV2_CLK_ROOT              24000000UL
#define BOARD_BOOTCLOCKRUN_ENET1_CLK_ROOT             24000000UL
#define BOARD_BOOTCLOCKRUN_ENET2_CLK_ROOT             24000000UL
#define BOARD_BOOTCLOCKRUN_ENET_1G_TX_CLK             24000000UL
#define BOARD_BOOTCLOCKRUN_ENET_25M_CLK_ROOT          24000000UL
#define BOARD_BOOTCLOCKRUN_ENET_QOS_CLK_ROOT          24000000UL
#define BOARD_BOOTCLOCKRUN_ENET_TIMER1_CLK_ROOT       24000000UL
#define BOARD_BOOTCLOCKRUN_ENET_TIMER2_CLK_ROOT       24000000UL
#define BOARD_BOOTCLOCKRUN_ENET_TIMER3_CLK_ROOT       24000000UL
#define BOARD_BOOTCLOCKRUN_ENET_TX_CLK                24000000UL
#define BOARD_BOOTCLOCKRUN_FLEXIO1_CLK_ROOT           24000000UL
#define BOARD_BOOTCLOCKRUN_FLEXIO2_CLK_ROOT           24000000UL
#define BOARD_BOOTCLOCKRUN_FLEXSPI1_CLK_ROOT          24000000UL
#define BOARD_BOOTCLOCKRUN_FLEXSPI2_CLK_ROOT          24000000UL
#define BOARD_BOOTCLOCKRUN_GC355_CLK_ROOT             492000012UL
#define BOARD_BOOTCLOCKRUN_GPT1_CLK_ROOT              24000000UL
#define BOARD_BOOTCLOCKRUN_GPT1_IPG_CLK_HIGHFREQ      24000000UL
#define BOARD_BOOTCLOCKRUN_GPT2_CLK_ROOT              24000000UL
#define BOARD_BOOTCLOCKRUN_GPT2_IPG_CLK_HIGHFREQ      24000000UL
#define BOARD_BOOTCLOCKRUN_GPT3_CLK_ROOT              24000000UL
#define BOARD_BOOTCLOCKRUN_GPT3_IPG_CLK_HIGHFREQ      24000000UL
#define BOARD_BOOTCLOCKRUN_GPT4_CLK_ROOT              24000000UL
#define BOARD_BOOTCLOCKRUN_GPT4_IPG_CLK_HIGHFREQ      24000000UL
#define BOARD_BOOTCLOCKRUN_GPT5_CLK_ROOT              24000000UL
#define BOARD_BOOTCLOCKRUN_GPT5_IPG_CLK_HIGHFREQ      24000000UL
#define BOARD_BOOTCLOCKRUN_GPT6_CLK_ROOT              24000000UL
#define BOARD_BOOTCLOCKRUN_GPT6_IPG_CLK_HIGHFREQ      24000000UL
#define BOARD_BOOTCLOCKRUN_LCDIFV2_CLK_ROOT           24000000UL
#define BOARD_BOOTCLOCKRUN_LPI2C1_CLK_ROOT            24000000UL
#define BOARD_BOOTCLOCKRUN_LPI2C2_CLK_ROOT            24000000UL
#define BOARD_BOOTCLOCKRUN_LPI2C3_CLK_ROOT            24000000UL
#define BOARD_BOOTCLOCKRUN_LPI2C4_CLK_ROOT            24000000UL
#define BOARD_BOOTCLOCKRUN_LPI2C5_CLK_ROOT            24000000UL
#define BOARD_BOOTCLOCKRUN_LPI2C6_CLK_ROOT            24000000UL
#define BOARD_BOOTCLOCKRUN_LPSPI1_CLK_ROOT            24000000UL
#define BOARD_BOOTCLOCKRUN_LPSPI2_CLK_ROOT            24000000UL
#define BOARD_BOOTCLOCKRUN_LPSPI3_CLK_ROOT            24000000UL
#define BOARD_BOOTCLOCKRUN_LPSPI4_CLK_ROOT            24000000UL
#define BOARD_BOOTCLOCKRUN_LPSPI5_CLK_ROOT            24000000UL
#define BOARD_BOOTCLOCKRUN_LPSPI6_CLK_ROOT            24000000UL
#define BOARD_BOOTCLOCKRUN_LPUART10_CLK_ROOT          24000000UL
#define BOARD_BOOTCLOCKRUN_LPUART11_CLK_ROOT          24000000UL
#define BOARD_BOOTCLOCKRUN_LPUART12_CLK_ROOT          24000000UL
#define BOARD_BOOTCLOCKRUN_LPUART1_CLK_ROOT           24000000UL
#define BOARD_BOOTCLOCKRUN_LPUART2_CLK_ROOT           24000000UL
#define BOARD_BOOTCLOCKRUN_LPUART3_CLK_ROOT           24000000UL
#define BOARD_BOOTCLOCKRUN_LPUART4_CLK_ROOT           24000000UL
#define BOARD_BOOTCLOCKRUN_LPUART5_CLK_ROOT           24000000UL
#define BOARD_BOOTCLOCKRUN_LPUART6_CLK_ROOT           24000000UL
#define BOARD_BOOTCLOCKRUN_LPUART7_CLK_ROOT           24000000UL
#define BOARD_BOOTCLOCKRUN_LPUART8_CLK_ROOT           24000000UL
#define BOARD_BOOTCLOCKRUN_LPUART9_CLK_ROOT           24000000UL
#define BOARD_BOOTCLOCKRUN_M4_CLK_ROOT                392727272UL
#define BOARD_BOOTCLOCKRUN_M4_SYSTICK_CLK_ROOT        24000000UL
#define BOARD_BOOTCLOCKRUN_M7_CLK_ROOT                996000000UL
#define BOARD_BOOTCLOCKRUN_M7_SYSTICK_CLK_ROOT        100000UL
#define BOARD_BOOTCLOCKRUN_MIC_CLK_ROOT               24000000UL
#define BOARD_BOOTCLOCKRUN_MIPI_DSI_TX_CLK_ESC_ROOT   24000000UL
#define BOARD_BOOTCLOCKRUN_MIPI_ESC_CLK_ROOT          24000000UL
#define BOARD_BOOTCLOCKRUN_MIPI_REF_CLK_ROOT          24000000UL
#define BOARD_BOOTCLOCKRUN_MQS_CLK_ROOT               24000000UL
#define BOARD_BOOTCLOCKRUN_MQS_MCLK                   24000000UL
#define BOARD_BOOTCLOCKRUN_OSC_24M                    24000000UL
#define BOARD_BOOTCLOCKRUN_OSC_32K                    32768UL
#define BOARD_BOOTCLOCKRUN_OSC_RC_16M                 16000000UL
#define BOARD_BOOTCLOCKRUN_OSC_RC_400M                400000000UL
#define BOARD_BOOTCLOCKRUN_OSC_RC_48M                 48000000UL
#define BOARD_BOOTCLOCKRUN_OSC_RC_48M_DIV2            24000000UL
#define BOARD_BOOTCLOCKRUN_PLL_AUDIO_CLK              0UL
#define BOARD_BOOTCLOCKRUN_PLL_AUDIO_SS_MODULATION    0UL
#define BOARD_BOOTCLOCKRUN_PLL_AUDIO_SS_RANGE         0UL
#define BOARD_BOOTCLOCKRUN_PLL_VIDEO_CLK              984000025UL
#define BOARD_BOOTCLOCKRUN_PLL_VIDEO_SS_MODULATION    0UL
#define BOARD_BOOTCLOCKRUN_PLL_VIDEO_SS_RANGE         0UL
#define BOARD_BOOTCLOCKRUN_SAI1_CLK_ROOT              24000000UL
#define BOARD_BOOTCLOCKRUN_SAI1_MCLK1                 24000000UL
#define BOARD_BOOTCLOCKRUN_SAI1_MCLK2                 0UL
#define BOARD_BOOTCLOCKRUN_SAI1_MCLK3                 24000000UL
#define BOARD_BOOTCLOCKRUN_SAI2_CLK_ROOT              24000000UL
#define BOARD_BOOTCLOCKRUN_SAI2_MCLK1                 24000000UL
#define BOARD_BOOTCLOCKRUN_SAI2_MCLK2                 0UL
#define BOARD_BOOTCLOCKRUN_SAI2_MCLK3                 24000000UL
#define BOARD_BOOTCLOCKRUN_SAI3_CLK_ROOT              24000000UL
#define BOARD_BOOTCLOCKRUN_SAI3_MCLK1                 24000000UL
#define BOARD_BOOTCLOCKRUN_SAI3_MCLK2                 0UL
#define BOARD_BOOTCLOCKRUN_SAI3_MCLK3                 24000000UL
#define BOARD_BOOTCLOCKRUN_SAI4_CLK_ROOT              24000000UL
#define BOARD_BOOTCLOCKRUN_SAI4_MCLK1                 24000000UL
#define BOARD_BOOTCLOCKRUN_SAI4_MCLK2                 0UL
#define BOARD_BOOTCLOCKRUN_SEMC_CLK_ROOT              198000000UL
#define BOARD_BOOTCLOCKRUN_SPDIF_CLK_ROOT             24000000UL
#define BOARD_BOOTCLOCKRUN_SPDIF_EXTCLK_OUT           0UL
#define BOARD_BOOTCLOCKRUN_SYS_PLL1_CLK               0UL
#define BOARD_BOOTCLOCKRUN_SYS_PLL1_DIV2_CLK          0UL
#define BOARD_BOOTCLOCKRUN_SYS_PLL1_DIV5_CLK          0UL
#define BOARD_BOOTCLOCKRUN_SYS_PLL1_SS_MODULATION     0UL
#define BOARD_BOOTCLOCKRUN_SYS_PLL1_SS_RANGE          0UL
#define BOARD_BOOTCLOCKRUN_SYS_PLL2_CLK               528000000UL
#define BOARD_BOOTCLOCKRUN_SYS_PLL2_PFD0_CLK          352000000UL
#define BOARD_BOOTCLOCKRUN_SYS_PLL2_PFD1_CLK          594000000UL
#define BOARD_BOOTCLOCKRUN_SYS_PLL2_PFD2_CLK          396000000UL
#define BOARD_BOOTCLOCKRUN_SYS_PLL2_PFD3_CLK          297000000UL
#define BOARD_BOOTCLOCKRUN_SYS_PLL2_SS_MODULATION     0UL
#define BOARD_BOOTCLOCKRUN_SYS_PLL2_SS_RANGE          0UL
#define BOARD_BOOTCLOCKRUN_SYS_PLL3_CLK               480000000UL
#define BOARD_BOOTCLOCKRUN_SYS_PLL3_DIV2_CLK          240000000UL
#define BOARD_BOOTCLOCKRUN_SYS_PLL3_PFD0_CLK          664615384UL
#define BOARD_BOOTCLOCKRUN_SYS_PLL3_PFD1_CLK          508235294UL
#define BOARD_BOOTCLOCKRUN_SYS_PLL3_PFD2_CLK          270000000UL
#define BOARD_BOOTCLOCKRUN_SYS_PLL3_PFD3_CLK          392727272UL
#define BOARD_BOOTCLOCKRUN_USDHC1_CLK_ROOT            24000000UL
#define BOARD_BOOTCLOCKRUN_USDHC2_CLK_ROOT            24000000UL


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

#endif /* _CLOCK_CONFIG_H_ */

