/*
 * Copyright 2018-2020 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _RTE_DEVICE_H
#define _RTE_DEVICE_H

extern void LPI2C2_InitPins();
extern void LPI2C2_DeinitPins();

extern void LPSPI3_InitPins();
extern void LPSPI3_DeinitPins();

extern void LPUART1_InitPins();
extern void LPUART1_DeinitPins();

/* Driver name mapping. */
/* User needs to provide the implementation of LPI2CX_GetFreq/LPI2CX_InitPins/LPI2CX_DeinitPins for the enabled LPI2C
 * instance. */
#define RTE_I2C2        1
#define RTE_I2C2_DMA_EN 0

/* User needs to provide the implementation of LPSPIX_GetFreq/LPSPIX_InitPins/LPSPIX_DeinitPins for the enabled LPSPI
 * instance. */
#define RTE_SPI3        1
#define RTE_SPI3_DMA_EN 0

/* User needs to provide the implementation of LPUARTX_GetFreq/LPUARTX_InitPins/LPUARTX_DeinitPins for the enabled
 * LPUART instance. */
#define RTE_USART1        1
#define RTE_USART1_DMA_EN 0

/* LPI2C configuration. */
#define RTE_I2C2_PIN_INIT           LPI2C2_InitPins
#define RTE_I2C2_PIN_DEINIT         LPI2C2_DeinitPins
#define RTE_I2C2_DMA_TX_CH          0
#define RTE_I2C2_DMA_TX_PERI_SEL    (uint8_t) kDmaRequestMuxLPI2C2
#define RTE_I2C2_DMA_TX_DMAMUX_BASE DMAMUX0
#define RTE_I2C2_DMA_TX_DMA_BASE    DMA0
#define RTE_I2C2_DMA_RX_CH          1
#define RTE_I2C2_DMA_RX_PERI_SEL    (uint8_t) kDmaRequestMuxLPI2C2
#define RTE_I2C2_DMA_RX_DMAMUX_BASE DMAMUX0
#define RTE_I2C2_DMA_RX_DMA_BASE    DMA0

/* SPI configuration. */
#define RTE_SPI3_PCS_TO_SCK_DELAY       1000
#define RTE_SPI3_SCK_TO_PSC_DELAY       1000
#define RTE_SPI3_BETWEEN_TRANSFER_DELAY 1000
#define RTE_SPI3_MASTER_PCS_PIN_SEL     (kLPSPI_MasterPcs0)
#define RTE_SPI3_SLAVE_PCS_PIN_SEL      (kLPSPI_SlavePcs0)
#define RTE_SPI3_PIN_INIT               LPSPI3_InitPins
#define RTE_SPI3_PIN_DEINIT             LPSPI3_DeinitPins
#define RTE_SPI3_DMA_TX_CH              0
#define RTE_SPI3_DMA_TX_PERI_SEL        (uint8_t) kDmaRequestMuxLPSPI3Tx
#define RTE_SPI3_DMA_TX_DMAMUX_BASE     DMAMUX0
#define RTE_SPI3_DMA_TX_DMA_BASE        DMA0
#define RTE_SPI3_DMA_RX_CH              1
#define RTE_SPI3_DMA_RX_PERI_SEL        (uint8_t) kDmaRequestMuxLPSPI3Rx
#define RTE_SPI3_DMA_RX_DMAMUX_BASE     DMAMUX0
#define RTE_SPI3_DMA_RX_DMA_BASE        DMA0

/* UART configuration. */
#define RTE_USART1_PIN_INIT           LPUART1_InitPins
#define RTE_USART1_PIN_DEINIT         LPUART1_DeinitPins
#define RTE_USART1_DMA_TX_CH          0
#define RTE_USART1_DMA_TX_PERI_SEL    (uint8_t) kDmaRequestMuxLPUART1Tx
#define RTE_USART1_DMA_TX_DMAMUX_BASE DMAMUX
#define RTE_USART1_DMA_TX_DMA_BASE    DMA0
#define RTE_USART1_DMA_RX_CH          1
#define RTE_USART1_DMA_RX_PERI_SEL    (uint8_t) kDmaRequestMuxLPUART1Rx
#define RTE_USART1_DMA_RX_DMAMUX_BASE DMAMUX
#define RTE_USART1_DMA_RX_DMA_BASE    DMA0

#endif /* _RTE_DEVICE_H */
