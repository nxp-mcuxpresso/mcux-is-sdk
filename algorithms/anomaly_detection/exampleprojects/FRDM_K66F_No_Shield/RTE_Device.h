/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __RTE_DEVICE_H
#define __RTE_DEVICE_H

/* Driver name mapping. */
#define UART0_Edma_Driver Driver_USART0

/* UART configuration. */
#define RTE_UART0_DMA_TX_CH 0
#define RTE_UART0_DMA_TX_PERI_SEL (uint8_t) kDmaRequestMux0UART0Tx
#define RTE_UART0_DMA_TX_DMAMUX_BASE DMAMUX0
#define RTE_UART0_DMA_TX_DMA_BASE DMA0
#define RTE_UART0_DMA_RX_CH 1
#define RTE_UART0_DMA_RX_PERI_SEL (uint8_t) kDmaRequestMux0UART0Rx
#define RTE_UART0_DMA_RX_DMAMUX_BASE DMAMUX0
#define RTE_UART0_DMA_RX_DMA_BASE DMA0

#define RTE_UART1_DMA_TX_CH 0
#define RTE_UART1_DMA_TX_PERI_SEL (uint8_t) kDmaRequestMux0UART1Tx
#define RTE_UART1_DMA_TX_DMAMUX_BASE DMAMUX0
#define RTE_UART1_DMA_TX_DMA_BASE DMA0
#define RTE_UART1_DMA_RX_CH 1
#define RTE_UART1_DMA_RX_PERI_SEL (uint8_t) kDmaRequestMux0UART1Rx
#define RTE_UART1_DMA_RX_DMAMUX_BASE DMAMUX0
#define RTE_UART1_DMA_RX_DMA_BASE DMA0

#define RTE_UART2_DMA_TX_CH 0
#define RTE_UART2_DMA_TX_PERI_SEL (uint8_t) kDmaRequestMux0UART2Tx
#define RTE_UART2_DMA_TX_DMAMUX_BASE DMAMUX0
#define RTE_UART2_DMA_TX_DMA_BASE DMA0
#define RTE_UART2_DMA_RX_CH 1
#define RTE_UART2_DMA_RX_PERI_SEL (uint8_t) kDmaRequestMux0UART2Rx
#define RTE_UART2_DMA_RX_DMAMUX_BASE DMAMUX0
#define RTE_UART2_DMA_RX_DMA_BASE DMA0

#define RTE_UART3_DMA_TX_CH 0
#define RTE_UART3_DMA_TX_PERI_SEL (uint8_t) kDmaRequestMux0UART3Tx
#define RTE_UART3_DMA_TX_DMAMUX_BASE DMAMUX0
#define RTE_UART3_DMA_TX_DMA_BASE DMA0
#define RTE_UART3_DMA_RX_CH 1
#define RTE_UART3_DMA_RX_PERI_SEL (uint8_t) kDmaRequestMux0UART3Rx
#define RTE_UART3_DMA_RX_DMAMUX_BASE DMAMUX0
#define RTE_UART3_DMA_RX_DMA_BASE DMA0

#define RTE_UART4_DMA_TX_CH 0
#define RTE_UART4_DMA_TX_PERI_SEL (uint8_t) kDmaRequestMux0UART4
#define RTE_UART4_DMA_TX_DMAMUX_BASE DMAMUX0
#define RTE_UART4_DMA_TX_DMA_BASE DMA0
#define RTE_UART4_DMA_RX_CH 1
#define RTE_UART4_DMA_RX_PERI_SEL (uint8_t) kDmaRequestMux0UART4
#define RTE_UART4_DMA_RX_DMAMUX_BASE DMAMUX0
#define RTE_UART4_DMA_RX_DMA_BASE DMA0



/*Driver name mapping*/
#define I2C0_Interrupt_Driver Driver_I2C0
#define I2C1_Interrupt_Driver Driver_I2C1

/*I2C configuration*/
#define RTE_I2C0_Master_DMA_BASE DMA0
#define RTE_I2C0_Master_DMA_CH 0
#define RTE_I2C0_Master_DMAMUX_BASE DMAMUX0
#define RTE_I2C0_Master_PERI_SEL kDmaRequestMux0I2C0

#define RTE_I2C1_Master_DMA_BASE DMA0
#define RTE_I2C1_Master_DMA_CH 1
#define RTE_I2C1_Master_DMAMUX_BASE DMAMUX0
#define RTE_I2C1_Master_PERI_SEL kDmaRequestMux0I2C1

#define RTE_I2C2_Master_DMA_BASE DMA0
#define RTE_I2C2_Master_DMA_CH 2
#define RTE_I2C2_Master_DMAMUX_BASE DMAMUX0
#define RTE_I2C2_Master_PERI_SEL kDmaRequestMux0I2C2

#define RTE_I2C3_Master_DMA_BASE DMA0
#define RTE_I2C3_Master_DMA_CH 3
#define RTE_I2C3_Master_DMAMUX_BASE DMAMUX0
#define RTE_I2C3_Master_PERI_SEL kDmaRequestMux0I2C3

#endif /* __RTE_DEVICE_H */
