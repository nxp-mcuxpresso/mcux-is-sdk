/*
 * Copyright 2020 NXP
 * All rights reserved.
 *
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef __RTE_DEVICE_H
#define __RTE_DEVICE_H

/* Driver name mapping. */
#define RTE_I2C1        1
#define RTE_I2C1_DMA_EN 0
#define RTE_I2C2        0
#define RTE_I2C2_DMA_EN 0
#define RTE_I2C3        0
#define RTE_I2C3_DMA_EN 0
#define RTE_I2C4        0
#define RTE_I2C4_DMA_EN 0

#define RTE_SPI1        1
#define RTE_SPI1_DMA_EN 0
#define RTE_SPI2        0
#define RTE_SPI2_DMA_EN 0
#define RTE_SPI3        0
#define RTE_SPI3_DMA_EN 0
#define RTE_SPI4        0
#define RTE_SPI4_DMA_EN 0

#define RTE_USART1        1
#define RTE_USART1_DMA_EN 0
#define RTE_USART2        0
#define RTE_USART2_DMA_EN 0
#define RTE_USART3        0
#define RTE_USART3_DMA_EN 0
#define RTE_USART4        0
#define RTE_USART4_DMA_EN 0
#define RTE_USART5        0
#define RTE_USART5_DMA_EN 0
#define RTE_USART6        0
#define RTE_USART6_DMA_EN 0
#define RTE_USART7        0
#define RTE_USART7_DMA_EN 0
#define RTE_USART8        0
#define RTE_USART8_DMA_EN 0

/* LPI2C configuration. */
#define RTE_I2C1_DMA_TX_CH          0
#define RTE_I2C1_DMA_TX_PERI_SEL    (uint8_t) kDmaRequestMuxLPI2C1
#define RTE_I2C1_DMA_TX_DMAMUX_BASE DMAMUX
#define RTE_I2C1_DMA_TX_DMA_BASE    DMA0
#define RTE_I2C1_DMA_RX_CH          1
#define RTE_I2C1_DMA_RX_PERI_SEL    (uint8_t) kDmaRequestMuxLPI2C1
#define RTE_I2C1_DMA_RX_DMAMUX_BASE DMAMUX
#define RTE_I2C1_DMA_RX_DMA_BASE    DMA0

#define RTE_I2C2_DMA_TX_CH          2
#define RTE_I2C2_DMA_TX_PERI_SEL    (uint8_t) kDmaRequestMuxLPI2C2
#define RTE_I2C2_DMA_TX_DMAMUX_BASE DMAMUX
#define RTE_I2C2_DMA_TX_DMA_BASE    DMA0
#define RTE_I2C2_DMA_RX_CH          3
#define RTE_I2C2_DMA_RX_PERI_SEL    (uint8_t) kDmaRequestMuxLPI2C2
#define RTE_I2C2_DMA_RX_DMAMUX_BASE DMAMUX
#define RTE_I2C2_DMA_RX_DMA_BASE    DMA0

#define RTE_I2C3_DMA_TX_CH          4
#define RTE_I2C3_DMA_TX_PERI_SEL    (uint8_t) kDmaRequestMuxLPI2C3
#define RTE_I2C3_DMA_TX_DMAMUX_BASE DMAMUX
#define RTE_I2C3_DMA_TX_DMA_BASE    DMA0
#define RTE_I2C3_DMA_RX_CH          5
#define RTE_I2C3_DMA_RX_PERI_SEL    (uint8_t) kDmaRequestMuxLPI2C3
#define RTE_I2C3_DMA_RX_DMAMUX_BASE DMAMUX
#define RTE_I2C3_DMA_RX_DMA_BASE    DMA0

#define RTE_I2C4_DMA_TX_CH          6
#define RTE_I2C4_DMA_TX_PERI_SEL    (uint8_t) kDmaRequestMuxLPI2C4
#define RTE_I2C4_DMA_TX_DMAMUX_BASE DMAMUX
#define RTE_I2C4_DMA_TX_DMA_BASE    DMA0
#define RTE_I2C4_DMA_RX_CH          7
#define RTE_I2C4_DMA_RX_PERI_SEL    (uint8_t) kDmaRequestMuxLPI2C4
#define RTE_I2C4_DMA_RX_DMAMUX_BASE DMAMUX
#define RTE_I2C4_DMA_RX_DMA_BASE    DMA0

/* SPI configuration. */
#define RTE_SPI1_PCS_TO_SCK_DELAY       1000
#define RTE_SPI1_SCK_TO_PSC_DELAY       1000
#define RTE_SPI1_BETWEEN_TRANSFER_DELAY 1000
#define RTE_SPI1_MASTER_PCS_PIN_SEL     (kLPSPI_MasterPcs0)
#define RTE_SPI1_SLAVE_PCS_PIN_SEL      (kLPSPI_SlavePcs0)
#define RTE_SPI1_DMA_TX_CH              0
#define RTE_SPI1_DMA_TX_PERI_SEL        (uint8_t) kDmaRequestMuxLPSPI1Tx
#define RTE_SPI1_DMA_TX_DMAMUX_BASE     DMAMUX
#define RTE_SPI1_DMA_TX_DMA_BASE        DMA0
#define RTE_SPI1_DMA_RX_CH              1
#define RTE_SPI1_DMA_RX_PERI_SEL        (uint8_t) kDmaRequestMuxLPSPI1Rx
#define RTE_SPI1_DMA_RX_DMAMUX_BASE     DMAMUX
#define RTE_SPI1_DMA_RX_DMA_BASE        DMA0

#define RTE_SPI2_PCS_TO_SCK_DELAY       1000
#define RTE_SPI2_SCK_TO_PSC_DELAY       1000
#define RTE_SPI2_BETWEEN_TRANSFER_DELAY 1000
#define RTE_SPI2_MASTER_PCS_PIN_SEL     (kLPSPI_MasterPcs0)
#define RTE_SPI2_SLAVE_PCS_PIN_SEL      (kLPSPI_SlavePcs0)
#define RTE_SPI2_DMA_TX_CH              2
#define RTE_SPI2_DMA_TX_PERI_SEL        (uint8_t) kDmaRequestMuxLPSPI2Tx
#define RTE_SPI2_DMA_TX_DMAMUX_BASE     DMAMUX
#define RTE_SPI2_DMA_TX_DMA_BASE        DMA0
#define RTE_SPI2_DMA_RX_CH              3
#define RTE_SPI2_DMA_RX_PERI_SEL        (uint8_t) kDmaRequestMuxLPSPI2Tx
#define RTE_SPI2_DMA_RX_DMAMUX_BASE     DMAMUX
#define RTE_SPI2_DMA_RX_DMA_BASE        DMA0

#define RTE_SPI3_PCS_TO_SCK_DELAY       1000
#define RTE_SPI3_SCK_TO_PSC_DELAY       1000
#define RTE_SPI3_BETWEEN_TRANSFER_DELAY 1000
#define RTE_SPI3_MASTER_PCS_PIN_SEL     (kLPSPI_MasterPcs0)
#define RTE_SPI3_SLAVE_PCS_PIN_SEL      (kLPSPI_SlavePcs0)
#define RTE_SPI3_DMA_TX_CH              4
#define RTE_SPI3_DMA_TX_PERI_SEL        (uint8_t) kDmaRequestMuxLPSPI3Tx
#define RTE_SPI3_DMA_TX_DMAMUX_BASE     DMAMUX
#define RTE_SPI3_DMA_TX_DMA_BASE        DMA0
#define RTE_SPI3_DMA_RX_CH              5
#define RTE_SPI3_DMA_RX_PERI_SEL        (uint8_t) kDmaRequestMuxLPSPI3Rx
#define RTE_SPI3_DMA_RX_DMAMUX_BASE     DMAMUX
#define RTE_SPI3_DMA_RX_DMA_BASE        DMA0

#define RTE_SPI4_PCS_TO_SCK_DELAY       1000
#define RTE_SPI4_SCK_TO_PSC_DELAY       1000
#define RTE_SPI4_BETWEEN_TRANSFER_DELAY 1000
#define RTE_SPI4_MASTER_PCS_PIN_SEL     (kLPSPI_MasterPcs0)
#define RTE_SPI4_SLAVE_PCS_PIN_SEL      (kLPSPI_SlavePcs0)
#define RTE_SPI4_DMA_TX_CH              6
#define RTE_SPI4_DMA_TX_PERI_SEL        (uint8_t) kDmaRequestMuxLPSPI4Tx
#define RTE_SPI4_DMA_TX_DMAMUX_BASE     DMAMUX
#define RTE_SPI4_DMA_TX_DMA_BASE        DMA0
#define RTE_SPI4_DMA_RX_CH              7
#define RTE_SPI4_DMA_RX_PERI_SEL        (uint8_t) kDmaRequestMuxLPSPI4Rx
#define RTE_SPI4_DMA_RX_DMAMUX_BASE     DMAMUX
#define RTE_SPI4_DMA_RX_DMA_BASE        DMA0

/* UART configuration. */
#define RTE_USART1_DMA_TX_CH          0
#define RTE_USART1_DMA_TX_PERI_SEL    (uint8_t) kDmaRequestMuxLPUART1Tx
#define RTE_USART1_DMA_TX_DMAMUX_BASE DMAMUX
#define RTE_USART1_DMA_TX_DMA_BASE    DMA0
#define RTE_USART1_DMA_RX_CH          1
#define RTE_USART1_DMA_RX_PERI_SEL    (uint8_t) kDmaRequestMuxLPUART1Rx
#define RTE_USART1_DMA_RX_DMAMUX_BASE DMAMUX
#define RTE_USART1_DMA_RX_DMA_BASE    DMA0

#define RTE_USART2_DMA_TX_CH          0
#define RTE_USART2_DMA_TX_PERI_SEL    (uint8_t) kDmaRequestMuxLPUART2Tx
#define RTE_USART2_DMA_TX_DMAMUX_BASE DMAMUX
#define RTE_USART2_DMA_TX_DMA_BASE    DMA0
#define RTE_USART2_DMA_RX_CH          1
#define RTE_USART2_DMA_RX_PERI_SEL    (uint8_t) kDmaRequestMuxLPUART2Rx
#define RTE_USART2_DMA_RX_DMAMUX_BASE DMAMUX
#define RTE_USART2_DMA_RX_DMA_BASE    DMA0

#define RTE_USART3_DMA_TX_CH          0
#define RTE_USART3_DMA_TX_PERI_SEL    (uint8_t) kDmaRequestMuxLPUART3Tx
#define RTE_USART3_DMA_TX_DMAMUX_BASE DMAMUX
#define RTE_USART3_DMA_TX_DMA_BASE    DMA0
#define RTE_USART3_DMA_RX_CH          1
#define RTE_USART3_DMA_RX_PERI_SEL    (uint8_t) kDmaRequestMuxLPUART3Rx
#define RTE_USART3_DMA_RX_DMAMUX_BASE DMAMUX
#define RTE_USART3_DMA_RX_DMA_BASE    DMA0

#define RTE_USART4_DMA_TX_CH          0
#define RTE_USART4_DMA_TX_PERI_SEL    (uint8_t) kDmaRequestMuxLPUART4Tx
#define RTE_USART4_DMA_TX_DMAMUX_BASE DMAMUX
#define RTE_USART4_DMA_TX_DMA_BASE    DMA0
#define RTE_USART4_DMA_RX_CH          1
#define RTE_USART4_DMA_RX_PERI_SEL    (uint8_t) kDmaRequestMuxLPUART4Rx
#define RTE_USART4_DMA_RX_DMAMUX_BASE DMAMUX
#define RTE_USART4_DMA_RX_DMA_BASE    DMA0

#define RTE_USART5_DMA_TX_CH          0
#define RTE_USART5_DMA_TX_PERI_SEL    (uint8_t) kDmaRequestMuxLPUART5Tx
#define RTE_USART5_DMA_TX_DMAMUX_BASE DMAMUX
#define RTE_USART5_DMA_TX_DMA_BASE    DMA0
#define RTE_USART5_DMA_RX_CH          1
#define RTE_USART5_DMA_RX_PERI_SEL    (uint8_t) kDmaRequestMuxLPUART5Rx
#define RTE_USART5_DMA_RX_DMAMUX_BASE DMAMUX
#define RTE_USART5_DMA_RX_DMA_BASE    DMA0

#define RTE_USART6_DMA_TX_CH          0
#define RTE_USART6_DMA_TX_PERI_SEL    (uint8_t) kDmaRequestMuxLPUART6Tx
#define RTE_USART6_DMA_TX_DMAMUX_BASE DMAMUX
#define RTE_USART6_DMA_TX_DMA_BASE    DMA0
#define RTE_USART6_DMA_RX_CH          1
#define RTE_USART6_DMA_RX_PERI_SEL    (uint8_t) kDmaRequestMuxLPUART6Rx
#define RTE_USART6_DMA_RX_DMAMUX_BASE DMAMUX
#define RTE_USART6_DMA_RX_DMA_BASE    DMA0

#define RTE_USART7_DMA_TX_CH          0
#define RTE_USART7_DMA_TX_PERI_SEL    (uint8_t) kDmaRequestMuxLPUART7Tx
#define RTE_USART7_DMA_TX_DMAMUX_BASE DMAMUX
#define RTE_USART7_DMA_TX_DMA_BASE    DMA0
#define RTE_USART7_DMA_RX_CH          1
#define RTE_USART7_DMA_RX_PERI_SEL    (uint8_t) kDmaRequestMuxLPUART7Rx
#define RTE_USART7_DMA_RX_DMAMUX_BASE DMAMUX
#define RTE_USART7_DMA_RX_DMA_BASE    DMA0

#define RTE_USART8_DMA_TX_CH          0
#define RTE_USART8_DMA_TX_PERI_SEL    (uint8_t) kDmaRequestMuxLPUART8Tx
#define RTE_USART8_DMA_TX_DMAMUX_BASE DMAMUX
#define RTE_USART8_DMA_TX_DMA_BASE    DMA0
#define RTE_USART8_DMA_RX_CH          1
#define RTE_USART8_DMA_RX_PERI_SEL    (uint8_t) kDmaRequestMuxLPUART8Rx
#define RTE_USART8_DMA_RX_DMAMUX_BASE DMAMUX
#define RTE_USART8_DMA_RX_DMA_BASE    DMA0

#endif /* __RTE_DEVICE_H */
