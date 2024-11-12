/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#ifndef __RTE_DEVICE_H
#define __RTE_DEVICE_H

/*Driver name mapping*/
#define RTE_I2C0 1
#define RTE_I2C0_DMA_EN 0
#define RTE_I2C1 1
#define RTE_I2C1_DMA_EN 0

#define RTE_SPI0 1
#define RTE_SPI0_DMA_EN 0
#define RTE_SPI1 0
#define RTE_SPI1_DMA_EN 0

#define RTE_USART0 1
#define RTE_USART0_DMA_EN 0

/* USART configuration. */
#define USART_RX_BUFFER_LEN 64
#define USART0_RX_BUFFER_ENABLE 1

/* I2C configuration */
#define RTE_I2C0_Master_DMA_BASE DMA0
#define RTE_I2C0_Master_DMA_CH 3

#define RTE_I2C1_Master_DMA_BASE DMA0
#define RTE_I2C1_Master_DMA_CH 5

/* SPI configuration. */
#define RTE_SPI0_SSEL_NUM kSPI_Ssel1
#define RTE_SPI0_SSEL_POL ((spi_spol_t)(kSPI_Spol0ActiveHigh | kSPI_Spol2ActiveHigh | kSPI_Spol3ActiveHigh))
#define RTE_SPI0_DMA_TX_CH 5
#define RTE_SPI0_DMA_TX_DMA_BASE DMA0
#define RTE_SPI0_DMA_RX_CH 4
#define RTE_SPI0_DMA_RX_DMA_BASE DMA0

#define RTE_SPI1_SSEL_NUM kSPI_Ssel1
#define RTE_SPI1_SSEL_POL ((spi_spol_t)(kSPI_Spol0ActiveHigh | kSPI_Spol2ActiveHigh | kSPI_Spol3ActiveHigh))
#define RTE_SPI1_DMA_TX_CH 7
#define RTE_SPI1_DMA_TX_DMA_BASE DMA0
#define RTE_SPI1_DMA_RX_CH 6
#define RTE_SPI1_DMA_RX_DMA_BASE DMA0

/* USART configuration. */
#define RTE_USART0_DMA_TX_CH 1
#define RTE_USART0_DMA_TX_DMA_BASE DMA0
#define RTE_USART0_DMA_RX_CH 0
#define RTE_USART0_DMA_RX_DMA_BASE DMA0

#endif /* __RTE_DEVICE_H */
