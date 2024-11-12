/*
 * Copyright (c) 2013-2016 ARM Limited. All rights reserved.
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Licensed under the Apache License, Version 2.0 (the License); you may
 * not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an AS IS BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*! File: Driver_SPI_SDK2.h
* @brief The \b Driver_SPI_SDK2.h file declares instances of SPI drivers for SPI0 and SPI1.
*/

#ifndef __DRIVER_SPI_SDK2_H__
#define __DRIVER_SPI_SDK2_H__

#include "Driver_SPI.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define ARM_SPI_DRV_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(2, 0) /* Driver version */

#define SPI_SS_ACTIVE_LOW (ARM_SPI_SS_INACTIVE)
#define SPI_SS_ACTIVE_HIGH (ARM_SPI_SS_ACTIVE)

/*! Function pointer for the slave read information*/
typedef void (*fpSpiReadPreprocessFn_t)(void *pCmdOut, uint32_t offset, uint32_t size);
/*! Function pointer for the slave write information*/
typedef void (*fpSpiWritePreprocessFn_t)(void *pCmdOut, uint32_t offset, uint32_t size, void *pWritebuffer);

/*! @brief The SPI Slave Control Command Params SDK2.0 Driver. */
typedef struct
{
    void *pTargetSlavePinID;
    uint8_t activeValue;
    uint8_t cmdCode;
} spiControlParams_t;

/*! @brief The SPI Slave Transfer Command Params SDK2.0 Driver. */
typedef struct spi_mater_SlaveCmd
{
    uint32_t size;         /*!< The tranfer size.*/
    uint8_t *pReadBuffer;  /*!< The handle the readbuffer.*/
    uint8_t *pWriteBuffer; /*!< The handle the writecommand.*/
} spiCmdParams_t;

/*! @brief This structure defines the spi slave command format. */
typedef struct
{
    fpSpiReadPreprocessFn_t pReadPreprocessFN;
    fpSpiWritePreprocessFn_t pWritePreprocessFN;
    void *pTargetSlavePinID;
    uint8_t spiCmdLen;
    uint8_t ssActiveValue;
} spiSlaveSpecificParams_t;

/*******************************************************************************
 * Interface Handles
 ******************************************************************************/
extern ARM_DRIVER_SPI Driver_SPI0_KSDK2_Blocking;
extern ARM_DRIVER_SPI Driver_SPI0_KSDK2_NonBlocking;

extern ARM_DRIVER_SPI Driver_SPI1_KSDK2_Blocking;
extern ARM_DRIVER_SPI Driver_SPI1_KSDK2_NonBlocking;

/*******************************************************************************
 * Constants
 ******************************************************************************/
/* Driver Version */
static const ARM_DRIVER_VERSION DriverVersion = {ARM_SPI_API_VERSION, ARM_SPI_DRV_VERSION};

/* Driver Capabilities */
static const ARM_SPI_CAPABILITIES DriverCapabilities = {
    1, /* Simplex Mode (Master and Slave) */
    1, /* TI Synchronous Serial Interface */
    1, /* Microwire Interface */
    0  /* Signal Mode Fault event: \ref ARM_SPI_EVENT_MODE_FAULT */
};

#endif // __DRIVER_SPI_SDK2_H__
