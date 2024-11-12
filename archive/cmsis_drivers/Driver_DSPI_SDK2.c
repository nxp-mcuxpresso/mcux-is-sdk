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

/*! File: Driver_DSPI_SDK2.c
* @brief CMSIS Driver API Adaption for Freescale Kinetis SDK 2.0 SPI Master Driver.
         Supports DSP devices only and Slave Mode is not supported by this Driver.
*/

/*******************************************************************************
 * SDK Includes
 ******************************************************************************/
#include "fsl_dspi.h"
#include "Driver_gpio.h"
#include "Driver_gpio_sdk.h"
#include "Driver_SPI_SDK2.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define DSPI_CHANNEL_0_BASEADDR SPI0
#define DSPI_CHANNEL_0_CLK_SRC DSPI0_CLK_SRC
#define DSPI_CHANNEL_0_IRQ SPI0_IRQn

#define DSPI_CHANNEL_1_BASEADDR SPI1
#define DSPI_CHANNEL_1_CLK_SRC DSPI1_CLK_SRC
#define DSPI_CHANNEL_1_IRQ SPI1_IRQn

typedef struct
{
    void *pDspiSShandle;
    volatile bool dspiMasterCompletionFlag;
    dspi_master_config_t dspiConfig;
    dspi_master_handle_t dspiHandle;
    ARM_SPI_SignalEvent_t cbDspiEvent;
} spiStaticConfiguration_t;

/*******************************************************************************
 * Variables
 ******************************************************************************/
/* SPI Channel Configurations */
dspi_transfer_t gDspi0Xfer, gDspi1Xfer;
spiStaticConfiguration_t gDspi0Config, gDspi1Config;

/* The GPIO Driver Handle. */
GENERIC_DRIVER_GPIO *pDspiGpioDriver = &Driver_GPIO_KSDK;

/*******************************************************************************
 * Common Functions
 ******************************************************************************/
ARM_DRIVER_VERSION KSDK2_SPI_GetVersion(void)
{
    return DriverVersion;
}

ARM_SPI_CAPABILITIES KSDK2_SPI_GetCapabilities(void)
{
    return DriverCapabilities;
}

/*******************************************************************************
 * SPI Channel #0 Functions
 ******************************************************************************/
void DSPI0_MasterUserCallback(SPI_Type *pBase, dspi_master_handle_t *pHandle, status_t status, void *pUserData)
{
    uint32_t event;

    switch (status)
    {
        case kStatus_Success:
            gDspi0Config.dspiMasterCompletionFlag = true;
            event = ARM_SPI_EVENT_TRANSFER_COMPLETE;
            break;
        default:
            event = ARM_SPI_EVENT_DATA_LOST;
            break;
    }

    if (gDspi0Config.cbDspiEvent)
    {
        gDspi0Config.cbDspiEvent(event);
    }
}

int32_t KSDK2_SPI0_MasterInitialize(ARM_SPI_SignalEvent_t cbEvent)
{
    /* Load the default SPI configuration. */
    DSPI_MasterGetDefaultConfig(&(gDspi0Config.dspiConfig));
    /* Initialize SPI0 in Master Mode with default SPI configuration. */
    DSPI_MasterInit(DSPI_CHANNEL_0_BASEADDR, &(gDspi0Config.dspiConfig), CLOCK_GetFreq(DSPI_CHANNEL_0_CLK_SRC));
    /* Set up master transfer. */
    DSPI_MasterTransferCreateHandle(DSPI_CHANNEL_0_BASEADDR, &(gDspi0Config.dspiHandle), DSPI0_MasterUserCallback,
                                    NULL);

    gDspi0Config.cbDspiEvent = cbEvent;
    gDspi0Config.dspiMasterCompletionFlag = false;

    return ARM_DRIVER_OK;
}

int32_t KSDK2_SPI0_MasterUninitialize(void)
{
    DSPI_Deinit(DSPI_CHANNEL_0_BASEADDR);

    return ARM_DRIVER_OK;
}

int32_t KSDK2_SPI0_PowerControl(ARM_POWER_STATE state)
{
    switch (state)
    {
        case ARM_POWER_OFF:
            KSDK2_SPI0_MasterUninitialize();
            break;

        case ARM_POWER_FULL:
            KSDK2_SPI0_MasterInitialize(NULL);
            break;

        default:
            return ARM_DRIVER_ERROR_UNSUPPORTED;
    }

    return ARM_DRIVER_OK;
}

int32_t KSDK2_SPI0_MasterSendNonBlocking(const void *pData, uint32_t num)
{
    return ARM_DRIVER_ERROR_UNSUPPORTED;
}

int32_t KSDK2_SPI0_MasterReceiveNonBlocking(void *pData, uint32_t num)
{
    return ARM_DRIVER_ERROR_UNSUPPORTED;
}

int32_t KSDK2_SPI0_MasterTransferNonBlocking(const void *pDataOut, void *pDataIn, uint32_t num)
{
    status_t status;

    /* Start master transfer */
    gDspi0Xfer.txData = (uint8_t *)pDataOut;
    gDspi0Xfer.rxData = (uint8_t *)pDataIn;
    gDspi0Xfer.dataSize = num;
    gDspi0Xfer.configFlags = kDSPI_MasterCtar0 | kDSPI_MasterPcs0 | kDSPI_MasterPcsContinuous;

    status = DSPI_MasterTransferNonBlocking(DSPI_CHANNEL_0_BASEADDR, &(gDspi0Config.dspiHandle), &(gDspi0Xfer));
    while (!gDspi0Config.dspiMasterCompletionFlag)
    {
        __NOP();
    }

    gDspi0Config.dspiMasterCompletionFlag = false;
    if (kStatus_Success == status)
    {
        return ARM_DRIVER_OK;
    }
    else
    {
        return ARM_DRIVER_ERROR;
    }
}

int32_t KSDK2_SPI0_MasterSendBlocking(const void *pData, uint32_t num)
{
    return ARM_DRIVER_ERROR_UNSUPPORTED;
}

int32_t KSDK2_SPI0_MasterReceiveBlocking(void *pData, uint32_t num)
{
    return ARM_DRIVER_ERROR_UNSUPPORTED;
}

int32_t KSDK2_SPI0_MasterTransferBlocking(const void *pDataOut, void *pDataIn, uint32_t num)
{
    status_t status;

    /* Start master transfer */
    gDspi0Xfer.txData = (uint8_t *)pDataOut;
    gDspi0Xfer.rxData = (uint8_t *)pDataIn;
    gDspi0Xfer.dataSize = num;
    gDspi0Xfer.configFlags = kDSPI_MasterCtar0 | kDSPI_MasterPcs0 | kDSPI_MasterPcsContinuous;

    status = DSPI_MasterTransferBlocking(DSPI_CHANNEL_0_BASEADDR, &(gDspi0Xfer));
    if (kStatus_Success == status)
    {
        return ARM_DRIVER_OK;
    }
    else
    {
        return ARM_DRIVER_ERROR;
    }
}

uint32_t KSDK2_SPI0_MasterGetDataCount(void)
{
    size_t count;

    if (ARM_DRIVER_OK == DSPI_MasterTransferGetCount(DSPI_CHANNEL_0_BASEADDR, &(gDspi0Config.dspiHandle), &count))
    {
        return count;
    }
    else
    {
        return 0;
    }
}

int32_t KSDK2_SPI0_MasterControl(uint32_t control, uint32_t arg)
{
    switch (control & ARM_SPI_CONTROL_Msk)
    {
        case ARM_SPI_MODE_MASTER:   // SPI Master (Output on MOSI, Input on MISO); arg = Bus Speed in bps
        case ARM_SPI_SET_BUS_SPEED: // Set Bus Speed in bps; arg = value
            gDspi0Config.dspiConfig.ctarConfig.baudRate = arg;
            if (DSPI_MasterSetBaudRate(DSPI_CHANNEL_0_BASEADDR, gDspi0Config.dspiConfig.whichCtar, arg,
                                       CLOCK_GetFreq(DSPI_CHANNEL_0_CLK_SRC)))
            {
                return ARM_DRIVER_OK;
            }
            else
            {
                return ARM_DRIVER_ERROR;
            }

        case ARM_SPI_GET_BUS_SPEED: // Get Bus Speed in bps
            return gDspi0Config.dspiConfig.ctarConfig.baudRate;

        case ARM_SPI_CONTROL_SS: // Control Slave Select based on inactive/active and active low/high.
            if (((spiControlParams_t *)arg)->cmdCode == ((spiControlParams_t *)arg)->activeValue)
            {
                pDspiGpioDriver->set_pin(((spiControlParams_t *)arg)->pTargetSlavePinID);
            }
            else
            {
                pDspiGpioDriver->clr_pin(((spiControlParams_t *)arg)->pTargetSlavePinID);
            }
            return 0;

        case ARM_SPI_ABORT_TRANSFER: // Abort current data transfer
            DSPI_MasterTransferAbort(DSPI_CHANNEL_0_BASEADDR, &(gDspi0Config.dspiHandle));
            return ARM_DRIVER_OK;

        default:
            return ARM_DRIVER_ERROR_UNSUPPORTED;
    }
}

ARM_SPI_STATUS KSDK2_SPI0_MasterGetStatus(void)
{
    ARM_SPI_STATUS status;
    // Read the SPI Status Register
    uint32_t spi_status = DSPI_GetStatusFlags(DSPI_CHANNEL_0_BASEADDR);

    // Create the return status from the register flags
    // Refer Section Document: K64P144M120SF5RM Section: 50.3.5
    status.busy = !SPI_SR_TCF(spi_status);         // Transfer Complete Flag Complete=1, Ongoing=0
    status.data_lost = SPI_SR_RFOF(spi_status);    // Receive FIFO Overflow Flag
    status.mode_fault = !SPI_SR_TXRXS(spi_status); // TX and RX Status Stop=0, Run=1

    return status;
}

/*******************************************************************************
 * SPI Channel #1 Functions
 ******************************************************************************/
void DSPI1_MasterUserCallback(SPI_Type *pBase, dspi_master_handle_t *pHandle, status_t status, void *pUserData)
{
    uint32_t event;

    switch (status)
    {
        case kStatus_Success:
            gDspi1Config.dspiMasterCompletionFlag = true;
            event = ARM_SPI_EVENT_TRANSFER_COMPLETE;
            break;
        default:
            event = ARM_SPI_EVENT_DATA_LOST;
            break;
    }

    if (gDspi1Config.cbDspiEvent)
    {
        gDspi1Config.cbDspiEvent(event);
    }
}

int32_t KSDK2_SPI1_MasterInitialize(ARM_SPI_SignalEvent_t cbEvent)
{
    /* Load the default SPI configuration. */
    DSPI_MasterGetDefaultConfig(&(gDspi1Config.dspiConfig));
    /* Initialize SPI1 in Master Mode with default SPI configuration. */
    DSPI_MasterInit(DSPI_CHANNEL_1_BASEADDR, &(gDspi1Config.dspiConfig), CLOCK_GetFreq(DSPI_CHANNEL_1_CLK_SRC));
    /* Set up master transfer. */
    DSPI_MasterTransferCreateHandle(DSPI_CHANNEL_1_BASEADDR, &(gDspi1Config.dspiHandle), DSPI1_MasterUserCallback,
                                    NULL);

    gDspi1Config.cbDspiEvent = cbEvent;
    gDspi1Config.dspiMasterCompletionFlag = false;

    return ARM_DRIVER_OK;
}

int32_t KSDK2_SPI1_MasterUninitialize(void)
{
    DSPI_Deinit(DSPI_CHANNEL_1_BASEADDR);

    return ARM_DRIVER_OK;
}

int32_t KSDK2_SPI1_PowerControl(ARM_POWER_STATE state)
{
    switch (state)
    {
        case ARM_POWER_OFF:
            KSDK2_SPI1_MasterUninitialize();
            break;

        case ARM_POWER_FULL:
            KSDK2_SPI1_MasterInitialize(NULL);
            break;

        default:
            return ARM_DRIVER_ERROR_UNSUPPORTED;
    }

    return ARM_DRIVER_OK;
}

int32_t KSDK2_SPI1_MasterSendNonBlocking(const void *pData, uint32_t num)
{
    return ARM_DRIVER_ERROR_UNSUPPORTED;
}

int32_t KSDK2_SPI1_MasterReceiveNonBlocking(void *pData, uint32_t num)
{
    return ARM_DRIVER_ERROR_UNSUPPORTED;
}

int32_t KSDK2_SPI1_MasterTransferNonBlocking(const void *pDataOut, void *pDataIn, uint32_t num)
{
    status_t status;

    /* Start master transfer */
    gDspi1Xfer.txData = (uint8_t *)pDataOut;
    gDspi1Xfer.rxData = (uint8_t *)pDataIn;
    gDspi1Xfer.dataSize = num;
    gDspi1Xfer.configFlags = kDSPI_MasterCtar0 | kDSPI_MasterPcs0 | kDSPI_MasterPcsContinuous;

    status = DSPI_MasterTransferNonBlocking(DSPI_CHANNEL_1_BASEADDR, &(gDspi1Config.dspiHandle), &(gDspi1Xfer));
    while (!gDspi1Config.dspiMasterCompletionFlag)
    {
        __NOP();
    }

    gDspi1Config.dspiMasterCompletionFlag = false;
    if (kStatus_Success == status)
    {
        return ARM_DRIVER_OK;
    }
    else
    {
        return ARM_DRIVER_ERROR;
    }
}

int32_t KSDK2_SPI1_MasterSendBlocking(const void *pData, uint32_t num)
{
    return ARM_DRIVER_ERROR_UNSUPPORTED;
}

int32_t KSDK2_SPI1_MasterReceiveBlocking(void *pData, uint32_t num)
{
    return ARM_DRIVER_ERROR_UNSUPPORTED;
}

int32_t KSDK2_SPI1_MasterTransferBlocking(const void *pDataOut, void *pDataIn, uint32_t num)
{
    status_t status;

    /* Start master transfer */
    gDspi1Xfer.txData = (uint8_t *)pDataOut;
    gDspi1Xfer.rxData = (uint8_t *)pDataIn;
    gDspi1Xfer.dataSize = num;
    gDspi1Xfer.configFlags = kDSPI_MasterCtar0 | kDSPI_MasterPcs0 | kDSPI_MasterPcsContinuous;

    status = DSPI_MasterTransferBlocking(DSPI_CHANNEL_1_BASEADDR, &(gDspi1Xfer));
    if (kStatus_Success == status)
    {
        return ARM_DRIVER_OK;
    }
    else
    {
        return ARM_DRIVER_ERROR;
    }
}

uint32_t KSDK2_SPI1_MasterGetDataCount(void)
{
    size_t count;

    if (ARM_DRIVER_OK == DSPI_MasterTransferGetCount(DSPI_CHANNEL_1_BASEADDR, &(gDspi1Config.dspiHandle), &count))
    {
        return count;
    }
    else
    {
        return 0;
    }
}

int32_t KSDK2_SPI1_MasterControl(uint32_t control, uint32_t arg)
{
    switch (control & ARM_SPI_CONTROL_Msk)
    {
        case ARM_SPI_MODE_MASTER:   // SPI Master (Output on MOSI, Input on MISO); arg = Bus Speed in bps
        case ARM_SPI_SET_BUS_SPEED: // Set Bus Speed in bps; arg = value
            gDspi1Config.dspiConfig.ctarConfig.baudRate = arg;
            if (DSPI_MasterSetBaudRate(DSPI_CHANNEL_1_BASEADDR, gDspi1Config.dspiConfig.whichCtar, arg,
                                       CLOCK_GetFreq(DSPI_CHANNEL_1_CLK_SRC)))
            {
                return ARM_DRIVER_OK;
            }
            else
            {
                return ARM_DRIVER_ERROR;
            }

        case ARM_SPI_GET_BUS_SPEED: // Get Bus Speed in bps
            return gDspi1Config.dspiConfig.ctarConfig.baudRate;

        case ARM_SPI_CONTROL_SS: // Control Slave Select based on inactive/active and active low/high.
            if (((spiControlParams_t *)arg)->cmdCode == ((spiControlParams_t *)arg)->activeValue)
            {
                pDspiGpioDriver->set_pin(((spiControlParams_t *)arg)->pTargetSlavePinID);
            }
            else
            {
                pDspiGpioDriver->clr_pin(((spiControlParams_t *)arg)->pTargetSlavePinID);
            }
            return 0;

        case ARM_SPI_ABORT_TRANSFER: // Abort current data transfer
            DSPI_MasterTransferAbort(DSPI_CHANNEL_1_BASEADDR, &(gDspi1Config.dspiHandle));
            return ARM_DRIVER_OK;

        default:
            return ARM_DRIVER_ERROR_UNSUPPORTED;
    }
}

ARM_SPI_STATUS KSDK2_SPI1_MasterGetStatus(void)
{
    ARM_SPI_STATUS status;
    // Read the SPI Status Register
    uint32_t spi_status = DSPI_GetStatusFlags(DSPI_CHANNEL_1_BASEADDR);

    // Create the return status from the register flags
    // Refer Section Document: K64P144M120SF5RM Section: 50.3.5
    status.busy = !SPI_SR_TCF(spi_status);         // Transfer Complete Flag Complete=1, Ongoing=0
    status.data_lost = SPI_SR_RFOF(spi_status);    // Receive FIFO Overflow Flag
    status.mode_fault = !SPI_SR_TXRXS(spi_status); // TX and RX Status Stop=0, Run=1

    return status;
}

/*******************************************************************************
 * Interface Handles
 ******************************************************************************/
ARM_DRIVER_SPI Driver_SPI1_KSDK2_Blocking = {
    KSDK2_SPI_GetVersion,
    KSDK2_SPI_GetCapabilities,
    KSDK2_SPI1_MasterInitialize,
    KSDK2_SPI1_MasterUninitialize,
    KSDK2_SPI1_PowerControl,
    KSDK2_SPI1_MasterSendBlocking,
    KSDK2_SPI1_MasterReceiveBlocking,
    KSDK2_SPI1_MasterTransferBlocking,
    KSDK2_SPI1_MasterGetDataCount,
    KSDK2_SPI1_MasterControl,
    KSDK2_SPI1_MasterGetStatus,
};

ARM_DRIVER_SPI Driver_SPI1_KSDK2_NonBlocking = {
    KSDK2_SPI_GetVersion,
    KSDK2_SPI_GetCapabilities,
    KSDK2_SPI1_MasterInitialize,
    KSDK2_SPI1_MasterUninitialize,
    KSDK2_SPI1_PowerControl,
    KSDK2_SPI1_MasterSendNonBlocking,
    KSDK2_SPI1_MasterReceiveNonBlocking,
    KSDK2_SPI1_MasterTransferNonBlocking,
    KSDK2_SPI1_MasterGetDataCount,
    KSDK2_SPI1_MasterControl,
    KSDK2_SPI1_MasterGetStatus,
};

ARM_DRIVER_SPI Driver_SPI0_KSDK2_Blocking = {
    KSDK2_SPI_GetVersion,
    KSDK2_SPI_GetCapabilities,
    KSDK2_SPI0_MasterInitialize,
    KSDK2_SPI0_MasterUninitialize,
    KSDK2_SPI0_PowerControl,
    KSDK2_SPI0_MasterSendBlocking,
    KSDK2_SPI0_MasterReceiveBlocking,
    KSDK2_SPI0_MasterTransferBlocking,
    KSDK2_SPI0_MasterGetDataCount,
    KSDK2_SPI0_MasterControl,
    KSDK2_SPI0_MasterGetStatus,
};

ARM_DRIVER_SPI Driver_SPI0_KSDK2_NonBlocking = {
    KSDK2_SPI_GetVersion,
    KSDK2_SPI_GetCapabilities,
    KSDK2_SPI0_MasterInitialize,
    KSDK2_SPI0_MasterUninitialize,
    KSDK2_SPI0_PowerControl,
    KSDK2_SPI0_MasterSendNonBlocking,
    KSDK2_SPI0_MasterReceiveNonBlocking,
    KSDK2_SPI0_MasterTransferNonBlocking,
    KSDK2_SPI0_MasterGetDataCount,
    KSDK2_SPI0_MasterControl,
    KSDK2_SPI0_MasterGetStatus,
};
