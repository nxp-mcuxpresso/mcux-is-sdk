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
/*! File: Driver_SPI_SDK2.c
* @brief CMSIS Driver API Adaption for Freescale Kinetis SDK 2.0 SPI Master Driver.
         Supports DSP devices only and Slave Mode is not supported by this Driver.
*/

/*******************************************************************************
 * SDK Includes
 ******************************************************************************/
#include "fsl_spi.h"
#include "Driver_gpio.h"
#include "Driver_gpio_sdk.h"
#include "Driver_SPI_SDK2.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define SPI_CHANNEL_0_BASEADDR SPI0
#define SPI_CHANNEL_0_CLK_SRC SPI0_CLK_SRC
#define SPI_CHANNEL_0_IRQ SPI0_IRQn

#define SPI_CHANNEL_1_BASEADDR SPI1
#define SPI_CHANNEL_1_CLK_SRC SPI1_CLK_SRC
#define SPI_CHANNEL_1_IRQ SPI1_IRQn

typedef struct
{
    void *pSpiSShandle;
    volatile bool spiMasterCompletionFlag;
    spi_master_config_t spiConfig;
    spi_master_handle_t spiHandle;
    ARM_SPI_SignalEvent_t cbSpiEvent;
} spiStaticConfiguration_t;

/*******************************************************************************
 * Variables
 ******************************************************************************/
/* SPI Channel Configurations */
spi_transfer_t gSpi0Xfer, gSpi1Xfer;
spiStaticConfiguration_t gSpi0Config, gSpi1Config;

/* The GPIO Driver Handle. */
GENERIC_DRIVER_GPIO *pSpiGpioDriver = &Driver_GPIO_KSDK;

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
void SPI0_MasterUserCallback(SPI_Type *pBase, spi_master_handle_t *pHandle, status_t status, void *pUserData)
{
    uint32_t event;

    switch (status)
    {
        case kStatus_Success:
            gSpi0Config.spiMasterCompletionFlag = true;
            event = ARM_SPI_EVENT_TRANSFER_COMPLETE;
            break;
        default:
            event = ARM_SPI_EVENT_DATA_LOST;
            break;
    }

    if (gSpi0Config.cbSpiEvent)
    {
        gSpi0Config.cbSpiEvent(event);
    }
}

int32_t KSDK2_SPI0_MasterInitialize(ARM_SPI_SignalEvent_t cbEvent)
{
    /* Load the default SPI configuration. */
    SPI_MasterGetDefaultConfig(&(gSpi0Config.spiConfig));
    gSpi0Config.spiConfig.outputMode = kSPI_SlaveSelectAsGpio;
    /* Initialize SPI0 in Master Mode with default SPI configuration. */
    SPI_MasterInit(SPI_CHANNEL_0_BASEADDR, &(gSpi0Config.spiConfig), CLOCK_GetFreq(SPI_CHANNEL_0_CLK_SRC));
    /* Set up master transfer. */
    SPI_MasterTransferCreateHandle(SPI_CHANNEL_0_BASEADDR, &(gSpi0Config.spiHandle), SPI0_MasterUserCallback, NULL);

    gSpi0Config.cbSpiEvent = cbEvent;
    gSpi0Config.spiMasterCompletionFlag = false;

    return ARM_DRIVER_OK;
}

int32_t KSDK2_SPI0_MasterUninitialize(void)
{
    SPI_Deinit(SPI_CHANNEL_0_BASEADDR);

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
    gSpi0Xfer.txData = (uint8_t *)pDataOut;
    gSpi0Xfer.rxData = (uint8_t *)pDataIn;
    gSpi0Xfer.dataSize = num;

    status = SPI_MasterTransferNonBlocking(SPI_CHANNEL_0_BASEADDR, &(gSpi0Config.spiHandle), &(gSpi0Xfer));
    while (!gSpi0Config.spiMasterCompletionFlag)
    {
        __NOP();
    }

    gSpi0Config.spiMasterCompletionFlag = false;
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
    gSpi0Xfer.txData = (uint8_t *)pDataOut;
    gSpi0Xfer.rxData = (uint8_t *)pDataIn;
    gSpi0Xfer.dataSize = num;

    status = SPI_MasterTransferBlocking(SPI_CHANNEL_0_BASEADDR, &(gSpi0Xfer));
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

    if (ARM_DRIVER_OK == SPI_MasterTransferGetCount(SPI_CHANNEL_0_BASEADDR, &(gSpi0Config.spiHandle), &count))
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
            gSpi0Config.spiConfig.baudRate_Bps = arg;
            SPI_MasterSetBaudRate(SPI_CHANNEL_0_BASEADDR, arg, CLOCK_GetFreq(SPI_CHANNEL_0_CLK_SRC));
            return ARM_DRIVER_OK;

        case ARM_SPI_GET_BUS_SPEED: // Get Bus Speed in bps
            return gSpi0Config.spiConfig.baudRate_Bps;

        case ARM_SPI_CONTROL_SS: // Control Slave Select based on inactive/active and active low/high.
            if (((spiControlParams_t *)arg)->cmdCode == ((spiControlParams_t *)arg)->activeValue)
            {
                pSpiGpioDriver->set_pin(((spiControlParams_t *)arg)->pTargetSlavePinID);
            }
            else
            {
                pSpiGpioDriver->clr_pin(((spiControlParams_t *)arg)->pTargetSlavePinID);
            }
            return 0;

        case ARM_SPI_ABORT_TRANSFER: // Abort current data transfer
            SPI_MasterTransferAbort(SPI_CHANNEL_0_BASEADDR, &(gSpi0Config.spiHandle));
            return ARM_DRIVER_OK;

        default:
            return ARM_DRIVER_ERROR_UNSUPPORTED;
    }
}

ARM_SPI_STATUS KSDK2_SPI0_MasterGetStatus(void)
{
    ARM_SPI_STATUS status;
    // Read the SPI Status Register
    uint32_t spi_status = SPI_GetStatusFlags(SPI_CHANNEL_0_BASEADDR);

    // Create the return status from the register flags
    // Refer Section Document: KL25P80M48SF0RM Section: 37.3.4
    status.busy = !SPI_S_SPTEF(spi_status);     // Tx Buf Empty Flag=1 Busy=0, Tx Buf Not Empty=0 Busy=1
    status.data_lost = SPI_S_SPRF(spi_status);  // Receive Buffer Full Flag
    status.mode_fault = SPI_S_MODF(spi_status); // Mode Fault flag

    return status;
}

/*******************************************************************************
 * SPI Channel #1 Functions
 ******************************************************************************/
void SPI1_MasterUserCallback(SPI_Type *pBase, spi_master_handle_t *pHandle, status_t status, void *pUserData)
{
    uint32_t event;

    switch (status)
    {
        case kStatus_Success:
            gSpi1Config.spiMasterCompletionFlag = true;
            event = ARM_SPI_EVENT_TRANSFER_COMPLETE;
            break;
        default:
            event = ARM_SPI_EVENT_DATA_LOST;
            break;
    }

    if (gSpi1Config.cbSpiEvent)
    {
        gSpi1Config.cbSpiEvent(event);
    }
}

int32_t KSDK2_SPI1_MasterInitialize(ARM_SPI_SignalEvent_t cbEvent)
{
    /* Load the default SPI configuration. */
    SPI_MasterGetDefaultConfig(&(gSpi1Config.spiConfig));
    /* Initialize SPI1 in Master Mode with default SPI configuration. */
    SPI_MasterInit(SPI_CHANNEL_1_BASEADDR, &(gSpi1Config.spiConfig), CLOCK_GetFreq(SPI_CHANNEL_1_CLK_SRC));
    /* Set up master transfer. */
    SPI_MasterTransferCreateHandle(SPI_CHANNEL_1_BASEADDR, &(gSpi1Config.spiHandle), SPI1_MasterUserCallback, NULL);

    gSpi1Config.cbSpiEvent = cbEvent;
    gSpi1Config.spiMasterCompletionFlag = false;

    return ARM_DRIVER_OK;
}

int32_t KSDK2_SPI1_MasterUninitialize(void)
{
    SPI_Deinit(SPI_CHANNEL_1_BASEADDR);

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
    gSpi1Xfer.txData = (uint8_t *)pDataOut;
    gSpi1Xfer.rxData = (uint8_t *)pDataIn;
    gSpi1Xfer.dataSize = num;

    status = SPI_MasterTransferNonBlocking(SPI_CHANNEL_1_BASEADDR, &(gSpi1Config.spiHandle), &(gSpi1Xfer));
    while (!gSpi1Config.spiMasterCompletionFlag)
    {
        __NOP();
    }

    gSpi1Config.spiMasterCompletionFlag = false;
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
    gSpi1Xfer.txData = (uint8_t *)pDataOut;
    gSpi1Xfer.rxData = (uint8_t *)pDataIn;
    gSpi1Xfer.dataSize = num;

    status = SPI_MasterTransferBlocking(SPI_CHANNEL_1_BASEADDR, &(gSpi1Xfer));
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

    if (ARM_DRIVER_OK == SPI_MasterTransferGetCount(SPI_CHANNEL_1_BASEADDR, &(gSpi1Config.spiHandle), &count))
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
            gSpi1Config.spiConfig.baudRate_Bps = arg;
            SPI_MasterSetBaudRate(SPI_CHANNEL_1_BASEADDR, arg, CLOCK_GetFreq(SPI_CHANNEL_1_CLK_SRC));
            return ARM_DRIVER_OK;

        case ARM_SPI_GET_BUS_SPEED: // Get Bus Speed in bps
            return gSpi1Config.spiConfig.baudRate_Bps;

        case ARM_SPI_CONTROL_SS: // Control Slave Select based on inactive/active and active low/high.
            if (((spiControlParams_t *)arg)->cmdCode == ((spiControlParams_t *)arg)->activeValue)
            {
                pSpiGpioDriver->set_pin(((spiControlParams_t *)arg)->pTargetSlavePinID);
            }
            else
            {
                pSpiGpioDriver->clr_pin(((spiControlParams_t *)arg)->pTargetSlavePinID);
            }
            return 0;

        case ARM_SPI_ABORT_TRANSFER: // Abort current data transfer
            SPI_MasterTransferAbort(SPI_CHANNEL_1_BASEADDR, &(gSpi1Config.spiHandle));
            return ARM_DRIVER_OK;

        default:
            return ARM_DRIVER_ERROR_UNSUPPORTED;
    }
}

ARM_SPI_STATUS KSDK2_SPI1_MasterGetStatus(void)
{
    ARM_SPI_STATUS status;
    // Read the SPI Status Register
    uint32_t spi_status = SPI_GetStatusFlags(SPI_CHANNEL_1_BASEADDR);

    // Create the return status from the register flags
    // Refer Section Document: KL25P80M48SF0RM Section: 37.3.4
    status.busy = !SPI_S_SPTEF(spi_status);     // Tx Buf Empty Flag=1 Busy=0, Tx Buf Not Empty=0 Busy=1
    status.data_lost = SPI_S_SPRF(spi_status);  // Receive Buffer Full Flag
    status.mode_fault = SPI_S_MODF(spi_status); // Mode Fault flag

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
