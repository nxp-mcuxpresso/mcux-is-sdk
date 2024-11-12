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

/*! File: Driver_I2C_SDK2.c
* @brief CMSIS Driver API Adaption to Freescale SDK 2.0 I2C Master Driver
*/

#include "Driver_I2C.h"
#include "fsl_i2c.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define ARM_I2C_DRV_VERSION ARM_DRIVER_VERSION_MAJOR_MINOR(2, 0) /* driver version */
#define I2C_CHANNEL0 0
#define I2C_CHANNEL1 1
#define I2C_IS_MASTER 1
#define I2C_TIMEOUT 1000
#define I2C0_MASTER_CLK_SRC I2C0_CLK_SRC
#define I2C1_MASTER_CLK_SRC I2C1_CLK_SRC
#define I2C_DEFAULT_BAUDRATE 400000

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/*******************************************************************************
 * Variables
 ******************************************************************************/
/* Driver Version */
static const ARM_DRIVER_VERSION DriverVersion = {ARM_I2C_API_VERSION, ARM_I2C_DRV_VERSION};

/* Driver Capabilities */
static const ARM_I2C_CAPABILITIES DriverCapabilities = {
    0 /* supports 10-bit addressing */
};

/* I2C Channel #0 Static Configuration */
ARM_I2C_SignalEvent_t KSDK2_I2C0_Event_Callback;
volatile bool g_I2C0_MasterCompletionFlag = false;
i2c_master_config_t i2c0_config;
i2c_master_transfer_t i2c0_transfer;
i2c_master_handle_t i2c0_handle;

/* I2C Channel #1 Static Configuration */
ARM_I2C_SignalEvent_t KSDK2_I2C1_Event_Callback;
volatile bool g_I2C1_MasterCompletionFlag = false;
i2c_master_config_t i2c1_config;
i2c_master_transfer_t i2c1_transfer;
i2c_master_handle_t i2c1_handle;

typedef struct
{
    uint32_t reserved1 : 6;
    uint32_t BBF : 1;
    uint32_t MBF : 1;
    uint32_t reserved2 : 9;
    uint32_t DMF : 1;
    uint32_t PLTF : 1;
    uint32_t FEF : 1;
    uint32_t ALF : 1;
    uint32_t NDF : 1;
    uint32_t SDF : 1;
    uint32_t EPF : 1;
    uint32_t reserved3 : 6;
    uint32_t RDF : 1;
    uint32_t TDF : 1;
} I2C_MSR_t;

/*******************************************************************************
 * Code
 ******************************************************************************/

/*******************************************************************************
 * Functions - I2C Channel #0
 ******************************************************************************/
static void i2c0_transfer_done_callback(I2C_Type *base, i2c_master_handle_t *handle, status_t status, void *userData)
{
    uint32_t event;

    switch (status)
    {
        case kStatus_Success:
            event = ARM_I2C_EVENT_TRANSFER_DONE;
            break;
        case kStatus_I2C_Nak:
            event = ARM_I2C_EVENT_ADDRESS_NACK;
            break;
        case kStatus_I2C_Timeout:
        default:
            event = ARM_I2C_EVENT_BUS_ERROR;
            break;
    }

    g_I2C0_MasterCompletionFlag = true;

    if (KSDK2_I2C0_Event_Callback)
    {
        KSDK2_I2C0_Event_Callback(event);
    }
}

ARM_DRIVER_VERSION KSDK2_I2C0_GetVersion(void)
{
    return DriverVersion;
}

ARM_I2C_CAPABILITIES KSDK2_I2C0_GetCapabilities(void)
{
    return DriverCapabilities;
}

int32_t KSDK2_I2C0_Initialize(ARM_I2C_SignalEvent_t cb_event)
{
    uint32_t sourceClock;

    memset(&i2c0_handle, 0, sizeof(i2c0_handle));
    memset(&i2c0_transfer, 0, sizeof(i2c0_transfer));

    // Initialize the I2C channel.
    I2C_MasterGetDefaultConfig(&i2c0_config);
    i2c0_config.baudRate_Bps = I2C_DEFAULT_BAUDRATE;
    sourceClock = CLOCK_GetFreq(I2C0_MASTER_CLK_SRC);
    I2C_MasterInit(I2C0, &i2c0_config, sourceClock);
    I2C_MasterTransferCreateHandle(I2C0, &i2c0_handle, i2c0_transfer_done_callback, NULL);

    // Store the event callback
    KSDK2_I2C0_Event_Callback = cb_event;

    return ARM_DRIVER_OK;
}

int32_t KSDK2_I2C0_Uninitialize(void)
{
    return ARM_DRIVER_ERROR;
}

int32_t KSDK2_I2C0_PowerControl(ARM_POWER_STATE state)
{
    switch (state)
    {
        case ARM_POWER_OFF:
            break;

        case ARM_POWER_LOW:
            break;

        case ARM_POWER_FULL:
            // Initialize the I2C channel.
            KSDK2_I2C0_Initialize(NULL);
            break;

        default:
            return ARM_DRIVER_ERROR_UNSUPPORTED;
    }

    return ARM_DRIVER_OK;
}

int32_t KSDK2_I2C0_MasterTransmitBlocking(uint32_t addr, const uint8_t *data, uint32_t num, bool xfer_pending)
{
    status_t result;

    // Set up the transfer information
    i2c0_transfer.direction = kI2C_Write;
    i2c0_transfer.slaveAddress = addr;
    i2c0_transfer.subaddress = data[0];
    i2c0_transfer.subaddressSize = 1;
    i2c0_transfer.data = (void *)&data[1];
    i2c0_transfer.dataSize = num - 1;

    // Send the data to the device and return the proper status
    do
    {
        result = I2C_MasterTransferBlocking(I2C0, &i2c0_transfer);
    } while (result == kStatus_I2C_Busy);

    if (kStatus_Success == result)
    {
        return ARM_DRIVER_OK;
    }
    else
    {
        return ARM_DRIVER_ERROR;
    }
}

int32_t KSDK2_I2C0_MasterTransmitNonBlocking(uint32_t addr, const uint8_t *data, uint32_t num, bool xfer_pending)
{
    status_t status;

    // Set up the transfer information
    i2c0_transfer.direction = kI2C_Write;
    i2c0_transfer.slaveAddress = addr;
    i2c0_transfer.subaddress = data[0];
    i2c0_transfer.subaddressSize = 1;
    i2c0_transfer.data = (void *)&data[1];
    i2c0_transfer.dataSize = num - 1;

    // Send the data to the device and return the proper status
    status = I2C_MasterTransferNonBlocking(I2C0, &i2c0_handle, &i2c0_transfer);

    while (!g_I2C0_MasterCompletionFlag)
    {
        //  do nothing
    }
    g_I2C0_MasterCompletionFlag = false;

    // Return status
    if (kStatus_Success == status)
    {
        return ARM_DRIVER_OK;
    }
    else
    {
        return ARM_DRIVER_ERROR;
    }
}

int32_t KSDK2_I2C0_MasterReceiveBlocking(uint32_t addr, uint8_t *data, uint32_t num, bool xfer_pending)
{
    // Set up the transfer information
    i2c0_transfer.direction = kI2C_Read;
    i2c0_transfer.slaveAddress = addr;
    i2c0_transfer.subaddress = data[0];
    i2c0_transfer.subaddressSize = 1;
    i2c0_transfer.data = (void *)&data[1];
    i2c0_transfer.dataSize = num - 1;

    // Send the data to the device and return the proper status
    if (kStatus_Success == I2C_MasterTransferBlocking(I2C0, &i2c0_transfer))
    {
        return ARM_DRIVER_OK;
    }
    else
    {
        return ARM_DRIVER_ERROR;
    }
}

int32_t KSDK2_I2C0_MasterReceiveNonBlocking(uint32_t addr, uint8_t *data, uint32_t num, bool xfer_pending)
{
    int32_t status;

    // Set up the transfer information
    i2c0_transfer.direction = kI2C_Read;
    i2c0_transfer.slaveAddress = addr;
    i2c0_transfer.subaddress = data[0];
    i2c0_transfer.subaddressSize = 1;
    i2c0_transfer.data = (void *)&data[1];
    i2c0_transfer.dataSize = num - 1;

    // Send the data to the device and return the proper status
    status = I2C_MasterTransferNonBlocking(I2C0, &i2c0_handle, &i2c0_transfer);

    while (!g_I2C0_MasterCompletionFlag)
    {
        //  do nothing
    }
    g_I2C0_MasterCompletionFlag = false;

    // Return status
    if (kStatus_Success == status)
    {
        return ARM_DRIVER_OK;
    }
    else
    {
        return ARM_DRIVER_ERROR;
    }
}

// Slave mode is not supported
int32_t KSDK2_I2C0_SlaveTransmit(const uint8_t *data, uint32_t num)
{
    return 0;
}

// Slave mode is not supported
int32_t KSDK2_I2C0_SlaveReceive(uint8_t *data, uint32_t num)
{
    return 0;
}

int32_t KSDK2_I2C0_GetDataCount(void)
{
    size_t count;

    // I2C driver does not expose counts
    I2C_MasterTransferGetCount(I2C0, &i2c0_handle, &count);

    return (int32_t)count;
}

int32_t KSDK2_I2C0_Control(uint32_t control, uint32_t arg)
{
    switch (control)
    {
        // Slave mode is not supported
        case ARM_I2C_OWN_ADDRESS:
            break;

        // Set the I2C bus speed
        case ARM_I2C_BUS_SPEED:
            switch (arg)
            {
                case ARM_I2C_BUS_SPEED_STANDARD:
                    i2c0_config.baudRate_Bps = 100000;
                    break;
                case ARM_I2C_BUS_SPEED_FAST:
                    i2c0_config.baudRate_Bps = 400000;
                    break;
                case ARM_I2C_BUS_SPEED_FAST_PLUS:
                    i2c0_config.baudRate_Bps = 1000000;
                    break;
                default:
                    return ARM_DRIVER_ERROR_UNSUPPORTED;
            }
            break;

        // Not supported
        case ARM_I2C_BUS_CLEAR:
            break;

        // Abort any ongoing transfer
        case ARM_I2C_ABORT_TRANSFER:
            I2C_MasterTransferAbort(I2C0, &i2c0_handle);
            return ARM_DRIVER_OK;

        default:
            return ARM_DRIVER_ERROR_UNSUPPORTED;
    }

    return ARM_DRIVER_OK;
}

ARM_I2C_STATUS KSDK2_I2C0_GetStatus(void)
{
    ARM_I2C_STATUS status;
    I2C_MSR_t *i2c_status_ptr;
    uint32_t lp_status;

    // Read the I2C Status Register
    lp_status = I2C_MasterGetStatusFlags(I2C0);

    // Cast a pointer to dereference
    i2c_status_ptr = (I2C_MSR_t *)&lp_status;

    // Create the return status from the register flags
    status.arbitration_lost = i2c_status_ptr->ALF;
    status.busy = i2c_status_ptr->BBF;
    status.bus_error = 0;
    status.direction = i2c0_transfer.direction;
    status.general_call = 0;
    status.mode = 1; // Master only

    return status;
}

ARM_DRIVER_I2C Driver_I2C0_KSDK2_Blocking = {KSDK2_I2C0_GetVersion,
                                             KSDK2_I2C0_GetCapabilities,
                                             KSDK2_I2C0_Initialize,
                                             KSDK2_I2C0_Uninitialize,
                                             KSDK2_I2C0_PowerControl,
                                             KSDK2_I2C0_MasterTransmitBlocking,
                                             KSDK2_I2C0_MasterReceiveBlocking,
                                             KSDK2_I2C0_SlaveTransmit,
                                             KSDK2_I2C0_SlaveReceive,
                                             KSDK2_I2C0_GetDataCount,
                                             KSDK2_I2C0_Control,
                                             KSDK2_I2C0_GetStatus};

ARM_DRIVER_I2C Driver_I2C0_KSDK2_NonBlocking = {KSDK2_I2C0_GetVersion,
                                                KSDK2_I2C0_GetCapabilities,
                                                KSDK2_I2C0_Initialize,
                                                KSDK2_I2C0_Uninitialize,
                                                KSDK2_I2C0_PowerControl,
                                                KSDK2_I2C0_MasterTransmitNonBlocking,
                                                KSDK2_I2C0_MasterReceiveNonBlocking,
                                                KSDK2_I2C0_SlaveTransmit,
                                                KSDK2_I2C0_SlaveReceive,
                                                KSDK2_I2C0_GetDataCount,
                                                KSDK2_I2C0_Control,
                                                KSDK2_I2C0_GetStatus};

/*******************************************************************************
 * Functions - I2C Channel #1
 ******************************************************************************/
static void i2c1_transfer_done_callback(I2C_Type *base, i2c_master_handle_t *handle, status_t status, void *userData)
{
    uint32_t event;

    switch (status)
    {
        case kStatus_Success:
            event = ARM_I2C_EVENT_TRANSFER_DONE;
            break;
        case kStatus_I2C_Nak:
            event = ARM_I2C_EVENT_ADDRESS_NACK;
            break;
        case kStatus_I2C_Timeout:
        default:
            event = ARM_I2C_EVENT_BUS_ERROR;
            break;
    }

    g_I2C1_MasterCompletionFlag = true;

    if (KSDK2_I2C1_Event_Callback)
    {
        KSDK2_I2C1_Event_Callback(event);
    }
}

ARM_DRIVER_VERSION KSDK2_I2C1_GetVersion(void)
{
    return DriverVersion;
}

ARM_I2C_CAPABILITIES KSDK2_I2C1_GetCapabilities(void)
{
    return DriverCapabilities;
}

int32_t KSDK2_I2C1_Initialize(ARM_I2C_SignalEvent_t cb_event)
{
    uint32_t sourceClock;

    memset(&i2c1_handle, 0, sizeof(i2c1_handle));
    memset(&i2c1_transfer, 0, sizeof(i2c1_transfer));

    // Initialize the I2C channel.
    I2C_MasterGetDefaultConfig(&i2c1_config);
    i2c1_config.baudRate_Bps = I2C_DEFAULT_BAUDRATE;
    sourceClock = CLOCK_GetFreq(I2C1_MASTER_CLK_SRC);
    I2C_MasterInit(I2C1, &i2c1_config, sourceClock);
    I2C_MasterTransferCreateHandle(I2C1, &i2c1_handle, i2c1_transfer_done_callback, NULL);

    // Store the event callback
    KSDK2_I2C1_Event_Callback = cb_event;

    return ARM_DRIVER_OK;
}

int32_t KSDK2_I2C1_Uninitialize(void)
{
    return ARM_DRIVER_ERROR;
}

int32_t KSDK2_I2C1_PowerControl(ARM_POWER_STATE state)
{
    switch (state)
    {
        case ARM_POWER_OFF:
            break;

        case ARM_POWER_LOW:
            break;

        case ARM_POWER_FULL:
            // Initialize the I2C channel.
            KSDK2_I2C1_Initialize(NULL);
            break;

        default:
            return ARM_DRIVER_ERROR_UNSUPPORTED;
    }

    return ARM_DRIVER_OK;
}

int32_t KSDK2_I2C1_MasterTransmitBlocking(uint32_t addr, const uint8_t *data, uint32_t num, bool xfer_pending)
{
    status_t result;

    // Set up the transfer information
    i2c1_transfer.direction = kI2C_Write;
    i2c1_transfer.slaveAddress = addr;
    i2c1_transfer.subaddress = data[0];
    i2c1_transfer.subaddressSize = 1;
    i2c1_transfer.data = (void *)&data[1];
    i2c1_transfer.dataSize = num - 1;

    // Send the data to the device and return the proper status
    do
    {
        result = I2C_MasterTransferBlocking(I2C1, &i2c1_transfer);
    } while (result == kStatus_I2C_Busy);

    if (kStatus_Success == result)
    {
        return ARM_DRIVER_OK;
    }
    else
    {
        return ARM_DRIVER_ERROR;
    }
}

int32_t KSDK2_I2C1_MasterTransmitNonBlocking(uint32_t addr, const uint8_t *data, uint32_t num, bool xfer_pending)
{
    status_t status;

    // Set up the transfer information
    i2c1_transfer.direction = kI2C_Write;
    i2c1_transfer.slaveAddress = addr;
    i2c1_transfer.subaddress = data[0];
    i2c1_transfer.subaddressSize = 1;
    i2c1_transfer.data = (void *)&data[1];
    i2c1_transfer.dataSize = num - 1;

    // Send the data to the device and return the proper status
    status = I2C_MasterTransferNonBlocking(I2C1, &i2c1_handle, &i2c1_transfer);

    while (!g_I2C1_MasterCompletionFlag)
    {
        //  do nothing
    }
    g_I2C1_MasterCompletionFlag = false;

    // Return status
    if (kStatus_Success == status)
    {
        return ARM_DRIVER_OK;
    }
    else
    {
        return ARM_DRIVER_ERROR;
    }
}

int32_t KSDK2_I2C1_MasterReceiveBlocking(uint32_t addr, uint8_t *data, uint32_t num, bool xfer_pending)
{
    // Set up the transfer information
    i2c1_transfer.flags = 0; // MES 30 June 2016
    i2c1_transfer.direction = kI2C_Read;
    i2c1_transfer.slaveAddress = addr;
    i2c1_transfer.subaddress = data[0];
    i2c1_transfer.subaddressSize = 1;
    i2c1_transfer.data = (void *)&data[1];
    i2c1_transfer.dataSize = num - 1;

    // Send the data to the device and return the proper status
    if (kStatus_Success == I2C_MasterTransferBlocking(I2C1, &i2c1_transfer))
    {
        return ARM_DRIVER_OK;
    }
    else
    {
        return ARM_DRIVER_ERROR;
    }
}

int32_t KSDK2_I2C1_MasterReceiveNonBlocking(uint32_t addr, uint8_t *data, uint32_t num, bool xfer_pending)
{
    int32_t status;

    // Set up the transfer information
    i2c1_transfer.direction = kI2C_Read;
    i2c1_transfer.slaveAddress = addr;
    i2c1_transfer.subaddress = data[0];
    i2c1_transfer.subaddressSize = 1;
    i2c1_transfer.data = (void *)&data[1];
    i2c1_transfer.dataSize = num - 1;

    // Send the data to the device and return the proper status
    status = I2C_MasterTransferNonBlocking(I2C1, &i2c1_handle, &i2c1_transfer);

    while (!g_I2C1_MasterCompletionFlag)
    {
        //  do nothing
    }
    g_I2C1_MasterCompletionFlag = false;

    // Return status
    if (kStatus_Success == status)
    {
        return ARM_DRIVER_OK;
    }
    else
    {
        return ARM_DRIVER_ERROR;
    }
}

// Slave mode is not supported
int32_t KSDK2_I2C1_SlaveTransmit(const uint8_t *data, uint32_t num)
{
    return 0;
}

// Slave mode is not supported
int32_t KSDK2_I2C1_SlaveReceive(uint8_t *data, uint32_t num)
{
    return 0;
}

int32_t KSDK2_I2C1_GetDataCount(void)
{
    size_t count;

    // I2C driver does not expose counts
    I2C_MasterTransferGetCount(I2C1, &i2c1_handle, &count);

    return (int32_t)count;
}

int32_t KSDK2_I2C1_Control(uint32_t control, uint32_t arg)
{
    switch (control)
    {
        // Slave mode is not supported
        case ARM_I2C_OWN_ADDRESS:
            break;

        // Set the I2C bus speed
        case ARM_I2C_BUS_SPEED:
            switch (arg)
            {
                case ARM_I2C_BUS_SPEED_STANDARD:
                    i2c1_config.baudRate_Bps = 100000;
                    break;
                case ARM_I2C_BUS_SPEED_FAST:
                    i2c1_config.baudRate_Bps = 400000;
                    break;
                case ARM_I2C_BUS_SPEED_FAST_PLUS:
                    i2c1_config.baudRate_Bps = 1000000;
                    break;
                default:
                    return ARM_DRIVER_ERROR_UNSUPPORTED;
            }
            break;

        // Not supported
        case ARM_I2C_BUS_CLEAR:
            break;

        // Abort any ongoing transfer
        case ARM_I2C_ABORT_TRANSFER:
            I2C_MasterTransferAbort(I2C1, &i2c1_handle);
            return ARM_DRIVER_OK;

        default:
            return ARM_DRIVER_ERROR_UNSUPPORTED;
    }

    return ARM_DRIVER_OK;
}

ARM_I2C_STATUS KSDK2_I2C1_GetStatus(void)
{
    ARM_I2C_STATUS status;
    I2C_MSR_t *i2c_status_ptr;
    uint32_t lp_status;

    // Read the I2C Status Register
    lp_status = I2C_MasterGetStatusFlags(I2C1);

    // Cast a pointer to dereference
    i2c_status_ptr = (I2C_MSR_t *)&lp_status;

    // Create the return status from the register flags
    status.arbitration_lost = i2c_status_ptr->ALF;
    status.busy = i2c_status_ptr->BBF;
    status.bus_error = 0;
    status.direction = i2c1_transfer.direction;
    status.general_call = 0;
    status.mode = 1; // Master only

    return status;
}

ARM_DRIVER_I2C Driver_I2C1_KSDK2_Blocking = {KSDK2_I2C1_GetVersion,
                                             KSDK2_I2C1_GetCapabilities,
                                             KSDK2_I2C1_Initialize,
                                             KSDK2_I2C1_Uninitialize,
                                             KSDK2_I2C1_PowerControl,
                                             KSDK2_I2C1_MasterTransmitBlocking,
                                             KSDK2_I2C1_MasterReceiveBlocking,
                                             KSDK2_I2C1_SlaveTransmit,
                                             KSDK2_I2C1_SlaveReceive,
                                             KSDK2_I2C1_GetDataCount,
                                             KSDK2_I2C1_Control,
                                             KSDK2_I2C1_GetStatus};

ARM_DRIVER_I2C Driver_I2C1_KSDK2_NonBlocking = {KSDK2_I2C1_GetVersion,
                                                KSDK2_I2C1_GetCapabilities,
                                                KSDK2_I2C1_Initialize,
                                                KSDK2_I2C1_Uninitialize,
                                                KSDK2_I2C1_PowerControl,
                                                KSDK2_I2C1_MasterTransmitNonBlocking,
                                                KSDK2_I2C1_MasterReceiveNonBlocking,
                                                KSDK2_I2C1_SlaveTransmit,
                                                KSDK2_I2C1_SlaveReceive,
                                                KSDK2_I2C1_GetDataCount,
                                                KSDK2_I2C1_Control,
                                                KSDK2_I2C1_GetStatus};
