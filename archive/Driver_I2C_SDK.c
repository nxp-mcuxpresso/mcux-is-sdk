/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

// CMSIS Driver API Adaption to Freescale KSDK 1.2 I2C Master Driver

#include "Driver_I2C.h"
#include "fsl_i2c_shared_function.h"
#include "fsl_i2c_master_driver.h"

/*******************************************************************************
* Definitions
******************************************************************************/
#define ARM_I2C_DRV_VERSION    ARM_DRIVER_VERSION_MAJOR_MINOR(2, 0) /* driver version */
#define I2C_CHANNEL0 0
#define I2C_CHANNEL1 1
#define I2C_IS_MASTER 1
#define I2C_TIMEOUT 1000

/*******************************************************************************
* Prototypes
******************************************************************************/
/*******************************************************************************
* Variables
******************************************************************************/
//! @brief Saves the state of the I2C channels.
i2c_master_state_t      I2C0_State;
i2c_master_state_t      I2C1_State;

//! @brief Saves the state of the I2C device.
i2c_device_t Device;

/* Driver Version */
static const ARM_DRIVER_VERSION DriverVersion = {
    ARM_I2C_API_VERSION,
    ARM_I2C_DRV_VERSION
};

/* Driver Capabilities */
static const ARM_I2C_CAPABILITIES DriverCapabilities = {
    0  /* supports 10-bit addressing */
};

/*******************************************************************************
* Functions - Common to I2C0 and I2C1
******************************************************************************/
ARM_DRIVER_VERSION KSDK_GetVersion(void)
{
    return DriverVersion;
}

ARM_I2C_CAPABILITIES KSDK_GetCapabilities(void)
{
    return DriverCapabilities;
}

// Slave mode is not supported
int32_t KSDK_SlaveTransmit(const uint8_t *data, uint32_t num)
{
    return 0;
}

// Slave mode is not supported
int32_t KSDK_SlaveReceive(uint8_t *data, uint32_t num)
{
    return 0;
}

/*******************************************************************************
* Functions - I2C0
******************************************************************************/
int32_t KSDK_I2C0_Initialize(ARM_I2C_SignalEvent_t cb_event)
{
    // Initialize the I2C channel.
    i2c_status_t status = I2C_DRV_MasterInit(I2C_CHANNEL0, &I2C0_State);

    // Doesn't implement the cb_event.

    // Return the proper status.
    if (kStatus_I2C_Success == status)
    {
        return ARM_DRIVER_OK;
    }
    else
    {
        return ARM_DRIVER_ERROR;
    }
}

int32_t KSDK_I2C0_Uninitialize(void)
{
    // De-initialize the I2C channel and return the proper status.
    if (kStatus_I2C_Success == I2C_DRV_MasterDeinit(I2C_CHANNEL0))
    {
        return ARM_DRIVER_OK;
    }
    else
    {
        return ARM_DRIVER_ERROR;
    }
}

int32_t KSDK_I2C0_PowerControl(ARM_POWER_STATE state)
{
    switch (state)
    {
    case ARM_POWER_OFF:
        // De-initialize the I2C channel.
        I2C_DRV_MasterDeinit(I2C_CHANNEL0);
        break;

    case ARM_POWER_LOW:
        break;

    case ARM_POWER_FULL:
        // Initialize the I2C channel.
        I2C_DRV_MasterInit(I2C_CHANNEL0, &I2C0_State);
        break;

    default:
        return ARM_DRIVER_ERROR_UNSUPPORTED;
    }

    return ARM_DRIVER_OK;
}

int32_t KSDK_I2C0_MasterTransmitBlocking(uint32_t addr, const uint8_t *data, uint32_t num, bool xfer_pending)
{
    // Set up the device information
    Device.address = addr;
    Device.baudRate_kbps = I2C0_State.lastBaudRate_kbps;

    // Send the data to the device and return the proper status
    if (kStatus_I2C_Success == I2C_DRV_MasterSendDataBlocking(I2C_CHANNEL0,&Device,&data[0],1,&data[1],num-1,I2C_TIMEOUT))
    {
        return ARM_DRIVER_OK;
    }
    else
    {
        return ARM_DRIVER_ERROR;
    }
}

int32_t KSDK_I2C0_MasterReceiveBlocking(uint32_t addr, uint8_t *data, uint32_t num, bool xfer_pending)
{
    // Set up the device information
    Device.address = addr;
    Device.baudRate_kbps = I2C0_State.lastBaudRate_kbps;

    if (kStatus_I2C_Success == I2C_DRV_MasterReceiveDataBlocking(I2C_CHANNEL0,&Device,&data[0],1,&data[1],num-1,I2C_TIMEOUT))
    {
        return ARM_DRIVER_OK;
    }
    else
    {
        return ARM_DRIVER_ERROR;
    }
}

int32_t KSDK_I2C0_GetDataCount(void)
{
    int32_t status;
    uint32_t bytes;

    // Access the channel base address and get the current transfer direction
    I2C_Type * base = g_i2cBase[I2C_CHANNEL0];
    i2c_direction_t direction = I2C_HAL_GetDirMode(base);

    // Use the direction to get the remaining byte count
    switch (direction)
    {
    case kI2CReceive:
        status = I2C_DRV_MasterGetReceiveStatus(I2C_CHANNEL0,&bytes);
        break;
    case kI2CSend:
        status = I2C_DRV_MasterGetSendStatus(I2C_CHANNEL0, &bytes);
        break;
    }

    return (int32_t)bytes;
}

int32_t KSDK_I2C0_Control(uint32_t control, uint32_t arg)
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
            I2C0_State.lastBaudRate_kbps = 100;
            break;
        case ARM_I2C_BUS_SPEED_FAST:
            I2C0_State.lastBaudRate_kbps = 400;
            break;
        case ARM_I2C_BUS_SPEED_FAST_PLUS:
            I2C0_State.lastBaudRate_kbps = 1000;
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
        if (kStatus_I2C_Success == I2C_DRV_MasterAbortSendData(I2C_CHANNEL0))
        {
            return ARM_DRIVER_OK;
        }
        else
        {
            return ARM_DRIVER_ERROR;
        }
        break;

    default:
        return ARM_DRIVER_ERROR_UNSUPPORTED;
    }

    return ARM_DRIVER_OK;
}

ARM_I2C_STATUS KSDK_I2C0_GetStatus(void)
{
    ARM_I2C_STATUS status;
    I2C_Type * base = g_i2cBase[I2C_CHANNEL0];

    // Get the I2C busy status
    status.busy = I2C_HAL_GetStatusFlag(base, kI2CBusBusy);

    // Default to Master mode
    status.mode = I2C_IS_MASTER;

    // Access the channel base address and get the current transfer direction
    status.direction = I2C_HAL_GetDirMode(base);

    // Not implemented
    status.general_call = 0;

    // Get the I2C arbitration lost status
    status.arbitration_lost = I2C_HAL_GetStatusFlag(base, kI2CArbitrationLost);

    // Not implemented
    status.bus_error = 0;

    return status;
}

void KSDK_I2C0_SignalEvent(uint32_t event)
{
    // function body
}

ARM_DRIVER_I2C Driver_I2C0_KSDK = {
    KSDK_GetVersion,
    KSDK_GetCapabilities,
    KSDK_I2C0_Initialize,
    KSDK_I2C0_Uninitialize,
    KSDK_I2C0_PowerControl,
    KSDK_I2C0_MasterTransmitBlocking,
    KSDK_I2C0_MasterReceiveBlocking,
    KSDK_SlaveTransmit,
    KSDK_SlaveReceive,
    KSDK_I2C0_GetDataCount,
    KSDK_I2C0_Control,
    KSDK_I2C0_GetStatus
};

/*******************************************************************************
* Functions - I2C1
******************************************************************************/
int32_t KSDK_I2C1_Initialize(ARM_I2C_SignalEvent_t cb_event)
{
    // Initialize the I2C channel.
    i2c_status_t status = I2C_DRV_MasterInit(I2C_CHANNEL1, &I2C1_State);

    // Doesn't implement the cb_event.

    // Return the proper status.
    if (kStatus_I2C_Success == status)
    {
        return ARM_DRIVER_OK;
    }
    else
    {
        return ARM_DRIVER_ERROR;
    }
}

int32_t KSDK_I2C1_Uninitialize(void)
{
    // De-initialize the I2C channel and return the proper status.
    if (kStatus_I2C_Success == I2C_DRV_MasterDeinit(I2C_CHANNEL1))
    {
        return ARM_DRIVER_OK;
    }
    else
    {
        return ARM_DRIVER_ERROR;
    }
}

int32_t KSDK_I2C1_PowerControl(ARM_POWER_STATE state)
{
    switch (state)
    {
    case ARM_POWER_OFF:
        // De-initialize the I2C channel.
        I2C_DRV_MasterDeinit(I2C_CHANNEL1);
        break;

    case ARM_POWER_LOW:
        break;

    case ARM_POWER_FULL:
        // Initialize the I2C channel.
        I2C_DRV_MasterInit(I2C_CHANNEL1, &I2C1_State);
        break;

    default:
        return ARM_DRIVER_ERROR_UNSUPPORTED;
    }

    return ARM_DRIVER_OK;
}

int32_t KSDK_I2C1_MasterTransmitBlocking(uint32_t addr, const uint8_t *data, uint32_t num, bool xfer_pending)
{
    // Set up the device information
    Device.address = addr;
    Device.baudRate_kbps = I2C1_State.lastBaudRate_kbps;

    // Send the data to the device and return the proper status
    if (kStatus_I2C_Success == I2C_DRV_MasterSendDataBlocking(I2C_CHANNEL1,&Device,&data[0],1,&data[1],num-1,I2C_TIMEOUT))
    {
        return ARM_DRIVER_OK;
    }
    else
    {
        return ARM_DRIVER_ERROR;
    }
}

int32_t KSDK_I2C1_MasterReceiveBlocking(uint32_t addr, uint8_t *data, uint32_t num, bool xfer_pending)
{
    // Set up the device information
    Device.address = addr;
    Device.baudRate_kbps = I2C1_State.lastBaudRate_kbps;

    if (kStatus_I2C_Success == I2C_DRV_MasterReceiveDataBlocking(I2C_CHANNEL1,&Device,&data[0],1,&data[1],num-1,I2C_TIMEOUT))
    {
        return ARM_DRIVER_OK;
    }
    else
    {
        return ARM_DRIVER_ERROR;
    }
}

int32_t KSDK_I2C1_GetDataCount(void)
{
    int32_t status;
    uint32_t bytes;

    // Access the channel base address and get the current transfer direction
    I2C_Type * base = g_i2cBase[I2C_CHANNEL1];
    i2c_direction_t direction = I2C_HAL_GetDirMode(base);

    // Use the direction to get the remaining byte count
    switch (direction)
    {
    case kI2CReceive:
        status = I2C_DRV_MasterGetReceiveStatus(I2C_CHANNEL1,&bytes);
        break;
    case kI2CSend:
        status = I2C_DRV_MasterGetSendStatus(I2C_CHANNEL1, &bytes);
        break;
    }

    return (int32_t)bytes;
}

int32_t KSDK_I2C1_Control(uint32_t control, uint32_t arg)
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
            I2C1_State.lastBaudRate_kbps = 100;
            break;
        case ARM_I2C_BUS_SPEED_FAST:
            I2C1_State.lastBaudRate_kbps = 400;
            break;
        case ARM_I2C_BUS_SPEED_FAST_PLUS:
            I2C1_State.lastBaudRate_kbps = 1000;
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
        if (kStatus_I2C_Success == I2C_DRV_MasterAbortSendData(I2C_CHANNEL1))
        {
            return ARM_DRIVER_OK;
        }
        else
        {
            return ARM_DRIVER_ERROR;
        }
        break;

    default:
        return ARM_DRIVER_ERROR_UNSUPPORTED;
    }

    return ARM_DRIVER_OK;
}

ARM_I2C_STATUS KSDK_I2C1_GetStatus(void)
{
    ARM_I2C_STATUS status;
    I2C_Type * base = g_i2cBase[I2C_CHANNEL1];

    // Get the I2C busy status
    status.busy = I2C_HAL_GetStatusFlag(base, kI2CBusBusy);

    // Default to Master mode
    status.mode = I2C_IS_MASTER;

    // Access the channel base address and get the current transfer direction
    status.direction = I2C_HAL_GetDirMode(base);

    // Not implemented
    status.general_call = 0;

    // Get the I2C arbitration lost status
    status.arbitration_lost = I2C_HAL_GetStatusFlag(base, kI2CArbitrationLost);

    // Not implemented
    status.bus_error = 0;

    return status;
}

void KSDK_I2C1_SignalEvent(uint32_t event)
{
    // function body
}

ARM_DRIVER_I2C Driver_I2C1_KSDK = {
    KSDK_GetVersion,
    KSDK_GetCapabilities,
    KSDK_I2C1_Initialize,
    KSDK_I2C1_Uninitialize,
    KSDK_I2C1_PowerControl,
    KSDK_I2C1_MasterTransmitBlocking,
    KSDK_I2C1_MasterReceiveBlocking,
    KSDK_SlaveTransmit,
    KSDK_SlaveReceive,
    KSDK_I2C1_GetDataCount,
    KSDK_I2C1_Control,
    KSDK_I2C1_GetStatus
};


