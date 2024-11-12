/*
 * Copyright (c) 2015-2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*! @file  fxls8952_drv.c
*   @brief The fxls8952_drv.c file implements the fxls8952 functional interface.
*/

//-----------------------------------------------------------------------
// ISSDK Includes
//-----------------------------------------------------------------------
#include "fxls8952_drv.h"
#include "register_io_i2c.h"

/*******************************************************************************
 * Functions
 ******************************************************************************/
/* ===================================================================
**     @brief      :  The interface function to initialize the sensor.
**     @param[in]  :  fxls8952_i2c_sensorhandle_t *pSensorHandle, handle to the sensor handle.
**     @param[in]  :  ARM_DRIVER_I2C* *pBus, pointer to the communication bus.
**     @param[in]  :  uint8_t whoAmi whoAMI value of the sensor.
**     @param[in]  :  uint16_t sAddress, slave address of the device.
**     @return        returns the status.
**     @constraints:  This should be the first function to be invoked or re-invoked only after FXLS8952_I2C_DeInit().
**     @reeentrant :  Yes
** =================================================================== */
int32_t FXLS8952_I2C_Initialize(fxls8952_i2c_sensorhandle_t *pSensorHandle,
                                ARM_DRIVER_I2C *pBus,
                                uint16_t sAddress,
                                uint8_t whoAmi)
{
    int32_t status;
    FXLS8952_WHO_AM_I_t reg;

    if ((pSensorHandle == NULL) || (pBus == NULL))
    {
        return SENSOR_ERROR_INVALID_PARAM;
    }

    /* Verify WhoAmI value. */
    status = Register_I2C_Read(pBus, sAddress, FXLS8952_WHO_AM_I, 1, (uint8_t *)&reg);
    if ((ARM_DRIVER_OK != status) || (whoAmi != reg))
    {
        pSensorHandle->isInitialized = false;
        return SENSOR_ERROR_INIT;
    }

    pSensorHandle->pCommDrv = pBus;
    pSensorHandle->slaveAddress = sAddress;
    pSensorHandle->isInitialized = true;

    return SENSOR_ERROR_NONE;
}

/* ===================================================================
**     @brief      :  The interface function to configure he sensor.
**     @param[in]  :  fxls8952_i2c_sensorhandle_t *pSensorHandle, handle to the sensor handle.
**     @param[in]  :  const registerwritelist_t* pRegWriteList, regsiter value pair array that contain register address
                      and corresponding value for that address.
**     @return        returns the status.
**     @constraints:  This can be called any number of times only after FXLS8952_I2C_Initialize().
**     @reeentrant :  No
** =================================================================== */
int32_t FXLS8952_I2C_Configure(fxls8952_i2c_sensorhandle_t *pSensorHandle, const registerwritelist_t *pRegWriteList)
{
    int32_t status;

    /* Validate for the correct handle.*/
    if ((pSensorHandle == NULL) || (pRegWriteList == NULL))
    {
        return SENSOR_ERROR_INVALID_PARAM;
    }

    if (pSensorHandle->isInitialized != true)
    {
        return SENSOR_ERROR_INIT;
    }

    /* Put the device into standby mode so that configuration can be applied. */
    status = Register_I2C_Write(pSensorHandle->pCommDrv, pSensorHandle->slaveAddress, FXLS8952_SENS_CONFIG1,
                                FXLS8952_SENS_CONFIG1_ACTIVE_DIS, FXLS8952_SENS_CONFIG1_ACTIVE_MASK, false);
    if (ARM_DRIVER_OK != status)
    {
        return SENSOR_ERROR_WRITE;
    }

    status = Sensor_I2C_Write(pSensorHandle->pCommDrv, pSensorHandle->slaveAddress, pRegWriteList);
    if (ARM_DRIVER_OK != status)
    {
        return SENSOR_ERROR_WRITE;
    }

    /* Put the device into active mode so that samples can be read. */
    status = Register_I2C_Write(pSensorHandle->pCommDrv, pSensorHandle->slaveAddress, FXLS8952_SENS_CONFIG1,
                                FXLS8952_SENS_CONFIG1_ACTIVE_EN, FXLS8952_SENS_CONFIG1_ACTIVE_MASK, false);
    if (ARM_DRIVER_OK != status)
    {
        return SENSOR_ERROR_WRITE;
    }

    return SENSOR_ERROR_NONE;
}

/* ===================================================================
**     @brief      :  The interface function to stop the sensor.
**     @param[in]  :  fxls8952_i2c_sensorhandle_t *pSensorHandle , handle to the sensor handle.
**     @return        returns the status.
**     @constraints:  This can be called only if FXLS8952_I2C_Initialize has been called earlier.
**     @reeentrant :  Yes
** =================================================================== */
int32_t FXLS8952_I2C_DeInit(fxls8952_i2c_sensorhandle_t *pSensorHandle)
{
    int32_t status;

    if (pSensorHandle == NULL)
    {
        return SENSOR_ERROR_INVALID_PARAM;
    }

    if (pSensorHandle->isInitialized != true)
    {
        return SENSOR_ERROR_INIT;
    }

    /* Reset the device to put it into default state. */
    status = Register_I2C_Write(pSensorHandle->pCommDrv, pSensorHandle->slaveAddress, FXLS8952_SENS_CONFIG1,
                                FXLS8952_SENS_CONFIG1_RST_RST, FXLS8952_SENS_CONFIG1_RST_MASK, false);
    if (ARM_DRIVER_OK != status)
    {
        return SENSOR_ERROR_WRITE;
    }
    else
    {
        pSensorHandle->isInitialized = false;
    }

    return SENSOR_ERROR_NONE;
}

/* ===================================================================
**     @brief      :  The interface function to read the sensor data.
**     @param[in]  :  fxls8952_i2c_sensorhandle_t *pSensorHandle , handle to the sensor handle.
**     @param[in]  :  const registerreadlist_t *pReadList, List of read command.
**     @param[out] :  uint8_t *pBuffer, return data, this data will be a back to back data based on the number of read.
**     @return        returns the status.
**     @constraints:  This can be called only after FXLS8952_I2C_Initialize().
**     @reeentrant :  Yes
** =================================================================== */
int32_t FXLS8952_I2C_ReadData(fxls8952_i2c_sensorhandle_t *pSensorHandle,
                              const registerreadlist_t *pReadList,
                              uint8_t *pBuffer)
{
    int32_t status;

    if ((pSensorHandle == NULL) || (pReadList == NULL) || (pBuffer == NULL))
    {
        return SENSOR_ERROR_INVALID_PARAM;
    }

    if (pSensorHandle->isInitialized != true)
    {
        return SENSOR_ERROR_INIT;
    }

    /* Parse through the read list and read the data one by one*/
    status = Sensor_I2C_Read(pSensorHandle->pCommDrv, pSensorHandle->slaveAddress, pReadList, pBuffer);
    if (ARM_DRIVER_OK != status)
    {
        return SENSOR_ERROR_READ;
    }

    return SENSOR_ERROR_NONE;
}
