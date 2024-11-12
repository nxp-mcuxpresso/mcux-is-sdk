/*
 * Copyright (c) 2015-2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*! File: fxls8952_drv.h
* @brief The fxls8952_drv.h file describes the fxls8952 driver interface and structures.
*/

#ifndef FXLS8952_FI_H_
#define FXLS8952_FI_H_

//-----------------------------------------------------------------------
// Standard C Includes
//-----------------------------------------------------------------------
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

//-----------------------------------------------------------------------
// CMSIS Includes
//-----------------------------------------------------------------------
#include "Driver_I2C_SDK2.h"

//-----------------------------------------------------------------------
// ISSDK Includes
//-----------------------------------------------------------------------
#include "sensor_drv.h"
#include "sensor_io_i2c.h"
#include "fxls8952.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/*! @brief This defines the function pointers and sensor specific information. */
typedef struct
{
    ARM_DRIVER_I2C *pCommDrv; /*!< Pointer to the i2c driver. */
    uint16_t slaveAddress;    /*!< slave address.*/
    bool isInitialized;
} fxls8952_i2c_sensorhandle_t;

/*! @brief This structure defines the fxls8952 raw data buffer.*/
typedef struct
{
    uint32_t timestamp; /*! The time, this sample was recorded.  */
    int16_t accel[3];   /*!< The accel data */
} fxls8952_acceldata_t;

/*******************************************************************************
 * APIs
 ******************************************************************************/
/*! @brief       The interface function to initialize the sensor.
 *  @details     This function initialize the sensor and sensor handle.
 *  @param[in]   pSensorHandle    handle to the sensor.
 *  @param[in]   pBus             pointer to I2C bus.
 *  @param[in]   sAddress         slave address
 *  @param[in]   whoAmi           whoAMI value of the sensor
 *  @return      ::fxls8952_Initialize() returns the status.
 *  @constraints This should be the first function to be invoked or re-invoked only after FXLS8952_I2C_DeInit().
 *  @reeentrant  Yes
 */
int32_t FXLS8952_I2C_Initialize(fxls8952_i2c_sensorhandle_t *pSensorHandle,
                                ARM_DRIVER_I2C *pBus,
                                uint16_t sAddress,
                                uint8_t whoAmi);

/*! @brief       The interface function to configure he sensor.
 *  @details     This function configure the sensor with requested ODR, Range and registers in the regsiter pair array.
 *  @param[in]   pSensorHandle      handle to the sensor.
 *  @param[in]   pRegWriteList      pointer to the register list.
 *  @return      ::fxls8952_Configure() returns the status.
 *  @constraints This can be called any number of times only after FXLS8952_I2C_Initialize().
 *  @reeentrant  No
 */
int32_t FXLS8952_I2C_Configure(fxls8952_i2c_sensorhandle_t *pSensorHandle, const registerwritelist_t *pRegWriteList);

/*! @brief       The interface function to read the sensor data.
 *  @details     This function read the sensor data out from the device and returns raw data in a byte stream.
 *  @param[in]   pSensorHandle      handle to the sensor.
 *  @param[out]  pBuffer            buffer it holds raw sensor data.This buffer may be back to back databuffer based
 * command read in the list.
 *  @return      ::fxls8952_ReadData() returns the status.
 *  @constraints This can be called only after FXLS8952_I2C_Initialize().
 *  @reeentrant  Yes
 */
int32_t FXLS8952_I2C_ReadData(fxls8952_i2c_sensorhandle_t *pSensorHandle,
                              const registerreadlist_t *pReadList,
                              uint8_t *pBuffer);

/*! @brief       The interface function to stop the sensor..
 *  @details     This function made sensor into standby mode and in a power safe state.
 *  @param[in]   pSensorHandle      handle to the sensor.
 *  @return      ::FXLS8952_I2C_DeInit() returns the status.
 *  @constraints This can be called only if FXLS8952_I2C_Initialize has been called earlier.
 *  @reeentrant  Yes
 */
int32_t FXLS8952_I2C_DeInit(fxls8952_i2c_sensorhandle_t *pSensorHandle);

#endif // FXLS8952_FI_H_
