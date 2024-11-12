/*
 * Copyright (c) 2015 - 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*! File: MMA845x_1.c
* @brief The \b MMA845x_1.c file implements Interface Wrappers for the MMA845x driver interfaces.
*/

#include "MMA845x_1.h"
#include "mma845x_drv.h"
#include "register_io.h"
#include "sensor_drv.h"

// declare the sensor instance.
mma845x_i2c_sensorhandle_t Sensor_MMA845x_1 =
{
    MMA845x_1_GetCapabilities,
    MMA845x_1_Initialize,
    MMA845x_1_Configure,
    MMA845x_1_StartData,
    MMA845x_1_ReadData,
    MMA845x_1_EndData,
    MMA845x_1_Calibrate,
    MMA845x_1_Shutdown,
    MMA845x_1_PowerControl,
    MMA845x_1_GetStatus,
};

/*******************************************************************************
 * Code
 ******************************************************************************/
/*Interface Wrappers for the MMA845x. */
inline int32_t MMA845x_1_GetCapabilities()
{
    return MMA845x_I2C_GetCapabilities(&Sensor_MMA845x_1);
}
inline int32_t MMA845x_1_Initialize(ARM_DRIVER_I2C * pCommDrv, uint16_t slaveAddr)
{
    return MMA845x_I2C_Initialize(&Sensor_MMA845x_1, pCommDrv, slaveAddr );
}
inline int32_t MMA845x_1_Configure(const registerwritelist_t *pSensorSettings)
{
    return MMA845x_I2C_Configure(&Sensor_MMA845x_1, pSensorSettings);
}
inline int32_t MMA845x_1_StartData()
{
    return MMA845x_I2C_StartData(&Sensor_MMA845x_1);
}
inline int32_t MMA845x_1_ReadData(const registerreadlist_t *readList, uint8_t* pReadBuffer)
{
    return MMA845x_I2C_ReadData(&Sensor_MMA845x_1, readList, pReadBuffer);
}
inline int32_t MMA845x_1_EndData()
{
    return MMA845x_I2C_EndData(&Sensor_MMA845x_1);
}
inline int32_t MMA845x_1_Calibrate( mma845x_i2c_caldata_t *calData)
{
    return MMA845x_I2C_Calibrate(&Sensor_MMA845x_1, calData);
}
inline int32_t MMA845x_1_Shutdown()
{
    return MMA845x_I2C_Shutdown(&Sensor_MMA845x_1);
}
inline int32_t MMA845x_1_PowerControl(void *powerState )
{
    return MMA845x_I2C_PowerControl(&Sensor_MMA845x_1,powerState);
}
inline int32_t MMA845x_1_GetStatus()
{
    return MMA845x_I2C_GetStatus(&Sensor_MMA845x_1);
}

