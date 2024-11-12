/*
 * Copyright (c) 2015 - 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*! File: MPL3115_1.c
* @brief The \b MPL3115_1.c file implements Interface Wrappers for the MPL3115 driver interfaces.
*/

#include "MPL3115_1.h"
#include "mpl3115_drv.h"
#include "sensor_drv.h"

// declare the sensor instance.
mpl3115_i2c_sensorHandle_t Sensor_MPL3115_1 =
{
    MPL3115_1_GetCapabilities,
    MPL3115_1_Initialize,
    MPL3115_1_Configure,
    MPL3115_1_StartData,
    MPL3115_1_ReadData,
    MPL3115_1_EndData,
    MPL3115_1_Calibrate,
    MPL3115_1_Shutdown,
    MPL3115_1_PowerControl,
    MPL3115_1_GetStatus,
};
/*******************************************************************************
 * Code
 ******************************************************************************/
/*Interface Wrappers for the MPL3115. */
inline int32_t MPL3115_1_GetCapabilities()
{
    return mpl3115_i2c_GetCapabilities(&Sensor_MPL3115_1);
}
inline int32_t MPL3115_1_Initialize(ARM_DRIVER_I2C * pCommDrv, uint16_t slaveAddr)
{
    return mpl3115_i2c_Initialize(&Sensor_MPL3115_1, pCommDrv, slaveAddr );
}
inline int32_t MPL3115_1_Configure(const RegisterWriteList_t *pSensorSettings)
{
    return mpl3115_i2c_Configure(&Sensor_MPL3115_1, pSensorSettings);
}
inline int32_t MPL3115_1_StartData()
{
    return mpl3115_i2c_StartData(&Sensor_MPL3115_1);
}
inline int32_t MPL3115_1_ReadData(const RegisterReadList_t *readList, uint8_t* pReadBuffer)
{
    return mpl3115_i2c_ReadData(&Sensor_MPL3115_1, readList, pReadBuffer);
}
inline int32_t MPL3115_1_EndData()
{
    return mpl3115_i2c_EndData(&Sensor_MPL3115_1);
}
inline int32_t MPL3115_1_Calibrate( mpl3115_i2c_calData_t *calData)
{
    return mpl3115_i2c_Calibrate(&Sensor_MPL3115_1, calData);
}
inline int32_t MPL3115_1_Shutdown()
{
    return mpl3115_i2c_Shutdown(&Sensor_MPL3115_1);
}
inline int32_t MPL3115_1_PowerControl(void *powerState )
{
    return mpl3115_i2c_PowerControl(&Sensor_MPL3115_1,powerState);
}
inline int32_t MPL3115_1_GetStatus()
{
    return mpl3115_i2c_GetStatus(&Sensor_MPL3115_1);
}

