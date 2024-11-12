/*
 * Copyright (c) 2015 - 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*! File: fxas21002_1.c
* @brief The \b fxas21002_1.c file implements Interface Wrappers for the FXAS21002 driver interfaces.
*/

#include "FXAS21002_1.h"
#include "fxas21002_drv.h"
#include "register_io.h"
#include "sensor_drv.h"

/* declare the sensor instance */
fxas21002_i2c_sensorhandle_t Sensor_FXAS21002_1 =
{
    FXAS21002_1_GetCapabilities,
    FXAS21002_1_Initialize,
    FXAS21002_1_Configure,
    FXAS21002_1_StartData,
    FXAS21002_1_ReadData,
    FXAS21002_1_EndData,
    FXAS21002_1_Calibrate,
    FXAS21002_1_Shutdown,
    FXAS21002_1_PowerControl,
    FXAS21002_1_GetStatus,
};

/*******************************************************************************
 * Code
 ******************************************************************************/
/*Interface Wrappers for the FXAS21002. */
inline int32_t FXAS21002_1_GetCapabilities()
{
    return FXAS21002_I2C_GetCapabilities(&Sensor_FXAS21002_1);
}
inline int32_t FXAS21002_1_Initialize(ARM_DRIVER_I2C * pCommDrv, uint16_t slaveAddr)
{
    return FXAS21002_I2C_Initialize(&Sensor_FXAS21002_1, pCommDrv, slaveAddr );
}
inline int32_t FXAS21002_1_Configure(const registerwritelist_t *pSensorSettings)
{
    return FXAS21002_I2C_Configure(&Sensor_FXAS21002_1, pSensorSettings);
}
inline int32_t FXAS21002_1_StartData()
{
    return FXAS21002_I2C_StartData(&Sensor_FXAS21002_1);
}
inline int32_t FXAS21002_1_ReadData(const registerreadlist_t *readList, uint8_t* pReadBuffer)
{
    return FXAS21002_I2C_ReadData(&Sensor_FXAS21002_1, readList, pReadBuffer);
}
inline int32_t FXAS21002_1_EndData()
{
    return FXAS21002_I2C_EndData(&Sensor_FXAS21002_1);
}
inline int32_t FXAS21002_1_Calibrate( fxas21002_i2c_caldata_t *calData)
{
    return FXAS21002_I2C_Calibrate(&Sensor_FXAS21002_1, calData);
}
inline int32_t FXAS21002_1_Shutdown()
{
    return FXAS21002_I2C_Shutdown(&Sensor_FXAS21002_1);
}
inline int32_t FXAS21002_1_PowerControl(void *powerState )
{
    return FXAS21002_I2C_PowerControl(&Sensor_FXAS21002_1,powerState);
}
inline int32_t FXAS21002_1_GetStatus()
{
    return FXAS21002_I2C_GetStatus(&Sensor_FXAS21002_1);
}
