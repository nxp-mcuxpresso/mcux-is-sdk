/*
 * Copyright (c) 2015 - 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*! File: FXOS8700_1.c
* @brief The \b FXOS8700_1.c file implements Interface Wrappers for the FXOS8700 driver interfaces.
*/

#include "FXOS8700_1.h"
#include "fxos8700_drv.h"
#include "sensor_io.h" 

fxos8700_i2c_sensorhandle_t Sensor_FXOS8700_1 =
{
    FXOS8700_1_GetCapabilities,
    FXOS8700_1_Initialize,
    FXOS8700_1_Configure,
    FXOS8700_1_StartData,
    FXOS8700_1_ReadData,
    FXOS8700_1_EndData,
    FXOS8700_1_Calibrate,
    FXOS8700_1_Shutdown,
    FXOS8700_1_PowerControl,
    FXOS8700_1_GetStatus,
};
/*******************************************************************************
 * Code
 ******************************************************************************/
/*Interface Wrappers for the FXOS8700. */
inline int32_t FXOS8700_1_GetCapabilities()
{
    return FXOS8700_I2C_GetCapabilities(&Sensor_FXOS8700_1);
}
int32_t FXOS8700_1_Initialize(ARM_DRIVER_I2C * pCommDrv, uint16_t slaveAddr)
{
    return FXOS8700_I2C_Initialize(&Sensor_FXOS8700_1, pCommDrv, slaveAddr );
}
int32_t FXOS8700_1_Configure(const registerwritelist_t *pSensorSettings)
{
    return FXOS8700_I2C_Configure(&Sensor_FXOS8700_1, pSensorSettings);
}
int32_t FXOS8700_1_StartData()
{
    return FXOS8700_I2C_StartData(&Sensor_FXOS8700_1);
}
int32_t FXOS8700_1_ReadData(const registerreadlist_t *readList, uint8_t* pReadBuffer)
{
    return FXOS8700_I2C_ReadData(&Sensor_FXOS8700_1, readList, pReadBuffer);
}
int32_t FXOS8700_1_EndData()
{
    return FXOS8700_I2C_EndData(&Sensor_FXOS8700_1);
}
int32_t FXOS8700_1_Calibrate( mma845x_i2c_caldata_t *calData)
{
    return FXOS8700_I2C_Calibrate(&Sensor_FXOS8700_1, calData);
}
int32_t FXOS8700_1_Shutdown()
{
    return FXOS8700_I2C_Shutdown(&Sensor_FXOS8700_1);
}
int32_t FXOS8700_1_PowerControl(void *powerState )
{
    return FXOS8700_I2C_PowerControl(&Sensor_FXOS8700_1,powerState);
}
int32_t FXOS8700_1_GetStatus()
{
    return FXOS8700_I2C_GetStatus(&Sensor_FXOS8700_1);
}

