/*
 * Copyright (c) 2015-2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */
#include "FXLS8952_1.h"
#include "fxls8952_drv.h"
#include "sensor_drv.h"

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
/* Read commands for accel data.*/
const RegisterReadList_t FXLS8952_ACCEL_DATA[] = {
    {
        .readFrom = FXLS8952_OUT_X_LSB,
        .numBytes = 6
    },
    __END_READ_DATA__
};
/* Read Commands for accel status and followed by data.*/
const RegisterReadList_t FXLS8952_ACCEL_STATUS_DATA[] = {
    {
        .readFrom = FXLS8952_INT_STATUS,
        .numBytes = 1
    },  
    {
        .readFrom = FXLS8952_OUT_X_LSB,
        .numBytes = 6
    },
    __END_READ_DATA__
};
/* FXLS8952 instance.*/
fxls8952_i2c_sensorHandle_t Sensor_FXLS8952_1 = 
{
    FXLS8952_1_GetCapabilities,
    FXLS8952_1_Initialize,
    FXLS8952_1_Configure,
    FXLS8952_1_StartData,
    FXLS8952_1_ReadData,
    FXLS8952_1_EndData,
    FXLS8952_1_Calibrate,
    FXLS8952_1_Shutdown,
    FXLS8952_1_PowerControl,
    FXLS8952_1_GetStatus,
};   
/*******************************************************************************
 * Code
 ******************************************************************************/
/*Interface Wrappers for the FXLS8952. */
inline int32_t FXLS8952_1_GetCapabilities()
{
    return fxls8952_i2c_GetCapabilities(&Sensor_FXLS8952_1);
}
int32_t FXLS8952_1_Initialize(ARM_DRIVER_I2C * pCommDrv, uint16_t slaveAddr)
{
    return fxls8952_i2c_Initialize(&Sensor_FXLS8952_1, pCommDrv, slaveAddr );
}
int32_t FXLS8952_1_Configure(const RegisterWriteList_t *pSensorSettings)  
{
    return fxls8952_i2c_Configure(&Sensor_FXLS8952_1, pSensorSettings);
}
int32_t FXLS8952_1_StartData()
{
    return fxls8952_i2c_StartData(&Sensor_FXLS8952_1);
}
int32_t FXLS8952_1_ReadData(const RegisterReadList_t *readList, uint8_t* pReadBuffer)
{
    return fxls8952_i2c_ReadData(&Sensor_FXLS8952_1, readList, pReadBuffer);
}
int32_t FXLS8952_1_EndData() 
{
    return fxls8952_i2c_EndData(&Sensor_FXLS8952_1);
}
int32_t FXLS8952_1_Calibrate( fxls8952_i2c_calData_t *calData)
{
    return fxls8952_i2c_Calibrate(&Sensor_FXLS8952_1, calData);
}
int32_t FXLS8952_1_Shutdown()
{
    return fxls8952_i2c_Shutdown(&Sensor_FXLS8952_1);
}
int32_t FXLS8952_1_PowerControl(void *powerState ) 
{
    return fxls8952_i2c_PowerControl(&Sensor_FXLS8952_1,powerState);
}
int32_t FXLS8952_1_GetStatus() 
{
    return fxls8952_i2c_GetStatus(&Sensor_FXLS8952_1);
}

