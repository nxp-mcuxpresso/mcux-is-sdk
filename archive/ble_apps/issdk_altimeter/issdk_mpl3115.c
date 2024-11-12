/*
 * Copyright (c) 2015 - 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file issdk_mpl3115.c
 * @brief The issdk_mpl3115.c file implements the ISSDK MPL3115 sensor driver
 *        example demonstration using wireless uart adapter.
 */

/* Standard C Includes */
#include <stdio.h>

/* CMSIS Includes */
#include "Driver_I2C.h"

/* ISSDK Includes */
#include "issdk_hal.h"
#include "mpl3115_drv.h"

/* ISSDK Interface Include */
#include "issdk_interface.h"

/*******************************************************************************
 * Macro Definitions
 ******************************************************************************/
#define MPL3115_DATA_SIZE (5) /* 3 byte Pressure/Altitude + 2 Byte Temperature. */
/*! In MPL3115 the Auto Acquisition Time Step (ODR) can be set only in powers of 2 (i.e. 2^x, where x is the
 *  SAMPLING_EXPONENT).
 *  This gives a range of 1 second to 2^15 seconds (9 hours). */
#define MPL3115_SAMPLING_EXPONENT (0) /* 1 second */

/*******************************************************************************
 * Constants
 ******************************************************************************/
/*! @brief Register settings for Normal (non buffered) mode. */
const registerwritelist_t cMpl3115ConfigNormal[] = {
    /* Enable Altitude output and set Over Sampling Ratio to 128. */
    {MPL3115_CTRL_REG1, MPL3115_CTRL_REG1_ALT_ALT | MPL3115_CTRL_REG1_OS_OSR_128,
     MPL3115_CTRL_REG1_ALT_MASK | MPL3115_CTRL_REG1_OS_MASK},
    /* Set Auto acquisition time step. */
    {MPL3115_CTRL_REG2, MPL3115_SAMPLING_EXPONENT, MPL3115_CTRL_REG2_ST_MASK},
    __END_WRITE_DATA__};

/*! @brief Address and size of Raw Temperature Data in Normal Mode. */
const registerreadlist_t cMpl3115OutputNormal[] = {{.readFrom = MPL3115_OUT_P_MSB, .numBytes = MPL3115_DATA_SIZE},
                                                   __END_READ_DATA__};

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
ARM_DRIVER_I2C *I2Cdrv = &I2C_S_DRIVER;
mpl3115_i2c_sensorhandle_t mpl3115Driver;

/************************************************************************************
* Functions
************************************************************************************/
static void getStringOfFloat(float f, char *s)
{
    int i = (int)f;
    int d = (int)(f*10.0);

    d = (f > 0.0)?(d-(i*10)):((i*10)-d);
    if(f<0.0&&f>-1.0)
    {
        sprintf(s, "-%d.%d", i, d);
    }
    else
    {
        sprintf(s, "%d.%d", i, d);
    }
}

void ISSDK_ReadAltitide(void)
{
    int32_t status;
    float tempC, altM;
    char tempCs[8]={0}, altMs[8]={0};
    mpl3115_altitudedata_t rawData;
    uint8_t data[MPL3115_DATA_SIZE];

    /*! Read the raw sensor data from the fxos8700. */
    status = MPL3115_I2C_ReadData(&mpl3115Driver, cMpl3115OutputNormal, data);
    if (ARM_DRIVER_OK != status)
    {
        BleApp_WriteToUART("\r\n Read Failed. \r\n");
        return;
    }

    /*! Convert the raw sensor data to signed 16-bit container for display. */
    rawData.altitude = (int32_t)((data[0]) << 24) | ((data[1]) << 16) | ((data[2]) << 8);
    rawData.temperature = (int16_t)((data[3]) << 8) | (data[4]);

    altM = rawData.altitude / MPL3115_ALTITUDE_CONV_FACTOR;
    tempC = rawData.temperature / MPL3115_TEMPERATURE_CONV_FACTOR;

    getStringOfFloat(altM, altMs);
    getStringOfFloat(tempC, tempCs);

    /* Send data over the air */
    BleApp_WriteToHost("ALTITUDE: %sm\nTEMPERATURE: %sC\n\n", altMs, tempCs);
}

void ISSDK_ReadPressure(void)
{
    int32_t status;
    float tempC, presP;
    char tempCs[8]={0}, presPs[8]={0};
    mpl3115_pressuredata_t rawData;
    uint8_t data[MPL3115_DATA_SIZE];

    /*! Read the raw sensor data from the fxos8700. */
    status = MPL3115_I2C_ReadData(&mpl3115Driver, cMpl3115OutputNormal, data);
    if (ARM_DRIVER_OK != status)
    {
        BleApp_WriteToUART("\r\n Read Failed. \r\n");
        return;
    }

    /*! Convert the raw sensor data to signed 16-bit container for display. */
    rawData.pressure = (uint32_t)((data[0]) << 16) | ((data[1]) << 8) | ((data[2]));
    rawData.temperature = (int16_t)((data[3]) << 8) | (data[4]);

    presP = rawData.pressure / (MPL3115_PRESSURE_CONV_FACTOR*1000);
    tempC = rawData.temperature / MPL3115_TEMPERATURE_CONV_FACTOR;

    getStringOfFloat(presP, presPs);
    getStringOfFloat(tempC, tempCs);

    /* Send data over the air */
    BleApp_WriteToHost("PRESSURE: %skPa\nTEMPERATURE: %sC\n\n", presPs, tempCs);
}

void ISSDK_InitSensor(void)
{
    int32_t status;

    BleApp_WriteToUART("\r\n ISSDK MPL3115 BLE example.\r\n");

    /*! Initialize the I2C driver. */
    status = I2Cdrv->Initialize(I2C_BB_SIGNAL_EVENT);
    if (ARM_DRIVER_OK != status)
    {
        BleApp_WriteToUART("\r\n I2C Initialization Failed\r\n");
        return;
    }

    /*! Set the I2C Power mode. */
    status = I2Cdrv->PowerControl(ARM_POWER_FULL);
    if (ARM_DRIVER_OK != status)
    {
        BleApp_WriteToUART("\r\n I2C Power Mode setting Failed\r\n");
        return;
    }

    /*! Set the I2C bus speed. */
    status = I2Cdrv->Control(ARM_I2C_BUS_SPEED, ARM_I2C_BUS_SPEED_FAST);
    if (ARM_DRIVER_OK != status)
    {
        BleApp_WriteToUART("\r\n I2C Control Mode setting Failed\r\n");
        return;
    }

    /*! Initialize the MPL3115 sensor driver. */
    status = MPL3115_I2C_Initialize(&mpl3115Driver, &I2C_S_DRIVER, I2C_S_DEVICE_INDEX, MPL3115_I2C_ADDR,
                                    MPL3115_WHOAMI_VALUE);
    if (SENSOR_ERROR_NONE != status)
    {
        BleApp_WriteToUART("\r\n Sensor Initialization Failed\r\n");
        return;
    }
    BleApp_WriteToUART("\r\n Successfully Initialized Sensor.\r\n");

    /* Apply MPL3115 Configuration based on the Register List */
    status = Sensor_I2C_Write(mpl3115Driver.pCommDrv, &mpl3115Driver.deviceInfo, mpl3115Driver.slaveAddress, cMpl3115ConfigNormal);
    if (ARM_DRIVER_OK != status)
    {
        BleApp_WriteToUART("\r\n Sensor Configuration Failed, Err = %d\r\n", status);
        return;
    }
    BleApp_WriteToUART("\r\n Successfully Configured Sensor.\r\n");
}

void ISSDK_ConfigureSensor(uint32_t arg)
{
    int32_t status;

    switch(arg)
    {
        case ISSDK_SENSOR_START:
            /* Put the device into active mode.*/
            status = Register_I2C_Write(mpl3115Driver.pCommDrv, &mpl3115Driver.deviceInfo, mpl3115Driver.slaveAddress,
                               MPL3115_CTRL_REG1, MPL3115_CTRL_REG1_SBYB_ACTIVE, MPL3115_CTRL_REG1_SBYB_MASK, false);
            if (ARM_DRIVER_OK != status)
            {
                BleApp_WriteToUART("\r\n Failed to put sensor to Active Mode.\r\n");
            }
            else
            {
                BleApp_WriteToUART("\r\n Successfully put Sensor to Active Mode.\r\n");
            }
            break;
        case ISSDK_SENSOR_STOP:
            /* Put the device into standby mode.*/
            status = Register_I2C_Write(mpl3115Driver.pCommDrv, &mpl3115Driver.deviceInfo, mpl3115Driver.slaveAddress,
                               MPL3115_CTRL_REG1, MPL3115_CTRL_REG1_SBYB_STANDBY, MPL3115_CTRL_REG1_SBYB_MASK, false);
            if (ARM_DRIVER_OK != status)
            {
                BleApp_WriteToUART("\r\n Failed to put sensor to Standby Mode.\r\n");
            }
            else
            {
                BleApp_WriteToUART("\r\n Successfully put Sensor to Standby Mode.\r\n");
            }
            break;
        case ISSDK_SENSOR_ALTITUDE_MODE:
            /* Put the device into Altitude mode.*/
            status = Register_I2C_Write(mpl3115Driver.pCommDrv, &mpl3115Driver.deviceInfo, mpl3115Driver.slaveAddress,
                               MPL3115_CTRL_REG1, MPL3115_CTRL_REG1_ALT_ALT, MPL3115_CTRL_REG1_ALT_MASK, false);
            if (ARM_DRIVER_OK != status)
            {
                BleApp_WriteToUART("\r\n Failed to put sensor to Active Mode.\r\n");
            }
            else
            {
                BleApp_WriteToUART("\r\n Successfully put Sensor to Active Mode.\r\n");
            }
            break;
        case ISSDK_SENSOR_PRESSURE_MODE:
            /* Put the device into Pressure mode.*/
            status = Register_I2C_Write(mpl3115Driver.pCommDrv, &mpl3115Driver.deviceInfo, mpl3115Driver.slaveAddress,
                               MPL3115_CTRL_REG1, MPL3115_CTRL_REG1_ALT_BAR, MPL3115_CTRL_REG1_ALT_MASK, false);
            if (ARM_DRIVER_OK != status)
            {
                BleApp_WriteToUART("\r\n Failed to put sensor to Active Mode.\r\n");
            }
            else
            {
                BleApp_WriteToUART("\r\n Successfully put Sensor to Active Mode.\r\n");
            }
            break;
        default:
            break;
    }
}