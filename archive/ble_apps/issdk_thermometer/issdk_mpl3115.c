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
#define MPL3115_DATA_SIZE (2) /* 2 byte Temperature. */
/*! In MPL3115 the Auto Acquisition Time Step (ODR) can be set only in powers of 2 (i.e. 2^x, where x is the
 *  SAMPLING_EXPONENT).
 *  This gives a range of 1 second to 2^15 seconds (9 hours). */
#define MPL3115_SAMPLING_EXPONENT (0) /* 1 second */

/*******************************************************************************
 * Constants
 ******************************************************************************/
/*! @brief Register settings for Normal (non buffered) mode. */
const registerwritelist_t cMpl3115ConfigNormal[] = {
    /* Set Over Sampling Ratio to 128. */
    {MPL3115_CTRL_REG1, MPL3115_CTRL_REG1_OS_OSR_128, MPL3115_CTRL_REG1_OS_MASK},
    /* Set Auto acquisition time step. */
    {MPL3115_CTRL_REG2, MPL3115_SAMPLING_EXPONENT, MPL3115_CTRL_REG2_ST_MASK},
    __END_WRITE_DATA__};

/*! @brief Address and size of Raw Temperature Data in Normal Mode. */
const registerreadlist_t cMpl3115OutputNormal[] = {{.readFrom = MPL3115_OUT_T_MSB, .numBytes = MPL3115_DATA_SIZE},
                                                   __END_READ_DATA__};

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
ARM_DRIVER_I2C *I2Cdrv = &I2C_S_DRIVER;
mpl3115_i2c_sensorhandle_t mpl3115Driver;

/************************************************************************************
* Functions
************************************************************************************/
void ISSDK_ReadSensor(int16_t *tempInDegrees)
{
    int32_t status;
    mpl3115_pressuredata_t rawData;
    uint8_t data[MPL3115_DATA_SIZE];

    /*! Read the raw sensor data from the fxos8700. */
    status = MPL3115_I2C_ReadData(&mpl3115Driver, cMpl3115OutputNormal, data);
    if (ARM_DRIVER_OK != status)
    {
        return;
    }

    rawData.temperature = (int16_t)((data[0]) << 8) | (data[1]);
    *tempInDegrees = rawData.temperature / MPL3115_TEMPERATURE_CONV_FACTOR;
}

void ISSDK_InitSensor(void)
{
    int32_t status;

    /*! Initialize the I2C driver. */
    status = I2Cdrv->Initialize(I2C_BB_SIGNAL_EVENT);
    if (ARM_DRIVER_OK != status)
    {
        return;
    }

    /*! Set the I2C Power mode. */
    status = I2Cdrv->PowerControl(ARM_POWER_FULL);
    if (ARM_DRIVER_OK != status)
    {
        return;
    }

    /*! Set the I2C bus speed. */
    status = I2Cdrv->Control(ARM_I2C_BUS_SPEED, ARM_I2C_BUS_SPEED_FAST);
    if (ARM_DRIVER_OK != status)
    {
        return;
    }

    /*! Initialize the MPL3115 sensor driver. */
    status = MPL3115_I2C_Initialize(&mpl3115Driver, &I2C_S_DRIVER, I2C_S_DEVICE_INDEX, MPL3115_I2C_ADDR,
                                    MPL3115_WHOAMI_VALUE);
    if (SENSOR_ERROR_NONE != status)
    {
        return;
    }

    /* Appy MPL3115 Configuration based on the Register List */
    status = Sensor_I2C_Write(mpl3115Driver.pCommDrv, &mpl3115Driver.deviceInfo, mpl3115Driver.slaveAddress, cMpl3115ConfigNormal);
    if (ARM_DRIVER_OK != status)
    {
        return;
    }
}

void ISSDK_ConfigureSensor(uint32_t arg)
{
    switch(arg)
    {
        case ISSDK_START_SENSOR:
            /* Put the device into active mode.*/
            Register_I2C_Write(mpl3115Driver.pCommDrv, &mpl3115Driver.deviceInfo, mpl3115Driver.slaveAddress,
                               MPL3115_CTRL_REG1, MPL3115_CTRL_REG1_SBYB_ACTIVE, MPL3115_CTRL_REG1_SBYB_MASK, false);
            break;
        case ISSDK_STOP_SENSOR:
            /* Put the device into standby mode.*/
            Register_I2C_Write(mpl3115Driver.pCommDrv, &mpl3115Driver.deviceInfo, mpl3115Driver.slaveAddress,
                               MPL3115_CTRL_REG1, MPL3115_CTRL_REG1_SBYB_STANDBY, MPL3115_CTRL_REG1_SBYB_MASK, false);
            break;
        default:
            break;
    }
}