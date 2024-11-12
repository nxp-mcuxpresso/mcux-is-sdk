/*
 * Copyright (c) 2015 - 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file issdk_fxos8700.c
 * @brief The issdk_fxos8700.c file implements the ISSDK FXOS8700 sensor driver
 *        example demonstration using wireless uart adapter.
 */

/* Standard C Includes */
#include <stdio.h>

/* CMSIS Includes */
#include "Driver_I2C.h"
#include "fsl_i2c_cmsis.h"

/* ISSDK Includes */
#include "issdk_hal.h"
#include "pedometer.h"
#include "fxos8700_drv.h"

/* BLE Framework includes */
#include "GPIO_Adapter.h"

/* ISSDK Interface Include */
#include "issdk_interface.h"

/*******************************************************************************
 * Macro Definitions
 ******************************************************************************/
#define FIFO_SIZE (32)
#define RAW_MAG_DATA_SIZE (6)
#define RAW_ACCEL_DATA_SIZE (6)
#define FXOS8700_DapIsrPrio_c (0x80)

/*******************************************************************************
 * Constants
 ******************************************************************************/
/* Configure the fxos8700 to STANDBY and 50Hz sampling rate.
 * fxos8700 is used as the acceleration source for the pedometer */
const registerwritelist_t fxos8700_Config[] = {
    /*! Prepare the register write list to configure FXOS8700 in FIFO mode. */
    {FXOS8700_F_SETUP, FXOS8700_F_SETUP_F_MODE_FIFO_STOP_OVF | (FIFO_SIZE << FXOS8700_F_SETUP_F_WMRK_SHIFT),
     FXOS8700_F_SETUP_F_MODE_MASK | FXOS8700_F_SETUP_F_WMRK_MASK},
    /*! Configure the fxos8700 to 50Hz sampling rate. */
    {FXOS8700_CTRL_REG1, FXOS8700_CTRL_REG1_DR_HYBRID_50_HZ, FXOS8700_CTRL_REG1_DR_MASK},
    /*! Active High, Push-Pull */
    {FXOS8700_CTRL_REG3, FXOS8700_CTRL_REG3_IPOL_ACTIVE_HIGH | FXOS8700_CTRL_REG3_PP_OD_PUSH_PULL,
     FXOS8700_CTRL_REG3_IPOL_MASK | FXOS8700_CTRL_REG3_PP_OD_MASK},
     /*! Data Ready Event. */
    {FXOS8700_CTRL_REG4, FXOS8700_CTRL_REG4_INT_EN_FIFO_EN, FXOS8700_CTRL_REG4_INT_EN_FIFO_MASK},
     /*! INT1 Pin  */
    {FXOS8700_CTRL_REG5, FXOS8700_CTRL_REG5_INT_CFG_FIFO_INT1, FXOS8700_CTRL_REG5_INT_CFG_FIFO_MASK},
     /*! Enable both ACC and MAG. */
    {FXOS8700_M_CTRL_REG1, FXOS8700_M_CTRL_REG1_M_HMS_HYBRID_MODE, FXOS8700_M_CTRL_REG1_M_HMS_MASK},
    /*! Automatic magnetic reset is disabled. */
    {FXOS8700_M_CTRL_REG2, FXOS8700_M_CTRL_REG2_M_RST_CNT_DISABLE, FXOS8700_M_CTRL_REG2_M_RST_CNT_MASK},
    __END_WRITE_DATA__};

/*! Command definition to read the FIFO_SIZE Bytes of Accel Data */
const registerreadlist_t FXOS8700_ACCEL_FIFO_READ[] = {
    {.readFrom = FXOS8700_STATUS, .numBytes = RAW_ACCEL_DATA_SIZE*FIFO_SIZE + 1}, __END_READ_DATA__};

/*! Command definition to read the Accel + Mag Data (without Auto Inc in 2 TXNs). */
const registerreadlist_t FXOS8700_ACCEL_MAG_READ[] = {{.readFrom = FXOS8700_OUT_X_MSB, .numBytes = RAW_ACCEL_DATA_SIZE},
                                                      {.readFrom = FXOS8700_M_OUT_X_MSB, .numBytes = RAW_MAG_DATA_SIZE},
                                                      __END_READ_DATA__};

/* Pedometer configuration. These configuration are algorithm and user dependent data. */
static const pedometer_config_t pedo_config = {
    .sleepcount_threshold = 1,
    .bits = {.config = 1},
    .keynetik =
        {
            .height = 175,
            .weight = 80,
            .filtersteps = 1,
            .bits =
                {
                    .filtertime = 1,
                    .male = 1,
                },
            .speedperiod = PEDO_SPEED_PERIOD_DEFAULT,
            .stepthreshold = PEDO_STEP_THRESHOLD_DEFAULT,
        },
    .stepcoalesce = 1,
    .oneG = PEDO_ONEG_2G,              // It is the One G representation in 2G scale.
    .frequency = PEDO_FREQHZ_DEFAULT,  // It is 50 HZ

};

const gpioInputPinConfig_t fxos8700_intCfg = {
    .gpioPort = gpioPort_C_c,
    .gpioPin = 1,
    .interruptSelect = pinInt_LogicOne_c,
    .pullSelect = pinPull_Up_c,
};

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
pedometer_t pedometer;
ARM_DRIVER_I2C *I2Cdrv = &I2C_S_DRIVER;
fxos8700_i2c_sensorhandle_t FXOS8700drv;

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

void fxos8700_isr_callback(void)
{
    int32_t status;
    fxos8700_acceldata_t rawData;
    uint8_t i, f_cnt, data[RAW_ACCEL_DATA_SIZE*FIFO_SIZE + 1];

    if(GpioIsPinIntPending(&fxos8700_intCfg))
    {
        /*! Read all the raw sensor data from the fxos8700. */
        status = FXOS8700_I2C_ReadData(&FXOS8700drv, FXOS8700_ACCEL_FIFO_READ, data);
        if (ARM_DRIVER_OK != status)
        {
            BleApp_WriteToUART("\r\n Read Failed. \r\n");
            return;
        }

        /* Get the number of samples in FIFO buffer. */
        f_cnt = data[0] & FXOS8700_F_STATUS_F_CNT_MASK;
        for (i = 0; i < f_cnt; i++)
        {   /*! Convert the raw sensor data to signed 16-bit container for display to the debug port. */
            rawData.accel[0] = ((uint16_t)data[i * RAW_ACCEL_DATA_SIZE + 1] << 8) | data[i * RAW_ACCEL_DATA_SIZE + 2];
            rawData.accel[1] = ((uint16_t)data[i * RAW_ACCEL_DATA_SIZE + 3] << 8) | data[i * RAW_ACCEL_DATA_SIZE + 4];
            rawData.accel[2] = ((uint16_t)data[i * RAW_ACCEL_DATA_SIZE + 5] << 8) | data[i * RAW_ACCEL_DATA_SIZE + 6];

            /*  Execute the pedometer Algorithm on all the received samples. */
            pedometer_run(&pedometer, (ped_accel_t *)&rawData.accel);
        }

        GpioInputPinInit(&fxos8700_intCfg, 1);
        GpioClearPinIntFlag(&fxos8700_intCfg);
    }
}

void ISSDK_ReadSensor(void)
{
    int32_t status;
    float aX, aY, aZ, mX, mY, mZ;
    fxos8700_accelmagdata_t rawData;
    uint8_t data[RAW_ACCEL_DATA_SIZE + RAW_MAG_DATA_SIZE];
    char aXs[8]={0}, aYs[8]={0}, aZs[8]={0}, mXs[8]={0}, mYs[8]={0}, mZs[8]={0};

    /*! Read the raw sensor data from the fxos8700. */
    status = FXOS8700_I2C_ReadData(&FXOS8700drv, FXOS8700_ACCEL_MAG_READ, data);
    if (ARM_DRIVER_OK != status)
    {
        BleApp_WriteToUART("\r\n Read Failed. \r\n");
        return;
    }

    /*! Convert the raw sensor data to signed 16-bit container for display to the debug port. */
    rawData.accel[0] = ((int16_t)data[0] << 8)  | data[1];
    rawData.accel[0] /= 4;
    rawData.accel[1] = ((int16_t)data[2] << 8)  | data[3];
    rawData.accel[1] /= 4;
    rawData.accel[2] = ((int16_t)data[4] << 8)  | data[5];
    rawData.accel[2] /= 4;
    rawData.mag[0]   = ((int16_t)data[6] << 8)  | data[7];
    rawData.mag[1]   = ((int16_t)data[8] << 8)  | data[9];
    rawData.mag[2]   = ((int16_t)data[10] << 8) | data[11];

    aX = 0.000244 * rawData.accel[0];
    aY = 0.000244 * rawData.accel[1];
    aZ = 0.000244 * rawData.accel[2];
    mX = 0.1 * rawData.mag[0];
    mY = 0.1 * rawData.mag[1];
    mZ = 0.1 * rawData.mag[2];

    getStringOfFloat(aX, aXs);
    getStringOfFloat(aY, aYs);
    getStringOfFloat(aZ, aZs);
    getStringOfFloat(mX, mXs);
    getStringOfFloat(mY, mYs);
    getStringOfFloat(mZ, mZs);

    /* Send data over the air */
    BleApp_WriteToHost("ACC: %sg %sg %sg\nMAG: %suT %suT %suT\n\n", aXs, aYs, aZs, mXs, mYs, mZs);
    //BleApp_WriteToHost("A: %sg %sg %sg\n", aXs, aYs, aZs);
}

void ISSDK_ReadPedometer(void)
{   /* Send data over the air */
    BleApp_WriteToHost("STEPS : %d\n", pedometer.status.stepcount);
}

void ISSDK_InitSensor(void)
{
    int32_t status;

    BleApp_WriteToUART("\r\n ISSDK FXOS8700 BLE example.\r\n");

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

    /*! Initialize the FXOS8700 sensor driver. */
    status = FXOS8700_I2C_Initialize(&FXOS8700drv, &I2C_BB_DRIVER, I2C_BB_DEVICE_INDEX, FXOS8700_BB_I2C_ADDR,
                                     FXOS8700_WHO_AM_I_PROD_VALUE);
    if (SENSOR_ERROR_NONE != status)
    {
        BleApp_WriteToUART("\r\n Sensor Initialization Failed\r\n");
        return;
    }
    BleApp_WriteToUART("\r\n Successfully Initialized Sensor.\r\n");

    /* Appy FXOS8700 Configuration based on the Register List */
    status = Sensor_I2C_Write(FXOS8700drv.pCommDrv, &FXOS8700drv.deviceInfo, FXOS8700drv.slaveAddress, fxos8700_Config);
    if (ARM_DRIVER_OK != status)
    {
        BleApp_WriteToUART("\r\n Sensor Configuration Failed, Err = %d\r\n", status);
        return;
    }
    BleApp_WriteToUART("\r\n Successfully Configured Sensor.\r\n");

    /* Initialize the pedometer */
    pedometer_init(&pedometer);

    /* Configure the pedometer */
    pedometer_configure(&pedometer, &pedo_config);

    /* Install Sensor Data Ready Callback INT */
    GpioInstallIsr(fxos8700_isr_callback, gGpioIsrPrioNormal_c, FXOS8700_DapIsrPrio_c, &fxos8700_intCfg);
    GpioInputPinInit(&fxos8700_intCfg, 1);
}

void ISSDK_ConfigureSensor(uint32_t arg)
{
    int32_t status;

    switch(arg)
    {
        case ISSDK_START_SENSOR:
            /* Put the device into active mode.*/
            status = Register_I2C_Write(FXOS8700drv.pCommDrv, &FXOS8700drv.deviceInfo, FXOS8700drv.slaveAddress,
                                        FXOS8700_CTRL_REG1, FXOS8700_CTRL_REG1_ACTIVE_ACTIVE_MODE,
                                        FXOS8700_CTRL_REG1_ACTIVE_MASK, false);
            if (ARM_DRIVER_OK != status)
            {
                BleApp_WriteToUART("\r\n Failed to put sensor to Active Mode.\r\n");
            }
            else
            {
                BleApp_WriteToUART("\r\n Successfully put Sensor to Active Mode.\r\n");
            }
            break;
        case ISSDK_STOP_SENSOR:
            /* Put the device into standby mode.*/
            status = Register_I2C_Write(FXOS8700drv.pCommDrv, &FXOS8700drv.deviceInfo, FXOS8700drv.slaveAddress,
                                        FXOS8700_CTRL_REG1, FXOS8700_CTRL_REG1_ACTIVE_STANDBY_MODE,
                                        FXOS8700_CTRL_REG1_ACTIVE_MASK, false);
            if (ARM_DRIVER_OK != status)
            {
                BleApp_WriteToUART("\r\n Failed to put sensor to Standby Mode.\r\n");
            }
            else
            {
                BleApp_WriteToUART("\r\n Successfully put Sensor to Standby Mode.\r\n");
            }
            break;
        default:
            break;
    }
}