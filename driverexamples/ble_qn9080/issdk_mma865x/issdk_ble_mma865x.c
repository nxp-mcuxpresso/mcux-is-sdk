/*
 * Copyright (c) 2015 - 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file issdk_ble_mma865x.c
 * @brief The issdk_ble_mma865x.c file implements the ISSDK MMA865x
 *        sensor events example demonstration using wireless uart adapter.
 * Supported Modes:
 *   MODE   = 'STANDBY'       (7 Fixed Characters to Stop All Sensor Data)
 *   MODE   = 'ACCELEROMETER' (13 Fixed Characters to Start Accelerometer Data)
 *   MODE   = 'FREEFALL'      (8 Fixed Characters to Start Freefall detection)
 *   MODE   = 'PEDOMETER'     (9 Fixed Characters to Pedometer Data capture)
 *   MODE   = 'ORIENTATION'   (11 Fixed Characters to Start Orientation detection)
 */

/* Standard C Includes */
#include <stdio.h>

/* CMSIS Includes */
#include "Driver_I2C.h"

/* ISSDK Includes */
#include "issdk_hal.h"
#include "pedometer.h"
#include "mma865x_drv.h"

/* ISSDK Interface Include */
#include "issdk_ble_interface.h"

/* BLE framework Includes */
#include "LED.h"
#include "MemManager.h"
#include "GPIO_Adapter.h"

/*******************************************************************************
 * Macro Definitions
 ******************************************************************************/
#define MMA865x_DATA_SIZE (6)
#define MMA865x_ACCEL_CONV (0.00098) /* Will give 'g' */
                                     /* FF_MT freefall counter register values for High resolution Mode and ODR = 100Hz.
                                      * These values have been derived based on the MMA865x DataSheet and Application Note AN4070 for MMA865x (the same is
                                      * applicable to MMA865x too).
                                      * http://cache.freescale.com/files/sensors/doc/app_note/AN4070.pdf */
#define FF_MT_WT_DBCNT 0x23          /* Debounce count value. */
#define FF_THS_VALUE 0x03            /* Threshold Value. */
#define MT_THS_VALUE 0x15            /* Threshold Value. */
#define PL_COUNT 0x30                /* Debounce count value. */
#define ASLP_COUNTER 0x10            /* Auto Sleep after ~5s. */

/*******************************************************************************
 * Constants
 ******************************************************************************/
/*! Prepare the register write list to configure MMA865x Normal mode. */
const registerwritelist_t cMma865xConfigInterrupt[] = {
    /*! Configure the MMA865x to set ASLP Counter. */
    {MMA865x_ASLP_COUNT, ASLP_COUNTER, 0},
    /*! Configure the MMA865x to set PL Counter. */
    {MMA865x_PL_COUNT, PL_COUNT, 0},
    /*! Configure the MMA865x to set High Resolution mode. */
    {MMA865x_CTRL_REG2, MMA865x_CTRL_REG2_MODS_HR, MMA865x_CTRL_REG2_MODS_MASK},
    /*! Configure the MMA865x to set interrupt polarity as Active High. */
    {MMA865x_CTRL_REG3, MMA865x_CTRL_REG3_IPOL_ACTIVE_HIGH, MMA865x_CTRL_REG3_IPOL_MASK},
    /*! Configure the MMA865x to route Data Ready and Event Ready Interrupts to INT2. */
    {MMA865x_CTRL_REG5, MMA865x_CTRL_REG5_INT_CFG_FF_MT_INT2 | MMA865x_CTRL_REG5_INT_CFG_DRDY_INT2 | MMA865x_CTRL_REG5_INT_CFG_LNDPRT_INT2,
                        MMA865x_CTRL_REG5_INT_CFG_FF_MT_MASK | MMA865x_CTRL_REG5_INT_CFG_DRDY_MASK | MMA865x_CTRL_REG5_INT_CFG_LNDPRT_MASK},
    __END_WRITE_DATA__};

/*! @brief MMA865x Start INT Mode Register Write List */
const registerwritelist_t cMma865xStartConfiguration[] = {
    /*! Configure the MMA865x to enable Interrupts for Data Ready. */
    {MMA865x_CTRL_REG4, MMA865x_CTRL_REG4_INT_EN_DRDY_EN, MMA865x_CTRL_REG4_INT_EN_DRDY_MASK},
    /*! Configure the MMA865x to set ODR to 1.56Hz and set Active Mode. */
    {MMA865x_CTRL_REG1, MMA865x_CTRL_REG1_DR_12_5HZ | MMA865x_CTRL_REG1_ACTIVE_ACTIVATED,
                        MMA865x_CTRL_REG1_DR_MASK | MMA865x_CTRL_REG1_ACTIVE_MASK},
    __END_WRITE_DATA__};

/*! @brief Register Start Free-fall Mode Register Write List. */
const registerwritelist_t cMma865xStartFreeFallConfiguration[] = {
    /*! Configure the MMA865x to set Debounce counter value. */
    {MMA865x_FF_MT_COUNT, FF_MT_WT_DBCNT, 0},
    /*! Configure the MMA865x to set Debounce counter to be cleared on favorable events and the thresholds . */
    {MMA865x_FF_MT_THS, MMA865x_FF_MT_THS_DBCNTM_INC_CLR | FF_THS_VALUE,
                        MMA865x_FF_MT_THS_DBCNTM_MASK | MMA865x_FF_MT_THS_THS_MASK},
    /*! Configure the MMA865x to set freefall Mode and enable all XYZ axis events and event latching. */
    {MMA865x_FF_MT_CFG, MMA865x_FF_MT_CFG_ELE_EN | MMA865x_FF_MT_CFG_OAE_FREEFALL | MMA865x_FF_MT_CFG_XEFE_EN | MMA865x_FF_MT_CFG_YEFE_EN | MMA865x_FF_MT_CFG_ZEFE_EN,
                        MMA865x_FF_MT_CFG_ELE_MASK | MMA865x_FF_MT_CFG_OAE_MASK | MMA865x_FF_MT_CFG_XEFE_MASK | MMA865x_FF_MT_CFG_YEFE_MASK | MMA865x_FF_MT_CFG_ZEFE_MASK},
    /*! Configure the MMA865x to enable Interrupts for Data Ready. */
    {MMA865x_CTRL_REG4, MMA865x_CTRL_REG4_INT_EN_FF_MT_EN, MMA865x_CTRL_REG4_INT_EN_FF_MT_MASK},
    /*! Configure the MMA865x to set ODR to 100Hz. */
    {MMA865x_CTRL_REG1, MMA865x_CTRL_REG1_DR_100HZ | MMA865x_CTRL_REG1_ACTIVE_ACTIVATED,
                        MMA865x_CTRL_REG1_DR_MASK | MMA865x_CTRL_REG1_ACTIVE_MASK},
    __END_WRITE_DATA__};

/*! @brief Register Start Motion-detect Mode Register Write List. */
const registerwritelist_t cMma865xStartMotionConfiguration[] = {
    {MMA865x_FF_MT_THS, MT_THS_VALUE, MMA865x_FF_MT_THS_THS_MASK}, /* Threshold */
    {MMA865x_FF_MT_CFG,
                        MMA865x_FF_MT_CFG_OAE_MOTION | MMA865x_FF_MT_CFG_ZEFE_EN | MMA865x_FF_MT_CFG_YEFE_EN | MMA865x_FF_MT_CFG_XEFE_EN,
                        MMA865x_FF_MT_CFG_OAE_MASK | MMA865x_FF_MT_CFG_ZEFE_MASK | MMA865x_FF_MT_CFG_YEFE_MASK | MMA865x_FF_MT_CFG_XEFE_MASK},
    {MMA865x_CTRL_REG4, MMA865x_CTRL_REG4_INT_EN_FF_MT_EN, MMA865x_CTRL_REG4_INT_EN_FF_MT_MASK},
    {MMA865x_CTRL_REG1, MMA865x_CTRL_REG1_DR_1_56HZ | MMA865x_CTRL_REG1_ACTIVE_ACTIVATED,
                        MMA865x_CTRL_REG1_DR_MASK | MMA865x_CTRL_REG1_ACTIVE_MASK},
    __END_WRITE_DATA__};

/*! @brief Register Start Orientation-detect Mode Register Write List. */
const registerwritelist_t cMma865xStartOrientationConfiguration[] = {
    {MMA865x_PL_CFG, MMA865x_PL_CFG_DBCNTM_CLEAR | MMA865x_PL_CFG_PL_EN_EN,
                     MMA865x_PL_CFG_PL_EN_MASK | MMA865x_PL_CFG_DBCNTM_MASK},
    {MMA865x_CTRL_REG4, MMA865x_CTRL_REG4_INT_EN_LNDPRT_EN | MMA865x_CTRL_REG4_INT_EN_ASLP_EN,
                        MMA865x_CTRL_REG4_INT_EN_LNDPRT_MASK | MMA865x_CTRL_REG4_INT_EN_ASLP_MASK},
    {MMA865x_CTRL_REG3, MMA865x_CTRL_REG3_WAKE_LNDPRT_EN, MMA865x_CTRL_REG3_WAKE_LNDPRT_MASK},
    {MMA865x_CTRL_REG2, MMA865x_CTRL_REG2_SLPE_EN, MMA865x_CTRL_REG2_SLPE_MASK},
    {MMA865x_CTRL_REG1, MMA865x_CTRL_REG1_DR_6_25HZ | MMA865x_CTRL_REG1_ACTIVE_ACTIVATED,
                        MMA865x_CTRL_REG1_DR_MASK | MMA865x_CTRL_REG1_ACTIVE_MASK},
    __END_WRITE_DATA__};

/*! @brief MMA865x Start Pedometer Mode Register Write List */
const registerwritelist_t cMma865xStartPedometerConfiguration[] = {
    /*! Prepare the register write list to configure MMA865x in Normal mode. */
    {MMA865x_CTRL_REG4, MMA865x_CTRL_REG4_INT_EN_DRDY_EN, MMA865x_CTRL_REG4_INT_EN_DRDY_MASK},
    {MMA865x_CTRL_REG1, MMA865x_CTRL_REG1_DR_50HZ | MMA865x_CTRL_REG1_ACTIVE_ACTIVATED,
                        MMA865x_CTRL_REG1_DR_MASK | MMA865x_CTRL_REG1_ACTIVE_MASK},
    __END_WRITE_DATA__};

/*! @brief MMA865x Stop Register Write List */
const registerwritelist_t cMma865xStopConfiguration[] = {
    /* Clear Active Mode. */
    {MMA865x_CTRL_REG1, MMA865x_CTRL_REG1_ACTIVE_STANDBY, MMA865x_CTRL_REG1_ACTIVE_MASK},
    {MMA865x_CTRL_REG2, MMA865x_CTRL_REG2_SLPE_DISABLED, MMA865x_CTRL_REG2_SLPE_MASK},
    {MMA865x_CTRL_REG3, MMA865x_CTRL_REG3_WAKE_LNDPRT_BYPASSED, MMA865x_CTRL_REG3_WAKE_LNDPRT_MASK},
    {MMA865x_FF_MT_THS, 0x00, MMA865x_FF_MT_THS_THS_MASK}, /* Threshold */
    {MMA865x_CTRL_REG4, 0x00, 0},
    {MMA865x_FF_MT_COUNT, 0x00, 0},
    {MMA865x_FF_MT_CFG, 0x00, 0},
    {MMA865x_PL_CFG, 0x00, 0},
    __END_WRITE_DATA__};

/*! Command definition to read the Accel+Mag Data. */
const registerreadlist_t cMma865xReadData[] = {{.readFrom = MMA865x_OUT_X_MSB, .numBytes = MMA865x_DATA_SIZE},
                                               __END_READ_DATA__};

/*! @brief Address of FF_MT Status Register. */
const registerreadlist_t cMma865xFFMTSrc[] = {{.readFrom = MMA865x_FF_MT_SRC, .numBytes = 1}, __END_READ_DATA__};

/*! @brief Address of INT Source Register. */
const registerreadlist_t cMma865xINTSrc[] = {{.readFrom = MMA865x_INT_SOURCE, .numBytes = 1}, __END_READ_DATA__};

/*! @brief Address of PL Status Register. */
const registerreadlist_t cMma865xOutputPLStatus[] = {{.readFrom = MMA865x_PL_STATUS, .numBytes = 1}, __END_READ_DATA__};

/* Pedometer configuration. These configuration are algorithm and user dependent data. */
const pedometer_config_t cPedoConfig = {
    .sleepcount_threshold = 1,
    .bits = {.config = 1},
    .keynetik =
        {
            .height = 175,
            .weight = 80,
            .filtersteps = 1,
            .bits =
                {
                    .filtertime = 1, .male = 1,
                },
            .speedperiod = PEDO_SPEED_PERIOD_DEFAULT,
            .stepthreshold = PEDO_STEP_THRESHOLD_DEFAULT,
        },
    .stepcoalesce = 1,
    .oneG = PEDO_ONEG_2G,             // It is the One G representation in 2G scale.
    .frequency = PEDO_FREQHZ_DEFAULT, // It is 50 HZ

};

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
bool gMotionDetect;
pedometer_t gPedometer;
uint16_t gLastReportedSteps;
uint8_t gReportMode, gReportFormat;
gpioInputPinConfig_t gMma865xIntCfg;
ARM_DRIVER_I2C *gI2cDriver = &I2C_BB_DRIVER;
mma865x_i2c_sensorhandle_t gMma865xDriver;

/************************************************************************************
* Functions
************************************************************************************/
/*! @brief Function to Read Acceleration Data */
static void MMA865x_ReadAcceleration(void)
{
    float accelX, accelY, accelZ;
    int32_t status, accelXi, accelYi, accelZi;
    char accelXs[8] = {0}, accelYs[8] = {0}, accelZs[8] = {0};
    mma865x_acceldata_t rawData;
    uint8_t data[MMA865x_DATA_SIZE];

    /*! Read the raw sensor data from the MMA865x. */
    status = MMA865x_I2C_ReadData(&gMma865xDriver, cMma865xReadData, data);
    if (ARM_DRIVER_OK != status)
    {
        BleApp_WriteToUART("ERROR : Read Acceleration Failed!\r\n");
        return;
    }

    /*! Convert the raw sensor data to signed 16-bit container for display to the debug port. */
    rawData.accel[0] = ((int16_t)data[0] << 8) | data[1];
    rawData.accel[0] /= 16;
    rawData.accel[1] = ((int16_t)data[2] << 8) | data[3];
    rawData.accel[1] /= 16;
    rawData.accel[2] = ((int16_t)data[4] << 8) | data[5];
    rawData.accel[2] /= 16;
    accelX = MMA865x_ACCEL_CONV * rawData.accel[0];
    accelY = MMA865x_ACCEL_CONV * rawData.accel[1];
    accelZ = MMA865x_ACCEL_CONV * rawData.accel[2];

    if (gReportFormat == ISSDK_READ_RAW)
    {
        /* Send raw data over the air */
        BleApp_WriteToHost("ACCEL: X=0x%04X Y=0x%04X Z=0x%04X\n", (uint16_t)rawData.accel[0],
                           (uint16_t)rawData.accel[1], (uint16_t)rawData.accel[2]);
    }
    if (gReportFormat == ISSDK_READ_NORMAL)
    {
        ISSDK_GetStringOfFloat(accelX, accelXs);
        ISSDK_GetStringOfFloat(accelY, accelYs);
        ISSDK_GetStringOfFloat(accelZ, accelZs);

        /* Send data over the air */
        BleApp_WriteToHost("ACCEL: X=%sg Y=%sg Z=%sg\n", accelXs, accelYs, accelZs);
    }
    if (gReportFormat == ISSDK_READ_STREAM)
    {
        accelXi = ISSDK_GetDecimalForFloat(accelX);
        accelYi = ISSDK_GetDecimalForFloat(accelY);
        accelZi = ISSDK_GetDecimalForFloat(accelZ);
        /* Send Data Stream over the air */
        BleApp_WriteToHost("%s:%08X:%08X:%08X\n", ISSDK_ACCEL_STREAM_HDR_STR, (uint32_t)accelXi, (uint32_t)accelYi,
                           (uint32_t)accelZi);
    }
}

/*! @brief Function to Read Pedometer Data */
void MMA865x_ReadPedometer(void)
{
    char speedS[8];

    /* Log the last reported steps. */
    gLastReportedSteps = gPedometer.status.stepcount;
    ISSDK_GetStringOfFloat(gPedometer.status.speed / 1000.0, speedS);

    /* Send data over the air */
    BleApp_WriteToHost("STEPS:%d | DISTANCE:%dm | SPEED:%skm/h\n", gPedometer.status.stepcount,
                       gPedometer.status.distance, speedS);
}

/*! @brief Function to Read and Write Sensor registers */
static int32_t MMA865x_RegisterInterface(uint8_t opCode, uint8_t mode, uint8_t offset, uint8_t bytes, uint8_t *buffer)
{
    int32_t result = SENSOR_ERROR_INVALID_PARAM;

    if (buffer == NULL || bytes == 0)
    {
        BleApp_WriteToUART("ERROR : Invalid Buffer or Size for Register Read!\r\n");
        return result;
    }

    switch (opCode)
    {
        case ISSDK_REGISTER_READ:
            switch (mode)
            {
                case ISSDK_REPORT_ACCELERATION:
                    /*! Read MMA865x registers. */
                    result = Register_I2C_Read(gMma865xDriver.pCommDrv, &gMma865xDriver.deviceInfo,
                                               gMma865xDriver.slaveAddress, offset, bytes, buffer);
                    if (result != SENSOR_ERROR_NONE)
                    {
                        BleApp_WriteToUART("ERROR : Read [%d] bytes form [0x%02X] of MMA865x!\r\n", offset, bytes);
                    }
                    break;
                default:
                    BleApp_WriteToUART("ERROR : Incorrect Device for Register Read [%d]!\r\n", mode);
                    break;
            }
            break;
        case ISSDK_REGISTER_WRITE:
            switch (mode)
            {
                case ISSDK_REPORT_ACCELERATION:
                    /*! Write MMA865x registers. */
                    result = Register_I2C_BlockWrite(gMma865xDriver.pCommDrv, &gMma865xDriver.deviceInfo,
                                                     gMma865xDriver.slaveAddress, offset, buffer, bytes);
                    if (result != SENSOR_ERROR_NONE)
                    {
                        BleApp_WriteToUART("ERROR : Write [%d] bytes to [0x%02X] of MMA865x!\r\n", offset, bytes);
                    }
                    break;
                default:
                    BleApp_WriteToUART("ERROR : Incorrect Device for Register Write [%d]!\r\n", mode);
                    break;
            }
            break;
        default:
            BleApp_WriteToUART("ERROR : Unknown Register OP Code [%d]!\r\n", opCode);
            break;
    }

    return result;
}

void MMA865x_ISR_Callback(void)
{
    int32_t status;
    mma865x_acceldata_t rawData;
    uint8_t dataReady, data[MMA865x_DATA_SIZE];

    if (GpioIsPinIntPending(&gMma865xIntCfg))
    {
        if (gReportMode == ISSDK_REPORT_FREEFALL)
        {
            /*! Read the Freefall event FLAGs from MMA865x. */
            status = MMA865x_I2C_ReadData(&gMma865xDriver, cMma865xFFMTSrc, &dataReady);
            if (SENSOR_ERROR_NONE != status)
            {
                BleApp_WriteToUART("ERROR : Read MMA865x Event Failed!\r\n");
                return;
            }

            if (MMA865x_FF_MT_SRC_EA_NONE == (dataReady & MMA865x_FF_MT_SRC_EA_MASK))
            { /* Return, if new event is not detected. */
                return;
            }

            /*! Display that a freefall event has been detected. */
            BleApp_WriteToUART("EVENT : Freefall detected...\r\n");
        }
        if (gReportMode == ISSDK_REPORT_PEDOMETER)
        {
            /*! Read all the raw sensor data from the fxos8700. */
            status = MMA865x_I2C_ReadData(&gMma865xDriver, cMma865xReadData, data);
            if (SENSOR_ERROR_NONE != status)
            {
                BleApp_WriteToUART("ERROR : Read MMA865x DATA Failed!\r\n");
                return;
            }

            /*! Convert the raw sensor data to signed 16-bit container for display to the debug port. */
            rawData.accel[0] = ((uint16_t)data[0] << 8) | data[1];
            rawData.accel[1] = ((uint16_t)data[2] << 8) | data[3];
            rawData.accel[2] = ((uint16_t)data[4] << 8) | data[5];

            /*  Execute the pedometer Algorithm on all the received samples. */
            pedometer_run(&gPedometer, (ped_accel_t *)&rawData.accel);

            if (gPedometer.status.stepcount != gLastReportedSteps)
            {
                MMA865x_ReadPedometer();
                Led2Toggle();
            }
        }
        if (gReportMode == ISSDK_REPORT_ORIENTATION)
        {
            if (gMotionDetect)
            {
                /*! Read the FFMT Status from the MMA865x. */
                status = MMA865x_I2C_ReadData(&gMma865xDriver, cMma865xFFMTSrc, &dataReady);
                if (ARM_DRIVER_OK != status)
                {
                    BleApp_WriteToUART("ERROR : Read MMA865x FFMT Failed!\r\n");
                    return;
                }

                if ((dataReady & MMA865x_FF_MT_SRC_EA_MASK) == MMA865x_FF_MT_SRC_EA_DETECTED)
                {
                    /*! Display that a Motion event has been detected. */
                    BleApp_WriteToUART("EVENT : Motion detected...\r\n");

                    /* Apply MMA865x Stop Mode Configuration. */
                    status = Sensor_I2C_Write(gMma865xDriver.pCommDrv, &gMma865xDriver.deviceInfo,
                                              gMma865xDriver.slaveAddress, cMma865xStopConfiguration);
                    if (SENSOR_ERROR_NONE != status)
                    {
                        BleApp_WriteToUART("ERROR : Write MMA865x Stop Configuration!\r\n");
                        return;
                    }

                    /* Apply MMA865x Configuration for Orientation Detection. */
                    status = Sensor_I2C_Write(gMma865xDriver.pCommDrv, &gMma865xDriver.deviceInfo,
                                              gMma865xDriver.slaveAddress, cMma865xStartOrientationConfiguration);
                    if (SENSOR_ERROR_NONE != status)
                    {
                        BleApp_WriteToUART("ERROR : Write MMA865x Orientation Configuration Failed!\r\n");
                        return;
                    }
                    gMotionDetect = false;
                }
            }
            else
            {
                /*! Read the Orientation Status from the MMA865x. */
                status = MMA865x_I2C_ReadData(&gMma865xDriver, cMma865xOutputPLStatus, &dataReady);
                if (ARM_DRIVER_OK != status)
                {
                    BleApp_WriteToUART("ERROR : Read Orientation Failed!\r\n");
                    return;
                }

                if (((dataReady & MMA865x_PL_STATUS_NEWLP_MASK) == MMA865x_PL_STATUS_NEWLP_DETECTED) &&
                    ((dataReady & MMA865x_PL_STATUS_LO_MASK) == MMA865x_PL_STATUS_LO_NOT_DETECTED))
                {
                    if ((dataReady & MMA865x_PL_STATUS_LAPO_MASK) == MMA865x_PL_STATUS_LAPO_PORTRAIT_UP)
                    {
                        BleApp_WriteToUART("EVENT : Portrait Up ...\r\n");
                    }
                    if ((dataReady & MMA865x_PL_STATUS_LAPO_MASK) == MMA865x_PL_STATUS_LAPO_PORTRAIT_DOWN)
                    {
                        BleApp_WriteToUART("EVENT : Portrait Down ...\r\n");
                    }
                    if ((dataReady & MMA865x_PL_STATUS_LAPO_MASK) == MMA865x_PL_STATUS_LAPO_LANDSCAPE_UP)
                    {
                        BleApp_WriteToUART("EVENT : Landscape Right ...\r\n");
                    }
                    if ((dataReady & MMA865x_PL_STATUS_LAPO_MASK) == MMA865x_PL_STATUS_LAPO_LANDSCAPE_DOWN)
                    {
                        BleApp_WriteToUART("EVENT : Landscape Left ...\r\n");
                    }
                }

                if (((dataReady & MMA865x_PL_STATUS_NEWLP_MASK) == MMA865x_PL_STATUS_NEWLP_DETECTED) &&
                    ((dataReady & MMA865x_PL_STATUS_LO_MASK) == MMA865x_PL_STATUS_LO_DETECTED))
                {
                    if ((dataReady & MMA865x_PL_STATUS_BAFRO_MASK) == MMA865x_PL_STATUS_BAFRO_FRONT)
                    {
                        BleApp_WriteToUART("EVENT : Front Side ...\r\n");
                    }
                    if ((dataReady & MMA865x_PL_STATUS_BAFRO_MASK) == MMA865x_PL_STATUS_BAFRO_BACK)
                    {
                        BleApp_WriteToUART("EVENT : Back Side ...\r\n");
                    }
                }

                status = MMA865x_I2C_ReadData(&gMma865xDriver, cMma865xINTSrc, &dataReady);
                if (ARM_DRIVER_OK != status)
                {
                    BleApp_WriteToUART("ERROR : Read INT SRC Failed!\r\n");
                    return;
                }
                if ((dataReady & MMA865x_INT_SOURCE_SRC_ASLP_MASK) == MMA865x_INT_SOURCE_SRC_ASLP_MASK)
                {
                    /* Apply MMA865x Stop Mode Configuration. */
                    status = Sensor_I2C_Write(gMma865xDriver.pCommDrv, &gMma865xDriver.deviceInfo,
                                              gMma865xDriver.slaveAddress, cMma865xStopConfiguration);
                    if (SENSOR_ERROR_NONE != status)
                    {
                        BleApp_WriteToUART("ERROR : Write MMA865x Stop Configuration!\r\n");
                        return;
                    }

                    /* Apply MMA865x Configuration for Motion Detection. */
                    status = Sensor_I2C_Write(gMma865xDriver.pCommDrv, &gMma865xDriver.deviceInfo,
                                              gMma865xDriver.slaveAddress, cMma865xStartMotionConfiguration);
                    if (SENSOR_ERROR_NONE != status)
                    {
                        BleApp_WriteToUART("ERROR : Write MMA865x Motion Configuration Failed!\r\n");
                        return;
                    }
                    gMotionDetect = true;
                    BleApp_WriteToUART("EVENT : Going into Inactive Mode ...\r\n");
                }
            }
        }
        if (gReportMode == ISSDK_REPORT_ACCELERATION)
        {
            MMA865x_ReadAcceleration();
            Led2Toggle();
        }
        GpioInputPinInit(&gMma865xIntCfg, 1);
        GpioClearPinIntFlag(&gMma865xIntCfg);
    }
}

void ISSDK_Initialize(void)
{
    int32_t status;

    BleApp_WriteToUART("Initializing Sensors for ISSDK MMA865x BLE example.\r\n");

    /*! Initialize the I2C driver. */
    status = gI2cDriver->Initialize(I2C_BB_SIGNAL_EVENT);
    if (ARM_DRIVER_OK != status)
    {
        BleApp_WriteToUART("ERROR : I2C Initialization Failed!\r\n");
        return;
    }

    /*! Set the I2C Power mode. */
    status = gI2cDriver->PowerControl(ARM_POWER_FULL);
    if (ARM_DRIVER_OK != status)
    {
        BleApp_WriteToUART("ERROR : I2C Power Mode setting Failed!\r\n");
        return;
    }

    /*! Set the I2C bus speed. */
    status = gI2cDriver->Control(ARM_I2C_BUS_SPEED, ARM_I2C_BUS_SPEED_FAST);
    if (ARM_DRIVER_OK != status)
    {
        BleApp_WriteToUART("ERROR : I2C Control setting Failed!\r\n");
        return;
    }

    /*! Initialize MMA865x sensor driver. */
    status = MMA865x_I2C_Initialize(&gMma865xDriver, &I2C_BB_DRIVER, I2C_BB_DEVICE_INDEX, MMA8652_BB_I2C_ADDR,
                                    MMA8652_WHOAMI_VALUE);
    if (SENSOR_ERROR_NONE != status)
    {
        BleApp_WriteToUART("ERROR : MMA865x Initialization Failed!\r\n");
        return;
    }

    MMA865x_I2C_SetIdleTask(&gMma865xDriver, (registeridlefunction_t)SMC_SetPowerModeVlpr, SMC);
    /* Apply MMA865x Configuration based on the Register List */
    status = Sensor_I2C_Write(gMma865xDriver.pCommDrv, &gMma865xDriver.deviceInfo, gMma865xDriver.slaveAddress,
                              cMma865xConfigInterrupt);
    if (ARM_DRIVER_OK != status)
    {
        BleApp_WriteToUART("ERROR : MMA865x Configuration Failed!\r\n");
        return;
    }
    BleApp_WriteToUART("Successfully Initialized MMA865x.\r\n");

    /* Initialize the pedometer */
    pedometer_init(&gPedometer);

    /* Initialize GPIO Configuration */
    gMma865xIntCfg.pullSelect = pinPull_Up_c;
    gMma865xIntCfg.interruptSelect = pinInt_LogicOne_c;
    gMma865xIntCfg.gpioPin = MMA8652_BB_INT1.pinNumber;
    gMma865xIntCfg.gpioPort = (gpioPort_t)MMA8652_BB_INT1.portNumber;
    /* Install MMA865x Event Callback INT */
    GpioInstallIsr(MMA865x_ISR_Callback, gGpioIsrPrioNormal_c, gGpioDefaultNvicPrio_c, &gMma865xIntCfg);
    GpioInputPinInit(&gMma865xIntCfg, 1);

    /* Set Default settings */
    gReportFormat = ISSDK_READ_NORMAL;
    gReportMode = ISSDK_REPORT_NONE;
    BleApp_WriteToUART("Report Mode is Standby.\r\n");
    BleApp_WriteToUART("Data Format is Converted.\r\n");
}

void ISSDK_Configure(uint8_t opCode)
{
    int32_t status;

    switch (opCode)
    {
        case ISSDK_ACTIVE_MODE:
            switch (gReportMode)
            {
                case ISSDK_REPORT_NONE:
                    break;
                case ISSDK_REPORT_ACCELERATION:
                    /* Put MMA865x device into active mode.*/
                    status = Sensor_I2C_Write(gMma865xDriver.pCommDrv, &gMma865xDriver.deviceInfo,
                                              gMma865xDriver.slaveAddress, cMma865xStartConfiguration);
                    if (ARM_DRIVER_OK != status)
                    {
                        BleApp_WriteToUART("ERROR : Failed to put MMA865x to Active Mode!\r\n");
                    }
                    else
                    {
                        BleApp_WriteToUART("Successfully put MMA865x to Active Mode.\r\n");
                    }
                    break;
                case ISSDK_REPORT_FREEFALL:
                    /* Put MMA865x into FreeFall detection mode.*/
                    status = Sensor_I2C_Write(gMma865xDriver.pCommDrv, &gMma865xDriver.deviceInfo,
                                              gMma865xDriver.slaveAddress, cMma865xStartFreeFallConfiguration);
                    if (ARM_DRIVER_OK != status)
                    {
                        BleApp_WriteToUART("ERROR : Failed to put MMA865x to FreeFall Mode!\r\n");
                    }
                    else
                    {
                        BleApp_WriteToUART("Successfully put MMA865x to FreeFall Mode.\r\n");
                        BleApp_WriteToHost("Waiting for Freefall Event...\n");
                    }
                    break;
                case ISSDK_REPORT_ORIENTATION:
                    /* Put MMA865x into Motion detection mode.*/
                    status = Sensor_I2C_Write(gMma865xDriver.pCommDrv, &gMma865xDriver.deviceInfo,
                                              gMma865xDriver.slaveAddress, cMma865xStartMotionConfiguration);
                    if (ARM_DRIVER_OK != status)
                    {
                        BleApp_WriteToUART("ERROR : Failed to put MMA865x to Motion Detect Mode!\r\n");
                    }
                    else
                    {
                        BleApp_WriteToUART("Successfully put MMA865x to Motion Detect Mode.\r\n");
                        BleApp_WriteToHost("Waiting for Motion Event...\n");
                        gMotionDetect = true;
                    }
                    break;
                case ISSDK_REPORT_PEDOMETER:
                    /* Configure the pedometer algorithm */
                    pedometer_configure(&gPedometer, &cPedoConfig);

                    /* Put MMA865x into Pedometer detection mode.*/
                    status = Sensor_I2C_Write(gMma865xDriver.pCommDrv, &gMma865xDriver.deviceInfo,
                                              gMma865xDriver.slaveAddress, cMma865xStartPedometerConfiguration);
                    if (ARM_DRIVER_OK != status)
                    {
                        BleApp_WriteToUART("ERROR : Failed to put MMA865x to Pedometer Mode!\r\n");
                    }
                    else
                    {
                        BleApp_WriteToUART("Successfully put MMA865x to Pedometer Mode.\r\n");
                        BleApp_WriteToHost("Waiting for Steps...\n");
                    }
                    break;
                default:
                    BleApp_WriteToUART("ERROR : Unknown Mode [%d]!\r\n", gReportMode);
                    break;
            }
            break;
        case ISSDK_STANDBY_MODE:
            switch (gReportMode)
            {
                case ISSDK_REPORT_NONE:
                    break;
                case ISSDK_REPORT_ACCELERATION:
                case ISSDK_REPORT_FREEFALL:
                case ISSDK_REPORT_PEDOMETER:
                case ISSDK_REPORT_ORIENTATION:
                    /* Put MMA865x into standby mode.*/
                    status = Sensor_I2C_Write(gMma865xDriver.pCommDrv, &gMma865xDriver.deviceInfo,
                                              gMma865xDriver.slaveAddress, cMma865xStopConfiguration);
                    if (ARM_DRIVER_OK != status)
                    {
                        BleApp_WriteToUART("ERROR : Failed to put MMA865x to Standby Mode!\r\n");
                    }
                    else
                    {
                        BleApp_WriteToUART("Successfully put MMA865x to Standby Mode.\r\n");
                    }
                    break;
                default:
                    BleApp_WriteToUART("ERROR : Unknown Mode [%d]!\r\n", gReportMode);
                    break;
            }
            break;
        default:
            BleApp_WriteToUART("ERROR : Unknown Option [%d]!\r\n", opCode);
            break;
    }
}

bool ISSDK_ProcessHostCommand(uint8_t *pCommand, uint16_t commandLength)
{
    bool result;
    int32_t status;
    char *pStr, byteStr[2];
    uint8_t target = ISSDK_REPORT_NONE, mode = ISSDK_REGISTER_NONE, offset, bytes, *pBuffer;
    const ISSDK_BLEcommand_t *hostCommand = (ISSDK_BLEcommand_t *)pCommand;

    /* Check if it is a RLI Interface command */
    if (0 == strncasecmp(hostCommand->rliCmd.tag, ISSDK_REGISTER_CMD_TAG, strlen(ISSDK_REGISTER_CMD_TAG)))
    {
        /* Parse RLI command for Target Device */
        if (0 == strncasecmp(hostCommand->rliCmd.device, ISSDK_REGISTER_ACCEL_TAG, strlen(ISSDK_REGISTER_ACCEL_TAG)))
        {
            target = ISSDK_REPORT_ACCELERATION;
            BleApp_WriteToUART("Target is MMA865x.\r\n");
        }
        if (target != ISSDK_REPORT_NONE)
        {
            /* Parse RLI command for required Action */
            if (0 ==
                strncasecmp(hostCommand->rliCmd.action, ISSDK_REGISTER_MODE_R_TAG, strlen(ISSDK_REGISTER_MODE_R_TAG)))
            {
                mode = ISSDK_REGISTER_READ;
                BleApp_WriteToUART("Mode is Read Register.\r\n");
            }
            else if (0 == strncasecmp(hostCommand->rliCmd.action, ISSDK_REGISTER_MODE_W_TAG,
                                      strlen(ISSDK_REGISTER_MODE_W_TAG)))
            {
                mode = ISSDK_REGISTER_WRITE;
                BleApp_WriteToUART("Mode is Write Register.\r\n");
            }
            if (mode != ISSDK_REGISTER_NONE)
            {
                /* Extract the Offset */
                offset = strtol(hostCommand->rliCmd.offset, NULL, 16);
                BleApp_WriteToUART("Offset is 0x%02X!\r\n", offset);
                if (mode == ISSDK_REGISTER_READ)
                {
                    /* Extract bytes to read */
                    sprintf(byteStr, "%c%c", *(hostCommand->rliCmd.bytes), *(hostCommand->rliCmd.bytes + 1));
                    bytes = strtol(byteStr, NULL, 16);
                    BleApp_WriteToUART("Bytes is 0x%02X!\r\n", bytes);
                    pBuffer = MEM_BufferAlloc(bytes);
                    pStr = MEM_BufferAlloc(bytes * 2);
                    if (pBuffer == NULL || pStr == NULL)
                    {
                        BleApp_WriteToUART("ERROR : Register Read buffer allocation Failed.\r\n");
                        return false;
                    }
                    /* Call Sensor Register Interface Function to read Sensor Registers */
                    status = MMA865x_RegisterInterface(mode, target, offset, bytes, pBuffer);
                    if (status != SENSOR_ERROR_NONE)
                    {
                        BleApp_WriteToUART("ERROR : Register Read Failed [0x%02X]!\r\n", offset);
                        result = false;
                    }
                    else
                    {
                        for (uint8_t i = 0; i < bytes; i++)
                        {
                            /* Convert Bytes to String for Display */
                            sprintf(pStr + i * 2, "%02X", pBuffer[i]);
                        }
                        BleApp_WriteToHost("Read [0x%02X] = [0x%s].\n", offset, pStr);
                        result = true;
                    }
                    MEM_BufferFree(pBuffer);
                    MEM_BufferFree(pStr);
                    return result;
                }
                if (mode == ISSDK_REGISTER_WRITE)
                {
                    /* Compute Bytes to write */
                    bytes = (commandLength - ISSDK_BLE_REG_CMD_OFFSET) / 2;
                    BleApp_WriteToUART("Bytes is 0x%02X!\r\n", bytes);
                    pBuffer = MEM_BufferAlloc(bytes);
                    pStr = MEM_BufferAlloc(bytes * 2);
                    if (pBuffer == NULL || pStr == NULL)
                    {
                        BleApp_WriteToUART("ERROR : Register Write buffer allocation Failed.\r\n");
                        return false;
                    }
                    for (uint8_t i = 0; i < bytes; i++)
                    {
                        /* Convert String to Bytes for writing */
                        sprintf(byteStr, "%c%c", *(hostCommand->rliCmd.bytes + i * 2),
                                *(hostCommand->rliCmd.bytes + i * 2 + 1));
                        pBuffer[i] = strtol(byteStr, NULL, 16);
                        sprintf(pStr + i * 2, "%02X", pBuffer[i]);
                    }
                    /* Call Sensor Register Interface Function to write Sensor Registers */
                    status = MMA865x_RegisterInterface(mode, target, offset, bytes, pBuffer);
                    if (status != SENSOR_ERROR_NONE)
                    {
                        BleApp_WriteToUART("ERROR : Register Write Failed [0x%02X]!\r\n", offset);
                        result = false;
                    }
                    else
                    {
                        BleApp_WriteToHost("Written [0x%02X] = [0x%s]\n", offset, pStr);
                        result = true;
                    }
                    MEM_BufferFree(pBuffer);
                    MEM_BufferFree(pStr);
                    return result;
                }
            }
        }
    }
    /* Check if it is a Data Mode command */
    else if (0 == strncasecmp(hostCommand->modeCmd.tag, ISSDK_MODE_CMD_TAG, strlen(ISSDK_MODE_CMD_TAG)))
    {
        /* Check for User Interface STOP command */
        if (0 == strncasecmp(hostCommand->modeCmd.mode, ISSDK_REPORT_NONE_STR, strlen(ISSDK_REPORT_NONE_STR)))
        {
            /* Change Sensor Configuration to STANDBY Mode */
            ISSDK_Configure(ISSDK_STANDBY_MODE);
            gReportMode = ISSDK_REPORT_NONE;

            BleApp_WriteToUART("Switched To Standby Mode.\r\n");
            return true;
        }
        /* Check for User Interface ACCEL START command */
        else if (0 == strncasecmp(hostCommand->modeCmd.mode, ISSDK_REPORT_ACCELERATION_STR,
                                  strlen(ISSDK_REPORT_ACCELERATION_STR)))
        {
            /* Change Sensor Configuration to Accelerometer Mode */
            ISSDK_Configure(ISSDK_STANDBY_MODE);
            gReportMode = ISSDK_REPORT_ACCELERATION;
            ISSDK_Configure(ISSDK_ACTIVE_MODE);

            BleApp_WriteToUART("Switched To Accelerometer Mode.\r\n");
            return true;
        }
        /* Check for User Interface ACCEL START FREEFALL command */
        else if (0 ==
                 strncasecmp(hostCommand->modeCmd.mode, ISSDK_REPORT_FREEFALL_STR, strlen(ISSDK_REPORT_FREEFALL_STR)))
        {
            /* Change Sensor Configuration to Accelerometer Mode */
            ISSDK_Configure(ISSDK_STANDBY_MODE);
            gReportMode = ISSDK_REPORT_FREEFALL;
            ISSDK_Configure(ISSDK_ACTIVE_MODE);

            BleApp_WriteToUART("Switched To Free-Fall Mode.\r\n");
            return true;
        }
        /* Check for User Interface ACCEL START ORIENTATION command */
        else if (0 == strncasecmp(hostCommand->modeCmd.mode, ISSDK_REPORT_ORIENTATION_STR,
                                  strlen(ISSDK_REPORT_ORIENTATION_STR)))
        {
            /* Change Sensor Configuration to Accelerometer Mode */
            ISSDK_Configure(ISSDK_STANDBY_MODE);
            gReportMode = ISSDK_REPORT_ORIENTATION;
            ISSDK_Configure(ISSDK_ACTIVE_MODE);

            BleApp_WriteToUART("Switched To Motion Detection Mode.\r\n");
            return true;
        }
        /* Check for User Interface ACCEL START PEDOMETER command */
        else if (0 ==
                 strncasecmp(hostCommand->modeCmd.mode, ISSDK_REPORT_PEDOMETER_STR, strlen(ISSDK_REPORT_PEDOMETER_STR)))
        {
            /* Change Sensor Configuration to Accelerometer Mode */
            ISSDK_Configure(ISSDK_STANDBY_MODE);
            gReportMode = ISSDK_REPORT_PEDOMETER;
            ISSDK_Configure(ISSDK_ACTIVE_MODE);

            BleApp_WriteToUART("Switched To Pedometer Mode.\r\n");
            return true;
        }
        /* Check for User Interface RAW Data Mode Switch command */
        else if (0 ==
                 strncasecmp(hostCommand->modeCmd.mode, ISSDK_REPORT_RAW_DATA_STR, strlen(ISSDK_REPORT_RAW_DATA_STR)))
        {
            gReportFormat = ISSDK_READ_RAW;
            BleApp_WriteToUART("Switching To RAW Data Format.\r\n");
            return true;
        }
        /* Check for User Interface Normal Data Mode Switch command */
        else if (0 ==
                 strncasecmp(hostCommand->modeCmd.mode, ISSDK_REPORT_CONV_DATA_STR, strlen(ISSDK_REPORT_CONV_DATA_STR)))
        {
            gReportFormat = ISSDK_READ_NORMAL;
            BleApp_WriteToUART("Switching To Converted Data Format.\r\n");
            return true;
        }
        /* Check for User Interface Streaming Mode Switch command */
        else if (0 == strncasecmp(hostCommand->modeCmd.mode, ISSDK_REPORT_STREAM_DATA_STR,
                                  strlen(ISSDK_REPORT_STREAM_DATA_STR)))
        {
            gReportFormat = ISSDK_READ_STREAM;
            BleApp_WriteToUART("Switching To Streaming Data Format.\r\n");
            return true;
        }
    }

    return false;
}
