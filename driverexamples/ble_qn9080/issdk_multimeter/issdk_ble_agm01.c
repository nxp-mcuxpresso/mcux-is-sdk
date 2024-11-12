/*
 * Copyright (c) 2015 - 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file issdk_ble_agm01.c
 * @brief The issdk_ble_agm01.c file implements the ISSDK AGM01 sensor driver
 *        example demonstration using BLE Wireless UART adapter.
 * Supported Modes:
 *   MODE   = 'STANDBY'       (7 Fixed Characters to Stop All Sensor Data)
 *   MODE   = 'ACCELEROMETER' (13 Fixed Characters to Start Accelerometer Data)
 *   MODE   = 'MAGNETOMETER'  (12 Fixed Characters to Start Magnetometer Data)
 *   MODE   = 'GYROSCOPE'     (9 Fixed Characters to Start Gyroscope Data)
 *   MODE   = 'ECOMPASS'      (8 Fixed Characters to Start eCompass Data)
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
#include "fxos8700_drv.h"
#include "fxas21002_drv.h"

/* ISSDK Interface Include */
#include "issdk_ble_interface.h"

/* BLE framework Includes */
#include "LED.h"
#include "MemManager.h"
#include "GPIO_Adapter.h"

/*******************************************************************************
 * Macro Definitions
 ******************************************************************************/
/* Sensor Data Lengths */
#define FXAS21002_DATA_SIZE (6)      /* 2 byte X,Y,Z */
#define FXOS8700_ACCEL_DATA_SIZE (6) /* 2 byte X,Y,Z ACCEL */
#define FXOS8700_MAG_DATA_SIZE (6)   /* 2 byte X,Y,Z MAG */
#define FXOS8700_DATA_SIZE (FXOS8700_ACCEL_DATA_SIZE + FXOS8700_MAG_DATA_SIZE)

/* Sensor Data Conversion Factors */
#define FXOS8700_ACCEL_CONV (0.000244) /* Will give 'g' */
#define FXOS8700_MAG_CONV (0.1)        /* Will give uT */
#define FXAS21002_GYRO_CONV (0.0625)   /* Will give d/s */

/* FF_MT freefall counter register values for High resolution Mode and ODR = 100Hz.
 * These values have been derived based on the MMA865x DataSheet and Application Note AN4070 for FXOS8700 (the same is
 * applicable to FXOS8700 too).
 * http://cache.freescale.com/files/sensors/doc/app_note/AN4070.pdf */
#define FF_A_FFMT_THS 0x03 /* Threshold Value. */
#define MT_A_FFMT_THS 0x15 /* Threshold Value. */
#define A_FFMT_COUNT 0x0A  /* Debounce count value. */
#define PL_COUNT 0x15      /* Debounce count value. */
#define ASLP_COUNTER 0x07  /* Auto Sleep after ~5s. */

/*******************************************************************************
 * Constants
 ******************************************************************************/
/*! Prepare the register write list to configure FXOS8700 Normal mode. */
const registerwritelist_t cFxos8700ConfigInterruptHybrid[] = {
    {FXOS8700_ASLP_COUNT, ASLP_COUNTER, 0},
    {FXOS8700_PL_COUNT, PL_COUNT, 0},
    {FXOS8700_CTRL_REG3, FXOS8700_CTRL_REG3_IPOL_ACTIVE_LOW | FXOS8700_CTRL_REG3_PP_OD_PUSH_PULL,
                         FXOS8700_CTRL_REG3_IPOL_MASK | FXOS8700_CTRL_REG3_PP_OD_MASK}, /*! Active Low, Push-Pull */
    {FXOS8700_CTRL_REG5, FXOS8700_CTRL_REG5_INT_CFG_DRDY_INT1 | FXOS8700_CTRL_REG5_INT_CFG_FFMT_INT1 |
                         FXOS8700_CTRL_REG5_INT_CFG_LNDPRT_INT1 | FXOS8700_CTRL_REG5_INT_CFG_ASLP_INT1,
                         FXOS8700_CTRL_REG5_INT_CFG_DRDY_MASK | FXOS8700_CTRL_REG5_INT_CFG_FFMT_MASK |
                         FXOS8700_CTRL_REG5_INT_CFG_LNDPRT_MASK | FXOS8700_CTRL_REG5_INT_CFG_ASLP_MASK},
    {FXOS8700_M_CTRL_REG1, FXOS8700_M_CTRL_REG1_M_ACAL_EN | FXOS8700_M_CTRL_REG1_M_HMS_HYBRID_MODE,
                           FXOS8700_M_CTRL_REG1_M_ACAL_MASK | FXOS8700_M_CTRL_REG1_M_HMS_MASK}, /*! Enable the Hybrid Mode. */
    {FXOS8700_M_CTRL_REG2, FXOS8700_M_CTRL_REG2_M_RST_CNT_DISABLE, FXOS8700_M_CTRL_REG2_M_RST_CNT_MASK}, /*! Automatic magnetic reset is disabled. */
    __END_WRITE_DATA__};

/*! @brief FXOS8700 Start INT Mode Register Write List */
const registerwritelist_t cFxos8700StartConfiguration[] = {
    {FXOS8700_CTRL_REG4, FXOS8700_CTRL_REG4_INT_EN_DRDY_EN, FXOS8700_CTRL_REG4_INT_EN_DRDY_MASK},
    {FXOS8700_CTRL_REG1, FXOS8700_CTRL_REG1_DR_HYBRID_6P25_HZ | FXOS8700_CTRL_REG1_ACTIVE_ACTIVE_MODE,
     FXOS8700_CTRL_REG1_DR_MASK | FXOS8700_CTRL_REG1_ACTIVE_MASK},
    __END_WRITE_DATA__};

/*! @brief Register Start Free-fall Mode Register Write List. */
const registerwritelist_t cFxos8700StartFreeFallConfiguration[] = {
    {FXOS8700_A_FFMT_COUNT, A_FFMT_COUNT, 0}, /* Debounce Counter */
    {FXOS8700_A_FFMT_THS, FF_A_FFMT_THS | FXOS8700_A_FFMT_THS_DBCNTM_MASK,
     FXOS8700_A_FFMT_THS_THS_MASK | FXOS8700_A_FFMT_THS_DBCNTM_MASK}, /* Threshold */
    {FXOS8700_A_FFMT_CFG,
     FXOS8700_A_FFMT_CFG_OAE_FREEFALL | FXOS8700_A_FFMT_CFG_ELE_EN | FXOS8700_A_FFMT_CFG_ZEFE_RAISE_EVENT |
         FXOS8700_A_FFMT_CFG_YEFE_RAISE_EVENT | FXOS8700_A_FFMT_CFG_XEFE_RAISE_EVENT,
     FXOS8700_A_FFMT_CFG_OAE_MASK | FXOS8700_A_FFMT_CFG_ELE_MASK | FXOS8700_A_FFMT_CFG_ZEFE_MASK |
         FXOS8700_A_FFMT_CFG_YEFE_MASK | FXOS8700_A_FFMT_CFG_XEFE_MASK},
    {FXOS8700_CTRL_REG4, FXOS8700_CTRL_REG4_INT_EN_FFMT_EN, FXOS8700_CTRL_REG4_INT_EN_FFMT_MASK},
    {FXOS8700_CTRL_REG1, FXOS8700_CTRL_REG1_DR_HYBRID_100_HZ | FXOS8700_CTRL_REG1_ACTIVE_ACTIVE_MODE,
     FXOS8700_CTRL_REG1_DR_MASK | FXOS8700_CTRL_REG1_ACTIVE_MASK},
    __END_WRITE_DATA__};

/*! @brief Register Start Motion-detect Mode Register Write List. */
const registerwritelist_t cFxos8700StartMotionConfiguration[] = {
    {FXOS8700_A_FFMT_THS, MT_A_FFMT_THS, FXOS8700_A_FFMT_THS_THS_MASK}, /* Threshold */
    {FXOS8700_A_FFMT_CFG, FXOS8700_A_FFMT_CFG_OAE_MOTION | FXOS8700_A_FFMT_CFG_ZEFE_RAISE_EVENT |
                              FXOS8700_A_FFMT_CFG_YEFE_RAISE_EVENT | FXOS8700_A_FFMT_CFG_XEFE_RAISE_EVENT,
     FXOS8700_A_FFMT_CFG_OAE_MASK | FXOS8700_A_FFMT_CFG_ZEFE_MASK | FXOS8700_A_FFMT_CFG_YEFE_MASK |
         FXOS8700_A_FFMT_CFG_XEFE_MASK},
    {FXOS8700_CTRL_REG4, FXOS8700_CTRL_REG4_INT_EN_FFMT_EN, FXOS8700_CTRL_REG4_INT_EN_FFMT_MASK},
    {FXOS8700_CTRL_REG1, FXOS8700_CTRL_REG1_DR_HYBRID_0P7813_HZ | FXOS8700_CTRL_REG1_ACTIVE_ACTIVE_MODE,
     FXOS8700_CTRL_REG1_DR_MASK | FXOS8700_CTRL_REG1_ACTIVE_MASK},
    __END_WRITE_DATA__};

/*! @brief Register Start Orientation-detect Mode Register Write List. */
const registerwritelist_t cFxos8700StartOrientationConfiguration[] = {
    {FXOS8700_PL_CFG, FXOS8700_PL_CFG_DBCNTM_CLEAR_MODE | FXOS8700_PL_CFG_PL_EN_ENABLE,
     FXOS8700_PL_CFG_PL_EN_MASK | FXOS8700_PL_CFG_DBCNTM_MASK},
    {FXOS8700_CTRL_REG4, FXOS8700_CTRL_REG4_INT_EN_LNDPRT_EN | FXOS8700_CTRL_REG4_INT_EN_ASLP_EN,
     FXOS8700_CTRL_REG4_INT_EN_LNDPRT_MASK | FXOS8700_CTRL_REG4_INT_EN_ASLP_MASK},
    {FXOS8700_CTRL_REG3, FXOS8700_CTRL_REG3_WAKE_LNDPRT_EN, FXOS8700_CTRL_REG3_WAKE_LNDPRT_MASK},
    {FXOS8700_CTRL_REG2, FXOS8700_CTRL_REG2_SLPE_EN, FXOS8700_CTRL_REG2_SLPE_MASK},
    {FXOS8700_CTRL_REG1, FXOS8700_CTRL_REG1_DR_HYBRID_3P125_HZ | FXOS8700_CTRL_REG1_ACTIVE_ACTIVE_MODE,
     FXOS8700_CTRL_REG1_DR_MASK | FXOS8700_CTRL_REG1_ACTIVE_MASK},
    __END_WRITE_DATA__};

/*! @brief FXOS8700 Start Pedometer Mode Register Write List */
const registerwritelist_t cFxos8700StartPedometerConfiguration[] = {
    /*! Prepare the register write list to configure FXOS8700 in Normal mode. */
    {FXOS8700_CTRL_REG4, FXOS8700_CTRL_REG4_INT_EN_DRDY_EN, FXOS8700_CTRL_REG4_INT_EN_DRDY_MASK},
    {FXOS8700_CTRL_REG1, FXOS8700_CTRL_REG1_DR_HYBRID_50_HZ | FXOS8700_CTRL_REG1_ACTIVE_ACTIVE_MODE,
                         FXOS8700_CTRL_REG1_DR_MASK | FXOS8700_CTRL_REG1_ACTIVE_MASK},
    __END_WRITE_DATA__};

/*! @brief FXOS8700 Stop Register Write List */
const registerwritelist_t cFxos8700StopConfiguration[] = {
    /* Clear Active Mode. */
    {FXOS8700_CTRL_REG1, FXOS8700_CTRL_REG1_ACTIVE_STANDBY_MODE, FXOS8700_CTRL_REG1_ACTIVE_MASK},
    {FXOS8700_CTRL_REG2, FXOS8700_CTRL_REG2_SLPE_DISABLE, FXOS8700_CTRL_REG2_SLPE_MASK},
    {FXOS8700_CTRL_REG3, FXOS8700_CTRL_REG3_WAKE_LNDPRT_DIS, FXOS8700_CTRL_REG3_WAKE_LNDPRT_MASK},
    {FXOS8700_A_FFMT_THS, 0x00, FXOS8700_A_FFMT_THS_THS_MASK}, /* Threshold */
    {FXOS8700_CTRL_REG4, 0x00, 0},
    {FXOS8700_A_FFMT_COUNT, 0x00, 0},
    {FXOS8700_A_FFMT_CFG, 0x00, 0},
    {FXOS8700_PL_CFG, 0x00, 0},
    __END_WRITE_DATA__};

/*! Command definition to read the Accel+Mag Data. */
const registerreadlist_t cFxos8700ReadData[] = {{.readFrom = FXOS8700_OUT_X_MSB, .numBytes = FXOS8700_ACCEL_DATA_SIZE},
                                                {.readFrom = FXOS8700_M_OUT_X_MSB, .numBytes = FXOS8700_MAG_DATA_SIZE},
                                                __END_READ_DATA__};

/*! @brief Address of Free-Fall Status Register. */
const registerreadlist_t cFxos8700FFMTSrc[] = {{.readFrom = FXOS8700_A_FFMT_SRC, .numBytes = 1}, __END_READ_DATA__};

/*! @brief Address of INT Source Register. */
const registerreadlist_t cFxos8700INTSrc[] = {{.readFrom = FXOS8700_INT_SOURCE, .numBytes = 1}, __END_READ_DATA__};

/*! @brief Address of PL Status Register. */
const registerreadlist_t cFxos8700OutputPLStatus[] = {{.readFrom = FXOS8700_PL_STATUS, .numBytes = 1},
                                                      __END_READ_DATA__};

/*! Prepare the register write list to configure FXAS21002 Normal mode. */
const registerwritelist_t fxas21002_Config_Normal[] = {
    /*! Configure CTRL_REG1 register to put FXAS21002 to 12.5Hz sampling rate. */
    {FXAS21002_CTRL_REG1, FXAS21002_CTRL_REG1_DR_12_5HZ, FXAS21002_CTRL_REG1_DR_MASK},
    /*! Configure CTRL_REG2 register to set interrupt configuration settings. */
    {FXAS21002_CTRL_REG2, FXAS21002_CTRL_REG2_IPOL_ACTIVE_HIGH | FXAS21002_CTRL_REG2_INT_EN_DRDY_ENABLE |
                              FXAS21002_CTRL_REG2_INT_CFG_DRDY_INT1,
     FXAS21002_CTRL_REG2_IPOL_MASK | FXAS21002_CTRL_REG2_INT_EN_DRDY_MASK | FXAS21002_CTRL_REG2_INT_CFG_DRDY_MASK},
    __END_WRITE_DATA__};

/*! @brief FXAS21002 Start Register Write List */
const registerwritelist_t gFxas21002StartConfiguration[] = {
    {FXAS21002_CTRL_REG1, FXAS21002_CTRL_REG1_MODE_ACTIVE, FXAS21002_CTRL_REG1_MODE_MASK}, // Set ACTIVE Bit
    __END_WRITE_DATA__};

/*! @brief FXAS21002 Stop Register Write List */
const registerwritelist_t gFxas21002StopConfiguration[] = {
    {FXAS21002_CTRL_REG1, FXAS21002_CTRL_REG1_MODE_STANDBY, FXAS21002_CTRL_REG1_MODE_MASK}, // Clear ACTIVE Bit
    __END_WRITE_DATA__};

/*! Prepare the register read list to read the raw gyro data from the FXAS21002. */
const registerreadlist_t fxas21002_Output_Values[] = {
    {.readFrom = FXAS21002_OUT_X_MSB, .numBytes = FXAS21002_DATA_SIZE}, __END_READ_DATA__};

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
ARM_DRIVER_I2C *gI2cDriver = &I2C_S_DRIVER;
fxos8700_i2c_sensorhandle_t gFxos8700Driver;
fxas21002_i2c_sensorhandle_t gFxas21002Driver;
gpioInputPinConfig_t gFxos8700IntCfg, gFxas21002IntCfg;

/************************************************************************************
* Functions
************************************************************************************/
/*! @brief Function to Read Acceleration Data */
static void FXOS8700_ReadAcceleration(void)
{
    float accelX, accelY, accelZ;
    int32_t status, accelXi, accelYi, accelZi;
    char accelXs[8] = {0}, accelYs[8] = {0}, accelZs[8] = {0};
    fxos8700_accelmagdata_t rawData;
    uint8_t data[FXOS8700_DATA_SIZE];

    /*! Read the raw sensor data from the FXOS8700. */
    status = FXOS8700_I2C_ReadData(&gFxos8700Driver, cFxos8700ReadData, data);
    if (ARM_DRIVER_OK != status)
    {
        BleApp_WriteToUART("ERROR : Read Acceleration Failed!\r\n");
        return;
    }

    /*! Convert the raw sensor data to signed 16-bit container for display to the debug port. */
    rawData.accel[0] = ((int16_t)data[0] << 8) | data[1];
    rawData.accel[0] /= 4;
    rawData.accel[1] = ((int16_t)data[2] << 8) | data[3];
    rawData.accel[1] /= 4;
    rawData.accel[2] = ((int16_t)data[4] << 8) | data[5];
    rawData.accel[2] /= 4;
    accelX = FXOS8700_ACCEL_CONV * rawData.accel[0];
    accelY = FXOS8700_ACCEL_CONV * rawData.accel[1];
    accelZ = FXOS8700_ACCEL_CONV * rawData.accel[2];

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

/*! @brief Function to Read Magnetic Data */
static void FXOS8700_ReadMagnetic(void)
{
    float magX, magY, magZ;
    int32_t status, magXi, magYi, magZi;
    char magXs[8] = {0}, magYs[8] = {0}, magZs[8] = {0};
    fxos8700_accelmagdata_t rawData;
    uint8_t data[FXOS8700_DATA_SIZE];

    /*! Read the raw sensor data from the FXOS8700. */
    status = FXOS8700_I2C_ReadData(&gFxos8700Driver, cFxos8700ReadData, data);
    if (ARM_DRIVER_OK != status)
    {
        BleApp_WriteToUART("ERROR : Read Magnetic Failed!\r\n");
        return;
    }

    /*! Convert the raw sensor data to signed 16-bit container for display to the debug port. */
    rawData.mag[0] = ((int16_t)data[6] << 8) | data[7];
    rawData.mag[1] = ((int16_t)data[8] << 8) | data[9];
    rawData.mag[2] = ((int16_t)data[10] << 8) | data[11];
    magX = FXOS8700_MAG_CONV * rawData.mag[0];
    magY = FXOS8700_MAG_CONV * rawData.mag[1];
    magZ = FXOS8700_MAG_CONV * rawData.mag[2];

    if (gReportFormat == ISSDK_READ_RAW)
    {
        /* Send raw data over the air */
        BleApp_WriteToHost("MAG: X=0x%04X Y=0x%04X Z=0x%04X\n", (uint16_t)rawData.mag[0], (uint16_t)rawData.mag[1],
                           (uint16_t)rawData.mag[2]);
    }
    if (gReportFormat == ISSDK_READ_NORMAL)
    {
        ISSDK_GetStringOfFloat(magX, magXs);
        ISSDK_GetStringOfFloat(magY, magYs);
        ISSDK_GetStringOfFloat(magZ, magZs);

        /* Send data over the air */
        BleApp_WriteToHost("MAG: X=%suT Y=%suT Z=%suT\n", magXs, magYs, magZs);
    }
    if (gReportFormat == ISSDK_READ_STREAM)
    {
        magXi = ISSDK_GetDecimalForFloat(magX);
        magYi = ISSDK_GetDecimalForFloat(magY);
        magZi = ISSDK_GetDecimalForFloat(magZ);
        /* Send Data Stream over the air */
        BleApp_WriteToHost("%s:%08X:%08X:%08X\n", ISSDK_MAG_STREAM_HDR_STR, (uint32_t)magXi, (uint32_t)magYi,
                           (uint32_t)magZi);
    }
}

/*! @brief Function to Read Acceleration + Magnetic Data */
static void FXOS8700_ReadCombo(void)
{
    float accelX, accelY, accelZ, magX, magY, magZ;
    int32_t status, accelXi, accelYi, accelZi, magXi, magYi, magZi;
    char accelXs[8] = {0}, accelYs[8] = {0}, accelZs[8] = {0},
         magXs[8] = {0}, magYs[8] = {0}, magZs[8] = {0};

    fxos8700_accelmagdata_t rawData;
    uint8_t data[FXOS8700_DATA_SIZE];

    /*! Read the raw sensor data from the FXOS8700. */
    status = FXOS8700_I2C_ReadData(&gFxos8700Driver, cFxos8700ReadData, data);
    if (ARM_DRIVER_OK != status)
    {
        BleApp_WriteToUART("ERROR : Read eCompass Failed!\r\n");
        return;
    }

    /*! Convert the raw sensor data to signed 16-bit container for display to the debug port. */
    rawData.accel[0] = ((int16_t)data[0] << 8) | data[1];
    rawData.accel[0] /= 4;
    rawData.accel[1] = ((int16_t)data[2] << 8) | data[3];
    rawData.accel[1] /= 4;
    rawData.accel[2] = ((int16_t)data[4] << 8) | data[5];
    rawData.accel[2] /= 4;
    rawData.mag[0]   = ((int16_t)data[6] << 8) | data[7];
    rawData.mag[1]   = ((int16_t)data[8] << 8) | data[9];
    rawData.mag[2]   = ((int16_t)data[10] << 8) | data[11];
    accelX = FXOS8700_ACCEL_CONV * rawData.accel[0];
    accelY = FXOS8700_ACCEL_CONV * rawData.accel[1];
    accelZ = FXOS8700_ACCEL_CONV * rawData.accel[2];
    magX   = FXOS8700_MAG_CONV * rawData.mag[0];
    magY   = FXOS8700_MAG_CONV * rawData.mag[1];
    magZ   = FXOS8700_MAG_CONV * rawData.mag[2];

    if (gReportFormat == ISSDK_READ_RAW)
    {
        /* Send raw data over the air */
        BleApp_WriteToHost("ACCEL: X=0x%04X Y=0x%04X Z=0x%04X\n", (uint16_t)rawData.accel[0],
                           (uint16_t)rawData.accel[1], (uint16_t)rawData.accel[2]);
        BleApp_WriteToHost("MAG: X=0x%04X Y=0x%04X Z=0x%04X\n", (uint16_t)rawData.mag[0],
                           (uint16_t)rawData.mag[1], (uint16_t)rawData.mag[2]);
    }
    if (gReportFormat == ISSDK_READ_NORMAL)
    {
        ISSDK_GetStringOfFloat(accelX, accelXs);
        ISSDK_GetStringOfFloat(accelY, accelYs);
        ISSDK_GetStringOfFloat(accelZ, accelZs);
        ISSDK_GetStringOfFloat(magX, magXs);
        ISSDK_GetStringOfFloat(magY, magYs);
        ISSDK_GetStringOfFloat(magZ, magZs);

        /* Send data over the air */
        BleApp_WriteToHost("ACCEL: X=%sg Y=%sg Z=%sg\n", accelXs, accelYs, accelZs);
        BleApp_WriteToHost("MAG: X=%suT Y=%suT Z=%suT\n", magXs, magYs, magZs);
    }
    if (gReportFormat == ISSDK_READ_STREAM)
    {
        accelXi = ISSDK_GetDecimalForFloat(accelX);
        accelYi = ISSDK_GetDecimalForFloat(accelY);
        accelZi = ISSDK_GetDecimalForFloat(accelZ);
        magXi   = ISSDK_GetDecimalForFloat(magX);
        magYi   = ISSDK_GetDecimalForFloat(magY);
        magZi   = ISSDK_GetDecimalForFloat(magZ);

        /* Send Data Stream over the air */
        BleApp_WriteToHost("%s:%08X:%08X:%08X:%08X:%08X:%08X\n", ISSDK_COMBO_STREAM_HDR_STR,
                           (uint32_t)accelXi, (uint32_t)accelYi, (uint32_t)accelZi,
                           (uint32_t)magXi, (uint32_t)magYi, (uint32_t)magZi);
    }
}

/*! @brief Function to Read Pedometer Data */
void FXOS8700_ReadPedometer(void)
{
    char speedS[8];

    /* Log the last reported steps. */
    gLastReportedSteps = gPedometer.status.stepcount;
    ISSDK_GetStringOfFloat(gPedometer.status.speed / 1000.0, speedS);

    /* Send data over the air */
    BleApp_WriteToHost("STEPS:%d | DISTANCE:%dm | SPEED:%skm/h\n", gPedometer.status.stepcount,
                       gPedometer.status.distance, speedS);
}

/*! @brief Function to Read Rotational Data */
static void FXAS21002_ReadRotation(void)
{
    float gyroX, gyroY, gyroZ;
    int32_t status, gyroXi, gyroYi, gyroZi;
    char gyroXs[8] = {0}, gyroYs[8] = {0}, gyroZs[8] = {0};
    fxas21002_gyrodata_t rawData;
    uint8_t data[FXAS21002_DATA_SIZE];

    /*! Read the raw sensor data from the FXAS21002. */
    status = FXAS21002_I2C_ReadData(&gFxas21002Driver, fxas21002_Output_Values, data);
    if (ARM_DRIVER_OK != status)
    {
        BleApp_WriteToUART("ERROR : Read Rotation Failed!\r\n");
        return;
    }

    /*! Convert the raw sensor data to signed 16-bit container for display to the debug port. */
    rawData.gyro[0] = ((int16_t)data[0] << 8) | data[1];
    rawData.gyro[1] = ((int16_t)data[2] << 8) | data[3];
    rawData.gyro[2] = ((int16_t)data[4] << 8) | data[5];
    gyroX = FXAS21002_GYRO_CONV * rawData.gyro[0];
    gyroY = FXAS21002_GYRO_CONV * rawData.gyro[1];
    gyroZ = FXAS21002_GYRO_CONV * rawData.gyro[2];

    if (gReportFormat == ISSDK_READ_RAW)
    {
        /* Send raw data over the air */
        BleApp_WriteToHost("GYRO: X=0x%04X Y=0x%04X Z=0x%04X\n", (uint16_t)rawData.gyro[0], (uint16_t)rawData.gyro[1],
                           (uint16_t)rawData.gyro[2]);
    }
    if (gReportFormat == ISSDK_READ_NORMAL)
    {
        ISSDK_GetStringOfFloat(gyroX, gyroXs);
        ISSDK_GetStringOfFloat(gyroY, gyroYs);
        ISSDK_GetStringOfFloat(gyroZ, gyroZs);

        /* Send data over the air */
        BleApp_WriteToHost("GYRO: X=%sd/s Y=%sd/s Z=%sd/s\n", gyroXs, gyroYs, gyroZs);
    }
    if (gReportFormat == ISSDK_READ_STREAM)
    {
        gyroXi = ISSDK_GetDecimalForFloat(gyroX);
        gyroYi = ISSDK_GetDecimalForFloat(gyroY);
        gyroZi = ISSDK_GetDecimalForFloat(gyroZ);
        /* Send Data Stream over the air */
        BleApp_WriteToHost("%s:%08X:%08X:%08X\n", ISSDK_GYRO_STREAM_HDR_STR, (uint32_t)gyroXi, (uint32_t)gyroYi,
                           (uint32_t)gyroZi);
    }
}

/*! @brief Function to Read and Write Sensor registers */
static int32_t AGM01_RegisterInterface(uint8_t opCode, uint8_t mode, uint8_t offset, uint8_t bytes, uint8_t *buffer)
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
                case ISSDK_REPORT_MAGNETIC:
                    /*! Read FXOS8700 registers. */
                    result = Register_I2C_Read(gFxos8700Driver.pCommDrv, &gFxos8700Driver.deviceInfo,
                                               gFxos8700Driver.slaveAddress, offset, bytes, buffer);
                    if (result != SENSOR_ERROR_NONE)
                    {
                        BleApp_WriteToUART("ERROR : Read [%d] bytes form [0x%02X] of FXOS8700!\r\n", offset, bytes);
                    }
                    break;
                case ISSDK_REPORT_ROTATION:
                    /*! Read FXAS21002 registers. */
                    result = Register_I2C_Read(gFxas21002Driver.pCommDrv, &gFxas21002Driver.deviceInfo,
                                               gFxas21002Driver.slaveAddress, offset, bytes, buffer);
                    if (result != SENSOR_ERROR_NONE)
                    {
                        BleApp_WriteToUART("ERROR : Read [%d] bytes form [0x%02X] of FXAS21002!\r\n", offset, bytes);
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
                case ISSDK_REPORT_MAGNETIC:
                    /*! Write FXOS8700 registers. */
                    result = Register_I2C_BlockWrite(gFxos8700Driver.pCommDrv, &gFxos8700Driver.deviceInfo,
                                                     gFxos8700Driver.slaveAddress, offset, buffer, bytes);
                    if (result != SENSOR_ERROR_NONE)
                    {
                        BleApp_WriteToUART("ERROR : Write [%d] bytes to [0x%02X] of FXOS8700!\r\n", offset, bytes);
                    }
                    break;
                case ISSDK_REPORT_ROTATION:
                    /*! Write FXAS21002 registers. */
                    result = Register_I2C_BlockWrite(gFxas21002Driver.pCommDrv, &gFxas21002Driver.deviceInfo,
                                                     gFxas21002Driver.slaveAddress, offset, buffer, bytes);
                    if (result != SENSOR_ERROR_NONE)
                    {
                        BleApp_WriteToUART("ERROR : Write [%d] bytes to [0x%02X] of FXAS21002!\r\n", offset, bytes);
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

void FXOS8700_ISR_Callback(void)
{
    int32_t status;
    fxos8700_acceldata_t rawData;
    uint8_t dataReady, data[FXOS8700_DATA_SIZE];

    if (GpioIsPinIntPending(&gFxos8700IntCfg))
    {
        if (gReportMode == ISSDK_REPORT_FREEFALL)
        {
            /*! Read the Freefall event FLAGs from FXOS8700. */
            status = FXOS8700_I2C_ReadData(&gFxos8700Driver, cFxos8700FFMTSrc, &dataReady);
            if (SENSOR_ERROR_NONE != status)
            {
                BleApp_WriteToUART("ERROR : Read FXOS8700 Event Failed!\r\n");
                return;
            }

            if (0 == (dataReady & FXOS8700_A_FFMT_SRC_EA_MASK))
            { /* Return, if new event is not detected. */
                return;
            }

            /*! Display that a freefall event has been detected. */
            BleApp_WriteToUART("EVENT : Freefall detected...\r\n");
        }
        if (gReportMode == ISSDK_REPORT_PEDOMETER)
        {
            /*! Read all the raw sensor data from the fxos8700. */
            status = FXOS8700_I2C_ReadData(&gFxos8700Driver, cFxos8700ReadData, data);
            if (SENSOR_ERROR_NONE != status)
            {
                BleApp_WriteToUART("ERROR : Read FXOS8700 Data Failed!\r\n");
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
                FXOS8700_ReadPedometer();
                Led2Toggle();
            }
        }
        if (gReportMode == ISSDK_REPORT_ORIENTATION)
        {
            if (gMotionDetect)
            {
                /*! Read the FFMT Status from the FXOS8700. */
                status = FXOS8700_I2C_ReadData(&gFxos8700Driver, cFxos8700FFMTSrc, &dataReady);
                if (ARM_DRIVER_OK != status)
                {
                    BleApp_WriteToUART("ERROR : Read FXOS8700 FFMT Failed!\r\n");
                    return;
                }

                if ((dataReady & FXOS8700_A_FFMT_SRC_EA_MASK) == 0x80)
                {
                    /*! Display that a Motion event has been detected. */
                    BleApp_WriteToUART("EVENT : Motion detected...\r\n");

                    /* Apply FXOS8700 Stop Mode Configuration. */
                    status = Sensor_I2C_Write(gFxos8700Driver.pCommDrv, &gFxos8700Driver.deviceInfo,
                                              gFxos8700Driver.slaveAddress, cFxos8700StopConfiguration);
                    if (SENSOR_ERROR_NONE != status)
                    {
                        BleApp_WriteToUART("ERROR : Write FXOS8700 Stop Configuration!\r\n");
                        return;
                    }

                    /* Apply FXOS8700 Configuration for Orientation Detection. */
                    status = Sensor_I2C_Write(gFxos8700Driver.pCommDrv, &gFxos8700Driver.deviceInfo,
                                              gFxos8700Driver.slaveAddress, cFxos8700StartOrientationConfiguration);
                    if (SENSOR_ERROR_NONE != status)
                    {
                        BleApp_WriteToUART("ERROR : Write FXOS8700 Orientation Configuration Failed!\r\n");
                        return;
                    }
                    gMotionDetect = false;
                }
            }
            else
            {
                /*! Read the Orientation Status from the FXOS8700. */
                status = FXOS8700_I2C_ReadData(&gFxos8700Driver, cFxos8700OutputPLStatus, &dataReady);
                if (ARM_DRIVER_OK != status)
                {
                    BleApp_WriteToUART("ERROR : Read Orientation Failed!\r\n");
                    return;
                }

                if (((dataReady & FXOS8700_PL_STATUS_NEWLP_MASK) == 0x80) &&
                    ((dataReady & FXOS8700_PL_STATUS_LO_MASK) == 0x00))
                {
                    if ((dataReady & FXOS8700_PL_STATUS_LAPO_MASK) == 0x00)
                    {
                        BleApp_WriteToUART("EVENT : Portrait Up ...\r\n");
                    }
                    if ((dataReady & FXOS8700_PL_STATUS_LAPO_MASK) == 0x02)
                    {
                        BleApp_WriteToUART("EVENT : Portrait Down ...\r\n");
                    }
                    if ((dataReady & FXOS8700_PL_STATUS_LAPO_MASK) == 0x04)
                    {
                        BleApp_WriteToUART("EVENT : Landscape Right ...\r\n");
                    }
                    if ((dataReady & FXOS8700_PL_STATUS_LAPO_MASK) == 0x06)
                    {
                        BleApp_WriteToUART("EVENT : Landscape Left ...\r\n");
                    }
                }

                if (((dataReady & FXOS8700_PL_STATUS_NEWLP_MASK) == 0x80) &&
                    ((dataReady & FXOS8700_PL_STATUS_LO_MASK) == 0x40))
                {
                    if ((dataReady & FXOS8700_PL_STATUS_BAFRO_MASK) == 0x00)
                    {
                        BleApp_WriteToUART("EVENT : Front Side ...\r\n");
                    }
                    if ((dataReady & FXOS8700_PL_STATUS_BAFRO_MASK) == 0x01)
                    {
                        BleApp_WriteToUART("EVENT : Back Side ...\r\n");
                    }
                }

                status = FXOS8700_I2C_ReadData(&gFxos8700Driver, cFxos8700INTSrc, &dataReady);
                if (ARM_DRIVER_OK != status)
                {
                    BleApp_WriteToUART("ERROR : Read INT SRC Failed!\r\n");
                    return;
                }
                if ((dataReady & FXOS8700_INT_SOURCE_SRC_ASLP_MASK) == FXOS8700_INT_SOURCE_SRC_ASLP_MASK)
                {
                    /* Apply FXOS8700 Stop Mode Configuration. */
                    status = Sensor_I2C_Write(gFxos8700Driver.pCommDrv, &gFxos8700Driver.deviceInfo,
                                              gFxos8700Driver.slaveAddress, cFxos8700StopConfiguration);
                    if (SENSOR_ERROR_NONE != status)
                    {
                        BleApp_WriteToUART("ERROR : Write FXOS8700 Stop Configuration!\r\n");
                        return;
                    }

                    /* Apply FXOS8700 Configuration for Motion Detection. */
                    status = Sensor_I2C_Write(gFxos8700Driver.pCommDrv, &gFxos8700Driver.deviceInfo,
                                              gFxos8700Driver.slaveAddress, cFxos8700StartMotionConfiguration);
                    if (SENSOR_ERROR_NONE != status)
                    {
                        BleApp_WriteToUART("ERROR : Write FXOS8700 Motion Configuration Failed!\r\n");
                        return;
                    }
                    gMotionDetect = true;
                    BleApp_WriteToUART("EVENT : Going into Inactive Mode ...\r\n");
                }
            }
        }
        if (gReportMode == ISSDK_REPORT_ACCELERATION)
        {
            FXOS8700_ReadAcceleration();
            Led2Toggle();
        }
        if (gReportMode == ISSDK_REPORT_MAGNETIC)
        {
            FXOS8700_ReadMagnetic();
            Led2Toggle();
        }
        if (gReportMode == ISSDK_REPORT_ECOMPASS)
        {
            FXOS8700_ReadCombo();
            Led2Toggle();
        }		
        GpioInputPinInit(&gFxos8700IntCfg, 1);
        GpioClearPinIntFlag(&gFxos8700IntCfg);
    }
}

void FXAS21002_ISR_Callback(void)
{
    if (GpioIsPinIntPending(&gFxas21002IntCfg))
    {
        if (gReportMode == ISSDK_REPORT_ROTATION)
        {
            FXAS21002_ReadRotation();
            Led2Toggle();
        }
        GpioInputPinInit(&gFxas21002IntCfg, 1);
        GpioClearPinIntFlag(&gFxas21002IntCfg);
    }
}

void ISSDK_Initialize(void)
{
    int32_t status;

    BleApp_WriteToUART("Initializing Sensors for ISSDK AGM01 BLE example.\r\n");

    /*! Initialize the I2C driver. */
    status = gI2cDriver->Initialize(I2C_S_SIGNAL_EVENT);
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

    /*! Initialize FXOS8700 sensor driver. */
    status = FXOS8700_I2C_Initialize(&gFxos8700Driver, &I2C_S_DRIVER, I2C_S_DEVICE_INDEX, FXOS8700_I2C_ADDR,
                                     FXOS8700_WHO_AM_I_PROD_VALUE);
    if (SENSOR_ERROR_NONE != status)
    {
        BleApp_WriteToUART("ERROR : FXOS8700 Initialization Failed!\r\n");
        return;
    }

    FXOS8700_I2C_SetIdleTask(&gFxos8700Driver, (registeridlefunction_t)SMC_SetPowerModeVlpr, SMC);
    /* Apply FXOS8700 Configuration based on the Register List */
    status = Sensor_I2C_Write(gFxos8700Driver.pCommDrv, &gFxos8700Driver.deviceInfo, gFxos8700Driver.slaveAddress,
                              cFxos8700ConfigInterruptHybrid);
    if (ARM_DRIVER_OK != status)
    {
        BleApp_WriteToUART("ERROR : FXOS8700 Configuration Failed!\r\n");
        return;
    }
    BleApp_WriteToUART("Successfully Initialized FXOS8700.\r\n");

    /*! Initialize the FXAS21002 sensor driver. */
    status = FXAS21002_I2C_Initialize(&gFxas21002Driver, &I2C_S_DRIVER, I2C_S_DEVICE_INDEX, FXAS21002_I2C_ADDR,
                                      FXAS21002_WHO_AM_I_WHOAMI_PROD_VALUE);
    if (SENSOR_ERROR_NONE != status)
    {
        BleApp_WriteToUART("ERROR : FXAS21002 Initialization Failed!\r\n");
        return;
    }

    FXAS21002_I2C_SetIdleTask(&gFxas21002Driver, (registeridlefunction_t)SMC_SetPowerModeVlpr, SMC);
    /* Apply FXAS21002 Configuration based on the Register List */
    status = Sensor_I2C_Write(gFxas21002Driver.pCommDrv, &gFxas21002Driver.deviceInfo, gFxas21002Driver.slaveAddress,
                              fxas21002_Config_Normal);
    if (ARM_DRIVER_OK != status)
    {
        BleApp_WriteToUART("ERROR : FXAS21002 Configuration Failed!\r\n");
        return;
    }
    BleApp_WriteToUART("Successfully Initialized FXAS21002.\r\n");
    
    /* Initialize the pedometer */
    pedometer_init(&gPedometer);

    /* Initialize GPIO Configuration */
    gFxos8700IntCfg.pullSelect = pinPull_Down_c;
    gFxos8700IntCfg.interruptSelect = pinInt_LogicZero_c;
    gFxos8700IntCfg.gpioPin = FXOS8700_INT1.pinNumber;
    gFxos8700IntCfg.gpioPort = (gpioPort_t)FXOS8700_INT1.portNumber;
    /* Install FXOS8700 Event Callback INT */
    GpioInstallIsr(FXOS8700_ISR_Callback, gGpioIsrPrioNormal_c, gGpioDefaultNvicPrio_c, &gFxos8700IntCfg);
    GpioInputPinInit(&gFxos8700IntCfg, 1);

    gFxas21002IntCfg.pullSelect = pinPull_Up_c;
    gFxas21002IntCfg.interruptSelect = pinInt_LogicOne_c;
    gFxas21002IntCfg.gpioPin = FXAS21002_INT1.pinNumber;
    gFxas21002IntCfg.gpioPort = (gpioPort_t)FXAS21002_INT1.portNumber;
    GpioInstallIsr(FXAS21002_ISR_Callback, gGpioIsrPrioNormal_c, gGpioDefaultNvicPrio_c, &gFxas21002IntCfg);
    GpioInputPinInit(&gFxas21002IntCfg, 1);

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
                case ISSDK_REPORT_MAGNETIC:
                case ISSDK_REPORT_ECOMPASS:
                    /* Put FXOS8700 device into active mode.*/
                    status = Sensor_I2C_Write(gFxos8700Driver.pCommDrv, &gFxos8700Driver.deviceInfo,
                                              gFxos8700Driver.slaveAddress, cFxos8700StartConfiguration);
                    if (ARM_DRIVER_OK != status)
                    {
                        BleApp_WriteToUART("ERROR : Failed to put FXOS8700 to Active Mode!\r\n");
                    }
                    else
                    {
                        BleApp_WriteToUART("Successfully put FXOS8700 to Active Mode.\r\n");
                    }
                    break;
                case ISSDK_REPORT_FREEFALL:
                    /* Put FXOS8700 into FreeFall detection mode.*/
                    status = Sensor_I2C_Write(gFxos8700Driver.pCommDrv, &gFxos8700Driver.deviceInfo,
                                              gFxos8700Driver.slaveAddress, cFxos8700StartFreeFallConfiguration);
                    if (ARM_DRIVER_OK != status)
                    {
                        BleApp_WriteToUART("ERROR : Failed to put FXOS8700 to FreeFall Mode!\r\n");
                    }
                    else
                    {
                        BleApp_WriteToUART("Successfully put FXOS8700 to FreeFall Mode.\r\n");
                        BleApp_WriteToHost("Waiting for Freefall Event...\n");
                    }
                    break;
                case ISSDK_REPORT_ORIENTATION:
                    /* Put FXOS8700 into Motion detection mode.*/
                    status = Sensor_I2C_Write(gFxos8700Driver.pCommDrv, &gFxos8700Driver.deviceInfo,
                                              gFxos8700Driver.slaveAddress, cFxos8700StartMotionConfiguration);
                    if (ARM_DRIVER_OK != status)
                    {
                        BleApp_WriteToUART("ERROR : Failed to put FXOS8700 to Motion Detect Mode!\r\n");
                    }
                    else
                    {
                        BleApp_WriteToUART("Successfully put FXOS8700 to Motion Detect Mode.\r\n");
                        BleApp_WriteToHost("Waiting for Motion Event...\n");
                        gMotionDetect = true;
                    }
                    break;
                case ISSDK_REPORT_PEDOMETER:
                    /* Configure the pedometer algorithm */
                    pedometer_configure(&gPedometer, &cPedoConfig);

                    /* Put FXOS8700 into Pedometer detection mode.*/
                    status = Sensor_I2C_Write(gFxos8700Driver.pCommDrv, &gFxos8700Driver.deviceInfo,
                                              gFxos8700Driver.slaveAddress, cFxos8700StartPedometerConfiguration);
                    if (ARM_DRIVER_OK != status)
                    {
                        BleApp_WriteToUART("ERROR : Failed to put FXOS8700 to Pedometer Mode!\r\n");
                    }
                    else
                    {
                        BleApp_WriteToUART("Successfully put FXOS8700 to Pedometer Mode.\r\n");
                        BleApp_WriteToHost("Waiting for Steps...\n");
                    }
                    break;
                case ISSDK_REPORT_ROTATION:
                    /* Put FXAS21002 device into active mode.*/
                    status = Sensor_I2C_Write(gFxas21002Driver.pCommDrv, &gFxas21002Driver.deviceInfo,
                                              gFxas21002Driver.slaveAddress, gFxas21002StartConfiguration);
                    if (ARM_DRIVER_OK != status)
                    {
                        BleApp_WriteToUART("ERROR : Failed to put FXAS21002 to Active Mode!\r\n");
                    }
                    else
                    {
                        BleApp_WriteToUART("Successfully put FXAS21002 to Active Mode.\r\n");
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
                case ISSDK_REPORT_MAGNETIC:
                case ISSDK_REPORT_ECOMPASS:
                case ISSDK_REPORT_FREEFALL:
                case ISSDK_REPORT_PEDOMETER:
                case ISSDK_REPORT_ORIENTATION:
                    /* Put FXOS8700 into standby mode.*/
                    status = Sensor_I2C_Write(gFxos8700Driver.pCommDrv, &gFxos8700Driver.deviceInfo,
                                              gFxos8700Driver.slaveAddress, cFxos8700StopConfiguration);
                    if (ARM_DRIVER_OK != status)
                    {
                        BleApp_WriteToUART("ERROR : Failed to put FXOS8700 to Standby Mode!\r\n");
                    }
                    else
                    {
                        BleApp_WriteToUART("Successfully put FXOS8700 to Standby Mode.\r\n");
                    }
                    break;
                case ISSDK_REPORT_ROTATION:
                    /* Put FXAS21002 into standby mode.*/
                    status = Sensor_I2C_Write(gFxas21002Driver.pCommDrv, &gFxas21002Driver.deviceInfo,
                                              gFxas21002Driver.slaveAddress, gFxas21002StopConfiguration);
                    if (ARM_DRIVER_OK != status)
                    {
                        BleApp_WriteToUART("ERROR : Failed to put FXAS21002 to Standby Mode!\r\n");
                    }
                    else
                    {
                        BleApp_WriteToUART("Successfully put FXAS21002 to Standby Mode.\r\n");
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
            BleApp_WriteToUART("Target is FXOS8700.\r\n");
        }
        if (0 == strncasecmp(hostCommand->rliCmd.device, ISSDK_REGISTER_MAG_TAG, strlen(ISSDK_REGISTER_MAG_TAG)))
        {
            target = ISSDK_REPORT_MAGNETIC;
            BleApp_WriteToUART("Target is FXOS8700.\r\n");
        }
        else if (0 == strncasecmp(hostCommand->rliCmd.device, ISSDK_REGISTER_GYRO_TAG, strlen(ISSDK_REGISTER_GYRO_TAG)))
        {
            target = ISSDK_REPORT_ROTATION;
            BleApp_WriteToUART("Target is FXAS21002.\r\n");
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
                    status = AGM01_RegisterInterface(mode, target, offset, bytes, pBuffer);
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
                    status = AGM01_RegisterInterface(mode, target, offset, bytes, pBuffer);
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
        /* Check for User Interface MAG START command */
        else if (0 ==
                 strncasecmp(hostCommand->modeCmd.mode, ISSDK_REPORT_MAGNETIC_STR, strlen(ISSDK_REPORT_MAGNETIC_STR)))
        {
            /* Change Sensor Configuration to Magnetometer Mode */
            ISSDK_Configure(ISSDK_STANDBY_MODE);
            gReportMode = ISSDK_REPORT_MAGNETIC;
            ISSDK_Configure(ISSDK_ACTIVE_MODE);

            BleApp_WriteToUART("Switched To Magnetometer Mode.\r\n");
            return true;
        }
        /* Check for User Interface GYRO START command */
        else if (0 ==
                 strncasecmp(hostCommand->modeCmd.mode, ISSDK_REPORT_ROTATION_STR, strlen(ISSDK_REPORT_ROTATION_STR)))
        {
            /* Change Sensor Configuration to Gyroscope Mode */
            ISSDK_Configure(ISSDK_STANDBY_MODE);
            gReportMode = ISSDK_REPORT_ROTATION;
            ISSDK_Configure(ISSDK_ACTIVE_MODE);

            BleApp_WriteToUART("Switched To Gyroscope Mode.\r\n");
            return true;
        }
        /* Check for User Interface COMBO START command */
        else if (0 ==
                 strncasecmp(hostCommand->modeCmd.mode, ISSDK_REPORT_ECOMPASS_STR, strlen(ISSDK_REPORT_ECOMPASS_STR)))
        {
            /* Change Sensor Configuration to Gyroscope Mode */
            ISSDK_Configure(ISSDK_STANDBY_MODE);
            gReportMode = ISSDK_REPORT_ECOMPASS;
            ISSDK_Configure(ISSDK_ACTIVE_MODE);

            BleApp_WriteToUART("Switched To eCompass Mode.\r\n");
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
