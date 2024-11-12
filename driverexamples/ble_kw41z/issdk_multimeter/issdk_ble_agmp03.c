/*
 * Copyright (c) 2015 - 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file issdk_ble_agmp03.c
 * @brief The issdk_ble_agmp03.c file implements the ISSDK AGMP03 sensor driver
 *        example demonstration using BLE Wireless UART adapter.
 * Supported Modes:
 *   MODE   = 'STANDBY'       (7 Fixed Characters to Stop All Sensor Data)
 *   MODE   = 'ACCELEROMETER' (13 Fixed Characters to Start Accelerometer Data)
 *   MODE   = 'MAGNETOMETER'  (12 Fixed Characters to Start Magnetometer Data)
 *   MODE   = 'GYROSCOPE'     (9 Fixed Characters to Start Gyroscope Data)
 *   MODE   = 'ECOMPASS'      (8 Fixed Characters to Start eCompass Data)
 *   MODE   = 'BAROMETER'     (9 Fixed Characters to Start Barometer Data)
 *   MODE   = 'ALTIMETER'     (9 Fixed Characters to Start Altimeter Data)
 *   MODE   = 'THERMOMETER'   (11 Fixed Characters to Start Temperature Data)
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
#include "fxls8962_drv.h"
#include "fxas21002_drv.h"
#include "mag3110_drv.h"
#include "mpl3115_drv.h"

/* ISSDK Interface Include */
#include "issdk_ble_interface.h"

/* BLE framework Includes */
#include "LED.h"
#include "MemManager.h"
#include "GPIO_Adapter.h"

/*******************************************************************************
 * Macro Definitions
 ******************************************************************************/
#define FXLS8962_DATA_SIZE (6)            /* 2 byte X,Y,Z*/
#define MAG3110_DATA_SIZE (6)             /* 2 byte X,Y,Z*/
#define FXAS21002_DATA_SIZE (6)           /* 2 byte X,Y,Z*/
#define MPL3115_PRESSURE_DATA_SIZE (3)    /* 3 byte Pressure/Altitude. */
#define MPL3115_TEMPERATURE_DATA_SIZE (2) /* 2 Byte Temperature. */

/* Sensor Data Conversion Factors */
#define FXLS8962_ACCEL_CONV (0.00098)                     /* Will give 'g' */
#define MAG3110_MAG_CONV (0.1)                            /* Will give uT */
#define FXAS21002_GYRO_CONV (0.0625)                      /* Will give deg/s */
#define MPL3115_PRES_CONV MPL3115_PRESSURE_CONV_FACTOR    /* Will give Pa */
#define MPL3115_ALT_CONV MPL3115_ALTITUDE_CONV_FACTOR     /* Will give meters above MSL */
#define MPL3115_TEMP_CONV MPL3115_TEMPERATURE_CONV_FACTOR /* Will give degC */

/* SDCD free fall, motion detect and orientation counter register values.
 * These values have been derived form the Application Note AN4070 for MMA8451 (the same is applicable to FXLS8962 too).
 * http://cache.freescale.com/files/sensors/doc/app_note/AN4070.pdf */
#define FF_SDCD_WT_DBCNT 0x0A /* Debounce count value. */
#define FF_SDCD_LTHS_LSB 0x33 /* Lower Threshold LSB value. */
#define FF_SDCD_LTHS_MSB 0x0F /* Lower Threshold MSB value. */
#define FF_SDCD_UTHS_LSB 0xCD /* Upper Threshold LSB value. */
#define FF_SDCD_UTHS_MSB 0x00 /* Upper Threshold MSB value. */

#define MD_SDCD_WT_DBCNT 0x00 /* Debounce count value. */
#define MD_SDCD_LTHS_LSB 0xC0 /* Lower Threshold LSB value. */
#define MD_SDCD_LTHS_MSB 0x0F /* Lower Threshold MSB value. */
#define MD_SDCD_UTHS_LSB 0x40 /* Upper Threshold LSB value. */
#define MD_SDCD_UTHS_MSB 0x00 /* Upper Threshold MSB value. */

#define OT_SDCD_WT_DBCNT 0x09 /* Debounce count value. */
#define OT_SDCD_LTHS_LSB 0xC0 /* Lower Threshold LSB value. */
#define OT_SDCD_LTHS_MSB 0x0F /* Lower Threshold MSB value. */
#define OT_SDCD_UTHS_LSB 0x40 /* Upper Threshold LSB value. */
#define OT_SDCD_UTHS_MSB 0x00 /* Upper Threshold MSB value. */

/*******************************************************************************
 * Constants
 ******************************************************************************/
/*! @brief Register settings for Normal Mode. */
const registerwritelist_t cFxls8962ConfigNormal[] = {
    /* Set Full-scale range as 2G. */
    {FXLS8962_SENS_CONFIG1, FXLS8962_SENS_CONFIG1_FSR_2G, FXLS8962_SENS_CONFIG1_FSR_MASK},
    __END_WRITE_DATA__};

/*! @brief FXLS8962 Start INT Mode Register Write List */
const registerwritelist_t cFxls8962StartIntConfiguration[] = {
    /* Set Wake Mode ODR Rate @ 12.5Hz. */
    {FXLS8962_SENS_CONFIG3, FXLS8962_SENS_CONFIG3_WAKE_ODR_12_5HZ, FXLS8962_SENS_CONFIG3_WAKE_ODR_MASK},
    /* Enable Interrupts for Data Ready Events. */
    {FXLS8962_INT_EN, FXLS8962_INT_EN_DRDY_EN_EN, FXLS8962_INT_EN_DRDY_EN_MASK},
    /* Lastly Set ACTIVE Bit */
    {FXLS8962_SENS_CONFIG1, FXLS8962_SENS_CONFIG1_ACTIVE_ACTIVE, FXLS8962_SENS_CONFIG1_ACTIVE_MASK},
    __END_WRITE_DATA__};

/*! @brief Register Start Free-fall Mode Register Write List. */
const registerwritelist_t cFxls8962StartFreeFallConfiguration[] = {
    /* Set Wake Mode ODR Rate as 100Hz. */
    {FXLS8962_SENS_CONFIG3, FXLS8962_SENS_CONFIG3_WAKE_ODR_100HZ, FXLS8962_SENS_CONFIG3_WAKE_ODR_MASK},
    /* Set WT INT Enable. */
    {FXLS8962_INT_EN, FXLS8962_INT_EN_SDCD_WT_EN_EN, FXLS8962_INT_EN_SDCD_WT_EN_MASK},
    /* Enable Within-Threshold Event-Laching-Enable and X,Y,Z Axis functions. */
    {FXLS8962_SDCD_CONFIG1, FXLS8962_SDCD_CONFIG1_WT_ELE_EN | FXLS8962_SDCD_CONFIG1_X_WT_EN_EN |
                                FXLS8962_SDCD_CONFIG1_Y_WT_EN_EN | FXLS8962_SDCD_CONFIG1_Z_WT_EN_EN,
     FXLS8962_SDCD_CONFIG1_WT_ELE_MASK | FXLS8962_SDCD_CONFIG1_X_WT_EN_MASK | FXLS8962_SDCD_CONFIG1_Y_WT_EN_MASK |
         FXLS8962_SDCD_CONFIG1_Z_WT_EN_MASK},
    /* Set SDCD internal reference values update mode to fixed value; Set Debounce counter to be
       cleared whenever the SDCD within thresholds. */
    {FXLS8962_SDCD_CONFIG2, FXLS8962_SDCD_CONFIG2_SDCD_EN_EN | FXLS8962_SDCD_CONFIG2_REF_UPDM_FIXED_VAL |
                                FXLS8962_SDCD_CONFIG2_WT_DBCTM_CLEARED,
     FXLS8962_SDCD_CONFIG2_SDCD_EN_MASK | FXLS8962_SDCD_CONFIG2_REF_UPDM_MASK | FXLS8962_SDCD_CONFIG2_WT_DBCTM_MASK},
    /* Set SDCD Debounce counter value. */
    {FXLS8962_SDCD_WT_DBCNT, FF_SDCD_WT_DBCNT, 0},
    /* Set SDCD Lower Threshold LSB value. */
    {FXLS8962_SDCD_LTHS_LSB, FF_SDCD_LTHS_LSB, 0},
    /* Set SDCD Lower Threshold MSB value. */
    {FXLS8962_SDCD_LTHS_MSB, FF_SDCD_LTHS_MSB, 0},
    /* Set SDCH Upper Threshold LSB value. */
    {FXLS8962_SDCD_UTHS_LSB, FF_SDCD_UTHS_LSB, 0},
    /* Set SDCH Upper Threshold MSB value. */
    {FXLS8962_SDCD_UTHS_MSB, FF_SDCD_UTHS_MSB, 0},
    /* Lastly Set ACTIVE Bit */
    {FXLS8962_SENS_CONFIG1, FXLS8962_SENS_CONFIG1_ACTIVE_ACTIVE, FXLS8962_SENS_CONFIG1_ACTIVE_MASK},
    __END_WRITE_DATA__};

/*! @brief Register Start Motion Detect Mode Register Write List. */
const registerwritelist_t cFxls8962ConfigMotionDetect[] = {
    /* Set Wake Mode ODR Rate as 0.781Hz. */
    {FXLS8962_SENS_CONFIG3, FXLS8962_SENS_CONFIG3_WAKE_ODR_0_781HZ, FXLS8962_SENS_CONFIG3_WAKE_ODR_MASK},
    {FXLS8962_INT_EN, FXLS8962_INT_EN_SDCD_OT_EN_EN, FXLS8962_INT_EN_SDCD_OT_EN_MASK},
    {FXLS8962_SDCD_CONFIG1,
     FXLS8962_SDCD_CONFIG1_X_OT_EN_EN | FXLS8962_SDCD_CONFIG1_Y_OT_EN_EN | FXLS8962_SDCD_CONFIG1_Z_OT_EN_EN,
     FXLS8962_SDCD_CONFIG1_X_OT_EN_MASK | FXLS8962_SDCD_CONFIG1_Y_OT_EN_MASK | FXLS8962_SDCD_CONFIG1_Z_OT_EN_MASK},
    {FXLS8962_SDCD_CONFIG2, FXLS8962_SDCD_CONFIG2_SDCD_EN_EN | FXLS8962_SDCD_CONFIG2_REF_UPDM_SDCD_REF |
                                FXLS8962_SDCD_CONFIG2_OT_DBCTM_CLEARED | FXLS8962_SDCD_CONFIG2_WT_DBCTM_CLEARED,
     FXLS8962_SDCD_CONFIG2_SDCD_EN_MASK | FXLS8962_SDCD_CONFIG2_REF_UPDM_MASK | FXLS8962_SDCD_CONFIG2_OT_DBCTM_MASK |
         FXLS8962_SDCD_CONFIG2_WT_DBCTM_MASK},
    /* Set SDCD Debounce counter value. */
    {FXLS8962_SDCD_WT_DBCNT, MD_SDCD_WT_DBCNT, 0},
    /* Set SDCD Lower Threshold LSB value. */
    {FXLS8962_SDCD_LTHS_LSB, MD_SDCD_LTHS_LSB, 0},
    /* Set SDCD Lower Threshold MSB value. */
    {FXLS8962_SDCD_LTHS_MSB, MD_SDCD_LTHS_MSB, 0},
    /* Set SDCH Upper Threshold LSB value. */
    {FXLS8962_SDCD_UTHS_LSB, MD_SDCD_UTHS_LSB, 0},
    /* Set SDCH Upper Threshold MSB value. */
    {FXLS8962_SDCD_UTHS_MSB, MD_SDCD_UTHS_MSB, 0},
    /* Lastly Set ACTIVE Bit */
    {FXLS8962_SENS_CONFIG1, FXLS8962_SENS_CONFIG1_ACTIVE_ACTIVE, FXLS8962_SENS_CONFIG1_ACTIVE_MASK},
    __END_WRITE_DATA__};

/*! @brief Register Start Motion Detect Mode Register Write List. */
const registerwritelist_t cFxls8962ConfigOrientDetect[] = {
    /* Set Wake Mode ODR Rate as 1.563Hz. */
    {FXLS8962_SENS_CONFIG3, FXLS8962_SENS_CONFIG3_WAKE_ODR_1_563HZ, FXLS8962_SENS_CONFIG3_WAKE_ODR_MASK},
    /* Pulse Generation Enabled */
    {FXLS8962_SENS_CONFIG4, FXLS8962_SENS_CONFIG4_DRDY_PUL_EN, FXLS8962_SENS_CONFIG4_DRDY_PUL_MASK},
    {FXLS8962_INT_EN, FXLS8962_INT_EN_DRDY_EN_EN, FXLS8962_INT_EN_DRDY_EN_MASK},
    {FXLS8962_ORIENT_CONFIG, FXLS8962_ORIENT_CONFIG_ORIENT_ENABLE_EN, FXLS8962_ORIENT_CONFIG_ORIENT_ENABLE_MASK},
    {FXLS8962_SDCD_CONFIG1, FXLS8962_SDCD_CONFIG1_WT_ELE_EN | FXLS8962_SDCD_CONFIG1_X_WT_EN_EN |
                                FXLS8962_SDCD_CONFIG1_Y_WT_EN_EN | FXLS8962_SDCD_CONFIG1_Z_WT_EN_EN,
     FXLS8962_SDCD_CONFIG1_WT_ELE_MASK | FXLS8962_SDCD_CONFIG1_X_WT_EN_MASK | FXLS8962_SDCD_CONFIG1_Y_WT_EN_MASK |
         FXLS8962_SDCD_CONFIG1_Z_WT_EN_MASK},
    {FXLS8962_SDCD_CONFIG2, FXLS8962_SDCD_CONFIG2_SDCD_EN_EN | FXLS8962_SDCD_CONFIG2_REF_UPDM_SDCD_REF |
                                FXLS8962_SDCD_CONFIG2_OT_DBCTM_CLEARED | FXLS8962_SDCD_CONFIG2_WT_DBCTM_CLEARED,
     FXLS8962_SDCD_CONFIG2_SDCD_EN_MASK | FXLS8962_SDCD_CONFIG2_REF_UPDM_MASK | FXLS8962_SDCD_CONFIG2_OT_DBCTM_MASK |
         FXLS8962_SDCD_CONFIG2_WT_DBCTM_MASK},
    /* Set SDCD Debounce counter value. */
    {FXLS8962_SDCD_WT_DBCNT, OT_SDCD_WT_DBCNT, 0},
    /* Set SDCD Lower Threshold LSB value. */
    {FXLS8962_SDCD_LTHS_LSB, OT_SDCD_LTHS_LSB, 0},
    /* Set SDCD Lower Threshold MSB value. */
    {FXLS8962_SDCD_LTHS_MSB, OT_SDCD_LTHS_MSB, 0},
    /* Set SDCH Upper Threshold LSB value. */
    {FXLS8962_SDCD_UTHS_LSB, OT_SDCD_UTHS_LSB, 0},
    /* Set SDCH Upper Threshold MSB value. */
    {FXLS8962_SDCD_UTHS_MSB, OT_SDCD_UTHS_MSB, 0},
    /* Lastly Set ACTIVE Bit */
    {FXLS8962_SENS_CONFIG1, FXLS8962_SENS_CONFIG1_ACTIVE_ACTIVE, FXLS8962_SENS_CONFIG1_ACTIVE_MASK},
    __END_WRITE_DATA__};

/*! @brief FXLS8962 Start Pedometer Mode Register Write List */
const registerwritelist_t cFxls8962StartPedometerConfiguration[] = {
    /* Set Wake Mode ODR Rate as 50Hz. */
    {FXLS8962_SENS_CONFIG3, FXLS8962_SENS_CONFIG3_WAKE_ODR_50HZ, FXLS8962_SENS_CONFIG3_WAKE_ODR_MASK},
    /* Enable Interrupts for Data Readay Events. */
    {FXLS8962_INT_EN, FXLS8962_INT_EN_DRDY_EN_EN, FXLS8962_INT_EN_DRDY_EN_MASK},
    /* Lastly Set ACTIVE Bit */
    {FXLS8962_SENS_CONFIG1, FXLS8962_SENS_CONFIG1_ACTIVE_ACTIVE, FXLS8962_SENS_CONFIG1_ACTIVE_MASK},
    __END_WRITE_DATA__};

/*! @brief FXLS8962 Stop Register Write List */
const registerwritelist_t cFxls8962StopConfiguration[] = {
    {FXLS8962_SENS_CONFIG1, FXLS8962_SENS_CONFIG1_ACTIVE_STANDBY, FXLS8962_SENS_CONFIG1_ACTIVE_MASK},
    {FXLS8962_SENS_CONFIG4, FXLS8962_SENS_CONFIG4_DRDY_PUL_DIS, FXLS8962_SENS_CONFIG4_DRDY_PUL_MASK},
    {FXLS8962_ORIENT_CONFIG, FXLS8962_ORIENT_CONFIG_ORIENT_ENABLE_DIS, FXLS8962_ORIENT_CONFIG_ORIENT_ENABLE_MASK},
    /* Disable SDCD function. */
    {FXLS8962_SDCD_CONFIG1, 0x00, 0},
    {FXLS8962_SDCD_CONFIG2, 0x00, 0},
    /* Clear all INT. */
    {FXLS8962_INT_EN, 0x00, 0},
    __END_WRITE_DATA__};

/*! @brief Address of Raw Accel Data in Normal Mode. */
const registerreadlist_t cFxls8962OutputNormal[] = {{.readFrom = FXLS8962_OUT_X_LSB, .numBytes = FXLS8962_DATA_SIZE},
                                                    __END_READ_DATA__};

/*! @brief Address of Free-Fall Status Register. */
const registerreadlist_t cFxls8962SDCDEvent[] = {{.readFrom = FXLS8962_SDCD_INT_SRC2, .numBytes = 1},
                                                 __END_READ_DATA__};

const registerreadlist_t cFxls8962OutputOrientStatus[] = {{.readFrom = FXLS8962_ORIENT_STATUS, .numBytes = 1},
                                                          __END_READ_DATA__};

/*! @brief Register settings for Normal mode. */
const registerwritelist_t cMag3110ConfigNormal[] = {
    /* Set Output Rate @ 10Hz (ODR = 3 and OSR = 16). */
    {MAG3110_CTRL_REG1, MAG3110_CTRL_REG1_DR_ODR_3 | MAG3110_CTRL_REG1_OS_OSR_16,
                        MAG3110_CTRL_REG1_DR_MASK | MAG3110_CTRL_REG1_OS_MASK},
    /* Set Auto Magnetic Sensor Reset. */
    {MAG3110_CTRL_REG2, MAG3110_CTRL_REG2_AUTO_MSRT_EN_DIS | MAG3110_CTRL_REG2_RAW_RAW,
                        MAG3110_CTRL_REG2_AUTO_MSRT_EN_MASK | MAG3110_CTRL_REG2_RAW_MASK},
    /* Interrupt is enabled by default */
    __END_WRITE_DATA__};

/*! @brief MAG3110 Start Register Write List */
const registerwritelist_t cMag3110StartConfiguration[] = {
    {MAG3110_CTRL_REG1, MAG3110_CTRL_REG1_AC_ACTIVE, MAG3110_CTRL_REG1_AC_MASK}, // Set ACTIVE Bit
    __END_WRITE_DATA__};

/*! @brief MAG3110 Stop Register Write List */
const registerwritelist_t cMag3110StopConfiguration[] = {
    {MAG3110_CTRL_REG1, MAG3110_CTRL_REG1_AC_STANDBY, MAG3110_CTRL_REG1_AC_MASK}, // Clear ACTIVE Bit
    __END_WRITE_DATA__};

/*! @brief Address of Raw Magnetic Data in Normal Mode. */
const registerreadlist_t cMag3110OutputNormal[] = {
    {.readFrom = MAG3110_OUT_X_MSB, .numBytes = MAG3110_DATA_SIZE},
    __END_READ_DATA__};

/*! Prepare the register write list to configure FXAS21002 Normal mode. */
const registerwritelist_t fxas21002_Config_Normal[] = {
    /*! Configure CTRL_REG1 register to put FXAS21002 to 12.5Hz sampling rate. */
    {FXAS21002_CTRL_REG1, FXAS21002_CTRL_REG1_DR_12_5HZ, FXAS21002_CTRL_REG1_DR_MASK},
    /*! Configure CTRL_REG2 register to set interrupt configuration settings. */
    {FXAS21002_CTRL_REG2, FXAS21002_CTRL_REG2_IPOL_ACTIVE_HIGH | FXAS21002_CTRL_REG2_INT_EN_DRDY_ENABLE | FXAS21002_CTRL_REG2_INT_CFG_DRDY_INT2,
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
    {.readFrom = FXAS21002_OUT_X_MSB, .numBytes = FXAS21002_DATA_SIZE},
    __END_READ_DATA__};

/*! @brief Register settings for Normal mode. */
const registerwritelist_t cMpl3115ConfigNormal[] = {
    /* Enable Data Ready and Event flags for Pressure, Temperature or either. */
    {MPL3115_PT_DATA_CFG, MPL3115_PT_DATA_CFG_TDEFE_ENABLED | MPL3115_PT_DATA_CFG_PDEFE_ENABLED | MPL3115_PT_DATA_CFG_DREM_ENABLED,
                          MPL3115_PT_DATA_CFG_TDEFE_MASK | MPL3115_PT_DATA_CFG_PDEFE_MASK | MPL3115_PT_DATA_CFG_DREM_MASK},
    /* Set 7Hz OSR. */
    {MPL3115_CTRL_REG1, MPL3115_CTRL_REG1_OS_OSR_32, MPL3115_CTRL_REG1_OS_MASK},
    /* Set INT1 Active High. */
    {MPL3115_CTRL_REG3, MPL3115_CTRL_REG3_IPOL1_HIGH, MPL3115_CTRL_REG3_IPOL1_MASK},
    /* Enable Interrupts for Data Ready Events. */
    {MPL3115_CTRL_REG4, MPL3115_CTRL_REG4_INT_EN_DRDY_INTENABLED, MPL3115_CTRL_REG4_INT_EN_DRDY_MASK},
    /* Route Interrupt to INT1. */
    {MPL3115_CTRL_REG5, MPL3115_CTRL_REG5_INT_CFG_DRDY_INT1, MPL3115_CTRL_REG5_INT_CFG_DRDY_MASK},
    __END_WRITE_DATA__};

/*! @brief MPL3115 Start Register Write List */
const registerwritelist_t cMpl3115StartConfiguration[] = {
    {MPL3115_CTRL_REG1, MPL3115_CTRL_REG1_ALT_BAR | MPL3115_CTRL_REG1_OST_SET,
                        MPL3115_CTRL_REG1_ALT_MASK | MPL3115_CTRL_REG1_OST_MASK}, // Set BAR and OST Bit
    __END_WRITE_DATA__};

/*! @brief MPL3115 Start ALT Mode Register Write List */
const registerwritelist_t cMpl3115StartAltConfiguration[] = {
    {MPL3115_CTRL_REG1, MPL3115_CTRL_REG1_ALT_ALT | MPL3115_CTRL_REG1_OST_SET,
                        MPL3115_CTRL_REG1_ALT_MASK | MPL3115_CTRL_REG1_OST_MASK}, // Set ALT and OST Bit
    __END_WRITE_DATA__};

/*! @brief Address and size of Raw Pressure/Altitude Data in Normal Mode. */
const registerreadlist_t cMpl3115OutputNormal[] = {
    {.readFrom = MPL3115_OUT_P_MSB, .numBytes = MPL3115_PRESSURE_DATA_SIZE},
    __END_READ_DATA__};

/*! @brief Address and size of Raw Pressure/Altitude Data in Normal Mode. */
const registerreadlist_t cMpl3115OutputTemperature[] = {
    {.readFrom = MPL3115_OUT_T_MSB, .numBytes = MPL3115_TEMPERATURE_DATA_SIZE},
    __END_READ_DATA__};

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
fxls8962_i2c_sensorhandle_t gFxls8962Driver;
mag3110_i2c_sensorhandle_t gMag3110Driver;
fxas21002_i2c_sensorhandle_t gFxas21002Driver;
mpl3115_i2c_sensorhandle_t gMpl3115Driver;
volatile uint8_t lastAccelData[FXLS8962_DATA_SIZE];
gpioInputPinConfig_t gFxls8962IntCfg, gMag3110IntCfg, gFxas21002IntCfg, gMpl3115IntCfg;

/************************************************************************************
* Functions
************************************************************************************/
/*! @brief Function to Read Temperature Data */
static void MPL3115_ReadTemperature(void)
{
    float tempC;
    char tempCs[8] = {0};
    int32_t status, tempCi;
    mpl3115_pressuredata_t rawData;
    uint8_t data[MPL3115_TEMPERATURE_DATA_SIZE];

    /* Trigger acquisition of the Next Sample. */
    status = Sensor_I2C_Write(gMpl3115Driver.pCommDrv, &gMpl3115Driver.deviceInfo, gMpl3115Driver.slaveAddress,
                              cMpl3115StartConfiguration);
    if (ARM_DRIVER_OK != status)
    {
        BleApp_WriteToUART("ERROR : Trigger Next Sample Failed!\r\n");
        return;
    }

    /*! Read the raw sensor data from the MPL3115. */
    status = MPL3115_I2C_ReadData(&gMpl3115Driver, cMpl3115OutputTemperature, data);
    if (ARM_DRIVER_OK != status)
    {
        BleApp_WriteToUART("ERROR : Read Temperature Failed!\r\n");
        return;
    }

    /*! Convert the raw sensor data to signed 16-bit container for display. */
    rawData.temperature = (int16_t)((data[0]) << 8) | (data[1]);
    tempC = rawData.temperature / MPL3115_TEMP_CONV;

    if (gReportFormat == ISSDK_READ_RAW)
    {
        /* Send raw data over the air */
        BleApp_WriteToHost("TEMP: 0x%04X\n", (uint16_t)rawData.temperature);
    }
    if (gReportFormat == ISSDK_READ_NORMAL)
    {
        ISSDK_GetStringOfFloat(tempC, tempCs);
        /* Send converted data over the air */
        BleApp_WriteToHost("TEMP: %sC\n", tempCs);
    }
    if (gReportFormat == ISSDK_READ_STREAM)
    {
        tempCi = ISSDK_GetDecimalForFloat(tempC);
        /* Send Data Stream over the air */
        BleApp_WriteToHost("%s:%08X\n", ISSDK_TEMP_STREAM_HDR_STR, (uint32_t)tempCi);
    }
}

/*! @brief Function to Read Altitude Data */
static void MPL3115_ReadAltitude(void)
{
    float altM;
    char altMs[8] = {0};
    int32_t status, altMi;
    mpl3115_altitudedata_t rawData;
    uint8_t data[MPL3115_PRESSURE_DATA_SIZE];

    /* Trigger acquisition of the Next Sample. */
    status = Sensor_I2C_Write(gMpl3115Driver.pCommDrv, &gMpl3115Driver.deviceInfo, gMpl3115Driver.slaveAddress,
                              cMpl3115StartAltConfiguration);
    if (ARM_DRIVER_OK != status)
    {
        BleApp_WriteToUART("ERROR : Trigger Next Sample Failed!\r\n");
        return;
    }

    /*! Read the raw sensor data from the MPL3115. */
    status = MPL3115_I2C_ReadData(&gMpl3115Driver, cMpl3115OutputNormal, data);
    if (ARM_DRIVER_OK != status)
    {
        BleApp_WriteToUART("ERROR : Read Altitude Failed!\r\n");
        return;
    }

    /*! Convert the raw sensor data to signed 32-bit container for display. */
    rawData.altitude = (int32_t)((data[0]) << 24) | ((data[1]) << 16) | ((data[2]) << 8);
    altM = rawData.altitude / MPL3115_ALT_CONV;

    if (gReportFormat == ISSDK_READ_RAW)
    {
        /* Send raw data over the air */
        BleApp_WriteToHost("ALTITUDE: 0x%08X\n", (uint32_t)rawData.altitude);
    }
    if (gReportFormat == ISSDK_READ_NORMAL)
    {
        ISSDK_GetStringOfFloat(altM, altMs);
        /* Send data over the air */
        BleApp_WriteToHost("ALTITUDE: %sm\n", altMs);
    }
    if (gReportFormat == ISSDK_READ_STREAM)
    {
        altMi = ISSDK_GetDecimalForFloat(altM);
        /* Send Data Stream over the air */
        BleApp_WriteToHost("%s:%08X\n", ISSDK_ALT_STREAM_HDR_STR, (uint32_t)altMi);
    }
}

/*! @brief Function to Read Pressure Data */
static void MPL3115_ReadPressure(void)
{
    float presP;
    char presPs[8] = {0};
    int32_t status, presPi;
    mpl3115_pressuredata_t rawData;
    uint8_t data[MPL3115_PRESSURE_DATA_SIZE];

    /* Trigger acquisition of the Next Sample. */
    status = Sensor_I2C_Write(gMpl3115Driver.pCommDrv, &gMpl3115Driver.deviceInfo, gMpl3115Driver.slaveAddress,
                              cMpl3115StartConfiguration);
    if (ARM_DRIVER_OK != status)
    {
        BleApp_WriteToUART("ERROR : Trigger Next Sample Failed!\r\n");
        return;
    }

    /*! Read the raw sensor data from the MPL3115. */
    status = MPL3115_I2C_ReadData(&gMpl3115Driver, cMpl3115OutputNormal, data);
    if (ARM_DRIVER_OK != status)
    {
        BleApp_WriteToUART("ERROR : Read Pressure Failed!\r\n");
        return;
    }

    /*! Convert the raw sensor data to signed 32-bit container for display. */
    rawData.pressure = (uint32_t)((data[0]) << 16) | ((data[1]) << 8) | ((data[2]));
    presP = rawData.pressure / MPL3115_PRES_CONV;
    presP /= 100000; /* Will give Bar */

    if (gReportFormat == ISSDK_READ_RAW)
    {
        /* Send raw data over the air */
        BleApp_WriteToHost("PRESSURE: 0x%08X\n", (uint32_t)rawData.pressure);
    }
    if (gReportFormat == ISSDK_READ_NORMAL)
    {
        ISSDK_GetStringOfFloat(presP, presPs);
        /* Send data over the air */
        BleApp_WriteToHost("PRESSURE: %sBar\n", presPs);
    }
    if (gReportFormat == ISSDK_READ_STREAM)
    {
        presPi = ISSDK_GetDecimalForFloat(presP);
        /* Send Data Stream over the air */
        BleApp_WriteToHost("%s:%08X\n", ISSDK_PRESSURE_STREAM_HDR_STR, (uint32_t)presPi);
    }
}

/*! @brief Function to Read Data and Clear State of MPL3115 */
static void MPL3115_ClearState(void)
{
    int32_t status;
    uint8_t data[MPL3115_PRESSURE_DATA_SIZE];

    /*! Read the raw sensor data from the MPL3115. */
    status = MPL3115_I2C_ReadData(&gMpl3115Driver, cMpl3115OutputNormal, data);
    if (ARM_DRIVER_OK != status)
    {
        BleApp_WriteToUART("ERROR : Read Pressure Failed!\r\n");
        return;
    }
}

/*! @brief Function to Read Acceleration Data */
static void FXLS8962_ReadAcceleration(void)
{
    float accelX, accelY, accelZ;
    int32_t status, accelXi, accelYi, accelZi;
    char accelXs[8] = {0}, accelYs[8] = {0}, accelZs[8] = {0};
    fxls8962_acceldata_t rawData;
    uint8_t data[FXLS8962_DATA_SIZE];

    /*! Read the raw sensor data from the FXLS8962. */
    status = FXLS8962_I2C_ReadData(&gFxls8962Driver, cFxls8962OutputNormal, data);
    if (ARM_DRIVER_OK != status)
    {
        BleApp_WriteToUART("ERROR : Read Acceleration Failed!\r\n");
        return;
    }

    /*! Convert the raw sensor data to signed 16-bit container for display to the debug port. */
    rawData.accel[0] = ((int16_t)data[1] << 8) | data[0];
    rawData.accel[1] = ((int16_t)data[3] << 8) | data[2];
    rawData.accel[2] = ((int16_t)data[5] << 8) | data[4];
    accelX = FXLS8962_ACCEL_CONV * rawData.accel[0];
    accelY = FXLS8962_ACCEL_CONV * rawData.accel[1];
    accelZ = FXLS8962_ACCEL_CONV * rawData.accel[2];

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
void FXLS8962_ReadPedometer(void)
{
    char speedS[8];

    /* Log the last reported steps. */
    gLastReportedSteps = gPedometer.status.stepcount;
    ISSDK_GetStringOfFloat(gPedometer.status.speed / 1000.0, speedS);

    /* Send data over the air */
    BleApp_WriteToHost("STEPS:%d | DISTANCE:%dm | SPEED:%skm/h\n", gPedometer.status.stepcount,
                       gPedometer.status.distance, speedS);
}

/*! @brief Function to Read Magnetic Data */
static void MAG3110_ReadMagnetic(void)
{
    float magX, magY, magZ;
    int32_t status, magXi, magYi, magZi;
    char magXs[8] = {0}, magYs[8] = {0}, magZs[8] = {0};
    mag3110_magdata_t rawData;
    uint8_t data[MAG3110_DATA_SIZE];

    /*! Read the raw sensor data from the MAG3110. */
    status = MAG3110_I2C_ReadData(&gMag3110Driver, cMag3110OutputNormal, data);
    if (ARM_DRIVER_OK != status)
    {
        BleApp_WriteToUART("ERROR : Read Magnetic Failed!\r\n");
        return;
    }

    /*! Process the sample and convert the raw sensor data to signed 16-bit container. */
    rawData.mag[0] = ((int16_t)data[0] << 8) | data[1];
    rawData.mag[1] = ((int16_t)data[2] << 8) | data[3];
    rawData.mag[2] = ((int16_t)data[4] << 8) | data[5];

    MAG3110_CalibrateHardIronOffset(&rawData.mag[0], &rawData.mag[1], &rawData.mag[2]);

    magX = MAG3110_MAG_CONV * rawData.mag[0];
    magY = MAG3110_MAG_CONV * rawData.mag[1];
    magZ = MAG3110_MAG_CONV * rawData.mag[2];

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

/*! @brief Function to Read Acceleration Data for eCompass */
static void FXLS8962_ReadCombo(void)
{
    int32_t status;

    /*! Read the raw sensor data from the FXLS8962. */
    status = FXLS8962_I2C_ReadData(&gFxls8962Driver, cFxls8962OutputNormal, (uint8_t*)lastAccelData);
    if (ARM_DRIVER_OK != status)
    {
        BleApp_WriteToUART("ERROR : Read Acceleration Failed!\r\n");
        return;
    }
}

/*! @brief Function to Read Data and Clear State of MAG3110 */
static void MAG3110_ClearState(void)
{
    int32_t status;
    mag3110_magdata_t rawData;
    uint8_t data[MAG3110_DATA_SIZE];

    /*! Read the raw sensor data from the MAG3110. */
    status = MAG3110_I2C_ReadData(&gMag3110Driver, cMag3110OutputNormal, data);
    if (ARM_DRIVER_OK != status)
    {
        BleApp_WriteToUART("ERROR : Read Magnetic Failed!\r\n");
        return;
    }

    /*! Process the sample and convert the raw sensor data to signed 16-bit container. */
    rawData.mag[0] = ((int16_t)data[0] << 8) | data[1];
    rawData.mag[1] = ((int16_t)data[2] << 8) | data[3];
    rawData.mag[2] = ((int16_t)data[4] << 8) | data[5];

    MAG3110_CalibrateHardIronOffset(&rawData.mag[0], &rawData.mag[1], &rawData.mag[2]);
}

/*! @brief Function to Read Acceleration + Magnetic Data */
static void MAG3110_ReadCombo(void)
{
    float accelX, accelY, accelZ, magX, magY, magZ;
    int32_t status, accelXi, accelYi, accelZi, magXi, magYi, magZi;
    char accelXs[8] = {0}, accelYs[8] = {0}, accelZs[8] = {0},
         magXs[8] = {0}, magYs[8] = {0}, magZs[8] = {0};

    fxls8962_acceldata_t rawDataAccel;
    mag3110_magdata_t rawDataMag;
    uint8_t data[MAG3110_DATA_SIZE];

    /*! Read the raw sensor data from the MAG3110. */
    status = MAG3110_I2C_ReadData(&gMag3110Driver, cMag3110OutputNormal, data);
    if (ARM_DRIVER_OK != status)
    {
        BleApp_WriteToUART("ERROR : Read Magnetic Failed!\r\n");
        return;
    }

    /*! Convert the raw sensor data to signed 16-bit container for display to the debug port. */
    rawDataAccel.accel[0] = ((int16_t)lastAccelData[1] << 8) | lastAccelData[0];
    rawDataAccel.accel[1] = ((int16_t)lastAccelData[3] << 8) | lastAccelData[2];
    rawDataAccel.accel[2] = ((int16_t)lastAccelData[5] << 8) | lastAccelData[4];
    rawDataMag.mag[0] = ((int16_t)data[0] << 8) | data[1];
    rawDataMag.mag[1] = ((int16_t)data[2] << 8) | data[3];
    rawDataMag.mag[2] = ((int16_t)data[4] << 8) | data[5];

    MAG3110_CalibrateHardIronOffset(&rawDataMag.mag[0], &rawDataMag.mag[1], &rawDataMag.mag[2]);

    accelX = FXLS8962_ACCEL_CONV * rawDataAccel.accel[0];
    accelY = FXLS8962_ACCEL_CONV * rawDataAccel.accel[1];
    accelZ = FXLS8962_ACCEL_CONV * rawDataAccel.accel[2];
    magX = MAG3110_MAG_CONV * rawDataMag.mag[0];
    magY = MAG3110_MAG_CONV * rawDataMag.mag[1];
    magZ = MAG3110_MAG_CONV * rawDataMag.mag[2];

    if (gReportFormat == ISSDK_READ_RAW)
    {
        /* Send raw data over the air */
        BleApp_WriteToHost("ACCEL: X=0x%04X Y=0x%04X Z=0x%04X\n", (uint16_t)rawDataAccel.accel[0],
                           (uint16_t)rawDataAccel.accel[1], (uint16_t)rawDataAccel.accel[2]);
        BleApp_WriteToHost("MAG: X=0x%04X Y=0x%04X Z=0x%04X\n", (uint16_t)rawDataMag.mag[0],
                           (uint16_t)rawDataMag.mag[1], (uint16_t)rawDataMag.mag[2]);
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
static int32_t AGMP03_RegisterInterface(uint8_t opCode, uint8_t mode, uint8_t offset, uint8_t bytes, uint8_t *buffer)
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
                    /*! Read FXLS8962 registers. */
                    result = Register_I2C_Read(gFxls8962Driver.pCommDrv, &gFxls8962Driver.deviceInfo,
                                               gFxls8962Driver.slaveAddress, offset, bytes, buffer);
                    if (result != SENSOR_ERROR_NONE)
                    {
                        BleApp_WriteToUART("ERROR : Read [%d] bytes form [0x%02X] of FXLS8962!\r\n", offset, bytes);
                    }
                    break;
                case ISSDK_REPORT_MAGNETIC:
                    /*! Read MAG3110 registers. */
                    result = Register_I2C_Read(gMag3110Driver.pCommDrv, &gMag3110Driver.deviceInfo,
                                               gMag3110Driver.slaveAddress, offset, bytes, buffer);
                    if (result != SENSOR_ERROR_NONE)
                    {
                        BleApp_WriteToUART("ERROR : Read [%d] bytes form [0x%02X] of MAG3110!\r\n", offset, bytes);
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
                case ISSDK_REPORT_PRESSURE:
                    /*! Read MPL3115 registers. */
                    result = Register_I2C_Read(gMpl3115Driver.pCommDrv, &gMpl3115Driver.deviceInfo,
                                               gMpl3115Driver.slaveAddress, offset, bytes, buffer);
                    if (result != SENSOR_ERROR_NONE)
                    {
                        BleApp_WriteToUART("ERROR : Read [%d] bytes form [0x%02X] of MPL3115!\r\n", offset, bytes);
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
                    /*! Write FXLS8962 registers. */
                    result = Register_I2C_BlockWrite(gFxls8962Driver.pCommDrv, &gFxls8962Driver.deviceInfo,
                                                     gFxls8962Driver.slaveAddress, offset, buffer, bytes);
                    if (result != SENSOR_ERROR_NONE)
                    {
                        BleApp_WriteToUART("ERROR : Write [%d] bytes to [0x%02X] of FXLS8962!\r\n", offset, bytes);
                    }
                    break;
                case ISSDK_REPORT_MAGNETIC:
                    /*! Write MAG3110 registers. */
                    result = Register_I2C_BlockWrite(gMag3110Driver.pCommDrv, &gMag3110Driver.deviceInfo,
                                                     gMag3110Driver.slaveAddress, offset, buffer, bytes);
                    if (result != SENSOR_ERROR_NONE)
                    {
                        BleApp_WriteToUART("ERROR : Write [%d] bytes to [0x%02X] of MAG3110!\r\n", offset, bytes);
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
                case ISSDK_REPORT_PRESSURE:
                    /*! Write MPL3115 registers. */
                    result = Register_I2C_BlockWrite(gMpl3115Driver.pCommDrv, &gMpl3115Driver.deviceInfo,
                                                     gMpl3115Driver.slaveAddress, offset, buffer, bytes);
                    if (result != SENSOR_ERROR_NONE)
                    {
                        BleApp_WriteToUART("ERROR : Write [%d] bytes to [0x%02X] of MPL3115!\r\n", offset, bytes);
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

void FXLS8962_ISR_Callback(void)
{
    int32_t status;
    fxls8962_acceldata_t rawData;
    uint8_t dataReady, data[FXLS8962_DATA_SIZE];

    if (GpioIsPinIntPending(&gFxls8962IntCfg))
    {
        if (gReportMode == ISSDK_REPORT_FREEFALL)
        {
            /*! Read the Freefall event FLAGs from FXLS8962. */
            status = FXLS8962_I2C_ReadData(&gFxls8962Driver, cFxls8962SDCDEvent, &dataReady);
            if (SENSOR_ERROR_NONE != status)
            {
                BleApp_WriteToUART("ERROR : Read FXLS8962 Event Failed!\r\n");
                return;
            }

            if (FXLS8962_SDCD_INT_SRC2_WT_EA_EVENT_NO == (dataReady & FXLS8962_SDCD_INT_SRC2_WT_EA_MASK))
            { /* Return, if new event is not detected. */
                return;
            }

            /*! Display that a freefall event has been detected. */
            BleApp_WriteToUART("EVENT : Freefall detected...\r\n");
        }
        if (gReportMode == ISSDK_REPORT_PEDOMETER)
        {
            /*! Read raw sensor data from the FXLS8962 buffer one by one. */
            status = FXLS8962_I2C_ReadData(&gFxls8962Driver, cFxls8962OutputNormal, data);
            if (ARM_DRIVER_OK != status)
            {
                BleApp_WriteToUART("ERROR : Read FXLS8962 FIFO Failed!\r\n");
                return;
            }

            /*! Convert the raw sensor data for display to the debug port. */
            rawData.accel[0] = ((int16_t)data[1] << 8) | data[0];
            rawData.accel[0] *= 16; /* Pedometer requires signed 12-bit Left justified data @1024 counts/g. */
            rawData.accel[1] = ((int16_t)data[3] << 8) | data[2];
            rawData.accel[1] *= 16; /* Pedometer requires signed 12-bit Left justified data @1024 counts/g. */
            rawData.accel[2] = ((int16_t)data[5] << 8) | data[4];
            rawData.accel[2] *= 16; /* Pedometer requires signed 12-bit Left justified data @1024 counts/g. */

            /*!  Execute the pedometer Algorithm */
            pedometer_run(&gPedometer, (ped_accel_t *)&rawData.accel);

            if (gPedometer.status.stepcount != gLastReportedSteps)
            {
                FXLS8962_ReadPedometer();
                Led2Toggle();
            }
        }
        if (gReportMode == ISSDK_REPORT_ORIENTATION)
        {
            if (gMotionDetect)
            {
                /*! Display that a Motion event has been detected. */
                BleApp_WriteToUART("EVENT : Motion detected...\r\n");

                /* Apply FXLS8962 Stop Mode Configuration. */
                status = Sensor_I2C_Write(gFxls8962Driver.pCommDrv, &gFxls8962Driver.deviceInfo,
                                          gFxls8962Driver.slaveAddress, cFxls8962StopConfiguration);
                if (SENSOR_ERROR_NONE != status)
                {
                    BleApp_WriteToUART("ERROR : Write FXLS8962 Stop Configuration!\r\n");
                    return;
                }

                /* Apply FXLS8962 Configuration for Orientation Detection. */
                status = Sensor_I2C_Write(gFxls8962Driver.pCommDrv, &gFxls8962Driver.deviceInfo,
                                          gFxls8962Driver.slaveAddress, cFxls8962ConfigOrientDetect);
                if (SENSOR_ERROR_NONE != status)
                {
                    BleApp_WriteToUART("ERROR : Write FXLS8962 Orientation Configuration Failed!\r\n");
                    return;
                }
                gMotionDetect = false;
            }
            else
            {
                /*! Read the Orientation Status from the FXLS8962. */
                status = FXLS8962_I2C_ReadData(&gFxls8962Driver, cFxls8962OutputOrientStatus, &dataReady);
                if (ARM_DRIVER_OK != status)
                {
                    BleApp_WriteToUART("ERROR : Read Orientation Failed!\r\n");
                    return;
                }

                if (((dataReady & FXLS8962_ORIENT_STATUS_NEW_ORIENT_MASK) ==
                     FXLS8962_ORIENT_STATUS_NEW_ORIENT_CHANGED) &&
                    ((dataReady & FXLS8962_ORIENT_STATUS_LO_MASK) == FXLS8962_ORIENT_STATUS_LO_NOT_DETECTED))
                {
                    if ((dataReady & FXLS8962_ORIENT_STATUS_LAPO_MASK) == FXLS8962_ORIENT_STATUS_LAPO_UP)
                    {
                        BleApp_WriteToUART("EVENT : Portrait Up ...\r\n");
                    }
                    if ((dataReady & FXLS8962_ORIENT_STATUS_LAPO_MASK) == FXLS8962_ORIENT_STATUS_LAPO_DOWN)
                    {
                        BleApp_WriteToUART("EVENT : Portrait Down ...\r\n");
                    }
                    if ((dataReady & FXLS8962_ORIENT_STATUS_LAPO_MASK) == FXLS8962_ORIENT_STATUS_LAPO_RIGHT)
                    {
                        BleApp_WriteToUART("EVENT : Landscape Right ...\r\n");
                    }
                    if ((dataReady & FXLS8962_ORIENT_STATUS_LAPO_MASK) == FXLS8962_ORIENT_STATUS_LAPO_LEFT)
                    {
                        BleApp_WriteToUART("EVENT : Landscape Left ...\r\n");
                    }
                }

                if (((dataReady & FXLS8962_ORIENT_STATUS_NEW_ORIENT_MASK) ==
                     FXLS8962_ORIENT_STATUS_NEW_ORIENT_CHANGED) &&
                    ((dataReady & FXLS8962_ORIENT_STATUS_LO_MASK) == FXLS8962_ORIENT_STATUS_LO_DETECTED))
                {
                    if ((dataReady & FXLS8962_ORIENT_STATUS_BAFRO_MASK) == FXLS8962_ORIENT_STATUS_BAFRO_FRONT)
                    {
                        BleApp_WriteToUART("EVENT : Front Side ...\r\n");
                    }
                    if ((dataReady & FXLS8962_ORIENT_STATUS_BAFRO_MASK) == FXLS8962_ORIENT_STATUS_BAFRO_BACK)
                    {
                        BleApp_WriteToUART("EVENT : Back Side ...\r\n");
                    }
                }

                status = FXLS8962_I2C_ReadData(&gFxls8962Driver, cFxls8962SDCDEvent, &dataReady);
                if (ARM_DRIVER_OK != status)
                {
                    BleApp_WriteToUART("ERROR : Read SDCD Failed!\r\n");
                    return;
                }
                if ((dataReady & FXLS8962_SDCD_INT_SRC2_WT_EA_MASK) == FXLS8962_SDCD_INT_SRC2_WT_EA_EVENT_YES)
                {
                    /* Apply FXLS8962 Stop Mode Configuration. */
                    status = Sensor_I2C_Write(gFxls8962Driver.pCommDrv, &gFxls8962Driver.deviceInfo,
                                              gFxls8962Driver.slaveAddress, cFxls8962StopConfiguration);
                    if (SENSOR_ERROR_NONE != status)
                    {
                        BleApp_WriteToUART("ERROR : Write FXLS8962 Stop Configuration!\r\n");
                        return;
                    }

                    /* Apply FXLS8962 Configuration for Motion Detection. */
                    status = Sensor_I2C_Write(gFxls8962Driver.pCommDrv, &gFxls8962Driver.deviceInfo,
                                              gFxls8962Driver.slaveAddress, cFxls8962ConfigMotionDetect);
                    if (SENSOR_ERROR_NONE != status)
                    {
                        BleApp_WriteToUART("ERROR : Write FXLS8962 Motion Configuration Failed!\r\n");
                        return;
                    }
                    gMotionDetect = true;
                    BleApp_WriteToUART("EVENT : Going into Inactive Mode ...\r\n");
                }
            }
        }
        if (gReportMode == ISSDK_REPORT_ACCELERATION)
        {
            Led2Toggle();
            FXLS8962_ReadAcceleration();
        }
        if (gReportMode == ISSDK_REPORT_ECOMPASS)
        {
            FXLS8962_ReadCombo();
        }
        GpioInputPinInit(&gFxls8962IntCfg, 1);
        GpioClearPinIntFlag(&gFxls8962IntCfg);
    }
}

void MAG3110_ISR_Callback(void)
{
    /* For MAG3110 we need to read data to clear the ISR state of sensor after it is in STANDBY. */
    bool spuriousInt = true;

    if (GpioIsPinIntPending(&gMag3110IntCfg))
    {
        if (gReportMode == ISSDK_REPORT_MAGNETIC)
        {
            Led2Toggle();
            MAG3110_ReadMagnetic();
            spuriousInt = false;
        }
        if (gReportMode == ISSDK_REPORT_ECOMPASS)
        {
            Led2Toggle();
            MAG3110_ReadCombo();
            spuriousInt = false;
        }
        if (spuriousInt)
        {
            MAG3110_ClearState();
        }
        GpioInputPinInit(&gMag3110IntCfg, 1);
        GpioClearPinIntFlag(&gMag3110IntCfg);
    }
}

void FXAS21002_ISR_Callback(void)
{
    if (GpioIsPinIntPending(&gFxas21002IntCfg))
    {
        if (gReportMode == ISSDK_REPORT_ROTATION)
        {
            Led2Toggle();
            FXAS21002_ReadRotation();
        }
        GpioInputPinInit(&gFxas21002IntCfg, 1);
        GpioClearPinIntFlag(&gFxas21002IntCfg);
    }
}

void MPL3115_ISR_Callback(void)
{
    /* For MPL3115 we need to read data to clear the ISR state of sensor after it is in STANDBY. */
    bool spuriousInt = true;

    if (GpioIsPinIntPending(&gMpl3115IntCfg))
    {
        if (gReportMode == ISSDK_REPORT_PRESSURE)
        {
            Led2Toggle();
            MPL3115_ReadPressure();
            spuriousInt = false;
        }
        if (gReportMode == ISSDK_REPORT_ALTITUDE)
        {
            Led2Toggle();
            MPL3115_ReadAltitude();
            spuriousInt = false;
        }
        if (gReportMode == ISSDK_REPORT_TEMPERATURE)
        {
            Led2Toggle();
            MPL3115_ReadTemperature();
            spuriousInt = false;
        }
        if (spuriousInt)
        {
            MPL3115_ClearState();
        }
        GpioInputPinInit(&gMpl3115IntCfg, 1);
        GpioClearPinIntFlag(&gMpl3115IntCfg);
    }
}

void ISSDK_Initialize(void)
{
    int32_t status;

    BleApp_WriteToUART("Initializing Sensors for ISSDK AGMP03 BLE example.\r\n");

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

    /*! Initialize FXLS8962 sensor driver. */
    status = FXLS8962_I2C_Initialize(&gFxls8962Driver, &I2C_S_DRIVER, I2C_S_DEVICE_INDEX, FXLS8962_I2C_ADDR,
                                     FXLS8962_WHOAMI_VALUE);
    if (SENSOR_ERROR_NONE != status)
    {
        BleApp_WriteToUART("ERROR : FXLS8962 Initialization Failed!\r\n");
        return;
    }

    FXLS8962_I2C_SetIdleTask(&gFxls8962Driver, (registeridlefunction_t)SMC_SetPowerModeVlpr, SMC);
    /* Apply FXLS8962 Configuration based on the Register List */
    status = Sensor_I2C_Write(gFxls8962Driver.pCommDrv, &gFxls8962Driver.deviceInfo, gFxls8962Driver.slaveAddress,
                              cFxls8962ConfigNormal);
    if (ARM_DRIVER_OK != status)
    {
        BleApp_WriteToUART("ERROR : FXLS8962 Configuration Failed!\r\n");
        return;
    }
    BleApp_WriteToUART("Successfully Initialized FXLS8962.\r\n");

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

    /*! Initialize MAG3110 sensor driver. */
    status = MAG3110_I2C_Initialize(&gMag3110Driver, &I2C_S_DRIVER, I2C_S_DEVICE_INDEX, MAG3110_I2C_ADDR,
                                    MAG3110_WHOAMI_VALUE);
    if (SENSOR_ERROR_NONE != status)
    {
        BleApp_WriteToUART("ERROR : MAG3110 Initialization Failed!\r\n");
        return;
    }

    MAG3110_I2C_SetIdleTask(&gMag3110Driver, (registeridlefunction_t)SMC_SetPowerModeVlpr, SMC);
    /* Apply MAG3110 Configuration based on the Register List */
    status = Sensor_I2C_Write(gMag3110Driver.pCommDrv, &gMag3110Driver.deviceInfo, gMag3110Driver.slaveAddress,
                              cMag3110ConfigNormal);
    if (ARM_DRIVER_OK != status)
    {
        BleApp_WriteToUART("ERROR : MAG3110 Configuration Failed!\r\n");
        return;
    }
    BleApp_WriteToUART("Successfully Initialized MAG3110.\r\n");

    /*! Initialize the MPL3115 sensor driver. */
    status = MPL3115_I2C_Initialize(&gMpl3115Driver, &I2C_S_DRIVER, I2C_S_DEVICE_INDEX, MPL3115_I2C_ADDR,
                                    MPL3115_WHOAMI_VALUE);
    if (SENSOR_ERROR_NONE != status)
    {
        BleApp_WriteToUART("ERROR : MPL3115 Initialization Failed!\r\n");
        return;
    }

    MPL3115_I2C_SetIdleTask(&gMpl3115Driver, (registeridlefunction_t)SMC_SetPowerModeVlpr, SMC);
    /* Apply MPL3115 Configuration based on the Register List */
    status = Sensor_I2C_Write(gMpl3115Driver.pCommDrv, &gMpl3115Driver.deviceInfo, gMpl3115Driver.slaveAddress,
                              cMpl3115ConfigNormal);
    if (ARM_DRIVER_OK != status)
    {
        BleApp_WriteToUART("ERROR : MPL3115 Configuration Failed!\r\n");
        return;
    }
    BleApp_WriteToUART("Successfully Initialized MPL3115.\r\n");
    
    /* Initialize the pedometer */
    pedometer_init(&gPedometer);

    /* Initialize GPIO Configuration */
    gFxls8962IntCfg.pullSelect = pinPull_Up_c;
    gFxls8962IntCfg.interruptSelect = pinInt_LogicOne_c;
    gFxls8962IntCfg.gpioPin = FXLS8962_INT1.pinNumber;
    gFxls8962IntCfg.gpioPort = (gpioPort_t)FXLS8962_INT1.portNumber;
    GpioInstallIsr(FXLS8962_ISR_Callback, gGpioIsrPrioNormal_c, gGpioDefaultNvicPrio_c, &gFxls8962IntCfg);
    GpioInputPinInit(&gFxls8962IntCfg, 1);

    gMag3110IntCfg.pullSelect = pinPull_Up_c;
    gMag3110IntCfg.interruptSelect = pinInt_LogicOne_c;
    gMag3110IntCfg.gpioPin = MAG3110_INT1.pinNumber;
    gMag3110IntCfg.gpioPort = (gpioPort_t)MAG3110_INT1.portNumber;
    GpioInstallIsr(MAG3110_ISR_Callback, gGpioIsrPrioNormal_c, gGpioDefaultNvicPrio_c, &gMag3110IntCfg);
    GpioInputPinInit(&gMag3110IntCfg, 1);

    gFxas21002IntCfg.pullSelect = pinPull_Up_c;
    gFxas21002IntCfg.interruptSelect = pinInt_LogicOne_c;
    gFxas21002IntCfg.gpioPin = FXAS21002_INT2.pinNumber;
    gFxas21002IntCfg.gpioPort = (gpioPort_t)FXAS21002_INT2.portNumber;
    GpioInstallIsr(FXAS21002_ISR_Callback, gGpioIsrPrioNormal_c, gGpioDefaultNvicPrio_c, &gFxas21002IntCfg);
    GpioInputPinInit(&gFxas21002IntCfg, 1);

    gMpl3115IntCfg.pullSelect = pinPull_Up_c;
    gMpl3115IntCfg.interruptSelect = pinInt_LogicOne_c;
    gMpl3115IntCfg.gpioPin = MPL3115_INT1.pinNumber;
    gMpl3115IntCfg.gpioPort = (gpioPort_t)MPL3115_INT1.portNumber;
    GpioInstallIsr(MPL3115_ISR_Callback, gGpioIsrPrioNormal_c, gGpioDefaultNvicPrio_c, &gMpl3115IntCfg);
    GpioInputPinInit(&gMpl3115IntCfg, 1); /* This Pin is also used as SDW_CLK and causes the debugger to crash */

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
                case ISSDK_REPORT_TEMPERATURE:
                case ISSDK_REPORT_PRESSURE:
                    /* Put MPL3115 device into active mode.*/
                    status = Sensor_I2C_Write(gMpl3115Driver.pCommDrv, &gMpl3115Driver.deviceInfo,
                                              gMpl3115Driver.slaveAddress, cMpl3115StartConfiguration);
                    if (ARM_DRIVER_OK != status)
                    {
                        BleApp_WriteToUART("ERROR : Failed to put MPL3115 to Active Mode!\r\n");
                    }
                    else
                    {
                        BleApp_WriteToUART("Successfully put MPL3115 to Active Mode.\r\n");
                    }
                    break;
                case ISSDK_REPORT_ALTITUDE:
                    /* Put MPL3115 device into altitude and active mode.*/
                    status = Sensor_I2C_Write(gMpl3115Driver.pCommDrv, &gMpl3115Driver.deviceInfo,
                                              gMpl3115Driver.slaveAddress, cMpl3115StartAltConfiguration);
                    if (ARM_DRIVER_OK != status)
                    {
                        BleApp_WriteToUART("ERROR : Failed to put MPL3115 to Active ALT Mode!\r\n");
                    }
                    else
                    {
                        BleApp_WriteToUART("Successfully put MPL3115 to Active ALT Mode.\r\n");
                    }
                    break;
                case ISSDK_REPORT_ACCELERATION:
                    /* Put FXLS8962 device into active mode.*/
                    status = Sensor_I2C_Write(gFxls8962Driver.pCommDrv, &gFxls8962Driver.deviceInfo,
                                              gFxls8962Driver.slaveAddress, cFxls8962StartIntConfiguration);
                    if (ARM_DRIVER_OK != status)
                    {
                        BleApp_WriteToUART("ERROR : Failed to put FXLS8962 to Active Mode!\r\n");
                    }
                    else
                    {
                        BleApp_WriteToUART("Successfully put FXLS8962 to Active Mode.\r\n");
                    }
                    break;
                case ISSDK_REPORT_FREEFALL:
                    /* Put FXLS8962 into FreeFall detection mode.*/
                    status = Sensor_I2C_Write(gFxls8962Driver.pCommDrv, &gFxls8962Driver.deviceInfo,
                                              gFxls8962Driver.slaveAddress, cFxls8962StartFreeFallConfiguration);
                    if (ARM_DRIVER_OK != status)
                    {
                        BleApp_WriteToUART("ERROR : Failed to put FXLS8962 to FreeFall Mode!\r\n");
                    }
                    else
                    {
                        BleApp_WriteToUART("Successfully put FXLS8962 to FreeFall Mode.\r\n");
                        BleApp_WriteToHost("Waiting for Freefall Event...\n");
                    }
                    break;
                case ISSDK_REPORT_ORIENTATION:
                    /* Put FXLS8962 into Motion detection mode.*/
                    status = Sensor_I2C_Write(gFxls8962Driver.pCommDrv, &gFxls8962Driver.deviceInfo,
                                              gFxls8962Driver.slaveAddress, cFxls8962ConfigMotionDetect);
                    if (ARM_DRIVER_OK != status)
                    {
                        BleApp_WriteToUART("ERROR : Failed to put FXLS8962 to Motion Detect Mode!\r\n");
                    }
                    else
                    {
                        BleApp_WriteToUART("Successfully put FXLS8962 to Motion Detect Mode.\r\n");
                        BleApp_WriteToHost("Waiting for Motion Event...\n");
                        gMotionDetect = true;
                    }
                    break;
                case ISSDK_REPORT_PEDOMETER:
                    /* Configure the pedometer algorithm */
                    pedometer_configure(&gPedometer, &cPedoConfig);

                    /* Put MMA865x into Pedometer detection mode.*/
                    status = Sensor_I2C_Write(gFxls8962Driver.pCommDrv, &gFxls8962Driver.deviceInfo,
                                              gFxls8962Driver.slaveAddress, cFxls8962StartPedometerConfiguration);
                    if (ARM_DRIVER_OK != status)
                    {
                        BleApp_WriteToUART("ERROR : Failed to put FXLS8962 to Pedometer Mode!\r\n");
                    }
                    else
                    {
                        BleApp_WriteToUART("Successfully put FXLS8962 to Pedometer Mode.\r\n");
                        BleApp_WriteToHost("Waiting for Steps...\n");
                    }
                    break;
                case ISSDK_REPORT_MAGNETIC:
                    /* Put MAG3110 device into active mode.*/
                    status = Sensor_I2C_Write(gMag3110Driver.pCommDrv, &gMag3110Driver.deviceInfo,
                                              gMag3110Driver.slaveAddress, cMag3110StartConfiguration);
                    if (ARM_DRIVER_OK != status)
                    {
                        BleApp_WriteToUART("ERROR : Failed to put MAG3110 to Active Mode!\r\n");
                    }
                    else
                    {
                        BleApp_WriteToUART("Successfully put MAG3110 to Active Mode.\r\n");
                    }
                    break;
                case ISSDK_REPORT_ECOMPASS:
                    /* Put FXLS8962 device into active mode.*/
                    status = Sensor_I2C_Write(gFxls8962Driver.pCommDrv, &gFxls8962Driver.deviceInfo,
                                              gFxls8962Driver.slaveAddress, cFxls8962StartIntConfiguration);
                    if (ARM_DRIVER_OK != status)
                    {
                        BleApp_WriteToUART("ERROR : Failed to put FXLS8962 to Active Mode!\r\n");
                    }
                    else
                    {
                        BleApp_WriteToUART("Successfully put FXLS8962 to Active Mode.\r\n");
                    }
                    /* Put MAG3110 device into active mode.*/
                    status = Sensor_I2C_Write(gMag3110Driver.pCommDrv, &gMag3110Driver.deviceInfo,
                                              gMag3110Driver.slaveAddress, cMag3110StartConfiguration);
                    if (ARM_DRIVER_OK != status)
                    {
                        BleApp_WriteToUART("ERROR : Failed to put MAG3110 to Active Mode!\r\n");
                    }
                    else
                    {
                        BleApp_WriteToUART("Successfully put MAG3110 to Active Mode.\r\n");
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
                case ISSDK_REPORT_TEMPERATURE:
                case ISSDK_REPORT_PRESSURE:
                case ISSDK_REPORT_ALTITUDE:
                    /* Since MPL3115 is in OST Mode we do not need to set it to STANDBY explicitly. */
                    break;
                case ISSDK_REPORT_ACCELERATION:
                case ISSDK_REPORT_FREEFALL:
                case ISSDK_REPORT_PEDOMETER:
                case ISSDK_REPORT_ORIENTATION:
                    /* Put FXLS8962 into standby mode.*/
                    status = Sensor_I2C_Write(gFxls8962Driver.pCommDrv, &gFxls8962Driver.deviceInfo,
                                              gFxls8962Driver.slaveAddress, cFxls8962StopConfiguration);
                    if (ARM_DRIVER_OK != status)
                    {
                        BleApp_WriteToUART("ERROR : Failed to put FXLS8962 to Standby Mode!\r\n");
                    }
                    else
                    {
                        BleApp_WriteToUART("Successfully put FXLS8962 to Standby Mode.\r\n");
                    }
                    break;
                case ISSDK_REPORT_MAGNETIC:
                    /* Put MAG3110 into standby mode.*/
                    status = Sensor_I2C_Write(gMag3110Driver.pCommDrv, &gMag3110Driver.deviceInfo,
                                              gMag3110Driver.slaveAddress, cMag3110StopConfiguration);
                    if (ARM_DRIVER_OK != status)
                    {
                        BleApp_WriteToUART("ERROR : Failed to put MAG3110 to Standby Mode!\r\n");
                    }
                    else
                    {
                        BleApp_WriteToUART("Successfully put MAG3110 to Standby Mode.\r\n");
                    }
                    break;
                case ISSDK_REPORT_ECOMPASS:
                    /* Put FXLS8962 into standby mode.*/
                    status = Sensor_I2C_Write(gFxls8962Driver.pCommDrv, &gFxls8962Driver.deviceInfo,
                                              gFxls8962Driver.slaveAddress, cFxls8962StopConfiguration);
                    if (ARM_DRIVER_OK != status)
                    {
                        BleApp_WriteToUART("ERROR : Failed to put FXLS8962 to Standby Mode!\r\n");
                    }
                    else
                    {
                        BleApp_WriteToUART("Successfully put FXLS8962 to Standby Mode.\r\n");
                    }
                    /* Put MAG3110 into standby mode.*/
                    status = Sensor_I2C_Write(gMag3110Driver.pCommDrv, &gMag3110Driver.deviceInfo,
                                              gMag3110Driver.slaveAddress, cMag3110StopConfiguration);
                    if (ARM_DRIVER_OK != status)
                    {
                        BleApp_WriteToUART("ERROR : Failed to put MAG3110 to Standby Mode!\r\n");
                    }
                    else
                    {
                        BleApp_WriteToUART("Successfully put MAG3110 to Standby Mode.\r\n");
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
            BleApp_WriteToUART("Target is FXLS8962.\r\n");
        }
        else if (0 == strncasecmp(hostCommand->rliCmd.device, ISSDK_REGISTER_MAG_TAG, strlen(ISSDK_REGISTER_MAG_TAG)))
        {
            target = ISSDK_REPORT_MAGNETIC;
            BleApp_WriteToUART("Target is MAG3110.\r\n");
        }
        else if (0 == strncasecmp(hostCommand->rliCmd.device, ISSDK_REGISTER_GYRO_TAG, strlen(ISSDK_REGISTER_GYRO_TAG)))
        {
            target = ISSDK_REPORT_ROTATION;
            BleApp_WriteToUART("Target is FXAS21002.\r\n");
        }
        else if (0 == strncasecmp(hostCommand->rliCmd.device, ISSDK_REGISTER_PRESSURE_TAG, strlen(ISSDK_REGISTER_PRESSURE_TAG)))
        {
            target = ISSDK_REPORT_PRESSURE;
            BleApp_WriteToUART("Target is MPL3115.\r\n");
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
                    status = AGMP03_RegisterInterface(mode, target, offset, bytes, pBuffer);
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
                    status = AGMP03_RegisterInterface(mode, target, offset, bytes, pBuffer);
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
        /* Check for User Interface PRESSURE START command */
        else if (0 ==
                 strncasecmp(hostCommand->modeCmd.mode, ISSDK_REPORT_PRESSURE_STR, strlen(ISSDK_REPORT_PRESSURE_STR)))
        {
            /* Change Sensor Configuration to Barometer Mode */
            ISSDK_Configure(ISSDK_STANDBY_MODE);
            gReportMode = ISSDK_REPORT_PRESSURE;
            ISSDK_Configure(ISSDK_ACTIVE_MODE);

            BleApp_WriteToUART("Switched To Barometer Mode.\r\n");
            return true;
        }
        /* Check for User Interface ALTITUDE START command */
        else if (0 ==
                 strncasecmp(hostCommand->modeCmd.mode, ISSDK_REPORT_ALTITUDE_STR, strlen(ISSDK_REPORT_ALTITUDE_STR)))
        {
            /* Change Sensor Configuration to Altimeter Mode */
            ISSDK_Configure(ISSDK_STANDBY_MODE);
            gReportMode = ISSDK_REPORT_ALTITUDE;
            ISSDK_Configure(ISSDK_ACTIVE_MODE);

            BleApp_WriteToUART("Switched To Altimeter Mode.\r\n");
            return true;
        }
        /* Check for User Interface TEMPERATURE START command */
        else if (0 == strncasecmp(hostCommand->modeCmd.mode, ISSDK_REPORT_TEMPERATURE_STR,
                                  strlen(ISSDK_REPORT_TEMPERATURE_STR)))
        {
            /* Change Sensor Configuration to Thermometer Mode */
            ISSDK_Configure(ISSDK_STANDBY_MODE);
            gReportMode = ISSDK_REPORT_TEMPERATURE;
            ISSDK_Configure(ISSDK_ACTIVE_MODE);

            BleApp_WriteToUART("Switched To Thermometer Mode.\r\n");
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
