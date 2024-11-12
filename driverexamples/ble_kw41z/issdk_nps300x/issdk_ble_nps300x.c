/*
 * Copyright (c) 2015 - 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file issdk_ble_nps300x.c
 * @brief The issdk_ble_nps300x.c file implements the ISSDK DP300x sensor driver
 *        example demonstration using BLE Wireless UART adapter.
 * Supported Modes:
 *   MODE   = 'STANDBY'       (7 Fixed Characters to Stop All Sensor Data)
 *   MODE   = 'BAROMETER'     (9 Fixed Characters to Start Barometer Data)
 *   MODE   = 'THRESHOLD'     (9 Fixed Characters to Start Threshold Data)
 *   MODE   = 'THERMOMETER'   (11 Fixed Characters to Start Temperature Data)
 */

/* Standard C Includes */
#include <stdio.h>

/* CMSIS Includes */
#include "Driver_I2C.h"

/* ISSDK Includes */
#include "issdk_hal.h"
#include "diff_p_drv.h"

/* ISSDK Interface Include */
#include "issdk_ble_interface.h"

/* BLE framework Includes */
#include "LED.h"
#include "MemManager.h"
#include "GPIO_Adapter.h"

/*******************************************************************************
 * Macro Definitions
 ******************************************************************************/
#define DIFF_P_PRESSURE_DATA_SIZE (2)    /* 2 byte Pressure. */
#define DIFF_P_TEMPERATURE_DATA_SIZE (1) /* 1 byte Temperature. */
/* Pressure Thresholds are 700, 800, 900 Counts */
#define P_TGT0_MSB 0x02
#define P_TGT0_LSB 0xBC
#define P_TGT1_MSB 0x03
#define P_TGT1_LSB 0x20
#define P_TGT2_MSB 0x03
#define P_TGT2_LSB 0x84
/* Temperature Threshold 25C */
#define T_TGT  0x19

/*******************************************************************************
 * Constants
 ******************************************************************************/
/*! @brief Register settings for Normal mode. */
const registerwritelist_t cDiffPConfigInitialize[] = {
    {DIFF_P_CTRL_REG1, DIFF_P_CTRL_REG1_OSR_OSR512, DIFF_P_CTRL_REG1_OSR_MASK},  
    {DIFF_P_CTRL_REG2, DIFF_P_CTRL_REG2_ODR_ODR6P25, DIFF_P_CTRL_REG2_ODR_MASK},
    {DIFF_P_CTRL_REG3, DIFF_P_CTRL_REG3_IPOL1_ACTIVE_HIGH, DIFF_P_CTRL_REG3_IPOL1_MASK},
    __END_WRITE_DATA__};

/*! @brief Register settings for Temperature mode. */
const registerwritelist_t cDiffPConfigStartTemperature[] = {
    {DIFF_P_INT_MASK0, DIFF_P_INT_MASK0_TDR_INT_EN, DIFF_P_INT_MASK0_TDR_MASK},
    {DIFF_P_CTRL_REG1, DIFF_P_CTRL_REG1_SBYB_ACTIVE, DIFF_P_CTRL_REG1_SBYB_MASK},
    __END_WRITE_DATA__};

/*! @brief Register settings for Pressure mode. */
const registerwritelist_t cDiffPConfigStartPressure[] = {
    {DIFF_P_INT_MASK0, DIFF_P_INT_MASK0_PDR_INT_EN, DIFF_P_INT_MASK0_PDR_MASK},
    {DIFF_P_CTRL_REG1, DIFF_P_CTRL_REG1_SBYB_ACTIVE, DIFF_P_CTRL_REG1_SBYB_MASK},
    __END_WRITE_DATA__};

/*! @brief Register settings for Threshold mode. */
const registerwritelist_t cDiffPConfigStartThreshold[] = {
    {DIFF_P_P_TGT0_LSB, P_TGT0_LSB, 0},
    {DIFF_P_P_TGT0_MSB, P_TGT0_MSB, 0},
    {DIFF_P_P_TGT1_LSB, P_TGT1_LSB, 0},
    {DIFF_P_P_TGT1_MSB, P_TGT1_MSB, 0},
    {DIFF_P_P_TGT2_LSB, P_TGT2_LSB, 0},
    {DIFF_P_P_TGT2_MSB, P_TGT2_MSB, 0},
    {DIFF_P_T_TGT, T_TGT, 0},
    {DIFF_P_INT_MASK1,
        DIFF_P_INT_MASK1_P_TGT0_INT_EN | DIFF_P_INT_MASK1_P_TGT1_INT_EN | DIFF_P_INT_MASK1_P_TGT2_INT_EN | DIFF_P_INT_MASK1_T_TGT_INT_EN,
        DIFF_P_INT_MASK1_P_TGT0_MASK | DIFF_P_INT_MASK1_P_TGT1_MASK | DIFF_P_INT_MASK1_P_TGT2_MASK | DIFF_P_INT_MASK1_T_TGT_MASK},
    {DIFF_P_CTRL_REG1, DIFF_P_CTRL_REG1_SBYB_ACTIVE, DIFF_P_CTRL_REG1_SBYB_MASK},
    __END_WRITE_DATA__};

/*! @brief Register settings for Threshold mode. */
const registerwritelist_t cDiffPConfigStop[] = {
    {DIFF_P_CTRL_REG1, DIFF_P_CTRL_REG1_SBYB_STANDBY, DIFF_P_CTRL_REG1_SBYB_MASK},
    {DIFF_P_INT_STATUS_0, 0x00, DIFF_P_INT_STATUS_0_TDR_MASK | DIFF_P_INT_STATUS_0_PDR_MASK},
    {DIFF_P_INT_STATUS_1, 0x00, DIFF_P_INT_STATUS_1_P_TGT0_MASK},
    {DIFF_P_INT_MASK0, 0x00, DIFF_P_INT_MASK0_PDR_MASK | DIFF_P_INT_MASK0_TDR_MASK},
    {DIFF_P_INT_MASK1, 0x00, DIFF_P_INT_MASK1_P_TGT0_MASK | DIFF_P_INT_MASK1_P_TGT1_MASK | DIFF_P_INT_MASK1_P_TGT2_MASK | DIFF_P_INT_MASK1_T_TGT_MASK},
    {DIFF_P_P_TGT0_LSB, 0x00, 0},
    {DIFF_P_P_TGT0_MSB, 0x00, 0},
    {DIFF_P_P_TGT1_LSB, 0x00, 0},
    {DIFF_P_P_TGT1_MSB, 0x00, 0},
    {DIFF_P_P_TGT2_LSB, 0x00, 0},
    {DIFF_P_P_TGT2_MSB, 0x00, 0},
    {DIFF_P_T_TGT, 0x00, 0},
    __END_WRITE_DATA__};

/*! @brief Register settings for Clearing Pressure and Temperature Data Ready Bits. */
const registerwritelist_t cDiffPClearStatus0[] = {
    {DIFF_P_INT_STATUS_0, 0x00, DIFF_P_INT_STATUS_0_TDR_MASK | DIFF_P_INT_STATUS_0_PDR_MASK}, __END_WRITE_DATA__};

const registerwritelist_t cDiffPClearStatus1[] = {{DIFF_P_INT_STATUS_1, 0x00, DIFF_P_INT_STATUS_1_P_TGT0_MASK | DIFF_P_INT_STATUS_1_P_TGT1_MASK | DIFF_P_INT_STATUS_1_P_TGT2_MASK | DIFF_P_INT_STATUS_1_T_TGT_MASK},
                                                  __END_WRITE_DATA__};

/*! @brief Address of Status Registers. */
const registerreadlist_t cDiffPStatus[] = {{.readFrom = DIFF_P_INT_STATUS_0, .numBytes = 2}, __END_READ_DATA__};

/*! @brief Address and size of Raw Pressure Data in Normal Mode. */
const registerreadlist_t cDiffPOutputPressure[] = {
    {.readFrom = DIFF_P_OUT_P_LSB, .numBytes = DIFF_P_PRESSURE_DATA_SIZE}, __END_READ_DATA__};

/*! @brief Address and size of Raw Temperature Data in Normal Mode. */
const registerreadlist_t cDiffPOutputTemperature[] = {
    {.readFrom = DIFF_P_OUT_T, .numBytes = DIFF_P_TEMPERATURE_DATA_SIZE}, __END_READ_DATA__};

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
gpioInputPinConfig_t gNps300xIntCfg;
diff_p_i2c_sensorhandle_t gNps300xDriver;
ARM_DRIVER_I2C *gI2cDriver = &I2C_S_DRIVER;
uint8_t gReportMode, gReportFormat, gConversionFactor;

/************************************************************************************
* Functions
************************************************************************************/
/* Definition for adding delays adjusted for FRDM-KW41Z. */
void BOARD_DELAY_ms(uint32_t delay_ms)
{
    delay_ms *= 10000;
    while(delay_ms--)
    {
        __NOP();
    }
}

/*! @brief Function to Read Temperature Data */
static void NPS300x_ReadTemperature(void)
{
    float tempC;
    char tempCs[8] = {0};
    int32_t status, tempCi;
    diff_p_pressuredata_t rawData;
    uint8_t data[DIFF_P_TEMPERATURE_DATA_SIZE];

    /*! Read new raw Temperature data from the DIFF_P. */
    status = DIFF_P_I2C_ReadData(&gNps300xDriver, cDiffPOutputTemperature, data);
    if (ARM_DRIVER_OK != status)
    {
        BleApp_WriteToUART("ERROR : Read Temperature Failed!\r\n");
        return;
    }

    /*! Explicitly clear the Status Bit. */
    status = Sensor_I2C_Write(gNps300xDriver.pCommDrv, &gNps300xDriver.deviceInfo, gNps300xDriver.slaveAddress,
                              cDiffPClearStatus0);
    if (ARM_DRIVER_OK != status)
    {
        BleApp_WriteToUART("ERROR : Clear Temperature Bit Failed!\r\n");
        return;
    }

    /*! Process the sample and convert the raw sensor data. */
    rawData.temperature = (int8_t)(data[0]);
    tempC = rawData.temperature;

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

/*! @brief Function to Read Pressure Data */
static void NPS300x_ReadPressure(void)
{
    float presP;
    char presPs[8] = {0};
    int32_t status, presPi;
    diff_p_pressuredata_t rawData;
    uint8_t data[DIFF_P_PRESSURE_DATA_SIZE];

    /*! Read new raw sensor data from the DIFF_P. */
    status = DIFF_P_I2C_ReadData(&gNps300xDriver, cDiffPOutputPressure, data);
    if (ARM_DRIVER_OK != status)
    {
        BleApp_WriteToUART("ERROR : Read Pressure Failed!\r\n");
        return;
    }

    /*! Explicitly clear the Status Bits. */
    status = Sensor_I2C_Write(gNps300xDriver.pCommDrv, &gNps300xDriver.deviceInfo, gNps300xDriver.slaveAddress,
                              cDiffPClearStatus0);
    if (ARM_DRIVER_OK != status)
    {
        BleApp_WriteToUART("ERROR : Clear Pressure Bit Failed!\r\n");
        return;
    }

    /*! Process the sample and convert the raw sensor data. */
    rawData.pressure = ((int16_t)(data[1]) << 8) | data[0];
    presP = rawData.pressure / gConversionFactor;
	presP /= 1000; /* Will give kPa */

    if (gReportFormat == ISSDK_READ_RAW)
    {
        /* Send raw data over the air */
        BleApp_WriteToHost("PRESSURE: 0x%08X\n", (uint32_t)rawData.pressure);
    }
    if (gReportFormat == ISSDK_READ_NORMAL)
    {
        ISSDK_GetStringOfFloat(presP, presPs);
        /* Send data over the air */
        BleApp_WriteToHost("PRESSURE: %skPa\n", presPs);
    }
    if (gReportFormat == ISSDK_READ_STREAM)
    {
        presPi = ISSDK_GetDecimalForFloat(presP);
        /* Send Data Stream over the air */
        BleApp_WriteToHost("%s:%08X\n", ISSDK_PRESSURE_STREAM_HDR_STR, (uint32_t)presPi);
    }
}

/*! @brief Function to Read and Write Sensor registers */
static int32_t NPS300x_RegisterInterface(uint8_t opCode, uint8_t mode, uint8_t offset, uint8_t bytes, uint8_t *buffer)
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
                case ISSDK_REPORT_PRESSURE:
                    /*! Read NPS300x registers. */
                    result = Register_I2C_Read(gNps300xDriver.pCommDrv, &gNps300xDriver.deviceInfo,
                                               gNps300xDriver.slaveAddress, offset, bytes, buffer);
                    if (result != SENSOR_ERROR_NONE)
                    {
                        BleApp_WriteToUART("ERROR : Read [%d] bytes form [0x%02X] of DIFF-P!\r\n", offset, bytes);
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
                case ISSDK_REPORT_PRESSURE:
                    /*! Write NPS300x registers. */
                    result = Register_I2C_BlockWrite(gNps300xDriver.pCommDrv, &gNps300xDriver.deviceInfo,
                                                     gNps300xDriver.slaveAddress, offset, buffer, bytes);
                    if (result != SENSOR_ERROR_NONE)
                    {
                        BleApp_WriteToUART("ERROR : Write [%d] bytes to [0x%02X] of DIFF!\r\n", offset, bytes);
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

void NPS300x_ISR_Callback(void)
{
    int32_t status;
    uint8_t dataReady[2];

    if (GpioIsPinIntPending(&gNps300xIntCfg))
    {
        /* Read INT_STATUS registers */
        status = DIFF_P_I2C_ReadData(&gNps300xDriver, cDiffPStatus, dataReady);
        if (ARM_DRIVER_OK != status)
        {
            BleApp_WriteToUART("ERROR : Read DIFF_P Status Failed!\r\n");
        }

        /*! Check for data ready bits from the DIFF_P. */
        if (gReportMode == ISSDK_REPORT_TEMPERATURE &&
            (DIFF_P_INT_STATUS_0_TDR_DRDY == (dataReady[0] & DIFF_P_INT_STATUS_0_TDR_MASK)))
        {
            NPS300x_ReadTemperature();
            Led2Toggle();
        }
        if (gReportMode == ISSDK_REPORT_PRESSURE &&
            (DIFF_P_INT_STATUS_0_PDR_DRDY == (dataReady[0] & DIFF_P_INT_STATUS_0_PDR_MASK)))
        {
            NPS300x_ReadPressure();
            Led2Toggle();
        }
        if (gReportMode == ISSDK_REPORT_THRESHOLD)
        {
            if(DIFF_P_INT_STATUS_1_P_TGT0_REACHED == (dataReady[1] & DIFF_P_INT_STATUS_1_P_TGT0_MASK))
            {
                /*! Display that a Threshold 0 event has been detected. */
                BleApp_WriteToUART("EVENT : Alert Level detected...\r\n");
            }
            if(DIFF_P_INT_STATUS_1_P_TGT1_REACHED == (dataReady[1] & DIFF_P_INT_STATUS_1_P_TGT1_MASK))
            {
                /*! Display that a Threshold 1 event has been detected. */
                BleApp_WriteToUART("EVENT : Warning Level detected...\r\n");
            }
            if(DIFF_P_INT_STATUS_1_P_TGT2_REACHED == (dataReady[1] & DIFF_P_INT_STATUS_1_P_TGT2_MASK))
            {
                /*! Display that a Threshold 2 event has been detected. */
                BleApp_WriteToUART("EVENT : Critical detected...\r\n");
            }
            if(DIFF_P_INT_STATUS_1_T_TGT_REACHED == (dataReady[1] & DIFF_P_INT_STATUS_1_T_TGT_MASK))
            {
                /*! Display that a Threshold event has been detected. */
                BleApp_WriteToUART("EVENT : Malfunction detected...\r\n");
            }
            /*! Explicitly clear the Status Bits. */
            status = Sensor_I2C_Write(gNps300xDriver.pCommDrv, &gNps300xDriver.deviceInfo, gNps300xDriver.slaveAddress,
                                      cDiffPClearStatus1);
            if (ARM_DRIVER_OK != status)
            {
                BleApp_WriteToUART("ERROR : Clear Threshold Bit Failed!\r\n");
            }
        }
        GpioInputPinInit(&gNps300xIntCfg, 1);
        GpioClearPinIntFlag(&gNps300xIntCfg);
    }
}

void ISSDK_Initialize(void)
{
    uint8_t whoAmI;
    int32_t status;

    BleApp_WriteToUART("Initializing Sensors for ISSDK DP300x BLE example.\r\n");

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

    do
    { /*! Initialize DIFF_P sensor driver. */
        status = DIFF_P_I2C_Initialize(&gNps300xDriver, &I2C_S_DRIVER, I2C_S_DEVICE_INDEX, DIFF_P_I2C_ADDR,
                                       DIFF_P_NPS3000VV_WHOAMI_VALUE);
        if (SENSOR_ERROR_NONE == status)
        {
            whoAmI = DIFF_P_NPS3000VV_WHOAMI_VALUE;
            gConversionFactor = NPS3000VV_PRESSURE_DIV_FACTOR;
            break;
        }
        status = DIFF_P_I2C_Initialize(&gNps300xDriver, &I2C_S_DRIVER, I2C_S_DEVICE_INDEX, DIFF_P_I2C_ADDR,
                                       DIFF_P_NPS3001DV_WHOAMI_VALUE);
        if (SENSOR_ERROR_NONE == status)
        {
            whoAmI = DIFF_P_NPS3001DV_WHOAMI_VALUE;
            gConversionFactor = NPS3001DV_PRESSURE_DIV_FACTOR;
            break;
        }
        status = DIFF_P_I2C_Initialize(&gNps300xDriver, &I2C_S_DRIVER, I2C_S_DEVICE_INDEX, DIFF_P_I2C_ADDR,
                                       DIFF_P_NPS3002VV_WHOAMI_VALUE);
        if (SENSOR_ERROR_NONE == status)
        {
            whoAmI = DIFF_P_NPS3002VV_WHOAMI_VALUE;
            gConversionFactor = NPS3002VV_PRESSURE_DIV_FACTOR;
            break;
        }
        status = DIFF_P_I2C_Initialize(&gNps300xDriver, &I2C_S_DRIVER, I2C_S_DEVICE_INDEX, DIFF_P_I2C_ADDR,
                                       DIFF_P_NPS3005DV_WHOAMI_VALUE);
        if (SENSOR_ERROR_NONE == status)
        {
            whoAmI = DIFF_P_NPS3005DV_WHOAMI_VALUE;
            gConversionFactor = NPS3005DV_PRESSURE_DIV_FACTOR;
            break;
        }

        BleApp_WriteToUART("ERROR : NPS300x Initialization Failed!\r\n");
        return;
    } while (false);

    DIFF_P_I2C_SetIdleTask(&gNps300xDriver, (registeridlefunction_t)SMC_SetPowerModeVlpr, SMC);
    /* Apply NPS300x Configuration based on the Register List */
    status = Sensor_I2C_Write(gNps300xDriver.pCommDrv, &gNps300xDriver.deviceInfo, gNps300xDriver.slaveAddress,
                              cDiffPConfigInitialize);
    if (ARM_DRIVER_OK != status)
    {
        BleApp_WriteToUART("ERROR : DIFF-P Configuration Failed!\r\n");
        return;
    }
    BleApp_WriteToUART("Successfully Initialized DIFF-P with WHO_AM_I = [0x%X].\r\n", whoAmI);

    /* Initialize GPIO Configuration */
    gNps300xIntCfg.pullSelect = pinPull_Up_c;
    gNps300xIntCfg.interruptSelect = pinInt_LogicOne_c;
    gNps300xIntCfg.gpioPin = DIFF_P_INT1.pinNumber;
    gNps300xIntCfg.gpioPort = (gpioPort_t)DIFF_P_INT1.portNumber;
    /* Install NPS300x Event Callback INT */
    GpioInstallIsr(NPS300x_ISR_Callback, gGpioIsrPrioNormal_c, gGpioDefaultNvicPrio_c, &gNps300xIntCfg);
    GpioInputPinInit(&gNps300xIntCfg, 1);

    /* Set Default settings */
    gReportFormat = ISSDK_READ_NORMAL;
    gReportMode = ISSDK_REPORT_NONE;
    BleApp_WriteToUART("Report Mode is Standby.\r\n");
    BleApp_WriteToUART("Data Format is Converted.\r\n");
}

void ISSDK_Configure(uint8_t opCode)
{
    int32_t status;
    int8_t thresholdValue;
    int16_t thresholdValue0, thresholdValue1, thresholdValue2;

    switch (opCode)
    {
        case ISSDK_ACTIVE_MODE:
            switch (gReportMode)
            {
                case ISSDK_REPORT_NONE:
                    break;
                case ISSDK_REPORT_TEMPERATURE:
                    /* Put NPS300x device into active mode.*/
                    status = Sensor_I2C_Write(gNps300xDriver.pCommDrv, &gNps300xDriver.deviceInfo,
                                              gNps300xDriver.slaveAddress, cDiffPConfigStartTemperature);
                    if (ARM_DRIVER_OK != status)
                    {
                        BleApp_WriteToUART("ERROR : Failed to put NPS300x to Temperature Mode!\r\n");
                    }
                    else
                    {
                        BleApp_WriteToUART("Successfully put NPS300x to Temperature Mode.\r\n");
                    }
                    break;
                case ISSDK_REPORT_PRESSURE:
                    /* Put NPS300x into active mode.*/
                    status = Sensor_I2C_Write(gNps300xDriver.pCommDrv, &gNps300xDriver.deviceInfo,
                                              gNps300xDriver.slaveAddress, cDiffPConfigStartPressure);
                    if (ARM_DRIVER_OK != status)
                    {
                        BleApp_WriteToUART("ERROR : Failed to put NPS300x to Pressure Mode!\r\n");
                    }
                    else
                    {
                        BleApp_WriteToUART("Successfully put NPS300x to Pressure Mode.\r\n");
                    }
                    break;
                case ISSDK_REPORT_THRESHOLD:
                    /* Put NPS300x into Threshold detection mode.*/
                    status = Sensor_I2C_Write(gNps300xDriver.pCommDrv, &gNps300xDriver.deviceInfo,
                                              gNps300xDriver.slaveAddress, cDiffPConfigStartThreshold);
                    if (ARM_DRIVER_OK != status)
                    {
                        BleApp_WriteToUART("ERROR : Failed to put NPS300x to Threshold Mode!\r\n");
                    }
                    else
                    {
                        BleApp_WriteToUART("Successfully put NPS300x to Threshold Mode.\r\n");
                        thresholdValue0 = (((int16_t)(P_TGT0_MSB) << 8) | P_TGT0_LSB) / gConversionFactor;
                        thresholdValue1 = (((int16_t)(P_TGT1_MSB) << 8) | P_TGT1_LSB) / gConversionFactor;
                        thresholdValue2 = (((int16_t)(P_TGT2_MSB) << 8) | P_TGT2_LSB) / gConversionFactor;
                        thresholdValue =  T_TGT;
                        BleApp_WriteToHost("Waiting for :\nAlert@[%dPa]\nWarning@[%dPa]\nCritical@[%dPa]\nMalfunction@[%dC]...\n", thresholdValue0, thresholdValue1, thresholdValue2, thresholdValue);
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
                case ISSDK_REPORT_TEMPERATURE:
                case ISSDK_REPORT_PRESSURE:
                case ISSDK_REPORT_THRESHOLD:
                    /* Put NPS300x into standby mode.*/
                    status = Sensor_I2C_Write(gNps300xDriver.pCommDrv, &gNps300xDriver.deviceInfo,
                                              gNps300xDriver.slaveAddress, cDiffPConfigStop);
                    if (ARM_DRIVER_OK != status)
                    {
                        BleApp_WriteToUART("ERROR : Failed to put NPS300x to Standby Mode!\r\n");
                    }
                    else
                    {
                        BleApp_WriteToUART("Successfully put NPS300x to Standby Mode.\r\n");
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
        if (0 ==
            strncasecmp(hostCommand->rliCmd.device, ISSDK_REGISTER_PRESSURE_TAG, strlen(ISSDK_REGISTER_PRESSURE_TAG)))
        {
            target = ISSDK_REPORT_PRESSURE;
            BleApp_WriteToUART("Target is NPS300x.\r\n");
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
                    status = NPS300x_RegisterInterface(mode, target, offset, bytes, pBuffer);
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
                    status = NPS300x_RegisterInterface(mode, target, offset, bytes, pBuffer);
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
        /* Check for User Interface TEMPERATURE START command */
        else if (0 == strncasecmp(hostCommand->modeCmd.mode, ISSDK_REPORT_TEMPERATURE_STR,
                                  strlen(ISSDK_REPORT_TEMPERATURE_STR)))
        {
            /* Change Sensor Configuration to Accelerometer Mode */
            ISSDK_Configure(ISSDK_STANDBY_MODE);
            gReportMode = ISSDK_REPORT_TEMPERATURE;
            ISSDK_Configure(ISSDK_ACTIVE_MODE);

            BleApp_WriteToUART("Switched To Thermometer Mode.\r\n");
            return true;
        }
        /* Check for User Interface BAROMETER START command */
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
        /* Check for User Interface PRESSURE THRESHOLD START command */
        else if (0 ==
                 strncasecmp(hostCommand->modeCmd.mode, ISSDK_REPORT_THRESHOLD_STR, strlen(ISSDK_REPORT_THRESHOLD_STR)))
        {
            /* Change Sensor Configuration to Barometer Mode */
            ISSDK_Configure(ISSDK_STANDBY_MODE);
            gReportMode = ISSDK_REPORT_THRESHOLD;
            ISSDK_Configure(ISSDK_ACTIVE_MODE);

            BleApp_WriteToUART("Switched To Threshold Mode.\r\n");
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