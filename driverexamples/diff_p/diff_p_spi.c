/*
 * Copyright 2017-2018 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file diff_p_spi.c
 * @brief The diff_p_spi.c file implements the ISSDK DIFF_P sensor driver
 *        example demonstration with SPI Mode.
 */

//-----------------------------------------------------------------------
// SDK Includes
//-----------------------------------------------------------------------
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#include "fsl_debug_console.h"

//-----------------------------------------------------------------------
// CMSIS Includes
//-----------------------------------------------------------------------
#include "Driver_SPI.h"

/* ISSDK Includes */
#include "issdk_hal.h"
#include "diff_p_drv.h"
#include "gpio_driver.h"
#include "systick_utils.h"

//-----------------------------------------------------------------------
// Macros
//-----------------------------------------------------------------------
#define DIFF_P_DATA_SIZE (3) /* 2 byte Pressure and 1 byte Temperature. */

//-----------------------------------------------------------------------
// Constants
//-----------------------------------------------------------------------
/*! @brief Register settings for Normal (non buffered) mode. */
const registerwritelist_t cDiffPConfigNormal[] = {
    {DIFF_P_CTRL_REG2, DIFF_P_CTRL_REG2_ODR_ODR6P25, DIFF_P_CTRL_REG2_ODR_MASK},
    {DIFF_P_CTRL_REG1, DIFF_P_CTRL_REG1_OSR_OSR512, DIFF_P_CTRL_REG1_OSR_MASK},
    __END_WRITE_DATA__};

/*! @brief Register settings for Clearing Pressure and Temperature Data Ready Bits. */
const registerwritelist_t cDiffPClearStatusBits[] = {
    {DIFF_P_INT_STATUS_0, 0x00, DIFF_P_INT_STATUS_0_TDR_MASK | DIFF_P_INT_STATUS_0_PDR_MASK}, __END_WRITE_DATA__};

/*! @brief Address of Status Register. */
const registerreadlist_t cDiffPStatus[] = {{.readFrom = DIFF_P_INT_STATUS_0, .numBytes = 1}, __END_READ_DATA__};

/*! @brief Address and size of Raw Pressure+Temperature Data in Normal Mode. */
const registerreadlist_t cDiffPOutputNormal[] = {{.readFrom = DIFF_P_OUT_P_LSB, .numBytes = DIFF_P_DATA_SIZE},
                                                 __END_READ_DATA__};

//-----------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------
/*!
 * @brief Main function
 */
int main(void)
{
    int32_t status;
    int8_t tempInDegrees;
    float pressureInPascals;
    diff_p_pressuredata_t rawData;
    uint8_t whoAmI, conversionFactor;
    uint8_t dataReady, data[DIFF_P_DATA_SIZE];

    diff_p_spi_sensorhandle_t diff_pDriver;
    ARM_DRIVER_SPI *SPIdrv = &SPI_S_DRIVER;

    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_SystickEnable();
    BOARD_InitDebugConsole();

    PRINTF("\r\n ISSDK DIFF-P Sensor SPI Example !\r\n");

    /*! Initialize the I2C driver. */
    status = SPIdrv->Initialize(SPI_S_SIGNAL_EVENT);
    if (ARM_DRIVER_OK != status)
    {
        PRINTF("\r\n SPI Initialization Failed.\r\n");
        return -1;
    }

    /*! Set the I2C Power mode. */
    status = SPIdrv->PowerControl(ARM_POWER_FULL);
    if (ARM_DRIVER_OK != status)
    {
        PRINTF("\r\n SPI Power Mode setting Failed.\r\n");
        return -1;
    }

    /*! Set the I2C bus speed. */
    status = SPIdrv->Control(ARM_SPI_MODE_MASTER | ARM_SPI_CPOL0_CPHA0, SPI_S_BAUDRATE);
    if (ARM_DRIVER_OK != status)
    {
        PRINTF("\r\n SPI Control Mode setting Failed\r\n");
        return -1;
    }

    do
    { /*! Initialize DIFF_P sensor driver. */
        status = DIFF_P_SPI_Initialize(&diff_pDriver, &SPI_S_DRIVER, SPI_S_DEVICE_INDEX, &DIFF_P_CS,
                                       DIFF_P_NPS3000VV_WHOAMI_VALUE);
        if (SENSOR_ERROR_NONE == status)
        {
            whoAmI = DIFF_P_NPS3000VV_WHOAMI_VALUE;
            conversionFactor = NPS3000VV_PRESSURE_DIV_FACTOR;
            break;
        }
        status = DIFF_P_SPI_Initialize(&diff_pDriver, &SPI_S_DRIVER, SPI_S_DEVICE_INDEX, &DIFF_P_CS,
                                       DIFF_P_NPS3001DV_WHOAMI_VALUE);
        if (SENSOR_ERROR_NONE == status)
        {
            whoAmI = DIFF_P_NPS3001DV_WHOAMI_VALUE;
            conversionFactor = NPS3001DV_PRESSURE_DIV_FACTOR;
            break;
        }
        status = DIFF_P_SPI_Initialize(&diff_pDriver, &SPI_S_DRIVER, SPI_S_DEVICE_INDEX, &DIFF_P_CS,
                                       DIFF_P_NPS3002VV_WHOAMI_VALUE);
        if (SENSOR_ERROR_NONE == status)
        {
            whoAmI = DIFF_P_NPS3002VV_WHOAMI_VALUE;
            conversionFactor = NPS3002VV_PRESSURE_DIV_FACTOR;
            break;
        }
        status = DIFF_P_SPI_Initialize(&diff_pDriver, &SPI_S_DRIVER, SPI_S_DEVICE_INDEX, &DIFF_P_CS,
                                       DIFF_P_NPS3005DV_WHOAMI_VALUE);
        if (SENSOR_ERROR_NONE == status)
        {
            whoAmI = DIFF_P_NPS3005DV_WHOAMI_VALUE;
            conversionFactor = NPS3005DV_PRESSURE_DIV_FACTOR;
            break;
        }

        PRINTF("\r\n Sensor Initialization Failed.\r\n");
        return -1;
    } while (false);
    PRINTF("\r\n Initiliazed Sensor with WHO_AM_I = [0x%X].\r\n", whoAmI);

    /*!  Set the task to be executed while waiting for SPI transactions to complete. */
    DIFF_P_SPI_SetIdleTask(&diff_pDriver, (registeridlefunction_t)SMC_SetPowerModeVlpr, SMC);

    /*! Configure the DIFF_P sensor. */
    status = DIFF_P_SPI_Configure(&diff_pDriver, cDiffPConfigNormal);
    if (SENSOR_ERROR_NONE != status)
    {
        PRINTF("\r\nDIFF_P now active and entering data read loop...\r\n");
        return -1;
    }
    PRINTF("\r\n Successfully Applied Sensor Poll Configuration.\r\n");

    for (;;) /* Forever loop */
    {
        /* Read INT_STATUS register */
        status = DIFF_P_SPI_ReadData(&diff_pDriver, cDiffPStatus, &dataReady);
        if (ARM_DRIVER_OK != status)
        {
            PRINTF("\r\n Read Failed. \r\n");
            return -1;
        }

        /*! Check for both data ready bits from the DIFF_P. */
        if ((DIFF_P_INT_STATUS_0_PDR_DRDY | DIFF_P_INT_STATUS_0_TDR_DRDY) !=
            (dataReady & (DIFF_P_INT_STATUS_0_PDR_MASK | DIFF_P_INT_STATUS_0_TDR_MASK)))
        { /* Loop, if new samples are not available. */
            continue;
        }

        /*! Read new raw sensor data from the DIFF_P. */
        status = DIFF_P_SPI_ReadData(&diff_pDriver, cDiffPOutputNormal, data);
        if (ARM_DRIVER_OK != status)
        {
            PRINTF("\r\n Read Failed. \r\n");
            return -1;
        }

        /*! Explicitly clear the Status Bits. */
        status = Sensor_SPI_Write(diff_pDriver.pCommDrv, &diff_pDriver.deviceInfo, &diff_pDriver.slaveParams,
                                  cDiffPClearStatusBits);
        if (ARM_DRIVER_OK != status)
        {
            PRINTF("\r\n Write Failed. \r\n");
            return -1;
        }

        /*! Process the sample and convert the raw sensor data. */
        rawData.pressure = ((int16_t)(data[1]) << 8) | data[0];
        pressureInPascals = (1.0 * rawData.pressure) / conversionFactor;

        rawData.temperature = (int8_t)(data[2]);
        tempInDegrees = rawData.temperature;

        PRINTF("\r\n Pressure    = %0.3f Pa", pressureInPascals);
        PRINTF("\r\n Temperature = %d degC\r\n", tempInDegrees);
    }
}
