/*
 * Copyright (c) 2015-2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*! @file  fxls8952_normal.c
*   @brief This is the fxls8952 sensor driver demo file for Normal Mode.
*/

//-----------------------------------------------------------------------
// SDK Includes
//-----------------------------------------------------------------------
#include "board.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_debug_console.h"
#include "fsl_i2c.h"

//-----------------------------------------------------------------------
// CMSIS Includes
//-----------------------------------------------------------------------
#include "Driver_I2C.h"
#include "Driver_I2C_SDK2.h"

//-----------------------------------------------------------------------
// ISSDK Includes
//-----------------------------------------------------------------------
#include "fxls8952_drv.h"

//-----------------------------------------------------------------------
// Macros
//-----------------------------------------------------------------------
#define FXLS8952_DATA_SIZE 6

//-----------------------------------------------------------------------
// Constants
//-----------------------------------------------------------------------
/*! @brief Register settings for Normal (non buffered) mode. */
const registerwritelist_t cFxls8952ConfigNormal[] = {
    /* Set Full-scale range as 4G. */
    {FXLS8952_SENS_CONFIG1, FXLS8952_SENS_CONFIG1_FSR_4G, FXLS8952_SENS_CONFIG1_FSR_MASK},
    /* Set Wake Mode ODR Rate as 6.25Hz. */
    {FXLS8952_SENS_CONFIG3, FXLS8952_SENS_CONFIG3_WAKE_ODR_6_25HZ, FXLS8952_SENS_CONFIG3_WAKE_ODR_MASK},
    __END_WRITE_DATA__};

/*! @brief Address of DATA Ready Status Register. */
const registerreadlist_t cFxls8952DRDYEvent[] = {{.readFrom = FXLS8952_INT_STATUS, .numBytes = 1}, __END_READ_DATA__};

/*! @brief Address of Raw Accel Data in Normal Mode. */
const registerreadlist_t cFxls8952OutputNormal[] = {{.readFrom = FXLS8952_OUT_X_LSB, .numBytes = FXLS8952_DATA_SIZE},
                                                    __END_READ_DATA__};

//-----------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------
volatile uint8_t gFxls8952DataReady;

//-----------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------
/*! -----------------------------------------------------------------------
 *  @brief       This is the The main function implementation.
 *  @details     This function invokes board initializes routines, then then brings up the sensor and
 *               finally enters an endless loop to continuously read available samples.
 *  @param[in]   void This is no input parameter.
 *  @return      void  There is no return value.
 *  @constraints None
 *  @reeentrant  No
 *  -----------------------------------------------------------------------*/
int main(void)
{
    int32_t status;
    uint8_t data[FXLS8952_DATA_SIZE];
    fxls8952_acceldata_t rawData;
    fxls8952_i2c_sensorhandle_t fxls8952Driver;
    ARM_DRIVER_I2C *pI2cDriver = &Driver_I2C1_KSDK2_NonBlocking;

    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();

    /*! Initialize the I2C driver. */
    pI2cDriver->Initialize(NULL);

    /*! Set the I2C bus speed. */
    pI2cDriver->Control(ARM_I2C_BUS_SPEED, ARM_I2C_BUS_SPEED_FAST);
    PRINTF("\r\nSet the I2C Bus Speed to Fast (400Kbps)\r\n");

    /*! Initialize FXLS8952 sensor driver. */
    FXLS8952_I2C_Initialize(&fxls8952Driver, pI2cDriver, FXLS8952_DEVICE_ADDRESS, FXLS8952_WHOAMI_VALUE);

    /*! Configure the FXLS8952 sensor.
     *  The FXLS8952 sensor driver will put it to Active Mode and sampling can be started. */
    FXLS8952_I2C_Configure(&fxls8952Driver, cFxls8952ConfigNormal);

    gFxls8952DataReady = false;
    PRINTF("\r\nFXLS8952 now active and entering data read loop...\r\n");

    /* The Forever loop. */
    for (;;)
    {
        /*! Check for data ready FLAG. */
        status = FXLS8952_I2C_ReadData(&fxls8952Driver, cFxls8952DRDYEvent, (uint8_t *)&gFxls8952DataReady);
        if (0 == (gFxls8952DataReady & FXLS8952_INT_STATUS_DRDY_MASK))
        { /* Loop, if new sample is not available. */
            continue;
        }

        /*! Read new raw sensor data from the FXLS8952. */
        status = FXLS8952_I2C_ReadData(&fxls8952Driver, cFxls8952OutputNormal, (uint8_t *)data);
        if (ARM_DRIVER_OK != status)
        { /* Read did not work, so loop. */
            PRINTF("\r\nRead Failed.\r\n");
            continue;
        }

        /*! Process the sample and convert the raw sensor data. */
        rawData.accel[0] = ((int16_t)data[1] << 8) | data[0];
        rawData.accel[1] = ((int16_t)data[3] << 8) | data[2];
        rawData.accel[2] = ((int16_t)data[5] << 8) | data[4];
        PRINTF("\r\nX=%5d Y=%5d Z=%5d\r\n", rawData.accel[0], rawData.accel[1], rawData.accel[2]);
    }
}
////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
