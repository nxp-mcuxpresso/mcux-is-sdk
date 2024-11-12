/*
 * Copyright (c) 2015-2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*! @file  fxls8952_freefall.c
*   @brief This is the fxls8952 sensor driver demo file for free-fall Mode.
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
/* SDCD free-fall counter register values.
 * These values have been derived form the Application Note AN4070 for MMA8451 (the same is applicable to FXLS8952 too).
 * http://cache.freescale.com/files/sensors/doc/app_note/AN4070.pdf */
#define SDCD_WT_DBCNT 0x0A /* Debounce count value. */
#define SDCD_LTHS_LSB 0x33 /* Lower Threshold LSB value. */
#define SDCD_LTHS_MSB 0x0F /* Lower Threshold MSB value. */
#define SDCD_UTHS_LSB 0xCD /* Upper Threshold LSB value. */
#define SDCD_UTHS_MSB 0x00 /* Upper Threshold MSB value. */

//-----------------------------------------------------------------------
// Constants
//-----------------------------------------------------------------------
/*! @brief Register settings for free-fall detection. */
const registerwritelist_t cFxls8952ConfigFreeFall[] = {
    /* Set Wake Mode ODR Rate as 100Hz. */
    {FXLS8952_SENS_CONFIG3, FXLS8952_SENS_CONFIG3_WAKE_ODR_100HZ, FXLS8952_SENS_CONFIG3_WAKE_ODR_MASK},
    /* Enable Within-Threshold Event-Laching-Enable and X,Y,Z Axis functions. */
    {FXLS8952_SDCD_CONFIG1, FXLS8952_SDCD_CONFIG1_WT_ELE_EN | FXLS8952_SDCD_CONFIG1_X_WT_EN_EN |
                                FXLS8952_SDCD_CONFIG1_Y_WT_EN_EN | FXLS8952_SDCD_CONFIG1_Z_WT_EN_EN,
     FXLS8952_SDCD_CONFIG1_WT_ELE_MASK | FXLS8952_SDCD_CONFIG1_X_WT_EN_MASK | FXLS8952_SDCD_CONFIG1_Y_WT_EN_MASK |
         FXLS8952_SDCD_CONFIG1_Z_WT_EN_MASK},
    /* Enable SDCD function; Set SDCD internal reference values update mode to fixed value; Set Debounce counter to be
       cleared whenever the SDCD within thresholds. */
    {FXLS8952_SDCD_CONFIG2, FXLS8952_SDCD_CONFIG2_SDCD_EN_EN | FXLS8952_SDCD_CONFIG2_REF_UPDM_FIXED_VAL |
                                FXLS8952_SDCD_CONFIG2_WT_DBCM_CLEARED,
     FXLS8952_SDCD_CONFIG2_SDCD_EN_MASK | FXLS8952_SDCD_CONFIG2_REF_UPDM_MASK | FXLS8952_SDCD_CONFIG2_WT_DBCM_MASK},
    /* Set SDCD Debounce counter value. */
    {FXLS8952_SDCD_WT_DBCNT, SDCD_WT_DBCNT, 0},
    /* Set SDCD Lower Threshold LSB value. */
    {FXLS8952_SDCD_LTHS_LSB, SDCD_LTHS_LSB, 0},
    /* Set X,Y,Z Data/Delta values to be within Threshold Event Flags; Set SDCD Lower Threshold MSB value. */
    {FXLS8952_SDCD_LTHS_MSB, FXLS8952_SDCD_LTHS_MSB_X_WT_EF_OUT_RANGE | FXLS8952_SDCD_LTHS_MSB_Y_WT_EF_OUT_RANGE |
                                 FXLS8952_SDCD_LTHS_MSB_Z_WT_EF_OUT_RANGE | SDCD_LTHS_MSB,
     FXLS8952_SDCD_LTHS_MSB_X_WT_EF_MASK | FXLS8952_SDCD_LTHS_MSB_Y_WT_EF_MASK | FXLS8952_SDCD_LTHS_MSB_Z_WT_EF_MASK |
         FXLS8952_SDCD_LTHS_MSB_SDCD_LTHS_MASK},
    /* Set SDCH Upper Threshold LSB value. */
    {FXLS8952_SDCD_UTHS_LSB, SDCD_UTHS_LSB, 0},
    /* Set SDCH Upper Threshold MSB value. */
    {FXLS8952_SDCD_UTHS_MSB, SDCD_UTHS_MSB, FXLS8952_SDCD_UTHS_MSB_SDCD_LTHS_MASK},
    __END_WRITE_DATA__};

/*! @brief Address of Free-Fall Status Register. */
const registerreadlist_t cFxls8952FreeFallEvent[] = {{.readFrom = FXLS8952_SDCD_INT_SRC, .numBytes = 1},
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
 *               finally enters an endless loop to continuously read available events.
 *  @param[in]   void This is no input parameter.
 *  @return      void  There is no return value.
 *  @constraints None
 *  @reeentrant  No
 *  -----------------------------------------------------------------------*/
int main(void)
{
    int32_t status;
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
    FXLS8952_I2C_Configure(&fxls8952Driver, cFxls8952ConfigFreeFall);

    gFxls8952DataReady = false;
    PRINTF("\r\nFXLS8952 now active and detecting free-fall...\r\n");

    /* The Forever loop. */
    for (;;)
    { /*! Check for the FIFO watermark FLAG. */
        status = FXLS8952_I2C_ReadData(&fxls8952Driver, cFxls8952FreeFallEvent, (uint8_t *)&gFxls8952DataReady);
        if (ARM_DRIVER_OK != status)
        { /* Read did not work, so loop. */
            PRINTF("\r\nRead Failed.\r\n");
            continue;
        }
        if (0 == (gFxls8952DataReady & FXLS8952_SDCD_INT_SRC_WT_EA_MASK))
        { /* Loop, if new sample is not available. */
            continue;
        }

        /*! Display that a free-fall event has been detected. */
        PRINTF("\r\nDetected free-fall !!!\r\n");
    }
}
////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
