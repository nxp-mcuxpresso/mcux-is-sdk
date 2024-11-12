/*
 * Copyright (c) 2015-2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*! @file  fxls8952_freefall_interrupt.c
*   @brief This is the fxls8952 sensor driver demo file for freefall+interrupt Mode.
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
#include "Driver_gpio.h"
#include "Driver_gpio_sdk.h"
#include "Driver_gpio_sdk_irq.h"
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
#define SDCD_UTHS_MSB 0x0  /* Upper Threshold MSB value. */

/* GPIO Configuration Selections for Interrupt Mode.
 * These PIN settings are for FRDM-K64F and STBC-AGM01 Boards only. */
#define INT1_8952 GPIO_PIN_ID(PortB, 9)
#define RGB_LED GPIO_PIN_ID(PortB, 22)
/*! @brief Create the GPIO pin handle and ID for the Sensor Interrupt. */
MAKE_GPIO_HANDLE(PortB, GPIOB, PORTB, 9, PORTB_IRQn, kCLOCK_PortB, PORTB_NUM)
/*! @brief Create the GPIO pin handle and ID for LED (near SW3 Switch) on FRDM-K64F. */
MAKE_GPIO_HANDLE(PortB, GPIOB, PORTB, 22, PORTB_IRQn, kCLOCK_PortB, PORTB_NUM)

//-----------------------------------------------------------------------
// Constants
//-----------------------------------------------------------------------
/*! @brief Register settings for free-fall detection. */
const registerwritelist_t cFxls8952ConfigFreeFall[] = {
    /* Set Wake Mode ODR Rate as 100Hz. */
    {FXLS8952_SENS_CONFIG3, FXLS8952_SENS_CONFIG3_WAKE_ODR_100HZ, FXLS8952_SENS_CONFIG3_WAKE_ODR_MASK},
    /* Enable X,Y,Z Axis functions. */
    {FXLS8952_SDCD_CONFIG1,
     FXLS8952_SDCD_CONFIG1_X_WT_EN_EN | FXLS8952_SDCD_CONFIG1_Y_WT_EN_EN | FXLS8952_SDCD_CONFIG1_Z_WT_EN_EN,
     FXLS8952_SDCD_CONFIG1_X_WT_EN_MASK | FXLS8952_SDCD_CONFIG1_Y_WT_EN_MASK | FXLS8952_SDCD_CONFIG1_Z_WT_EN_MASK},
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
    /* Enable SDCD Within Thresholds Event Interrupt. */
    {FXLS8952_INT_EN, FXLS8952_INT_EN_SDCD_WT_EN_EN, FXLS8952_INT_EN_SDCD_WT_EN_MASK},
    __END_WRITE_DATA__};

//-----------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------
volatile uint8_t gFxls8952DataReady;

//-----------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------
/*! -----------------------------------------------------------------------
 *  @brief       This is the Sensor Data Ready ISR implementation.
 *  @details     This function sets the flag which indicates if a new sample(s) is available for reading.
 *  @param[in]   pUserData This is a void pointer to the instance of the user specific data structure for the ISR.
 *  @return      void  There is no return value.
 *  @constraints None
 *  @reeentrant  Yes
 *  -----------------------------------------------------------------------*/
void fxls8952_int_data_ready_callback(void *pUserData)
{ /*! @brief Set flag to indicate Sensor has signalled data ready. */
    gFxls8952DataReady = true;
}

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
    fxls8952_i2c_sensorhandle_t fxls8952Driver;
    ARM_DRIVER_I2C *pI2cDriver = &Driver_I2C1_KSDK2_NonBlocking;
    GENERIC_DRIVER_GPIO *pGpioDriver = &Driver_GPIO_KSDK;

    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();

    /* Here INT1_8952 and RGB_LED are the GPIO PIN IDs.
     * These are generated using the same Port ID(PortB for both pins) and Pin Nos(9 and 22 for the pins) used for
     * creating the GPIO handles above. */
    pGpioDriver->pin_init(INT1_8952, GPIO_DIRECTION_IN, NULL, &fxls8952_int_data_ready_callback, NULL);
    pGpioDriver->pin_init(RGB_LED, GPIO_DIRECTION_OUT, NULL, NULL, NULL);

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
    {
        /* In ISR Mode we do not need to check Event Register.
         * The receipt of interrupt will indicate event detection. */
        if (true == gFxls8952DataReady)
        { /*! Clear the data ready flag, it will be set again by the ISR. */
            gFxls8952DataReady = false;
            pGpioDriver->toggle_pin(RGB_LED);
        }
        else
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
