/*
 * Copyright (c) 2015-2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*! @file  fxls8952_fifo_interrupt.c
*   @brief This is the fxls8952 sensor driver demo file for FIFO+Interrupt Mode.
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
/* Application Configurations Selections */
#define FIFO_WMRK_SIZE 16
#define FXLS8952_DATA_SIZE 6

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
/*! @brief Register settings for FIFO (buffered) mode. */
const registerwritelist_t cFxls8952ConfigFIFO[] = {
    /* Set Full-scale range as 4G. */
    {FXLS8952_SENS_CONFIG1, FXLS8952_SENS_CONFIG1_FSR_4G, FXLS8952_SENS_CONFIG1_FSR_MASK},
    /* Set Wake Mode ODR Rate as 100Hz. */
    {FXLS8952_SENS_CONFIG3, FXLS8952_SENS_CONFIG3_WAKE_ODR_100HZ, FXLS8952_SENS_CONFIG3_WAKE_ODR_MASK},
    /* Set Buffering Mode as Stop-when-Full.  */
    {FXLS8952_BUF_CONFIG1, FXLS8952_BUF_CONFIG1_BUF_MODE_STOP_MODE, FXLS8952_BUF_CONFIG1_BUF_MODE_MASK},
    /* Set FIFO Water Mark. */
    {FXLS8952_BUF_CONFIG2, FIFO_WMRK_SIZE, FXLS8952_BUF_CONFIG2_BUF_CFG_WMRK_MASK},
    /* Enable Interrupts for Buffer Events. */
    {FXLS8952_INT_EN, FXLS8952_INT_EN_BUF_EN_EN, FXLS8952_INT_EN_BUF_EN_MASK},
    __END_WRITE_DATA__};

/*! @brief Address of Raw Accel Data in FIFO Mode. */
const registerreadlist_t cFxls8952OutputFIF0[] = {
    {.readFrom = FXLS8952_BUF_X_LSB, .numBytes = FXLS8952_DATA_SIZE * FIFO_WMRK_SIZE}, __END_READ_DATA__};

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
 *               finally enters an endless loop to continuously read available samples.
 *  @param[in]   void This is no input parameter.
 *  @return      void  There is no return value.
 *  @constraints None
 *  @reeentrant  No
 *  -----------------------------------------------------------------------*/
int main(void)
{
    int32_t status;
    uint8_t data[FXLS8952_DATA_SIZE * FIFO_WMRK_SIZE];
    fxls8952_acceldata_t rawData;
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
    FXLS8952_I2C_Configure(&fxls8952Driver, cFxls8952ConfigFIFO);

    gFxls8952DataReady = true;
    PRINTF("\r\nFXLS8952 now active and entering data read loop...\r\n");

    /* The Forever loop (with logic optimized for FXLS8952).
     * This is to enable support for very High ODR Rates of FXLS8952. */
    for (;;)
    {
        /* In ISR Mode we do not need to check Ready.
         * The receipt of interrupt will indicate data is ready. */
        if (true == gFxls8952DataReady)
        { /*! Clear the data ready flag, it will be set again by the ISR. */
            gFxls8952DataReady = false;
            pGpioDriver->toggle_pin(RGB_LED);
        }
        else
        { /* Loop, if new sample is not available. */
            continue;
        }

        /* To reduce effects of processing we process previously received sample first.
         * This logic makes sure no ISR is missed at very High ODR when processing time is of the order of ODR Rate.
         * Due to processing delay, the Interupt for the next ODR may come up immediately after data read. */
        for (uint8_t i = 0; i < FIFO_WMRK_SIZE; i++)
        { /*! Process all samples and convert the raw sensor data. */
            rawData.accel[0] = ((int16_t)data[i * FXLS8952_DATA_SIZE + 1] << 8) | data[i * FXLS8952_DATA_SIZE + 0];
            rawData.accel[1] = ((int16_t)data[i * FXLS8952_DATA_SIZE + 3] << 8) | data[i * FXLS8952_DATA_SIZE + 2];
            rawData.accel[2] = ((int16_t)data[i * FXLS8952_DATA_SIZE + 5] << 8) | data[i * FXLS8952_DATA_SIZE + 4];
        }
        /*! Display to the debug port the last sample.
         *  This is done to reduce the effects of PRINTF which is relatively expensive in terms of CPU time. */
        PRINTF("\r\nX=%5d Y=%5d Z=%5d\r\n", rawData.accel[0], rawData.accel[1], rawData.accel[2]);

        /*! Read new raw sensor data from the FXLS8952. */
        status = FXLS8952_I2C_ReadData(&fxls8952Driver, cFxls8952OutputFIF0, (uint8_t *)data);
        if (ARM_DRIVER_OK != status)
        { /* Read did not work, so loop. */
            PRINTF("\r\nRead Failed.\r\n");
            continue;
        }
    }
}
////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
