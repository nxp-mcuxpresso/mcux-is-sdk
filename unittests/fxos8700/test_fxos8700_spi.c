/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*! File: test_fxas21002_spi.c
* @brief The \b test_fxas21002_spi.c file contains unittest for fxos8700 SPI sensor
*        functional interfaces.
*/

/* Test Scope of the FXOS8700 unit tests:
*           (1) To test FXOS8700 driver functional interfaces, sensor i/o
*              interfaces and register i/o interfaces.
*           (2) To test negative scenarios by providing bad configurations
*               or inputs and check for appropriate sensor state and error
*               code retured.
*           (3) To make sure that the code coverage for FXOS8700 driver
*               functional interfaces, sensor i/o interfaces and register i/o
*               interfaces is > 85%.
*/

/* CMSIS Includes */
#include "Driver_SPI.h"

/*  SDK Includes */
#include "board.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_debug_console.h"

/* ISSDK Includes */
#include "issdk_hal.h"
#include "gpio_driver.h"
#include "fxos8700_drv.h"

/* Unity Includes */
#include "unity.h"

/*******************************************************************************
 * Macro Definitions
 ******************************************************************************/
/*! @def    FIFO_SIZE
*  @brief  The watermark value configured for FXOS8700 FIFO Buffer.
*/
#define FIFO_SIZE 16 // Must be between 1 - 32

/* Reference Test Values */
/*---------------------------------------------------------------------------*/
/*! @def    CTRLREG1_TESTVAL
 *  @brief  Test FXOS8700 CTRL_REG1 Value: mode = ACTIVE and odr = 12.5Hz.
 */
#define CTRLREG1_TESTVAL 0x29

#define CTRLREG3_TESTVAL 0x08
/*! @def    FSETUP_TESTVAL
 *  @brief  Test FXOS8700 F_SETUP Value: FIFO mode = STOP,
 *          watermark size = FIFO_SIZE.
 */
#define FSETUP_TESTVAL 0x90

#define FXOS8700_WHO_AM_I_PROD_VALUE_SPI_FIX (0xC9)

/*******************************************************************************
 * Constants
 ******************************************************************************/

/*! Prepare the register write list to configure FXOS8700 in FIFO mode. */
const registerwritelist_t fxos8700_Config_with_Fifo[] = {
    {FXOS8700_CTRL_REG1, FXOS8700_CTRL_REG1_DR_SINGLE_12P5_HZ, 0}, /*! Configure the fxos8700 to 12.5Hz sampling rate */
    {FXOS8700_F_SETUP, FXOS8700_F_SETUP_F_MODE_FIFO_STOP_OVF | (16 << FXOS8700_F_SETUP_F_WMRK_SHIFT),
     0}, /*! Prepare the register write list to configure FXOS8700 in FIFO mode. */
    __END_WRITE_DATA__};

/*! Prepare the register read list to read FXOS8700 WHOAMI value. */
const registerreadlist_t fxos8700_WHO_AM_I[] = {{.readFrom = FXOS8700_WHO_AM_I, .numBytes = 1}, __END_READ_DATA__};

/*******************************************************************************
 * Reference Test Registers List
 ******************************************************************************/
/*! Read the FXOS8700_CTRL_REG1 value. */
const registerreadlist_t fxos8700_CTRL_REG1[] = {{.readFrom = FXOS8700_CTRL_REG1, .numBytes = 1}, __END_READ_DATA__};

/*! Read the FXOS8700_F_SETUP value. */
const registerreadlist_t fxos8700_F_SETUP[] = {{.readFrom = FXOS8700_F_SETUP, .numBytes = 1}, __END_READ_DATA__};

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
static int test_counter = 0;
fxos8700_spi_sensorhandle_t gFxos8700Driver;
ARM_DRIVER_SPI *pSPIdriver = &SPI_S_DRIVER;

/*******************************************************************************
 * Code
 ******************************************************************************/
/*! SetUp Required for test framework. */
void setUp(void)
{
    int32_t status;

    /*! Initialize the MCU hardware */
    BOARD_InitPins();
    BOARD_BootClockRUN();
    /*! Initialize the debug console. */
    BOARD_InitDebugConsole();

    /*! Initialize the SPI driver. */
    status = pSPIdriver->Initialize(SPI_S_SIGNAL_EVENT);
    if (ARM_DRIVER_OK != status)
    {
        PRINTF("\r\n SPI Initialization Failed\r\n");
        return;
    }

    /*! Set the SPI Power mode. */
    status = pSPIdriver->PowerControl(ARM_POWER_FULL);
    if (ARM_DRIVER_OK != status)
    {
        PRINTF("\r\n SPI Power Mode setting Failed\r\n");
        return;
    }

    /*! Set the SPI Slave speed. */
    status = pSPIdriver->Control(ARM_SPI_MODE_MASTER | ARM_SPI_CPOL0_CPHA0, SPI_S_BAUDRATE);
    if (ARM_DRIVER_OK != status)
    {
        PRINTF("\r\n SPI Control Mode setting Failed\r\n");
        return;
    }
}

/*! @brief       FXOS8700 Sensor Driver Negative Test 1 Function.
 *  @details     Test function to negatively test FXOS8700 sensor driver
 *               functional interfaces by applying corrupted SPI driver metadata.
 *               Checks:
 *               return status of the sensor driver functions.
 *               sensor handle state,
 */
void test_fxos8700_negative_1(void)
{
    int32_t status;
    uint8_t whoAmI;
    ARM_DRIVER_SPI *corruptedSPIdriver = NULL;

    PRINTF(" \r\n \r\n");
    PRINTF("\r\n---------------------------------------------------------\r\n");
    test_counter++;
    PRINTF("Test %d: FXOS8700 Sensor Driver Negative Test 1\r\n", test_counter);

    // Initialize the FXOS8700 sensor driver.
    status = FXOS8700_SPI_Initialize(&gFxos8700Driver, corruptedSPIdriver, SPI_S_DEVICE_INDEX, &FXOS8700_CS,
                                     FXOS8700_WHO_AM_I_PROD_VALUE);

    PRINTF("\r\nStart performing init parameters check\r\n");
    // CHECK - Check Sensor Driver Init Success/Failure
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_INVALID_PARAM, status);

    // CHECK - Check Sensor handle
    TEST_ASSERT_EQUAL_INT(false, gFxos8700Driver.isInitialized);

    // Reading sensor data with bad read register list
    status = FXOS8700_SPI_ReadData(&gFxos8700Driver, fxos8700_WHO_AM_I, &whoAmI);

    // CHECK - Check Sensor Read Status
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_INIT, status);

    // Configure the FXOS8700 sensor driver with FIFO mode.
    status = FXOS8700_SPI_Configure(&gFxos8700Driver, fxos8700_Config_with_Fifo);

    PRINTF("\r\nStart performing sensor state parameters check\r\n");
    // CHECK - Check Sensor Driver Init Success/Failure
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_INIT, status);

    PRINTF("\r\n \r\n");
}

/*! @brief       FXOS8700 Sensor Driver Negative Test 1 Function.
 *  @details     Test function to negatively test FXOS8700 sensor driver
 *               functional interfaces by applying bad SPI address.
 *               Checks:
 *               return status of the sensor driver functions.
 *               sensor handle state,
 */

void test_fxos8700_negative_2(void)
{
    int32_t status;
    void *badSlaveHandle = NULL;

    PRINTF(" \r\n \r\n");
    PRINTF("\r\n---------------------------------------------------------\r\n");
    test_counter++;
    PRINTF("Test %d: FXOS8700 Sensor Driver Init Negative Test 2\r\n", test_counter);

    // Initialize the FXOS8700 sensor driver.
    status = FXOS8700_SPI_Initialize(&gFxos8700Driver, &SPI_S_DRIVER, SPI_S_DEVICE_INDEX, badSlaveHandle,
                                     FXOS8700_WHO_AM_I_PROD_VALUE);

    // CHECK - Check Sensor Driver Init Success/Failure
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_INVALID_PARAM, status);

    // CHECK - Check Sensor handle
    TEST_ASSERT_EQUAL_INT(false, gFxos8700Driver.isInitialized);

    // Configure the FXOS8700 sensor driver with FIFO mode.
    status = FXOS8700_SPI_Configure(&gFxos8700Driver, fxos8700_Config_with_Fifo);

    PRINTF("\r\nStart performing sensor state parameters check\r\n");
    // CHECK - Check Sensor Driver Init Success/Failure
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_INIT, status);

    PRINTF("\r\n \r\n");
}

/*! @brief       FXOS8700 Sensor Driver Init Test Function.
 *  @details     Test function to test FXOS8700 sensor driver Init interface.
 *               Checks:
 *               return status of the sensor driver init function,
 *               sensor handle state after successful init,
 *               who_am_i value of the sensor.
 */
void test_fxos8700_init(void)
{
    int32_t status;

    PRINTF("\r\n---------------------------------------------------------\r\n");
    test_counter++;
    PRINTF("\r\nTest %d: FXOS8700 Sensor Driver Init Test\r\n", test_counter);

    /*! Initialize the FXOS8700 sensor driver. */
    status = FXOS8700_SPI_Initialize(&gFxos8700Driver, &SPI_S_DRIVER, SPI_S_DEVICE_INDEX, &FXOS8700_CS,
                                     FXOS8700_WHO_AM_I_PROD_VALUE_SPI_FIX);

    PRINTF("\r\nStart performing init parameters check\r\n");
    // CHECK - Check Sensor Driver Init Success/Failure
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_NONE, status);

    // CHECK - Check Sensor handle
    TEST_ASSERT_EQUAL_INT(true, gFxos8700Driver.isInitialized);

    PRINTF("\r\n \r\n");
}

/*! @brief       FXOS8700 Sensor Driver Negative Test 3 Function.
 *  @details     Test function to negatively test FXOS8700 sensor driver
 *               functional interfaces by applying bad Write command List.
 *               Checks:
 *               return status of the sensor driver functions.
 *               sensor handle state,
 */
void test_fxos8700_negative_3(void)
{
    int32_t status;
    const registerwritelist_t *badRegWriteList = NULL;

    PRINTF(" \r\n \r\n");
    PRINTF("\r\n---------------------------------------------------------\r\n");
    test_counter++;
    PRINTF("Test %d: FXOS8700 Sensor Driver Negative Test 3\r\n", test_counter);

    // Configure the FXOS8700 sensor driver with FIFO mode.
    status = FXOS8700_SPI_Configure(&gFxos8700Driver, badRegWriteList);

    PRINTF("\r\nStart performing sensor state parameters check\r\n");
    // CHECK - Check Sensor Driver Config Success/Failure
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_INVALID_PARAM, status);

    PRINTF("\r\n \r\n");
}

/*! @brief       FXOS8700 Sensor Driver Config Test Function.
 *  @details     Test function to test FXOS8700 sensor driver Config interface.
 *               Checks:
 *               return status of the sensor driver config function,
 *               sensor handle state,
 *               ctrl_reg1 value configured for the sensor,
 *               ctrl_reg2 value configured for the sensor,
 *               ctrl_reg3 value configured for the sensor,
 *               f_setup value configured for the sensor.
 */
void test_fxos8700_config(void)
{
    int32_t status;
    uint8_t ctrlReg1;
    uint8_t fSetup;

    PRINTF(" \n \r\n");
    PRINTF("\r\n---------------------------------------------------------\r\n");
    test_counter++;
    PRINTF("Test %d: FXOS8700 Sensor Driver Config Test\r\n", test_counter);

    // Configure the FXOS8700 sensor driver with FIFO mode.
    status = FXOS8700_SPI_Configure(&gFxos8700Driver, fxos8700_Config_with_Fifo);

    PRINTF("\r\nStart performing FXOS8700 configured parameters check\r\n");
    // CHECK - Check Sensor Driver Config Success/Failure
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_NONE, status);

    // CHECK - Check Sensor handle
    TEST_ASSERT_EQUAL_INT(true, gFxos8700Driver.isInitialized);

    // Read the CTRL_REG1 value and check with reference test value.
    status = FXOS8700_SPI_ReadData(&gFxos8700Driver, fxos8700_CTRL_REG1, &ctrlReg1);

    // CHECK - Check CTRL_REG1 value
    TEST_ASSERT_EQUAL_HEX(CTRLREG1_TESTVAL, ctrlReg1);

    // Read the F_SETUP value and check with reference test value.
    status = FXOS8700_SPI_ReadData(&gFxos8700Driver, fxos8700_F_SETUP, &fSetup);
    // CHECK - Check F_SETUP value
    TEST_ASSERT_EQUAL_HEX(FSETUP_TESTVAL, fSetup);

    PRINTF("\r\n \r\n");
}

/*! @brief       FXOS8700 Sensor Driver Negative Test 4 Function.
 *  @details     Test function to negatively test FXOS8700 sensor driver
 *               functional interfaces by applying bad read command List.
 *               Checks:
 *               return status of the sensor driver functions.
 *               sensor handle state,
 */
void test_fxos8700_negative_4(void)
{
    int32_t status;
    uint8_t whoAmI;
    const registerreadlist_t *badRegReadList = NULL;

    PRINTF(" \r\n \r\n");
    PRINTF("\r\n---------------------------------------------------------\r\n");
    test_counter++;
    PRINTF("Test %d: FXOS8700 Sensor Driver Negative Test 3\r\n", test_counter);

    // Reading sensor data with bad read register list
    status = FXOS8700_SPI_ReadData(&gFxos8700Driver, badRegReadList, &whoAmI);

    // CHECK - Check Sensor Read Status
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_INVALID_PARAM, status);

    PRINTF("\r\n \r\n");
}

/*! @brief       FXOS8700 Sensor Driver deInit Test Function.
 *  @details     Test function to test FXOS8700 sensor driver deInit interface.
 *               Checks:
 *               return status of the sensor driver deInit function,
 *               sensor handle state after successful deInit,
 */
void test_fxos8700_deInit(void)
{
    int32_t status;

    PRINTF(" \r\n \r\n");
    PRINTF("\r\n---------------------------------------------------------\r\n");
    test_counter++;
    PRINTF("Test %d: FXOS8700 Sensor Driver endData Test\r\n", test_counter);

    // End the FXOS8700 sensor driver sampling.
    status = FXOS8700_SPI_Deinit(&gFxos8700Driver);

    PRINTF("\r\nStart performing sensor state parameters check\r\n");
    // CHECK - Check Sensor Driver Init Success/Failure
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_NONE, status);

    // CHECK - Check Sensor handle
    TEST_ASSERT_EQUAL_INT(false, gFxos8700Driver.isInitialized);

    PRINTF("\r\n \r\n");
}

void tearDown(void)
{
    // TearDown Required for test framework.
}
