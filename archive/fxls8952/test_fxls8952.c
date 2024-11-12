/*
 * Copyright (c) 2015 - 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*! @file  test_fxls8952.c
 *  @brief This is the unit test file for fxls8952 sensor driver.
 *         The scope of the unit tests is:
 *             1) To verify functionality of each of the driver APIs.
 *             2) Ensure maximun code coverage at the end of unit test runs.
 *             3) To verify handling of negative scenarios by returning appropriate return codes.
 */

//-----------------------------------------------------------------------
// SDK Includes
//-----------------------------------------------------------------------
#include "board.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_debug_console.h"

//-----------------------------------------------------------------------
// ISSDK Includes
//-----------------------------------------------------------------------
#include "Driver_I2C_SDK2.h"
#include "fxls8952_drv.h"
#include "unity.h"

//-----------------------------------------------------------------------
// Macros
//-----------------------------------------------------------------------
/* Configuration Values */
#define FIFO_WMRK_SIZE 16
#define FXLS8952_DATA_SIZE 6

/* Test Values */
// Test FXLS8952 SENS_CONFIG3 Value: ODR = 100Hz.
#define SENS_CONFIG3_TESTVAL FXLS8952_SENS_CONFIG3_WAKE_ODR_100HZ
// Test FXLS8952 BUF_CONFIG1 Value: FIFO mode = STOP.
#define BUF_CONFIG1_TESTVAL FXLS8952_BUF_CONFIG1_BUF_MODE_STOP_MODE
// Test FXLS8952 BUF_CONFIG2 Value: Watermark Size = FIFO_WMRK_SIZE.
#define BUF_CONFIG2_TESTVAL FIFO_WMRK_SIZE
// Test FXLS8952 INT_EN Value: INTERRUPT = Enabled.
#define INT_EN_TESTVAL FXLS8952_INT_EN_BUF_EN_EN

//-----------------------------------------------------------------------
// Constants
//-----------------------------------------------------------------------
/* Register settings for FIFO (buffered) mode. */
const registerwritelist_t cFxls8952ConfigFIFO[] = {
    /* Set Wake Mode ODR Rate as 100Hz. */
    {FXLS8952_SENS_CONFIG3, FXLS8952_SENS_CONFIG3_WAKE_ODR_100HZ, FXLS8952_SENS_CONFIG3_WAKE_ODR_MASK},
    /* Set Buffering Mode as Stop-when-Full.  */
    {FXLS8952_BUF_CONFIG1, FXLS8952_BUF_CONFIG1_BUF_MODE_STOP_MODE, FXLS8952_BUF_CONFIG1_BUF_MODE_MASK},
    /* Set FIFO Water Mark. */
    {FXLS8952_BUF_CONFIG2, FIFO_WMRK_SIZE, FXLS8952_BUF_CONFIG2_BUF_CFG_WMRK_MASK},
    /* Enable Interrupts for Buffer Events. */
    {FXLS8952_INT_EN, FXLS8952_INT_EN_BUF_EN_EN, FXLS8952_INT_EN_BUF_EN_MASK},
    __END_WRITE_DATA__};

/* Address of Who am I Register. */
const registerreadlist_t cFxls8952WhoAmICommand[] = {{.readFrom = FXLS8952_WHO_AM_I, .numBytes = 1}, __END_READ_DATA__};

/* Address of SENS_CONFIG3 Register. */
const registerreadlist_t cFxls8952SENS_CONFIG3[] = {{.readFrom = FXLS8952_SENS_CONFIG3, .numBytes = 1},
                                                    __END_READ_DATA__};

/* Address of BUF_CONFIG1 Register. */
const registerreadlist_t cFxls8952BUF_CONFIG1[] = {{.readFrom = FXLS8952_BUF_CONFIG1, .numBytes = 1},
                                                   __END_READ_DATA__};

/* Address of BUF_CONFIG2 Register. */
const registerreadlist_t cFxls8952BUF_CONFIG2[] = {{.readFrom = FXLS8952_BUF_CONFIG2, .numBytes = 1},
                                                   __END_READ_DATA__};

/* Address of INT_EN Register. */
const registerreadlist_t cFxls8952INT_EN[] = {{.readFrom = FXLS8952_INT_EN, .numBytes = 1}, __END_READ_DATA__};

//-----------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------
static int gTestCounter = 0;
fxls8952_i2c_sensorhandle_t gFxls8952Driver;
ARM_DRIVER_I2C *pI2cDriver = &Driver_I2C1_KSDK2_NonBlocking;

//-----------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------
/* Dummy I2C Callback function. */
void fxls8952_i2c_callback(uint32_t event)
{
}

/*! -----------------------------------------------------------------------
 *  @brief       This is the setup required for test framework.
 *  @details     This function initializes the hardware and setups the framework for test execution.
 *  @param[in]   void This is no input parameter.
 *  @return      void  There is no return value.
 *  @constraints This should be called before test functions are invoked.
 *  @reeentrant  No
 *  -----------------------------------------------------------------------*/
void setUp(void)
{
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();

    /*! Initialize the I2C driver. */
    pI2cDriver->Initialize(fxls8952_i2c_callback);

    /*! Set the I2C bus speed. */
    pI2cDriver->Control(ARM_I2C_BUS_SPEED, ARM_I2C_BUS_SPEED_FAST);
}

/*! -----------------------------------------------------------------------
 *  @brief       This is the FXLS8952 Sensor Driver Negative 1 Test Function.
 *  @details     Checks:
 *                  Return status of the sensor driver functions, when applied
 *                  Corrupted I2C driver metadata
 *  @param[in]   void This is no input parameter.
 *  @return      void  There is no return value.
 *  @constraints This should be the first testfunction to be executed after setUp().
 *  @reeentrant  No
 *  -----------------------------------------------------------------------*/
void test_fxls8952_negative_1(void)
{
    uint8_t whoAmI;
    int32_t status;
    ARM_DRIVER_I2C *pCorruptedI2Cdriver = NULL;

    PRINTF("\r\n \r\n");
    PRINTF("\r\n---------------------------------------------------------\r\n");
    gTestCounter++;
    PRINTF("Test %d: FXLS8952 Sensor Driver Negative 1 Test.\r\n", gTestCounter);

    // Initialize the FXLS8952 sensor driver.
    status =
        FXLS8952_I2C_Initialize(&gFxls8952Driver, pCorruptedI2Cdriver, FXLS8952_DEVICE_ADDRESS, FXLS8952_WHOAMI_VALUE);

    PRINTF("\r\nStart performing init parameters check\r\n");
    // CHECK - Check Sensor Driver Init Success/Failure
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_INVALID_PARAM, status);

    // CHECK - Check Sensor handle
    TEST_ASSERT_EQUAL_INT(false, gFxls8952Driver.isInitialized);

    // Reading sensor data with bad read register list
    status = FXLS8952_I2C_ReadData(&gFxls8952Driver, cFxls8952WhoAmICommand, &whoAmI);

    // CHECK - Check Sensor Read Status
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_INIT, status);

    // Configure the FXLS8952 sensor driver with FIFO mode.
    status = FXLS8952_I2C_Configure(&gFxls8952Driver, cFxls8952ConfigFIFO);

    PRINTF("\r\nStart performing sensor state parameters check\r\n");
    // CHECK - Check Sensor Driver Init Success/Failure
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_INIT, status);

    PRINTF("\r\n \r\n");
}

/*! -----------------------------------------------------------------------
 *  @brief       This is the FXLS8952 Sensor Driver Negative 2 Test Function.
 *  @details     Checks:
 *                  Return status of the sensor driver functions, when applied
 *                  Bad I2C address
 *  @param[in]   void This is no input parameter.
 *  @return      void  There is no return value.
 *  @constraints This should be the second testfunction to be executed after test_fxls8952_negative_1().
 *  @reeentrant  No
 *  -----------------------------------------------------------------------*/
void test_fxls8952_negative_2(void)
{
    int32_t status;
    uint16_t badSlaveaddr = 0x21;

    PRINTF("\r\n \r\n");
    PRINTF("\r\n---------------------------------------------------------\r\n");
    gTestCounter++;
    PRINTF("Test %d: FXLS8952 Sensor Driver Init Negative 2 Test.\r\n", gTestCounter);

    // Initialize the FXLS8952 sensor driver.
    status = FXLS8952_I2C_Initialize(&gFxls8952Driver, pI2cDriver, badSlaveaddr, FXLS8952_WHOAMI_VALUE);

    // CHECK - Check Sensor Driver Init Success/Failure
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_INIT, status);

    // CHECK - Check Sensor handle
    TEST_ASSERT_EQUAL_INT(false, gFxls8952Driver.isInitialized);

    /* Configure device to verify Register_Write failure.*/
    status = FXLS8952_I2C_Configure(&gFxls8952Driver, cFxls8952ConfigFIFO);

    // CHECK - Check Sensor read status.
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_INIT, status);

    PRINTF("\r\n \r\n");
}

/*! -----------------------------------------------------------------------
 *  @brief       This is the FXLS8952 Sensor Driver Init Test Function.
 *  @details     Checks:
 *                  Return status of the sensor driver init function.
 *                  Who_Am_I of the sensor.
 *  @param[in]   void This is no input parameter.
 *  @return      void  There is no return value.
 *  @constraints This should be the third testfunction to be executed after test_fxls8952_negative_2().
 *  @reeentrant  No
 *  -----------------------------------------------------------------------*/
void test_fxls8952_init(void)
{
    int32_t status;

    PRINTF("\r\n---------------------------------------------------------\r\n");
    gTestCounter++;
    PRINTF("\r\nTest %d: FXLS8952 Sensor Driver Init Test\r\n", gTestCounter);

    // Initialize FXLS8952 sensor driver.
    status = FXLS8952_I2C_Initialize(&gFxls8952Driver, pI2cDriver, FXLS8952_DEVICE_ADDRESS, FXLS8952_WHOAMI_VALUE);

    PRINTF("\r\nStart performing init parameters check\r\n");
    // CHECK - Check Sensor Driver Init Success/Failure
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_NONE, status);

    // CHECK - Check Sensor State
    TEST_ASSERT_EQUAL_INT(true, gFxls8952Driver.isInitialized);

    PRINTF("\r\n \r\n");
}

/*! -----------------------------------------------------------------------
 *  @brief       This is the FXLS8952 Sensor Driver Negative 3 Test Function.
 *  @details     Checks:
 *                  Return status of the sensor driver functions, when applied
 *                  Bad Write command List
 *  @param[in]   void This is no input parameter.
 *  @return      void  There is no return value.
 *  @constraints This should be the fourth testfunction to be executed after test_fxls8952_init().
 *  @reeentrant  No
 *  -----------------------------------------------------------------------*/
void test_fxls8952_negative_3(void)
{
    int32_t status;
    const registerwritelist_t *pBadRegWriteList = NULL;

    PRINTF("\r\n \r\n");
    PRINTF("\r\n---------------------------------------------------------\r\n");
    gTestCounter++;
    PRINTF("Test %d: FXLS8952 Sensor Driver Negative 3 Test.\r\n", gTestCounter);

    // Configure the FXLS8952 sensor driver with FIFO mode.
    status = FXLS8952_I2C_Configure(&gFxls8952Driver, pBadRegWriteList);

    PRINTF("\r\nStart performing sensor state parameters check\r\n");
    // CHECK - Check Sensor Driver Config Success/Failure
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_INVALID_PARAM, status);

    PRINTF("\r\n \r\n");
}

/*! -----------------------------------------------------------------------
 *  @brief       This is the FXLS8952 Sensor Driver Config Test Function.
 *  @details     Checks:
 *                  Return status of the sensor driver config function,
 *                  SENS_CONFIG3 value configured for the sensor.
 *                  BUF_CONFIG1  value configured for the sensor.
 *                  BUF_CONFIG2  value configured for the sensor.
 *                  INT_EN       value configured for the sensor.
 *  @param[in]   void This is no input parameter.
 *  @return      void  There is no return value.
 *  @constraints This should be the fifth testfunction to be executed after test_fxls8952_negative_3().
 *  @reeentrant  No
 *  -----------------------------------------------------------------------*/
void test_fxls8952_config(void)
{
    int32_t status;
    uint8_t regBuf;

    PRINTF("\r\n \r\n");
    PRINTF("\r\n---------------------------------------------------------\r\n");
    gTestCounter++;
    PRINTF("Test %d: FXLS8952 Sensor Driver Config Test\r\n", gTestCounter);

    // Configure the FXLS8952 sensor driver with FIFO mode.
    status = FXLS8952_I2C_Configure(&gFxls8952Driver, cFxls8952ConfigFIFO);

    PRINTF("\r\nStart performing FXLS8952 configured parameters check\r\n");
    // CHECK - Check Sensor Driver Config Success/Failure
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_NONE, status);

    // CHECK - Check Sensor handle
    TEST_ASSERT_EQUAL_INT(true, gFxls8952Driver.isInitialized);

    // Read the SENS_CONFIG3 value and check with reference test value.
    FXLS8952_I2C_ReadData(&gFxls8952Driver, cFxls8952SENS_CONFIG3, &regBuf);
    // CHECK - Check SENS_CONFIG3 value
    TEST_ASSERT_EQUAL_HEX(SENS_CONFIG3_TESTVAL, regBuf);

    // Read the BUF_CONFIG1 value and check with reference test value.
    FXLS8952_I2C_ReadData(&gFxls8952Driver, cFxls8952BUF_CONFIG1, &regBuf);
    // CHECK - Check BUF_CONFIG1 value
    TEST_ASSERT_EQUAL_HEX(BUF_CONFIG1_TESTVAL, regBuf);

    // Read the BUF_CONFIG2 value and check with reference test value.
    FXLS8952_I2C_ReadData(&gFxls8952Driver, cFxls8952BUF_CONFIG2, &regBuf);
    // CHECK - Check BUF_CONFIG2 value
    TEST_ASSERT_EQUAL_HEX(BUF_CONFIG2_TESTVAL, regBuf);

    // Read the INT_EN value and check with reference test value.
    FXLS8952_I2C_ReadData(&gFxls8952Driver, cFxls8952INT_EN, &regBuf);
    // CHECK - Check BUF_CONFIG2 value
    TEST_ASSERT_EQUAL_HEX(INT_EN_TESTVAL, regBuf);

    PRINTF("\r\n \r\n");
}

/*! -----------------------------------------------------------------------
 *  @brief       This is the FXLS8952 Sensor Driver Negative 4 Test Function.
 *  @details     Checks:
 *                  Return status of the sensor driver functions, when applied
 *                  Bad Read command List
 *  @param[in]   void This is no input parameter.
 *  @return      void  There is no return value.
 *  @constraints This should be the sixth testfunction to be executed after test_fxls8952_config().
 *  @reeentrant  No
 *  -----------------------------------------------------------------------*/
void test_fxls8952_negative_4(void)
{
    int32_t status;
    uint8_t whoAmI;
    const registerreadlist_t *badRegReadList = NULL;

    PRINTF("\r\n \r\n");
    PRINTF("\r\n---------------------------------------------------------\r\n");
    gTestCounter++;
    PRINTF("Test %d: FXLS8952 Sensor Driver Negative 4 Test\r\n", gTestCounter);

    // Reading sensor data with bad read register list
    status = FXLS8952_I2C_ReadData(&gFxls8952Driver, badRegReadList, &whoAmI);

    // CHECK - Check Sensor Read Status
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_INVALID_PARAM, status);

    PRINTF("\r\n \r\n");
}

/*! -----------------------------------------------------------------------
 *  @brief       This is the FXLS8952 Sensor Driver deInit Test Function.
 *  @details     Checks:
 *                  Return status of the sensor driver deInit function,
 *                  State of the sensor after deInit
 *  @param[in]   void This is no input parameter.
 *  @return      void  There is no return value.
 *  @constraints This should be the seventh testfunction to be executed after test_fxls8952_negative_4().
 *  @reeentrant  No
 *  -----------------------------------------------------------------------*/
void test_fxls8952_deInit(void)
{
    int32_t status;

    PRINTF("\r\n \r\n");
    PRINTF("\r\n---------------------------------------------------------\r\n");
    gTestCounter++;
    PRINTF("Test %d: FXLS8952 Sensor Driver deInit Test\r\n", gTestCounter);

    // End the FXLS8952 sensor driver sampling.
    status = FXLS8952_I2C_DeInit(&gFxls8952Driver);

    PRINTF("\r\nStart performing sensor state parameters check\r\n");
    // CHECK - Check Sensor Driver Init Success/Failure
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_NONE, status);

    // CHECK - Check Sensor State
    TEST_ASSERT_EQUAL_INT(false, gFxls8952Driver.isInitialized);

    PRINTF("\r\n \r\n");
}

/* TearDown Required for test framework. */
void tearDown(void)
{
}

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
