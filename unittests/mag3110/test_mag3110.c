/*
 * Copyright (c) 2015 - 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*! @file  test_mag3110.c
 *  @brief This is the unit test file for mag3110 sensor driver.
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
// CMSIS Includes
//-----------------------------------------------------------------------
#include "Driver_I2C.h"

//-----------------------------------------------------------------------
// ISSDK Includes
//-----------------------------------------------------------------------
#include "issdk_hal.h"
#include "mag3110_drv.h"

//-----------------------------------------------------------------------
// Unity Includes
//-----------------------------------------------------------------------
#include "unity.h"

//-----------------------------------------------------------------------
// Macros
//-----------------------------------------------------------------------
/* Configuration Values */
#define MAG3110_DATA_SIZE (6) /* 2 byte X,Y,Z Axis Data each. */

/* Reference Values */
// Test MAG3110 CTRL_REG1   Value: ODR = 2 and OSR = 32 and Mode = Active Mode.
#define CTRL_REG1_TESTVAL (MAG3110_CTRL_REG1_DR_ODR_2 | MAG3110_CTRL_REG1_OS_OSR_32 | MAG3110_CTRL_REG1_AC_ACTIVE)
// Test MAG3110 CTRL_REG2   Value: RESET = DISABLED. (Though we set RESET ENABLED this value always reads '0' as per
// data sheet.)
#define CTRL_REG2_TESTVAL (MAG3110_CTRL_REG2_AUTO_MSRT_EN_DIS)

//-----------------------------------------------------------------------
// Constants
//-----------------------------------------------------------------------
/*! @brief Register settings for Normal (non buffered) mode. */
const registerwritelist_t cMag3110ConfigNormal[] = {
    /* Set Ouput Rate @10HZ (ODR = 2 and OSR = 32). */
    {MAG3110_CTRL_REG1, MAG3110_CTRL_REG1_DR_ODR_2 | MAG3110_CTRL_REG1_OS_OSR_32,
     MAG3110_CTRL_REG1_DR_MASK | MAG3110_CTRL_REG1_OS_MASK},
    /* Set Auto Magnetic Sensor Reset. */
    {MAG3110_CTRL_REG2, MAG3110_CTRL_REG2_AUTO_MSRT_EN_EN, MAG3110_CTRL_REG2_AUTO_MSRT_EN_MASK},
    __END_WRITE_DATA__};

/* Address of Who am I Register. */
const registerreadlist_t cMag3110WhoAmICommand[] = {{.readFrom = MAG3110_WHO_AM_I, .numBytes = 1}, __END_READ_DATA__};

/* Address of CTRL_REG1 Register. */
const registerreadlist_t cMag3110CTRL_REG1[] = {{.readFrom = MAG3110_CTRL_REG1, .numBytes = 1}, __END_READ_DATA__};

/* Address of CTRL_REG2 Register. */
const registerreadlist_t cMag3110CTRL_REG2[] = {{.readFrom = MAG3110_CTRL_REG2, .numBytes = 1}, __END_READ_DATA__};

//-----------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------
static int gTestCounter = 0;
mag3110_i2c_sensorhandle_t gMag3110Driver;
ARM_DRIVER_I2C *I2Cdrv = &I2C_S_DRIVER; // Now using the shield.h value!!!

//-----------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------
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
    int32_t status;

    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();

    /*! Initialize the I2C driver. */
    status = I2Cdrv->Initialize(I2C_S_SIGNAL_EVENT);
    if (ARM_DRIVER_OK != status)
    {
        PRINTF("\r\n I2C Initialization Failed\r\n");
        return;
    }

    /*! Set the I2C Power mode. */
    status = I2Cdrv->PowerControl(ARM_POWER_FULL);
    if (ARM_DRIVER_OK != status)
    {
        PRINTF("\r\n I2C Power Mode setting Failed\r\n");
        return;
    }

    /*! Set the I2C bus speed. */
    status = I2Cdrv->Control(ARM_I2C_BUS_SPEED, ARM_I2C_BUS_SPEED_FAST);
    if (ARM_DRIVER_OK != status)
    {
        PRINTF("\r\n I2C Control Mode setting Failed\r\n");
        return;
    }
}

/*! -----------------------------------------------------------------------
 *  @brief       This is the MAG3110 Sensor Driver Negative 1 Test Function.
 *  @details     Checks:
 *                  Return status of the sensor driver functions, when applied
 *                  Corrupted I2C driver metadata
 *  @param[in]   void This is no input parameter.
 *  @return      void  There is no return value.
 *  @constraints This should be the first testfunction to be executed after setUp().
 *  @reeentrant  No
 *  -----------------------------------------------------------------------*/
void test_mag3110_negative_1(void)
{
    uint8_t whoAmI;
    int32_t status;
    ARM_DRIVER_I2C *pCorruptedI2Cdriver = NULL;

    PRINTF("\r\n \r\n");
    PRINTF("\r\n---------------------------------------------------------\r\n");
    gTestCounter++;
    PRINTF("Test %d: MAG3110 Sensor Driver Negative 1 Test.\r\n", gTestCounter);

    // Initialize the MAG3110 sensor driver.
    status = MAG3110_I2C_Initialize(&gMag3110Driver, pCorruptedI2Cdriver, I2C_S_DEVICE_INDEX, MAG3110_I2C_ADDR,
                                    MAG3110_WHOAMI_VALUE);

    PRINTF("\r\nStart performing init parameters check\r\n");
    // CHECK - Check Sensor Driver Init Success/Failure
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_INVALID_PARAM, status);

    // CHECK - Check Sensor handle
    TEST_ASSERT_EQUAL_INT(false, gMag3110Driver.isInitialized);

    // Reading sensor data with bad read register list
    status = MAG3110_I2C_ReadData(&gMag3110Driver, cMag3110WhoAmICommand, &whoAmI);

    // CHECK - Check Sensor Read Status
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_INIT, status);

    // Configure the MAG3110 sensor driver with Normal mode.
    status = MAG3110_I2C_Configure(&gMag3110Driver, cMag3110ConfigNormal);

    PRINTF("\r\nStart performing sensor state parameters check\r\n");
    // CHECK - Check Sensor Driver Init Success/Failure
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_INIT, status);

    PRINTF("\r\n \r\n");
}

/*! -----------------------------------------------------------------------
 *  @brief       This is the MAG3110 Sensor Driver Negative 2 Test Function.
 *  @details     Checks:
 *                  Return status of the sensor driver functions, when applied
 *                  Bad I2C address
 *  @param[in]   void This is no input parameter.
 *  @return      void  There is no return value.
 *  @constraints This should be the second testfunction to be executed after test_mag3110_negative_1().
 *  @reeentrant  No
 *  -----------------------------------------------------------------------*/
void test_mag3110_negative_2(void)
{
    int32_t status;
    uint16_t badSlaveaddr = 0x21;

    PRINTF("\r\n \r\n");
    PRINTF("\r\n---------------------------------------------------------\r\n");
    gTestCounter++;
    PRINTF("Test %d: MAG3110 Sensor Driver Init Negative 2 Test.\r\n", gTestCounter);

    // Initialize the MAG3110 sensor driver.
    status =
        MAG3110_I2C_Initialize(&gMag3110Driver, &I2C_S_DRIVER, I2C_S_DEVICE_INDEX, badSlaveaddr, MAG3110_WHOAMI_VALUE);

    // CHECK - Check Sensor Driver Init Success/Failure
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_INIT, status);

    // CHECK - Check Sensor handle
    TEST_ASSERT_EQUAL_INT(false, gMag3110Driver.isInitialized);

    /* Configure device to verify Register_Write failure.*/
    status = MAG3110_I2C_Configure(&gMag3110Driver, cMag3110ConfigNormal);

    // CHECK - Check Sensor read status.
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_INIT, status);

    PRINTF("\r\n \r\n");
}

/*! -----------------------------------------------------------------------
 *  @brief       This is the MAG3110 Sensor Driver Init Test Function.
 *  @details     Checks:
 *                  Return status of the sensor driver init function.
 *                  Who_Am_I of the sensor.
 *  @param[in]   void This is no input parameter.
 *  @return      void  There is no return value.
 *  @constraints This should be the third testfunction to be executed after test_mag3110_negative_2().
 *  @reeentrant  No
 *  -----------------------------------------------------------------------*/
void test_mag3110_init(void)
{
    int32_t status;

    PRINTF("\r\n---------------------------------------------------------\r\n");
    gTestCounter++;
    PRINTF("\r\nTest %d: MAG3110 Sensor Driver Init Test\r\n", gTestCounter);

    // Initialize MAG3110 sensor driver.
    status = MAG3110_I2C_Initialize(&gMag3110Driver, &I2C_S_DRIVER, I2C_S_DEVICE_INDEX, MAG3110_I2C_ADDR,
                                    MAG3110_WHOAMI_VALUE);

    PRINTF("\r\nStart performing init parameters check\r\n");
    // CHECK - Check Sensor Driver Init Success/Failure
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_NONE, status);

    // CHECK - Check Sensor State
    TEST_ASSERT_EQUAL_INT(true, gMag3110Driver.isInitialized);

    PRINTF("\r\n \r\n");
}

/*! -----------------------------------------------------------------------
 *  @brief       This is the MAG3110 Sensor Driver Negative 3 Test Function.
 *  @details     Checks:
 *                  Return status of the sensor driver functions, when applied
 *                  Bad Write command List
 *  @param[in]   void This is no input parameter.
 *  @return      void  There is no return value.
 *  @constraints This should be the fourth testfunction to be executed after test_mag3110_init().
 *  @reeentrant  No
 *  -----------------------------------------------------------------------*/
void test_mag3110_negative_3(void)
{
    int32_t status;
    const registerwritelist_t *pBadRegWriteList = NULL;

    PRINTF("\r\n \r\n");
    PRINTF("\r\n---------------------------------------------------------\r\n");
    gTestCounter++;
    PRINTF("Test %d: MAG3110 Sensor Driver Negative 3 Test.\r\n", gTestCounter);

    // Configure the MAG3110 sensor driver with Normal mode.
    status = MAG3110_I2C_Configure(&gMag3110Driver, pBadRegWriteList);

    PRINTF("\r\nStart performing sensor state parameters check\r\n");
    // CHECK - Check Sensor Driver Config Success/Failure
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_INVALID_PARAM, status);

    PRINTF("\r\n \r\n");
}

/*! -----------------------------------------------------------------------
 *  @brief       This is the MAG3110 Sensor Driver Config Test Function.
 *  @details     Checks:
 *                  Return status of the sensor driver config function,
 *                  CTRL_REG1    value configured for the sensor.
 *                  CTRL_REG2    value configured for the sensor.
 *  @param[in]   void This is no input parameter.
 *  @return      void  There is no return value.
 *  @constraints This should be the fifth testfunction to be executed after test_mag3110_negative_3().
 *  @reeentrant  No
 *  -----------------------------------------------------------------------*/
void test_mag3110_config(void)
{
    int32_t status;
    uint8_t regBuf;

    PRINTF("\r\n \r\n");
    PRINTF("\r\n---------------------------------------------------------\r\n");
    gTestCounter++;
    PRINTF("Test %d: MAG3110 Sensor Driver Config Test\r\n", gTestCounter);

    // Configure the MAG3110 sensor driver with Normal mode.
    status = MAG3110_I2C_Configure(&gMag3110Driver, cMag3110ConfigNormal);

    PRINTF("\r\nStart performing MAG3110 configured parameters check\r\n");
    // CHECK - Check Sensor Driver Config Success/Failure
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_NONE, status);

    // CHECK - Check Sensor handle
    TEST_ASSERT_EQUAL_INT(true, gMag3110Driver.isInitialized);

    // Read the CTRL_REG1 value and check with reference test value.
    MAG3110_I2C_ReadData(&gMag3110Driver, cMag3110CTRL_REG1, &regBuf);
    // CHECK - Check CTRL_REG1 value
    TEST_ASSERT_EQUAL_HEX(CTRL_REG1_TESTVAL, regBuf);

    // Read the CTRL_REG2 value and check with reference test value.
    MAG3110_I2C_ReadData(&gMag3110Driver, cMag3110CTRL_REG2, &regBuf);
    // CHECK - Check CTRL_REG2 value
    TEST_ASSERT_EQUAL_HEX(CTRL_REG2_TESTVAL, regBuf);

    PRINTF("\r\n \r\n");
}

/*! -----------------------------------------------------------------------
 *  @brief       This is the MAG3110 Sensor Driver Negative 4 Test Function.
 *  @details     Checks:
 *                  Return status of the sensor driver functions, when applied
 *                  Bad Read command List
 *  @param[in]   void This is no input parameter.
 *  @return      void  There is no return value.
 *  @constraints This should be the sixth testfunction to be executed after test_mag3110_config().
 *  @reeentrant  No
 *  -----------------------------------------------------------------------*/
void test_mag3110_negative_4(void)
{
    int32_t status;
    uint8_t whoAmI;
    const registerreadlist_t *badRegReadList = NULL;

    PRINTF("\r\n \r\n");
    PRINTF("\r\n---------------------------------------------------------\r\n");
    gTestCounter++;
    PRINTF("Test %d: MAG3110 Sensor Driver Negative 4 Test\r\n", gTestCounter);

    // Reading sensor data with bad read register list
    status = MAG3110_I2C_ReadData(&gMag3110Driver, badRegReadList, &whoAmI);

    // CHECK - Check Sensor Read Status
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_INVALID_PARAM, status);

    PRINTF("\r\n \r\n");
}

/*! -----------------------------------------------------------------------
 *  @brief       This is the MAG3110 Sensor Driver deInit Test Function.
 *  @details     Checks:
 *                  Return status of the sensor driver deInit function,
 *                  State of the sensor after deInit
 *  @param[in]   void This is no input parameter.
 *  @return      void  There is no return value.
 *  @constraints This should be the seventh testfunction to be executed after test_mag3110_negative_4().
 *  @reeentrant  No
 *  -----------------------------------------------------------------------*/
void test_mag3110_deInit(void)
{
    int32_t status;

    PRINTF("\r\n \r\n");
    PRINTF("\r\n---------------------------------------------------------\r\n");
    gTestCounter++;
    PRINTF("Test %d: MAG3110 Sensor Driver deInit Test\r\n", gTestCounter);

    // End the MAG3110 sensor driver sampling.
    status = MAG3110_I2C_DeInit(&gMag3110Driver);

    PRINTF("\r\nStart performing sensor state parameters check\r\n");
    // CHECK - Check Sensor Driver De-Init Success/Failure
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_NONE, status);

    // CHECK - Check Sensor State
    TEST_ASSERT_EQUAL_INT(false, gMag3110Driver.isInitialized);

    PRINTF("\r\n \r\n");
}

/* TearDown Required for test framework. */
void tearDown(void)
{
}

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
