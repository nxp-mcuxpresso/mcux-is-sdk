/*
 * Copyright (c) 2015 - 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*! @file  test_fxls8962.c
 *  @brief This is the unit test file for fxls8962 sensor driver.
 *         The scope of the unit tests is:
 *             1) To verify functionality of each of the driver APIs.
 *             2) Ensure maximun code coverage at the end of unit test runs.
 *             3) To verify handling of negative scenarios by returning appropriate return codes.
 */

//-----------------------------------------------------------------------
// CMSIS Includes
//-----------------------------------------------------------------------
#include "Driver_I2C.h"

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
#include "issdk_hal.h"
#include "fxls8962_drv.h"
#include "systick_utils.h"

//-----------------------------------------------------------------------
// Unity Includes
//-----------------------------------------------------------------------
#include "unity.h"

//-----------------------------------------------------------------------
// Macros
//-----------------------------------------------------------------------
/* Test Values */
// Test FXLS8962 SENS_CONFIG3 Value: ODR = 100Hz.
#define SENS_CONFIG3_TESTVAL FXLS8962_SENS_CONFIG3_WAKE_ODR_100HZ
// Test FXLS8962 SENS_CONFIG1 Value: FSR = 4G and Mode = ACTIVE.
#define SENS_CONFIG1_TESTVAL FXLS8962_SENS_CONFIG1_FSR_4G | FXLS8962_SENS_CONFIG1_ACTIVE_ACTIVE
// Test FXLS8962 INT_EN Value: INTERRUPT = Enabled.
#define INT_EN_TESTVAL FXLS8962_INT_EN_DRDY_EN_EN

//-----------------------------------------------------------------------
// Constants
//-----------------------------------------------------------------------
/*! @brief Register settings for Interrupt (non buffered) mode. */
const registerwritelist_t cFxls8962ConfigNormal[] = {
    /* Set Full-scale range as 4G. */
    {FXLS8962_SENS_CONFIG1, FXLS8962_SENS_CONFIG1_FSR_4G, FXLS8962_SENS_CONFIG1_FSR_MASK},
    /* Set Wake Mode ODR Rate as 100Hz. */
    {FXLS8962_SENS_CONFIG3, FXLS8962_SENS_CONFIG3_WAKE_ODR_100HZ, FXLS8962_SENS_CONFIG3_WAKE_ODR_MASK},
    /* Enable Interrupts for Data Ready Events. */
    {FXLS8962_INT_EN, FXLS8962_INT_EN_DRDY_EN_EN, FXLS8962_INT_EN_DRDY_EN_MASK},
    __END_WRITE_DATA__};

/* Address of Who am I Register. */
const registerreadlist_t cFxls8962WhoAmICommand[] = {{.readFrom = FXLS8962_WHO_AM_I, .numBytes = 1}, __END_READ_DATA__};

/* Address of SENS_CONFIG3 Register. */
const registerreadlist_t cFxls8962SENS_CONFIG3[] = {{.readFrom = FXLS8962_SENS_CONFIG3, .numBytes = 1},
                                                    __END_READ_DATA__};

/* Address of SENS_CONFIG1 Register. */
const registerreadlist_t cFxls8962SENS_CONFIG1[] = {{.readFrom = FXLS8962_SENS_CONFIG1, .numBytes = 1},
                                                    __END_READ_DATA__};
/* Address of INT_EN Register. */
const registerreadlist_t cFxls8962INT_EN[] = {{.readFrom = FXLS8962_INT_EN, .numBytes = 1}, __END_READ_DATA__};

//-----------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------
static int gTestCounter = 0;
fxls8962_i2c_sensorhandle_t gFxls8962Driver;
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
    BOARD_SystickEnable();
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
 *  @brief       This is the FXLS8962 Sensor Driver Negative 1 Test Function.
 *  @details     Checks:
 *                  Return status of the sensor driver functions, when applied
 *                  Corrupted I2C driver metadata
 *  @param[in]   void This is no input parameter.
 *  @return      void  There is no return value.
 *  @constraints This should be the first testfunction to be executed after setUp().
 *  @reeentrant  No
 *  -----------------------------------------------------------------------*/
void test_fxls8962_negative_1(void)
{
    uint8_t whoAmI;
    int32_t status;
    ARM_DRIVER_I2C *pCorruptedI2Cdriver = NULL;

    PRINTF("\r\n \r\n");
    PRINTF("\r\n---------------------------------------------------------\r\n");
    gTestCounter++;
    PRINTF("Test %d: FXLS8962 Sensor Driver Negative 1 Test.\r\n", gTestCounter);

    // Initialize the FXLS8962 sensor driver.
    status = FXLS8962_I2C_Initialize(&gFxls8962Driver, pCorruptedI2Cdriver, I2C_S_DEVICE_INDEX, FXLS8962_I2C_ADDR,
                                     FXLS8962_WHOAMI_VALUE);

    PRINTF("\r\nStart performing init parameters check\r\n");
    // CHECK - Check Sensor Driver Init Success/Failure
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_INVALID_PARAM, status);

    // CHECK - Check Sensor handle
    TEST_ASSERT_EQUAL_INT(false, gFxls8962Driver.isInitialized);

    // Reading sensor data with bad read register list
    status = FXLS8962_I2C_ReadData(&gFxls8962Driver, cFxls8962WhoAmICommand, &whoAmI);

    // CHECK - Check Sensor Read Status
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_INIT, status);

    // Configure the FXLS8962 sensor driver with FIFO mode.
    status = FXLS8962_I2C_Configure(&gFxls8962Driver, cFxls8962ConfigNormal);

    PRINTF("\r\nStart performing sensor state parameters check\r\n");
    // CHECK - Check Sensor Driver Init Success/Failure
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_INIT, status);

    PRINTF("\r\n \r\n");
}

/*! -----------------------------------------------------------------------
 *  @brief       This is the FXLS8962 Sensor Driver Negative 2 Test Function.
 *  @details     Checks:
 *                  Return status of the sensor driver functions, when applied
 *                  Bad I2C address
 *  @param[in]   void This is no input parameter.
 *  @return      void  There is no return value.
 *  @constraints This should be the second testfunction to be executed after test_fxls8962_negative_1().
 *  @reeentrant  No
 *  -----------------------------------------------------------------------*/
void test_fxls8962_negative_2(void)
{
    int32_t status;
    uint16_t badSlaveaddr = 0x21;

    PRINTF("\r\n \r\n");
    PRINTF("\r\n---------------------------------------------------------\r\n");
    gTestCounter++;
    PRINTF("Test %d: FXLS8962 Sensor Driver Init Negative 2 Test.\r\n", gTestCounter);

    // Initialize the FXLS8962 sensor driver.
    status = FXLS8962_I2C_Initialize(&gFxls8962Driver, &I2C_S_DRIVER, I2C_S_DEVICE_INDEX, badSlaveaddr,
                                     FXLS8962_WHOAMI_VALUE);

    // CHECK - Check Sensor Driver Init Success/Failure
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_INIT, status);

    // CHECK - Check Sensor handle
    TEST_ASSERT_EQUAL_INT(false, gFxls8962Driver.isInitialized);

    /* Configure device to verify Register_Write failure.*/
    status = FXLS8962_I2C_Configure(&gFxls8962Driver, cFxls8962ConfigNormal);

    // CHECK - Check Sensor read status.
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_INIT, status);

    PRINTF("\r\n \r\n");
}

/*! -----------------------------------------------------------------------
 *  @brief       This is the FXLS8962 Sensor Driver Init Test Function.
 *  @details     Checks:
 *                  Return status of the sensor driver init function.
 *                  Who_Am_I of the sensor.
 *  @param[in]   void This is no input parameter.
 *  @return      void  There is no return value.
 *  @constraints This should be the third testfunction to be executed after test_fxls8962_negative_2().
 *  @reeentrant  No
 *  -----------------------------------------------------------------------*/
void test_fxls8962_init(void)
{
    int32_t status;

    PRINTF("\r\n---------------------------------------------------------\r\n");
    gTestCounter++;
    PRINTF("\r\nTest %d: FXLS8962 Sensor Driver Init Test\r\n", gTestCounter);

    // Initialize FXLS8962 sensor driver.
    status = FXLS8962_I2C_Initialize(&gFxls8962Driver, &I2C_S_DRIVER, I2C_S_DEVICE_INDEX, FXLS8962_I2C_ADDR,
                                     FXLS8962_WHOAMI_VALUE);

    PRINTF("\r\nStart performing init parameters check\r\n");
    // CHECK - Check Sensor Driver Init Success/Failure
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_NONE, status);

    // CHECK - Check Sensor State
    TEST_ASSERT_EQUAL_INT(true, gFxls8962Driver.isInitialized);

    PRINTF("\r\n \r\n");
}

/*! -----------------------------------------------------------------------
 *  @brief       This is the FXLS8962 Sensor Driver Negative 3 Test Function.
 *  @details     Checks:
 *                  Return status of the sensor driver functions, when applied
 *                  Bad Write command List
 *  @param[in]   void This is no input parameter.
 *  @return      void  There is no return value.
 *  @constraints This should be the fourth testfunction to be executed after test_fxls8962_init().
 *  @reeentrant  No
 *  -----------------------------------------------------------------------*/
void test_fxls8962_negative_3(void)
{
    int32_t status;
    const registerwritelist_t *pBadRegWriteList = NULL;

    PRINTF("\r\n \r\n");
    PRINTF("\r\n---------------------------------------------------------\r\n");
    gTestCounter++;
    PRINTF("Test %d: FXLS8962 Sensor Driver Negative 3 Test.\r\n", gTestCounter);

    // Configure the FXLS8962 sensor driver with FIFO mode.
    status = FXLS8962_I2C_Configure(&gFxls8962Driver, pBadRegWriteList);

    PRINTF("\r\nStart performing sensor state parameters check\r\n");
    // CHECK - Check Sensor Driver Config Success/Failure
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_INVALID_PARAM, status);

    PRINTF("\r\n \r\n");
}

/*! -----------------------------------------------------------------------
 *  @brief       This is the FXLS8962 Sensor Driver Config Test Function.
 *  @details     Checks:
 *                  Return status of the sensor driver config function,
 *                  SENS_CONFIG3 value configured for the sensor.
 *                  BUF_CONFIG1  value configured for the sensor.
 *                  BUF_CONFIG2  value configured for the sensor.
 *                  INT_EN       value configured for the sensor.
 *  @param[in]   void This is no input parameter.
 *  @return      void  There is no return value.
 *  @constraints This should be the fifth testfunction to be executed after test_fxls8962_negative_3().
 *  @reeentrant  No
 *  -----------------------------------------------------------------------*/
void test_fxls8962_config(void)
{
    int32_t status;
    uint8_t regBuf;

    PRINTF("\r\n \r\n");
    PRINTF("\r\n---------------------------------------------------------\r\n");
    gTestCounter++;
    PRINTF("Test %d: FXLS8962 Sensor Driver Config Test\r\n", gTestCounter);

    // Configure the FXLS8962 sensor driver with FIFO mode.
    status = FXLS8962_I2C_Configure(&gFxls8962Driver, cFxls8962ConfigNormal);

    PRINTF("\r\nStart performing FXLS8962 configured parameters check\r\n");
    // CHECK - Check Sensor Driver Config Success/Failure
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_NONE, status);

    // CHECK - Check Sensor handle
    TEST_ASSERT_EQUAL_INT(true, gFxls8962Driver.isInitialized);

    // Read the SENS_CONFIG3 value and check with reference test value.
    FXLS8962_I2C_ReadData(&gFxls8962Driver, cFxls8962SENS_CONFIG3, &regBuf);
    // CHECK - Check SENS_CONFIG3 value
    TEST_ASSERT_EQUAL_HEX(SENS_CONFIG3_TESTVAL, regBuf);

    // Read the SENS_CONFIG1 value and check with reference test value.
    FXLS8962_I2C_ReadData(&gFxls8962Driver, cFxls8962SENS_CONFIG1, &regBuf);
    // CHECK - Check SENS_CONFIG1 value
    TEST_ASSERT_EQUAL_HEX(SENS_CONFIG1_TESTVAL, regBuf);

    // Read the INT_EN value and check with reference test value.
    FXLS8962_I2C_ReadData(&gFxls8962Driver, cFxls8962INT_EN, &regBuf);
    // CHECK - Check BUF_CONFIG2 value
    TEST_ASSERT_EQUAL_HEX(INT_EN_TESTVAL, regBuf);

    PRINTF("\r\n \r\n");
}

/*! -----------------------------------------------------------------------
 *  @brief       This is the FXLS8962 Sensor Driver Negative 4 Test Function.
 *  @details     Checks:
 *                  Return status of the sensor driver functions, when applied
 *                  Bad Read command List
 *  @param[in]   void This is no input parameter.
 *  @return      void  There is no return value.
 *  @constraints This should be the sixth testfunction to be executed after test_fxls8962_config().
 *  @reeentrant  No
 *  -----------------------------------------------------------------------*/
void test_fxls8962_negative_4(void)
{
    int32_t status;
    uint8_t whoAmI;
    const registerreadlist_t *badRegReadList = NULL;

    PRINTF("\r\n \r\n");
    PRINTF("\r\n---------------------------------------------------------\r\n");
    gTestCounter++;
    PRINTF("Test %d: FXLS8962 Sensor Driver Negative 4 Test\r\n", gTestCounter);

    // Reading sensor data with bad read register list
    status = FXLS8962_I2C_ReadData(&gFxls8962Driver, badRegReadList, &whoAmI);

    // CHECK - Check Sensor Read Status
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_INVALID_PARAM, status);

    PRINTF("\r\n \r\n");
}

/*! -----------------------------------------------------------------------
 *  @brief       This is the FXLS8962 Sensor Driver deInit Test Function.
 *  @details     Checks:
 *                  Return status of the sensor driver deInit function,
 *                  State of the sensor after deInit
 *  @param[in]   void This is no input parameter.
 *  @return      void  There is no return value.
 *  @constraints This should be the seventh testfunction to be executed after test_fxls8962_negative_4().
 *  @reeentrant  No
 *  -----------------------------------------------------------------------*/
void test_fxls8962_deInit(void)
{
    int32_t status;

    PRINTF("\r\n \r\n");
    PRINTF("\r\n---------------------------------------------------------\r\n");
    gTestCounter++;
    PRINTF("Test %d: FXLS8962 Sensor Driver deInit Test\r\n", gTestCounter);

    // End the FXLS8962 sensor driver sampling.
    status = FXLS8962_I2C_DeInit(&gFxls8962Driver);

    PRINTF("\r\nStart performing sensor state parameters check\r\n");
    // CHECK - Check Sensor Driver Init Success/Failure
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_NONE, status);

    // CHECK - Check Sensor State
    TEST_ASSERT_EQUAL_INT(false, gFxls8962Driver.isInitialized);

    PRINTF("\r\n \r\n");
}

/* TearDown Required for test framework. */
void tearDown(void)
{
}

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
