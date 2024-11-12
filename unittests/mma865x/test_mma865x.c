/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*! @file  test_mma865x.c
 *  @brief This is the unit test file for mma865x sensor driver.
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
#include "mma865x_drv.h"

//-----------------------------------------------------------------------
// ISSDK Includes
//-----------------------------------------------------------------------
#include "unity.h"

//-----------------------------------------------------------------------
// Macros
//-----------------------------------------------------------------------
/* Configuration Values */
#define FIFO_WMRK_SIZE 16
#define MMA865x_DATA_SIZE 6

/* Test Values */
// Test MMA865x_CTRL_REG1 Value: ODR = 6.25Hz.
#define CTRL_REG1_TESTVAL MMA865x_CTRL_REG1_DR_6_25HZ | MMA865x_CTRL_REG1_ACTIVE_ACTIVATED
// Test MMA865x_F_SETUP Value: FIFO mode = STOP and Watermark Size = FIFO_WMRK_SIZE.
#define F_SETUP_TESTVAL MMA865x_F_SETUP_F_MODE_STOP_MODE | FIFO_WMRK_SIZE
// Test MMA865x_XYZ_DATA_CFG Value: FS range = 4g.
#define XYZ_DATA_CFG_TESTVAL MMA865x_XYZ_DATA_CFG_FS_4G
// Test MMA865x_CTRL_REG4 Value: FIFO INTERRUPT = Enabled.
#define CTRL_REG4_TESTVAL MMA865x_CTRL_REG4_INT_EN_FIFO_EN

//-----------------------------------------------------------------------
// Constants
//-----------------------------------------------------------------------
/* Register settings for FIFO (buffered) mode. */
const registerwritelist_t cMma865xConfigFIFO[] = {
    /* Set ODR Rate as 6.25Hz. */
    {MMA865x_CTRL_REG1, MMA865x_CTRL_REG1_DR_6_25HZ, MMA865x_CTRL_REG1_DR_MASK},
    /* Set Buffering Mode as Stop-when-Full and Buffer depth as WaterMark.  */
    {MMA865x_F_SETUP, MMA865x_F_SETUP_F_MODE_STOP_MODE | FIFO_WMRK_SIZE,
     MMA865x_F_SETUP_F_MODE_MASK | MMA865x_F_SETUP_F_WMRK_MASK},
    /* Set FS Range as 4g. */
    {MMA865x_XYZ_DATA_CFG, MMA865x_XYZ_DATA_CFG_FS_4G, MMA865x_XYZ_DATA_CFG_FS_MASK},
    /* Enable Interrupts for Buffer Events. */
    {MMA865x_CTRL_REG4, MMA865x_CTRL_REG4_INT_EN_FIFO_EN, MMA865x_CTRL_REG4_INT_EN_FIFO_MASK},
    __END_WRITE_DATA__};

/* Address of Who am I Register. */
const registerreadlist_t cMma865xWhoAmICommand[] = {{.readFrom = MMA865x_WHO_AM_I, .numBytes = 1}, __END_READ_DATA__};

/* Address of CTRL_REG1 Register. */
const registerreadlist_t cMma865xCTRL_REG1[] = {{.readFrom = MMA865x_CTRL_REG1, .numBytes = 1}, __END_READ_DATA__};

/* Address of F_SETUP Register. */
const registerreadlist_t cMma865xF_SETUP[] = {{.readFrom = MMA865x_F_SETUP, .numBytes = 1}, __END_READ_DATA__};

/* Address of XYZ_DATA_CFG Register. */
const registerreadlist_t cMma865xXYZ_DATA_CFG[] = {{.readFrom = MMA865x_XYZ_DATA_CFG, .numBytes = 1},
                                                   __END_READ_DATA__};

/* Address of CTRL_REG4 Register. */
const registerreadlist_t cMma865xCTRL_REG4[] = {{.readFrom = MMA865x_CTRL_REG4, .numBytes = 1}, __END_READ_DATA__};

//-----------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------
static int gTestCounter = 0;
mma865x_i2c_sensorhandle_t gMma865xDriver;
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
 *  @brief       This is the MMA865x Sensor Driver Negative 1 Test Function.
 *  @details     Checks:
 *                  Return status of the sensor driver functions, when applied
 *                  Corrupted I2C driver metadata
 *  @param[in]   void This is no input parameter.
 *  @return      void  There is no return value.
 *  @constraints This should be the first testfunction to be executed after setUp().
 *  @reeentrant  No
 *  -----------------------------------------------------------------------*/
void test_mma865x_negative_1(void)
{
    uint8_t whoAmI;
    int32_t status;
    ARM_DRIVER_I2C *pCorruptedI2Cdriver = NULL;

    PRINTF("\r\n \r\n");
    PRINTF("\r\n---------------------------------------------------------\r\n");
    gTestCounter++;
    PRINTF("Test %d: MMA865x Sensor Driver Negative 1 Test.\r\n", gTestCounter);

    // Initialize the MMA865x sensor driver.
    status = MMA865x_I2C_Initialize(&gMma865xDriver, pCorruptedI2Cdriver, I2C_S_DEVICE_INDEX, MMA8652_I2C_ADDR,
                                    MMA8652_WHOAMI_VALUE);

    PRINTF("\r\nStart performing init parameters check\r\n");
    // CHECK - Check Sensor Driver Init Success/Failure
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_INVALID_PARAM, status);

    // CHECK - Check Sensor handle
    TEST_ASSERT_EQUAL_INT(false, gMma865xDriver.isInitialized);

    // Reading sensor data with bad read register list
    status = MMA865x_I2C_ReadData(&gMma865xDriver, cMma865xWhoAmICommand, &whoAmI);

    // CHECK - Check Sensor Read Status
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_INIT, status);

    // Configure the MMA865x sensor driver with FIFO mode.
    status = MMA865x_I2C_Configure(&gMma865xDriver, cMma865xConfigFIFO);

    PRINTF("\r\nStart performing sensor state parameters check\r\n");
    // CHECK - Check Sensor Driver Init Success/Failure
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_INIT, status);

    PRINTF("\r\n \r\n");
}

/*! -----------------------------------------------------------------------
 *  @brief       This is the MMA865x Sensor Driver Negative 2 Test Function.
 *  @details     Checks:
 *                  Return status of the sensor driver functions, when applied
 *                  Bad I2C address
 *  @param[in]   void This is no input parameter.
 *  @return      void  There is no return value.
 *  @constraints This should be the second testfunction to be executed after test_mma865x_negative_1().
 *  @reeentrant  No
 *  -----------------------------------------------------------------------*/
void test_mma865x_negative_2(void)
{
    int32_t status;
    uint16_t badSlaveaddr = 0x21;

    PRINTF("\r\n \r\n");
    PRINTF("\r\n---------------------------------------------------------\r\n");
    gTestCounter++;
    PRINTF("Test %d: MMA865x Sensor Driver Init Negative 2 Test.\r\n", gTestCounter);

    // Initialize the MMA865x sensor driver.
    status =
        MMA865x_I2C_Initialize(&gMma865xDriver, &I2C_S_DRIVER, I2C_S_DEVICE_INDEX, badSlaveaddr, MMA8652_WHOAMI_VALUE);

    // CHECK - Check Sensor Driver Init Success/Failure
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_INIT, status);

    // CHECK - Check Sensor handle
    TEST_ASSERT_EQUAL_INT(false, gMma865xDriver.isInitialized);

    /* Configure device to verify Register_Write failure.*/
    status = MMA865x_I2C_Configure(&gMma865xDriver, cMma865xConfigFIFO);

    // CHECK - Check Sensor read status.
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_INIT, status);

    PRINTF("\r\n \r\n");
}

/*! -----------------------------------------------------------------------
 *  @brief       This is the MMA865x Sensor Driver Init Test Function.
 *  @details     Checks:
 *                  Return status of the sensor driver init function.
 *                  Who_Am_I of the sensor.
 *  @param[in]   void This is no input parameter.
 *  @return      void  There is no return value.
 *  @constraints This should be the third testfunction to be executed after test_mma865x_negative_2().
 *  @reeentrant  No
 *  -----------------------------------------------------------------------*/
void test_mma865x_init(void)
{
    int32_t status;

    PRINTF("\r\n---------------------------------------------------------\r\n");
    gTestCounter++;
    PRINTF("\r\nTest %d: MMA865x Sensor Driver Init Test\r\n", gTestCounter);

    // Initialize MMA865x sensor driver.
    status = MMA865x_I2C_Initialize(&gMma865xDriver, &I2C_S_DRIVER, I2C_S_DEVICE_INDEX, MMA8652_I2C_ADDR,
                                    MMA8652_WHOAMI_VALUE);

    PRINTF("\r\nStart performing init parameters check\r\n");
    // CHECK - Check Sensor Driver Init Success/Failure
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_NONE, status);

    // CHECK - Check Sensor State
    TEST_ASSERT_EQUAL_INT(true, gMma865xDriver.isInitialized);

    PRINTF("\r\n \r\n");
}

/*! -----------------------------------------------------------------------
 *  @brief       This is the MMA865x Sensor Driver Negative 3 Test Function.
 *  @details     Checks:
 *                  Return status of the sensor driver functions, when applied
 *                  Bad Write command List
 *  @param[in]   void This is no input parameter.
 *  @return      void  There is no return value.
 *  @constraints This should be the fourth testfunction to be executed after test_mma865x_init().
 *  @reeentrant  No
 *  -----------------------------------------------------------------------*/
void test_mma865x_negative_3(void)
{
    int32_t status;
    const registerwritelist_t *pBadRegWriteList = NULL;

    PRINTF("\r\n \r\n");
    PRINTF("\r\n---------------------------------------------------------\r\n");
    gTestCounter++;
    PRINTF("Test %d: MMA865x Sensor Driver Negative 3 Test.\r\n", gTestCounter);

    // Configure the MMA865x sensor driver with FIFO mode.
    status = MMA865x_I2C_Configure(&gMma865xDriver, pBadRegWriteList);

    PRINTF("\r\nStart performing sensor state parameters check\r\n");
    // CHECK - Check Sensor Driver Config Success/Failure
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_INVALID_PARAM, status);

    PRINTF("\r\n \r\n");
}

/*! -----------------------------------------------------------------------
 *  @brief       This is the MMA865x Sensor Driver Config Test Function.
 *  @details     Checks:
 *                  Return status of the sensor driver config function,
 *                  SENS_CONFIG3 value configured for the sensor.
 *                  BUF_CONFIG1  value configured for the sensor.
 *                  BUF_CONFIG2  value configured for the sensor.
 *                  INT_EN       value configured for the sensor.
 *  @param[in]   void This is no input parameter.
 *  @return      void  There is no return value.
 *  @constraints This should be the fifth testfunction to be executed after test_mma865x_negative_3().
 *  @reeentrant  No
 *  -----------------------------------------------------------------------*/
void test_mma865x_config(void)
{
    int32_t status;
    uint8_t regBuf;

    PRINTF("\r\n \r\n");
    PRINTF("\r\n---------------------------------------------------------\r\n");
    gTestCounter++;
    PRINTF("Test %d: MMA865x Sensor Driver Config Test\r\n", gTestCounter);

    // Configure the MMA865x sensor driver with FIFO mode.
    status = MMA865x_I2C_Configure(&gMma865xDriver, cMma865xConfigFIFO);

    PRINTF("\r\nStart performing MMA865x configured parameters check\r\n");
    // CHECK - Check Sensor Driver Config Success/Failure
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_NONE, status);

    // CHECK - Check Sensor handle
    TEST_ASSERT_EQUAL_INT(true, gMma865xDriver.isInitialized);

    // Read the CTRL_REG1 value and check with reference test value.
    MMA865x_I2C_ReadData(&gMma865xDriver, cMma865xCTRL_REG1, &regBuf);
    // CHECK - Check CTRL_REG1 value
    TEST_ASSERT_EQUAL_HEX(CTRL_REG1_TESTVAL, regBuf);

    // Read the F_SETUP value and check with reference test value.
    MMA865x_I2C_ReadData(&gMma865xDriver, cMma865xF_SETUP, &regBuf);
    // CHECK - Check F_SETUP value
    TEST_ASSERT_EQUAL_HEX(F_SETUP_TESTVAL, regBuf);

    // Read the XYZ_DATA_CFG value and check with reference test value.
    MMA865x_I2C_ReadData(&gMma865xDriver, cMma865xXYZ_DATA_CFG, &regBuf);
    // CHECK - Check XYZ_DATA_CFG value
    TEST_ASSERT_EQUAL_HEX(XYZ_DATA_CFG_TESTVAL, regBuf);

    // Read the CTRL_REG4 value and check with reference test value.
    MMA865x_I2C_ReadData(&gMma865xDriver, cMma865xCTRL_REG4, &regBuf);
    // CHECK - Check CTRL_REG4 value
    TEST_ASSERT_EQUAL_HEX(CTRL_REG4_TESTVAL, regBuf);

    PRINTF("\r\n \r\n");
}

/*! -----------------------------------------------------------------------
 *  @brief       This is the MMA865x Sensor Driver Negative 4 Test Function.
 *  @details     Checks:
 *                  Return status of the sensor driver functions, when applied
 *                  Bad Read command List
 *  @param[in]   void This is no input parameter.
 *  @return      void  There is no return value.
 *  @constraints This should be the sixth testfunction to be executed after test_mma865x_config().
 *  @reeentrant  No
 *  -----------------------------------------------------------------------*/
void test_mma865x_negative_4(void)
{
    int32_t status;
    uint8_t whoAmI;
    const registerreadlist_t *badRegReadList = NULL;

    PRINTF("\r\n \r\n");
    PRINTF("\r\n---------------------------------------------------------\r\n");
    gTestCounter++;
    PRINTF("Test %d: MMA865x Sensor Driver Negative 4 Test\r\n", gTestCounter);

    // Reading sensor data with bad read register list
    status = MMA865x_I2C_ReadData(&gMma865xDriver, badRegReadList, &whoAmI);

    // CHECK - Check Sensor Read Status
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_INVALID_PARAM, status);

    PRINTF("\r\n \r\n");
}

/*! -----------------------------------------------------------------------
 *  @brief       This is the MMA865x Sensor Driver deInit Test Function.
 *  @details     Checks:
 *                  Return status of the sensor driver deInit function,
 *                  State of the sensor after deInit
 *  @param[in]   void This is no input parameter.
 *  @return      void  There is no return value.
 *  @constraints This should be the seventh testfunction to be executed after test_mma865x_negative_4().
 *  @reeentrant  No
 *  -----------------------------------------------------------------------*/
void test_mma865x_deInit(void)
{
    int32_t status;

    PRINTF("\r\n \r\n");
    PRINTF("\r\n---------------------------------------------------------\r\n");
    gTestCounter++;
    PRINTF("Test %d: MMA865x Sensor Driver deInit Test\r\n", gTestCounter);

    // End the MMA865x sensor driver sampling.
    status = MMA865x_I2C_DeInit(&gMma865xDriver);

    PRINTF("\r\nStart performing sensor state parameters check\r\n");
    // CHECK - Check Sensor Driver Init Success/Failure
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_NONE, status);

    // CHECK - Check Sensor State
    TEST_ASSERT_EQUAL_INT(false, gMma865xDriver.isInitialized);

    PRINTF("\r\n \r\n");
}

/* TearDown Required for test framework. */
void tearDown(void)
{
}

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
