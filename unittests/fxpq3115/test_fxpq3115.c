/*
 * Copyright (c) 2015 - 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*! @file  test_fxpq3115.c
 *  @brief This is the unit test file for fxpq3115 sensor driver.
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
#include "fxpq3115_drv.h"

//-----------------------------------------------------------------------
// Unity Includes
//-----------------------------------------------------------------------
#include "unity.h"

//-----------------------------------------------------------------------
// Macros
//-----------------------------------------------------------------------
/* Configuration Values */
#define FIFO_WMRK_SIZE (8)     /* Buffer 8 Safxpqes. */
#define FXPQ3115_DATA_SIZE (5) /* 3 byte Pressure/Altitude and 2 byte Temperature. */

/* Reference Values */
// Test FXPQ3115 F_SETUP     Value: FIFO mode = STOP and Watermark Size = FIFO_WMRK_SIZE.
#define F_SETUP_TESTVAL (FXPQ3115_F_SETUP_F_MODE_STOP_MODE | FIFO_WMRK_SIZE)
// Test FXPQ3115 PT_DATA_CFG Value: DataReady = Enabled, Pressute/Altitude Evant = Enabled and Temperature Event =
// Enabled.
#define PT_DATA_CFG_TESTVAL \
    (FXPQ3115_PT_DATA_CFG_TDEFE_ENABLED | FXPQ3115_PT_DATA_CFG_PDEFE_ENABLED | FXPQ3115_PT_DATA_CFG_DREM_ENABLED)
// Test FXPQ3115 CTRL_REG1   Value: Over Safxpqing Rate = 128 and Mode = Active Mode.
#define CTRL_REG1_TESTVAL (FXPQ3115_CTRL_REG1_OS_OSR_128 | FXPQ3115_CTRL_REG1_SBYB_ACTIVE)
// Test FXPQ3115 CTRL_REG4   Value: FIFO INTERRUPT = Enabled.
#define CTRL_REG4_TESTVAL (FXPQ3115_CTRL_REG4_INT_EN_FIFO_INTENABLED)

//-----------------------------------------------------------------------
// Constants
//-----------------------------------------------------------------------
/*! @brief Register settings for FIFO (buffered) mode with interrupts. */
const registerwritelist_t cFxpq3115ConfigFIFO[] = {
    /* Set FIFO Mode and set FIFO Watermark Level. */
    {FXPQ3115_F_SETUP, FXPQ3115_F_SETUP_F_MODE_STOP_MODE | FIFO_WMRK_SIZE,
     FXPQ3115_F_SETUP_F_MODE_MASK | FXPQ3115_F_SETUP_F_WMRK_MASK},
    /* Enable Data Ready and Event flags for Pressure, Temperature or either. */
    {FXPQ3115_PT_DATA_CFG,
     FXPQ3115_PT_DATA_CFG_TDEFE_ENABLED | FXPQ3115_PT_DATA_CFG_PDEFE_ENABLED | FXPQ3115_PT_DATA_CFG_DREM_ENABLED,
     FXPQ3115_PT_DATA_CFG_TDEFE_MASK | FXPQ3115_PT_DATA_CFG_PDEFE_MASK | FXPQ3115_PT_DATA_CFG_DREM_MASK},
    /* Set Over Safxpqing Ratio to 128. */
    {FXPQ3115_CTRL_REG1, FXPQ3115_CTRL_REG1_OS_OSR_128, FXPQ3115_CTRL_REG1_OS_MASK},
    /* Enable Interrupts for FIFO Events. */
    {FXPQ3115_CTRL_REG4, FXPQ3115_CTRL_REG4_INT_EN_FIFO_INTENABLED, FXPQ3115_CTRL_REG4_INT_EN_FIFO_MASK},
    __END_WRITE_DATA__};

/* Address of Who am I Register. */
const registerreadlist_t cFxpq3115WhoAmICommand[] = {{.readFrom = FXPQ3115_WHO_AM_I, .numBytes = 1}, __END_READ_DATA__};

/* Address of F_SETUP Register. */
const registerreadlist_t cFxpq3115F_SETUP[] = {{.readFrom = FXPQ3115_F_SETUP, .numBytes = 1}, __END_READ_DATA__};

/* Address of PT_DATA_CFG Register. */
const registerreadlist_t cFxpq3115PT_DATA_CFG[] = {{.readFrom = FXPQ3115_PT_DATA_CFG, .numBytes = 1},
                                                   __END_READ_DATA__};

/* Address of CTRL_REG1 Register. */
const registerreadlist_t cFxpq3115CTRL_REG1[] = {{.readFrom = FXPQ3115_CTRL_REG1, .numBytes = 1}, __END_READ_DATA__};

/* Address of CTRL_REG4 Register. */
const registerreadlist_t cFxpq3115CTRL_REG4[] = {{.readFrom = FXPQ3115_CTRL_REG4, .numBytes = 1}, __END_READ_DATA__};

//-----------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------
static int gTestCounter = 0;
fxpq3115_i2c_sensorhandle_t gFxpq3115Driver;
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
 *  @brief       This is the FXPQ3115 Sensor Driver Negative 1 Test Function.
 *  @details     Checks:
 *                  Return status of the sensor driver functions, when applied
 *                  Corrupted I2C driver metadata
 *  @param[in]   void This is no input parameter.
 *  @return      void  There is no return value.
 *  @constraints This should be the first testfunction to be executed after setUp().
 *  @reeentrant  No
 *  -----------------------------------------------------------------------*/
void test_fxpq3115_negative_1(void)
{
    uint8_t whoAmI;
    int32_t status;
    ARM_DRIVER_I2C *pCorruptedI2Cdriver = NULL;

    PRINTF("\r\n \r\n");
    PRINTF("\r\n---------------------------------------------------------\r\n");
    gTestCounter++;
    PRINTF("Test %d: FXPQ3115 Sensor Driver Negative 1 Test.\r\n", gTestCounter);

    // Initialize the FXPQ3115 sensor driver.
    status = FXPQ3115_I2C_Initialize(&gFxpq3115Driver, pCorruptedI2Cdriver, I2C_S_DEVICE_INDEX, FXPQ3115_I2C_ADDR,
                                     FXPQ3115_WHOAMI_VALUE);

    PRINTF("\r\nStart performing init parameters check\r\n");
    // CHECK - Check Sensor Driver Init Success/Failure
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_INVALID_PARAM, status);

    // CHECK - Check Sensor handle
    TEST_ASSERT_EQUAL_INT(false, gFxpq3115Driver.isInitialized);

    // Reading sensor data with bad read register list
    status = FXPQ3115_I2C_ReadData(&gFxpq3115Driver, cFxpq3115WhoAmICommand, &whoAmI);

    // CHECK - Check Sensor Read Status
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_INIT, status);

    // Configure the FXPQ3115 sensor driver with FIFO mode.
    status = FXPQ3115_I2C_Configure(&gFxpq3115Driver, cFxpq3115ConfigFIFO);

    PRINTF("\r\nStart performing sensor state parameters check\r\n");
    // CHECK - Check Sensor Driver Init Success/Failure
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_INIT, status);

    PRINTF("\r\n \r\n");
}

/*! -----------------------------------------------------------------------
 *  @brief       This is the FXPQ3115 Sensor Driver Negative 2 Test Function.
 *  @details     Checks:
 *                  Return status of the sensor driver functions, when applied
 *                  Bad I2C address
 *  @param[in]   void This is no input parameter.
 *  @return      void  There is no return value.
 *  @constraints This should be the second testfunction to be executed after test_fxpq3115_negative_1().
 *  @reeentrant  No
 *  -----------------------------------------------------------------------*/
void test_fxpq3115_negative_2(void)
{
    int32_t status;
    uint16_t badSlaveaddr = 0x21;

    PRINTF("\r\n \r\n");
    PRINTF("\r\n---------------------------------------------------------\r\n");
    gTestCounter++;
    PRINTF("Test %d: FXPQ3115 Sensor Driver Init Negative 2 Test.\r\n", gTestCounter);

    // Initialize the FXPQ3115 sensor driver.
    status = FXPQ3115_I2C_Initialize(&gFxpq3115Driver, &I2C_S_DRIVER, I2C_S_DEVICE_INDEX, badSlaveaddr,
                                     FXPQ3115_WHOAMI_VALUE);

    // CHECK - Check Sensor Driver Init Success/Failure
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_INIT, status);

    // CHECK - Check Sensor handle
    TEST_ASSERT_EQUAL_INT(false, gFxpq3115Driver.isInitialized);

    /* Configure device to verify Register_Write failure.*/
    status = FXPQ3115_I2C_Configure(&gFxpq3115Driver, cFxpq3115ConfigFIFO);

    // CHECK - Check Sensor read status.
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_INIT, status);

    PRINTF("\r\n \r\n");
}

/*! -----------------------------------------------------------------------
 *  @brief       This is the FXPQ3115 Sensor Driver Init Test Function.
 *  @details     Checks:
 *                  Return status of the sensor driver init function.
 *                  Who_Am_I of the sensor.
 *  @param[in]   void This is no input parameter.
 *  @return      void  There is no return value.
 *  @constraints This should be the third testfunction to be executed after test_fxpq3115_negative_2().
 *  @reeentrant  No
 *  -----------------------------------------------------------------------*/
void test_fxpq3115_init(void)
{
    int32_t status;

    PRINTF("\r\n---------------------------------------------------------\r\n");
    gTestCounter++;
    PRINTF("\r\nTest %d: FXPQ3115 Sensor Driver Init Test\r\n", gTestCounter);

    // Initialize FXPQ3115 sensor driver.
    status = FXPQ3115_I2C_Initialize(&gFxpq3115Driver, &I2C_S_DRIVER, I2C_S_DEVICE_INDEX, FXPQ3115_I2C_ADDR,
                                     FXPQ3115_WHOAMI_VALUE);

    PRINTF("\r\nStart performing init parameters check\r\n");
    // CHECK - Check Sensor Driver Init Success/Failure
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_NONE, status);

    // CHECK - Check Sensor State
    TEST_ASSERT_EQUAL_INT(true, gFxpq3115Driver.isInitialized);

    PRINTF("\r\n \r\n");
}

/*! -----------------------------------------------------------------------
 *  @brief       This is the FXPQ3115 Sensor Driver Negative 3 Test Function.
 *  @details     Checks:
 *                  Return status of the sensor driver functions, when applied
 *                  Bad Write command List
 *  @param[in]   void This is no input parameter.
 *  @return      void  There is no return value.
 *  @constraints This should be the fourth testfunction to be executed after test_fxpq3115_init().
 *  @reeentrant  No
 *  -----------------------------------------------------------------------*/
void test_fxpq3115_negative_3(void)
{
    int32_t status;
    const registerwritelist_t *pBadRegWriteList = NULL;

    PRINTF("\r\n \r\n");
    PRINTF("\r\n---------------------------------------------------------\r\n");
    gTestCounter++;
    PRINTF("Test %d: FXPQ3115 Sensor Driver Negative 3 Test.\r\n", gTestCounter);

    // Configure the FXPQ3115 sensor driver with FIFO mode.
    status = FXPQ3115_I2C_Configure(&gFxpq3115Driver, pBadRegWriteList);

    PRINTF("\r\nStart performing sensor state parameters check\r\n");
    // CHECK - Check Sensor Driver Config Success/Failure
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_INVALID_PARAM, status);

    PRINTF("\r\n \r\n");
}

/*! -----------------------------------------------------------------------
 *  @brief       This is the FXPQ3115 Sensor Driver Config Test Function.
 *  @details     Checks:
 *                  Return status of the sensor driver config function,
 *                  F_SETUP      value configured for the sensor.
 *                  PT_DATA_CFG  value configured for the sensor.
 *                  CTRL_REG1    value configured for the sensor.
 *                  CTRL_REG4    value configured for the sensor.
 *  @param[in]   void This is no input parameter.
 *  @return      void  There is no return value.
 *  @constraints This should be the fifth testfunction to be executed after test_fxpq3115_negative_3().
 *  @reeentrant  No
 *  -----------------------------------------------------------------------*/
void test_fxpq3115_config(void)
{
    int32_t status;
    uint8_t regBuf;

    PRINTF("\r\n \r\n");
    PRINTF("\r\n---------------------------------------------------------\r\n");
    gTestCounter++;
    PRINTF("Test %d: FXPQ3115 Sensor Driver Config Test\r\n", gTestCounter);

    // Configure the FXPQ3115 sensor driver with FIFO mode.
    status = FXPQ3115_I2C_Configure(&gFxpq3115Driver, cFxpq3115ConfigFIFO);

    PRINTF("\r\nStart performing FXPQ3115 configured parameters check\r\n");
    // CHECK - Check Sensor Driver Config Success/Failure
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_NONE, status);

    // CHECK - Check Sensor handle
    TEST_ASSERT_EQUAL_INT(true, gFxpq3115Driver.isInitialized);

    // Read the F_SETUP value and check with reference test value.
    FXPQ3115_I2C_ReadData(&gFxpq3115Driver, cFxpq3115F_SETUP, &regBuf);
    // CHECK - Check F_SETUP value
    TEST_ASSERT_EQUAL_HEX(F_SETUP_TESTVAL, regBuf);

    // Read the PT_DATA_CFG value and check with reference test value.
    FXPQ3115_I2C_ReadData(&gFxpq3115Driver, cFxpq3115PT_DATA_CFG, &regBuf);
    // CHECK - Check PT_DATA_CFG value
    TEST_ASSERT_EQUAL_HEX(PT_DATA_CFG_TESTVAL, regBuf);

    // Read the CTRL_REG1 value and check with reference test value.
    FXPQ3115_I2C_ReadData(&gFxpq3115Driver, cFxpq3115CTRL_REG1, &regBuf);
    // CHECK - Check CTRL_REG1 value
    TEST_ASSERT_EQUAL_HEX(CTRL_REG1_TESTVAL, regBuf & (~FXPQ3115_CTRL_REG1_RST_EN));

    // Read the CTRL_REG4 value and check with reference test value.
    FXPQ3115_I2C_ReadData(&gFxpq3115Driver, cFxpq3115CTRL_REG4, &regBuf);
    // CHECK - Check CTRL_REG4 value
    TEST_ASSERT_EQUAL_HEX(CTRL_REG4_TESTVAL, regBuf);

    PRINTF("\r\n \r\n");
}

/*! -----------------------------------------------------------------------
 *  @brief       This is the FXPQ3115 Sensor Driver Negative 4 Test Function.
 *  @details     Checks:
 *                  Return status of the sensor driver functions, when applied
 *                  Bad Read command List
 *  @param[in]   void This is no input parameter.
 *  @return      void  There is no return value.
 *  @constraints This should be the sixth testfunction to be executed after test_fxpq3115_config().
 *  @reeentrant  No
 *  -----------------------------------------------------------------------*/
void test_fxpq3115_negative_4(void)
{
    int32_t status;
    uint8_t whoAmI;
    const registerreadlist_t *badRegReadList = NULL;

    PRINTF("\r\n \r\n");
    PRINTF("\r\n---------------------------------------------------------\r\n");
    gTestCounter++;
    PRINTF("Test %d: FXPQ3115 Sensor Driver Negative 4 Test\r\n", gTestCounter);

    // Reading sensor data with bad read register list
    status = FXPQ3115_I2C_ReadData(&gFxpq3115Driver, badRegReadList, &whoAmI);

    // CHECK - Check Sensor Read Status
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_INVALID_PARAM, status);

    PRINTF("\r\n \r\n");
}

/*! -----------------------------------------------------------------------
 *  @brief       This is the FXPQ3115 Sensor Driver deInit Test Function.
 *  @details     Checks:
 *                  Return status of the sensor driver deInit function,
 *                  State of the sensor after deInit
 *  @param[in]   void This is no input parameter.
 *  @return      void  There is no return value.
 *  @constraints This should be the seventh testfunction to be executed after test_fxpq3115_negative_4().
 *  @reeentrant  No
 *  -----------------------------------------------------------------------*/
void test_fxpq3115_deInit(void)
{
    int32_t status;

    PRINTF("\r\n \r\n");
    PRINTF("\r\n---------------------------------------------------------\r\n");
    gTestCounter++;
    PRINTF("Test %d: FXPQ3115 Sensor Driver deInit Test\r\n", gTestCounter);

    // End the FXPQ3115 sensor driver safxpqing.
    status = FXPQ3115_I2C_DeInit(&gFxpq3115Driver);

    PRINTF("\r\nStart performing sensor state parameters check\r\n");
    // CHECK - Check Sensor Driver De-Init Success/Failure
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_NONE, status);

    // CHECK - Check Sensor State
    TEST_ASSERT_EQUAL_INT(false, gFxpq3115Driver.isInitialized);

    PRINTF("\r\n \r\n");
}

/* TearDown Required for test framework. */
void tearDown(void)
{
}

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
