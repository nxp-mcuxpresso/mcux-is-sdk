/*
 * Copyright (c) 2015 - 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*! @file  test_mma9553_i2c.c
 *  @brief This is the unit test file for mma9553 I2C sensor driver.
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
#include "mma9553_drv.h"
#include "systick_utils.h"

//-----------------------------------------------------------------------
// Unity Includes
//-----------------------------------------------------------------------
#include "unity.h"

//-----------------------------------------------------------------------
// Macros
//-----------------------------------------------------------------------
/* Configuration Values */
#define MMA9553_ACCEL_DATA_SIZE (6) /* 2 byte each of X,Y,Z Data. */

/* Reference Values */
// Test MMA9553 FS_RANGE       Value: 2G.
#define FS_RANGE_TESTVAL (0x40)
// Test MMA9553 SAMPLE_RATE    Value: 30Hz.
#define SAMPLE_RATE_TESTVAL (0x0B)
// Test MMA9553 AFE_PRIORITY   Value: For 30Hz.
#define AFE_PRIORITY_TESTVAL (0xD3)
// Test MMA9553 MBox_PRIORITY  Value: For 30 Hz.
#define MBox_PRIORITY_TESTVAL (0xD3)

//-----------------------------------------------------------------------
// Constants
//-----------------------------------------------------------------------
const uint8_t GetFSRange[] = {0x06, 0x10, 0x00, 0x01};
const uint8_t GetSampleRate[] = {0x06, 0x10, 0x0C, 0x01};
const uint8_t GetAFEPriority[] = {0x01, 0x10, 0x32, 0x01};
const uint8_t GetMBoxPriority[] = {0x01, 0x10, 0x30, 0x01};

/*! Prepare the register write list to configure MMA9553L in 30Hz mode. */
const registercommandlist_t cMma9553Config30Hz[] = {
    {SetFSRange_2g, 0, sizeof(SetFSRange_2g)},                     /* Set FS Range 2G */
    {SetSampleRate_30Hz, 0, sizeof(SetSampleRate_30Hz)},           /* Set Sensor Sampling Rate 30Hz */
    {SetAFEPriority_for30Hz, 0, sizeof(SetAFEPriority_for30Hz)},   /* Set AFE Priority for 30Hz Sampling Rate */
    {SetMBoxPriority_for30Hz, 0, sizeof(SetMBoxPriority_for30Hz)}, /* Set MBox Priority for 30Hz Sampling Rate */
    __END_WRITE_CMD__};

/*! Prepare the register write list to fetch MMA9553L FS Range. */
const registercommandlist_t cMma9553_Get_FS_Range[] = {{GetFSRange, 0, sizeof(GetFSRange)}, /* Get FS Range */
                                                       __END_WRITE_CMD__};

/*! Prepare the register write list to fetch MMA9553L Sample Rate. */
const registercommandlist_t cMma9553_Get_Sample_Rate[] = {
    {GetSampleRate, 0, sizeof(GetSampleRate)}, /* Get Sample Rate */
    __END_WRITE_CMD__};

/*! Prepare the register write list to fetch MMA9553L AFE Priority. */
const registercommandlist_t cMma9553_Get_AFE_Priority[] = {
    {GetAFEPriority, 0, sizeof(GetAFEPriority)}, /* Get AFE Priority */
    __END_WRITE_CMD__};

/*! Prepare the register write list to fetch MMA9553L MBox Priority. */
const registercommandlist_t cMma9553_Get_MBox_Priority[] = {
    {GetMBoxPriority, 0, sizeof(GetMBoxPriority)}, /* Get MBox Priority */
    __END_WRITE_CMD__};

/* Address of Command Response. */
const registerreadlist_t cMma9553_CMD_Response[] = {{.readFrom = 0, .numBytes = MMA9553_HDR_SIZE + 1},
                                                    __END_READ_DATA__};

//-----------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------
static int gTestCounter = 0;
mma9553_i2c_sensorhandle_t gMma9553Driver;
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
 *  @brief       This is the MMA9553 Sensor Driver Negative 1 Test Function.
 *  @details     Checks:
 *                  Return status of the sensor driver functions, when applied
 *                  Corrupted I2C driver metadata
 *  @param[in]   void This is no input parameter.
 *  @return      void  There is no return value.
 *  @constraints This should be the first testfunction to be executed after setUp().
 *  @reeentrant  No
 *  -----------------------------------------------------------------------*/
void test_mma9553_negative_1(void)
{
    uint8_t cmdResp;
    int32_t status;
    ARM_DRIVER_I2C *pCorruptedI2Cdriver = NULL;

    PRINTF("\r\n \r\n");
    PRINTF("\r\n---------------------------------------------------------\r\n");
    gTestCounter++;
    PRINTF("Test %d: MMA9553 Sensor Driver Negative 1 Test.\r\n", gTestCounter);

    // Initialize the MMA9553 sensor driver.
    status = MMA9553_I2C_Initialize(&gMma9553Driver, pCorruptedI2Cdriver, I2C_S_DEVICE_INDEX, MMA9553_I2C_ADDR);

    PRINTF("\r\nStart performing init parameters check\r\n");
    // CHECK - Check Sensor Driver Init Success/Failure
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_INVALID_PARAM, status);

    // CHECK - Check Sensor handle
    TEST_ASSERT_EQUAL_INT(false, gMma9553Driver.isInitialized);

    // Reading sensor data with bad read register list
    status = MMA9553_I2C_CommandResponse(&gMma9553Driver, NULL, cMma9553_CMD_Response, &cmdResp);

    // CHECK - Check Sensor Read Status
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_INIT, status);

    // Configure the MMA9553 sensor driver with FIFO mode.
    status = MMA9553_I2C_Configure(&gMma9553Driver, cMma9553Config30Hz);

    PRINTF("\r\nStart performing sensor state parameters check\r\n");
    // CHECK - Check Sensor Driver Init Success/Failure
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_INIT, status);

    PRINTF("\r\n \r\n");
}

/*! -----------------------------------------------------------------------
 *  @brief       This is the MMA9553 Sensor Driver Negative 2 Test Function.
 *  @details     Checks:
 *                  Return status of the sensor driver functions, when applied
 *                  Bad I2C address
 *  @param[in]   void This is no input parameter.
 *  @return      void  There is no return value.
 *  @constraints This should be the second testfunction to be executed after test_mma9553_negative_1().
 *  @reeentrant  No
 *  -----------------------------------------------------------------------*/
void test_mma9553_negative_2(void)
{
    int32_t status;
    uint16_t badSlaveaddr = 0x21;

    PRINTF("\r\n \r\n");
    PRINTF("\r\n---------------------------------------------------------\r\n");
    gTestCounter++;
    PRINTF("Test %d: MMA9553 Sensor Driver Init Negative 2 Test.\r\n", gTestCounter);

    // Initialize the MMA9553 sensor driver.
    status = MMA9553_I2C_Initialize(&gMma9553Driver, &I2C_S_DRIVER, I2C_S_DEVICE_INDEX, badSlaveaddr);

    // CHECK - Check Sensor Driver Init Success/Failure
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_INIT, status);

    // CHECK - Check Sensor handle
    TEST_ASSERT_EQUAL_INT(false, gMma9553Driver.isInitialized);

    /* Configure device to verify Register_Write failure.*/
    status = MMA9553_I2C_Configure(&gMma9553Driver, cMma9553Config30Hz);

    // CHECK - Check Sensor read status.
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_INIT, status);

    PRINTF("\r\n \r\n");
}

/*! -----------------------------------------------------------------------
 *  @brief       This is the MMA9553 Sensor Driver Init Test Function.
 *  @details     Checks:
 *                  Return status of the sensor driver init function.
 *                  Who_Am_I of the sensor.
 *  @param[in]   void This is no input parameter.
 *  @return      void  There is no return value.
 *  @constraints This should be the third testfunction to be executed after test_mma9553_negative_2().
 *  @reeentrant  No
 *  -----------------------------------------------------------------------*/
void test_mma9553_init(void)
{
    int32_t status;

    PRINTF("\r\n---------------------------------------------------------\r\n");
    gTestCounter++;
    PRINTF("\r\nTest %d: MMA9553 Sensor Driver Init Test\r\n", gTestCounter);

    // Initialize MMA9553 sensor driver.
    status = MMA9553_I2C_Initialize(&gMma9553Driver, &I2C_S_DRIVER, I2C_S_DEVICE_INDEX, MMA9553_I2C_ADDR);

    PRINTF("\r\nStart performing init parameters check\r\n");
    // CHECK - Check Sensor Driver Init Success/Failure
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_NONE, status);

    // CHECK - Check Sensor State
    TEST_ASSERT_EQUAL_INT(true, gMma9553Driver.isInitialized);

    PRINTF("\r\n \r\n");
}

/*! -----------------------------------------------------------------------
 *  @brief       This is the MMA9553 Sensor Driver Negative 3 Test Function.
 *  @details     Checks:
 *                  Return status of the sensor driver functions, when applied
 *                  Bad Write command List
 *  @param[in]   void This is no input parameter.
 *  @return      void  There is no return value.
 *  @constraints This should be the fourth testfunction to be executed after test_mma9553_init().
 *  @reeentrant  No
 *  -----------------------------------------------------------------------*/
void test_mma9553_negative_3(void)
{
    int32_t status;
    const registercommandlist_t *pBadRegCmdList = NULL;

    PRINTF("\r\n \r\n");
    PRINTF("\r\n---------------------------------------------------------\r\n");
    gTestCounter++;
    PRINTF("Test %d: MMA9553 Sensor Driver Negative 3 Test.\r\n", gTestCounter);

    // Configure the MMA9553 sensor driver with FIFO mode.
    status = MMA9553_I2C_Configure(&gMma9553Driver, pBadRegCmdList);

    PRINTF("\r\nStart performing sensor state parameters check\r\n");
    // CHECK - Check Sensor Driver Config Success/Failure
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_INVALID_PARAM, status);

    PRINTF("\r\n \r\n");
}

/*! -----------------------------------------------------------------------
 *  @brief       This is the MMA9553 Sensor Driver Config Test Function.
 *  @details     Checks:
 *                  Return status of the sensor driver config function,
 *                  FS_RANGE       value configured for the sensor.
 *                  SAMPLE_RATE    value configured for the sensor.
 *                  AFE_PRIORITY   value configured for the sensor.
 *                  MBox_PRIORITY  value configured for the sensor.
 *  @param[in]   void This is no input parameter.
 *  @return      void  There is no return value.
 *  @constraints This should be the fifth testfunction to be executed after test_mma9553_negative_3().
 *  @reeentrant  No
 *  -----------------------------------------------------------------------*/
void test_mma9553_config(void)
{
    int32_t status;
    uint8_t respBuf[MMA9553_HDR_SIZE + 1];

    PRINTF("\r\n \r\n");
    PRINTF("\r\n---------------------------------------------------------\r\n");
    gTestCounter++;
    PRINTF("Test %d: MMA9553 Sensor Driver Config Test\r\n", gTestCounter);

    // Configure the MMA9553 sensor driver with FIFO mode.
    status = MMA9553_I2C_Configure(&gMma9553Driver, cMma9553Config30Hz);

    PRINTF("\r\nStart performing MMA9553 configured parameters check\r\n");
    // CHECK - Check Sensor Driver Config Success/Failure
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_NONE, status);

    // CHECK - Check Sensor handle
    TEST_ASSERT_EQUAL_INT(true, gMma9553Driver.isInitialized);

    // Read the FS_RANGE value and check with reference test value.
    MMA9553_I2C_CommandResponse(&gMma9553Driver, cMma9553_Get_FS_Range, cMma9553_CMD_Response, respBuf);
    // CHECK - Check FS_RANGE value
    TEST_ASSERT_EQUAL_HEX(FS_RANGE_TESTVAL, respBuf[MMA9553_HDR_SIZE]);

    // Read the SAMPLE_RATE value and check with reference test value.
    status = MMA9553_I2C_CommandResponse(&gMma9553Driver, cMma9553_Get_Sample_Rate, cMma9553_CMD_Response, respBuf);
    // CHECK - Check SAMPLE_RATE value
    TEST_ASSERT_EQUAL_HEX(SAMPLE_RATE_TESTVAL, respBuf[MMA9553_HDR_SIZE]);

    // Read the AFE_PRIORITY value and check with reference test value.
    status = MMA9553_I2C_CommandResponse(&gMma9553Driver, cMma9553_Get_AFE_Priority, cMma9553_CMD_Response, respBuf);
    // CHECK - Check AFE_PRIORITY value
    TEST_ASSERT_EQUAL_HEX(AFE_PRIORITY_TESTVAL, respBuf[MMA9553_HDR_SIZE]);

    // Read the MBox_PRIORITY value and check with reference test value.
    status = MMA9553_I2C_CommandResponse(&gMma9553Driver, cMma9553_Get_MBox_Priority, cMma9553_CMD_Response, respBuf);
    // CHECK - Check MBox_PRIORITY value
    TEST_ASSERT_EQUAL_HEX(MBox_PRIORITY_TESTVAL, respBuf[MMA9553_HDR_SIZE]);

    PRINTF("\r\n \r\n");
}

/*! -----------------------------------------------------------------------
 *  @brief       This is the MMA9553 Sensor Driver Negative 4 Test Function.
 *  @details     Checks:
 *                  Return status of the sensor driver functions, when applied
 *                  Bad Read command List
 *  @param[in]   void This is no input parameter.
 *  @return      void  There is no return value.
 *  @constraints This should be the sixth testfunction to be executed after test_mma9553_config().
 *  @reeentrant  No
 *  -----------------------------------------------------------------------*/
void test_mma9553_negative_4(void)
{
    int32_t status;
    uint8_t cmdResp;
    const registerreadlist_t *badRegReadList = NULL;

    PRINTF("\r\n \r\n");
    PRINTF("\r\n---------------------------------------------------------\r\n");
    gTestCounter++;
    PRINTF("Test %d: MMA9553 Sensor Driver Negative 4 Test\r\n", gTestCounter);

    // Reading sensor data with bad read register list
    status = MMA9553_I2C_CommandResponse(&gMma9553Driver, NULL, badRegReadList, &cmdResp);

    // CHECK - Check Sensor Read Status
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_INVALID_PARAM, status);

    PRINTF("\r\n \r\n");
}

/*! -----------------------------------------------------------------------
 *  @brief       This is the MMA9553 Sensor Driver deInit Test Function.
 *  @details     Checks:
 *                  Return status of the sensor driver deInit function,
 *                  State of the sensor after deInit
 *  @param[in]   void This is no input parameter.
 *  @return      void  There is no return value.
 *  @constraints This should be the seventh testfunction to be executed after test_mma9553_negative_4().
 *  @reeentrant  No
 *  -----------------------------------------------------------------------*/
void test_mma9553_deInit(void)
{
    int32_t status;

    PRINTF("\r\n \r\n");
    PRINTF("\r\n---------------------------------------------------------\r\n");
    gTestCounter++;
    PRINTF("Test %d: MMA9553 Sensor Driver deInit Test\r\n", gTestCounter);

    // End the MMA9553 sensor driver sampling.
    status = MMA9553_I2C_DeInit(&gMma9553Driver);

    PRINTF("\r\nStart performing sensor state parameters check\r\n");
    // CHECK - Check Sensor Driver De-Init Success/Failure
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_NONE, status);

    // CHECK - Check Sensor State
    TEST_ASSERT_EQUAL_INT(false, gMma9553Driver.isInitialized);

    PRINTF("\r\n \r\n");
}

/* TearDown Required for test framework. */
void tearDown(void)
{
}

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
