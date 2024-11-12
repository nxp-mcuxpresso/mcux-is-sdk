/*
 * Copyright (c) 2015 - 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*! @file  test_fxlc95000.c
 *  @brief This is the unit test file for fxlc95000 sensor driver.
 *         The scope of the unit tests is:
 *             1) To verify functionality of each of the driver APIs.
 *             2) Ensure maximun code coverage at the end of unit test runs.
 *             3) To verify handling of negative scenarios by returning appropriate return codes.
 */

//-----------------------------------------------------------------------
// CMSIS Includes
//-----------------------------------------------------------------------
#include "Driver_SPI.h"

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
#include "gpio_driver.h"
#include "systick_utils.h"
#include "fxlc95000_drv.h"

//-----------------------------------------------------------------------
// Unity Includes
//-----------------------------------------------------------------------
#include "unity.h"

//-----------------------------------------------------------------------
// Macros
//-----------------------------------------------------------------------
/* Configuration Values */
#define FXLC95000_ACCEL_DATA_SIZE (10) /* 4 bytes timestamp + 2 byte each of X,Y,Z Data. */
#define FXLC95000_ODR_RATE 100000      /* 100,000 us */
/* Reference Values */
// Test FXLC95000 FS_RANGE        Value: 4G.
#define FS_RANGE_TESTVAL (0x80)
// Test FXLC95000 SAMPLE_RATE     Value: 10Hz (LE --> BE).
#define SAMPLE_RATE_TESTVAL                                                                                         \
    (FXLC95000_ODR_RATE & 0xFF) << 24 | (FXLC95000_ODR_RATE & 0xFF00) << 8 | (FXLC95000_ODR_RATE & 0xFF0000) >> 8 | \
        (FXLC95000_ODR_RATE & 0xFF000000) >> 24
// Test FXLC95000 RESOLUTION      Value: 14-Bit.
#define RESOLUTION_TESTVAL (0x04)

//-----------------------------------------------------------------------
// Constants
//-----------------------------------------------------------------------
/*! Create commands for fetching FXLC95000L requested configuration. */
const uint8_t GetFSRange[] = {0x02, 0x01, 0x08, 0x01};
const uint8_t GetSampleRate[] = {0x02, 0x01, 0x03, 0x04};
const uint8_t GetResolution[] = {0x02, 0x01, 0x07, 0x01};

/*! Create commands for setting FXLC95000L desired configuration. */
const uint8_t SetSampleRate_10Hz[] = {FXLC95000_SET_ODR_CMD_HDR,
                                      FXLC95000_SST_ODR_PAYLOAD(100000)}; /* ODR equal to 10Hz. */
const uint8_t SetResolution_14Bit[] = {FXLC95000_SET_RESOLUTION_CMD_HDR,
                                       FXLC95000_ACCEL_RESOLUTION_14_BIT};               /* Resolution 14-bits. */
const uint8_t SetFSRange_4g[] = {FXLC95000_SET_RANGE_CMD_HDR, FXLC95000_ACCEL_RANGE_4G}; /* FS Range 4G. */

/*! Prepare the register write list to configure FXLC95000L in Normal mode. */
const registercommandlist_t cFxlc95000ConfigNormal[] = {{SetFSRange_4g, 0, sizeof(SetFSRange_4g)},
                                                        {SetSampleRate_10Hz, 0, sizeof(SetSampleRate_10Hz)},
                                                        {SetResolution_14Bit, 0, sizeof(SetResolution_14Bit)},
                                                        __END_WRITE_CMD__};

/*! Prepare the register write list to fetch FXLC95000L FS Range. */
const registercommandlist_t cFxlc95000_Get_FS_Range[] = {{GetFSRange, 0, sizeof(GetFSRange)}, /* Get FS Range */
                                                         __END_WRITE_CMD__};

/*! Prepare the register write list to fetch FXLC95000L Sample Rate. */
const registercommandlist_t cFxlc95000_Get_Sample_Rate[] = {
    {GetSampleRate, 0, sizeof(GetSampleRate)}, /* Get Sample Rate */
    __END_WRITE_CMD__};

/*! Prepare the register write list to fetch FXLC95000L Resolution. */
const registercommandlist_t cFxlc95000_Get_Resolution[] = {
    {GetResolution, 0, sizeof(GetResolution)}, /* Get Resolution */
    __END_WRITE_CMD__};

/* 1-Byte payload Response. */
const registerreadlist_t cFxlc95000_CMD_Response1B[] = {{.readFrom = FXLC95000_HDR_SIZE, .numBytes = 1},
                                                        __END_READ_DATA__};

/* 4-Byte payload Response. */
const registerreadlist_t cFxlc95000_CMD_Response4B[] = {{.readFrom = FXLC95000_HDR_SIZE, .numBytes = 4},
                                                        __END_READ_DATA__};

//-----------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------
static int gTestCounter = 0;
fxlc95000_spi_sensorhandle_t gFxlc95000Driver;
ARM_DRIVER_SPI *pSPIdriver = &SPI_S_DRIVER;

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

/*! -----------------------------------------------------------------------
 *  @brief       This is the FXLC95000 Sensor Driver Negative 1 Test Function.
 *  @details     Checks:
 *                  Return status of the sensor driver functions, when applied
 *                  Corrupted SPI driver metadata
 *  @param[in]   void This is no input parameter.
 *  @return      void  There is no return value.
 *  @constraints This should be the first testfunction to be executed after setUp().
 *  @reeentrant  No
 *  -----------------------------------------------------------------------*/
void test_fxlc95000_negative_1(void)
{
    uint8_t cmdResp;
    int32_t status;
    ARM_DRIVER_SPI *pCorruptedSPIdriver = NULL;

    PRINTF("\r\n \r\n");
    PRINTF("\r\n---------------------------------------------------------\r\n");
    gTestCounter++;
    PRINTF("Test %d: FXLC95000 Sensor Driver Negative 1 Test.\r\n", gTestCounter);

    // Initialize the FXLC95000 sensor driver.
    status = FXLC95000_SPI_Initialize(&gFxlc95000Driver, pCorruptedSPIdriver, SPI_S_DEVICE_INDEX, &FXLC95000_PDB_B,
                                      &FXLC95000_SSB_IO3, &FXLC95000_RST_GPIO, FXLC95000_BUILD_ID);

    PRINTF("\r\nStart performing init parameters check\r\n");
    // CHECK - Check Sensor Driver Init Success/Failure
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_INVALID_PARAM, status);

    // CHECK - Check Sensor handle
    TEST_ASSERT_EQUAL_INT(false, gFxlc95000Driver.isInitialized);

    // Reading sensor data with bad read register list
    status = FXLC95000_SPI_CommandResponse(&gFxlc95000Driver, NULL, cFxlc95000_CMD_Response1B, &cmdResp);

    // CHECK - Check Sensor Read Status
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_INIT, status);

    // Configure the FXLC95000 sensor driver with FIFO mode.
    status = FXLC95000_SPI_CommandResponse(&gFxlc95000Driver, cFxlc95000ConfigNormal, NULL, NULL);

    PRINTF("\r\nStart performing sensor state parameters check\r\n");
    // CHECK - Check Sensor Driver Init Success/Failure
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_INIT, status);

    PRINTF("\r\n \r\n");
}

/*! -----------------------------------------------------------------------
 *  @brief       This is the FXLC95000 Sensor Driver Negative 2 Test Function.
 *  @details     Checks:
 *                  Return status of the sensor driver functions, when applied
 *                  Bad SPI address
 *  @param[in]   void This is no input parameter.
 *  @return      void  There is no return value.
 *  @constraints This should be the second testfunction to be executed after test_fxlc95000_negative_1().
 *  @reeentrant  No
 *  -----------------------------------------------------------------------*/
void test_fxlc95000_negative_2(void)
{
    int32_t status;
    void *badSlaveHandle = NULL;

    PRINTF("\r\n \r\n");
    PRINTF("\r\n---------------------------------------------------------\r\n");
    gTestCounter++;
    PRINTF("Test %d: FXLC95000 Sensor Driver Init Negative 2 Test.\r\n", gTestCounter);

    // Initialize the FXLC95000 sensor driver.
    status = FXLC95000_SPI_Initialize(&gFxlc95000Driver, &SPI_S_DRIVER, SPI_S_DEVICE_INDEX, &FXLC95000_PDB_B,
                                      badSlaveHandle, &FXLC95000_RST_GPIO, FXLC95000_BUILD_ID);

    // CHECK - Check Sensor Driver Init Success/Failure
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_INVALID_PARAM, status);

    // CHECK - Check Sensor handle
    TEST_ASSERT_EQUAL_INT(false, gFxlc95000Driver.isInitialized);

    /* Configure device to verify Register_Write failure.*/
    status = FXLC95000_SPI_CommandResponse(&gFxlc95000Driver, cFxlc95000ConfigNormal, NULL, NULL);

    // CHECK - Check Sensor read status.
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_INIT, status);

    PRINTF("\r\n \r\n");
}

/*! -----------------------------------------------------------------------
 *  @brief       This is the FXLC95000 Sensor Driver Init Test Function.
 *  @details     Checks:
 *                  Return status of the sensor driver init function.
 *                  Who_Am_I of the sensor.
 *  @param[in]   void This is no input parameter.
 *  @return      void  There is no return value.
 *  @constraints This should be the third testfunction to be executed after test_fxlc95000_negative_2().
 *  @reeentrant  No
 *  -----------------------------------------------------------------------*/
void test_fxlc95000_init(void)
{
    int32_t status;

    PRINTF("\r\n---------------------------------------------------------\r\n");
    gTestCounter++;
    PRINTF("\r\nTest %d: FXLC95000 Sensor Driver Init Test\r\n", gTestCounter);

    // Initialize FXLC95000 sensor driver.
    status = FXLC95000_SPI_Initialize(&gFxlc95000Driver, &SPI_S_DRIVER, SPI_S_DEVICE_INDEX, &FXLC95000_PDB_B,
                                      &FXLC95000_SSB_IO3, &FXLC95000_RST_GPIO, FXLC95000_BUILD_ID);

    PRINTF("\r\nStart performing init parameters check\r\n");
    // CHECK - Check Sensor Driver Init Success/Failure
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_NONE, status);

    // CHECK - Check Sensor State
    TEST_ASSERT_EQUAL_INT(true, gFxlc95000Driver.isInitialized);

    PRINTF("\r\n \r\n");
}

/*! -----------------------------------------------------------------------
 *  @brief       This is the FXLC95000 Sensor Driver Negative 3 Test Function.
 *  @details     Checks:
 *                  Return status of the sensor driver functions, when applied
 *                  Bad Write command List
 *  @param[in]   void This is no input parameter.
 *  @return      void  There is no return value.
 *  @constraints This should be the fourth testfunction to be executed after test_fxlc95000_init().
 *  @reeentrant  No
 *  -----------------------------------------------------------------------*/
void test_fxlc95000_negative_3(void)
{
    int32_t status;
    const registercommandlist_t *pBadRegCmdList = NULL;

    PRINTF("\r\n \r\n");
    PRINTF("\r\n---------------------------------------------------------\r\n");
    gTestCounter++;
    PRINTF("Test %d: FXLC95000 Sensor Driver Negative 3 Test.\r\n", gTestCounter);

    // Configure the FXLC95000 sensor driver with FIFO mode.
    status = FXLC95000_SPI_CommandResponse(&gFxlc95000Driver, pBadRegCmdList, NULL, NULL);

    PRINTF("\r\nStart performing sensor state parameters check\r\n");
    // CHECK - Check Sensor Driver Config Success/Failure
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_INVALID_PARAM, status);

    PRINTF("\r\n \r\n");
}

/*! -----------------------------------------------------------------------
 *  @brief       This is the FXLC95000 Sensor Driver Config Test Function.
 *  @details     Checks:
 *                  Return status of the sensor driver config function,
 *                  FS_RANGE       value configured for the sensor.
 *                  SAMPLE_RATE    value configured for the sensor.
 *                  AFE_PRIORITY   value configured for the sensor.
 *                  MBox_PRIORITY  value configured for the sensor.
 *  @param[in]   void This is no input parameter.
 *  @return      void  There is no return value.
 *  @constraints This should be the fifth testfunction to be executed after test_fxlc95000_negative_3().
 *  @reeentrant  No
 *  -----------------------------------------------------------------------*/
void test_fxlc95000_config(void)
{
    uint8_t resp1B;
    int32_t status;
    uint32_t resp4B;

    PRINTF("\r\n \r\n");
    PRINTF("\r\n---------------------------------------------------------\r\n");
    gTestCounter++;
    PRINTF("Test %d: FXLC95000 Sensor Driver Config Test\r\n", gTestCounter);

    // Configure the FXLC95000 sensor driver with FIFO mode.
    status = FXLC95000_SPI_CommandResponse(&gFxlc95000Driver, cFxlc95000ConfigNormal, NULL, NULL);

    PRINTF("\r\nStart performing FXLC95000 configured parameters check\r\n");
    // CHECK - Check Sensor Driver Config Success/Failure
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_NONE, status);

    // CHECK - Check Sensor handle
    TEST_ASSERT_EQUAL_INT(true, gFxlc95000Driver.isInitialized);

    // Read the FS_RANGE value and check with reference test value.
    FXLC95000_SPI_CommandResponse(&gFxlc95000Driver, cFxlc95000_Get_FS_Range, cFxlc95000_CMD_Response1B, &resp1B);
    // CHECK - Check FS_RANGE value
    TEST_ASSERT_EQUAL_HEX(FS_RANGE_TESTVAL, resp1B);

    // Read the RESOLUTION value and check with reference test value.
    status =
        FXLC95000_SPI_CommandResponse(&gFxlc95000Driver, cFxlc95000_Get_Resolution, cFxlc95000_CMD_Response1B, &resp1B);
    // CHECK - Check RESOLUTION value
    TEST_ASSERT_EQUAL_HEX(RESOLUTION_TESTVAL, resp1B);

    // Read the SAMPLE_RATE value and check with reference test value.
    status = FXLC95000_SPI_CommandResponse(&gFxlc95000Driver, cFxlc95000_Get_Sample_Rate, cFxlc95000_CMD_Response4B,
                                           (uint8_t *)&resp4B);
    // CHECK - Check SAMPLE_RATE value
    TEST_ASSERT_EQUAL_HEX(SAMPLE_RATE_TESTVAL, resp4B);

    PRINTF("\r\n \r\n");
}

/*! -----------------------------------------------------------------------
 *  @brief       This is the FXLC95000 Sensor Driver Negative 4 Test Function.
 *  @details     Checks:
 *                  Return status of the sensor driver functions, when applied
 *                  Bad Read Buffer.
 *  @param[in]   void This is no input parameter.
 *  @return      void  There is no return value.
 *  @constraints This should be the sixth testfunction to be executed after test_fxlc95000_config().
 *  @reeentrant  No
 *  -----------------------------------------------------------------------*/
void test_fxlc95000_negative_4(void)
{
    int32_t status;
    uint8_t *badReadBuffer = NULL;

    PRINTF("\r\n \r\n");
    PRINTF("\r\n---------------------------------------------------------\r\n");
    gTestCounter++;
    PRINTF("Test %d: FXLC95000 Sensor Driver Negative 4 Test\r\n", gTestCounter);

    // Reading sensor data with bad read register list
    status = FXLC95000_SPI_CommandResponse(&gFxlc95000Driver, NULL, cFxlc95000_CMD_Response1B, badReadBuffer);

    // CHECK - Check Sensor Read Status
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_READ, status);

    PRINTF("\r\n \r\n");
}

/* TearDown Required for test framework. */
void tearDown(void)
{
}

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
