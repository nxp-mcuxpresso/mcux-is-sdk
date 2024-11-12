/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*! File: test_diff_p_spi.c
* @brief The test_diff_p_spi.c file contains unittest for DIFF-P SPI sensor
*        functional interfaces.
*/

/* Test Scope of the DIFF-P unit tests:
*           (1) To test DIFF-P driver functional interfaces, sensor i/o
*              interfaces and register i/o interfaces.
*           (2) To make sure that the code coverage for DIFF_P driver
*               functional interfaces, sensor i/o interfaces and register i/o
*               interfaces is > 85%.
*/

/*  SDK Includes */
#include "board.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_debug_console.h"

/* CMSIS Includes */
#include "Driver_SPI.h"

/* ISSDK Includes */
#include "issdk_hal.h"
#include "diff_p_drv.h"
#include "gpio_driver.h"
#include "systick_utils.h"

/* Unity Includes */
#include "unity.h"

/*******************************************************************************
 * Macro Definitions
 ******************************************************************************/
/* Reference Test Values */
/*---------------------------------------------------------------------------*/
/*! @def    CTRLREG1_TESTVAL
 *  @brief  Test DIFF_P CTRL_REG1 Value: mode = ACTIVE and OSR = 512.
 */
#define CTRLREG1_TESTVAL (DIFF_P_CTRL_REG1_OSR_OSR512 | DIFF_P_CTRL_REG1_SBYB_ACTIVE)
/*---------------------------------------------------------------------------*/
/*! @def    CTRLREG2_TESTVAL
 *  @brief  Test DIFF_P CTRL_REG2 Value: ODR = 6.25Hz
 */
#define CTRLREG2_TESTVAL (DIFF_P_CTRL_REG2_ODR_ODR6P25)

/*---------------------------------------------------------------------------*/
/*! @def    CTRLREG3_TESTVAL
 *  @brief  Test DIFF_P CTRL_REG3 Value: IPOL = Active High.
 */
#define CTRLREG3_TESTVAL (DIFF_P_CTRL_REG3_IPOL1_ACTIVE_HIGH)

/*---------------------------------------------------------------------------*/
/*! @def    INTRMASK0_TESTVAL
 *  @brief  Test DIFF_P INT_MASK0 Value: Pressure Data Ready INT = ENABLED,
 *          Temperature Data Ready INT = ENABLED.
 */
#define INTMASK0_TESTVAL (DIFF_P_INT_MASK0_TDR_INT_EN | DIFF_P_INT_MASK0_PDR_INT_EN)

/*******************************************************************************
 * Constants
 ******************************************************************************/
/*! @brief Register settings for Normal (non buffered) mode. */
const registerwritelist_t cDiffPConfigNormal[] = {
    {DIFF_P_CTRL_REG3, DIFF_P_CTRL_REG3_IPOL1_ACTIVE_HIGH, DIFF_P_CTRL_REG3_IPOL1_MASK},
    {DIFF_P_CTRL_REG2, DIFF_P_CTRL_REG2_ODR_ODR6P25, DIFF_P_CTRL_REG2_ODR_MASK},
    {DIFF_P_CTRL_REG1, DIFF_P_CTRL_REG1_OSR_OSR512, DIFF_P_CTRL_REG1_OSR_MASK},
    {DIFF_P_INT_MASK0, DIFF_P_INT_MASK0_TDR_INT_EN | DIFF_P_INT_MASK0_PDR_INT_EN,
     DIFF_P_INT_MASK0_TDR_MASK | DIFF_P_INT_MASK0_PDR_MASK},
    __END_WRITE_DATA__};

/*! Prepare the register read list to read DIFF_P WHOAMI value. */
const registerreadlist_t diff_p_WHO_AM_I[] = {{.readFrom = DIFF_P_WHO_AM_I, .numBytes = 1}, __END_READ_DATA__};

/*******************************************************************************
 * Reference Test Registers List
 ******************************************************************************/
/*! Read the DIFF_P_CTRL_REG1 value. */
const registerreadlist_t diff_p_CTRL_REG1[] = {{.readFrom = DIFF_P_CTRL_REG1, .numBytes = 1}, __END_READ_DATA__};

/*! Read the DIFF_P_CTRL_REG2 value. */
const registerreadlist_t diff_p_CTRL_REG2[] = {{.readFrom = DIFF_P_CTRL_REG2, .numBytes = 1}, __END_READ_DATA__};

/*! Read the DIFF_P_CTRL_REG3 value. */
const registerreadlist_t diff_p_CTRL_REG3[] = {{.readFrom = DIFF_P_CTRL_REG3, .numBytes = 1}, __END_READ_DATA__};

/*! Read the DIFF_P_INT_MASK0 value. */
const registerreadlist_t diff_p_INT_MASK0[] = {{.readFrom = DIFF_P_INT_MASK0, .numBytes = 1}, __END_READ_DATA__};

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
static int test_counter = 0;
diff_p_spi_sensorhandle_t gDiff_pDriver;
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

/*! @brief       DIFF_P Sensor Driver Negative Test 1 Function.
 *  @details     Test function to negatively test DIFF_P sensor driver
 *               functional interfaces by applying corrupted SPI driver metadata.
 *               Checks:
 *               return status of the sensor driver functions.
 *               sensor handle state,
 */
void test_diff_p_negative_1(void)
{
    int32_t status;
    uint8_t whoAmI;
    ARM_DRIVER_SPI *corruptedSPIdriver = NULL;

    PRINTF(" \r\n \r\n");
    PRINTF("\r\n---------------------------------------------------------\r\n");
    test_counter++;
    PRINTF("Test %d: DIFF_P Sensor Driver Negative Test 1\r\n", test_counter);

    // Initialize the DIFF_P sensor driver.
    PRINTF("\r\nInitialize DIFF_P Sensor with corrupted SPI driver metadata\r\n");
    status = DIFF_P_SPI_Initialize(&gDiff_pDriver, corruptedSPIdriver, SPI_S_DEVICE_INDEX, &DIFF_P_CS,
                                   DIFF_P_NPS3000VV_WHOAMI_VALUE);

    PRINTF("\r\nCheck error returned by Sensor Init and sensor handle state\r\n");
    // CHECK - Check Sensor Driver Init Success/Failure
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_INVALID_PARAM, status);

    // CHECK - Check Sensor handle
    TEST_ASSERT_EQUAL_INT(false, gDiff_pDriver.isInitialized);

    // Reading sensor data with bad read register list
    PRINTF("\r\nRead DIFF_P data.\r\n");
    status = DIFF_P_SPI_ReadData(&gDiff_pDriver, diff_p_WHO_AM_I, &whoAmI);

    PRINTF("\r\nCheck error returned by Sensor Read\r\n");
    // CHECK - Check Sensor Read Status
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_INIT, status);

    // Configure the DIFF_P sensor driver with FIFO mode.
    PRINTF("\r\nTry to configure DIFF_P.\r\n");
    status = DIFF_P_SPI_Configure(&gDiff_pDriver, cDiffPConfigNormal);

    PRINTF("\r\nCheck error returned by Sensor Configure\r\n");
    // CHECK - Check Sensor Driver Init Success/Failure
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_INIT, status);

    PRINTF("\r\n \r\n");
}

/*! @brief       DIFF_P Sensor Driver Negative Test 2 Function.
 *  @details     Test function to negatively test DIFF_P sensor driver
 *               functional interfaces by applying bad SPI address.
 *               Checks:
 *               return status of the sensor driver functions.
 *               sensor handle state,
 */
void test_diff_p_negative_2(void)
{
    int32_t status;
    void *badSlaveHandle = NULL;

    PRINTF(" \r\n \r\n");
    PRINTF("\r\n---------------------------------------------------------\r\n");
    test_counter++;
    PRINTF("Test %d: DIFF_P Sensor Driver Init Negative Test 2\r\n", test_counter);

    // Initialize the DIFF_P sensor driver.
    PRINTF("\r\nInitialize DIFF_P Sensor with bad SPI slave address\r\n");
    status = DIFF_P_SPI_Initialize(&gDiff_pDriver, &SPI_S_DRIVER, SPI_S_DEVICE_INDEX, badSlaveHandle,
                                   DIFF_P_NPS3000VV_WHOAMI_VALUE);

    PRINTF("\r\nCheck error returned by Sensor Init and sensor handle state\r\n");
    // CHECK - Check Sensor Driver Init Success/Failure
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_INVALID_PARAM, status);

    // CHECK - Check Sensor handle
    TEST_ASSERT_EQUAL_INT(false, gDiff_pDriver.isInitialized);

    // Configure the DIFF_P sensor driver with INT mode.
    PRINTF("\r\nTry to Configure DIFF_P.\r\n");
    status = DIFF_P_SPI_Configure(&gDiff_pDriver, cDiffPConfigNormal);

    PRINTF("\r\nCheck error returned by Sensor Configure\r\n");
    // CHECK - Check Sensor Driver Init Success/Failure
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_INIT, status);

    PRINTF("\r\n \r\n");
}
/*! @brief       DIFF_P Sensor Driver Init Test Function.
 *  @details     Test function to test DIFF_P sensor driver Init interface.
 *               Checks:
 *               return status of the sensor driver init function,
 *               sensor handle state after successful init,
 *               who_am_i value of the sensor.
 */
void test_diff_p_init(void)
{
    int32_t status;

    PRINTF("\r\n---------------------------------------------------------\r\n");
    test_counter++;
    PRINTF("\r\nTest %d: DIFF_P Sensor Driver Init Test\r\n", test_counter);

    // Initialize the DIFF_P sensor driver.
    PRINTF("\r\nInitialize DIFF_P Sensor\r\n");
    status = DIFF_P_SPI_Initialize(&gDiff_pDriver, &SPI_S_DRIVER, SPI_S_DEVICE_INDEX, &DIFF_P_CS,
                                   DIFF_P_NPS3000VV_WHOAMI_VALUE);

    PRINTF("\r\nCheck DIFF_P Init parameters\r\n");
    // CHECK - Check Sensor Driver Init Success/Failure
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_NONE, status);

    // CHECK - Check Sensor handle
    TEST_ASSERT_EQUAL_INT(true, gDiff_pDriver.isInitialized);

    PRINTF("\r\n \r\n");
}

/*! @brief       DIFF_P Sensor Driver Negative Test 3 Function.
 *  @details     Test function to negatively test DIFF_P sensor driver
 *               functional interfaces by applying bad Write command List.
 *               Checks:
 *               return status of the sensor driver functions.
 *               sensor handle state,
 */
void test_diff_p_negative_3(void)
{
    int32_t status;
    const registerwritelist_t *badRegWriteList = NULL;

    PRINTF(" \r\n \r\n");
    PRINTF("\r\n---------------------------------------------------------\r\n");
    test_counter++;
    PRINTF("Test %d: DIFF_P Sensor Driver Negative Test 3\r\n", test_counter);

    // Configure the DIFF_P sensor driver.
    PRINTF("\r\nConfigure DIFF_P Sensor with bad register write list\r\n");
    status = DIFF_P_SPI_Configure(&gDiff_pDriver, badRegWriteList);

    PRINTF("\r\nCheck error returned by Sensor Configure\r\n");
    // CHECK - Check Sensor Driver Config Success/Failure
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_INVALID_PARAM, status);

    PRINTF("\r\n \r\n");
}

/*! @brief       DIFF_P Sensor Driver Config Test Function.
 *  @details     Test function to test DIFF_P sensor driver Config interface.
 *               Checks:
 *               return status of the sensor driver config function,
 *               sensor handle state,
 *               ctrl_reg1 value configured for the sensor,
 *               ctrl_reg2 value configured for the sensor,
 *               ctrl_reg3 value configured for the sensor,
 *               f_setup value configured for the sensor.
 */
void test_diff_p_config(void)
{
    int32_t status;
    uint8_t ctrlReg1;
    uint8_t ctrlReg2;
    uint8_t ctrlReg3;
    uint8_t intMask0;

    PRINTF(" \n \r\n");
    PRINTF("\r\n---------------------------------------------------------\r\n");
    test_counter++;
    PRINTF("Test %d: DIFF_P Sensor Driver Config Test\r\n", test_counter);

    // Configure the DIFF_P sensor driver with FIFO mode.
    PRINTF("\r\nConfigure DIFF_P Sensor\r\n");
    status = DIFF_P_SPI_Configure(&gDiff_pDriver, cDiffPConfigNormal);

    PRINTF("\r\nCheck DIFF_P configured parameters\r\n");
    // CHECK - Check Sensor Driver Config Success/Failure
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_NONE, status);

    // CHECK - Check Sensor handle
    TEST_ASSERT_EQUAL_INT(true, gDiff_pDriver.isInitialized);

    // Read the CTRL_REG1 value and check with reference test value.
    status = DIFF_P_SPI_ReadData(&gDiff_pDriver, diff_p_CTRL_REG1, &ctrlReg1);

    // CHECK - Check CTRL_REG1 value
    TEST_ASSERT_EQUAL_HEX(CTRLREG1_TESTVAL, ctrlReg1);

    status = DIFF_P_SPI_ReadData(&gDiff_pDriver, diff_p_CTRL_REG2, &ctrlReg2);

    // CHECK - Check CTRL_REG2 value
    TEST_ASSERT_EQUAL_HEX(CTRLREG2_TESTVAL, ctrlReg2);

    // Read the CTRL_REG3 value and check with reference test value.
    status = DIFF_P_SPI_ReadData(&gDiff_pDriver, diff_p_CTRL_REG3, &ctrlReg3);
    // CHECK - Check CTRL_REG3 value
    TEST_ASSERT_EQUAL_HEX(CTRLREG3_TESTVAL, ctrlReg3);

    // Read the INT_MASK0 value and check with reference test value.
    status = DIFF_P_SPI_ReadData(&gDiff_pDriver, diff_p_INT_MASK0, &intMask0);
    // CHECK - Check INT_MASK0 value
    TEST_ASSERT_EQUAL_HEX(INTMASK0_TESTVAL, intMask0);

    PRINTF("\r\n \r\n");
}

/*! @brief       DIFF_P Sensor Driver Negative Test 4 Function.
 *  @details     Test function to negatively test DIFF_P sensor driver
 *               functional interfaces by applying bad read command List.
 *               Checks:
 *               return status of the sensor driver functions.
 *               sensor handle state,
 */
void test_diff_p_negative_4(void)
{
    int32_t status;
    uint8_t whoAmI;
    const registerreadlist_t *badRegReadList = NULL;

    PRINTF(" \r\n \r\n");
    PRINTF("\r\n---------------------------------------------------------\r\n");
    test_counter++;
    PRINTF("Test %d: DIFF_P Sensor Driver Negative Test 4\r\n", test_counter);

    // Reading sensor data with bad read register list
    PRINTF("\r\nTry to read DIFF_P Sensor data bad register read list\r\n");
    status = DIFF_P_SPI_ReadData(&gDiff_pDriver, badRegReadList, &whoAmI);

    // CHECK - Check Sensor Read Status
    PRINTF("\r\nCheck error returned by Sensor Read\r\n");
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_INVALID_PARAM, status);

    PRINTF("\r\n \r\n");
}
/*! @brief       DIFF_P Sensor Driver deInit Test Function.
 *  @details     Test function to test DIFF_P sensor driver deInit interface.
 *               Checks:
 *               return status of the sensor driver deInit function,
 *               sensor handle state after successful deInit,
 */
void test_diff_p_deInit(void)
{
    int32_t status;

    PRINTF(" \r\n \r\n");
    PRINTF("\r\n---------------------------------------------------------\r\n");
    test_counter++;
    PRINTF("Test %d: DIFF_P Sensor Driver endData Test\r\n", test_counter);

    // End the DIFF_P sensor driver sampling.
    PRINTF("\r\nDeInit DIFF_P Sensor\r\n");
    status = DIFF_P_SPI_DeInit(&gDiff_pDriver);

    PRINTF("\r\nCheck sensor handle state after deInit\r\n");
    // CHECK - Check Sensor Driver Init Success/Failure
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_NONE, status);

    // CHECK - Check Sensor handle
    TEST_ASSERT_EQUAL_INT(false, gDiff_pDriver.isInitialized);

    PRINTF("\r\n \r\n");
}

void tearDown(void)
{
    // TearDown Required for test framework.
}
