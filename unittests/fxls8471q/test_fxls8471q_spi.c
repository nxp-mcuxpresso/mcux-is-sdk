/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*! File: test_fxls8471q.c
* @brief The \b test_fxls8471q.c file contains unittest for fxls8471q sensor
*        functional interfaces.
*/

/* Test Scope of the FXLS8471Q unit tests:
*           (1) To test FXLS8471Q driver functional interfaces, sensor i/o
*              interfaces and register i/o interfaces.
*           (2) To make sure that the code coverage for FXLS8471Q driver
*               functional interfaces, sensor i/o interfaces and register i/o
*               interfaces is > 85%.
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
#include "fxls8471q_drv.h"

/* Unity Includes */
#include "unity.h"

/*******************************************************************************
 * Macro Definitions
 ******************************************************************************/
/*! @def    FIFO_SIZE
*  @brief  The watermark value configured for FXLS8471Q FIFO Buffer.
*/
#define FIFO_SIZE 16 // Must be between 1 - 32

/* Reference Test Values */
/*---------------------------------------------------------------------------*/
/*! @def    CTRLREG1_TESTVAL
 *  @brief  Test FXLS8471Q CTRL_REG1 Value: mode = ACTIVE and odr = 12.5Hz.
 */
#define CTRLREG1_TESTVAL (FXLS8471Q_CTRL_REG1_MODE_ACTIVE | FXLS8471Q_CTRL_REG1_DR_12DOT5HZ)
/*---------------------------------------------------------------------------*/
/*! @def    CTRLREG2_TESTVAL
 *  @brief  Test FXLS8471Q CTRL_REG2 Value: Interrupt polarity = HIGH
 *          Data ready interrupt Enable, Interrupt is routed to INT1 pin.
 */
#define CTRLREG2_TESTVAL (FXLS8471Q_CTRL_REG2_MODS_LOWNOISE | FXLS8471Q_CTRL_REG2_SMODS_HIGHRES)
/*---------------------------------------------------------------------------*/
/*! @def    CTRLREG3_TESTVAL
 *  @brief  Test FXLS8471Q CTRL_REG3 Value:
 *          Auto-increment address pointer rolls over to address 0x01.
 */
#define CTRLREG3_TESTVAL (FXLS8471Q_CTRL_REG3_IPOL_HIGH | FXLS8471Q_CTRL_REG3_WAKE_PULSE_WAKEUP)
/*---------------------------------------------------------------------------*/
/*! @def    FSETUP_TESTVAL
 *  @brief  Test FXLS8471Q F_SETUP Value: FIFO mode = STOP,
 *          watermark size = FIFO_SIZE.
 */
#define FSETUP_TESTVAL (FXLS8471Q_F_SETUP_F_MODE_FIFOSTOP | FIFO_SIZE)

/*******************************************************************************
 * Constants
 ******************************************************************************/
/*! Prepare the register write list to configure FXLS8471Q in FIFO mode. */
const registerwritelist_t fxls8471q_Config_with_Fifo[] = {
    /*! Configure CTRL_REG1 register to set 12.5Hz sampling rate. */
    {FXLS8471Q_CTRL_REG1, FXLS8471Q_CTRL_REG1_DR_12DOT5HZ, 0},
    /*! Configure F_SETUP register to set FIFO in Stop mode, Watermark of 16*/
    {FXLS8471Q_F_SETUP, FXLS8471Q_F_SETUP_F_MODE_FIFOSTOP | (FIFO_SIZE << FXLS8471Q_F_SETUP_F_WMRK_SHIFT), 0},
    __END_WRITE_DATA__};

/*! Prepare the register read list to read FXLS8471Q WHOAMI value. */
const registerreadlist_t fxls8471q_WHO_AM_I[] = {{.readFrom = FXLS8471Q_WHO_AM_I, .numBytes = 1}, __END_READ_DATA__};

/*******************************************************************************
 * Reference Test Registers List
 ******************************************************************************/
/*! Read the FXLS8471Q_CTRL_REG1 value. */
const registerreadlist_t fxls8471q_CTRL_REG1[] = {{.readFrom = FXLS8471Q_CTRL_REG1, .numBytes = 1}, __END_READ_DATA__};

/*! Read the FXLS8471Q_CTRL_REG2 value. */
const registerreadlist_t fxls8471q_CTRL_REG2[] = {{.readFrom = FXLS8471Q_CTRL_REG2, .numBytes = 1}, __END_READ_DATA__};

/*! Read the FXLS8471Q_CTRL_REG3 value. */
const registerreadlist_t fxls8471q_CTRL_REG3[] = {{.readFrom = FXLS8471Q_CTRL_REG3, .numBytes = 1}, __END_READ_DATA__};

/*! Read the FXLS8471Q_F_SETUP value. */
const registerreadlist_t fxls8471q_F_SETUP[] = {{.readFrom = FXLS8471Q_F_SETUP, .numBytes = 1}, __END_READ_DATA__};

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
static int test_counter = 0;
fxls8471q_spi_sensorhandle_t fxls8471qDriver;
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

/*! @brief       FXLS8471Q Sensor Driver Init Test Function.
 *  @details     Test function to test FXLS8471Q sensor driver Init interface.
 *               Checks:
 *               return status of the sensor driver init function,
 *               sensor handle state after successful init,
 *               who_am_i value of the sensor.
 */
void test_fxls8471q_init(void)
{
    int32_t status;

    PRINTF("\r\n---------------------------------------------------------\r\n");
    test_counter++;
    PRINTF("\r\nTest %d: FXLS8471Q Sensor Driver Init Test\r\n", test_counter);

    // Initialize the FXLS8471Q sensor driver.
    PRINTF("\r\nInitialize FXLS8471Q Sensor\r\n");
    status = FXLS8471Q_SPI_Initialize(&fxls8471qDriver, &SPI_S_DRIVER, SPI_S_DEVICE_INDEX, &FXLS8471_SPI_CS,
                                      FXLS8471Q_WHO_AM_I_WHOAMI_VALUE);

    PRINTF("\r\nCheck FXLS8471Q Init parameters\r\n");
    // CHECK - Check Sensor Driver Init Success/Failure
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_NONE, status);

    // CHECK - Check Sensor handle
    TEST_ASSERT_EQUAL_INT(true, fxls8471qDriver.isInitialized);

    PRINTF("\r\n \r\n");
}

/*! @brief       FXLS8471Q Sensor Driver Config Test Function.
 *  @details     Test function to test FXLS8471Q sensor driver Config interface.
 *               Checks:
 *               return status of the sensor driver config function,
 *               sensor handle state,
 *               ctrl_reg1 value configured for the sensor,
 *               ctrl_reg2 value configured for the sensor,
 *               ctrl_reg3 value configured for the sensor,
 *               f_setup value configured for the sensor.
 */
void test_fxls8471q_config(void)
{
    int32_t status;
    uint8_t ctrlReg1;
    uint8_t fSetup;

    PRINTF(" \n \r\n");
    PRINTF("\r\n---------------------------------------------------------\r\n");
    test_counter++;
    PRINTF("Test %d: FXLS8471Q Sensor Driver Config Test\r\n", test_counter);

    // Configure the FXLS8471Q sensor driver with FIFO mode.
    PRINTF("\r\nConfigure FXLS8471Q Sensor\r\n");
    status = FXLS8471Q_SPI_Configure(&fxls8471qDriver, fxls8471q_Config_with_Fifo);

    PRINTF("\r\nCheck FXLS8471Q configured parameters\r\n");
    // CHECK - Check Sensor Driver Config Success/Failure
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_NONE, status);

    // CHECK - Check Sensor handle
    TEST_ASSERT_EQUAL_INT(true, fxls8471qDriver.isInitialized);

    // Read the CTRL_REG1 value and check with reference test value.
    status = FXLS8471Q_SPI_ReadData(&fxls8471qDriver, fxls8471q_CTRL_REG1, &ctrlReg1);

    TEST_ASSERT_EQUAL_HEX(CTRLREG1_TESTVAL, ctrlReg1);
    // Read the F_SETUP value and check with reference test value.
    status = FXLS8471Q_SPI_ReadData(&fxls8471qDriver, fxls8471q_F_SETUP, &fSetup);
    // CHECK - Check F_SETUP value
    TEST_ASSERT_EQUAL_HEX(FSETUP_TESTVAL, fSetup);

    PRINTF("\r\n \r\n");
}

/*! @brief       FXLS8471Q Sensor Driver deInit Test Function.
 *  @details     Test function to test FXLS8471Q sensor driver deInit interface.
 *               Checks:
 *               return status of the sensor driver deInit function,
 *               sensor handle state after successful deInit,
 */
void test_fxls8471q_deInit(void)
{
    int32_t status;

    PRINTF(" \r\n \r\n");
    PRINTF("\r\n---------------------------------------------------------\r\n");
    test_counter++;
    PRINTF("Test %d: FXLS8471Q Sensor Driver endData Test\r\n", test_counter);

    // End the FXLS8471Q sensor driver sampling.
    PRINTF("\r\nDeInit FXLS8471Q Sensor\r\n");
    status = FXLS8471Q_SPI_Deinit(&fxls8471qDriver);

    PRINTF("\r\nCheck sensor handle state after deInit\r\n");
    // CHECK - Check Sensor Driver Init Success/Failure
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_NONE, status);

    // CHECK - Check Sensor handle
    TEST_ASSERT_EQUAL_INT(false, fxls8471qDriver.isInitialized);

    PRINTF("\r\n \r\n");
}

/*! @brief       FXLS8471Q Sensor Driver Negative Test 1 Function.
 *  @details     Test function to negatively test FXLS8471Q sensor driver
 *               functional interfaces by applying corrupted SPI driver metadata.
 *               Checks:
 *               return status of the sensor driver functions.
 *               sensor handle state,
 */

void test_fxls8471q_negative_1(void)
{
    int32_t status;
    uint8_t whoAmI;
    ARM_DRIVER_SPI *corruptedSPIdriver = NULL;

    PRINTF(" \r\n \r\n");
    PRINTF("\r\n---------------------------------------------------------\r\n");
    test_counter++;
    PRINTF("Test %d: FXLS8471Q Sensor Driver Negative Test 1\r\n", test_counter);

    // Initialize the FXLS8471Q sensor driver.
    PRINTF("\r\nInitialize FXLS8471Q Sensor with corrupted SPI driver metadata\r\n");
    status = FXLS8471Q_SPI_Initialize(&fxls8471qDriver, corruptedSPIdriver, SPI_S_DEVICE_INDEX, &FXLS8471_SPI_CS,
                                      FXLS8471Q_WHO_AM_I_WHOAMI_VALUE);

    PRINTF("\r\nCheck error returned by Sensor Init and sensor handle state\r\n");
    // CHECK - Check Sensor Driver Init Success/Failure
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_INVALID_PARAM, status);

    // CHECK - Check Sensor handle
    TEST_ASSERT_EQUAL_INT(false, fxls8471qDriver.isInitialized);

    // Reading sensor data with bad read register list
    PRINTF("\r\nRead FXLS8471Q data.\r\n");
    status = FXLS8471Q_SPI_ReadData(&fxls8471qDriver, fxls8471q_WHO_AM_I, &whoAmI);

    PRINTF("\r\nCheck error returned by Sensor Read\r\n");
    // CHECK - Check Sensor Read Status
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_INIT, status);

    // Configure the FXLS8471Q sensor driver with FIFO mode.
    PRINTF("\r\nTry to configure FXLS8471Q.\r\n");
    status = FXLS8471Q_SPI_Configure(&fxls8471qDriver, fxls8471q_Config_with_Fifo);

    PRINTF("\r\nCheck error returned by Sensor Configure\r\n");
    // CHECK - Check Sensor Driver Init Success/Failure
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_INIT, status);

    // De-Initialize the FXLS8471Q sensor driver.
    PRINTF("\r\nTry to DeInit FXLS8471Q.\r\n");
    status = FXLS8471Q_SPI_Deinit(&fxls8471qDriver);

    PRINTF("\r\nCheck error returned by Sensor DeInit\r\n");
    // CHECK - Check Sensor De-Init Status
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_INIT, status);

    PRINTF("\r\n \r\n");
}

/*! @brief       FXLS8471Q Sensor Driver Negative Test 2 Function.
 *  @details     Test function to negatively test FXLS8471Q sensor driver
 *               functional interfaces by applying bad SPI address.
 *               Checks:
 *               return status of the sensor driver functions.
 *               sensor handle state,
 */

void test_fxls8471q_negative_2(void)
{
    int32_t status;
    uint8_t whoAmI;

    PRINTF(" \r\n \r\n");
    PRINTF("\r\n---------------------------------------------------------\r\n");
    test_counter++;
    PRINTF("Test %d: FXLS8471Q Sensor Driver Init Negative Test 2\r\n", test_counter);

    // Initialize the FXLS8471Q sensor driver.
    PRINTF("\r\nInitialize FXLS8471Q Sensor with bad SPI slave address\r\n");
    status = FXLS8471Q_SPI_Initialize(&fxls8471qDriver, &SPI_S_DRIVER, SPI_S_DEVICE_INDEX, NULL,
                                      FXLS8471Q_WHO_AM_I_WHOAMI_VALUE);

    PRINTF("\r\nCheck error returned by Sensor Init and sensor handle state\r\n");
    // CHECK - Check Sensor Driver Init Success/Failure
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_INVALID_PARAM, status);

    // Read the WHO_AM_I value to make sure we are talking to the FXLS8471Q.
    PRINTF("\r\nTry to Read FXLS8471Q WHOAMI.\r\n");
    status = FXLS8471Q_SPI_ReadData(&fxls8471qDriver, fxls8471q_WHO_AM_I, &whoAmI);

    PRINTF("\r\nCheck WHOAMI returned by Sensor Read\r\n");
    // CHECK - Check Sensor Driver whoami
    TEST_ASSERT_NOT_EQUAL(FXLS8471Q_WHO_AM_I_WHOAMI_VALUE, whoAmI);

    // Configure the FXLS8471Q sensor driver with FIFO mode.
    PRINTF("\r\nTry to Configure FXLS8471Q.\r\n");
    status = FXLS8471Q_SPI_Configure(&fxls8471qDriver, fxls8471q_Config_with_Fifo);

    PRINTF("\r\nCheck error returned by Sensor Configure\r\n");
    // CHECK - Check Sensor Driver Init Success/Failure
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_INIT, status);

    PRINTF("\r\n \r\n");
}

/*! @brief       FXLS8471Q Sensor Driver Negative Test 3 Function.
 *  @details     Test function to negatively test FXLS8471Q sensor driver
 *               functional interfaces by applying bad Write command List.
 *               Checks:
 *               return status of the sensor driver functions.
 *               sensor handle state,
 */

void test_fxls8471q_negative_3(void)
{
    int32_t status;
    const registerwritelist_t *badRegWriteList = NULL;

    PRINTF(" \r\n \r\n");
    PRINTF("\r\n---------------------------------------------------------\r\n");
    test_counter++;
    PRINTF("Test %d: FXLS8471Q Sensor Driver Negative Test 3\r\n", test_counter);

    // Initialize the FXLS8471Q sensor driver.
    PRINTF("\r\nInitialize FXLS8471Q Sensor\r\n");
    status = FXLS8471Q_SPI_Initialize(&fxls8471qDriver, &SPI_S_DRIVER, SPI_S_DEVICE_INDEX, &FXLS8471_SPI_CS,
                                      FXLS8471Q_WHO_AM_I_WHOAMI_VALUE);

    // Configure the FXLS8471Q sensor driver.
    PRINTF("\r\nConfigure FXLS8471Q Sensor with bad register write list\r\n");
    status = FXLS8471Q_SPI_Configure(&fxls8471qDriver, badRegWriteList);

    PRINTF("\r\nCheck error returned by Sensor Configure\r\n");
    // CHECK - Check Sensor Driver Config Success/Failure
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_INVALID_PARAM, status);
}

/*! @brief       FXLS8471Q Sensor Driver Negative Test 4 Function.
 *  @details     Test function to negatively test FXLS8471Q sensor driver
 *               functional interfaces by applying bad read command List.
 *               Checks:
 *               return status of the sensor driver functions.
 *               sensor handle state,
 */

void test_fxls8471q_negative_4(void)
{
    int32_t status;
    uint8_t whoAmI;
    const registerreadlist_t *badRegReadList = NULL;

    PRINTF(" \r\n \r\n");
    PRINTF("\r\n---------------------------------------------------------\r\n");
    test_counter++;
    PRINTF("Test %d: FXLS8471Q Sensor Driver Negative Test 4\r\n", test_counter);

    // Initialize the FXLS8471Q sensor driver.
    PRINTF("\r\nInitialize FXLS8471Q Sensor\r\n");
    status = FXLS8471Q_SPI_Initialize(&fxls8471qDriver, &SPI_S_DRIVER, SPI_S_DEVICE_INDEX, &FXLS8471_SPI_CS,
                                      FXLS8471Q_WHO_AM_I_WHOAMI_VALUE);

    // Reading sensor data with bad read register list
    PRINTF("\r\nTry to read FXLS8471Q Sensor data bad register read list\r\n");
    status = FXLS8471Q_SPI_ReadData(&fxls8471qDriver, badRegReadList, &whoAmI);

    // CHECK - Check Sensor Read Status
    PRINTF("\r\nCheck error returned by Sensor Read\r\n");
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_INVALID_PARAM, status);
}

void tearDown(void)
{
    // TearDown Required for test framework.
}
