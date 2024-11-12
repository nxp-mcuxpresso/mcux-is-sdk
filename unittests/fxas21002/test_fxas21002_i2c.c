/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*! File: test_fxas21002_i2c.c
* @brief The \b test_fxas21002_i2c.c file contains unittest for fxas21002 sensor
*        functional interfaces.
*/

/* Test Scope of the FXAS21002 unit tests:
*           (1) To test FXAS21002 driver functional interfaces, sensor i/o
*              interfaces and register i/o interfaces.
*           (2) To make sure that the code coverage for FXAS21002 driver
*               functional interfaces, sensor i/o interfaces and register i/o
*               interfaces is > 85%.
*/

/*  SDK Includes */
#include "board.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_debug_console.h"

/* CMSIS Includes */
#include "Driver_I2C.h"

/* ISSDK Includes */
#include "issdk_hal.h"
#include "fxas21002_drv.h"

/* Unity Includes */
#include "unity.h"

/*******************************************************************************
 * Macro Definitions
 ******************************************************************************/
/*! @def    FIFO_SIZE
*  @brief  The watermark value configured for FXAS21002 FIFO Buffer.
*/
#define FIFO_SIZE 16 // Must be between 1 - 32

/* Reference Test Values */
/*---------------------------------------------------------------------------*/
/*! @def    CTRLREG1_TESTVAL
 *  @brief  Test FXAS21002 CTRL_REG1 Value: mode = ACTIVE and odr = 12.5Hz.
 */
#define CTRLREG1_TESTVAL (FXAS21002_CTRL_REG1_MODE_ACTIVE | FXAS21002_CTRL_REG1_DR_12_5HZ)
/*---------------------------------------------------------------------------*/
/*! @def    CTRLREG2_TESTVAL
 *  @brief  Test FXAS21002 CTRL_REG2 Value: Interrupt polarity = HIGH
 *          Data ready interrupt Enable, Interrupt is routed to INT1 pin.
 */
#define CTRLREG2_TESTVAL                                                             \
    (FXAS21002_CTRL_REG2_IPOL_ACTIVE_HIGH | FXAS21002_CTRL_REG2_INT_EN_DRDY_ENABLE | \
     FXAS21002_CTRL_REG2_INT_CFG_FIFO_INT1)
/*---------------------------------------------------------------------------*/
/*! @def    CTRLREG3_TESTVAL
 *  @brief  Test FXAS21002 CTRL_REG3 Value:
 *          Auto-increment address pointer rolls over to address 0x01.
 */
#define CTRLREG3_TESTVAL (FXAS21002_CTRL_REG3_WRAPTOONE_ROLL_DATA)
/*---------------------------------------------------------------------------*/
/*! @def    FSETUP_TESTVAL
 *  @brief  Test FXAS21002 F_SETUP Value: FIFO mode = STOP,
 *          watermark size = FIFO_SIZE.
 */
#define FSETUP_TESTVAL (FXAS21002_F_SETUP_F_MODE_STOP_MODE | FIFO_SIZE)

/*******************************************************************************
 * Constants
 ******************************************************************************/
/*! Prepare the register write list to configure FXAS21002 in FIFO mode. */
const registerwritelist_t fxas21002_Config_with_Fifo[] = {
    /*! Configure CTRL_REG1 register to set 12.5Hz sampling rate. */
    {FXAS21002_CTRL_REG1, FXAS21002_CTRL_REG1_DR_12_5HZ, 0},
    /*! Configure F_SETUP register to set FIFO in Stop mode, Watermark of 16. */
    {FXAS21002_F_SETUP, FXAS21002_F_SETUP_F_MODE_STOP_MODE | FIFO_SIZE, 0},
    /*! Configure CTRL_REG2 register to set interrupt configuration settings. */
    {FXAS21002_CTRL_REG2, FXAS21002_CTRL_REG2_IPOL_ACTIVE_HIGH | FXAS21002_CTRL_REG2_INT_EN_DRDY_ENABLE |
                              FXAS21002_CTRL_REG2_INT_CFG_FIFO_INT1,
     0},
    /*! Configure CTRL_REG3 register Auto-increment address configuration: pointer rolls over to address 0x01*/
    {FXAS21002_CTRL_REG3, FXAS21002_CTRL_REG3_WRAPTOONE_ROLL_DATA, 0},
    __END_WRITE_DATA__};

/*! Prepare the register read list to read FXAS21002 WHOAMI value. */
const registerreadlist_t fxas21002_WHO_AM_I[] = {{.readFrom = FXAS21002_WHO_AM_I, .numBytes = 1}, __END_READ_DATA__};

/*******************************************************************************
 * Reference Test Registers List
 ******************************************************************************/
/*! Read the FXAS21002_CTRL_REG1 value. */
const registerreadlist_t fxas21002_CTRL_REG1[] = {{.readFrom = FXAS21002_CTRL_REG1, .numBytes = 1}, __END_READ_DATA__};

/*! Read the FXAS21002_CTRL_REG2 value. */
const registerreadlist_t fxas21002_CTRL_REG2[] = {{.readFrom = FXAS21002_CTRL_REG2, .numBytes = 1}, __END_READ_DATA__};

/*! Read the FXAS21002_CTRL_REG3 value. */
const registerreadlist_t fxas21002_CTRL_REG3[] = {{.readFrom = FXAS21002_CTRL_REG3, .numBytes = 1}, __END_READ_DATA__};

/*! Read the FXAS21002_F_SETUP value. */
const registerreadlist_t fxas21002_F_SETUP[] = {{.readFrom = FXAS21002_F_SETUP, .numBytes = 1}, __END_READ_DATA__};

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
static int test_counter = 0;
fxas21002_i2c_sensorhandle_t FXAS21002drv;
ARM_DRIVER_I2C *I2Cdrv = &I2C_S_DRIVER; // Now using the shield.h value!!!

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

/*! @brief       FXAS21002 Sensor Driver Init Test Function.
 *  @details     Test function to test FXAS21002 sensor driver Init interface.
 *               Checks:
 *               return status of the sensor driver init function,
 *               sensor handle state after successful init,
 *               who_am_i value of the sensor.
 */
void test_fxas21002_init(void)
{
    int32_t status;

    PRINTF("\r\n---------------------------------------------------------\r\n");
    test_counter++;
    PRINTF("\r\nTest %d: FXAS21002 Sensor Driver Init Test\r\n", test_counter);

    // Set the I2C bus speed.
    I2Cdrv->Control(ARM_I2C_BUS_SPEED, ARM_I2C_BUS_SPEED_FAST);

    // Initialize the FXAS21002 sensor driver.
    PRINTF("\r\nInitialize FXAS21002 Sensor\r\n");
    status = FXAS21002_I2C_Initialize(&FXAS21002drv, &I2C_S_DRIVER, I2C_S_DEVICE_INDEX, FXAS21002_I2C_ADDR,
                                      FXAS21002_WHO_AM_I_WHOAMI_PROD_VALUE);

    PRINTF("\r\nCheck FXAS21002 Init parameters\r\n");
    // CHECK - Check Sensor Driver Init Success/Failure
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_NONE, status);

    // CHECK - Check Sensor handle
    TEST_ASSERT_EQUAL_INT(true, FXAS21002drv.isInitialized);

    PRINTF("\r\n \r\n");
}

/*! @brief       FXAS21002 Sensor Driver Config Test Function.
 *  @details     Test function to test FXAS21002 sensor driver Config interface.
 *               Checks:
 *               return status of the sensor driver config function,
 *               sensor handle state,
 *               ctrl_reg1 value configured for the sensor,
 *               ctrl_reg2 value configured for the sensor,
 *               ctrl_reg3 value configured for the sensor,
 *               f_setup value configured for the sensor.
 */
void test_fxas21002_config(void)
{
    int32_t status;
    uint8_t ctrlReg1;
    uint8_t ctrlReg2;
    uint8_t ctrlReg3;
    uint8_t fSetup;

    PRINTF(" \n \r\n");
    PRINTF("\r\n---------------------------------------------------------\r\n");
    test_counter++;
    PRINTF("Test %d: FXAS21002 Sensor Driver Config Test\r\n", test_counter);

    // Configure the FXAS21002 sensor driver with FIFO mode.
    PRINTF("\r\nConfigure FXAS21002 Sensor\r\n");
    status = FXAS21002_I2C_Configure(&FXAS21002drv, fxas21002_Config_with_Fifo);

    PRINTF("\r\nCheck FXAS21002 configured parameters\r\n");
    // CHECK - Check Sensor Driver Config Success/Failure
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_NONE, status);

    // CHECK - Check Sensor handle
    TEST_ASSERT_EQUAL_INT(true, FXAS21002drv.isInitialized);

    // Read the CTRL_REG1 value and check with reference test value.
    status = FXAS21002_I2C_ReadData(&FXAS21002drv, fxas21002_CTRL_REG1, &ctrlReg1);

    // CHECK - Check CTRL_REG1 value
    TEST_ASSERT_EQUAL_HEX(CTRLREG1_TESTVAL, ctrlReg1);

    status = FXAS21002_I2C_ReadData(&FXAS21002drv, fxas21002_CTRL_REG2, &ctrlReg2);

    // CHECK - Check CTRL_REG2 value
    TEST_ASSERT_EQUAL_HEX(CTRLREG2_TESTVAL, ctrlReg2);

    // Read the CTRL_REG3 value and check with reference test value.
    status = FXAS21002_I2C_ReadData(&FXAS21002drv, fxas21002_CTRL_REG3, &ctrlReg3);
    // CHECK - Check CTRL_REG3 value
    TEST_ASSERT_EQUAL_HEX(CTRLREG3_TESTVAL, ctrlReg3);

    // Read the F_SETUP value and check with reference test value.
    status = FXAS21002_I2C_ReadData(&FXAS21002drv, fxas21002_F_SETUP, &fSetup);
    // CHECK - Check F_SETUP value
    TEST_ASSERT_EQUAL_HEX(FSETUP_TESTVAL, fSetup);

    PRINTF("\r\n \r\n");
}

/*! @brief       FXAS21002 Sensor Driver deInit Test Function.
 *  @details     Test function to test FXAS21002 sensor driver deInit interface.
 *               Checks:
 *               return status of the sensor driver deInit function,
 *               sensor handle state after successful deInit,
 */
void test_fxas21002_deInit(void)
{
    int32_t status;

    PRINTF(" \r\n \r\n");
    PRINTF("\r\n---------------------------------------------------------\r\n");
    test_counter++;
    PRINTF("Test %d: FXAS21002 Sensor Driver endData Test\r\n", test_counter);

    // End the FXAS21002 sensor driver sampling.
    PRINTF("\r\nDeInit FXAS21002 Sensor\r\n");
    status = FXAS21002_I2C_Deinit(&FXAS21002drv);

    PRINTF("\r\nCheck sensor handle state after deInit\r\n");
    // CHECK - Check Sensor Driver Init Success/Failure
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_NONE, status);

    // CHECK - Check Sensor handle
    TEST_ASSERT_EQUAL_INT(false, FXAS21002drv.isInitialized);

    PRINTF("\r\n \r\n");
}

/*! @brief       FXAS21002 Sensor Driver Negative Test 1 Function.
 *  @details     Test function to negatively test FXAS21002 sensor driver
 *               functional interfaces by applying corrupted I2C driver metadata.
 *               Checks:
 *               return status of the sensor driver functions.
 *               sensor handle state,
 */
void test_fxas21002_negative_1(void)
{
    int32_t status;
    uint8_t whoAmI;
    ARM_DRIVER_I2C *corruptedI2Cdriver = NULL;

    PRINTF(" \r\n \r\n");
    PRINTF("\r\n---------------------------------------------------------\r\n");
    test_counter++;
    PRINTF("Test %d: FXAS21002 Sensor Driver Negative Test 1\r\n", test_counter);

    // Initialize the FXAS21002 sensor driver.
    PRINTF("\r\nInitialize FXAS21002 Sensor with corrupted I2C driver metadata\r\n");
    status = FXAS21002_I2C_Initialize(&FXAS21002drv, corruptedI2Cdriver, I2C_S_DEVICE_INDEX, FXAS21002_I2C_ADDR,
                                      FXAS21002_WHO_AM_I_WHOAMI_PROD_VALUE);

    PRINTF("\r\nCheck error returned by Sensor Init and sensor handle state\r\n");
    // CHECK - Check Sensor Driver Init Success/Failure
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_INVALID_PARAM, status);

    // CHECK - Check Sensor handle
    TEST_ASSERT_EQUAL_INT(false, FXAS21002drv.isInitialized);

    // Reading sensor data with bad read register list
    PRINTF("\r\nRead FXAS21002 data.\r\n");
    status = FXAS21002_I2C_ReadData(&FXAS21002drv, fxas21002_WHO_AM_I, &whoAmI);

    PRINTF("\r\nCheck error returned by Sensor Read\r\n");
    // CHECK - Check Sensor Read Status
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_INIT, status);

    // Configure the FXAS21002 sensor driver with FIFO mode.
    PRINTF("\r\nTry to configure FXAS21002.\r\n");
    status = FXAS21002_I2C_Configure(&FXAS21002drv, fxas21002_Config_with_Fifo);

    PRINTF("\r\nCheck error returned by Sensor Configure\r\n");
    // CHECK - Check Sensor Driver Init Success/Failure
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_INIT, status);

    // De-Initialize the FXAS21002 sensor driver.
    PRINTF("\r\nTry to DeInit FXAS21002.\r\n");
    status = FXAS21002_I2C_Deinit(&FXAS21002drv);

    PRINTF("\r\nCheck error returned by Sensor DeInit\r\n");
    // CHECK - Check Sensor De-Init Status
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_INIT, status);

    PRINTF("\r\n \r\n");
}

/*! @brief       FXAS21002 Sensor Driver Negative Test 2 Function.
 *  @details     Test function to negatively test FXAS21002 sensor driver
 *               functional interfaces by applying bad I2C address.
 *               Checks:
 *               return status of the sensor driver functions.
 *               sensor handle state,
 */
void test_fxas21002_negative_2(void)
{
    int32_t status;
    uint8_t whoAmI;
    uint16_t badSlaveaddr = 0x21;

    PRINTF(" \r\n \r\n");
    PRINTF("\r\n---------------------------------------------------------\r\n");
    test_counter++;
    PRINTF("Test %d: FXAS21002 Sensor Driver Init Negative Test 2\r\n", test_counter);

    // Initialize the FXAS21002 sensor driver.
    PRINTF("\r\nInitialize FXAS21002 Sensor with bad I2C slave address\r\n");
    status = FXAS21002_I2C_Initialize(&FXAS21002drv, &I2C_S_DRIVER, I2C_S_DEVICE_INDEX, badSlaveaddr,
                                      FXAS21002_WHO_AM_I_WHOAMI_PROD_VALUE);

    PRINTF("\r\nCheck error returned by Sensor Init and sensor handle state\r\n");
    // CHECK - Check Sensor Driver Init Success/Failure
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_INIT, status);

    // Read the WHO_AM_I value to make sure we are talking to the FXAS21002.
    PRINTF("\r\nTry to Read FXAS21002 WHOAMI.\r\n");
    status = FXAS21002_I2C_ReadData(&FXAS21002drv, fxas21002_WHO_AM_I, &whoAmI);

    PRINTF("\r\nCheck WHOAMI returned by Sensor Read\r\n");
    // CHECK - Check Sensor Driver whoami
    TEST_ASSERT_NOT_EQUAL(FXAS21002_WHO_AM_I_WHOAMI_PROD_VALUE, whoAmI);

    // Configure the FXAS21002 sensor driver with FIFO mode.
    PRINTF("\r\nTry to Configure FXAS21002.\r\n");
    status = FXAS21002_I2C_Configure(&FXAS21002drv, fxas21002_Config_with_Fifo);

    PRINTF("\r\nCheck error returned by Sensor Configure\r\n");
    // CHECK - Check Sensor Driver Init Success/Failure
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_INIT, status);

    PRINTF("\r\n \r\n");
}

/*! @brief       FXAS21002 Sensor Driver Negative Test 3 Function.
 *  @details     Test function to negatively test FXAS21002 sensor driver
 *               functional interfaces by applying bad Write command List.
 *               Checks:
 *               return status of the sensor driver functions.
 *               sensor handle state,
 */
void test_fxas21002_negative_3(void)
{
    int32_t status;
    const registerwritelist_t *badRegWriteList = NULL;

    PRINTF(" \r\n \r\n");
    PRINTF("\r\n---------------------------------------------------------\r\n");
    test_counter++;
    PRINTF("Test %d: FXAS21002 Sensor Driver Negative Test 3\r\n", test_counter);

    // Initialize the FXAS21002 sensor driver.
    PRINTF("\r\nInitialize FXAS21002 Sensor\r\n");
    status = FXAS21002_I2C_Initialize(&FXAS21002drv, &I2C_S_DRIVER, I2C_S_DEVICE_INDEX, FXAS21002_I2C_ADDR,
                                      FXAS21002_WHO_AM_I_WHOAMI_PROD_VALUE);

    // Configure the FXAS21002 sensor driver.
    PRINTF("\r\nConfigure FXAS21002 Sensor with bad register write list\r\n");
    status = FXAS21002_I2C_Configure(&FXAS21002drv, badRegWriteList);

    PRINTF("\r\nCheck error returned by Sensor Configure\r\n");
    // CHECK - Check Sensor Driver Config Success/Failure
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_INVALID_PARAM, status);
}

/*! @brief       FXAS21002 Sensor Driver Negative Test 4 Function.
 *  @details     Test function to negatively test FXAS21002 sensor driver
 *               functional interfaces by applying bad read command List.
 *               Checks:
 *               return status of the sensor driver functions.
 *               sensor handle state,
 */
void test_fxas21002_negative_4(void)
{
    int32_t status;
    uint8_t whoAmI;
    const registerreadlist_t *badRegReadList = NULL;

    PRINTF(" \r\n \r\n");
    PRINTF("\r\n---------------------------------------------------------\r\n");
    test_counter++;
    PRINTF("Test %d: FXAS21002 Sensor Driver Negative Test 3\r\n", test_counter);

    // Initialize the FXAS21002 sensor driver.
    PRINTF("\r\nInitialize FXAS21002 Sensor\r\n");
    status = FXAS21002_I2C_Initialize(&FXAS21002drv, &I2C_S_DRIVER, I2C_S_DEVICE_INDEX, FXAS21002_I2C_ADDR,
                                      FXAS21002_WHO_AM_I_WHOAMI_PROD_VALUE);

    // Reading sensor data with bad read register list
    PRINTF("\r\nTry to read FXAS21002 Sensor data bad register read list\r\n");
    status = FXAS21002_I2C_ReadData(&FXAS21002drv, badRegReadList, &whoAmI);

    // CHECK - Check Sensor Read Status
    PRINTF("\r\nCheck error returned by Sensor Read\r\n");
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_INVALID_PARAM, status);
}

void tearDown(void)
{
    // TearDown Required for test framework.
}
