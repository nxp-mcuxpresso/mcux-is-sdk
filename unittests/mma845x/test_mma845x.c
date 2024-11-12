/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*! File: test_mma845x.c
* @brief The \b test_mma845x.c file contains unittest for mma845x sensor
*        functional interfaces.
*/

/* Test Scope of the MMA845x unit tests:
*           (1) To test MMA845x driver functional interfaces, sensor i/o
*              interfaces and register i/o interfaces.
*           (2) To make sure that the code coverage for MMA845x driver
*               functional interfaces, sensor i/o interfaces and register i/o
*               interfaces is > 85%.
*/

/* CMSIS Includes */
#include "Driver_I2C.h"

/*  SDK Includes */
#include "board.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "fsl_debug_console.h"

/* ISSDK Includes */
#include "issdk_hal.h"
#include "mma845x_drv.h"

/* Unity Includes */
#include "unity.h"

/*******************************************************************************
 * Macro Definitions
 ******************************************************************************/
/*! @def    FIFO_SIZE
*  @brief  The watermark value configured for MMA845x FIFO Buffer.
*/
#define FIFO_SIZE 16 // Must be between 1 - 32

/* Reference Test Values */
/*---------------------------------------------------------------------------*/
/*! @def    CTRLREG1_TESTVAL
 *  @brief  Test MMA845x CTRL_REG1 Value: mode = ACTIVE and odr = 12.5Hz.
 */
#define CTRLREG1_TESTVAL (MMA845x_CTRL_REG1_MODE_ACTIVE | MMA845x_CTRL_REG1_DR_12DOT5HZ)
/*---------------------------------------------------------------------------*/
/*! @def    CTRLREG2_TESTVAL
 *  @brief  Test MMA845x CTRL_REG2 Value: Oversampling mode = High Resolution.
 */
#define CTRLREG2_TESTVAL (MMA845x_CTRL_REG2_SMODS_HIGHRES)
/*---------------------------------------------------------------------------*/
/*! @def    CTRLREG3_TESTVAL
 *  @brief  Test MMA845x CTRL_REG3 Value: Interrupt polarity = ACTIVE high.
 */
#define CTRLREG3_TESTVAL (MMA845x_CTRL_REG3_IPOL_HIGH)
/*---------------------------------------------------------------------------*/
/*! @def    CTRLREG4_TESTVAL
 *  @brief  Test MMA845x CTRL_REG4 Value: The data ready interrupt is enabled.
 */
#define CTRLREG4_TESTVAL (MMA845x_CTRL_REG4_INT_EN_DRDY_ENABLED)
/*---------------------------------------------------------------------------*/
/*! @def    CTRLREG5_TESTVAL
*  @brief  Test MMA845x CTRL_REG5 Value: Route Interrupt to pin = INT1.
 */
#define CTRLREG5_TESTVAL (MMA845x_CTRL_REG5_INT_CFG_DRDY_INT1)
/*---------------------------------------------------------------------------*/
/*! @def    FSETUP_TESTVAL
 *  @brief  Test MMA845x F_SETUP Value: Watermark size = FIFO_SIZE.
 */
#define FSETUP_TESTVAL (FIFO_SIZE)

/*******************************************************************************
 * Global Variables
 ******************************************************************************/
static int test_counter = 0;
mma845x_i2c_sensorhandle_t MMA845xdrv;
ARM_DRIVER_I2C *I2Cdrv = &I2C_S_DRIVER; // Now using the shield.h value!!!

/*******************************************************************************
 * Constants
 ******************************************************************************/
/*! Prepare the register write list to configure MMA845x in FIFO and ISR mode. */
const registerwritelist_t mma845x_Config_FifoIsr[] = {
    /*! Configure the MMA845x CTRL_REG1 to set mode to STANDBY and odr to 12.5Hz. */
    {MMA845x_CTRL_REG1, MMA845x_CTRL_REG1_DR_12DOT5HZ, 0},
    /*! Configure the MMA845x F_SETUP to set FIFO mode to STOP, set the watermark size to FIFO_SIZE. */
    {MMA845x_F_SETUP, FIFO_SIZE, 0},
    /*! Configure the MMA845x CTRL_REG2 to set the Oversampling mode to High Resolution. */
    {MMA845x_CTRL_REG2, MMA845x_CTRL_REG2_SMODS_HIGHRES, 0},
    /*! Configure the MMA845x CTRL_REG3 to set the Interrupt polarity to ACTIVE high. */
    {MMA845x_CTRL_REG3, MMA845x_CTRL_REG3_IPOL_HIGH, 0},
    /*! Configure the MMA845x CTRL_REG4 to enable the data ready interrupt. */
    {MMA845x_CTRL_REG4, MMA845x_CTRL_REG4_INT_EN_DRDY_ENABLED, 0},
    /*! Configure the MMA845x CTRL_REG5 to route Interrupt to INT1 pin. */
    {MMA845x_CTRL_REG5, MMA845x_CTRL_REG5_INT_CFG_DRDY_INT1, 0},
    __END_WRITE_DATA__};

/*! Prepare the register read list to read MMA845x WHOAMI value. */
const registerreadlist_t mma845x_WHO_AM_I[] = {{.readFrom = MMA845x_WHO_AM_I, .numBytes = 1}, __END_READ_DATA__};

/*******************************************************************************
 * Test Registers List
 ******************************************************************************/
/*! Read the MMA845x_CTRL_REG1 value to check against reference test value. */
const registerreadlist_t mma845x_CTRL_REG1[] = {{.readFrom = MMA845x_CTRL_REG1, .numBytes = 1}, __END_READ_DATA__};

/*! Read the MMA845x_CTRL_REG2 value to check against reference test value. */
const registerreadlist_t mma845x_CTRL_REG2[] = {{.readFrom = MMA845x_CTRL_REG2, .numBytes = 1}, __END_READ_DATA__};

/*! Read the MMA845x_CTRL_REG3 value to check against reference test value. */
const registerreadlist_t mma845x_CTRL_REG3[] = {{.readFrom = MMA845x_CTRL_REG3, .numBytes = 1}, __END_READ_DATA__};

/*! Read the MMA845x_CTRL_REG4 value to check against reference test value. */
const registerreadlist_t mma845x_CTRL_REG4[] = {{.readFrom = MMA845x_CTRL_REG4, .numBytes = 1}, __END_READ_DATA__};

/*! Read the MMA845x_CTRL_REG5 value to check against reference test value. */
const registerreadlist_t mma845x_CTRL_REG5[] = {{.readFrom = MMA845x_CTRL_REG5, .numBytes = 1}, __END_READ_DATA__};

/*! Read the MMA845x_F_SETUP value to check against reference test value. */
const registerreadlist_t mma845x_F_SETUP[] = {{.readFrom = MMA845x_F_SETUP, .numBytes = 1}, __END_READ_DATA__};

/*******************************************************************************
 * Code
 ******************************************************************************/
/*! SetUp Required for test framework. */
void setUp(void)
{
    int32_t status;

    /*!Initialize the MCU hardware */
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

/*! @brief       MMA845x Sensor Driver Init Test Function.
 *  @details     Test function to test MMA845x sensor driver Init interface.
 *               Checks:
 *               return status of the sensor driver init function,
 *               sensor handle state after successful init,
 *               who_am_i value of the sensor.
 */
void test_mma845x_init(void)
{
    int32_t status;

    PRINTF("\r\n---------------------------------------------------------\r\n");
    test_counter++;
    PRINTF("\r\nTest %d: MMA845x Sensor Driver Init Test\r\n", test_counter);

    /*! Initialize the MMA845x sensor driver. */
    PRINTF("\r\nInitialize MMA845x Sensor\r\n");
    status = MMA845x_I2C_Initialize(&MMA845xdrv, &I2C_S_DRIVER, I2C_S_DEVICE_INDEX, MMA845x_I2C_ADDR,
                                    MMA8451_WHO_AM_I_WHOAMI_VALUE);

    PRINTF("\r\nCheck MMA845x Init parameters\r\n");
    // CHECK - Check Sensor Driver Init Success/Failure
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_NONE, status);

    // CHECK - Check Sensor handle
    TEST_ASSERT_EQUAL_INT(true, MMA845xdrv.isInitialized);

    PRINTF("\r\n \r\n");
}

/*! @brief       MMA845x Sensor Driver Config Test Function.
 *  @details     Test function to test MMA845x sensor driver Config interface.
 *               Checks:
 *               return status of the sensor driver config function,
 *               sensor handle state,
 *               ctrl_reg1 value configured for the sensor,
 *               ctrl_reg2 value configured for the sensor,
 *               ctrl_reg3 value configured for the sensor,
 *               ctrl_reg4 value configured for the sensor,
 *               ctrl_reg5 value configured for the sensor,
 *               f_setup value configured for the sensor.
 */
void test_mma845x_config(void)
{
    int32_t status;
    uint8_t ctrlReg1;
    uint8_t ctrlReg2;
    uint8_t ctrlReg3;
    uint8_t ctrlReg4;
    uint8_t ctrlReg5;
    uint8_t fSetup;

    PRINTF("\r\n \r\n");
    PRINTF("\r\n---------------------------------------------------------\r\n");
    test_counter++;
    PRINTF("Test %d: MMA845x Sensor Driver Config Test\r\n", test_counter);

    /*! Configure the MMA845x sensor driver with FIFO mode. */
    PRINTF("\r\nConfigure MMA845x Sensor\r\n");
    status = MMA845x_I2C_Configure(&MMA845xdrv, mma845x_Config_FifoIsr);

    PRINTF("\r\nCheck MMA845x configured parameters\r\n");
    /*! CHECK - Check Sensor Driver Config Success/Failure. */
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_NONE, status);

    // CHECK - Check Sensor handle
    TEST_ASSERT_EQUAL_INT(true, MMA845xdrv.isInitialized);

    /*! Read the CTRL_REG1 value and check with reference test value. */
    status = MMA845x_I2C_ReadData(&MMA845xdrv, mma845x_CTRL_REG1, &ctrlReg1);

    /*! CHECK - Check CTRL_REG1 value. */
    TEST_ASSERT_EQUAL_HEX(CTRLREG1_TESTVAL, ctrlReg1);

    /*! Read the CTRL_REG2 value and check with reference test value. */
    status = MMA845x_I2C_ReadData(&MMA845xdrv, mma845x_CTRL_REG2, &ctrlReg2);
    /*! CHECK - Check CTRL_REG2 value. */
    TEST_ASSERT_EQUAL_HEX(CTRLREG2_TESTVAL, ctrlReg2);

    /*! Read the CTRL_REG3 value and check with reference test value. */
    status = MMA845x_I2C_ReadData(&MMA845xdrv, mma845x_CTRL_REG3, &ctrlReg3);
    /*! CHECK - Check CTRL_REG3 value. */
    TEST_ASSERT_EQUAL_HEX(CTRLREG3_TESTVAL, ctrlReg3);

    /*! Read the CTRL_REG4 value and check with reference test value. */
    status = MMA845x_I2C_ReadData(&MMA845xdrv, mma845x_CTRL_REG4, &ctrlReg4);
    /*! CHECK - Check CTRL_REG4 value. */
    TEST_ASSERT_EQUAL_HEX(CTRLREG4_TESTVAL, ctrlReg4);

    /*! Read the CTRL_REG5 value and check with reference test value. */
    status = MMA845x_I2C_ReadData(&MMA845xdrv, mma845x_CTRL_REG5, &ctrlReg5);
    /*! CHECK - Check CTRL_REG5 value. */
    TEST_ASSERT_EQUAL_HEX(CTRLREG5_TESTVAL, ctrlReg5);

    /*! Read the F_SETUP value and check with reference test value. */
    status = MMA845x_I2C_ReadData(&MMA845xdrv, mma845x_F_SETUP, &fSetup);
    /*! CHECK - Check F_SETUP value. */
    TEST_ASSERT_EQUAL_HEX(FSETUP_TESTVAL, fSetup);

    PRINTF("\r\n \r\n");
}

/*! @brief       MMA845x Sensor Driver deInit Test Function.
 *  @details     Test function to test MMA845x sensor driver deInit interface.
 *               Checks:
 *               return status of the sensor driver deInit function,
 *               sensor handle state after successful deInit,
 */
void test_mma845x_deInit(void)
{
    int32_t status;

    PRINTF(" \r\n \r\n");
    PRINTF("\r\n---------------------------------------------------------\r\n");
    test_counter++;
    PRINTF("Test %d: MMA845x Sensor Driver DeInit Test\r\n", test_counter);

    // DeInit the MMA845x sensor driver sampling.
    PRINTF("\r\nDeInit MMA845x Sensor\r\n");
    status = MMA845x_I2C_Deinit(&MMA845xdrv);

    PRINTF("\r\nCheck sensor handle state after deInit\r\n");
    // CHECK - Check Sensor Driver Init Success/Failure
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_NONE, status);

    // CHECK - Check Sensor handle
    TEST_ASSERT_EQUAL_INT(false, MMA845xdrv.isInitialized);

    PRINTF("\r\n \r\n");
}

/*! @brief       MMA845x Sensor Driver Negative Test 1 Function.
 *  @details     Test function to negatively test MMA845x sensor driver
 *               functional interfaces by applying corrupted I2C driver metadata.
 *               Checks:
 *               return status of the sensor driver functions.
 *               sensor handle state,
 */
void test_mma845x_negative_1(void)
{
    int32_t status;
    uint8_t whoAmI;
    ARM_DRIVER_I2C *corruptedI2Cdriver = NULL;

    PRINTF("\r\n \r\n");
    PRINTF("\r\n---------------------------------------------------------\r\n");
    test_counter++;
    PRINTF("Test %d: MMA845x Sensor Driver Negative Test 1\r\n", test_counter);

    /*! Initialize the MMA845x sensor driver. */
    PRINTF("\r\nInitialize MMA845x Sensor with corrupted I2C driver metadata\r\n");
    status = MMA845x_I2C_Initialize(&MMA845xdrv, corruptedI2Cdriver, I2C_S_DEVICE_INDEX, MMA845x_I2C_ADDRESS_SA0_0,
                                    MMA8452_WHO_AM_I_WHOAMI_VALUE);

    PRINTF("\r\nStart performing init parameters check\r\n");
    /*! CHECK - Check Sensor Driver Init Success/Failure. */
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_INVALID_PARAM, status);

    // CHECK - Check Sensor handle
    TEST_ASSERT_EQUAL_INT(false, MMA845xdrv.isInitialized);

    // Reading sensor data with bad read register list
    PRINTF("\r\nRead MMA845x data.\r\n");
    status = MMA845x_I2C_ReadData(&MMA845xdrv, mma845x_WHO_AM_I, &whoAmI);

    PRINTF("\r\nCheck error returned by Sensor Read\r\n");
    // CHECK - Check Sensor Read Status
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_INIT, status);

    /*! Configure the MMA845x sensor driver with FIFO mode. */
    PRINTF("\r\nconfigure MMA845x.\r\n");
    status = MMA845x_I2C_Configure(&MMA845xdrv, mma845x_Config_FifoIsr);

    PRINTF("\r\nCheck error returned by Sensor Configure\r\n");
    /*! CHECK - Check Sensor Driver Configure Success/Failure */
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_INIT, status);

    /*! De-Initialize the MMA845x sensor driver. */
    PRINTF("\r\nDeInit MMA845x.\r\n");
    status = MMA845x_I2C_Deinit(&MMA845xdrv);

    PRINTF("\r\nCheck error returned by Sensor DeInit\r\n");
    // CHECK - Check Sensor De-Init Status
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_INIT, status);

    PRINTF("\r\n \r\n");
}

/*! @brief       MMA845x Sensor Driver Negative Test 2 Function.
 *  @details     Test function to negatively test MMA845x sensor driver
 *               functional interfaces by applying bad I2C address.
 *               Checks:
 *               return status of the sensor driver functions.
 *               sensor handle state,
 */
void test_mma845x_negative_2(void)
{
    int32_t status;
    uint8_t whoAmI;
    uint16_t badSlaveaddr = 0x21;

    PRINTF(" \r\n \r\n");
    PRINTF("\r\n---------------------------------------------------------\r\n");
    test_counter++;
    PRINTF("Test %d: MMA845x Sensor Driver Init Negative Test 2\r\n", test_counter);

    // Initialize the MMA845x sensor driver.
    PRINTF("\r\nInitialize MMA845x Sensor with bad I2C slave address\r\n");
    status = MMA845x_I2C_Initialize(&MMA845xdrv, &I2C_S_DRIVER, I2C_S_DEVICE_INDEX, badSlaveaddr,
                                    MMA8452_WHO_AM_I_WHOAMI_VALUE);

    PRINTF("\r\nCheck error returned by Sensor Init and sensor handle state\r\n");
    // CHECK - Check Sensor Driver Init Success/Failure
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_INIT, status);

    // Read the WHO_AM_I value to make sure we are talking to the MMA845x.
    PRINTF("\r\nRead MMA845x WHOAMI.\r\n");
    status = MMA845x_I2C_ReadData(&MMA845xdrv, mma845x_WHO_AM_I, &whoAmI);

    PRINTF("\r\nCheck WHOAMI returned by Sensor Read\r\n");
    // CHECK - Check Sensor Driver whoami
    TEST_ASSERT_NOT_EQUAL(MMA8452_WHO_AM_I_WHOAMI_VALUE, whoAmI);

    // Configure the MMA845x sensor driver with FIFO mode.
    PRINTF("\r\nConfigure MMA845x.\r\n");
    status = MMA845x_I2C_Configure(&MMA845xdrv, mma845x_Config_FifoIsr);

    PRINTF("\r\nCheck error returned by Sensor Configure\r\n");
    // CHECK - Check Sensor Driver Init Success/Failure
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_INIT, status);

    PRINTF("\r\n \r\n");
}

/*! @brief       MMA845x Sensor Driver Negative Test 3 Function.
 *  @details     Test function to negatively test MMA845x sensor driver
 *               functional interfaces by applying bad Write command List.
 *               Checks:
 *               return status of the sensor driver functions.
 *               sensor handle state,
 */
void test_mma845x_negative_3(void)
{
    int32_t status;
    const registerwritelist_t *badRegWriteList = NULL;

    PRINTF(" \r\n \r\n");
    PRINTF("\r\n---------------------------------------------------------\r\n");
    test_counter++;
    PRINTF("Test %d: MMA845x Sensor Driver Negative Test 3\r\n", test_counter);

    // Initialize the MMA845x sensor driver.
    PRINTF("\r\nInitialize MMA845x Sensor\r\n");
    status = MMA845x_I2C_Initialize(&MMA845xdrv, &I2C_S_DRIVER, I2C_S_DEVICE_INDEX, MMA845x_I2C_ADDRESS_SA0_0,
                                    MMA8452_WHO_AM_I_WHOAMI_VALUE);

    // Configure the MMA845x sensor driver with FIFO mode.
    PRINTF("\r\nConfigure MMA845x Sensor with bad register write list\r\n");
    status = MMA845x_I2C_Configure(&MMA845xdrv, badRegWriteList);

    PRINTF("\r\nCheck error returned by Sensor Configure\r\n");
    // CHECK - Check Sensor Driver Config Success/Failure
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_INVALID_PARAM, status);
}

/*! @brief       MMA845x Sensor Driver Negative Test 4 Function.
 *  @details     Test function to negatively test MMA845x sensor driver
 *               functional interfaces by applying bad read command List.
 *               Checks:
 *               return status of the sensor driver functions.
 *               sensor handle state,
 */
void test_mma845x_negative_4(void)
{
    int32_t status;
    uint8_t whoAmI;
    const registerreadlist_t *badRegReadList = NULL;

    PRINTF(" \r\n \r\n");
    PRINTF("\r\n---------------------------------------------------------\r\n");
    test_counter++;
    PRINTF("Test %d: MMA845x Sensor Driver Negative Test 4\r\n", test_counter);

    // Initialize the MMA845x sensor driver.
    PRINTF("\r\nInitialize MMA845x Sensor\r\n");
    status = MMA845x_I2C_Initialize(&MMA845xdrv, &I2C_S_DRIVER, I2C_S_DEVICE_INDEX, MMA845x_I2C_ADDRESS_SA0_0,
                                    MMA8452_WHO_AM_I_WHOAMI_VALUE);

    // Reading sensor data with bad read register list
    PRINTF("\r\nread MMA845x Sensor data bad register read list\r\n");
    status = MMA845x_I2C_ReadData(&MMA845xdrv, badRegReadList, &whoAmI);

    // CHECK - Check Sensor Read Status
    PRINTF("\r\nCheck error returned by Sensor Read\r\n");
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_INVALID_PARAM, status);
}

void tearDown(void)
{
    // TearDown Required for test framework.
}
