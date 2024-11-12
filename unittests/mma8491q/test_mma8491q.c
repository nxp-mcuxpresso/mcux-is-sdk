/*
 * Copyright (c) 2015 - 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/*! @file  test_mma8491q.c
 *  @brief This is the unit test file for MMA8491Q sensor driver.
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
#include "gpio_driver.h"
#include "mma8491q_drv.h"
#include "systick_utils.h"

//-----------------------------------------------------------------------
// ISSDK Includes
//-----------------------------------------------------------------------
#include "unity.h"

//-----------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------
static int gTestCounter = 0;
mma8491q_i2c_sensorhandle_t gMma8491qDriver;
ARM_DRIVER_I2C *I2Cdrv = &I2C_S_DRIVER; // Now using the shield.h value!!!
GENERIC_DRIVER_GPIO *pGpioDriver = &Driver_GPIO_KSDK;

//-----------------------------------------------------------------------
// Constants
//-----------------------------------------------------------------------
/*! @brief Address of Status Register. */
const registerreadlist_t cMma8491qStatus[] = {{.readFrom = MMA8491Q_STATUS, .numBytes = 1}, __END_READ_DATA__};

/*! @brief Address and size of Raw Acceleration Data. */
const registerreadlist_t cMma8491qOutput[] = {{.readFrom = MMA8491Q_OUT_X_MSB, .numBytes = MMA8491Q_DATA_SIZE},
                                              __END_READ_DATA__};

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

    /*! Initialize the EN Pin. */
    pGpioDriver->pin_init(&MMA8491_EN, GPIO_DIRECTION_OUT, NULL, NULL, NULL);

    /* Set EN = 1 to put the Sensor into Active Mode and enable I2C communication. */
    pGpioDriver->set_pin(&MMA8491_EN);

    BOARD_DELAY_ms(1);
}

/*! -----------------------------------------------------------------------
 *  @brief       This is the MMA8491Q Sensor Driver Negative 1 Test Function.
 *  @details     Checks:
 *                  Return status of the sensor driver functions, when applied
 *                  Corrupted I2C driver metadata
 *  @param[in]   void This is no input parameter.
 *  @return      void  There is no return value.
 *  @constraints This should be the first testfunction to be executed after setUp().
 *  @reeentrant  No
 *  -----------------------------------------------------------------------*/
void test_mma8491q_negative_1(void)
{
    uint8_t statusReg;
    int32_t status;
    ARM_DRIVER_I2C *pCorruptedI2Cdriver = NULL;

    PRINTF("\r\n \r\n");
    PRINTF("\r\n---------------------------------------------------------\r\n");
    gTestCounter++;
    PRINTF("Test %d: MMA8491Q Sensor Driver Negative 1 Test.\r\n", gTestCounter);

    // Initialize the MMA8491Q sensor driver.
    status = MMA8491Q_I2C_Initialize(&gMma8491qDriver, pCorruptedI2Cdriver, I2C_S_DEVICE_INDEX, MMA8491_I2C_ADDR);

    PRINTF("\r\nStart performing init parameters check\r\n");
    // CHECK - Check Sensor Driver Init Success/Failure
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_INVALID_PARAM, status);

    // CHECK - Check Sensor handle
    TEST_ASSERT_EQUAL_INT(false, gMma8491qDriver.isInitialized);

    // Reading sensor data with bad read register list
    status = MMA8491Q_I2C_ReadData(&gMma8491qDriver, cMma8491qStatus, &statusReg);

    // CHECK - Check Sensor Read Status
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_INIT, status);

    PRINTF("\r\n \r\n");
}

/*! -----------------------------------------------------------------------
 *  @brief       This is the MMA8491Q Sensor Driver Negative 2 Test Function.
 *  @details     Checks:
 *                  Return status of the sensor driver functions, when applied
 *                  Bad I2C address
 *  @param[in]   void This is no input parameter.
 *  @return      void  There is no return value.
 *  @constraints This should be the second testfunction to be executed after test_mma8491q_negative_1().
 *  @reeentrant  No
 *  -----------------------------------------------------------------------*/
void test_mma8491q_negative_2(void)
{
    uint8_t statusReg;
    int32_t status;
    uint16_t badSlaveaddr = 0x21;

    PRINTF("\r\n \r\n");
    PRINTF("\r\n---------------------------------------------------------\r\n");
    gTestCounter++;
    PRINTF("Test %d: MMA8491Q Sensor Driver Init Negative 2 Test.\r\n", gTestCounter);

    // Initialize the MMA8491Q sensor driver.
    status = MMA8491Q_I2C_Initialize(&gMma8491qDriver, &I2C_S_DRIVER, I2C_S_DEVICE_INDEX, badSlaveaddr);

    // CHECK - Check Sensor Driver Init Success/Failure
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_INIT, status);

    // CHECK - Check Sensor handle
    TEST_ASSERT_EQUAL_INT(false, gMma8491qDriver.isInitialized);

    /* Read device to verify Register_Read failure.*/
    status = MMA8491Q_I2C_ReadData(&gMma8491qDriver, cMma8491qStatus, &statusReg);

    // CHECK - Check Sensor read status.
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_INIT, status);

    PRINTF("\r\n \r\n");
}

/*! -----------------------------------------------------------------------
 *  @brief       This is the MMA8491Q Sensor Driver Init Test Function.
 *  @details     Checks:
 *                  Return status of the sensor driver init function.
 *                  Who_Am_I of the sensor.
 *  @param[in]   void This is no input parameter.
 *  @return      void  There is no return value.
 *  @constraints This should be the third testfunction to be executed after test_mma8491q_negative_2().
 *  @reeentrant  No
 *  -----------------------------------------------------------------------*/
void test_mma8491q_init(void)
{
    int32_t status;

    PRINTF("\r\n---------------------------------------------------------\r\n");
    gTestCounter++;
    PRINTF("\r\nTest %d: MMA8491Q Sensor Driver Init Test\r\n", gTestCounter);

    // Initialize MMA8491Q sensor driver.
    status = MMA8491Q_I2C_Initialize(&gMma8491qDriver, &I2C_S_DRIVER, I2C_S_DEVICE_INDEX, MMA8491_I2C_ADDR);

    PRINTF("\r\nStart performing init parameters check\r\n");
    // CHECK - Check Sensor Driver Init Success/Failure
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_NONE, status);

    // CHECK - Check Sensor State
    TEST_ASSERT_EQUAL_INT(true, gMma8491qDriver.isInitialized);

    PRINTF("\r\n \r\n");
}

/*! -----------------------------------------------------------------------
 *  @brief       This is the MMA8491Q Sensor Driver Read Test Function.
 *  @details     Checks:
 *                  Return status of the sensor data read function.
 *  @param[in]   void This is no input parameter.
 *  @return      void  There is no return value.
 *  @constraints This should be the fourth testfunction to be executed after test_mma8491q_init().
 *  @reeentrant  No
 *  -----------------------------------------------------------------------*/
void test_mma8491q_read(void)
{
    int32_t status;
    uint8_t data[MMA8491Q_DATA_SIZE];

    PRINTF("\r\n \r\n");
    PRINTF("\r\n---------------------------------------------------------\r\n");
    gTestCounter++;
    PRINTF("Test %d: MMA8491Q Sensor Driver Config Test\r\n", gTestCounter);

    // Read the Sensor Data values.
    status = MMA8491Q_I2C_ReadData(&gMma8491qDriver, cMma8491qOutput, data);
    // CHECK - Check Sensor Driver Init Success/Failure
    TEST_ASSERT_EQUAL_INT(SENSOR_ERROR_NONE, status);

    PRINTF("\r\n \r\n");
}

/* TearDown Required for test framework. */
void tearDown(void)
{
    pGpioDriver->set_pin(&MMA8491_EN);
}

////////////////////////////////////////////////////////////////////////////////
// EOF
////////////////////////////////////////////////////////////////////////////////
