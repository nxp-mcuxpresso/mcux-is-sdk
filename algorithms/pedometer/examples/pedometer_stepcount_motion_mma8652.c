/*
 * Copyright (c) 2016, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/**
 * @file pedometer_stepcount_mma8652.c
 * @brief The pedometer_stepcount_mma8652.c file implements the ISSDK MMA8652 sensor
 *        driver example demonstration for Motion Activated Pedometer.
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
#include "pedometer.h"
#include "gpio_driver.h"
#include "mma865x_drv.h"

//-----------------------------------------------------------------------
// Macros
//-----------------------------------------------------------------------
#define MT_FF_MT_THS 0x15  /* Threshold Value. */
#define ASLP_COUNTER 0x10   /* Auto Sleep after ~5s. */
#define MMA865x_DATA_SIZE 6 /* 2-byte Accel x,y,z each */

//-----------------------------------------------------------------------
// Constants
//-----------------------------------------------------------------------
/*! @brief MMA865x Motion based Pedometer Register Write List. */
const registerwritelist_t cMma865xCommonConfiguration[] = {
    {MMA865x_ASLP_COUNT, ASLP_COUNTER, 0},
    {MMA865x_FF_MT_THS, MT_FF_MT_THS, MMA865x_FF_MT_THS_THS_MASK},
    {MMA865x_FF_MT_CFG, MMA865x_FF_MT_CFG_OAE_MOTION | MMA865x_FF_MT_CFG_ZEFE_EN | MMA865x_FF_MT_CFG_YEFE_EN | MMA865x_FF_MT_CFG_XEFE_EN,
                        MMA865x_FF_MT_CFG_OAE_MASK | MMA865x_FF_MT_CFG_ZEFE_MASK | MMA865x_FF_MT_CFG_YEFE_MASK | MMA865x_FF_MT_CFG_XEFE_MASK},
    {MMA865x_CTRL_REG1, MMA865x_CTRL_REG1_ASLP_RATE_1_56HZ | MMA865x_CTRL_REG1_DR_50HZ,
                        MMA865x_CTRL_REG1_ASLP_RATE_MASK | MMA865x_CTRL_REG1_DR_MASK},
    {MMA865x_CTRL_REG2, MMA865x_CTRL_REG2_SLPE_EN | MMA865x_CTRL_REG2_SMODS_LP,
                        MMA865x_CTRL_REG2_SLPE_MASK | MMA865x_CTRL_REG2_SMODS_MASK},
    {MMA865x_CTRL_REG3, MMA865x_CTRL_REG3_WAKE_FF_MT_EN | MMA865x_CTRL_REG3_IPOL_ACTIVE_HIGH | MMA865x_CTRL_REG3_PP_OD_PUSH_PULL,
                        MMA865x_CTRL_REG3_WAKE_FF_MT_MASK | MMA865x_CTRL_REG3_IPOL_MASK | MMA865x_CTRL_REG3_PP_OD_MASK},
    {MMA865x_CTRL_REG4, MMA865x_CTRL_REG4_INT_EN_FF_MT_EN, MMA865x_CTRL_REG4_INT_EN_FF_MT_MASK},
    {MMA865x_CTRL_REG5, MMA865x_CTRL_REG5_INT_CFG_DRDY_INT1 | MMA865x_CTRL_REG5_INT_CFG_ASLP_INT1,
                        MMA865x_CTRL_REG5_INT_CFG_DRDY_MASK | MMA865x_CTRL_REG5_INT_CFG_ASLP_MASK},
    __END_WRITE_DATA__};

/*! @brief MMA865x Motion Detect Mode Register Write List. */
const registerwritelist_t cMma865xConfigMotionDetect[] = {
    /* Only FFMT INT Enabled */
    {MMA865x_CTRL_REG4, MMA865x_CTRL_REG4_INT_EN_DRDY_DISABLED | MMA865x_CTRL_REG4_INT_EN_ASLP_DISABLED,
                        MMA865x_CTRL_REG4_INT_EN_DRDY_MASK | MMA865x_CTRL_REG4_INT_EN_ASLP_MASK},
    /* Route FFMT to INT1 */
    {MMA865x_CTRL_REG5, MMA865x_CTRL_REG5_INT_CFG_FF_MT_INT1, MMA865x_CTRL_REG5_INT_CFG_FF_MT_MASK},
    __END_WRITE_DATA__};

/*! @brief MMA865x DRDY and ASLP Detect Mode Register Write List. */
const registerwritelist_t cMma865xConfigDataReady[] = {
    /* Data and ASLP INT Enabled */
    {MMA865x_CTRL_REG4, MMA865x_CTRL_REG4_INT_EN_DRDY_EN | MMA865x_CTRL_REG4_INT_EN_ASLP_EN,
                        MMA865x_CTRL_REG4_INT_EN_DRDY_MASK | MMA865x_CTRL_REG4_INT_EN_ASLP_MASK},
    /* Route FFMT to INT2 (Ignore FFMT INT) */						
    {MMA865x_CTRL_REG5, MMA865x_CTRL_REG5_INT_CFG_FF_MT_INT2, MMA865x_CTRL_REG5_INT_CFG_FF_MT_MASK},
    __END_WRITE_DATA__};

/*! @brief Address of INT Source Register. */
const registerreadlist_t cMma865xINTSrc[] = {{.readFrom = MMA865x_INT_SOURCE, .numBytes = 1}, __END_READ_DATA__};

/*! @brief Address of Data Output Registers. */
const registerreadlist_t cMma865xOutput[] = {{.readFrom = MMA865x_OUT_X_MSB, .numBytes = MMA865x_DATA_SIZE}, __END_READ_DATA__};

/*! @brief Pedometer Mode Name Strings. */
const char *pActivity[5] = {"Unknown ", "Rest    ", "Walking ", "Jogging ", "Running "};

//-----------------------------------------------------------------------
// Global Variables
//-----------------------------------------------------------------------
volatile bool gMma865xEventReady = false;

/* Pedometer configuration. These configuration are algorithm and user dependent data. */
pedometer_config_t cPedoConfig =
{
    .sleepcount_threshold = 1,
    .bits = {.config = 1},
    .keynetik =
    {
        .steplength = 0,
        .height = 175,
        .weight = 80,
        .filtersteps = 1,
        .bits =
        {
            .filtertime = 2, .male = 1,
        },
        .speedperiod = 3,
        .stepthreshold = 0,
    },
    .stepcoalesce = 1,
    .oneG = PEDO_ONEG_2G,
    .frequency = 50,
};

//-----------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------
/*! -----------------------------------------------------------------------
 *  @brief       This is the Sensor Event Ready ISR implementation.
 *  @details     This function sets the flag which indicates if a new sample(s) is available for reading.
 *  @param[in]   pUserData This is a void pointer to the instance of the user specific data structure for the ISR.
 *  @return      void  There is no return value.
 *  @constraints None
 *  @reeentrant  Yes
 *  -----------------------------------------------------------------------*/
void mma865x_int_callback(void *pUserData)
{ /*! @brief Set flag to indicate Sensor has signaled event ready. */
    gMma865xEventReady = true;
}

/*! -----------------------------------------------------------------------
 *  @brief       This is the The main function implementation.
 *  @details     This function invokes board initializes routines, then then brings up the sensor and
 *               finally enters an endless loop to continuously read available samples.
 *  @param[in]   void This is no input parameter.
 *  @return      void  There is no return value.
 *  @constraints None
 *  @reeentrant  No
 *  -----------------------------------------------------------------------*/
int main(void)
{
    int32_t status;
    bool motionDetect;
    uint16_t lastReportedSteps;
    uint8_t intStatus, data[MMA865x_DATA_SIZE];

    pedometer_t pedometer; /* This handle holds the current configuration and status value for the pedometer.*/
    mma865x_acceldata_t rawData;
    mma865x_i2c_sensorhandle_t mma865xDriver;

    ARM_DRIVER_I2C *I2Cdrv = &I2C_S_DRIVER; // Now using the shield.h value!!!
    GENERIC_DRIVER_GPIO *pGpioDriver = &Driver_GPIO_KSDK;

    /*! Initialize the MCU hardware. */
    BOARD_InitPins();
    BOARD_BootClockRUN();
    BOARD_InitDebugConsole();

    PRINTF("\r\n ISSDK MMA865x sensor driver example for Motion Activated Pedometer.\r\n");

    /*! Initialize MMA865x pin used by FRDM board */
    pGpioDriver->pin_init(&MMA8652_INT1, GPIO_DIRECTION_IN, NULL, &mma865x_int_callback, NULL);

    /*! Initialize RGB LED pin used by FRDM board */
    pGpioDriver->pin_init(&GREEN_LED, GPIO_DIRECTION_OUT, NULL, NULL, NULL);

    /*! Initialize the I2C driver. */
    status = I2Cdrv->Initialize(I2C_S_SIGNAL_EVENT);
    if (ARM_DRIVER_OK != status)
    {
        PRINTF("\r\n I2C Initialization Failed\r\n");
        return -1;
    }

    /*! Set the I2C Power mode. */
    status = I2Cdrv->PowerControl(ARM_POWER_FULL);
    if (ARM_DRIVER_OK != status)
    {
        PRINTF("\r\n I2C Power Mode setting Failed\r\n");
        return -1;
    }

    /*! Set the I2C bus speed. */
    status = I2Cdrv->Control(ARM_I2C_BUS_SPEED, ARM_I2C_BUS_SPEED_FAST);
    if (ARM_DRIVER_OK != status)
    {
        PRINTF("\r\n I2C Control Mode setting Failed\r\n");
        return -1;
    }

    /*! Initialize MMA865x sensor driver. */
    status = MMA865x_I2C_Initialize(&mma865xDriver, &I2C_S_DRIVER, I2C_S_DEVICE_INDEX, MMA8652_I2C_ADDR,
                                     MMA8652_WHOAMI_VALUE);
    if (SENSOR_ERROR_NONE != status)
    {
        PRINTF("\r\n Sensor Initialization Failed\r\n");
        return -1;
    }

    /*!  Set the task to be executed while waiting for I2C transactions to complete. */
    MMA865x_I2C_SetIdleTask(&mma865xDriver, (registeridlefunction_t)SMC_SetPowerModeVlpr, SMC);

    /* Apply MMA865x Configuration for Motion Activated Pedometer. */
    status = Sensor_I2C_Write(mma865xDriver.pCommDrv, &mma865xDriver.deviceInfo,
                              mma865xDriver.slaveAddress, cMma865xCommonConfiguration);
    if (SENSOR_ERROR_NONE != status)
    {
        PRINTF("\r\n Write MMA865x Initialization Failed.\r\n");
        return -1;
    }

    /*! Initialize the pedometer*/
    pedometer_init(&pedometer);
    PRINTF("\r\n Pedometer successfully Initialized and Ready for measurements.");

    for (;;) /* Forever loop for Motion Detection */
    {
        /*! Configure the pedometer*/
        pedometer_configure(&pedometer, &cPedoConfig);

        /* Apply MMA865x Configuration for Motion Detection. */
        status = MMA865x_I2C_Configure(&mma865xDriver, cMma865xConfigMotionDetect);
        if (SENSOR_ERROR_NONE != status)
        {
            PRINTF("\r\n Write MMA865x Motion Configuration Failed!\r\n");
            return -1;
        }

        motionDetect = true;
        pGpioDriver->set_pin(&GREEN_LED);
        PRINTF("\r\n\r\n Waiting for Motion | MCU switching to Sleep Mode ...\r\n");

        for (;;) /* Loop for Motion Detection */
        {   /* In ISR Mode we do not need to check Data Ready Register.
             * The receipt of interrupt will indicate event is ready. */
            if (false == gMma865xEventReady)
            { /* Loop, if new sample is not available. */
                SMC_SetPowerModeWait(SMC);
                continue;
            }
            else
            { /*! Clear the data ready flag, it will be set again by the ISR. */
                gMma865xEventReady = false;
            }

            /*! Read the INT_SRC from the MMA865x. */
            status = MMA865x_I2C_ReadData(&mma865xDriver, cMma865xINTSrc, &intStatus);
            if (ARM_DRIVER_OK != status)
            {
                PRINTF("\r\n Read INT SRC Failed!\r\n");
                return -1;
            }

            if (motionDetect)
            {
                if ((intStatus & MMA865x_INT_SOURCE_SRC_FF_MT_MASK) == MMA865x_INT_SOURCE_SRC_FF_MT_READY)
                {
                    /*! Display that a Motion event has been detected. */
                    PRINTF(" Motion detected...\r\n");

                    /* Apply MMA865x Configuration for ASLP Detection with DATA. */
                    status = MMA865x_I2C_Configure(&mma865xDriver, cMma865xConfigDataReady);
                    if (SENSOR_ERROR_NONE != status)
                    {
                        PRINTF("\r\n Write MMA865x DRDY Configuration Failed.\r\n");
                        return -1;
                    }
                    motionDetect = false;
                    pGpioDriver->clr_pin(&GREEN_LED);
                }
            }
            else
            {
                /*! Read the Output from the MMA865x. */
                status = MMA865x_I2C_ReadData(&mma865xDriver, cMma865xOutput, data);
                if (ARM_DRIVER_OK != status)
                {
                    PRINTF("\r\n ERROR : Read Data Failed!\r\n");
                    return -1;
                }

                /*! Convert the raw sensor data for feeding to pedometer algorithm. */
                rawData.accel[0] = ((int16_t)data[0] << 8) | data[1];
                rawData.accel[1] = ((int16_t)data[2] << 8) | data[3];
                rawData.accel[2] = ((int16_t)data[4] << 8) | data[5];

                /*!  Execute the pedometer Algorithm */
                pedometer_run(&pedometer, (ped_accel_t *)&rawData.accel);

                /* Display Steps when there is a change */
                if (pedometer.status.stepcount && pedometer.status.stepcount != lastReportedSteps)
                {
                    pGpioDriver->toggle_pin(&GREEN_LED);
                    lastReportedSteps = pedometer.status.stepcount;
                    if(pedometer.status.stepcount == 1)
                    {
                        PRINTF("\r\n | Steps | Distance |  Speed  | Calories | Activity |\r\n");
                    }
                    PRINTF("\033[K | %5d |    %4dm |  %2dkmph |      %3d | %s |\r",
                           pedometer.status.stepcount, pedometer.status.distance, pedometer.status.speed/1000, pedometer.status.calories, pActivity[pedometer.status.status.bits.activity]);
                }

                if ((intStatus & MMA865x_INT_SOURCE_SRC_ASLP_MASK) == MMA865x_INT_SOURCE_SRC_ASLP_READY)
                {
                    break;
                }
            }
        }
    }
}
